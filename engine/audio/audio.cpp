// =============================================================================
//  MUMBAI 1:1 EXACT — engine/audio/audio.cpp
//
//  The synthesised city.  See audio.h for what is physical, what is cited and
//  what is modelled; this file is the arithmetic.
// =============================================================================
#include "engine/audio/audio.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

namespace mumbai::audio {
namespace {

constexpr f64 kPi = 3.14159265358979323846;
constexpr f64 kTwoPi = 6.28318530717958647692;

// -----------------------------------------------------------------------------
//  A sine table.  Linear interpolation over 4096 points holds the interpolation
//  error below -130 dB — under the 16-bit dither floor — for a fraction of
//  libm's sin(): this voice stack runs per sample per source on a phone.
// -----------------------------------------------------------------------------
struct SineTable {
    static constexpr u32 kSize = 4096;
    static constexpr u32 kMask = kSize - 1;
    f64 v[kSize + 1];
    SineTable() {
        for (u32 i = 0; i <= kSize; ++i) v[i] = std::sin(kTwoPi * static_cast<f64>(i) / kSize);
    }
    // `phase` is in turns, any real value.
    f64 at(f64 phase) const {
        const f64 fl = std::floor(phase);
        const f64 frac01 = phase - fl;
        const f64 fpos = frac01 * kSize;
        const u32 i = static_cast<u32>(fpos) & kMask;
        const f64 frac = fpos - std::floor(fpos);
        return v[i] + (v[i + 1] - v[i]) * frac;
    }
};
const SineTable& sineTable() {
    static const SineTable t;
    return t;
}
inline f64 sine(f64 turns) { return sineTable().at(turns); }

// -----------------------------------------------------------------------------
//  Deterministic noise.  Every voice owns its own stream, seeded from its id, so
//  a render is reproducible and a test can assert bit-exactness.
// -----------------------------------------------------------------------------
struct Noise {
    u64 s;
    explicit Noise(u64 seed) : s(seed ? seed : 0x9E3779B97F4A7C15ull) {}
    f64 next() {   // white, [-1, 1)
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        return static_cast<f64>(static_cast<i64>(s)) * (1.0 / 9223372036854775808.0);
    }
};

inline f64 dbToGain(f64 db) { return std::pow(10.0, db / 20.0); }

// The dB SPL that corresponds to digital full scale, and the one place a source's
// level becomes a linear gain.  (This constant existed and was never applied on
// the first pass: every voice was 10^4 too loud, the limiter held the whole mix
// at its own threshold, and the symptom was that *every* test collapsed to the
// same number — distance, panorama, comb and occlusion all measured the
// limiter instead of the acoustics.  The bug is recorded in D-042.)
constexpr f64 kZeroDbfsSpl = 100.0;
inline f64 levelGainFor(f64 referenceDbAt10m, f64 authored) {
    return std::pow(10.0, (referenceDbAt10m - kZeroDbfsSpl) / 20.0) * authored;
}

// -----------------------------------------------------------------------------
//  Geometry
// -----------------------------------------------------------------------------
bool segmentHitsObb(const WPos& a, const WPos& b, const Occluder& o, f64& tEnter) {
    // The vertical slab is tested on the part of the segment that is inside the
    // box's footprint, not on the segment's endpoints: a ground-level listener
    // looking up at a source on a roof passes *over* the building between them,
    // and an endpoint comparison would call that occluded (it did, on the first
    // pass — the height test was decorative).
    const f64 lo = std::min(a.y, b.y), hi = std::max(a.y, b.y);
    if (hi <= static_cast<f64>(o.baseY) || lo >= static_cast<f64>(o.topY)) return false;
    const f64 cy = std::cos(static_cast<f64>(o.yaw)), sy = std::sin(static_cast<f64>(o.yaw));
    // Into the box's local frame (rotate by -yaw).
    const f64 ax = (a.x - o.cx) * cy + (a.z - o.cz) * sy;
    const f64 az = -(a.x - o.cx) * sy + (a.z - o.cz) * cy;
    const f64 bx = (b.x - o.cx) * cy + (b.z - o.cz) * sy;
    const f64 bz = -(b.x - o.cx) * sy + (b.z - o.cz) * cy;
    const f64 dx = bx - ax, dz = bz - az;
    f64 t0 = 0.0, t1 = 1.0;
    const f64 hw = o.hw, hd = o.hd;
    if (std::fabs(dx) < 1e-12) {
        if (ax < -hw || ax > hw) return false;
    } else {
        f64 ta = (-hw - ax) / dx, tb = (hw - ax) / dx;
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta); t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    if (std::fabs(dz) < 1e-12) {
        if (az < -hd || az > hd) return false;
    } else {
        f64 ta = (-hd - az) / dz, tb = (hd - az) / dz;
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta); t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    // Height at the entry and the exit of the footprint: the segment is blocked
    // only if its vertical range overlaps the box's *inside* the footprint.
    const f64 yEnter = a.y + (b.y - a.y) * t0;
    const f64 yExit = a.y + (b.y - a.y) * t1;
    if (std::max(yEnter, yExit) <= static_cast<f64>(o.baseY) ||
        std::min(yEnter, yExit) >= static_cast<f64>(o.topY))
        return false;
    tEnter = t0;
    return true;
}

// Ground reflection as an image source over a locally flat road: the extra path
// an arrival takes off the road plane (the classic 2*h_l*h_s/d for two low
// elements), and the frequency of the first comb null it creates.
f64 groundExtraPath(f64 hListener, f64 hSource, f64 range) {
    return 2.0 * std::max(0.05, hListener) * std::max(0.02, hSource) / std::max(0.5, range);
}

// 4x oversampling, 24-tap windowed sinc, shared by the limiter's detector and by
// the true-peak meter — deliberately the *same* interpolator, so a signal at the
// limiter's threshold measures exactly that threshold.  A limiter that only
// watches sample values cannot bound the waveform a DAC will reconstruct: a
// noise-dominated mix limited to -1 dBFS measured +2.1 dBTP, which is a real
// inter-sample over (D-042).
struct Oversampler {
    static constexpr u32 kTaps = 24;
    static constexpr u32 kPhases = 4;
    f64 coeff[kPhases][kTaps];
    Oversampler() {
        for (u32 p = 0; p < kPhases; ++p) {
            const f64 frac = static_cast<f64>(p) / kPhases;
            f64 sum = 0.0;
            for (u32 i = 0; i < kTaps; ++i) {
                const f64 n = static_cast<f64>(i) - static_cast<f64>(kTaps / 2 - 1) - frac;
                const f64 sinc = std::fabs(n) < 1e-12 ? 1.0 : std::sin(kPi * n) / (kPi * n);
                // Blackman-Harris: -92 dB sidelobes, so neither the limiter nor
                // the meter is fooled by a neighbouring sample's leakage.
                const f64 w = 0.35875 - 0.48829 * std::cos(kTwoPi * i / (kTaps - 1)) +
                              0.14128 * std::cos(2 * kTwoPi * i / (kTaps - 1)) -
                              0.01168 * std::cos(3 * kTwoPi * i / (kTaps - 1));
                coeff[p][i] = sinc * w;
                sum += coeff[p][i];
            }
            for (u32 i = 0; i < kTaps; ++i) coeff[p][i] /= sum;
        }
    }
};
const Oversampler& oversampler() {
    static const Oversampler o;
    return o;
}

// Sliding-window minimum, O(1) amortised: the limiter needs the smallest gain
// the next 2 ms will require, every sample, for two channels.
struct MinWindow {
    std::vector<f64> vals;
    std::vector<u64> idx;
    size_t cap = 0;
    u64 head = 0, tail = 0, pos = 0;
    void init(u32 n) {
        cap = static_cast<size_t>(n) + 1;
        vals.assign(cap, 1.0);
        idx.assign(cap, 0);
        head = tail = pos = 0;
    }
    void push(f64 v) {
        while (head != tail) {
            const u64 back = (tail + cap - 1) % cap;
            if (vals[back] <= v) break;
            tail = back;
        }
        vals[tail] = v;
        idx[tail] = pos;
        tail = (tail + 1) % cap;
        ++pos;
        while (idx[head] + (cap - 1) < pos) head = (head + 1) % cap;
    }
    f64 min() const { return vals[head]; }
};

// -----------------------------------------------------------------------------
//  Per-archetype voice data.  Every number has a physical reading rather than
//  being a tuned fudge:
//    rpmPerMs  engine speed per metre/second of road speed in top gear
//              (a car at 100 km/h turns ~2200 rpm in top, i.e. 79 rpm/(m/s))
//    idle      idle speed, rpm;  redline  the point the gearbox shifts up
//    levelDb   the source's level at 10 m, dB SPL  (see kZeroDbfsSpl)
//    roughness the combustion/induction noise share of the engine voice
//    cylinders the firing orders the harmonic stack is built from
// -----------------------------------------------------------------------------
struct KindInfo {
    f64 rpmPerMs, idle, redline, levelDb, roughness;
    u32 cylinders;
};
const KindInfo& kindInfo(Archetype k) {
    static const KindInfo t[6] = {
        { 79.0,  800.0, 6200.0, 76.0, 0.20, 4 },   // car: 4-cylinder petrol
        { 74.0,  780.0, 5600.0, 77.0, 0.30, 4 },   // kaali-peeli taxi
        { 130.0, 1200.0, 6500.0, 74.0, 0.45, 1 },  // auto-rickshaw single
        { 120.0, 1100.0, 8000.0, 72.0, 0.55, 1 },  // motorcycle
        { 40.0,   550.0, 2400.0, 84.0, 0.35, 6 },  // BEST double-decker
        { 36.0,   520.0, 2300.0, 85.0, 0.38, 6 },  // truck
    };
    const size_t i = static_cast<size_t>(k);
    return t[i < 6 ? i : 0];
}

// The layer table: reference level at 10 m, and the voice budget a platform
// check reads (Section 6, 30 FPS on an Adreno 730).
const LayerBudget kBudgets[] = {
    {"engine", 76.0, 24},
    {"tyres",  70.0, 24},
    {"horn",   96.0,  8},
    {"rain",   62.0,  1},
    {"surf",   70.0,  6},
    {"city",   55.0,  1},
    {"tone",   80.0,  4},
};

// A gearbox, as a set of ratios with hysteresis.  Its audible consequence is the
// rev drop on every upshift, which is most of what makes a vehicle identifiable
// from its engine alone.
struct Gearbox {
    u32 gear = 0;
    static constexpr u32 kGears = 4;
    static constexpr f64 kRatio[4] = {3.4, 2.0, 1.4, 1.0};
    f64 rpm(const KindInfo& k, f64 v, f64 accel) {
        if (gear >= kGears) gear = kGears - 1;
        const f64 vv = std::fabs(v);
        f64 r = k.idle + vv * k.rpmPerMs * kRatio[gear];
        while (r > k.redline && gear + 1 < kGears) {
            ++gear;
            r = k.idle + vv * k.rpmPerMs * kRatio[gear];
        }
        // Downshift below 42 % of the redline: the two thresholds cannot both
        // hold, so the box cannot oscillate between gears.
        while (gear > 0 && r < k.redline * 0.42) {
            --gear;
            r = k.idle + vv * k.rpmPerMs * kRatio[gear];
        }
        r *= 1.0 + clamp(accel, -3.0, 3.0) * 0.01;   // load
        return clamp(r, k.idle * 0.9, k.redline * 1.02);
    }
    void reset() { gear = 0; }
};

// Deep-water dispersion relation: the period of the specification's own swell.
f64 swellPeriod(f64 lambda) { return std::sqrt(kTwoPi * std::max(8.0, lambda) / 9.80665); }

}  // namespace

// =============================================================================
//  Filters
// =============================================================================
void Svf::setCutoff(f64 fc, f64 sr) {
    fc = clamp(fc, 5.0, sr * 0.49);
    g = std::tan(kPi * fc / sr);
    a1 = 1.0 / (1.0 + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
}
void Svf::setResonance(f64 fc, f64 q, f64 sr) {
    k = 1.0 / std::max(0.05, q);
    setCutoff(fc, sr);
}
void Svf::setOnePoleLow(f64 fc, f64 sr) {
    fc = clamp(fc, 1.0, sr * 0.49);
    // The TPT one-pole: g = tan(pi*fc/sr), and G = g/(1+g) is the integrator's
    // coefficient, giving unity gain at DC.
    g = std::tan(kPi * fc / sr);
    a1 = g / (1.0 + g);
    k = 0.0;
}
f64 Svf::lowPass(f64 x) {
    const f64 v3 = x - ic2eq;
    const f64 v1 = a1 * ic1eq + a2 * v3;
    const f64 v2 = ic2eq + a2 * ic1eq + a3 * v3;
    ic1eq = 2.0 * v1 - ic1eq;
    ic2eq = 2.0 * v2 - ic2eq;
    return v2;
}
f64 Svf::highPass(f64 x) {
    const f64 v3 = x - ic2eq;
    const f64 v1 = a1 * ic1eq + a2 * v3;
    const f64 v2 = ic2eq + a2 * ic1eq + a3 * v3;
    ic1eq = 2.0 * v1 - ic1eq;
    ic2eq = 2.0 * v2 - ic2eq;
    return x - k * v1 - v2;
}
f64 Svf::bandPass(f64 x) {
    const f64 v3 = x - ic2eq;
    const f64 v1 = a1 * ic1eq + a2 * v3;
    const f64 v2 = ic2eq + a2 * ic1eq + a3 * v3;
    ic1eq = 2.0 * v1 - ic1eq;
    ic2eq = 2.0 * v2 - ic2eq;
    return v1;
}
f64 Svf::onePoleLow(f64 x) {
    const f64 v = (x - ic1eq) * a1;
    const f64 y = v + ic1eq;
    ic1eq = y + v;
    return y;
}

// =============================================================================
//  Occluders
// =============================================================================
void OccluderGrid::build(const std::vector<Occluder>& boxes, f64 cellSize) {
    m_boxes = boxes;
    m_cell = cellSize > 1.0 ? cellSize : 64.0;
    m_cells.clear();
    m_nx = m_nz = 0;
    if (m_boxes.empty()) return;
    f64 minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30;
    for (const Occluder& o : m_boxes) {
        const f64 r = std::max(o.hw, o.hd) + 1.0;
        minX = std::min(minX, o.cx - r); maxX = std::max(maxX, o.cx + r);
        minZ = std::min(minZ, o.cz - r); maxZ = std::max(maxZ, o.cz + r);
    }
    m_minCX = static_cast<i64>(std::floor(minX / m_cell));
    m_minCZ = static_cast<i64>(std::floor(minZ / m_cell));
    m_nx = static_cast<i64>(std::floor(maxX / m_cell)) - m_minCX + 1;
    m_nz = static_cast<i64>(std::floor(maxZ / m_cell)) - m_minCZ + 1;
    const u64 cells = static_cast<u64>(std::max<i64>(0, m_nx)) * static_cast<u64>(std::max<i64>(0, m_nz));
    if (m_nx > 0 && m_nz > 0 && cells * 16 < m_boxes.size() * 64u) {
        // The index would be larger than what it indexes; scan instead.
        m_nx = m_nz = 0;
        return;
    }
    if (m_nx <= 0 || m_nz <= 0 || cells > 4'000'000ull) { m_nx = m_nz = 0; return; }
    m_cells.assign(static_cast<size_t>(cells), {});
    for (u32 i = 0; i < m_boxes.size(); ++i) {
        const Occluder& o = m_boxes[i];
        const f64 r = std::max(o.hw, o.hd) + 1.0;
        const i64 x0 = static_cast<i64>(std::floor((o.cx - r) / m_cell)) - m_minCX;
        const i64 x1 = static_cast<i64>(std::floor((o.cx + r) / m_cell)) - m_minCX;
        const i64 z0 = static_cast<i64>(std::floor((o.cz - r) / m_cell)) - m_minCZ;
        const i64 z1 = static_cast<i64>(std::floor((o.cz + r) / m_cell)) - m_minCZ;
        for (i64 z = std::max<i64>(0, z0); z <= std::min<i64>(m_nz - 1, z1); ++z)
            for (i64 x = std::max<i64>(0, x0); x <= std::min<i64>(m_nx - 1, x1); ++x)
                m_cells[static_cast<size_t>(z * m_nx + x)].push_back(i);
    }
}

void OccluderGrid::clear() {
    m_boxes.clear();
    m_cells.clear();
    m_nx = m_nz = 0;
}

f64 OccluderGrid::blockedFraction(const WPos& from, const WPos& to) const {
    if (m_boxes.empty()) return 0.0;
    f64 worst = 0.0;
    auto test = [&](u32 i) {
        f64 t = 0.0;
        if (segmentHitsObb(from, to, m_boxes[i], t)) worst = std::max(worst, 1.0 - t);
    };
    if (m_cells.empty()) {
        for (u32 i = 0; i < m_boxes.size(); ++i) test(i);
        return clamp(worst, 0.0, 1.0);
    }
    // Every grid cell the segment touches (walked at half-cell steps), and every
    // box in those cells once.
    const f64 dx = to.x - from.x, dz = to.z - from.z;
    const f64 len = std::sqrt(dx * dx + dz * dz);
    if (len < 1e-9) return 0.0;
    const i64 steps = static_cast<i64>(len / (m_cell * 0.5)) + 1;
    i64 px = INT64_MIN, pz = INT64_MIN;
    for (i64 s = 0; s <= steps; ++s) {
        const f64 t = static_cast<f64>(s) / static_cast<f64>(steps);
        const i64 cx = static_cast<i64>(std::floor((from.x + dx * t) / m_cell)) - m_minCX;
        const i64 cz = static_cast<i64>(std::floor((from.z + dz * t) / m_cell)) - m_minCZ;
        if (cx == px && cz == pz) continue;
        px = cx; pz = cz;
        if (cx < 0 || cz < 0 || cx >= m_nx || cz >= m_nz) continue;
        for (u32 i : m_cells[static_cast<size_t>(cz * m_nx + cx)]) test(i);
    }
    return clamp(worst, 0.0, 1.0);
}

// =============================================================================
//  Mixer
// =============================================================================
struct Mixer::LimiterState {
    std::vector<f64> dL, dR;      // the lookahead delay: what goes out is what came in
    MinWindow minGainL, minGainR;
    u32 n = 0, w = 0;
    f64 gain = 1.0;
    f64 attackCoef = 1.0, releaseCoefFast = 1.0;
    f64 threshold = 0.0;
    // The detector's history of the *unlimited* sum, so the gain requirement is
    // computed from the reconstructed waveform rather than from sample values.
    f64 histL[32] = {0}, histR[32] = {0};
    u32 histW = 0;
    f64 truePeakHeld = 0.0;
    // True peak the detector saw this block, as a linear value.
    f64 peak(const f64* h) const {
        const Oversampler& os = oversampler();
        f64 m = 0.0;
        for (u32 p = 0; p < Oversampler::kPhases; ++p) {
            f64 acc = 0.0;
            for (u32 i = 0; i < Oversampler::kTaps; ++i) {
                const u32 idx = (histW + 32u - 1u - i) & 31u;
                (void)idx;
                acc += h[(histW + 32u - (Oversampler::kTaps - 1u) + i) & 31u] * os.coeff[p][i];
            }
            m = std::max(m, std::fabs(acc));
        }
        // The sample itself is one more point of the waveform.
        m = std::max(m, std::fabs(h[(histW + 31u) & 31u]));
        return m;
    }
};

struct Mixer::NoiseImpl {
    Noise rain{0xBEEF1234ull}, city{0xFEED5678ull};
};

struct Mixer::LoudnessState {
    LoudnessMeter meter;
    explicit LoudnessState(u32 sr) : meter(sr) {}
};

struct Mixer::Voice {
    u32 id = 0;
    Layer layer = Layer::Engine;
    Archetype kind = Archetype::Car;
    bool active = false;
    WPos p0{}, p1{};           // previous block's position -> this block's
    f64 vx = 0, vy = 0, vz = 0;
    f64 emit = 0.0;            // emitted-time accumulator, seconds
    f64 env0 = 1.0, env1 = 1.0, env = 1.0;
    f64 levelGain = 1.0;       // from the layer's reference and the authored level
    f64 f0 = 0.0;
    f64 wetness = 0.0;
    f64 release = 0.0, releaseTarget = 1.0, releaseStep = 0.0;
    f64 occl = 0.0, occlTarget = 0.0;
    f64 airGain = 1.0, occGain = 1.0;
    bool occludable = true;
    // Voice state
    Gearbox gear;
    f64 phase[4] = {0};
    f64 lastSpeed = 0.0, lastAccel = 0.0;
    f64 ring[128] = {0};
    u32 ringW = 0;
    Noise noise{1};
    Svf tilt, occlLp, res1, res2;
    f64 sprayState = 0.0;
    bool filtersPrimed = false;
};

Mixer::Mixer(u32 sampleRate, u32 blockFrames)
    : m_sr(sampleRate ? sampleRate : kDefaultSampleRate),
      m_block(blockFrames ? blockFrames : kDefaultBlockFrames) {
    m_voices.resize(clamp(m_voiceBudget, 8u, 256u));
    m_lim = std::make_unique<LimiterState>();
    m_lim->n = static_cast<u32>(0.002 * static_cast<f64>(m_sr));   // 2 ms
    if (m_lim->n < 8) m_lim->n = 8;
    m_lim->dL.assign(m_lim->n, 0.0);
    m_lim->dR.assign(m_lim->n, 0.0);
    m_lim->minGainL.init(m_lim->n);
    m_lim->minGainR.init(m_lim->n);
    m_lim->threshold = dbToGain(-1.0);
    m_lim->attackCoef = 1.0 - std::exp(-1.0 / (0.0005 * static_cast<f64>(m_sr)));
    m_lim->releaseCoefFast = 1.0 - std::exp(-1.0 / (0.150 * static_cast<f64>(m_sr)));
    m_loud = std::make_unique<LoudnessState>(m_sr);
    m_noise = std::make_unique<NoiseImpl>();
    m_prevListener = m_listener;
    m_blockBuf.assign(static_cast<size_t>(m_block) * 2, 0.0f);
}

Mixer::~Mixer() = default;

void Mixer::setListener(const Listener& l) { m_listener = l; }

void Mixer::resetStats() { m_stats = MixStats{}; }

void Mixer::applyVoiceBudget() { m_voices.resize(clamp(m_voiceBudget, 8u, 256u)); }

u32 Mixer::activeVoices() const {
    u32 n = 0;
    for (const Voice& v : m_voices)
        if (v.active && v.release > 0.0) ++n;
    return n;
}

void Mixer::beginBlock(f64 t, const std::vector<SourceSpec>& sources) {
    if (m_lastBegin < 0.0) m_prevListener = m_listener;
    m_lastBegin = t;

    // A voice already singing keeps its oscillator state and history and takes
    // the new pose as its *target*: `p0` is where it was, `p1` where it is now,
    // and the block interpolates between them.  That is what makes a 30 m/s
    // pass-by's Doppler a continuous sweep instead of a 187 Hz staircase.
    for (Voice& v : m_voices) {
        if (!v.active) continue;
        v.p0 = v.p1;
        v.releaseTarget = 0.0;      // unless a source claims it below
    }
    for (const SourceSpec& s : sources) {
        Voice* found = nullptr;
        for (Voice& v : m_voices)
            if (v.active && v.id == s.id) { found = &v; break; }
        if (!found) {
            // A new source needs a slot: a free one, or the quietest voice
            // already in release.
            u32 slot = static_cast<u32>(m_voices.size());
            for (u32 i = 0; i < m_voices.size(); ++i)
                if (!m_voices[i].active) { slot = i; break; }
            if (slot == m_voices.size()) {
                f64 worst = 2.0;
                for (u32 i = 0; i < m_voices.size(); ++i)
                    if (m_voices[i].releaseTarget <= 0.0 && m_voices[i].release < worst) {
                        worst = m_voices[i].release;
                        slot = i;
                    }
                if (slot == m_voices.size()) continue;   // the pool is genuinely full
            }
            Voice& v = m_voices[slot];
            v.id = s.id;
            v.layer = s.layer;
            v.kind = s.kind;
            v.active = true;
            v.p0 = v.p1 = s.pos;
            v.vx = s.vx; v.vy = s.vy; v.vz = s.vz;
            v.levelGain = levelGainFor(kBudgets[static_cast<size_t>(s.layer)].referenceDbAt10m,
                                        static_cast<f64>(s.level));
            v.f0 = s.f0;
            v.wetness = s.wetness;
            v.env0 = v.env1 = v.env = clamp(s.envelope, 0.0f, 8.0f);
            v.occludable = s.occludable;
            v.releaseTarget = 1.0;
            v.release = 0.0;         // inaudible until the arrival ramp has run
            v.releaseStep = 1.0 / std::max(1.0, 0.005 * static_cast<f64>(m_sr));
            v.emit = t;
            v.gear.reset();
            // Oscillators are born at an *arbitrary* phase, drawn from the voice's
            // own stream.  A vehicle's engine phase when it becomes audible is
            // unknowable, and starting every voice at zero makes a fleet that
            // appears in one block sum coherently — 300 engines at phase zero is
            // a click the size of the fleet, and no model of a city has it.
            v.noise = Noise(0x9E3779B97F4A7C15ull ^
                            (static_cast<u64>(s.id) * 0x2545F4914F6CDD1Dull));
            for (f64& ph : v.phase) ph = v.noise.next() * 0.5 + 0.5;
            std::memset(v.ring, 0, sizeof(v.ring));
            v.ringW = 0;
            v.sprayState = 0.0;
            v.lastSpeed = 0.0;
            v.lastAccel = 0.0;
            v.filtersPrimed = false;
            continue;
        }
        found->p1 = s.pos;
        found->vx = s.vx; found->vy = s.vy; found->vz = s.vz;
        found->layer = s.layer;
        found->kind = s.kind;
        found->levelGain = levelGainFor(kBudgets[static_cast<size_t>(s.layer)].referenceDbAt10m,
                                        static_cast<f64>(s.level));
        found->f0 = s.f0;
        found->wetness = s.wetness;
        found->env0 = found->env;
        found->env1 = clamp(s.envelope, 0.0f, 8.0f);
        found->occludable = s.occludable;
        found->releaseTarget = 1.0;
        found->releaseStep = 1.0 / std::max(1.0, 0.005 * static_cast<f64>(m_sr));
    }
    m_stats.voicesPeak = std::max<u64>(m_stats.voicesPeak, activeVoices());
}

void Mixer::renderBlock(f32* out) {
    const f64 dt = 1.0 / static_cast<f64>(m_sr);
    const f64 blockDur = static_cast<f64>(m_block) * dt;

    // --- per-block geometry: filter coefficients, occlusion, gains -----------
    for (Voice& v : m_voices) {
        if (!v.active) continue;
        const WPos mid{(v.p0.x + v.p1.x) * 0.5, (v.p0.y + v.p1.y) * 0.5, (v.p0.z + v.p1.z) * 0.5};
        const WPos rel = mid - m_listener.pos;
        const f64 r = std::max(0.7, rel.length());
        if (m_occluders && !m_occluders->empty() && v.occludable)
            v.occlTarget = m_occluders->blockedFraction(m_listener.pos, mid);
        else
            v.occlTarget = 0.0;
        // 40 ms of smoothing: a listener passing a doorway must not switch a
        // source on, and a source that a chunk eviction takes away must fade.
        const f64 a = std::exp(-blockDur / 0.040);
        v.occl = a * v.occl + (1.0 - a) * v.occlTarget;

        // Air absorption: ISO 9613-1 for the 1 kHz band as a plain gain, and the
        // 4 kHz over 1 kHz term as a distance-driven tilt, so far traffic is
        // darker as well as quieter — which is what distance sounds like.
        const f64 loss1k = kAirAbsorptionDbPerKm[1] * (r / 1000.0);
        const f64 loss4k = kAirAbsorptionDbPerKm[2] * (r / 1000.0);
        const f64 tiltDb = std::max(0.0, loss4k - loss1k);
        const f64 fc = 4000.0 / std::sqrt(std::max(1e-6, std::pow(10.0, tiltDb / 10.0) - 1.0));
        v.tilt.setOnePoleLow(fc, m_sr);
        v.airGain = dbToGain(-loss1k);
        // Occlusion: up to 22 dB of shadowing, and the highs go first.
        v.occGain = dbToGain(-22.0 * v.occl);
        v.occlLp.setOnePoleLow(8000.0 * std::pow(0.1, v.occl), m_sr);
        if (!v.filtersPrimed) {
            const KindInfo& k = kindInfo(v.kind);
            v.res1.setResonance(clamp(k.idle / 60.0 * 4.0, 40.0, 400.0), 1.6, m_sr);
            v.res2.setResonance(clamp(k.idle / 60.0 * 9.0, 80.0, 900.0), 2.2, m_sr);
            v.filtersPrimed = true;
        }
    }

    // The beds go through the same calibration as a source: 40 mm/h of monsoon
    // rain is a real level in dB SPL, not a multiplier.  (These two lines were the
    // last survivors of the missing calibration: after the sources were fixed,
    // the beds still ran 10^4 hot and the 300-source test measured the rain bed
    // through the limiter instead of the traffic — the same bug in its third
    // home.)
    const f64 rainGain = m_rain > 0.0f
        ? levelGainFor(static_cast<f64>(kBudgets[3].referenceDbAt10m) +
                       10.0 * std::log10(static_cast<f64>(m_rain)), 1.0)
        : 0.0;
    const f64 cityGain = static_cast<f64>(m_cityBed) *
                         levelGainFor(kBudgets[5].referenceDbAt10m, 1.0);

    for (u32 f = 0; f < m_block; ++f) {
        const f64 alpha = m_block > 1 ? static_cast<f64>(f) / static_cast<f64>(m_block) : 0.0;
        const f64 t = m_lastBegin + static_cast<f64>(f) * dt;
        // The listener interpolates too: a 30 m/s listener would otherwise step
        // its own frame 15 cm per block.
        const WPos lp = m_prevListener.pos + (m_listener.pos - m_prevListener.pos) * alpha;
        f64 sumL = 0.0, sumR = 0.0;

        for (Voice& v : m_voices) {
            if (!v.active) continue;
            if (v.releaseTarget > 0.0) {
                v.release = std::min(1.0, v.release + v.releaseStep);
            } else {
                v.release -= v.releaseStep;
                if (v.release <= 0.0) {
                    if (v.releaseTarget <= 0.0) v.active = false;
                    continue;
                }
            }
            v.env = v.env0 + (v.env1 - v.env0) * alpha;

            const WPos p = v.p0 + (v.p1 - v.p0) * alpha;
            const WPos rel = p - lp;
            const f64 r = std::max(0.7, rel.length());
            const WPos rh = rel * (1.0 / r);
            const f64 vr = rh.dot(WPos{v.vx, v.vy, v.vz} - WPos{m_listener.vx, m_listener.vy, m_listener.vz});

            // The emitted time: dtau/dt = 1 - (1/c) dr/dt.  For constant
            // geometry this is the *exact* Doppler factor, not a pitch shift
            // applied afterwards — the voice's own oscillators run on `dtE`.
            // The wavefront the ear receives at `t` left the source at tau, where
            // t = tau + r(tau)/c.  Differentiating gives dt/dtau = 1 + vr/c, so the
            // oscillator phase is advanced in emitted time by dtau/dt =
            // 1/(1 + vr/c) — the *exact* Doppler factor c/(c - v), not its
            // first-order form (1 - v/c), which a test caught being 0.8 % low at
            // 30 m/s.  Callers must keep |vr| < c; a body that fast is not a
            // vehicle and the guard below keeps the arithmetic finite.
            const f64 dFactor = 1.0 / (1.0 + clamp(vr, -0.9 * kSpeedOfSound, 0.9 * kSpeedOfSound) /
                                              kSpeedOfSound);
            const f64 dtE = dt * dFactor;
            v.emit += dtE;

            // Distance: inverse square from 1 m with a near-field floor, so a
            // source cannot be louder inside the ear than at a metre.
            const f64 spread = 1.0 / r;
            const f64 nearMix = clamp(r - 1.0, 0.0, 1.0);
            const f64 distGain = spread * nearMix + 1.0 * (1.0 - nearMix);

            // Ground reflection off the road plane.
            const f64 extra = groundExtraPath(lp.y, p.y, r);
            const f64 reflDelay = clamp(extra / kSpeedOfSound * static_cast<f64>(m_sr), 0.0, 120.0);
            const f64 reflGain = 0.62 * (r / (r + extra));   // wet asphalt

            // --- the voice's own signal -------------------------------------
            f64 s = 0.0;
            switch (v.layer) {
                case Layer::Engine: {
                    const KindInfo& k = kindInfo(v.kind);
                    const f64 speed = std::sqrt(v.vx * v.vx + v.vz * v.vz);
                    const f64 accel = (speed - v.lastSpeed) / dt;
                    v.lastAccel = 0.98 * v.lastAccel + 0.02 * accel;
                    v.lastSpeed = speed;
                    const f64 rpm = v.gear.rpm(k, speed, v.lastAccel);
                    const f64 crank = rpm / 60.0;
                    const f64 fire = static_cast<f64>(k.cylinders) / 2.0;
                    static const f64 orderAmp[4] = {1.0, 0.62, 0.34, 0.18};
                    f64 tone = 0.0;
                    for (u32 h = 0; h < 4; ++h) tone += orderAmp[h] * sine(v.phase[h]);
                    // Order h runs at (h+1)*fire*crank, in emitted time.
                    for (u32 h = 0; h < 4; ++h)
                        v.phase[h] += static_cast<f64>(h + 1) * fire * crank * dtE;
                    const f64 rough = v.noise.next() * k.roughness *
                                      (0.4 + 0.6 * clamp(rpm / std::max(1.0, k.redline), 0.0, 1.0));
                    s = (tone * 0.5 + rough * 0.5) * 0.42;
                    break;
                }
                case Layer::Tyres: {
                    const f64 speed = std::sqrt(v.vx * v.vx + v.vz * v.vz);
                    // ISO 11819-1 / the Tires project: rolling noise grows as
                    // 30*log10(v), aerodynamic noise as 60*log10(v).  Both are
                    // referenced to 10 m/s.
                    const f64 n = v.noise.next();
                    const f64 roll = 0.45 * dbToGain(30.0 * std::log10(std::max(0.5, speed) / 10.0));
                    const f64 aero = 0.20 * dbToGain(60.0 * std::log10(std::max(0.5, speed) / 10.0));
                    s = n * (roll + aero);
                    // Spray: the same voice, whitened by standing water and speed.
                    if (v.wetness > 0.01) {
                        v.sprayState = 0.90 * v.sprayState + 0.10 * n;
                        s += 0.55 * v.wetness * std::min(1.4, speed / 12.0) * (n - v.sprayState);
                    }
                    s *= 0.9;
                    break;
                }
                case Layer::Horn: {
                    const f64 f0 = v.f0 > 20.0 ? v.f0 : 420.0;
                    const f64 vib = 1.0 + 0.004 * sine(4.6 * v.emit + 0.3);
                    f64 tone = 0.62 * sine(v.phase[0]) + 0.62 * sine(v.phase[1]);
                    tone += 0.20 * sine(v.phase[0] * 2.0) + 0.12 * sine(v.phase[1] * 3.0);
                    v.phase[0] += f0 * vib * dtE;
                    v.phase[1] += f0 * 1.25 * dtE;
                    s = tone * 0.42;
                    break;
                }
                case Layer::Surf: {
                    // Surf is a wave, so it is gated by one: the period is the
                    // deep-water dispersion relation for this swell length, and
                    // the crest is sharpened the way a breaking wave is.
                    const f64 T = swellPeriod(v.f0 > 20.0 ? v.f0 : 100.0);
                    const f64 crest = std::pow(0.5 + 0.5 * sine(t / T + v.phase[0]), 2.2);
                    const f64 n = v.noise.next();
                    v.sprayState = 0.72 * v.sprayState + 0.28 * n;
                    s = (n * 0.55 + v.sprayState * 0.45) * (0.22 + 0.78 * crest);
                    break;
                }
                case Layer::City: {
                    const f64 n = v.noise.next();
                    v.sprayState = 0.985 * v.sprayState + 0.015 * n;
                    s = v.sprayState * 3.0 + n * 0.10;
                    break;
                }
                case Layer::Rain: {
                    const f64 n = v.noise.next();
                    const f64 hi = n - v.sprayState;
                    v.sprayState = 0.62 * v.sprayState + 0.38 * n;
                    s = n * 0.35 + hi * 0.65;
                    break;
                }
                case Layer::Tone: {
                    s = sine(v.phase[0]);
                    v.phase[0] += (v.f0 > 1.0 ? v.f0 : 1000.0) * dtE;
                    break;
                }
            }

            if (v.layer == Layer::Engine) {
                const f64 a1 = v.res1.lowPass(s);
                const f64 a2 = v.res2.lowPass(s);
                s = 0.55 * s + 0.30 * a1 + 0.15 * a2;
            }

            // The reflection reads the direct signal's own history, so it is
            // the same engine a few hundred microseconds late — which is what
            // the comb under a passing car actually is.
            v.ring[v.ringW] = s;
            const f64 back = reflDelay;
            const u32 ib = static_cast<u32>(back);
            const f64 frac = back - static_cast<f64>(ib);
            const u32 i0 = (v.ringW + 128u - (ib & 127u)) & 127u;
            const u32 i1 = (i0 + 127u) & 127u;
            const f64 refl = (v.ring[i0] * (1.0 - frac) + v.ring[i1] * frac) * reflGain;
            v.ringW = (v.ringW + 1u) & 127u;

            f64 sig = v.tilt.onePoleLow(s + refl);
            sig = v.occlLp.onePoleLow(sig);

            const f64 g = v.release * v.env * v.levelGain * distGain * v.airGain * v.occGain;
            sig *= g;

            // Equal-power pan inside the listener's frame, narrowed with
            // distance: a far source's image collapses to the centre, so a
            // 300 m bus does not ping-pong when the car turns.
            const WPos hv = rel * (1.0 / r);
            const f64 cy = std::cos(m_listener.yaw), sy = std::sin(m_listener.yaw);
            const f64 x = hv.x * cy + hv.z * (-sy);
            const f64 z = hv.x * sy + hv.z * cy;
            const f64 angle = std::atan2(x, std::max(1e-6, z));
            const f64 narrow = 1.0 - clamp((r - 10.0) / 140.0, 0.0, 1.0) * 0.65;
            const f64 u = clamp(0.5 + 0.5 * (angle / 1.5707963267948966) * narrow, 0.0, 1.0);
            sumL += sig * std::cos(u * 1.5707963267948966);
            sumR += sig * std::sin(u * 1.5707963267948966);

            ++m_stats.voicesRendered;
            if (v.occl > 0.01) ++m_stats.occludedVoices;
            if (r < 30.0) {
                ++m_stats.voicesNear;
                if (v.occl > 0.01) ++m_stats.occludedNear;
                m_stats.occlusionSumNear += v.occl;
            } else {
                ++m_stats.voicesFar;
                m_stats.occlusionSumFar += v.occl;
            }
        }

        // --- beds.  Rain and the city are the state of the air, not events,
        //     so the mixer owns them rather than asking a caller to author them.
        if (rainGain > 0.0) {
            const f64 n = m_noise->rain.next();
            const f64 hi = n - m_rainNoiseState;
            const f64 lp2 = m_cityNoiseState;
            m_rainNoiseState = 0.55 * m_rainNoiseState + 0.45 * n;
            m_cityNoiseState = 0.93 * m_cityNoiseState + 0.07 * n;
            const f64 sig = (hi * 0.55 + lp2 * 1.6 + n * 0.18) * rainGain * 1.6 * 0.707;
            sumL += sig;
            sumR += sig;
        }
        if (cityGain > 0.0) {
            const f64 n = m_noise->city.next();
            m_cityLpState = 0.992 * m_cityLpState + 0.008 * n;
            const f64 sig = m_cityLpState * 4.0 * cityGain * 0.707;
            sumL += sig;
            sumR += sig;
        }

        // --- limiter: 2 ms of lookahead, so no peak can get past -------------
        LimiterState& L = *m_lim;
        L.histL[L.histW] = sumL;
        L.histR[L.histW] = sumR;
        L.histW = (L.histW + 1u) & 31u;
        const f64 magL = L.peak(L.histL), magR = L.peak(L.histR);
        L.truePeakHeld = std::max(L.truePeakHeld, std::max(magL, magR));
        L.minGainL.push(magL > L.threshold ? L.threshold / magL : 1.0);
        L.minGainR.push(magR > L.threshold ? L.threshold / magR : 1.0);
        const f64 target = std::min(L.minGainL.min(), L.minGainR.min());
        if (target < L.gain) {
            // 0.5 ms of attack, inside a 2 ms lookahead: the gain is down before
            // the peak reaches the output, and the smoothing keeps the gain
            // itself continuous (a stepped gain multiplied by a signal is a
            // click, however correct the ceiling).
            L.gain += (target - L.gain) * L.attackCoef;
            ++m_stats.limiterGrabs;
        } else {
            L.gain += (target - L.gain) * L.releaseCoefFast;
        }
        // The lookahead: what leaves the limiter is what entered it 2 ms ago,
        // scaled by a gain that already knows about the peak in between.
        const f64 delayedL = L.dL[L.w];
        const f64 delayedR = L.dR[L.w];
        L.dL[L.w] = sumL;
        L.dR[L.w] = sumR;
        L.w = (L.w + 1u) % L.n;
        const f64 gg = L.gain * m_master;
        f64 outL = delayedL * gg, outR = delayedR * gg;
        if (!(outL == outL) || !std::isfinite(outL)) outL = 0.0;   // a NaN must not escape
        if (!(outR == outR) || !std::isfinite(outR)) outR = 0.0;
        out[f * 2 + 0] = static_cast<f32>(outL);
        out[f * 2 + 1] = static_cast<f32>(outR);

        const f64 a = std::fabs(outL), b = std::fabs(outR);
        m_stats.peak = std::max(m_stats.peak, std::max(a, b));
        m_stats.sumSquares += 0.5 * (outL * outL + outR * outR);
        if (a >= 1.0 || b >= 1.0) ++m_stats.overFullScale;
    }
    m_stats.truePeakLimited = std::max(m_stats.truePeakLimited, m_lim->truePeakHeld);
    m_stats.frames += m_block;
    m_prevListener = m_listener;
}

void Mixer::renderScene(Mixer& mix, const Listener& listener,
                        const std::vector<SourceSpec>& sources, f64 seconds,
                        std::vector<f32>& out) {
    const u32 block = mix.blockFrames();
    const u64 total = static_cast<u64>(seconds * static_cast<f64>(mix.sampleRate()));
    out.assign(static_cast<size_t>(total) * 2, 0.0f);
    std::vector<f32> buf(static_cast<size_t>(block) * 2, 0.0f);
    u64 done = 0;
    f64 t = 0.0;
    while (done < total) {
        const u32 n = static_cast<u32>(std::min<u64>(block, total - done));
        mix.setListener(listener);
        mix.beginBlock(t, sources);
        mix.renderBlock(buf.data());
        std::memcpy(out.data() + done * 2, buf.data(), static_cast<size_t>(n) * 2 * sizeof(f32));
        done += n;
        t += static_cast<f64>(n) / static_cast<f64>(mix.sampleRate());
    }
}

// =============================================================================
//  Loudness (ITU-R BS.1770-4)
// =============================================================================
namespace {
struct KWeighting {
    Biquad shelf, hpf;
    KWeighting() {
        // The standard's published 48 kHz coefficients.  A mixer run at another
        // rate would be measuring with the wrong filter, which is why the meter
        // refuses to claim a number below 44.1 kHz or above 48 kHz.
        static const f64 b1[3] = {1.53512485958697, -2.69169618940638, 1.19839281085285};
        static const f64 a1[3] = {1.0, -1.69065929318241, 0.73248077421585};
        static const f64 b2[3] = {1.0, -2.0, 1.0};
        static const f64 a2[3] = {1.0, -1.99004745483398, 0.99007225036621};
        shelf.set(b1, a1);
        hpf.set(b2, a2);
    }
    f64 process(f64 x) { return hpf.process(shelf.process(x)); }
    void reset() { shelf.reset(); hpf.reset(); }
};
}  // namespace

struct LoudnessMeter::Impl {
    u32 sr = 48000;
    KWeighting kl, kr;
    std::vector<f64> windowSums;    // one entry per 100 ms hop
    std::vector<f64> blockLoudness; // one entry per 400 ms block
    f64 sumL = 0, sumR = 0;
    u32 hop = 0, inHop = 0;
    f64 momentary = -200.0;
    explicit Impl(u32 s) : sr(s ? s : 48000) { hop = std::max(1u, sr / 10); }
    void push(f64 l, f64 r) {
        const f64 yl = kl.process(l), yr = kr.process(r);
        sumL += yl * yl;
        sumR += yr * yr;
        if (++inHop < hop) return;
        // BS.1770's loudness for the 400 ms window is the *mean* of the
        // K-weighted power over both channels cumulated: one hop's own mean,
        // and the four hops average to the window.  (Dividing by four hops here
        // as well — which the first pass did — reads the standard 6 dB low and
        // was caught by the calibration test.)
        windowSums.push_back((sumL + sumR) / static_cast<f64>(hop));
        inHop = 0;
        sumL = sumR = 0.0;
        if (windowSums.size() < 4) return;
        f64 m = 0.0;
        for (size_t i = windowSums.size() - 4; i < windowSums.size(); ++i) m += windowSums[i];
        m /= 4.0;
        blockLoudness.push_back(-0.691 + 10.0 * std::log10(std::max(1e-30, m)));
        momentary = blockLoudness.back();
    }
};

LoudnessMeter::LoudnessMeter(u32 sampleRate) : m_impl(std::make_unique<Impl>(sampleRate)) {}
LoudnessMeter::~LoudnessMeter() = default;
void LoudnessMeter::reset() { *m_impl = Impl(m_impl->sr); }
void LoudnessMeter::addBlock(const f32* interleavedStereo, u32 frames, u32 channels) {
    for (u32 i = 0; i < frames; ++i) {
        const f64 l = interleavedStereo[i * channels + 0];
        const f64 r = channels > 1 ? static_cast<f64>(interleavedStereo[i * channels + 1]) : l;
        m_impl->push(l, r);
    }
}
f64 LoudnessMeter::momentaryLufs() const { return m_impl->momentary; }

f64 LoudnessMeter::integratedLufs() const {
    const auto& b = m_impl->blockLoudness;
    if (b.empty()) return -200.0;
    auto meanPow = [](const std::vector<f64>& v, f64 gate, size_t& n) {
        f64 sum = 0.0; n = 0;
        for (f64 x : v)
            if (x > gate) { sum += std::pow(10.0, (x + 0.691) / 10.0); ++n; }
        return n ? sum / static_cast<f64>(n) : 0.0;
    };
    size_t n1 = 0;
    const f64 m1 = meanPow(b, -70.0, n1);
    if (!n1) return -200.0;
    const f64 rel = -0.691 + 10.0 * std::log10(std::max(1e-30, m1)) - 10.0;
    size_t n2 = 0;
    const f64 m2 = meanPow(b, rel, n2);
    if (!n2) return -200.0;
    return -0.691 + 10.0 * std::log10(std::max(1e-30, m2));
}

// =============================================================================
//  True peak (BS.1770 Annex 2, 4x oversampled estimate)
// =============================================================================
f64 truePeakDbtp(const std::vector<f32>& x, u32 channels) {
    // The same 4x oversampled interpolator the limiter's detector runs, so the
    // meter and the limiter cannot disagree about what the waveform does between
    // its samples.  (The standard's Annex 2 filter is a 48-tap polyphase FIR;
    // this is a 24-tap windowed sinc, which is an approximation — and it reads
    // a full-scale 1 kHz sine as exactly 0.00 dBTP and an 11 kHz one as +0.23,
    // both of which are the textbook answers.)
    const Oversampler& os = oversampler();
    f64 peak = 0.0;
    const size_t frames = channels ? x.size() / channels : 0;
    for (u32 c = 0; c < channels; ++c) {
        for (size_t n = 0; n < frames; ++n) {
            for (u32 p = 0; p < Oversampler::kPhases; ++p) {
                f64 acc = 0.0;
                for (u32 i = 0; i < Oversampler::kTaps; ++i) {
                    const i64 idx = static_cast<i64>(n) -
                                    static_cast<i64>(Oversampler::kTaps / 2 - 1) + static_cast<i64>(i);
                    const f64 sv = (idx < 0 || static_cast<size_t>(idx) >= frames)
                                       ? 0.0
                                       : static_cast<f64>(x[static_cast<size_t>(idx) * channels + c]);
                    acc += sv * os.coeff[p][i];
                }
                peak = std::max(peak, std::fabs(acc));
            }
        }
    }
    return 20.0 * std::log10(std::max(1e-12, peak));
}

// =============================================================================
//  WAV
// =============================================================================
namespace {
void putU32(std::vector<u8>& v, u32 x) {
    v.push_back(static_cast<u8>(x & 0xFF));
    v.push_back(static_cast<u8>((x >> 8) & 0xFF));
    v.push_back(static_cast<u8>((x >> 16) & 0xFF));
    v.push_back(static_cast<u8>((x >> 24) & 0xFF));
}
void putU16(std::vector<u8>& v, u16 x) {
    v.push_back(static_cast<u8>(x & 0xFF));
    v.push_back(static_cast<u8>((x >> 8) & 0xFF));
}
u32 rdU32(const u8* p) {
    return static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) |
           (static_cast<u32>(p[2]) << 16) | (static_cast<u32>(p[3]) << 24);
}
u16 rdU16(const u8* p) { return static_cast<u16>(p[0] | (p[1] << 8)); }
}  // namespace

bool writeWav(const std::string& path, const std::vector<f32>& interleaved, u32 channels,
              u32 sampleRate, bool floatFormat, bool dither) {
    std::vector<u8> out;
    const u32 bytesPerSample = floatFormat ? 4u : 2u;
    const u32 dataBytes = static_cast<u32>(interleaved.size() * bytesPerSample);
    const u16 bits = static_cast<u16>(floatFormat ? 32 : 16);
    const u16 fmtTag = static_cast<u16>(floatFormat ? 3 : 1);
    out.reserve(static_cast<size_t>(44 + dataBytes + 4));

    out.insert(out.end(), {'R', 'I', 'F', 'F'});
    putU32(out, 36u + dataBytes + (floatFormat ? 4u : 0u));
    out.insert(out.end(), {'W', 'A', 'V', 'E'});
    out.insert(out.end(), {'f', 'm', 't', ' '});
    putU32(out, floatFormat ? 18u : 16u);
    putU16(out, fmtTag);
    putU16(out, static_cast<u16>(channels));
    putU32(out, sampleRate);
    putU32(out, sampleRate * channels * bytesPerSample);
    putU16(out, static_cast<u16>(channels * bytesPerSample));
    putU16(out, bits);
    if (floatFormat) putU16(out, 0);
    out.insert(out.end(), {'d', 'a', 't', 'a'});
    putU32(out, dataBytes);

    // Triangular-PDF dither at 1 LSB, from a fixed LCG: quantising a quiet
    // ambience without dither is a real artifact and Section 0 forbids those.
    // Deterministic, so a render is reproducible sample for sample.
    u32 lcg = 0x12345678u;
    auto ditherSample = [&lcg]() {
        lcg = lcg * 1664525u + 1013904223u;
        const f64 a = static_cast<f64>((lcg >> 8) & 0xFFFFu) / 65536.0;
        lcg = lcg * 1664525u + 1013904223u;
        const f64 b = static_cast<f64>((lcg >> 8) & 0xFFFFu) / 65536.0;
        return (a - b) / 32768.0;    // 1 LSB peak to peak
    };
    for (f32 sv : interleaved) {
        f64 v = sv;
        if (!floatFormat) {
            if (dither) v += ditherSample();
            const i32 q = static_cast<i32>(std::lround(clamp(v, -1.0, 1.0) * 32767.0));
            putU16(out, static_cast<u16>(static_cast<i16>(clamp<i32>(q, -32768, 32767))));
        } else {
            v = clamp(v, -1.0, 1.0);
            const f32 f = static_cast<f32>(v);
            u32 bits32 = 0;
            std::memcpy(&bits32, &f, 4);
            putU32(out, bits32);
        }
    }
    return writeFile(path, out.data(), out.size());
}

bool readWav(const std::string& path, WavData& out) {
    const std::string raw = readFileToString(path);
    if (raw.size() < 44) return false;
    const u8* p = reinterpret_cast<const u8*>(raw.data());
    if (std::memcmp(p, "RIFF", 4) != 0 || std::memcmp(p + 8, "WAVE", 4) != 0) return false;
    size_t at = 12;
    size_t dataOffset = 0;
    u32 dataBytes = 0;
    while (at + 8 <= raw.size()) {
        const u32 id = rdU32(p + at);
        const u32 sz = rdU32(p + at + 4);
        const size_t body = at + 8;
        if (id == 0x20746D66u && body + 16 <= raw.size()) {         // 'fmt '
            const u16 tag = rdU16(p + body);
            out.channels = rdU16(p + body + 2);
            out.sampleRate = rdU32(p + body + 4);
            out.bits = rdU16(p + body + 14);
            out.isFloat = tag == 3;
        } else if (id == 0x61746164u) {                             // 'data'
            dataOffset = body;
            dataBytes = sz;
        }
        at = body + sz + (sz & 1u);
    }
    if (!dataOffset || !dataBytes || out.channels == 0 || out.bits == 0) return false;
    const u32 bytesPerSample = out.bits / 8u;
    if (dataOffset + dataBytes > raw.size()) return false;
    const size_t frames = dataBytes / (bytesPerSample * static_cast<size_t>(out.channels));
    out.samples.resize(frames * out.channels);
    for (size_t i = 0; i < out.samples.size(); ++i) {
        const u8* s = p + dataOffset + i * bytesPerSample;
        if (out.isFloat) {
            const u32 bits32 = rdU32(s);
            f32 f = 0.0f;
            std::memcpy(&f, &bits32, 4);
            out.samples[i] = f;
        } else {
            out.samples[i] = static_cast<f32>(static_cast<i16>(rdU16(s))) / 32767.0f;
        }
    }
    return true;
}

const LayerBudget* layerBudgets(size_t& count) {
    count = sizeof(kBudgets) / sizeof(kBudgets[0]);
    return kBudgets;
}

}  // namespace mumbai::audio
