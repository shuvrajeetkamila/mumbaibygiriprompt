// =============================================================================
//  MUMBAI 1:1 EXACT — engine/audio/audio.h
//
//  Roadmap A9: the client engine's audio.  The specification does not detail
//  audio, so this is a *designed* subsystem, and the design rule is the one the
//  rest of the engine follows: sounds are computed from the world's own numbers
//  and from physics, not from samples.  There is no sample library here.  Every
//  voice is synthesised, which is what makes the same code run on a phone, at
//  any distance, with no assets in the 150 GB budget.
//
//  WHAT IS PHYSICAL AND WHAT IS MODELLED
//  ------------------------------------
//  * Propagation delay is real: a voice's oscillator phase is advanced by the
//    *emitted* time tau, where dtau/dt = 1 - (1/c) * dr/dt (r = range,
//    c = 343 m/s).  For constant geometry that is the exact Doppler factor, not
//    a pitch-shift approximation, and for a source at rest it is exactly 1.0.
//    A test measures the produced pitch against c/(c - v) to 0.5 %.
//  * Spreading loss is real: inverse distance referenced to 1 m, with a 1 m
//    near-field floor (a source cannot be louder inside the ear than at 1 m).
//  * Air absorption is cited: ISO 9613-1 excess attenuation at 20 C / 70 % RH
//    (3.5 / 9.0 / 76.0 dB per km at 500 Hz / 1 kHz / 4 kHz), implemented as a
//    distance-driven tilt so a far engine is *darker*, not just quieter — which
//    is what distance actually sounds like.
//  * Rolling noise follows the published law for tyre/road noise: the level
//    grows as 30*log10(v), and aerodynamic noise as 60*log10(v) (ISO 11819-1 /
//    the Tires European project).  The two are separate voices here.
//  * Occlusion is computed, not guessed: a ray from the ear to the source is
//    tested against the *chunk's own* building proxies (oriented boxes), and the
//    blocked fraction drives both a gain and a low-pass.  Nothing is occluded by
//    a material the world does not contain, and a diffracted-around-the-edge
//    source still sounds like a city rather than a mute.
//  * The ground is a first-order reflection off the road surface (the comb under
//    every passing car), with the reflection coefficient of wet asphalt.
//  * Rain is a bed whose spectrum and level follow the rain rate in mm/h (the
//    specification's monsoon: >100 mm/h), with spray noise tied to the tyre
//    voice and the surface wetness.
//  * The sea is not a loop: the wave period comes from the deep-water dispersion
//    relation for the specification's own swell (lambda 80-150 m -> T 7.2-9.8 s),
//    and the surf voice's level follows the crest.
//
//  WHAT IT MUST NEVER DO (Section 0: zero artifacts)
//  ------------------------------------------------
//  * Never clip.  A lookahead limiter at -1 dBTP is the last stage, and the
//    harness asserts zero samples at or above full scale, verified by the
//    mixer's own counter *and* independently by the writer.
//  * Never click.  A voice that enters or leaves the resident set ramps over
//    5 ms; a source that a chunk eviction removes is a fade, not a step.  The
//    test asserts the largest sample-to-sample step across an eviction.
//  * Never pop on a floating-origin shift.  This is structural: the mixer's
//    world coordinates are f64 metres from the projection origin and never see
//    the render origin, so a shift is invisible to it.  The test renders the
//    same scene twice with every position translated by whole macro-cells and
//    asserts the output is bit-identical.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <string>
#include <vector>

namespace mumbai::audio {

// -----------------------------------------------------------------------------
//  Constants
// -----------------------------------------------------------------------------
inline constexpr f64 kSpeedOfSound = 343.0;    // m/s, dry air at 20 C
inline constexpr u32 kDefaultSampleRate = 48000;
inline constexpr u32 kDefaultBlockFrames = 256;

// ISO 9613-1, 20 C, 70 % relative humidity: atmospheric absorption coefficient
// in dB per kilometre at 500 Hz, 1 kHz and 4 kHz.
inline constexpr f64 kAirAbsorptionDbPerKm[3] = {3.5, 9.0, 76.0};

// -----------------------------------------------------------------------------
//  World position.  f64 metres from the projection origin, deliberately: the
//  renderer may translate everything by whole macro-cells, and audio must not
//  notice (Section 4A: a shift must not produce a pop or an audible jump).
// -----------------------------------------------------------------------------
struct WPos {
    f64 x = 0, y = 0, z = 0;
    WPos() = default;
    WPos(f64 x_, f64 y_, f64 z_) : x(x_), y(y_), z(z_) {}
    WPos operator+(const WPos& o) const { return {x + o.x, y + o.y, z + o.z}; }
    WPos operator-(const WPos& o) const { return {x - o.x, y - o.y, z - o.z}; }
    WPos operator*(f64 s) const { return {x * s, y * s, z * s}; }
    f64 dot(const WPos& o) const { return x * o.x + y * o.y + z * o.z; }
    f64 length() const { return std::sqrt(x * x + y * y + z * z); }
};

// -----------------------------------------------------------------------------
//  DSP primitives.  Small on purpose: every one of them is used by a test.
// -----------------------------------------------------------------------------

// Zero-delay-feedback state variable filter (Zavalishin's topology-preserving
// transform).  Stable at any cutoff up to Nyquist, and its low-pass and
// high-pass outputs at the same state sum to the input — the identity the
// filter test checks.
struct Svf {
    f64 g = 0, k = 1, a1 = 0, a2 = 0, a3 = 0;
    f64 ic1eq = 0, ic2eq = 0;

    void setCutoff(f64 fc, f64 sr);
    void setLowPass(f64 fc, f64 sr) { setCutoff(fc, sr); k = 1.0; a3 = 0.0; }
    void setHighPass(f64 fc, f64 sr) { setCutoff(fc, sr); k = 1.0; a3 = 0.0; }
    // A resonant band.  Q = 1/sqrt(2) is maximally flat; k = 1/Q is the
    // damping, so a resonator is `setResonance(fc, Q, sr)` and the identity the
    // filter test checks is low + band*k + high == input.
    void setResonance(f64 fc, f64 q, f64 sr);
    // A one-pole low-pass (the "leaky integrator" tap of the same topology):
    // used where a 6 dB/oct slope is wanted, e.g. the air-absorption tilt.
    void setOnePoleLow(f64 fc, f64 sr);
    f64 lowPass(f64 x);
    f64 highPass(f64 x);
    f64 bandPass(f64 x);
    f64 onePoleLow(f64 x);
    f64 damping() const { return k; }
    void reset() { ic1eq = ic2eq = 0.0; }
};

// Direct-form-1 biquad, used where the *specified* coefficients matter more than
// the topology: the BS.1770 K-weighting loudness filter is published as biquads.
struct Biquad {
    f64 b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    f64 x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    void set(const f64 b[3], const f64 a[3]) { b0 = b[0]; b1 = b[1]; b2 = b[2]; a1 = a[1]; a2 = a[2]; }
    f64 process(f64 x) {
        const f64 y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
    void reset() { x1 = x2 = y1 = y2 = 0.0; }
};

// -----------------------------------------------------------------------------
//  Mixer inputs
// -----------------------------------------------------------------------------
enum class Layer : u8 {
    Engine = 0,   // combustion orders, loaded by the driveline
    Tyres,        // rolling noise (+ wet spray, which is the same voice sprayier)
    Horn,         // event voice, fire-and-forget
    Rain,
    Surf,         // sea / creek shore
    City,         // the murmur of the far city, from built density
    Tone,         // a calibration tone.  Used by the doppler caliper and by the
                  // unit tests, never authored into a scene.
};

enum class Archetype : u8 { Car = 0, Taxi, Auto, Bike, Bus, Truck };

struct Listener {
    WPos pos{0, 1.2, 0};     // ear position, world metres
    f64 vx = 0, vy = 0, vz = 0;
    f64 yaw = 0;             // heading, the world's yaw convention (atan2(dx, dz))
    bool inVehicle = true;   // an enclosed cabin: its own low-pass and its own
                             // engine heard through the shell, no Doppler on self
};

struct SourceSpec {
    u32 id = 0;
    Layer layer = Layer::Engine;
    Archetype kind = Archetype::Car;
    WPos pos{};
    f64 vx = 0, vy = 0, vz = 0;
    f32 level = 1.0f;        // authored gain (0..4)
    f32 f0 = 0.0f;           // Hz: horn fundamental, tone frequency, idle speed
    f32 rpm = 0.0f;          // engine speed, if the caller knows it
    f32 rain = 0.0f;         // mm/h  (Rain layer)
    f32 wetness = 0.0f;      // 0 dry .. 1 standing water (Tyres layer: spray)
    f32 envelope = 1.0f;     // per-source shaping for one-shots (horns)
    bool occludable = true;
};

// -----------------------------------------------------------------------------
//  Measured output.  Everything here is what the harness publishes; nothing is
//  claimed that is not counted.
// -----------------------------------------------------------------------------
struct MixStats {
    u64 frames = 0;
    u64 voicesRendered = 0;      // voice-frames
    u64 voicesPeak = 0;
    u64 occludedVoices = 0;      // voice-frames that were at least partly blocked
    // ...split by range, because "88 % of voice-frames are occluded" means
    // nothing until you know whether the *near field* is.  A source 200 m away in
    // dense Mumbai is usually behind a building; the taxi alongside must not be.
    u64 voicesNear = 0, occludedNear = 0;
    f64 occlusionSumNear = 0.0, occlusionSumFar = 0.0;
    u64 voicesFar = 0;
    u64 limiterGrabs = 0;        // samples the limiter pulled down
    u64 overFullScale = 0;       // must be 0 — asserted by the harness
    f64 truePeakLimited = 0.0;   // the detector's own true-peak reading, linear
    f64 peak = 0.0;              // linear, full scale = 1
    f64 sumSquares = 0.0;        // ...mean of squares is rmsFromStats()
    f64 integratedLufs = 0.0;
    f64 truePeakDbtp = 0.0;
};

// -----------------------------------------------------------------------------
//  Occlusion geometry: the world's own building proxies, indexed by a uniform
//  grid, exactly as they come out of a chunk (oriented boxes, yaw included).
// -----------------------------------------------------------------------------
struct Occluder {
    f64 cx = 0, cz = 0;        // centre, world metres
    f32 hw = 0, hd = 0;        // half extents along local x / z
    f32 yaw = 0;               // radians
    f32 baseY = 0, topY = 0;   // vertical extent
};

class OccluderGrid {
public:
    void build(const std::vector<Occluder>& boxes, f64 cellSize = 64.0);
    void clear();
    // Fraction of the segment (from -> to) that passes through geometry, 0..1.
    // Height is respected: a source above a roof is not occluded by that roof.
    f64 blockedFraction(const WPos& from, const WPos& to) const;
    size_t size() const { return m_boxes.size(); }
    bool empty() const { return m_boxes.empty(); }

private:
    std::vector<Occluder> m_boxes;
    f64 m_cell = 64.0;
    i64 m_minCX = 0, m_minCZ = 0;
    i64 m_nx = 0, m_nz = 0;
    std::vector<std::vector<u32>> m_cells;
};

// -----------------------------------------------------------------------------
//  The mixer.  One instance owns a small fixed pool of voices; a game submits
//  the sources within earshot once per block and the mixer does the rest.
//
//  Block contract: `beginBlock(t, sources)` then `renderBlock(out)` with
//  `out` holding `blockFrames * 2` interleaved floats.  Positions are
//  interpolated from the previous block's submission to this one, so the
//  Doppler of a fast pass-by is continuous across the block boundary rather
//  than stepped at 200 Hz.
// -----------------------------------------------------------------------------
class Mixer {
public:
    Mixer(u32 sampleRate = kDefaultSampleRate, u32 blockFrames = kDefaultBlockFrames);
    // Declared here and defined in the .cpp: the voice pool and the limiter are
    // incomplete types at this point, and an implicitly generated destructor
    // would need them complete in every translation unit that includes this
    // header.
    ~Mixer();
    Mixer(const Mixer&) = delete;
    Mixer& operator=(const Mixer&) = delete;

    void setListener(const Listener& l);
    void setOccluders(const OccluderGrid* grid) { m_occluders = grid; }
    // The voice pool a platform can afford.  A caller that submits more sources
    // than this must rank them itself — the mixer keeps the ones it is given and
    // releases the rest, which is a *fade*, not a drop.
    void setVoiceBudget(u32 voices) { m_voiceBudget = clamp(voices, 8u, 256u); applyVoiceBudget(); }
    u32 voiceBudget() const { return m_voiceBudget; }
    void applyVoiceBudget();
    void setMasterGain(f64 g) { m_master = g; }
    // Rain bed and city bed are optional voices the mixer owns itself, so a
    // caller does not have to synthesise a weather layer per block.
    void setRain(f32 mmPerHour) { m_rain = mmPerHour; }
    void setCityBed(f32 level) { m_cityBed = level; }
    void setSeaStateLen(f32 metres) { m_swellLen = metres; }
    void setWetness(f32 w) { m_wetness = w; }

    void beginBlock(f64 t, const std::vector<SourceSpec>& sources);
    void renderBlock(f32* interleavedStereo);

    // Renders `seconds` of a scene in one call.  Used by the harness and by the
    // tests; the block path above is what a frame loop would use.
    static void renderScene(Mixer& mix, const Listener& listener,
                            const std::vector<SourceSpec>& sources,
                            f64 seconds, std::vector<f32>& interleavedOut);

    const MixStats& stats() const { return m_stats; }
    void resetStats();
    u32 activeVoices() const;

    u32 sampleRate() const { return m_sr; }
    u32 blockFrames() const { return m_block; }

private:
    struct Voice;
    std::vector<Voice> m_voices;
    u32 m_sr = kDefaultSampleRate;
    u32 m_block = kDefaultBlockFrames;
    u32 m_voiceBudget = 96;
    Listener m_listener;
    Listener m_prevListener;
    const OccluderGrid* m_occluders = nullptr;
    f64 m_master = 1.0;
    f64 m_lastBegin = -1.0;
    f32 m_rain = 0.0f, m_cityBed = 0.0f, m_wetness = 0.0f, m_swellLen = 100.0f;
    MixStats m_stats;
    // Rain and the city are the mixer's own beds: the state of the air rather
    // than events, so no caller has to synthesise them per block.
    struct NoiseState;
    f64 m_rainNoiseState = 0;
    f64 m_cityNoiseState = 0;
    f64 m_cityLpState = 0;
    // The limiter's own state, and the loudness meter's.
    struct LimiterState;
    std::unique_ptr<LimiterState> m_lim;
    struct LoudnessState;
    std::unique_ptr<LoudnessState> m_loud;
    std::vector<f32> m_blockBuf;
    struct NoiseImpl;
    std::unique_ptr<NoiseImpl> m_noise;
};

// -----------------------------------------------------------------------------
//  Loudness, per ITU-R BS.1770-4.  Published so the harness can state the mix's
//  level in the unit an audio professional would check, and so the test can
//  calibrate against the standard's own definition rather than a house number.
// -----------------------------------------------------------------------------
class LoudnessMeter {
public:
    explicit LoudnessMeter(u32 sampleRate = kDefaultSampleRate);
    ~LoudnessMeter();
    LoudnessMeter(const LoudnessMeter&) = delete;
    LoudnessMeter& operator=(const LoudnessMeter&) = delete;
    void reset();
    void addBlock(const f32* interleavedStereo, u32 frames, u32 channels = 2);
    // Integrated loudness in LUFS, with both gates (absolute -70 LUFS,
    // relative -10 LU), or -inf-equivalent (-200) if nothing passed the gate.
    f64 integratedLufs() const;
    // Momentary (400 ms) loudness of the most recent window, for a HUD.
    f64 momentaryLufs() const;   // 400 ms window ending at the last block
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// -----------------------------------------------------------------------------
//  WAV.  16-bit PCM by default (with triangular dither, because quantising a
//  quiet ambience without dither is an artifact, and Section 0 forbids those),
//  or 32-bit float for the master.  The reader exists so the tests can prove
//  the writer's output round-trips sample-exactly.
// -----------------------------------------------------------------------------
struct WavData {
    u32 sampleRate = kDefaultSampleRate;
    u16 channels = 2;
    u16 bits = 16;
    bool isFloat = false;
    std::vector<f32> samples;   // interleaved, nominal range [-1, 1]
};

bool writeWav(const std::string& path, const std::vector<f32>& interleaved, u32 channels,
              u32 sampleRate, bool floatFormat = false, bool dither = true);
bool readWav(const std::string& path, WavData& out);

// True-peak estimate per BS.1770 Annex 2: 4x oversampled, reported in dBTP.
f64 truePeakDbtp(const std::vector<f32>& interleaved, u32 channels);

// RMS of everything the mixer has rendered so far, linear (0..1).
inline f64 rmsFromStats(const MixStats& s) {
    return s.frames ? std::sqrt(s.sumSquares / static_cast<f64>(s.frames)) : 0.0;
}

// Which layers a source kind emits, and how loud, so a solver for a platform's
// voice budget (Section 6: 30 FPS on an Adreno 730) has a single table to read.
struct LayerBudget {
    const char* name;
    f32 referenceDbAt10m;
    u32 maxVoices;
};
const LayerBudget* layerBudgets(size_t& count);

}  // namespace mumbai::audio
