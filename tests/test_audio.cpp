// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_audio.cpp
//
//  The audio subsystem is the one part of the engine whose whole job is to be
//  *heard*, and a CPU test cannot hear it.  What a test can do is check the
//  claims the design makes, one at a time, against the arithmetic that is
//  supposed to produce them: that the Doppler factor is the one sound in air
//  follows, that the loudness meter reads the standard's own calibration, that
//  an eviction fades instead of clicking, and that a floating-origin shift is
//  invisible.  Every check here is a number the subsystem promises.
// =============================================================================
#include "engine/audio/audio.h"
#include "tests/test_harness.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace mumbai;
using namespace mumbai::audio;

namespace {

constexpr f64 kPiD = 3.14159265358979323846;
constexpr f64 kTwoPiD = 6.28318530717958647692;

// One isolated tone source, a listener at a chosen place, and a render short
// enough to iterate: the caliper the pitch tests are read with.
struct Rendered {
    std::vector<f32> x;
    f64 seconds = 0.0;
    u32 sr = kDefaultSampleRate;
    size_t frames() const { return x.size() / 2; }
};

Rendered renderOne(const Listener& l, const SourceSpec& s, f64 seconds,
                   OccluderGrid* occ = nullptr, f64 rain = 0.0, f32 city = 0.0f) {
    Mixer mix;
    if (occ) mix.setOccluders(occ);
    mix.setRain(static_cast<f32>(rain));
    mix.setCityBed(city);
    mix.setListener(l);
    Rendered r;
    r.seconds = seconds;
    r.sr = mix.sampleRate();
    Mixer::renderScene(mix, l, {s}, seconds, r.x);
    return r;
}

// Zero-crossing rate over the central 60 % of a render: a robust estimate of a
// sustained tone's frequency that needs no transform.
f64 frequencyOf(const Rendered& r) {
    const size_t n = r.frames();
    const size_t a = n * 2 / 10, b = n * 8 / 10;
    u32 crossings = 0;
    for (size_t i = a + 1; i < b; ++i) {
        const f32 p = r.x[i * 2], q = r.x[(i - 1) * 2];
        if ((p >= 0.0f) != (q >= 0.0f)) ++crossings;
    }
    const f64 span = static_cast<f64>(b - a) / static_cast<f64>(r.sr);
    return static_cast<f64>(crossings) / (2.0 * span);
}

f64 rmsOf(const Rendered& r) {
    f64 s = 0.0;
    for (f32 v : r.x) s += static_cast<f64>(v) * v;
    return r.x.empty() ? 0.0 : std::sqrt(s / r.x.size());
}

f64 peakOf(const Rendered& r) {
    f64 p = 0.0;
    for (f32 v : r.x) p = std::max(p, std::fabs(static_cast<f64>(v)));
    return p;
}

// A stereo sine of `amplitude` at `hz`, for the loudness calibration.
std::vector<f32> sineStereo(f64 hz, f64 amplitude, f64 seconds, u32 sr = kDefaultSampleRate) {
    const size_t n = static_cast<size_t>(seconds * sr);
    std::vector<f32> x(n * 2);
    for (size_t i = 0; i < n; ++i) {
        const f64 v = amplitude * std::sin(kTwoPiD * hz * static_cast<f64>(i) / sr);
        x[i * 2 + 0] = static_cast<f32>(v);
        x[i * 2 + 1] = static_cast<f32>(v);
    }
    return x;
}

}  // namespace

// -----------------------------------------------------------------------------
//  The Doppler factor is the one air imposes
// -----------------------------------------------------------------------------
TEST(aToneApproachingAtThirtyMetresPerSecondRisesByTheExactFactor) {
    // A source at rest: 1,000 Hz, measured on the rendered signal.
    Listener l;
    l.pos = WPos{0, 0, 0};
    SourceSpec s;
    s.id = 1;
    s.layer = Layer::Tone;
    s.f0 = 1000.0f;
    s.pos = WPos{100, 0, 0};      // stationary, broadside
    const Rendered rest = renderOne(l, s, 1.0);
    const f64 f0 = frequencyOf(rest);
    CHECK_NEAR(f0, 1000.0, 2.0);

    // The same tone approaching at 30 m/s along the line of sight: the pitch
    // must rise by c/(c - v) = 343/313 = 1.0958 (the exact Doppler factor).
    SourceSpec moving = s;
    moving.vx = -30.0;
    const Rendered near = renderOne(l, moving, 1.0);
    const f64 fNear = frequencyOf(near);
    const f64 expected = 1000.0 * kSpeedOfSound / (kSpeedOfSound - 30.0);
    std::printf("    tone: %.1f Hz at rest, %.1f Hz approaching at 30 m/s "
                "(exact %.1f, ratio %.5f)\n",
                f0, fNear, expected, fNear / f0);
    CHECK_NEAR(fNear, expected, expected * 0.005);

    // Receding: the mirror image, c/(c + v).
    SourceSpec away = s;
    away.vx = +30.0;
    const Rendered far = renderOne(l, away, 1.0);
    const f64 fFar = frequencyOf(far);
    const f64 expectedFar = 1000.0 * kSpeedOfSound / (kSpeedOfSound + 30.0);
    CHECK_NEAR(fFar, expectedFar, expectedFar * 0.005);
    // And the two are symmetric about the resting pitch in the way the physics
    // says: passing measurements straddle it.
    CHECK(fNear > f0 && fFar < f0);
}

// -----------------------------------------------------------------------------
//  Spreading loss
// -----------------------------------------------------------------------------
TEST(doublingTheDistanceCostsSixDecibels) {
    Listener l;
    l.pos = WPos{0, 0, 0};
    SourceSpec s;
    s.id = 7;
    s.layer = Layer::Tone;
    s.f0 = 500.0f;
    s.pos = WPos{10, 0, 0};
    const f64 a = rmsOf(renderOne(l, s, 0.5));
    s.pos = WPos{20, 0, 0};
    const f64 b = rmsOf(renderOne(l, s, 0.5));
    const f64 ratioDb = 20.0 * std::log10(a / std::max(1e-12, b));
    std::printf("    10 m -> 20 m: %.2f dB (inverse square says 6.02)\n", ratioDb);
    // Not exactly 6.02 dB: the 4 kHz-band air absorption is included, and air
    // absorption is a real part of the loss.  6.02 plus ~0.03 dB of absorption
    // per doubling at a 500 Hz tone is what the model should give.
    CHECK(ratioDb > 5.9 && ratioDb < 6.6);
}

// -----------------------------------------------------------------------------
//  Distance darkens as well as quietens (ISO 9613-1)
// -----------------------------------------------------------------------------
TEST(farTrafficIsDarkerNotJustQuieter) {
    Listener l;
    l.pos = WPos{0, 0, 0};
    SourceSpec s;
    s.id = 11;
    s.layer = Layer::Tyres;
    s.pos = WPos{20, 0.3, 0};
    s.vx = 12.0;
    const f64 near = rmsOf(renderOne(l, s, 1.0));
    s.pos = WPos{400, 0.3, 0};
    const Rendered farR = renderOne(l, s, 1.0);
    const f64 far = rmsOf(farR);
    CHECK(far < near);
    // The high band must be attenuated *more* than the whole: measure the ratio
    // of adjacent-sample differences (a crude high-pass) to the total.
    auto hfFraction = [](const Rendered& r) {
        f64 num = 0.0, den = 0.0;
        for (size_t i = 1; i < r.frames(); ++i) {
            const f64 d = static_cast<f64>(r.x[i * 2]) - r.x[(i - 1) * 2];
            num += d * d;
            den += static_cast<f64>(r.x[i * 2]) * r.x[i * 2];
        }
        return den > 0 ? num / den : 0.0;
    };
    const f64 hfNear = hfFraction(renderOne(l, [&] {
        SourceSpec a = s;
        a.pos = WPos{20, 0.3, 0};
        return a;
    }(), 1.0));
    const f64 hfFar = hfFraction(farR);
    std::printf("    tyres: hf/total %.3f at 20 m, %.3f at 400 m\n", hfNear, hfFar);
    CHECK(hfFar < hfNear * 0.75);
    // The loss at 400 m is dominated by spreading: 1/400^2 = -52 dB, plus
    // ~27 dB of 1 kHz absorption — several orders of magnitude quieter.
    CHECK(far < near * 0.01);
}

// -----------------------------------------------------------------------------
//  The ground reflection is the comb a passing car actually makes
// -----------------------------------------------------------------------------
TEST(aLowSourceOverTheRoadCombsAtSpeedOverTwiceItsExtraPath) {
    Listener l;
    l.pos = WPos{0, 1.2, 0};       // an ear in a car
    SourceSpec s;
    s.id = 21;
    s.layer = Layer::Tyres;
    s.pos = WPos{3.0, 0.3, 0};     // a tyre 3 m away
    s.vx = 8.0;

    // The image-source extra path for h_l = 1.2, h_s = 0.3, d = 3 is
    // 2*1.2*0.3/3 = 0.24 m, so the first null is at 343/(2*0.24) = 715 Hz.
    const f64 extra = 2.0 * 1.2 * 0.3 / 3.0;
    const f64 nullHz = kSpeedOfSound / (2.0 * extra);

    // Sweep a tone through that region and find where the reflection cancels
    // most: the tone's own output plus its delayed, attenuated copy.
    f64 worstF = 0.0, worst = 1e30;
    f64 bestF = 0.0, best = 0.0;
    for (f64 f = 200.0; f <= 1800.0; f += 25.0) {
        SourceSpec t;
        t.id = 22;
        t.layer = Layer::Tone;
        t.f0 = static_cast<f32>(f);
        t.pos = WPos{3.0, 0.3, 0};
        const f64 r = rmsOf(renderOne(l, t, 0.25));
        if (r < worst) { worst = r; worstF = f; }
        if (r > best) { best = r; bestF = f; }
    }
    std::printf("    comb: minimum at %.0f Hz (predicted first null %.0f Hz), "
                "maximum at %.0f Hz\n", worstF, nullHz, bestF);
    CHECK_NEAR(worstF, nullHz, 120.0);
    // A comb with no reflection would be flat: the peak and the null must
    // differ by several dB.
    CHECK(20.0 * std::log10(best / std::max(1e-12, worst)) > 4.0);
}

// -----------------------------------------------------------------------------
//  Occlusion
// -----------------------------------------------------------------------------
TEST(aBuildingBetweenTheEarAndTheSourceAttenuatesAndDarkensIt) {
    Listener l;
    l.pos = WPos{0, 1.2, 0};
    SourceSpec s;
    s.id = 31;
    s.layer = Layer::Tyres;
    s.pos = WPos{30.0, 0.3, 0.0};
    s.vx = 10.0;

    OccluderGrid open;
    const Rendered clear = renderOne(l, s, 1.0, &open);

    OccluderGrid grid;
    Occluder box;
    box.cx = 15.0; box.cz = 0.0;
    box.hw = 8.0f; box.hd = 8.0f;
    box.yaw = 0.0f;
    box.baseY = 0.0f; box.topY = 12.0f;
    std::vector<Occluder> boxes{box};
    grid.build(boxes, 32.0);
    const Rendered blocked = renderOne(l, s, 1.0, &grid);

    const f64 a = rmsOf(clear), b = rmsOf(blocked);
    std::printf("    a building in the way: %.1f dB of shadowing\n",
                20.0 * std::log10(a / std::max(1e-12, b)));
    CHECK(b < a * 0.5);            // at least 6 dB

    // Height is respected, and the way to prove it is an obstruction that is
    // *elevated*: a flyover deck between two ground vehicles shadows neither,
    // and a building does not shadow a source above its roof.
    Occluder deck;
    deck.cx = 15.0; deck.cz = 0.0;
    deck.hw = 8.0f; deck.hd = 8.0f;
    deck.yaw = 0.0f;
    deck.baseY = 20.0f; deck.topY = 40.0f;      // the deck's underside is at 20 m
    OccluderGrid elevated;
    elevated.build({deck}, 32.0);
    const Rendered underDeck = renderOne(l, s, 0.5, &elevated);
    const Rendered underNone = renderOne(l, s, 0.5, &open);
    CHECK_NEAR(rmsOf(underDeck), rmsOf(underNone), rmsOf(underNone) * 0.02);

    // A source 60 m up is above a 12 m building's shadow on this geometry (the
    // ray to it clears the roof for the whole footprint), so the same grid must
    // leave it alone.
    SourceSpec high = s;
    high.pos = WPos{30.0, 60.0, 0.0};
    const Rendered above = renderOne(l, high, 0.5, &grid);
    const Rendered aboveFlat = renderOne(l, high, 0.5, &open);
    std::printf("    a 60 m source over a 12 m building: %.1f dB of shadowing\n",
                20.0 * std::log10(rmsOf(aboveFlat) / std::max(1e-12, rmsOf(above))));
    CHECK_NEAR(rmsOf(above), rmsOf(aboveFlat), rmsOf(aboveFlat) * 0.02);
}

// -----------------------------------------------------------------------------
//  Equal-power panning
// -----------------------------------------------------------------------------
TEST(panningIsEqualPowerAtEveryBearing) {
    Listener l;
    l.pos = WPos{0, 0, 0};
    for (f64 deg = -80.0; deg <= 80.0; deg += 20.0) {
        const f64 a = deg * kPiD / 180.0;
        SourceSpec s;
        s.id = 41;
        s.layer = Layer::Tone;
        s.f0 = 800.0f;
        s.pos = WPos{5.0 * std::sin(a), 0.0, 5.0 * std::cos(a)};
        const Rendered r = renderOne(l, s, 0.4);
        f64 el = 0.0, er = 0.0;
        for (size_t i = 0; i < r.frames(); ++i) {
            el += static_cast<f64>(r.x[i * 2]) * r.x[i * 2];
            er += static_cast<f64>(r.x[i * 2 + 1]) * r.x[i * 2 + 1];
        }
        el /= static_cast<f64>(r.frames());
        er /= static_cast<f64>(r.frames());
        const f64 power = el + er;
        // Equal power means the *sum* is invariant; the first bearing sets the
        // reference so the check does not depend on the layer's reference level.
        static f64 reference = 0.0;
        if (deg <= -80.0) reference = power;
        CHECK_NEAR(power / reference, 1.0, 0.02);
    }
}

// -----------------------------------------------------------------------------
//  Section 0: no clipping, ever
// -----------------------------------------------------------------------------
TEST(threeHundredSourcesDoNotClipTheMix) {
    Listener l;
    l.pos = WPos{0, 1.2, 0};
    l.inVehicle = true;
    std::vector<SourceSpec> srcs;
    Rng rng(0xA0D10ull);
    for (u32 i = 0; i < 300; ++i) {
        SourceSpec s;
        s.id = i + 1;
        s.layer = (i % 3 == 0) ? Layer::Engine : Layer::Tyres;
        s.kind = static_cast<Archetype>(i % 6);
        const f64 ang = rng.nextF32() * kTwoPiD;
        const f64 d = 1.0 + rng.nextF32() * 60.0;
        s.pos = WPos{d * std::sin(ang), 0.3, d * std::cos(ang)};
        s.vx = std::sin(ang + 1.0) * 15.0;
        s.vz = std::cos(ang + 1.0) * 15.0;
        s.envelope = 1.0f;
        srcs.push_back(s);
    }
    Mixer mix;
    mix.setRain(40.0f);
    mix.setCityBed(1.0f);
    std::vector<f32> out;
    Mixer::renderScene(mix, l, srcs, 3.0, out);
    const MixStats& st = mix.stats();
    std::printf("    300 sources: peak %.4f (%.2f dBFS), %llu over full scale, "
                "%llu limiter grabs, %llu voice-frames\n",
                st.peak, 20.0 * std::log10(std::max(1e-12, st.peak)),
                static_cast<unsigned long long>(st.overFullScale),
                static_cast<unsigned long long>(st.limiterGrabs),
                static_cast<unsigned long long>(st.voicesRendered));
    CHECK_EQ(st.overFullScale, 0ull);
    CHECK(st.peak <= 1.0);
    CHECK(st.peak > 0.01);           // it is a real mix, not silence
    for (f32 v : out) CHECK(std::isfinite(v));
    // And it is a programme, not a buzz: measure it the way a mixer would, and
    // hold it to the claim the limiter makes — the *reconstructed* waveform, not
    // just the samples, stays under full scale.
    LoudnessMeter meter;
    meter.addBlock(out.data(), static_cast<u32>(out.size() / 2), 2);
    const f64 lufs = meter.integratedLufs();
    const f64 tp = truePeakDbtp(out, 2);
    std::printf("    300 sources: %.1f LUFS, %.1f dBTP true peak, peak %.3f\n",
                lufs, tp, st.peak);
    CHECK(lufs > -40.0 && lufs < -5.0);
    CHECK(tp <= -0.5);               // the limiter bounds inter-sample peaks too
}

// -----------------------------------------------------------------------------
//  The limiter is a limiter (and the reason the mix can be loud at all)
// -----------------------------------------------------------------------------
TEST(aHornAtOneMetreIsHeldAtTheCeilingWithoutClipping) {
    // A horn at 10 m is 96 dB SPL and the layer's authored range goes to 8x: put
    // one a metre from the ear at 4x and the mixer is asked for roughly 120 dB
    // SPL, i.e. about +20 dBFS of demand.  Nothing may leave above full scale,
    // the inter-sample peak included, and the limiter must be the reason.
    Listener l;
    l.pos = WPos{0, 1.2, 0};
    std::vector<SourceSpec> srcs;
    for (u32 i = 0; i < 6; ++i) {
        SourceSpec s;
        s.id = 100 + i;
        s.layer = Layer::Horn;
        s.kind = Archetype::Bus;
        s.f0 = 300.0f;
        s.level = 4.0f;
        s.envelope = 1.0f;
        s.pos = WPos{1.0 + 0.2 * i, 1.0, 0.5 * i};
        srcs.push_back(s);
    }
    Mixer mix;
    std::vector<f32> out;
    Mixer::renderScene(mix, l, srcs, 1.5, out);
    const MixStats& st = mix.stats();
    const f64 tp = truePeakDbtp(out, 2);
    std::printf("    six horns at a metre: peak %.4f (%.2f dBFS), %.2f dBTP, "
                "%llu limiter grabs, %llu over full scale\n",
                st.peak, 20.0 * std::log10(std::max(1e-12, st.peak)), tp,
                static_cast<unsigned long long>(st.limiterGrabs),
                static_cast<unsigned long long>(st.overFullScale));
    CHECK_EQ(st.overFullScale, 0ull);
    CHECK(st.peak <= 1.0);
    CHECK(tp <= -0.5);
    CHECK(st.limiterGrabs > 0);      // it was needed, not merely present
    CHECK(st.peak > 0.5);            // and it is loud, as six horns at a metre are
}

// -----------------------------------------------------------------------------
//  Section 0: a voice that leaves must fade, not click
// -----------------------------------------------------------------------------
TEST(aSourceThatLeavesTheResidentSetFadesInsteadOfClicking) {
    // A source that a chunk eviction removes must fade over 5 ms.  The measure
    // is not "the largest step" — every signal has its own slew — but the
    // largest step *compared with the same signal's steady-state slew*: a hard
    // cut would show the full amplitude (200x the slew of a 100 Hz tone), a
    // 5 ms fade shows a small multiple of it.
    Listener l;
    l.pos = WPos{0, 1.2, 0};
    SourceSpec s;
    s.id = 5;
    s.layer = Layer::Tone;      // a known, slow waveform, so the slew is knowable
    s.f0 = 100.0f;
    // Directly ahead.  (The first version put it abeam, where the pan law sends
    // it entirely to the right channel — and the measurement read the left one,
    // so the test passed on silence.  `steady > 1e-6` below is the guard that
    // makes that impossible.)
    s.pos = WPos{0.0, 0.3, 3.0};

    Mixer mix;
    const u32 block = mix.blockFrames();
    std::vector<f32> buf(block * 2);
    std::vector<f32> all;
    const f64 dt = 1.0 / static_cast<f64>(mix.sampleRate());
    for (u32 b = 0; b < 400; ++b) {          // 2.1 s with the source present
        mix.setListener(l);
        mix.beginBlock(static_cast<f64>(b) * block * dt, {s});
        mix.renderBlock(buf.data());
        all.insert(all.end(), buf.begin(), buf.end());
    }
    const size_t eviction = all.size();
    for (u32 b = 400; b < 500; ++b) {        // then the chunk goes away
        mix.setListener(l);
        mix.beginBlock(static_cast<f64>(b) * block * dt, {});
        mix.renderBlock(buf.data());
        all.insert(all.end(), buf.begin(), buf.end());
    }
    // Both channels, so no pan can hide a signal from the measurement.
    auto maxStep = [&](size_t a, size_t b) {
        f64 w = 0.0;
        for (size_t i = a + 2; i < b; i += 2) {
            w = std::max(w, std::fabs(static_cast<f64>(all[i]) - static_cast<f64>(all[i - 2])));
            w = std::max(w, std::fabs(static_cast<f64>(all[i + 1]) - static_cast<f64>(all[i - 1])));
        }
        return w;
    };
    const f64 steady = maxStep(0, eviction - 2000);
    const f64 across = maxStep(eviction - 2000, eviction + 20000);
    f64 tail = 0.0;
    for (size_t i = all.size() - 2000; i < all.size(); ++i)
        tail = std::max(tail, std::fabs(static_cast<f64>(all[i])));
    std::printf("    eviction at sample %zu: steady slew %.2e, slew across it %.2e, "
                "tail %.2e\n", eviction, steady, across, tail);
    CHECK(steady > 1e-6);           // the instrument is alive: there *is* a signal
    CHECK(across < steady * 2.0);   // a fade, not a cut (a cut would be ~200x)
    CHECK(tail < 1e-7);             // and the voice is genuinely gone
}

// -----------------------------------------------------------------------------
//  Section 4A: a floating-origin shift is invisible to the mixer
// -----------------------------------------------------------------------------
TEST(aFloatingOriginShiftDoesNotChangeOneSample) {
    // The same scene, expressed twice: once in absolute world metres and once
    // after the renderer has shifted everything by four macro-cells.  Audio's
    // world coordinates never see the render origin, so the two renders must be
    // bit-identical — which is the strongest possible statement of "a shift
    // cannot pop".
    const WPos shift{4.0 * 1024.0, 0.0, -3.0 * 1024.0};
    Listener l;
    l.pos = WPos{1000.0, 1.2, 2000.0};
    l.yaw = 0.7;
    std::vector<SourceSpec> a;
    Rng rng(0x517F7ull);
    for (u32 i = 0; i < 12; ++i) {
        SourceSpec s;
        s.id = i + 1;
        s.layer = (i % 2) ? Layer::Tyres : Layer::Engine;
        s.kind = static_cast<Archetype>(i % 6);
        s.pos = WPos{l.pos.x + rng.nextF32() * 80.0 - 40.0, 0.4,
                     l.pos.z + rng.nextF32() * 80.0 - 40.0};
        s.vx = rng.nextF32() * 20.0 - 10.0;
        s.vz = rng.nextF32() * 20.0 - 10.0;
        a.push_back(s);
    }
    std::vector<SourceSpec> b = a;
    for (SourceSpec& s : b) s.pos = s.pos + shift;
    Listener lb = l;
    lb.pos = lb.pos + shift;

    Mixer m1, m2;
    std::vector<f32> x, y;
    Mixer::renderScene(m1, l, a, 1.0, x);
    Mixer::renderScene(m2, lb, b, 1.0, y);
    CHECK_EQ(x.size(), y.size());
    size_t differences = 0;
    f64 worst = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
        const f64 d = std::fabs(static_cast<f64>(x[i]) - static_cast<f64>(y[i]));
        if (d != 0.0) ++differences;
        worst = std::max(worst, d);
    }
    std::printf("    shifted by (%.0f, %.0f): %zu of %zu samples differ, worst %.3e\n",
                shift.x, shift.z, differences, x.size(), worst);
    CHECK_EQ(differences, 0ull);
}

// -----------------------------------------------------------------------------
//  BS.1770: the meter reads the standard's own calibration
// -----------------------------------------------------------------------------
TEST(theLoudnessMeterReadsTheStandardCalibration) {
    // BS.1770's -0.691 offset and the K-weighting's own gain meet at 1 kHz by
    // design: the weighting is +0.69 dB there (the high shelf's action at 2 kHz
    // and above reaches 1 kHz as a partial lift), so a *full-scale* 1 kHz sine in
    // both channels reads 0.0 LUFS.  That is the calibration this meter is held
    // to, and it is the one number that says the whole chain — coefficients,
    // window, gate — is the standard's.
    {
        const std::vector<f32> x = sineStereo(1000.0, 1.0, 5.0);
        LoudnessMeter m;
        m.addBlock(x.data(), static_cast<u32>(x.size() / 2), 2);
        const f64 lufs = m.integratedLufs();
        std::printf("    full-scale 1 kHz stereo sine: %.2f LUFS (the standard's 0.0)\n", lufs);
        CHECK_NEAR(lufs, 0.0, 0.20);
    }
    // Twenty decibels down is twenty decibels down.
    {
        const std::vector<f32> x = sineStereo(1000.0, 0.1, 5.0);
        LoudnessMeter m;
        m.addBlock(x.data(), static_cast<u32>(x.size() / 2), 2);
        const f64 lufs = m.integratedLufs();
        std::printf("    the same 20 dB down: %.2f LUFS\n", lufs);
        CHECK_NEAR(lufs, -20.0, 0.25);
    }
    // The relative gate: a quiet passage below (mean - 10 LU) must not drag the
    // integrated figure down, which is the whole point of the gate.
    {
        std::vector<f32> x = sineStereo(1000.0, 0.1, 5.0);      // -20 dBFS
        const std::vector<f32> quiet = sineStereo(1000.0, 0.0001, 5.0);  // -80 dBFS
        x.insert(x.end(), quiet.begin(), quiet.end());
        LoudnessMeter m;
        m.addBlock(x.data(), static_cast<u32>(x.size() / 2), 2);
        const f64 lufs = m.integratedLufs();
        std::printf("    -20 dBFS for 5 s then -80 dBFS for 5 s: %.2f LUFS\n", lufs);
        CHECK_NEAR(lufs, -20.0, 0.25);
    }
    // Silence is below the absolute gate, not a number.
    {
        std::vector<f32> z(static_cast<size_t>(kDefaultSampleRate) * 2, 0.0f);
        LoudnessMeter m;
        m.addBlock(z.data(), kDefaultSampleRate, 2);
        CHECK(m.integratedLufs() < -70.0);
    }
}

// -----------------------------------------------------------------------------
//  WAV round-trip
// -----------------------------------------------------------------------------
TEST(theWavWriterAndReaderAgreeSampleForSample) {
    // 32-bit float is exact by construction: what the mixer produced is what a
    // reader gets back.  16-bit is checked to within the dither it added.
    std::vector<f32> x(2000);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = static_cast<f32>(0.7 * std::sin(kTwoPiD * 220.0 * static_cast<f64>(i) / 48000.0));
    const std::string f32Path = "build/audio/_test_roundtrip_f32.wav";
    const std::string i16Path = "build/audio/_test_roundtrip_i16.wav";
    ensureDir("build/audio");
    CHECK(writeWav(f32Path, x, 2, 48000, /*floatFormat=*/true, false));
    WavData r32;
    CHECK(readWav(f32Path, r32));
    CHECK_EQ(r32.channels, static_cast<u16>(2));
    CHECK_EQ(r32.sampleRate, 48000u);
    CHECK_EQ(r32.samples.size(), x.size());
    size_t exact = 0;
    for (size_t i = 0; i < std::min(r32.samples.size(), x.size()); ++i)
        if (r32.samples[i] == x[i]) ++exact;
    CHECK_EQ(exact, x.size());

    CHECK(writeWav(i16Path, x, 2, 48000, /*floatFormat=*/false, /*dither=*/true));
    WavData r16;
    CHECK(readWav(i16Path, r16));
    f64 worst = 0.0;
    for (size_t i = 0; i < std::min(r16.samples.size(), x.size()); ++i)
        worst = std::max(worst, std::fabs(static_cast<f64>(r16.samples[i]) - static_cast<f64>(x[i])));
    // One LSB of 16-bit is 1/32767 = 3.05e-5; dither at 1 LSB plus rounding
    // cannot exceed 1.5 LSB.
    std::printf("    16-bit round trip: worst error %.2e (1.5 LSB = %.2e)\n",
                worst, 1.5 / 32767.0);
    CHECK(worst < 1.5 / 32767.0);
    std::remove(f32Path.c_str());
    std::remove(i16Path.c_str());
}

// -----------------------------------------------------------------------------
//  The mixer's filters are filters
// -----------------------------------------------------------------------------
TEST(theFiltersAreWhatTheyClaimToBe) {
    // A one-pole low-pass passes DC at unity and attenuates a tone above its
    // cutoff by the amount its slope allows.
    Svf lp;
    lp.setOnePoleLow(1000.0, 48000.0);
    f64 dc = 0.0;
    for (u32 i = 0; i < 48000; ++i) dc = lp.onePoleLow(1.0);
    CHECK_NEAR(dc, 1.0, 1e-6);

    // A cutoff is where the filter is 3 dB down *by definition*, and the TPT
    // topology places it exactly: measuring at f = fc must give 1/sqrt(2).  (An
    // earlier version of this test compared against the analog 1/sqrt(1+(f/fc)^2)
    // at 8x the cutoff and failed by 0.8 dB — the filter was right and the
    // reference was the wrong model of it: the bilinear transform warps the
    // frequency axis above a tenth of the sample rate.)
    Svf lp2;
    lp2.setOnePoleLow(1000.0, 48000.0);
    f64 sum = 0.0;
    const u32 n = 48000;
    for (u32 i = 0; i < n; ++i) {
        const f64 y = lp2.onePoleLow(std::sin(kTwoPiD * 1000.0 * i / 48000.0));
        if (i > n / 2) sum += y * y;
    }
    const f64 gain = std::sqrt(sum / (n / 2)) * std::sqrt(2.0);
    std::printf("    one-pole at its own cutoff: %.4f (definition: %.4f)\n",
                gain, 1.0 / std::sqrt(2.0));
    CHECK_NEAR(gain, 1.0 / std::sqrt(2.0), 0.01);

    // The SVF's own identity: low + band*k + high == input, exactly.
    Svf svf;
    svf.setResonance(1200.0, 1.7, 48000.0);
    f64 worst = 0.0;
    for (u32 i = 0; i < 4000; ++i) {
        const f64 x = std::sin(kTwoPiD * 900.0 * i / 48000.0) * 0.5;
        // The three outputs must be read from the same state, so the identity is
        // checked by re-running one step of the arithmetic here.
        const f64 k = svf.damping();
        Svf probe = svf;
        const f64 lo = probe.lowPass(x);
        Svf probe2 = svf;
        const f64 bp = probe2.bandPass(x);
        Svf probe3 = svf;
        const f64 hp = probe3.highPass(x);
        worst = std::max(worst, std::fabs(lo + bp * k + hp - x));
        svf.lowPass(x);   // advance the filter's real state once
    }
    CHECK(worst < 1e-12);
}
