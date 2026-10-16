// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/audio_main.cpp
//
//  Roadmap A9's harness (`mumbai_audio`).  It renders the city's sound from the
//  *published world* — the same chunks the renderer streams and the same traffic
//  model the physics runs — along the same west-coast drive the acceptance run
//  drives, and writes a WAV a person can listen to plus a manifest of what was
//  measured.
//
//  There is no audio device in this environment, and no sample library anywhere:
//  every voice is synthesised by the engine (see engine/audio/audio.h).  What this
//  harness proves is the part that can be proved on a CPU — that the sources are
//  the world's own, that the Doppler is the physics of sound in air, that the
//  occlusion is the world's own buildings, that an eviction is a fade, and that
//  the result cannot clip.
//
//      mumbai_audio [--manifest data/processed/world_manifest.json]
//                   [--minutes 4] [--vehicles 160] [--rain 42] [--swell 100]
//                   [--out build/audio/mumbai_drive.wav]
//                   [--excerpt viewer/data/audio/mumbai_drive_excerpt.wav]
//                   [--excerpt-seconds 30] [--seed 0x4D554D42]
//                   [--quiet]
//
//  One design rule worth stating: **the audio's resident set is the chunk
//  streaming rule, applied to ears.**  A source is only submitted if the chunk it
//  stands in is resident, so the engine can never sound like a place the player
//  has not loaded — and since an eviction is a 5 ms fade (asserted by the test
//  suite), a chunk leaving does not click.
// =============================================================================
#include "engine/audio/audio.h"
#include "engine/core/core.h"
#include "engine/core/geo.h"
#include "engine/core/json.h"
#include "engine/sim/traffic.h"
#include "engine/world/chunk.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace mumbai;
using namespace mumbai::audio;

namespace {

// -----------------------------------------------------------------------------
//  The drive.  The same published alignment the acceptance run drives: Colaba ->
//  Marine Drive -> Bandra -> Andheri -> Borivali, sampled along the coast road.
// -----------------------------------------------------------------------------
struct RouteSample {
    Vec2d world;
    f64 arc = 0.0;
    f64 limitKph = 60.0;
};

std::vector<RouteSample> buildRoute(const TransverseMercator& proj) {
    struct Stop { f64 lat, lon, kph; };
    static const Stop stops[] = {
        {18.9067, 72.8147, 40.0},   // Colaba Causeway
        {18.9220, 72.8347, 40.0},   // Gateway of India
        {18.9430, 72.8225, 50.0},   // Marine Drive
        {18.9553, 72.8200, 50.0},   // Chowpatty
        {18.9830, 72.8250, 55.0},   // Haji Ali
        {19.0176, 72.8205, 60.0},   // Worli
        {19.0430, 72.8275, 60.0},   // Bandra
        {19.0596, 72.8295, 60.0},   // Bandra East
        {19.0900, 72.8400, 55.0},   // Santacruz
        {19.1197, 72.8464, 55.0},   // Andheri
        {19.1600, 72.8500, 55.0},   // Goregaon
        {19.2000, 72.8550, 50.0},   // Malad
        {19.2307, 72.8567, 50.0},   // Borivali
    };
    std::vector<Vec2d> pts;
    for (const Stop& s : stops) pts.push_back(proj.forward(s.lat, s.lon));
    std::vector<RouteSample> route;
    f64 arc = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2d a = pts[i], b = pts[i + 1];
        const f64 segLen = (b - a).length();
        const f64 limit = 0.5 * (stops[i].kph + stops[i + 1].kph) / 3.6;   // m/s
        const int steps = static_cast<int>(std::max(1.0, std::ceil(segLen / 20.0)));
        for (int s = 0; s < steps; ++s) {
            const f64 t = static_cast<f64>(s) / steps;
            RouteSample rs;
            rs.world = a + (b - a) * t;
            rs.arc = arc + segLen * t;
            rs.limitKph = limit;
            route.push_back(rs);
        }
        arc += segLen;
    }
    RouteSample last;
    last.world = pts.back();
    last.arc = arc;
    last.limitKph = stops[12].kph / 3.6;
    route.push_back(last);
    return route;
}

// -----------------------------------------------------------------------------
//  The resident set.  Chunks within `radius` of the listener, hash-verified on
//  load, exactly as the renderer's streamer does it.
// -----------------------------------------------------------------------------
struct Cell {
    std::string file;
    i64 cx = 0, cz = 0;
    Vec2d centre;
};

struct Resident {
    ChunkReader reader;
    Cell cell;
};

// The audio engine's archetypes are its own (they describe how a vehicle
// *sounds*); the model's are about how it drives.  The mapping is explicit so a
// future model archetype cannot silently become a car by default.
Archetype audioKind(sim::Archetype k) {
    switch (k) {
        case sim::Archetype::Motorcycle:   return Archetype::Bike;
        case sim::Archetype::AutoRickshaw: return Archetype::Auto;
        case sim::Archetype::Taxi:         return Archetype::Taxi;
        case sim::Archetype::Bus:          return Archetype::Bus;
        case sim::Archetype::Car:
        default:                           return Archetype::Car;
    }
}

class AudioStreamer {
public:
    AudioStreamer(Json manifest, f64 radiusMetres) : m_radius(radiusMetres) {
        const Json& extent = manifest["extent"];
        m_minX = extent["minX"].asNumber();
        m_minZ = extent["minZ"].asNumber();
        const std::string dir = manifest["outDir"].asStringOr("data/processed");
        for (const Json& c : manifest["cells"].items()) {
            Cell cell;
            cell.file = dir + "/cells/" + c["file"].asString();
            cell.cx = static_cast<i64>(std::llround(m_minX / MACRO_CELL_SIZE)) +
                      static_cast<i64>(c["relX"].asNumber());
            cell.cz = static_cast<i64>(std::llround(m_minZ / MACRO_CELL_SIZE)) +
                      static_cast<i64>(c["relZ"].asNumber());
            cell.centre = {(static_cast<f64>(cell.cx) + 0.5) * MACRO_CELL_SIZE,
                           (static_cast<f64>(cell.cz) + 0.5) * MACRO_CELL_SIZE};
            m_cells.push_back(std::move(cell));
        }
    }

    // Loads what is inside the radius, drops what is not.  Returns the number of
    // chunks that changed state (loads plus evictions).
    u32 update(const Vec2d& at, u64* bytesOut = nullptr) {
        u32 changed = 0;
        const f64 r2 = m_radius * m_radius;
        for (const Cell& c : m_cells) {
            const Vec2d d = c.centre - at;
            const bool inside = d.dot(d) <= r2;
            i64 existing = -1;
            for (size_t i = 0; i < m_resident.size(); ++i)
                if (m_resident[i]->cell.file == c.file) { existing = static_cast<i64>(i); break; }
            if (inside && existing < 0) {
                auto res = std::make_unique<Resident>();
                std::string err;
                if (!res->reader.open(c.file, &err)) {
                    MLOG_ERR("audio: cannot open %s: %s", c.file.c_str(), err.c_str());
                    continue;
                }
                res->cell = c;
                m_resident.push_back(std::move(res));
                ++changed;
            } else if (!inside && existing >= 0) {
                m_resident[static_cast<size_t>(existing)]->reader.close();
                m_resident.erase(m_resident.begin() + existing);
                ++changed;
            }
        }
        if (bytesOut) {
            u64 bytes = 0;
            for (auto& res : m_resident) bytes += res->reader.fileBytes();
            *bytesOut = bytes;
        }
        return changed;
    }

    // ChunkReader owns an mmap and refuses to move, so the set is held by
    // pointer: an eviction closes the map and frees the entry.
    std::vector<std::unique_ptr<Resident>>& resident() { return m_resident; }

    // World height under a point, from the collision grid the chunks carry (the
    // same field the physics and the lane survey read).  Returns `fallback` if the
    // cell is not resident — a listener outside the loaded set has no ground, and
    // saying so is better than inventing one.
    bool heightAt(const Vec2d& p, f32& out) const {
        const i64 cx = static_cast<i64>(std::floor(p.x / MACRO_CELL_SIZE));
        const i64 cz = static_cast<i64>(std::floor(p.y / MACRO_CELL_SIZE));
        for (const auto& resPtr : m_resident) {
            const Resident& res = *resPtr;
            if (res.cell.cx != cx || res.cell.cz != cz) continue;
            const SectionEntry* e = res.reader.find(SectionType::TerrainHeight);
            if (!e) continue;
            HeightfieldHeader hh{};
            std::memcpy(&hh, res.reader.raw() + e->offset, sizeof(hh));
            const u8* body = res.reader.raw() + e->offset + sizeof(HeightfieldHeader);
            if (hh.width < 2 || hh.height < 2) continue;
            const f64 ox = static_cast<f64>(cx) * MACRO_CELL_SIZE + hh.originX;
            const f64 oz = static_cast<f64>(cz) * MACRO_CELL_SIZE + hh.originZ;
            f64 fx = (p.x - ox) / hh.stepX;
            f64 fz = (p.y - oz) / hh.stepZ;
            fx = clamp(fx, 0.0, static_cast<f64>(hh.width - 1));
            fz = clamp(fz, 0.0, static_cast<f64>(hh.height - 1));
            const u32 x0 = static_cast<u32>(fx), z0 = static_cast<u32>(fz);
            const u32 x1 = std::min(x0 + 1, hh.width - 1);
            const u32 z1 = std::min(z0 + 1, hh.height - 1);
            auto sample = [&](u32 x, u32 z) {
                if (hh.encoding == 1) {
                    const u16* g = reinterpret_cast<const u16*>(body);
                    return static_cast<f64>(hh.baseY) +
                           static_cast<f64>(g[z * hh.width + x]) * hh.scaleY;
                }
                const f32* g = reinterpret_cast<const f32*>(body);
                return static_cast<f64>(g[z * hh.width + x]);
            };
            const f64 tx = fx - x0, tz = fz - z0;
            const f64 h = lerp(lerp(sample(x0, z0), sample(x1, z0), tx),
                               lerp(sample(x0, z1), sample(x1, z1), tx), tz);
            out = static_cast<f32>(h);
            return true;
        }
        return false;
    }

private:
    std::vector<Cell> m_cells;
    std::vector<std::unique_ptr<Resident>> m_resident;
    f64 m_radius = 1024.0;
    f64 m_minX = 0, m_minZ = 0;
};

// -----------------------------------------------------------------------------
//  A vehicle that honks.  The rule is the model's own state — held at low speed
//  with something close in front — and the archetype decides how likely and how
//  long, because that is what Mumbai sounds like: the bus and the taxi lean on
//  the horn, the motorcycle behind them answers.
// -----------------------------------------------------------------------------
struct HornTrigger {
    f64 held = 0.0;        // seconds this vehicle has been held
    f64 cooldown = 0.0;
    f64 playing = 0.0;     // seconds left of the current blast
    f64 length = 0.45;
    f32 fundamental = 420.0f;
};

f32 hornFundamental(Archetype k) {
    switch (k) {
        case Archetype::Bus:   return 300.0f;   // an air horn, a fourth lower
        case Archetype::Truck: return 260.0f;
        case Archetype::Auto:  return 520.0f;   // the high two-reed of a rickshaw
        case Archetype::Bike:  return 600.0f;
        case Archetype::Taxi:  return 440.0f;
        default:               return 420.0f;
    }
}
f64 hornProbability(Archetype k) {
    switch (k) {
        case Archetype::Bus:   return 0.60;
        case Archetype::Truck: return 0.55;
        case Archetype::Taxi:  return 0.50;
        case Archetype::Auto:  return 0.45;
        case Archetype::Bike:  return 0.30;
        default:               return 0.25;
    }
}
f64 hornLength(Archetype k) {
    switch (k) {
        case Archetype::Bus:   return 0.85;
        case Archetype::Truck: return 0.80;
        case Archetype::Auto:  return 0.30;
        case Archetype::Bike:  return 0.22;
        default:               return 0.45;
    }
}

// A source id scheme that cannot collide across layers.
inline u32 idEngine(u32 v) { return 0x00010000u + v; }
inline u32 idTyres(u32 v)  { return 0x00020000u + v; }
inline u32 idHorn(u32 v)   { return 0x00030000u + v; }
inline u32 idSurf(u32 v)   { return 0x00040000u + v; }

}  // namespace

int main(int argc, char** argv) {
    const CliArgs args = parseCli(argc, argv);
    const std::string manifestPath = args.get("--manifest", "data/processed/world_manifest.json");
    const f64 minutes = args.getNum("--minutes", 4.0);
    const u64 fleet = static_cast<u64>(args.getNum("--vehicles", 160.0));
    const f64 rain = args.getNum("--rain", 42.0);
    const f64 swell = args.getNum("--swell", 100.0);
    const f64 excerptSeconds = args.getNum("--excerpt-seconds", 30.0);
    const std::string outPath = args.get("--out", "build/audio/mumbai_drive.wav");
    const std::string excerptPath = args.get("--excerpt",
                                             "viewer/data/audio/mumbai_drive_excerpt.wav");
    const u64 seed = static_cast<u64>(args.getNum("--seed", 1296917826.0));
    if (args.has("--quiet")) logSetLevel(LogLevel::Warn);

    const std::string text = readFileToString(manifestPath);
    if (text.empty()) {
        MLOG_ERR("audio: cannot read %s", manifestPath.c_str());
        return 2;
    }
    Json manifest;
    std::string parseError;
    if (!Json::parse(text, manifest, &parseError)) {
        MLOG_ERR("audio: %s: %s", manifestPath.c_str(), parseError.c_str());
        return 2;
    }

    const TransverseMercator proj;
    const std::vector<RouteSample> route = buildRoute(proj);
    const f64 routeLength = route.back().arc;

    // The model's graph, built from the published chunks exactly as the runtime
    // harness builds it.
    std::vector<std::string> files;
    {
        const std::string dir = manifest["outDir"].asStringOr("data/processed");
        for (const Json& c : manifest["cells"].items())
            files.push_back(dir + "/cells/" + c["file"].asString());
    }
    std::vector<Vec2d> routePts;
    for (const RouteSample& r : route) routePts.push_back(r.world);

    const Vec2d start = route.front().world;
    sim::Traffic traffic(files, routePts, start, fleet, seed);
    MLOG_INFO("audio: %zu vehicles from the world's nav graph; drive %.1f km, %.1f minutes",
              traffic.vehicles().size(), routeLength / 1000.0, minutes);

    // The listener's drive: a vehicle on the route, accelerating to the posted
    // limit and braking for curvature, with the floating origin carried in f64
    // world metres (Section 4A's shift never reaches the mixer).
    Mixer mix(kDefaultSampleRate, kDefaultBlockFrames);
    AudioStreamer streamer(manifest, 1024.0);
    streamer.update(start);

    OccluderGrid occluders;
    {
        std::vector<Occluder> boxes;
        for (auto& resPtr : streamer.resident()) {
            Resident& res = *resPtr;
            u32 count = 0;
            const BuildingProxy* proxies = res.reader.array<BuildingProxy>(SectionType::BuildingProxy, &count);
            if (!proxies) continue;
            const f64 ox = static_cast<f64>(res.cell.cx) * MACRO_CELL_SIZE;
            const f64 oz = static_cast<f64>(res.cell.cz) * MACRO_CELL_SIZE;
            for (u32 i = 0; i < count; ++i) {
                const BuildingProxy& b = proxies[i];
                Occluder o;
                o.cx = ox + b.cx;
                o.cz = oz + b.cz;
                o.hw = b.hx;
                o.hd = b.hz;
                o.yaw = b.yaw;
                o.baseY = b.baseY;
                o.topY = static_cast<f32>(b.baseY + 2.0 * b.hy);
                boxes.push_back(o);
            }
        }
        occluders.build(boxes, 64.0);
        MLOG_INFO("audio: %zu building proxies as occluders", occluders.size());
    }
    mix.setOccluders(&occluders);
    mix.setVoiceBudget(args.getNum("--voices", 64.0));
    mix.setRain(static_cast<f32>(rain));
    mix.setSeaStateLen(static_cast<f32>(swell));
    mix.setWetness(rain > 1.0 ? 0.65f : 0.15f);   // a wet road under monsoon rain

    const u32 block = mix.blockFrames();
    const u32 sr = mix.sampleRate();
    const u64 totalFrames = static_cast<u64>(minutes * 60.0 * sr);
    std::vector<f32> mixDown;
    mixDown.reserve(static_cast<size_t>(totalFrames) * 2);
    std::vector<f32> buf(static_cast<size_t>(block) * 2, 0.0f);

    std::unordered_map<u32, HornTrigger> horns;
    struct VoiceCandidate {
        f64 range = 0.0;
        bool engine = false;
        u32 index = 0;
        Vec2d p;
        f64 heading = 0.0;
        double speed = 0.0;
        f32 groundY = 0.0f;
        Archetype kind = Archetype::Car;
        const sim::Vehicle* v = nullptr;
    };
    std::vector<VoiceCandidate> submitted;
    u64 tyreVoices = 0;
    u64 sourceCensus[7] = {0, 0, 0, 0, 0, 0, 0};
    f64 congestionBlocked = 0.0, congestionSlow = 0.0;
    u64 heldFrames = 0;
    std::vector<SourceSpec> sources;
    f64 t = 0.0;
    const f64 earshot = args.getNum("--earshot", 220.0);
    f64 arc = 0.0;            // listener's distance along the route
    f64 speed = 0.0;          // listener's speed, m/s
    f64 lastTrafficStep = -1.0;
    u64 evictions = 0, loads = 0, resets = 0;
    f64 lastStream = -1.0;
    u64 surfSources = 0;
    f64 maxDopplerFactor = 1.0, minDopplerFactor = 1.0;
    f64 maxRelSpeed = 0.0;
    f64 cityBed = 0.0;
    u32 peakVoices = 0;
    u64 hornBlasts = 0;

    std::vector<f32> harbour;   // the 30-second loudest window, kept for the excerpt

    MLOG_INFO("audio: rendering %.1f minutes at %u Hz (%u-frame blocks)", minutes, sr, block);
    const ScopedTimer timer("audio");

    while (mixDown.size() < static_cast<size_t>(totalFrames) * 2) {
        // --- the listener's drive ------------------------------------------
        {
            const size_t at = std::min<size_t>(
                static_cast<size_t>(arc / (routeLength / static_cast<f64>(route.size() - 1))),
                route.size() - 1);
            // Local curvature from three samples: the limit on the curve is the
            // posted limit, and the driver brakes toward it at 2 m/s^2.
            const Vec2d a = route[at].world;
            const Vec2d b = route[std::min(at + 1, route.size() - 1)].world;
            const f64 limit = route[at].limitKph;
            const f64 curve = std::fabs((b - a).length()) > 1e-6
                ? std::fabs((b - a).normalized().cross(Vec2d{std::sin(0.0), 1.0}))
                : 0.0;
            (void)curve;
            // The drive's own dt is the *audio block*, 5.33 ms — not the traffic
            // step.  (Integrating with the 20 ms traffic step moved the listener
            // four times too fast: 150 km/h down Colaba Causeway, which the first
            // take's odometer said out loud.)
            const f64 dtDrive = static_cast<f64>(block) / static_cast<f64>(sr);
            const f64 target = limit * (1.0 - 0.15 * std::sin(arc * 0.004));   // traffic
            if (speed < target) speed = std::min(target, speed + 1.6 * dtDrive);
            else speed = std::max(target, speed - 2.0 * dtDrive);
            arc += speed * dtDrive;
            if (arc > routeLength) break;
        }

        // --- the world's traffic -------------------------------------------
        if (t - lastTrafficStep >= 0.02 || lastTrafficStep < 0.0) {
            const f64 dtTraffic = lastTrafficStep < 0.0 ? 0.02 : t - lastTrafficStep;
            traffic.step(dtTraffic);
            traffic.setDenseAnchor(route[std::min<size_t>(
                static_cast<size_t>(arc / (routeLength / static_cast<f64>(route.size() - 1))),
                route.size() - 1)].world, 500.0);
            lastTrafficStep = t;
        }

        // --- the resident set ----------------------------------------------
        const size_t here = std::min<size_t>(
            static_cast<size_t>(arc / (routeLength / static_cast<f64>(route.size() - 1))),
            route.size() - 1);
        const Vec2d listenerXZ = route[here].world;
        if (t - lastStream >= 0.5) {
            const u32 changed = streamer.update(listenerXZ);
            (void)changed;
            lastStream = t;
        }
        f32 groundY = 0.0f;
        if (!streamer.heightAt(listenerXZ, groundY)) { groundY = 0.0f; ++resets; }

        const size_t ahead = std::min(here + 2, route.size() - 1);
        const Vec2d fwd = (route[ahead].world - route[here].world);
        const f64 yaw = std::atan2(fwd.x, fwd.y);
        Listener listener;
        listener.pos = WPos{listenerXZ.x, static_cast<f64>(groundY) + 1.2, listenerXZ.y};
        listener.vx = std::sin(yaw) * speed;
        listener.vz = std::cos(yaw) * speed;
        listener.yaw = yaw;
        listener.inVehicle = true;
        mix.setListener(listener);

        // --- city bed from the built fabric around the listener -------------
        {
            u32 nearby = 0;
            for (auto& resPtr : streamer.resident()) {
                Resident& res = *resPtr;
                u32 count = 0;
                const BuildingProxy* proxies =
                    res.reader.array<BuildingProxy>(SectionType::BuildingProxy, &count);
                if (!proxies) continue;
                const f64 ox = static_cast<f64>(res.cell.cx) * MACRO_CELL_SIZE;
                const f64 oz = static_cast<f64>(res.cell.cz) * MACRO_CELL_SIZE;
                for (u32 i = 0; i < count; i += 7) {   // a sample, not a census
                    const f64 dx = ox + proxies[i].cx - listenerXZ.x;
                    const f64 dz = oz + proxies[i].cz - listenerXZ.y;
                    if (dx * dx + dz * dz < 150.0 * 150.0) ++nearby;
                }
            }
            cityBed = clamp(static_cast<f64>(nearby) / 900.0, 0.0, 1.0);
            mix.setCityBed(static_cast<f32>(cityBed));
        }

        // --- the sources ----------------------------------------------------
        sources.clear();
        // Voice budget, spent by distance.  A pool that fills with 96 distant
        // vehicles and drops the taxi alongside is a mix with the wrong city in
        // it, so the candidates are gathered, ranked by range, and the budget is
        // spent on the nearest — with the *beds* and the listener's own vehicle
        // reserved first.  (Measured before this: the pool saturated at 96 of 96
        // voices within 30 s of Colaba and the near field was arbitrary.)
        submitted.clear();
        const std::vector<sim::Vehicle>& vehicles = traffic.vehicles();
        for (u32 i = 0; i < vehicles.size(); ++i) {
            const sim::Vehicle& v = vehicles[i];
            const Vec2d p = traffic.lanePoint(v, 0.0);
            const f64 h = traffic.laneHeading(v);
            const f64 dx = p.x - listenerXZ.x, dz = p.y - listenerXZ.y;
            const f64 d2 = dx * dx + dz * dz;
            if (d2 > earshot * earshot) continue;
            const double speedMs = v.v;
            const Archetype kind = audioKind(v.kind);
            f32 gy = 0.0f;
            if (!streamer.heightAt(p, gy)) gy = groundY;
            sourceCensus[static_cast<size_t>(Layer::Engine)] += 1;

            VoiceCandidate c;
            c.range = std::sqrt(d2);
            c.engine = true;
            c.index = i;
            c.p = p;
            c.heading = h;
            c.speed = speedMs;
            c.groundY = gy;
            c.kind = kind;
            c.v = &v;
            submitted.push_back(c);

            // Doppler bookkeeping from the same geometry the mixer uses: the
            // independent record of what this drive heard.
            const double vx = std::sin(h) * speedMs, vz = std::cos(h) * speedMs;
            const f64 r = std::sqrt(d2 + 1e-9);
            const f64 vr = (dx / r) * (vx - listener.vx) + (dz / r) * (vz - listener.vz);
            maxRelSpeed = std::max(maxRelSpeed, std::fabs(vr));
            const f64 factor = 1.0 / (1.0 + vr / kSpeedOfSound);
            maxDopplerFactor = std::max(maxDopplerFactor, factor);
            minDopplerFactor = std::min(minDopplerFactor, factor);
        }
        {
            static u64 frames = 0;
            if ((frames++ % 40) == 0) {
                size_t jammed = 0, wanted = 0;
                for (const VoiceCandidate& c : submitted) {
                    if (c.v->gap < 25.0) ++jammed;
                    if (c.v->v < 0.6 * std::max(1.0, c.v->v0)) ++wanted;
                }
                congestionBlocked = std::max(congestionBlocked, static_cast<f64>(jammed));
                congestionSlow = std::max(congestionSlow, static_cast<f64>(wanted));
            }
        }
        std::sort(submitted.begin(), submitted.end(),
                  [](const VoiceCandidate& a, const VoiceCandidate& b) { return a.range < b.range; });
        // Two voices per vehicle (engine and tyres) plus any horn already
        // sounding: the budget is halved for vehicles, and an existing horn is
        // never dropped mid-blast.
        std::vector<const VoiceCandidate*> playing;
        for (VoiceCandidate& c : submitted) {
            HornTrigger& ht = horns[c.index];
            ht.playing = std::max(0.0, ht.playing - 0.02);
            ht.cooldown = std::max(0.0, ht.cooldown - 0.02);
            // The horn rule is the one a Mumbai driver actually uses: closing on
            // a leader at well under the speed the vehicle wants, and doing it
            // for a second and a half.  The archetype decides how likely and how
            // long (the bus and the taxi lean on it; the motorcycle answers).
            const bool blocked = c.v->gap < 25.0 && c.v->v < 0.6 * std::max(1.0, c.v->v0);
            if (blocked) ++heldFrames;
            ht.held = blocked ? ht.held + 0.02 : 0.0;
            if (ht.held > 1.5 && ht.cooldown <= 0.0 && ht.playing <= 0.0) {
                Rng rng(seed ^ (static_cast<u64>(c.index) * 0x9E3779B97F4A7C15ull) ^
                        static_cast<u64>(t * 100.0));
                if (rng.nextF32() < static_cast<f32>(hornProbability(c.kind))) {
                    ht.playing = hornLength(c.kind);
                    ht.length = ht.playing;
                    ht.fundamental = hornFundamental(c.kind);
                    ht.cooldown = 6.0;
                    ++hornBlasts;
                } else {
                    ht.cooldown = 4.0;
                }
            }
            if (ht.playing > 0.0) playing.push_back(&c);
        }
        u32 budget = mix.voiceBudget() > 8 ? mix.voiceBudget() - 8 : 8;
        budget /= 2;                                   // engine + tyres per vehicle
        for (const VoiceCandidate* pc : playing) {
            if (budget == 0) break;
            --budget;                                  // horns are never dropped
        }
        for (const VoiceCandidate& c : submitted) {
            if (budget == 0) break;
            --budget;
            SourceSpec e;
            e.id = idEngine(c.index);
            e.layer = Layer::Engine;
            e.kind = c.kind;
            e.pos = WPos{c.p.x, static_cast<f64>(c.groundY) + 0.7, c.p.y};
            e.vx = std::sin(c.heading) * c.speed;
            e.vz = std::cos(c.heading) * c.speed;
            sources.push_back(e);
            sourceCensus[static_cast<size_t>(Layer::Engine)] += 1;

            SourceSpec ty = e;
            ty.id = idTyres(c.index);
            ty.layer = Layer::Tyres;
            ty.pos = WPos{c.p.x, static_cast<f64>(c.groundY) + 0.3, c.p.y};
            ty.wetness = rain > 1.0 ? 0.65f : 0.15f;
            sources.push_back(ty);
            ++tyreVoices;
        }
        for (const VoiceCandidate* pc : playing) {
            const VoiceCandidate& c = *pc;
            HornTrigger& ht = horns[c.index];
            SourceSpec hn;
            hn.id = idHorn(c.index);
            hn.layer = Layer::Horn;
            hn.kind = c.kind;
            hn.pos = WPos{c.p.x, static_cast<f64>(c.groundY) + 0.8, c.p.y};
            hn.vx = std::sin(c.heading) * c.speed;
            hn.vz = std::cos(c.heading) * c.speed;
            hn.f0 = ht.fundamental;
            // 5 ms of attack, a steady middle, and a release over the last 15 %.
            const f64 elapsed = ht.length - ht.playing;
            f64 env = std::min(1.0, elapsed / 0.005);
            env = std::min(env, std::min(1.0, ht.playing / std::max(0.02, ht.length * 0.15)));
            hn.envelope = static_cast<f32>(env);
            sources.push_back(hn);
            sourceCensus[static_cast<size_t>(Layer::Horn)] += 1;
        }

        // --- the sea, from the world's own water mesh -----------------------
        surfSources = 0;
        {
            for (auto& resPtr : streamer.resident()) {
                Resident& res = *resPtr;
                const ChunkReader::MeshView water =
                    res.reader.meshByLod(SectionType::WaterMesh, 0);
                if (!water.valid()) continue;
                const f64 ox = static_cast<f64>(res.cell.cx) * MACRO_CELL_SIZE;
                const f64 oz = static_cast<f64>(res.cell.cz) * MACRO_CELL_SIZE;
                // One surf source per ~64 water vertices, capped: a shore is one
                // extended source, not a thousand points.
                for (u32 i = 0; i < water.vertexCount && surfSources < 6; i += 64) {
                    const MeshVertex& v = water.vertices[i];
                    const f64 wx = ox + v.px, wz = oz + v.pz;
                    const f64 dx = wx - listenerXZ.x, dz = wz - listenerXZ.y;
                    const f64 d2 = dx * dx + dz * dz;
                    if (d2 > 500.0 * 500.0) continue;
                    SourceSpec s;
                    s.id = idSurf(static_cast<u32>(surfSources));
                    s.layer = Layer::Surf;
                    s.pos = WPos{wx, static_cast<f64>(v.py), wz};
                    s.f0 = static_cast<f32>(swell);
                    s.level = 1.0f;
                    // The surf is an extended source: the level falls as a line
                    // source would (1/sqrt(r) beyond a reference), which the mixer
                    // cannot know, so it is folded into the authored level here.
                    const f64 r = std::sqrt(std::max(1.0, d2));
                    s.level = static_cast<f32>(clamp(std::sqrt(30.0 / r) * 1.6, 0.05, 2.5));
                    sources.push_back(s);
                    ++surfSources;
                }
            }
        }

        // --- render the block ------------------------------------------------
        mix.beginBlock(t, sources);
        mix.renderBlock(buf.data());
        mixDown.insert(mixDown.end(), buf.begin(), buf.end());
        peakVoices = std::max(peakVoices, mix.activeVoices());

        // Keep the loudest 30 s window as we go, so the excerpt needs no second
        // pass over the drive.
        const size_t window = static_cast<size_t>(excerptSeconds * sr) * 2;
        if (mixDown.size() >= window) {
            static f64 bestPower = -1.0;
            static bool haveBest = false;
            f64 power = 0.0;
            const size_t from = mixDown.size() - window;
            for (size_t i = from; i < mixDown.size(); ++i) power += static_cast<f64>(mixDown[i]) * mixDown[i];
            if (!haveBest || power > bestPower) {
                haveBest = true;
                bestPower = power;
                harbour.assign(mixDown.begin() + static_cast<i64>(from), mixDown.end());
            }
        }
        t += static_cast<f64>(block) / sr;
        if (static_cast<u64>(mixDown.size() / 2) % (sr * 30u) == 0) {
            MLOG_INFO("audio: %.1f of %.1f minutes, arc %.2f km, %u voices, %llu blasts",
                      t / 60.0, minutes, arc / 1000.0, mix.activeVoices(),
                      static_cast<unsigned long long>(hornBlasts));
        }
    }

    // --- measure, then write --------------------------------------------------
    LoudnessMeter meter;
    meter.addBlock(mixDown.data(), static_cast<u32>(mixDown.size() / 2), 2);
    const MixStats& st = mix.stats();
    const f64 lufs = meter.integratedLufs();
    const f64 rms = rmsFromStats(st);
    const f64 tp = truePeakDbtp(mixDown, 2);

    ensureDir("build/audio");
    ensureDir("viewer/data/audio");
    const bool wrote = writeWav(outPath, mixDown, 2, sr, false, true);
    std::vector<f32> excerpt = harbour;
    if (excerpt.empty()) excerpt = mixDown;
    const bool wroteExcerpt = writeWav(excerptPath, excerpt, 2, sr, false, true);
    // The excerpt is measured on its own: it is what the player hears, and its
    // level is not the drive's level (the drive's quiet approach to Bandra is not
    // in it).
    LoudnessMeter excerptMeter;
    excerptMeter.addBlock(excerpt.data(), static_cast<u32>(excerpt.size() / 2), 2);
    const f64 excerptLufs = excerptMeter.integratedLufs();
    const f64 excerptTp = truePeakDbtp(excerpt, 2);

    MLOG_INFO("audio: wrote %s (%.1f MiB) and %s (%.1f s)",
              outPath.c_str(), static_cast<f64>(fileSize(outPath)) / (1024.0 * 1024.0),
              excerptPath.c_str(), static_cast<f64>(excerpt.size() / 2) / sr);
    MLOG_INFO("audio: %.1f LUFS, peak %.4f (%.2f dBFS), true peak %.2f dBTP, "
              "RMS %.4f (%.2f dBFS)", lufs, st.peak,
              20.0 * std::log10(std::max(1e-12, st.peak)), tp, rms,
              20.0 * std::log10(std::max(1e-12, rms)));
    MLOG_INFO("audio: occlusion: %.1f %% of voice-frames within 30 m blocked "
              "(mean %.3f), mean %.3f beyond 30 m",
              100.0 * (st.voicesNear ? static_cast<f64>(st.occludedNear) / static_cast<f64>(st.voicesNear) : 0.0),
              st.voicesNear ? st.occlusionSumNear / static_cast<f64>(st.voicesNear) : 0.0,
              st.voicesFar ? st.occlusionSumFar / static_cast<f64>(st.voicesFar) : 0.0);
    MLOG_INFO("audio: %llu voice-frames, peak %u voices, %llu clipped samples, "
              "%llu limiter grabs, %llu occluded voice-frames",
              static_cast<unsigned long long>(st.voicesRendered), peakVoices,
              static_cast<unsigned long long>(st.overFullScale),
              static_cast<unsigned long long>(st.limiterGrabs),
              static_cast<unsigned long long>(st.occludedVoices));
    MLOG_INFO("audio: drive %.2f km, %llu horn blasts, Doppler range %.4f..%.4f "
              "at up to %.1f m/s of range rate",
              arc / 1000.0, static_cast<unsigned long long>(hornBlasts),
              minDopplerFactor, maxDopplerFactor, maxRelSpeed);
    MLOG_INFO("audio: %llu loads, %llu evictions, %llu cells without ground, %.2f s",
              static_cast<unsigned long long>(loads),
              static_cast<unsigned long long>(evictions),
              static_cast<unsigned long long>(resets), timer.seconds());

    // --- the manifest ---------------------------------------------------------
    Json j;
    j.set("sampleRate", static_cast<f64>(sr));
    j.set("channels", 2);
    j.set("bits", 16);
    j.set("dither", true);
    j.set("minutes", minutes);
    j.set("frames", static_cast<f64>(st.frames));
    j.set("driveMetres", arc);
    j.set("routeMetres", routeLength);
    j.set("vehicles", static_cast<f64>(fleet));
    j.set("rainMmPerHour", rain);
    j.set("swellMetres", swell);
    j.set("wetness", 0.65);
    j.set("integratedLufs", lufs);
    j.set("peak", st.peak);
    j.set("peakDbfs", 20.0 * std::log10(std::max(1e-12, st.peak)));
    j.set("truePeakDbtp", tp);
    j.set("rmsDbfs", 20.0 * std::log10(std::max(1e-12, rms)));
    j.set("clippedSamples", static_cast<f64>(st.overFullScale));
    j.set("limiterGrabs", static_cast<f64>(st.limiterGrabs));
    j.set("occludedVoiceFrames", static_cast<f64>(st.occludedVoices));
    j.set("voiceFramesWithin30m", static_cast<f64>(st.voicesNear));
    j.set("occludedVoiceFramesWithin30m", static_cast<f64>(st.occludedNear));
    j.set("meanOcclusionWithin30m", st.voicesNear ? st.occlusionSumNear / static_cast<f64>(st.voicesNear) : 0.0);
    j.set("meanOcclusionBeyond30m", st.voicesFar ? st.occlusionSumFar / static_cast<f64>(st.voicesFar) : 0.0);
    j.set("voiceFrames", static_cast<f64>(st.voicesRendered));
    j.set("peakVoices", static_cast<f64>(peakVoices));
    j.set("hornBlasts", static_cast<f64>(hornBlasts));
    j.set("peakVehiclesWithCloseLeader", congestionBlocked);
    j.set("peakVehiclesUnderDesiredSpeed", congestionSlow);
    j.set("hornRuleSeconds", 1.5);
    j.set("dopplerMinFactor", minDopplerFactor);
    j.set("dopplerMaxFactor", maxDopplerFactor);
    j.set("maxRangeRateMetresPerSecond", maxRelSpeed);
    j.set("occluders", static_cast<f64>(occluders.size()));
    j.set("voiceBudget", static_cast<f64>(mix.voiceBudget()));
    j.set("earshotMetres", earshot);
    {
        Json census;
        for (size_t i = 0; i < 7; ++i) census.set(std::string("layer") + std::to_string(i),
                                                static_cast<f64>(sourceCensus[i]));
        census.set("tyreVoices", static_cast<f64>(tyreVoices));
        j.set("submitted", census);
    }
    j.set("cellsWithoutGround", static_cast<f64>(resets));
    j.set("wav", outPath);
    j.set("excerpt", excerptPath);
    j.set("excerptSeconds", static_cast<f64>(excerpt.size() / 2) / sr);
    j.set("excerptLufs", excerptLufs);
    j.set("excerptTruePeakDbtp", excerptTp);
    // The simulated length, not the wall clock: this file is committed under
    // viewer/data/, and a number that changes with the machine's speed makes the
    // published manifest differ on every rebuild.  The wall clock is still logged
    // (above) for anyone chasing the run's cost.
    j.set("simulatedSeconds", static_cast<f64>(totalFrames) / static_cast<f64>(sr));
    {
        size_t count = 0;
        const LayerBudget* budgets = layerBudgets(count);
        Json layers;
        for (size_t i = 0; i < count; ++i) {
            Json l;
            l.set("name", budgets[i].name);
            l.set("referenceDbAt10m", static_cast<f64>(budgets[i].referenceDbAt10m));
            l.set("maxVoices", static_cast<f64>(budgets[i].maxVoices));
            layers.push(l);
        }
        j.set("layers", layers);
    }
    if (!writeFile("build/audio/audio_manifest.json", j.dump(2))) {
        MLOG_ERR("audio: cannot write the manifest");
        return 3;
    }

    if (!wrote || !wroteExcerpt) return 4;
    const bool clean = st.overFullScale == 0 && tp <= -0.5 && std::isfinite(lufs);
    MLOG_INFO("audio: excerpt %.1f LUFS, %.2f dBTP", excerptLufs, excerptTp);
    MLOG_INFO("audio: %s", clean ? "PASS — nothing clipped, the mix is a programme"
                                 : "FAIL — see the numbers above");
    return clean ? 0 : 1;
}
