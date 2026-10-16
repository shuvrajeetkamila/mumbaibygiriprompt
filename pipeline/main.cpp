// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/main.cpp
//
//  The offline world builder's entry point.  Ingests the raw geospatial layers
//  and emits a directory of .mumbai_chunk files plus the world manifest.
//
//    mumbai_pipeline --config configs/world.json
//    mumbai_pipeline --report data/processed/world_manifest.json
//    mumbai_pipeline --dem survey.tif [--config ...]
// =============================================================================
#include "pipeline/pipeline.h"

using namespace mumbai;
using namespace mumbai::pipeline;

namespace {

int runReport(const std::string& manifestPath) {
    const std::string text = readFileToString(manifestPath);
    if (text.empty()) {
        MLOG_ERR("cannot read manifest %s", manifestPath.c_str());
        return 1;
    }
    Json j;
    std::string err;
    if (!Json::parse(text, j, &err)) {
        MLOG_ERR("manifest parse error: %s", err.c_str());
        return 1;
    }
    const Json& t = j["totals"];
    std::printf("MUMBAI world manifest: %s\n", manifestPath.c_str());
    std::printf("  extent            %.1f km^2\n", j["extent"]["squareKilometres"].asNumber());
    std::printf("  cells             %lld\n", (long long)t["cellCount"].asInt());
    std::printf("  storage           %.3f GB\n", t["bytes"].asNumber() / 1e9);
    std::printf("  buildings         %lld\n", (long long)t["buildings"].asInt());
    std::printf("  road centreline   %.1f km\n", t["roadCentrelineMetres"].asNumber() / 1000.0);
    std::printf("  navigation        %lld nodes / %lld edges\n",
                (long long)t["navNodes"].asInt(), (long long)t["navEdges"].asInt());
    std::printf("  triangles         %lld\n", (long long)t["triangles"].asInt());
    std::printf("  instances         %lld\n", (long long)t["instances"].asInt());
    std::printf("  hero assets       %lld\n", (long long)t["heroAssets"].asInt());
    std::printf("  QA                %s\n", j["quality"]["ok"].asBool() ? "PASS" : "FAIL");
    return j["quality"]["ok"].asBool() ? 0 : 2;
}

} // namespace

int main(int argc, char** argv) {
    const CliArgs args = parseCli(argc, argv);

    if (args.has("--help") || args.has("-h")) {
        std::printf(
            "mumbai_pipeline — offline world builder for MUMBAI 1:1 EXACT\n\n"
            "  --config <file>    build configuration (default configs/world.json)\n"
            "  --raw <dir>        raw source data directory (default data/raw)\n"
            "  --out <dir>        output directory (default data/processed)\n"
            "  --dem <file>       use a real DEM instead of synthesised terrain\n"
            "  --report <file>    print a manifest report and exit\n"
            "  --no-verify        skip re-opening written chunks\n"
            "  --verbose          debug logging\n");
        return 0;
    }

    if (args.has("--report")) {
        const std::string path = args.get("--report", "data/processed/world_manifest.json");
        return runReport(path);
    }

    const std::string configPath = args.get("--config", "configs/world.json");
    BuildConfig cfg;
    {
        const std::string text = readFileToString(configPath);
        if (!text.empty()) {
            Json j;
            std::string err;
            if (Json::parse(text, j, &err)) {
                cfg = BuildConfig::fromJson(j);
                MLOG_INFO("config loaded from %s", configPath.c_str());
            } else {
                MLOG_WARN("config parse error (%s) — using defaults", err.c_str());
            }
        } else {
            MLOG_WARN("no config at %s — using defaults", configPath.c_str());
        }
    }
    if (args.has("--raw")) cfg.rawDir = args.get("--raw");
    if (args.has("--out")) cfg.outDir = args.get("--out");
    if (args.has("--no-verify")) cfg.verifyChunks = false;
    if (args.has("--verbose")) logSetLevel(LogLevel::Debug);

    Context ctx;
    ctx.cfg = cfg;
    ctx.proj = TransverseMercator();

    std::printf("\n=== MUMBAI 1:1 EXACT — geospatial pipeline ===\n");
    std::printf("  raw: %s\n  out: %s\n\n", cfg.rawDir.c_str(), cfg.outDir.c_str());
    ensureDir(cfg.outDir);

    f64 tStart = ScopedTimer::now();
    struct Stage { const char* name; bool (*fn)(Context&); };
    const Stage stages[] = {
        {"loadSources",        loadSources},
        {"buildTerrain",       buildTerrain},
        {"buildZoning",        buildZoning},
        {"emitTerrainMeshes",  emitTerrainMeshes},
        {"buildWater",         buildWater},
        {"buildRoads",         buildRoads},
        {"buildRailAndLanduse",buildRailAndLanduse},
        {"buildBuildings",     buildBuildings},
        {"buildProps",         buildProps},
        {"buildHeroAssets",    buildHeroAssets},
        // Textures come after every material exists (facade variants are minted
        // per building) and before anything is serialised.  A8.
        {"buildTextures",      buildTextures},
        {"buildFloodModel",    buildFloodModel},
        {"writeChunks",        writeChunks},
        {"writeManifest",      writeManifest},
        {"exportViewer",       exportViewer},
    };

    for (const Stage& s : stages) {
        const f64 t0 = ScopedTimer::now();
        MLOG_INFO("--- stage: %s", s.name);
        if (!s.fn(ctx)) {
            MLOG_ERR("stage %s failed", s.name);
            printReport(ctx);
            return 1;
        }
        const f64 dt = ScopedTimer::now() - t0;
        std::printf("  [%-20s] %7.2f s\n", s.name, dt);
        // A timing, not a fact: see QaReport::timings (D-051).
        ctx.qa.addTiming(std::string("stage.") + s.name + ".seconds", dt);
    }

    {
        const f64 t0 = ScopedTimer::now();
        if (!finaliseQa(ctx, t0)) return 1;
        std::printf("  [%-20s] %7.2f s\n", "finaliseQa", ScopedTimer::now() - t0);
    }

    const f64 total = ScopedTimer::now() - tStart;
    std::printf("\n  total build time: %.2f s\n", total);
    printReport(ctx);
    ctx.qa.logSummary();

    return ctx.qa.ok() ? 0 : 2;
}
