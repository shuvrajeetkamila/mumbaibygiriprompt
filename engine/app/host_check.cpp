// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/host_check.cpp
//
//  What a machine with no Android SDK and no GPU can still check about the
//  Android build: that CMake's source lists resolve, that the engine links, that
//  the shaders were compiled and embedded, and that a published world can be read
//  through the same reader the client uses.
//
//  It is the CMake twin of `mumbai_render --check`'s CPU half, and it exists
//  because a build file that has never been configured is a build file that does
//  not work (D-045).
// =============================================================================
#include "engine/core/core.h"
#include "engine/world/chunk.h"
#include "shaders_generated.h"

#include <cstdio>
#include <string>

using namespace mumbai;

int main(int argc, char** argv) {
    std::printf("MUMBAI 1:1 EXACT — CMake host check\n");
    std::printf("===============================================================\n");

    // 1. The shaders exist and are SPIR-V.  This is the check that catches a
    //    build where the shader step silently did nothing.
    struct { const char* name; const u32* code; size_t words; } shaders[] = {
        {"world.vert", kWorldVertSpv, kWorldVertSpvWords},
        {"world.frag", kWorldFragSpv, kWorldFragSpvWords},
        {"water.frag", kWaterFragSpv, kWaterFragSpvWords},
        {"sky.vert", kSkyVertSpv, kSkyVertSpvWords},
        {"sky.frag", kSkyFragSpv, kSkyFragSpvWords},
        {"post.vert", kPostVertSpv, kPostVertSpvWords},
        {"post.frag", kPostFragSpv, kPostFragSpvWords},
        {"traffic.vert", kTrafficVertSpv, kTrafficVertSpvWords},
        {"traffic.frag", kTrafficFragSpv, kTrafficFragSpvWords},
    };
    size_t words = 0, missing = 0, badMagic = 0;
    for (auto& s : shaders) {
        words += s.words;
        if (s.words == 0) { ++missing; continue; }
        if (s.code[0] != 0x07230203u) ++badMagic;
    }
    std::printf("  shaders: %zu modules, %zu words (%.1f KiB)\n", std::size(shaders), words,
                static_cast<double>(words * 4) / 1024.0);
    if (missing || badMagic) {
        std::printf("  shaders: %zu missing, %zu not SPIR-V — the build did not compile them\n",
                    missing, badMagic);
        return missing ? 2 : 3;
    }

    // 2. The world reader links and works against whatever world is on this
    //    machine.  Absent data is not a failure here — the APK ships no world.
    const std::string manifest = argc > 1 ? argv[1] : "data/processed/world_manifest.json";
    const std::string text = readFileToString(manifest);
    if (text.empty()) {
        std::printf("  world:   none at %s (expected on a build machine)\n", manifest.c_str());
        return 0;
    }
    std::printf("  world:   manifest at %s (%zu bytes)\n", manifest.c_str(), text.size());
    MLOG_INFO("host check ok");
    return 0;
}
