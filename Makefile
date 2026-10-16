# =============================================================================
#  MUMBAI 1:1 EXACT — build system
# -----------------------------------------------------------------------------
#  Portable C++20 build. No third-party C++ dependencies are required to build
#  the pipeline, engine core, simulation or tests (see DOCS/00_ARCHITECTURE.md,
#  decision D-002 for why the toolchain is C++ rather than the Rust proposed in
#  the original specification, and decision D-014 for the dependency policy).
#
#  Targets:
#    make            - build pipeline, engine, exporter and tests
#    make test       - run the full unit/integration suite
#    make world      - run the geospatial pipeline over data/raw -> data/processed
#    make simulate   - run the headless engine validation suite
#    make viewer     - vendored three.js + viewer assets
#    make all-checks - test + simulate + verify world + verify artifacts
# =============================================================================

# The default goal is stated rather than inherited from whichever rule happens to
# come first: a rule inserted above `all` silently becomes the whole build, and
# `make` then says "up to date" for a binary it never linked.  (Found by exactly
# that: the A1 shader rule made `make` build the shader header and stop.)
.DEFAULT_GOAL := all

CXX      ?= g++
CC       ?= gcc
CXXSTD   ?= -std=c++20
OPT      ?= -O2
WARN     := -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
# The two zlib flags live in the general flag set on purpose.  The vendored zlib is
# compiled with -DZ_PREFIX, which renames every zlib symbol to `z_*` so that an
# Android process's own zlib cannot bind to ours; if the code that *includes*
# zlib.h were compiled without it, the engine would call `inflate` and the vendored
# object would define `z_inflate`, and the link would fail with two names for one
# function.  One flag set for both sides of that include removes the failure mode
# instead of documenting it (third_party/zlib/README.md).
CXXFLAGS ?= $(CXXSTD) $(OPT) -g -pthread $(WARN) -I. -Ithird_party/zlib -DZ_PREFIX -MMD -MP
LDFLAGS  ?=
LDLIBS   ?= -pthread

BUILD    := build
BIN      := $(BUILD)/bin
OBJ      := $(BUILD)/obj

# --- Sources ----------------------------------------------------------------
CORE_SRC    := $(wildcard engine/core/*.cpp)
WORLD_SRC   := $(wildcard engine/world/*.cpp)
SIM_SRC     := $(wildcard engine/sim/*.cpp)
AUDIO_SRC   := $(wildcard engine/audio/*.cpp)
MGEOM_SRC   := $(wildcard pipeline/mgeo/*.cpp)
STAGE_SRC   := $(wildcard pipeline/stages/*.cpp)

PHYS_SRC    := $(wildcard engine/physics/*.cpp)
PHYS_OBJ    := $(patsubst %.cpp,$(OBJ)/%.o,$(PHYS_SRC))
PHYS_MAIN   := $(OBJ)/engine/app/phys_main.o
ENGINE_SRC  := $(CORE_SRC) $(WORLD_SRC) $(SIM_SRC) $(AUDIO_SRC)
PIPE_SRC    := $(MGEOM_SRC) $(STAGE_SRC)

# --- Shaders (A1) ------------------------------------------------------------
# SPIR-V is compiled at build time and embedded in the binary by
# tools/embed_spirv.py, so a device that can start the engine can always find its
# shaders.  `glslc` is optional: when it is absent the generated header carries
# empty arrays and every pipeline fails loudly with "shader missing", which is a
# better failure than a silent black screen.
GLSLC       ?= glslc
SHADER_DIR  := engine/render/shaders
SHADER_SRC  := $(wildcard $(SHADER_DIR)/*.vert $(SHADER_DIR)/*.frag $(SHADER_DIR)/*.glsl)
SHADER_HDR  := $(OBJ)/shaders_generated.h
SHADERS_OK  := $(shell command -v $(GLSLC) 2>/dev/null)

$(SHADER_HDR): $(SHADER_SRC) tools/embed_spirv.py tools/compile_shaders.py
	@mkdir -p $(OBJ)/spv
	@python3 tools/compile_shaders.py $(SHADER_DIR) $(OBJ)/spv
	@python3 tools/embed_spirv.py $@ $(OBJ)/spv

# `make shaders` compiles and embeds in one step and is worth running on its own:
# it is the only thing in this project that a GPU driver will ever read.
shaders: $(SHADER_HDR)
	@echo "== shaders embedded in $<=="

# --- Vulkan headers (A1) -----------------------------------------------------
# Vendored (SDK 1.3.290, Apache-2.0) so the renderer compiles anywhere, GPU or
# not: the point of the compile check in `make render` is that A1 is *built*,
# not merely written.
RENDER_SRC  := $(wildcard engine/render/*.cpp)
RENDER_OBJ  := $(patsubst %.cpp,$(OBJ)/%.o,$(RENDER_SRC))
RENDER_FLAGS := $(CXXFLAGS) -Ithird_party/vulkan/include -I$(OBJ)
# camera.cpp is the one renderer source with no Vulkan dependency, so the CPU-only
# harness can link it.  (Defined here, above every rule that uses it: a variable
# assigned below its first use expands to nothing, and the linker then reports a
# missing symbol in a file that is sitting right there on disk.)
RENDER_CPU_OBJ := $(OBJ)/engine/render/camera.o $(OBJ)/engine/render/hud.o \
                  $(OBJ)/engine/render/fallback_atlas.o $(OBJ)/engine/render/world_index.o \
                  $(OBJ)/engine/render/dynamic_resolution.o

# The Vulkan *loader* is a runtime dependency, not a build one.  When it is not
# installed the renderer's sources are still compiled — which is the part of A1 a
# machine without a GPU can verify — and the harness links in CPU-only mode
# instead of failing at link time with "cannot find -lvulkan".
# The test is an actual link, not an inspection of the filesystem.  An earlier
# version looked for libvulkan.so* and found libvulkan.so.1 on GitHub's Ubuntu
# image — the runtime library, which ships without the development symlink that
# -lvulkan needs — so the build tried to link something that was never linkable
# and stopped with "cannot find -lvulkan" on a machine that had *more* Vulkan
# installed than the sandbox this was written in.  A probe cannot make that
# mistake, and it costs one trivial compile per make invocation.
VULKAN_LOADER := $(shell printf 'int main(){return 0;}\n' | $(CXX) $(CXXSTD) -x c++ - -lvulkan \
                          -o /dev/null 2>/dev/null && echo yes)
ifeq ($(VULKAN_LOADER),)
RENDER_MAIN_FLAGS := -DMUMBAI_NO_VULKAN
else
RENDER_MAIN_FLAGS :=
endif

$(OBJ)/engine/render/%.o: engine/render/%.cpp $(SHADER_HDR)
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(RENDER_FLAGS) -c $< -o $@

# --- Vendored physics engine (A2) -------------------------------------------
# Jolt ships CMake; this environment has none, so its file list is transcribed
# into third_party/jolt_sources.mk and built by the rules below.  The flags are
# Jolt's own Distribution preset: no asserts, no profiler, no debug renderer, no
# object stream, x86-64 baseline SIMD (SSE4.2 + popcnt — not AVX2, so a binary
# built here runs on any host).  Warnings are silenced: it is vendored code and
# its warnings are not this project's to fix.
include third_party/jolt_sources.mk

JOLT_OBJ    := $(patsubst %.cpp,$(OBJ)/third_party/%.o,$(JOLT_SRC))
# LZCNT/TZCNT are deliberately *not* defined: the macros make Jolt call the
# `_tzcnt_u32`/`_lzcnt_u32` intrinsics, which need -mbmi/-mlzcnt, and -mbmi
# would raise the deployed binary's instruction-set floor above the x86-64
# baseline this project targets.  Undefined, Math.h uses its software fallback.
JOLT_FLAGS  := -std=c++17 $(OPT) -g -pthread -w -fno-rtti -fno-exceptions \
               -msse4.2 -mpopcnt -Ithird_party \
               -DJPH_USE_SSE4_1 -DJPH_USE_SSE4_2

# engine/physics includes Jolt's headers, so it needs the same defines and the
# same include root.  It is the only engine source that does.
PHYS_FLAGS  := $(CXXFLAGS) -fno-rtti -fno-exceptions -msse4.2 -mpopcnt \
               -Ithird_party -DJPH_USE_SSE4_1 -DJPH_USE_SSE4_2

$(OBJ)/engine/physics/%.o: engine/physics/%.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(PHYS_FLAGS) -c $< -o $@

$(OBJ)/third_party/%.o: third_party/%.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX  $< (jolt)"
	@$(CXX) $(JOLT_FLAGS) -MMD -MP -c $< -o $@

ENGINE_OBJ  := $(patsubst %.cpp,$(OBJ)/%.o,$(ENGINE_SRC))

# --- Vendored zlib, inflate only (D-050) -------------------------------------
# The world arrives as an uncompressed tar whose members are gzipped, so the client
# needs an inflater; third_party/zlib/README.md says which files and why only the
# inflate half.  It is a C library, so it gets its own rule and its own compiler
# variable rather than being smuggled through the C++ ones.  Its objects are part
# of ENGINE_OBJ because every binary that links the engine links the code that
# calls into them, and a link line that could forget them is a link line that will.
ZLIB_DIR   := third_party/zlib
ZLIB_SRC   := $(wildcard $(ZLIB_DIR)/*.c)
ZLIB_OBJ   := $(patsubst $(ZLIB_DIR)/%.c,$(OBJ)/$(ZLIB_DIR)/%.o,$(ZLIB_SRC))
CFLAGS     ?= $(OPT) -g -Wall -Wextra

$(OBJ)/$(ZLIB_DIR)/%.o: $(ZLIB_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "  CC   $<"
	@$(CC) $(CFLAGS) -DZ_PREFIX -I$(ZLIB_DIR) -c $< -o $@

ENGINE_OBJ  += $(ZLIB_OBJ)
PIPE_OBJ    := $(patsubst %.cpp,$(OBJ)/%.o,$(PIPE_SRC))

MAIN_OBJ     := $(OBJ)/pipeline/main.o
PIPELINE_BIN := $(BIN)/mumbai_pipeline
# One object per binary gets a name of its own when it has to be listed in the
# dependency include below — see the note there.  `sim_main.o` is the second of
# the two, and it is the one that was forgotten: the runtime harness includes
# `engine/sim/traffic.h`, whose `Traffic` object is built by value on the stack,
# and an object compiled against a different layout of it wrote the model's own
# containers with the wrong offsets.  The symptom was a segmentation fault at
# tick 45 of a 60-minute run with `m_nodes.size()` reading 0x3333333333333332 —
# plausible-looking traffic code blamed for a missing `-include` (D-034).
SIM_MAIN     := $(OBJ)/engine/app/sim_main.o
RENDER_MAIN  := $(OBJ)/engine/app/render_main.o
RENDER_BIN   := $(BIN)/mumbai_render
SIM_BIN      := $(BIN)/mumbai_sim
TEST_BIN     := $(BIN)/mumbai_tests
PHYS_BIN     := $(BIN)/mumbai_phys
AUDIO_MAIN   := $(OBJ)/engine/app/audio_main.o
AUDIO_BIN    := $(BIN)/mumbai_audio

TEST_SRC := $(wildcard tests/*.cpp)

.PHONY: all test world simulate physics audio render render-objects shaders glslang host-check viewer serve clean all-checks stats verify-artifacts android-check

all: $(PIPELINE_BIN) $(SIM_BIN) $(TEST_BIN) $(PHYS_BIN) $(AUDIO_BIN) $(RENDER_BIN)

$(OBJ)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

# Header dependencies.  Without these, editing a header (a struct layout, a
# section enum) leaves objects compiled against the old definition linked into
# the same binary: the ABI mismatch corrupts containers and manifests as
# impossible failures (a std::bad_alloc inside an unrelated stage was the
# symptom that motivated adding this).
# main.o and sim_main.o are named explicitly as prerequisites of their binaries
# and their dependency files must be included with the rest: leaving one out
# means no header edit ever rebuilds it, and a header that changes the layout of
# the type it builds by value then links a stale object against fresh ones.  That
# is not a missing rebuild, it is a segmentation fault — at startup for Context,
# mid-run for Traffic.  Both have now happened once; every object named on a link
# line below belongs on this line too.
-include $(ENGINE_OBJ:.o=.d) $(PIPE_OBJ:.o=.d) $(MAIN_OBJ:.o=.d) $(SIM_MAIN:.o=.d) \
         $(PHYS_OBJ:.o=.d) $(PHYS_MAIN:.o=.d) $(JOLT_OBJ:.o=.d) $(AUDIO_MAIN:.o=.d) \
         $(RENDER_OBJ:.o=.d) $(RENDER_MAIN:.o=.d)

$(PIPELINE_BIN): $(PIPE_OBJ) $(ENGINE_OBJ) $(MAIN_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(SIM_BIN): $(ENGINE_OBJ) $(SIM_MAIN)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

# The runtime harness builds `sim::Traffic` by value, so the model's header is a
# hard prerequisite of its object in the strong sense: a layout change must
# recompile it whatever the generated dependency file says.  (It does not include
# a Jolt header, so the generic rule's flags are the right ones for it.)
$(SIM_MAIN): engine/sim/traffic.h

# The harness reads Jolt headers too, so it uses the same flag set as the
# physics sources (an explicit rule, because it lives outside engine/physics).
$(PHYS_MAIN): engine/app/phys_main.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(PHYS_FLAGS) -c $< -o $@

$(PHYS_BIN): $(PHYS_OBJ) $(PHYS_MAIN) $(ENGINE_OBJ) $(JOLT_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXSTD) $(OPT) -g -pthread -I. -fno-rtti -fno-exceptions $^ -o $@ $(LDFLAGS) $(LDLIBS)

# The audio harness builds `sim::Traffic` by value too, so its own header rule
# below is the same one sim_main carries (and for the same reason).
$(AUDIO_MAIN): engine/app/audio_main.cpp engine/sim/traffic.h engine/world/chunk.h
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(AUDIO_BIN): $(ENGINE_OBJ) $(AUDIO_MAIN)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(RENDER_MAIN): engine/app/render_main.cpp engine/render/renderer.h $(SHADER_HDR)
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(RENDER_FLAGS) $(RENDER_MAIN_FLAGS) -c $< -o $@

# The renderer's own sources are part of this binary only: they need Vulkan
# headers, and the headless harnesses must keep building on a machine that has
# none.
ifeq ($(VULKAN_LOADER),)
$(RENDER_BIN): $(RENDER_MAIN) $(RENDER_CPU_OBJ) $(ENGINE_OBJ) $(WORLD_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@ (no Vulkan loader: CPU-only harness)"
	@$(CXX) $(CXXSTD) $(OPT) -g -pthread $(RENDER_FLAGS) $(RENDER_MAIN_FLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)
else
$(RENDER_BIN): $(RENDER_OBJ) $(RENDER_MAIN) $(ENGINE_OBJ) $(WORLD_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXSTD) $(OPT) -g -pthread $(RENDER_FLAGS) $^ -o $@ $(LDFLAGS) -lvulkan $(LDLIBS)
endif

# The compile check for the device layer itself: these objects are built whether
# or not a loader exists, because "does the Vulkan code compile" is a question
# with an answer everywhere.
render-objects: $(RENDER_OBJ)

# A1's compile check: builds the renderer, its shaders and its harness.  On a
# machine with no GPU this is the whole verification available, and it is the one
# that matters most — a renderer that does not compile is not a renderer.
render: $(RENDER_BIN) render-objects
	@$(RENDER_BIN) --check

# tests/test_render.cpp needs render::Camera, which lives in the one renderer
# source with no Vulkan dependency — so the headless suite links camera.o and not
# the device layer.
$(TEST_BIN): $(TEST_SRC) $(ENGINE_OBJ) $(PIPE_OBJ) $(RENDER_CPU_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $(TEST_SRC) $(ENGINE_OBJ) $(PIPE_OBJ) $(RENDER_CPU_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

# --- Workflows --------------------------------------------------------------
test: $(TEST_BIN)
	@$(TEST_BIN)

world: $(PIPELINE_BIN)
	@$(PIPELINE_BIN) --config configs/world.json
# Pin the published payload.  The build is byte-reproducible (verified: two
# runs produce identical hashes), so a committed checksum is meaningful, and it
# is what catches a binary that was mangled outside the build — see
# tools/verify_artifacts.sh.
	@cd viewer/data && sha256sum mumbai.bin mumbai.json atlas_basecolor.png > SHA256SUMS
	@echo "  artifacts  viewer/data/SHA256SUMS"

# Acceptance-scale run: 60 simulated minutes along Colaba -> Borivali at 60 Hz
# with a 620-vehicle fleet — the Section 7 continuity requirement.
simulate: $(SIM_BIN)
	@$(SIM_BIN) --minutes 60 --vehicles 620

# The city's sound (A9): the same drive, rendered from the published world into a
# WAV a person can listen to, plus the measurements.  Four minutes by default
# because that is a listenable artefact; `--minutes` scales it.
audio: $(AUDIO_BIN)
	@$(AUDIO_BIN) --minutes 4 --vehicles 160 --rain 42
	@cp build/audio/audio_manifest.json viewer/data/audio/audio_manifest.json

# Physics acceptance harness (A2): the world's own baked collision geometry,
# the Sea Link's stay capsules, both vehicle controllers, and the bulk
# floating-origin shift — measured, not asserted.
physics: $(PHYS_BIN)
	@$(PHYS_BIN) --chunks data/processed/cells --minutes 1 --seconds 30 --props 200 \
	             --fleet 24 --report build/physics_report.json

# Independent verification of the published artifacts: the unit suite, the
# runtime validation run, the viewer payload re-derived from its own offsets by
# the Node verifier, and the integrity of the raw source data.
all-checks: test simulate physics audio host-check android-check
	@node tests/verify_export.mjs
	@node tests/verify_viewer_shaders.mjs
	@node tests/verify_viewer_orientation.mjs
	@python3 tools/verify_shader_layout.py
	@python3 tools/verify_java_bridge.py
	@bash tools/verify_artifacts.sh
	@bash tools/fetch_data.sh --verify-only
	@echo "== all checks complete =="

# The slice of the city that travels inside the APK: South Bombay, 25 cells and
# the albedo array today, packed by the tool the CI workflow also runs.  It is a
# separate target because it needs a built world and nothing else.
starter-world:
	@python3 tools/make_starter_world.py --manifest data/processed/world_manifest.json \
	     --out android/app/src/main/assets/world --region south-bombay --textures albedo

stats: $(PIPELINE_BIN)
	@$(PIPELINE_BIN) --report data/processed/world_manifest.json

verify-artifacts:
	@bash tools/verify_artifacts.sh

# A GLSL compiler for machines that have none.  The shaders are the one thing in
# this project a GPU driver reads; "they have never been compiled" is not a state
# a renderer can ship in.
glslang:
	@bash tools/get_glslang.sh

# Configure and build the Android project's host twin: the same CMakeLists, the
# same source lists, the shaders compiled and embedded, and no Android toolchain
# needed.  It is what a machine without an SDK can check about the APK build
# (D-045); CI builds the real thing.
HOST_CHECK_DIR := build/cmake-host-check
host-check:
	@command -v cmake >/dev/null 2>&1 || { \
	    echo "cmake is needed for the host check (pip install cmake ninja)"; exit 1; }
	@python3 tools/compile_shaders.py engine/render/shaders $(OBJ)/spv
	@cmake -S android -B $(HOST_CHECK_DIR) -DMUMBAI_HOST_CHECK=ON -DMUMBAI_PHYSICS=ON \
	       -DCMAKE_BUILD_TYPE=Release >/dev/null
	@cmake --build $(HOST_CHECK_DIR) -j2 --target mumbai_cmake_hostcheck
	@$(HOST_CHECK_DIR)/mumbai_cmake_hostcheck

# engine/app/android_main.cpp is the one source file the host build never sees:
# it is compiled for the NDK only, inside CI, so a typo in it used to cost a full
# CI round trip.  The Android surface it uses is small enough to mirror as host
# headers (engine/app/android_stubs/, signatures copied from NDK r26), and then the
# same file compiles here with __ANDROID__ defined.  This is a compile check, not a
# run: the stubs have no implementation, which is exactly right for one.
ANDROID_STUBS    := engine/app/android_stubs
# android_http.cpp is the JNI bridge, and it is in this list on purpose: it is
# compiled here against engine/app/android_stubs/jni.h, so a wrong method name or a
# wrong argument type costs a compile on a machine with no NDK rather than a runtime
# failure on a phone.
ANDROID_APP_SRC  := engine/app/android_main.cpp engine/app/fallback_screen.cpp \
                    engine/app/android_http.cpp
ANDROID_APP_OBJ  := $(patsubst engine/app/%.cpp,$(OBJ)/engine/app/%.hostcheck.o,$(ANDROID_APP_SRC))
android-check: $(ANDROID_APP_OBJ)
	@echo "== android_main.cpp compiles with __ANDROID__ against the host stubs =="

$(OBJ)/engine/app/%.hostcheck.o: engine/app/%.cpp $(wildcard $(ANDROID_STUBS)/*.h $(ANDROID_STUBS)/android/*.h) \
        engine/render/hud.h engine/render/renderer.h engine/render/world_index.h \
        engine/world/world_download.h engine/world/world_release.h
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -D__ANDROID__ -isystem $(ANDROID_STUBS) -c $< -o $@

viewer:
	@bash tools/setup_viewer.sh

# Serve the viewer locally.  The published payload is fetched by the page from
# ./data/, which a file:// URL and a sandboxed preview iframe both refuse, so
# the scene is served over HTTP.
serve: viewer
	@echo "== serving http://127.0.0.1:8000/ (ctrl-c to stop) =="
	@python3 -m http.server 8000 --bind 0.0.0.0 --directory viewer

clean:
	@rm -rf $(BUILD)
	@echo "  cleaned $(BUILD)"
