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

CXX      ?= g++
CXXSTD   ?= -std=c++20
OPT      ?= -O2
WARN     := -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CXXFLAGS ?= $(CXXSTD) $(OPT) -g -pthread $(WARN) -I. -MMD -MP
LDFLAGS  ?=
LDLIBS   ?= -pthread

BUILD    := build
BIN      := $(BUILD)/bin
OBJ      := $(BUILD)/obj

# --- Sources ----------------------------------------------------------------
CORE_SRC    := $(wildcard engine/core/*.cpp)
WORLD_SRC   := $(wildcard engine/world/*.cpp)
SIM_SRC     := $(wildcard engine/sim/*.cpp)
RENDER_SRC  := $(wildcard engine/render/*.cpp)
MGEOM_SRC   := $(wildcard pipeline/mgeo/*.cpp)
STAGE_SRC   := $(wildcard pipeline/stages/*.cpp)

ENGINE_SRC  := $(CORE_SRC) $(WORLD_SRC) $(SIM_SRC) $(RENDER_SRC)
PIPE_SRC    := $(MGEOM_SRC) $(STAGE_SRC)

ENGINE_OBJ  := $(patsubst %.cpp,$(OBJ)/%.o,$(ENGINE_SRC))
PIPE_OBJ    := $(patsubst %.cpp,$(OBJ)/%.o,$(PIPE_SRC))

MAIN_OBJ     := $(OBJ)/pipeline/main.o
PIPELINE_BIN := $(BIN)/mumbai_pipeline
SIM_BIN      := $(BIN)/mumbai_sim
TEST_BIN     := $(BIN)/mumbai_tests

TEST_SRC := $(wildcard tests/*.cpp)

.PHONY: all test world simulate viewer serve clean all-checks stats verify-artifacts

all: $(PIPELINE_BIN) $(SIM_BIN) $(TEST_BIN)

$(OBJ)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX  $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

# Header dependencies.  Without these, editing a header (a struct layout, a
# section enum) leaves objects compiled against the old definition linked into
# the same binary: the ABI mismatch corrupts containers and manifests as
# impossible failures (a std::bad_alloc inside an unrelated stage was the
# symptom that motivated adding this).
# main.o is listed explicitly as a prerequisite of the pipeline binary, and its
# dependency file must be included with the rest: leaving it out means no header
# edit ever rebuilds main.cpp, and a header that changes the layout of Context
# then links a stale main against fresh stages.  That is not a missing rebuild,
# it is a segmentation fault at startup.
-include $(ENGINE_OBJ:.o=.d) $(PIPE_OBJ:.o=.d) $(MAIN_OBJ:.o=.d)

$(PIPELINE_BIN): $(PIPE_OBJ) $(ENGINE_OBJ) $(MAIN_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(SIM_BIN): $(ENGINE_OBJ) $(OBJ)/engine/app/sim_main.o
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_BIN): $(TEST_SRC) $(ENGINE_OBJ) $(PIPE_OBJ)
	@mkdir -p $(BIN)
	@echo "  LD   $@"
	@$(CXX) $(CXXFLAGS) $(TEST_SRC) $(ENGINE_OBJ) $(PIPE_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

# --- Workflows --------------------------------------------------------------
test: $(TEST_BIN)
	@$(TEST_BIN)

world: $(PIPELINE_BIN)
	@$(PIPELINE_BIN) --config configs/world.json
# Pin the published payload.  The build is byte-reproducible (verified: two
# runs produce identical hashes), so a committed checksum is meaningful, and it
# is what catches a binary that was mangled outside the build — see
# tools/verify_artifacts.sh.
	@cd viewer/data && sha256sum mumbai.bin mumbai.json > SHA256SUMS
	@echo "  artifacts  viewer/data/SHA256SUMS"

# Acceptance-scale run: 60 simulated minutes along Colaba -> Borivali at 60 Hz
# with a 620-vehicle fleet — the Section 7 continuity requirement.
simulate: $(SIM_BIN)
	@$(SIM_BIN) --minutes 60 --vehicles 620

# Independent verification of the published artifacts: the unit suite, the
# runtime validation run, the viewer payload re-derived from its own offsets by
# the Node verifier, and the integrity of the raw source data.
all-checks: test simulate
	@node tests/verify_export.mjs
	@bash tools/verify_artifacts.sh
	@bash tools/fetch_data.sh --verify-only
	@echo "== all checks complete =="

stats: $(PIPELINE_BIN)
	@$(PIPELINE_BIN) --report data/processed/world_manifest.json

verify-artifacts:
	@bash tools/verify_artifacts.sh

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
