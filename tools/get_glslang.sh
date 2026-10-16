#!/usr/bin/env bash
# =============================================================================
#  tools/get_glslang.sh — a GLSL compiler for machines that have none
#
#  The build wants to compile the eleven shaders in engine/render/shaders.  glslc
#  ships with the Vulkan SDK; Ubuntu has glslangValidator in `glslang-tools`; a
#  machine with neither would otherwise never compile a shader until CI did.
#
#  This fetches the glslang source (from codeload, which is reachable when the
#  release-asset CDN is not), builds glslangValidator with cmake+ninja, and
#  leaves it in build/tools/glslang/, which is derived and untracked.  It is a
#  one-time two-minute cost that turns "the shaders have never been compiled"
#  into "the shaders compile", which is the difference between a renderer and a
#  pile of GLSL.
#
#  Usage:  bash tools/get_glslang.sh && export PATH="$PWD/build/tools/glslang:$PATH"
# =============================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="${ROOT}/build/tools"
OUT="${WORK}/glslang"
VERSION="${GLSLANG_VERSION:-16.5.0}"
SPIRV_VERSION="${SPIRV_HEADERS_VERSION:-vulkan-sdk-1.3.290.0}"

if command -v glslc >/dev/null 2>&1; then echo "glslc already present: $(command -v glslc)"; exit 0; fi
if command -v glslangValidator >/dev/null 2>&1; then
  echo "glslangValidator already present: $(command -v glslangValidator)"; exit 0
fi
if [ -x "${OUT}/glslangValidator" ]; then echo "already built: ${OUT}/glslangValidator"; exit 0; fi

for tool in cmake ninja curl tar; do
  command -v "$tool" >/dev/null 2>&1 || { echo "need $tool on PATH" >&2; exit 1; }
done

mkdir -p "${WORK}"
cd "${WORK}"
echo "== fetching glslang ${VERSION} and SPIR-V headers ${SPIRV_VERSION}"
curl -sL -o glslang.tar.gz "https://codeload.github.com/KhronosGroup/glslang/tar.gz/refs/tags/${VERSION}"
curl -sL -o spirv-headers.tar.gz "https://codeload.github.com/KhronosGroup/SPIRV-Headers/tar.gz/refs/tags/${SPIRV_VERSION}"
rm -rf "glslang-${VERSION}" "SPIRV-Headers-${SPIRV_VERSION}"
tar xzf glslang.tar.gz
tar xzf spirv-headers.tar.gz

echo "== building glslangValidator (a few minutes)"
cd "glslang-${VERSION}"
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DENABLE_OPT=OFF \
      -DENABLE_HLSL=OFF -DBUILD_EXTERNAL=OFF -DCMAKE_CXX_FLAGS="-w" >/dev/null
ninja -C build glslang-standalone >/dev/null
mkdir -p "${OUT}"
cp build/StandAlone/glslang "${OUT}/glslangValidator"
echo "== built ${OUT}/glslangValidator"
"${OUT}/glslangValidator" --version
