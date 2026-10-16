# =============================================================================
#  MUMBAI 1:1 EXACT — android/CMakeLists.txt
#
#  Builds the Android client: `libmumbai.so`, which is the whole engine — core,
#  world reader, simulation, audio, physics (Jolt) and the Vulkan renderer — plus
#  the NativeActivity entry point in engine/app/android_main.cpp.
#
#  Three things here are decisions rather than plumbing:
#
#  1. **The Vulkan headers are the vendored ones** (third_party/vulkan, SDK
#     1.3.290), not the NDK's.  The desktop build compiles against the same
#     headers, so a Vulkan API mistake is a compile error on whichever machine is
#     closest, and the header set cannot drift between the two builds.
#
#  2. **Jolt's source list is parsed from third_party/jolt_sources.mk**, the same
#     file the Makefile builds from (D-033).  One list, two build systems: a file
#     added to one cannot silently vanish from the other.
#
#  3. **The shaders are compiled and embedded at build time**, exactly as the
#     Makefile does.  SPIR-V that is not in the binary is SPIR-V that can go
#     missing from an APK's assets.
#
#  The same file configures a host build (`-DMUMBAI_HOST_CHECK=ON`) with no
#  Android toolchain, no Vulkan loader and (possibly) no shader compiler, which
#  is how a machine without an Android SDK still checks that this build file is
#  not broken and that every source it names exists and links.
# =============================================================================
cmake_minimum_required(VERSION 3.22)
# C as well as CXX: the NDK's native_app_glue is a C file, and a project that
# enables only C++ cannot determine a linker language for it ("CMake Error: CMake
# can not determine linker language for target: native_app_glue" — which is
# exactly how the first Android build failed).
project(mumbai C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUMBAI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
option(MUMBAI_PHYSICS "Build Jolt and engine/physics into the client" ON)
option(MUMBAI_HOST_CHECK "Configure for a host compile check (no Android, no Vulkan)" OFF)

# Defined before use: CMake executes commands as it reads them, and a function
# called above its definition is "Unknown CMake command", not a forward
# declaration.
function(mumbai_add_shaders target)
  # GENERATED TRUE is what makes CMake order the compile jobs after the generator:
  # a file listed in a target's sources but not marked generated is assumed to
  # exist already, and the first build then tries to compile a .cpp that includes
  # a header nobody has written yet.
  set_source_files_properties("${GENERATED_DIR}/shaders_generated.h"
                              PROPERTIES GENERATED TRUE)
  add_dependencies(${target} mumbai_shaders)
  target_sources(${target} PRIVATE "${GENERATED_DIR}/shaders_generated.h")
endfunction()

# -----------------------------------------------------------------------------
#  Shaders: compiled to SPIR-V, then embedded as C++ arrays
# -----------------------------------------------------------------------------
set(SHADER_DIR "${MUMBAI_ROOT}/engine/render/shaders")
set(SPV_DIR "${CMAKE_BINARY_DIR}/spv")
set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(GLOB MUMBAI_SHADER_SRC "${SHADER_DIR}/*.vert" "${SHADER_DIR}/*.frag" "${SHADER_DIR}/*.glsl")

# One command does the whole thing: compile every stage to SPIR-V, then embed the
# modules as C++ arrays.  Both halves report what they did, and compile_shaders.py
# fails the build on a shader error rather than leaving last week's module behind.
add_custom_command(
  OUTPUT "${GENERATED_DIR}/shaders_generated.h"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_DIR}"
  COMMAND python3 "${MUMBAI_ROOT}/tools/compile_shaders.py" "${SHADER_DIR}" "${SPV_DIR}"
  COMMAND python3 "${MUMBAI_ROOT}/tools/embed_spirv.py"
          "${GENERATED_DIR}/shaders_generated.h" "${SPV_DIR}"
  DEPENDS ${MUMBAI_SHADER_SRC}
          "${MUMBAI_ROOT}/tools/compile_shaders.py"
          "${MUMBAI_ROOT}/tools/embed_spirv.py"
  COMMENT "  SHADERS engine/render/shaders -> shaders_generated.h"
  VERBATIM)

add_custom_target(mumbai_shaders DEPENDS "${GENERATED_DIR}/shaders_generated.h")

# -----------------------------------------------------------------------------
#  Sources
# -----------------------------------------------------------------------------
file(GLOB MUMBAI_CORE_SRC "${MUMBAI_ROOT}/engine/core/*.cpp")
file(GLOB MUMBAI_WORLD_SRC "${MUMBAI_ROOT}/engine/world/*.cpp")
file(GLOB MUMBAI_SIM_SRC "${MUMBAI_ROOT}/engine/sim/*.cpp")
file(GLOB MUMBAI_AUDIO_SRC "${MUMBAI_ROOT}/engine/audio/*.cpp")
file(GLOB MUMBAI_PHYS_SRC "${MUMBAI_ROOT}/engine/physics/*.cpp")
file(GLOB MUMBAI_RENDER_SRC "${MUMBAI_ROOT}/engine/render/*.cpp")

set(MUMBAI_ENGINE_SRC ${MUMBAI_CORE_SRC} ${MUMBAI_WORLD_SRC} ${MUMBAI_SIM_SRC}
    ${MUMBAI_AUDIO_SRC} ${MUMBAI_RENDER_SRC})
if(MUMBAI_PHYSICS)
  list(APPEND MUMBAI_ENGINE_SRC ${MUMBAI_PHYS_SRC})
endif()

# Jolt: `JOLT_SRC := path \` continuation lines.  The continuations are joined
# first and the list parsed second, because a regex that tries to do both at once
# is greedy in exactly the wrong place: `[^\n]*` swallows the backslash that the
# continuation rule needs, the match ends at the first line, and the build then
# reports "2 sources" for a 122-file library.  (Which is what it did.)
if(MUMBAI_PHYSICS)
  file(READ "${MUMBAI_ROOT}/third_party/jolt_sources.mk" JOLT_MK)
  string(REPLACE "\\\n" " " JOLT_JOINED "${JOLT_MK}")
  string(REGEX MATCH "JOLT_SRC[ \t]*:=[^\n]*" JOLT_LINE "${JOLT_JOINED}")
  if(NOT JOLT_LINE)
    message(FATAL_ERROR "third_party/jolt_sources.mk: no JOLT_SRC assignment found")
  endif()
  string(REGEX REPLACE "^JOLT_SRC[ \t]*:=[ \t]*" "" JOLT_LIST "${JOLT_LINE}")
  string(REGEX REPLACE "[ \t]+" ";" JOLT_SRC "${JOLT_LIST}")
  # The list is relative to third_party/ (that is how the Makefile reads it);
  # CMake resolves relative paths against the source dir of *this* file.
  set(JOLT_ABS "")
  foreach(jolt_src IN LISTS JOLT_SRC)
    list(APPEND JOLT_ABS "${MUMBAI_ROOT}/third_party/${jolt_src}")
  endforeach()
  set(JOLT_SRC "${JOLT_ABS}")
  list(LENGTH JOLT_SRC JOLT_COUNT)
  if(JOLT_COUNT LESS 100)
    message(FATAL_ERROR "Jolt: parsed only ${JOLT_COUNT} sources; the list parse is wrong")
  endif()
  message(STATUS "Jolt: ${JOLT_COUNT} translation units from third_party/jolt_sources.mk")
endif()

# Jolt's architecture defines have to reach both Jolt's own sources and the
# engine sources that include its headers: a mismatch there is an ABI error that
# compiles cleanly and corrupts at runtime, not a build error.
set(MUMBAI_JOLT_DEFINES "")
set(MUMBAI_JOLT_FLAGS "")
if(ANDROID)
  if(ANDROID_ABI STREQUAL "armeabi-v7a")
    set(MUMBAI_JOLT_DEFINES JPH_USE_NEON)
    set(MUMBAI_JOLT_FLAGS -mfpu=neon)
  elseif(ANDROID_ABI STREQUAL "arm64-v8a")
    set(MUMBAI_JOLT_DEFINES JPH_USE_NEON)
  elseif(ANDROID_ABI MATCHES "^x86")
    set(MUMBAI_JOLT_DEFINES JPH_USE_SSE4_1 JPH_USE_SSE4_2)
    set(MUMBAI_JOLT_FLAGS -msse4.2 -mpopcnt)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(MUMBAI_JOLT_DEFINES JPH_USE_SSE4_1 JPH_USE_SSE4_2)
  set(MUMBAI_JOLT_FLAGS -msse4.2 -mpopcnt)
endif()

if(MUMBAI_PHYSICS)
  add_library(jolt_objects STATIC ${JOLT_SRC})
  target_include_directories(jolt_objects PUBLIC "${MUMBAI_ROOT}/third_party")
  target_compile_options(jolt_objects PRIVATE
    -fno-rtti -fno-exceptions -w -pthread ${MUMBAI_JOLT_FLAGS})
  target_compile_definitions(jolt_objects PRIVATE ${MUMBAI_JOLT_DEFINES})
endif()

# -----------------------------------------------------------------------------
#  Android client
# -----------------------------------------------------------------------------
if(ANDROID)
  add_library(native_app_glue STATIC
    "${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c")
  target_include_directories(native_app_glue PUBLIC
    "${ANDROID_NDK}/sources/android/native_app_glue")

  add_library(mumbai SHARED
    "${MUMBAI_ROOT}/engine/app/android_main.cpp"
    ${MUMBAI_ENGINE_SRC})

  target_include_directories(mumbai PRIVATE
    "${MUMBAI_ROOT}"
    "${MUMBAI_ROOT}/third_party/vulkan/include"
    "${MUMBAI_ROOT}/third_party"
    "${GENERATED_DIR}")

  # VK_USE_PLATFORM_ANDROID_KHR is what makes vulkan.h declare
  # VkAndroidSurfaceCreateInfoKHR; the renderer's surface path is compiled from
  # this define, not from a separate Android fork of the file.
  target_compile_definitions(mumbai PRIVATE VK_USE_PLATFORM_ANDROID_KHR ${MUMBAI_JOLT_DEFINES})
  target_compile_options(mumbai PRIVATE ${MUMBAI_JOLT_FLAGS})

  mumbai_add_shaders(mumbai)

  target_link_libraries(mumbai PRIVATE native_app_glue vulkan android log ${CMAKE_DL_LIBS})
  if(MUMBAI_PHYSICS)
    target_link_libraries(mumbai PRIVATE jolt_objects)
  endif()

  # NativeActivity is entered through ANativeActivity_onCreate, which lives in the
  # glue library; without -u the linker drops it and the app dies before main.
  target_link_options(mumbai PRIVATE "-u ANativeActivity_onCreate")

  set_target_properties(mumbai PROPERTIES CXX_VISIBILITY_PRESET hidden)

# -----------------------------------------------------------------------------
#  Host compile check: the same source lists minus Vulkan, no Android toolchain
# -----------------------------------------------------------------------------
else()
  if(NOT MUMBAI_HOST_CHECK)
    message(FATAL_ERROR
      "This build file targets Android.  Configure with -DMUMBAI_HOST_CHECK=ON to "
      "check the source lists and the shader pipeline on a machine without an "
      "Android toolchain (see the `host-check` target in the Makefile).")
  endif()
  # The renderer is deliberately out of the host link: it references the Vulkan
  # loader, and a machine with no loader cannot link it.  `make render` covers the
  # renderer's own compilation.
  set(MUMBAI_HOST_SRC ${MUMBAI_CORE_SRC} ${MUMBAI_WORLD_SRC} ${MUMBAI_SIM_SRC}
      ${MUMBAI_AUDIO_SRC})
  if(MUMBAI_PHYSICS)
    list(APPEND MUMBAI_HOST_SRC ${MUMBAI_PHYS_SRC})
  endif()

  # android_main.cpp is compiled on the host too, as an object library that is
  # never linked (its !__ANDROID__ branch is a main() of its own).  Without this,
  # the only machine that ever saw the file compile was the CI runner — which is
  # how "unknown type name 'u32'" got as far as a red build.  An OBJECT library
  # builds it and links nothing.
  add_library(mumbai_android_main_compilecheck OBJECT
    "${MUMBAI_ROOT}/engine/app/android_main.cpp")
  target_include_directories(mumbai_android_main_compilecheck PRIVATE
    "${MUMBAI_ROOT}" "${MUMBAI_ROOT}/third_party" "${GENERATED_DIR}")
  mumbai_add_shaders(mumbai_android_main_compilecheck)

  add_executable(mumbai_hostcheck
    "${MUMBAI_ROOT}/engine/app/host_check.cpp"
    ${MUMBAI_HOST_SRC})
  target_include_directories(mumbai_hostcheck PRIVATE
    "${MUMBAI_ROOT}" "${MUMBAI_ROOT}/third_party" "${GENERATED_DIR}")
  mumbai_add_shaders(mumbai_hostcheck)
  target_compile_definitions(mumbai_hostcheck PRIVATE ${MUMBAI_JOLT_DEFINES})
  target_compile_options(mumbai_hostcheck PRIVATE -pthread ${MUMBAI_JOLT_FLAGS})
  target_link_libraries(mumbai_hostcheck PRIVATE ${CMAKE_DL_LIBS})
  if(MUMBAI_PHYSICS)
    target_link_libraries(mumbai_hostcheck PRIVATE jolt_objects)
  endif()
endif()
