# =============================================================================
#  MUMBAI 1:1 EXACT — android/CMakeLists.txt
#
#  The Android build of the engine: one shared library, `libmumbai.so`, holding
#  the core, the world reader, the simulation, the audio engine, the physics
#  engine (Jolt), the Vulkan renderer and the NativeActivity entry point.
#
#  TWO THINGS ABOUT THIS FILE THAT ARE DELIBERATE
#
#  1. **The source lists are globs, and the Jolt list is not.**  The engine's own
#     directories are globbed so that a new `.cpp` is built without an edit here —
#     the Makefile does the same, and a file that is in one build and not the other
#     is a class of bug this project has already paid for.  Jolt is *vendored* and
#     comes with its own list (`third_party/jolt_sources.mk`, transcribed from
#     upstream's CMake in D-033), so it is parsed from that file rather than
#     re-globbed: globbing Jolt would start compiling the debug renderer and the
#     compute backends the list deliberately excludes.
#
#  2. **It configures on a host.**  With no NDK this file builds a small
#     executable instead of a shared library, so the shader compilation, the
#     embedding step and the source lists are all verified on a machine without
#     Android — which is the only kind of machine this project's author has.
# =============================================================================
cmake_minimum_required(VERSION 3.22)
project(mumbai LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

get_filename_component(MUMBAI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

option(MUMBAI_PHYSICS "Build the Jolt-backed physics engine" ON)

# --- sources -----------------------------------------------------------------
file(GLOB MUMBAI_CORE_SRC  ${MUMBAI_ROOT}/engine/core/*.cpp)
file(GLOB MUMBAI_WORLD_SRC ${MUMBAI_ROOT}/engine/world/*.cpp)
file(GLOB MUMBAI_SIM_SRC   ${MUMBAI_ROOT}/engine/sim/*.cpp)
file(GLOB MUMBAI_AUDIO_SRC ${MUMBAI_ROOT}/engine/audio/*.cpp)
file(GLOB MUMBAI_RENDER_SRC ${MUMBAI_ROOT}/engine/render/*.cpp)

set(MUMBAI_SOURCES ${MUMBAI_CORE_SRC} ${MUMBAI_WORLD_SRC} ${MUMBAI_SIM_SRC} ${MUMBAI_AUDIO_SRC}
                   ${MUMBAI_RENDER_SRC})

if(MUMBAI_PHYSICS)
    file(GLOB MUMBAI_PHYSICS_SRC ${MUMBAI_ROOT}/engine/physics/*.cpp)
    # Jolt's list, from the hand-written build of D-033: read the `JOLT_SRC :=`
    # assignment, join its line continuations, and keep the `Jolt/*.cpp` entries.
    file(READ ${MUMBAI_ROOT}/third_party/jolt_sources.mk MUMBAI_JOLT_MK)
    string(REGEX REPLACE "\\\\\n" " " MUMBAI_JOLT_MK "${MUMBAI_JOLT_MK}")
    string(REGEX MATCH "JOLT_SRC[ \t]*:=[^\n]*" MUMBAI_JOLT_LINE "${MUMBAI_JOLT_MK}")
    if(NOT MUMBAI_JOLT_LINE)
        message(FATAL_ERROR "third_party/jolt_sources.mk has no JOLT_SRC assignment")
    endif()
    string(REPLACE "JOLT_SRC" "" MUMBAI_JOLT_LINE "${MUMBAI_JOLT_LINE}")
    string(REPLACE ":=" "" MUMBAI_JOLT_LINE "${MUMBAI_JOLT_LINE}")
    string(REPLACE "\t" " " MUMBAI_JOLT_LINE "${MUMBAI_JOLT_LINE}")
    string(REPLACE " " ";" MUMBAI_JOLT_LIST "${MUMBAI_JOLT_LINE}")
    set(MUMBAI_JOLT_SRC "")
    foreach(entry ${MUMBAI_JOLT_LIST})
        if(entry MATCHES "\\.cpp$")
            list(APPEND MUMBAI_JOLT_SRC ${MUMBAI_ROOT}/third_party/${entry})
        endif()
    endforeach()
    list(LENGTH MUMBAI_JOLT_SRC MUMBAI_JOLT_COUNT)
    if(MUMBAI_JOLT_COUNT LESS 100)
        message(FATAL_ERROR "third_party/jolt_sources.mk yielded only ${MUMBAI_JOLT_COUNT} "
                            "translation units; the parse is broken, not the list")
    endif()
    message(STATUS "Jolt: ${MUMBAI_JOLT_COUNT} translation units")
    list(APPEND MUMBAI_SOURCES ${MUMBAI_PHYSICS_SRC} ${MUMBAI_JOLT_SRC})
endif()

# --- shaders -----------------------------------------------------------------
# SPIR-V is compiled at build time and embedded in the binary (tools/embed_spirv.py),
# so a device that can start the engine can always find its shaders.  A machine with
# no GLSL compiler gets the placeholders and a warning from the same tools the
# Makefile uses; the failure mode is a pipeline that says "shader missing", which is
# a better failure than a silent black screen.
set(MUMBAI_SHADER_DIR ${MUMBAI_ROOT}/engine/render/shaders)
set(MUMBAI_SPV_DIR ${CMAKE_BINARY_DIR}/spv)
set(MUMBAI_SHADER_HEADER ${CMAKE_BINARY_DIR}/shaders_generated.h)
file(GLOB MUMBAI_SHADER_SRC ${MUMBAI_SHADER_DIR}/*.vert ${MUMBAI_SHADER_DIR}/*.frag
     ${MUMBAI_SHADER_DIR}/*.glsl)

add_custom_command(
    OUTPUT ${MUMBAI_SHADER_HEADER}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${MUMBAI_SPV_DIR}
    COMMAND python3 ${MUMBAI_ROOT}/tools/compile_shaders.py ${MUMBAI_SHADER_DIR} ${MUMBAI_SPV_DIR}
    COMMAND python3 ${MUMBAI_ROOT}/tools/embed_spirv.py ${MUMBAI_SHADER_HEADER} ${MUMBAI_SPV_DIR}
    DEPENDS ${MUMBAI_SHADER_SRC} ${MUMBAI_ROOT}/tools/compile_shaders.py
            ${MUMBAI_ROOT}/tools/embed_spirv.py
    COMMENT "  SPIR-V  compiling and embedding the shaders"
    VERBATIM)
add_custom_target(mumbai_shaders DEPENDS ${MUMBAI_SHADER_HEADER})

# --- the client --------------------------------------------------------------
if(ANDROID)
    add_library(mumbai SHARED
        ${MUMBAI_ROOT}/engine/app/android_main.cpp
        ${MUMBAI_SHADER_HEADER}
        ${MUMBAI_SOURCES})

    target_include_directories(mumbai PRIVATE
        ${MUMBAI_ROOT}
        ${MUMBAI_ROOT}/third_party/vulkan/include
        ${CMAKE_BINARY_DIR})

    # The Android surface is created from an ANativeWindow; without this the
    # renderer compiles against the desktop path and fails at the first frame.
    target_compile_definitions(mumbai PRIVATE VK_USE_PLATFORM_ANDROID_KHR)
    if(MUMBAI_PHYSICS)
        # Jolt on ARM: NEON, and the same feature set the host build uses so that a
        # scene baked on one machine behaves the same on the other.
        target_compile_definitions(mumbai PRIVATE JPH_USE_NEON)
    endif()

    target_link_libraries(mumbai PRIVATE vulkan android log)

    # android_native_app_glue ships with the NDK, as source.
    add_library(native_app_glue STATIC
        ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)
    target_include_directories(native_app_glue PUBLIC
        ${ANDROID_NDK}/sources/android/native_app_glue)
    target_link_libraries(mumbai PRIVATE native_app_glue)

    # Nothing in this library is a public API: no symbol needs to be exported, and
    # hidden visibility lets the linker drop what the app cannot reach.
    set_target_properties(mumbai PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
        # `-u ANativeActivity_onCreate` keeps the entry point the glue library
        # provides: without it the linker discards the object that defines it.
        LINK_FLAGS "-u ANativeActivity_onCreate")
else()
    message(STATUS "Not an Android build: verifying the source lists and shaders only.")
    add_executable(mumbai_cmake_hostcheck
        ${MUMBAI_ROOT}/engine/app/host_check.cpp
        ${MUMBAI_CORE_SRC} ${MUMBAI_WORLD_SRC} ${MUMBAI_SIM_SRC} ${MUMBAI_AUDIO_SRC})
    target_include_directories(mumbai_cmake_hostcheck PRIVATE ${MUMBAI_ROOT} ${CMAKE_BINARY_DIR})
    target_link_libraries(mumbai_cmake_hostcheck PRIVATE pthread)
    add_dependencies(mumbai_cmake_hostcheck mumbai_shaders)
endif()

if(TARGET mumbai)
    add_dependencies(mumbai mumbai_shaders)
endif()
