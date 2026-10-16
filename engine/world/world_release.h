// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/world_release.h
//
//  WHICH WORLD THIS BUILD IS LOOKING FOR.
//
//  The APK carries South Bombay inside itself (D-047) and the other 899 cells are a
//  download from the `world-data` release.  A download has to name four things
//  exactly — the repository, the archive, the checksum list, and the sha256 of the
//  world manifest the build was made for — and every one of them is a fact about
//  `data/processed/world_manifest.json`, which does not exist until `make world`
//  has run.  So they are not source code: the workflow writes them into
//  `world_release.generated.h` beside this file, after the world is built and
//  before Gradle is invoked, and this header is what every other build sees.
//
//  THE ARCHIVE IS AN UNCOMPRESSED TAR (D-049).  The client streams it and resumes
//  inside it, and a pause can only be remembered at a tar entry boundary: in a
//  compressed stream the byte after that boundary is somewhere inside a frame, and
//  finding it would mean decompressing from zero on every resume.
//
//  THE FALLBACK IS EMPTY STRINGS, ON PURPOSE.  A bare `make`, a test run, a
//  checkout with no world: `known()` is false, and the client says on its panel
//  that this build does not know of a published world — instead of inventing a URL
//  and reporting a network error that means nothing.  A build that *does* ship a
//  world download says so in one place, and D-048 records why the identity of a
//  world is its manifest hash rather than a version number.
//
//  `__has_include` rather than a `-D` from the build system, because the same line
//  then holds in the Makefile's host check, in CMake under the NDK, and in an
//  editor that is only reading the file.  The include is quoted, so it resolves
//  beside this header first and needs no include path of its own.
// =============================================================================
#pragma once

#if defined(__has_include)
#if __has_include("world_release.generated.h")
#include "world_release.generated.h"
#endif
#endif

// The repository the release lives in, as `owner/name`.
#ifndef MUMBAI_WORLD_REPOSITORY
#define MUMBAI_WORLD_REPOSITORY ""
#endif
// The release tag; a fixed name, because the assets inside it are versioned.
#ifndef MUMBAI_WORLD_TAG
#define MUMBAI_WORLD_TAG "world-data"
#endif
// `mumbai-world-<manifest sha256:16>.tar`
#ifndef MUMBAI_WORLD_ASSET
#define MUMBAI_WORLD_ASSET ""
#endif
// `sha256sums-<manifest sha256:16>.txt`, the list the downloader verifies against.
#ifndef MUMBAI_WORLD_CHECKSUMS
#define MUMBAI_WORLD_CHECKSUMS ""
#endif
// The sha256 of `world_manifest.json`, first 16 hex digits, lowercase.
#ifndef MUMBAI_WORLD_MANIFEST_SHA16
#define MUMBAI_WORLD_MANIFEST_SHA16 ""
#endif

namespace mumbai::world {

// One place where "we do not know of a published world" is decided.  The client
// asks `known()` before it offers a download, and the panel prints the reason when
// the answer is no.
struct WorldRelease {
    const char* repository;
    const char* tag;
    const char* asset;
    const char* checksums;
    const char* manifestSha16;

    bool known() const {
        return repository[0] != '\0' && tag[0] != '\0' && asset[0] != '\0' &&
               checksums[0] != '\0' && manifestSha16[0] != '\0';
    }
};

inline WorldRelease publishedWorldRelease() {
    return WorldRelease{MUMBAI_WORLD_REPOSITORY, MUMBAI_WORLD_TAG, MUMBAI_WORLD_ASSET,
                        MUMBAI_WORLD_CHECKSUMS, MUMBAI_WORLD_MANIFEST_SHA16};
}

}  // namespace mumbai::world
