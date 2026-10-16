// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/asset_manager.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for what the
//  stubs are for and the rule they follow: names, types and argument order come
//  from the NDK (r26) verbatim; the implementation and the unused surface do not.
//
//  The client uses exactly five entry points here — open, getLength, read, close,
//  and the mode flag — which is why the shim is small.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_ASSET_MANAGER_H
#define MUMBAI_ANDROID_STUBS_ASSET_MANAGER_H

#include <stddef.h>
#include <sys/types.h>   // off_t, as the NDK's own header requires

#include <android/log.h>   // no-op here; mirrors the NDK's include chain shape

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AAssetManager AAssetManager;
typedef struct AAssetDir AAssetDir;
typedef struct AAsset AAsset;

// The access mode passed to AAssetManager_open.  The client reads whole files, so
// it asks for AASSET_MODE_BUFFER (1) — the values are the platform's.
enum {
    AASSET_MODE_UNKNOWN = 0,
    AASSET_MODE_BUFFER = 1,
    AASSET_MODE_STREAMING = 2,
    AASSET_MODE_RANDOM = 3,
};

AAssetDir* AAssetManager_openDir(AAssetManager* mgr, const char* dirName);
AAsset* AAssetManager_open(AAssetManager* mgr, const char* filename, int mode);

const char* AAssetDir_getNextFileName(AAssetDir* assetDir);
void AAssetDir_rewind(AAssetDir* assetDir);
void AAssetDir_close(AAssetDir* assetDir);

const void* AAsset_getBuffer(AAsset* asset);
off_t AAsset_getLength(AAsset* asset);
off_t AAsset_getRemainingLength(AAsset* asset);
int AAsset_read(AAsset* asset, void* buf, size_t count);
off_t AAsset_seek(AAsset* asset, off_t offset, int whence);
void AAsset_close(AAsset* asset);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_ASSET_MANAGER_H
