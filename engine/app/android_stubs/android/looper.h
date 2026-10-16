// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/looper.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule.
//  The client's whole main loop is `ALooper_pollOnce` (blocking until the surface
//  arrives, then non-blocking), so the signature of that one call — including the
//  `void** outData` that receives the poll source — is the part that matters.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_LOOPER_H
#define MUMBAI_ANDROID_STUBS_LOOPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALOOPER_POLL_WAKE (-1)
#define ALOOPER_POLL_CALLBACK (-2)
#define ALOOPER_POLL_TIMEOUT (-3)
#define ALOOPER_POLL_ERROR (-4)

#define ALOOPER_EVENT_INPUT 0x00000001

struct ALooper;
typedef struct ALooper ALooper;

ALooper* ALooper_prepare(int opts);
ALooper* ALooper_forThread(void);
void ALooper_acquire(ALooper* looper);
void ALooper_release(ALooper* looper);
int ALooper_pollOnce(int timeoutMillis, int* outFd, int* outEvents, void** outData);
int ALooper_pollAll(int timeoutMillis, int* outFd, int* outEvents, void** outData);
void ALooper_wake(ALooper* looper);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_LOOPER_H
