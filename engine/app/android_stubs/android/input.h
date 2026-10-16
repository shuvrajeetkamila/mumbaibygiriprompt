// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/input.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule:
//  names, types and argument order are the NDK's (r26); this file is a compile
//  surface, not an implementation.  The motion constants in particular are the
//  platform's values — AMOTION_EVENT_ACTION_POINTER_INDEX_MASK is what the client
//  shifts to find out which finger a multi-touch event belongs to, and a shim that
//  guessed it would make the touch code pass a check it should fail.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_INPUT_H
#define MUMBAI_ANDROID_STUBS_INPUT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque on purpose, exactly as in the NDK: the client must go through the
// accessors, and it does.
struct AInputEvent;
typedef struct AInputEvent AInputEvent;

// A struct the glue library manipulates (android_app_set_input) and the client
// never touches; declared here because <android/input.h> is where the NDK puts it.
struct AInputQueue;
typedef struct AInputQueue AInputQueue;

// AInputEvent_getType
enum {
    AINPUT_EVENT_TYPE_KEY = 1,
    AINPUT_EVENT_TYPE_MOTION = 2,
};

// AKeyEvent_getAction
enum {
    AKEY_EVENT_ACTION_DOWN = 0,
    AKEY_EVENT_ACTION_UP = 1,
    AKEY_EVENT_ACTION_MULTIPLE = 2,
};

// AKeyEvent_getKeyCode (the one key the client cares about)
enum {
    AKEYCODE_BACK = 4,
};

// AMotionEvent_getAction, and the two halves of the packed value it returns
enum {
    AMOTION_EVENT_ACTION_MASK = 0xff,
    AMOTION_EVENT_ACTION_POINTER_INDEX_MASK = 0xff00,
    AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT = 8,
    AMOTION_EVENT_ACTION_DOWN = 0,
    AMOTION_EVENT_ACTION_UP = 1,
    AMOTION_EVENT_ACTION_MOVE = 2,
    AMOTION_EVENT_ACTION_CANCEL = 3,
    AMOTION_EVENT_ACTION_OUTSIDE = 4,
    AMOTION_EVENT_ACTION_POINTER_DOWN = 5,
    AMOTION_EVENT_ACTION_POINTER_UP = 6,
};

// AMotionEvent_getMetaState / AKeyEvent_getMetaState bits the client may ask about
enum {
    AMETA_NONE = 0x00000000,
};

int32_t AInputEvent_getType(const AInputEvent* event);
int32_t AInputEvent_getDeviceId(const AInputEvent* event);
int32_t AInputEvent_getSource(const AInputEvent* event);

int32_t AKeyEvent_getAction(const AInputEvent* key_event);
int32_t AKeyEvent_getFlags(const AInputEvent* key_event);
int32_t AKeyEvent_getKeyCode(const AInputEvent* key_event);
int32_t AKeyEvent_getMetaState(const AInputEvent* key_event);

int32_t AMotionEvent_getAction(const AInputEvent* motion_event);
int32_t AMotionEvent_getMetaState(const AInputEvent* motion_event);
size_t AMotionEvent_getPointerCount(const AInputEvent* motion_event);
int32_t AMotionEvent_getPointerId(const AInputEvent* motion_event, size_t pointer_index);
float AMotionEvent_getX(const AInputEvent* motion_event, size_t pointer_index);
float AMotionEvent_getY(const AInputEvent* motion_event, size_t pointer_index);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_INPUT_H
