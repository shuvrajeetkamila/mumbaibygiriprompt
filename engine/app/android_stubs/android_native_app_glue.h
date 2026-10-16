// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android_native_app_glue.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule.
//
//  The glue library is the platform's own loop for a NativeActivity: it owns the
//  thread, hides the message queue, and turns the lifecycle into APP_CMD_* commands
//  with a poll source.  This shim declares that contract — the command values, the
//  two handler signatures, the poll source, and app_dummy() — so that the client's
//  lifecycle code is type-checked on the host.
//
//  android_app IS NARROWED HERE.  The NDK's struct also carries the message pipe,
//  the ALooper, the saved state and the pending-window fields; the client uses
//  userData, activity, window, onAppCmd, onInputEvent and destroyRequested and
//  nothing else.  Narrowing a struct the client only reads is safe; changing the
//  type of a member it writes would not be, and none is changed.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_NATIVE_APP_GLUE_H
#define MUMBAI_ANDROID_STUBS_NATIVE_APP_GLUE_H

#include <stdint.h>

#include <android/input.h>
#include <android/looper.h>
#include <android/native_activity.h>
#include <android/native_window.h>

#ifdef __cplusplus
extern "C" {
#endif

struct android_app;
struct android_poll_source;

typedef void (*android_app_cmd_handler)(struct android_app* app, int32_t cmd);
typedef int32_t (*android_app_input_handler)(struct android_app* app, AInputEvent* event);

// The lifecycle commands, with the platform's values.  The client's onCommand()
// switches on these; a shim that renumbered them would hide exactly the class of
// mistake this header exists to catch.
enum {
    APP_CMD_INPUT_CHANGED = 0x0001,
    APP_CMD_INIT_WINDOW = 0x0002,
    APP_CMD_TERM_WINDOW = 0x0003,
    APP_CMD_WINDOW_RESIZED = 0x0004,
    APP_CMD_WINDOW_REDRAW_NEEDED = 0x0005,
    APP_CMD_CONTENT_RECT_CHANGED = 0x0006,
    APP_CMD_GAINED_FOCUS = 0x0007,
    APP_CMD_LOST_FOCUS = 0x0008,
    APP_CMD_CONFIG_CHANGED = 0x0009,
    APP_CMD_LOW_MEMORY = 0x000a,
    APP_CMD_START = 0x000b,
    APP_CMD_RESUME = 0x000c,
    APP_CMD_SAVE_STATE = 0x000d,
    APP_CMD_PAUSE = 0x000e,
    APP_CMD_STOP = 0x000f,
    APP_CMD_DESTROY = 0x0010,
};

struct android_poll_source {
    int32_t id;
    struct android_app* app;
    void (*process)(struct android_app* app, struct android_poll_source* source);
};

struct android_app {
    void* userData;
    ANativeActivity* activity;
    ANativeWindow* window;
    void (*onAppCmd)(struct android_app* app, int32_t cmd);
    int32_t (*onInputEvent)(struct android_app* app, AInputEvent* event);
    int32_t destroyRequested;
};

// The glue library must be kept: app_dummy() is the reference that stops the
// linker from dropping android_native_app_glue.o, which is what defines
// ANativeActivity_onCreate — the symbol the platform actually calls.
void app_dummy(void);

void android_app_write_cmd(struct android_app* android_app, int8_t cmd);
void android_app_set_input(struct android_app* android_app, AInputQueue* inputQueue);
void android_app_set_window(struct android_app* android_app, ANativeWindow* window);
void android_app_set_activity_state(struct android_app* android_app, int8_t state);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_NATIVE_APP_GLUE_H
