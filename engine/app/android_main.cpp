// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_main.cpp
//
//  The Android client's entry point: a NativeActivity with no Java in the loop.
//
//  WHY NO JAVA.  The app's whole job is to own a surface and feed it frames; the
//  platform's own `android.app.NativeActivity` does exactly that, and the glue
//  library (android_native_app_glue) turns the lifecycle into commands on a
//  looper.  A Java activity would add a Gradle/AndroidX dependency tree, a second
//  language between the debugger and the crash, and nothing else.  The manifest
//  declares `android:hasCode="false"`.
//
//  THE LIFECYCLE, AND WHY IT IS WRITTEN OUT LONGHAND.  On Android the window is
//  not a resource the app asks for once: it arrives (APP_CMD_INIT_WINDOW), it is
//  taken away when the app goes to the background (APP_CMD_TERM_WINDOW), and it
//  comes back.  Every one of those is a place a naive renderer keeps a
//  VkSurfaceKHR that is no longer valid, which is the "works until you press home
//  and come back" class of crash.  The renderer is therefore created on
//  INIT_WINDOW, destroyed on TERM_WINDOW, and every frame is skipped while there
//  is no surface.
//
//  WHAT THE FIRST BUILD DOES.  It starts the engine, creates the Vulkan device
//  and swapchain on the device's own surface, builds the pipelines, and draws the
//  sky and the tonemap every frame at the display's refresh rate, logging its
//  stats once a second.  The world is *not* in the APK — 1.3 GB of chunks is a
//  first-run download — so there is no city yet; the app says so on its log, with
//  the path the world belongs in, rather than showing a black screen and no
//  explanation.  Everything it logs is the same engine the desktop harness runs.
//
//  Compiling this file on a machine without the NDK is deliberate: the Android
//  parts are behind `__ANDROID__`, and the `#else` branch is a real main() that
//  explains itself, so a host build catches the typos that would otherwise be
//  found only by a CI run.
// =============================================================================
#include "engine/core/core.h"
#include "engine/render/renderer.h"

#include <cstdarg>
#include <cstdio>
#include <string>

using namespace mumbai;

#if defined(__ANDROID__)
#include <android/log.h>
#include <android/native_window.h>
#include <android_native_app_glue.h>
#include <time.h>
#endif

namespace {

// The engine's own logging goes to stdout/stderr, which Android discards for an
// app with no Java.  core.cpp routes MLOG_* to logcat under __ANDROID__; this is
// the app-level equivalent for the lines that must be visible even before the
// engine's logger is configured.
void appLog(const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
#if defined(__ANDROID__)
    __android_log_print(ANDROID_LOG_INFO, "mumbai", "%s", buffer);
#else
    std::printf("[mumbai] %s\n", buffer);
#endif
}

struct Client {
    render::Renderer renderer;
    bool ready = false;
    bool haveWorld = false;
    std::string dataDir;
    u32 width = 0, height = 0;
    u64 frames = 0;
    u64 lastStatMs = 0;
    u64 nextRetryMs = 0;      // a failed start is retried once a second, not per frame
    std::string lastError;
};

u64 nowMillis() {
#if defined(__ANDROID__)
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<u64>(ts.tv_sec) * 1000ull + static_cast<u64>(ts.tv_nsec) / 1000000ull;
#else
    return 0;
#endif
}

}  // namespace

#if defined(__ANDROID__)

namespace {

void startRenderer(android_app* app, Client* client) {
    if (client->ready || !app->window) return;
    client->width = static_cast<u32>(ANativeWindow_getWidth(app->window));
    client->height = static_cast<u32>(ANativeWindow_getHeight(app->window));

    std::string error;
    if (!client->renderer.init(app->window, client->width, client->height, client->dataDir, &error)) {
        // Not fatal: the next INIT_WINDOW (or the next frame) may succeed, and a
        // renderer that gives up on the first failure gives up on devices that
        // hand the surface over late, which is most of them.
        client->lastError = error;
        appLog("renderer did not start: %s", error.c_str());
        return;
    }

    render::Environment env;                 // golden hour, monsoon humidity
    client->renderer.setEnvironment(env);
    render::Camera camera;
    camera.pos = Vec3{0.0f, 60.0f, 0.0f};
    camera.yaw = 0.0f;
    camera.pitch = -0.08f;
    client->renderer.setCamera(camera);
    client->ready = true;
    appLog("renderer started: %ux%u, data dir %s", client->width, client->height,
           client->dataDir.c_str());
}

void stopRenderer(Client* client) {
    if (!client->ready) return;
    client->renderer.shutdown();
    client->ready = false;
    appLog("renderer stopped (surface withdrawn)");
}

void onCommand(android_app* app, int32_t cmd) {
    auto* client = static_cast<Client*>(app->userData);
    switch (cmd) {
        case APP_CMD_INIT_WINDOW:
            startRenderer(app, client);
            break;
        case APP_CMD_TERM_WINDOW:
            stopRenderer(client);
            break;
        case APP_CMD_WINDOW_RESIZED:
        case APP_CMD_CONFIG_CHANGED:
            client->width = static_cast<u32>(ANativeWindow_getWidth(app->window));
            client->height = static_cast<u32>(ANativeWindow_getHeight(app->window));
            if (client->ready) client->renderer.resize(client->width, client->height);
            client->renderer.setEnvironment(render::Environment{});
            break;
        case APP_CMD_LOW_MEMORY:
            appLog("low memory: %u chunks resident", client->renderer.chunkCount());
            break;
        case APP_CMD_GAINED_FOCUS:
            appLog("focus gained");
            break;
        case APP_CMD_LOST_FOCUS:
            appLog("focus lost");
            break;
        case APP_CMD_DESTROY:
            stopRenderer(client);
            break;
        default:
            break;
    }
}

}  // namespace

void android_main(android_app* app) {
    // The glue library must not be stripped: it is the only reference to
    // ANativeActivity_onCreate, which is how the platform enters this code.
    app_dummy();

    Client client;
    app->userData = &client;
    app->onAppCmd = onCommand;

    if (app->activity && app->activity->internalDataPath) {
        client.dataDir = std::string(app->activity->internalDataPath) + "/world";
    } else {
        client.dataDir = "world";
    }
    // MLOG_* is routed to logcat by core.cpp on Android; the engine's own
    // timestamped trail and this client's lines land in the same place.
    logSetLevel(LogLevel::Info);
    appLog("MUMBAI 1:1 EXACT — client starting");
    appLog("world data dir: %s (no world in the APK: it is a first-run download)",
           client.dataDir.c_str());

    const std::string manifest = client.dataDir + "/world_manifest.json";
    client.haveWorld = !readFileToString(manifest).empty();
    appLog("world manifest: %s", client.haveWorld ? "found" : "absent — the sky pass is all this "
                                                     "build can draw until the world is downloaded");

    while (true) {
        int events = 0;
        android_poll_source* source = nullptr;
        // Block while there is nothing to draw; with a surface, poll with a zero
        // timeout so the loop runs at the display's rate instead of being woken
        // by input.
        const int timeout = client.ready ? 0 : -1;
        while (ALooper_pollOnce(timeout, nullptr, &events,
                               reinterpret_cast<void**>(&source)) >= 0) {
            if (source) source->process(app, source);
            if (app->destroyRequested) {
                stopRenderer(&client);
                appLog("%llu frames rendered", static_cast<unsigned long long>(client.frames));
                return;
            }
        }

        if (!client.ready) {
            // The platform may hand the surface over late, or the first device
            // created may have been one that could not present.  One retry a
            // second, so a device that recovers recovers, and one that cannot
            // does not spin.
            const u64 now = nowMillis();
            if (app->window && now >= client.nextRetryMs) {
                client.nextRetryMs = now + 1000;
                startRenderer(app, &client);
            }
            continue;
        }

        std::string error;
        if (!client.renderer.drawFrame(&error)) {
            // A lost swapchain is not a failure: the next frame recreates it.  A
            // device loss would be, and it would say so every frame.
            if (error != client.lastError) {
                client.lastError = error;
                appLog("frame dropped: %s", error.c_str());
            }
            continue;
        }
        client.lastError.clear();
        ++client.frames;

        const u64 now = nowMillis();
        if (client.lastStatMs == 0) client.lastStatMs = now;
        if (now - client.lastStatMs >= 1000) {
            const render::FrameStats& st = client.renderer.stats();
            const f64 seconds = static_cast<f64>(now - client.lastStatMs) / 1000.0;
            appLog("%llu frames, %.1f fps, %u draws, %u triangles, %u instances, "
                   "%u chunks, %.1f MiB GPU, %ux%u",
                   static_cast<unsigned long long>(client.frames),
                   static_cast<f64>(client.frames) / seconds, st.drawCalls, st.triangles,
                   st.instances, client.renderer.chunkCount(),
                   static_cast<f64>(st.gpuBytes) / (1024.0 * 1024.0),
                   st.renderWidth, st.renderHeight);
            client.lastStatMs = now;
            client.frames = 0;
        }
    }
}

#else   // !__ANDROID__

// A host build of this file exists so that its typos are found on a machine
// without an NDK.  It is not the Android client and does not pretend to be.
int main() {
    std::printf("engine/app/android_main.cpp is the Android entry point.\n");
    std::printf("Configure the app build with the NDK toolchain to produce libmumbai.so;\n");
    std::printf("on a host, `mumbai_render` (make render) is the renderer's harness.\n");
    return 0;
}

#endif
