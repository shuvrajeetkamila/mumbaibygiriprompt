// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_main.cpp
//
//  The Android client: a NativeActivity with no Java in the loop.
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
//  WHAT THIS BUILD DOES.  It starts the engine, brings up Vulkan, installs the
//  starter world out of the APK's own assets on first run (the whole 1.3 GB city
//  is a download, but a slice of it travels inside the package so that a fresh
//  install shows Mumbai rather than an explanation), streams the cells around the
//  camera, flies the camera with touch, and draws a status panel over the frame.
//
//  WHY THE PANEL IS HERE AND NOT LATER.  The first APK this project produced drew
//  a black screen on a real phone, and that was the entire diagnostic channel: no
//  frame counter, no reason, no "the world is missing".  The panel (engine/render/
//  hud.h) is the answer to that, and it is written before the city because a
//  client that cannot report itself cannot be debugged by anyone holding it.
//
//  Compiling this file on a machine without the NDK is deliberate: the Android
//  parts are behind `__ANDROID__`, and the `#else` branch is a real main() that
//  explains itself, so a host build catches the typos that would otherwise be
//  found only by a CI run.
// =============================================================================
#include "engine/core/core.h"
#include "engine/app/android_http.h"
#include "engine/app/fallback_screen.h"
#include "engine/core/geo.h"     // MACRO_CELL_SIZE: the floating origin of section 4A
#include "engine/render/hud.h"
#include "engine/render/renderer.h"
#include "engine/render/world_index.h"
#include "engine/world/world_download.h"
#include "engine/world/world_release.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace mumbai;

#if defined(__ANDROID__)
#include <android/asset_manager.h>
#include <android/input.h>
#include <android/log.h>
#include <android/native_window.h>
#include <android_native_app_glue.h>
#include <sys/stat.h>
#include <time.h>
#endif

// THE NAMESPACE OPENS INSIDE THE ANDROID BRANCH, and the braces are why.  It used to
// open before it, with the closing brace at the end of the client body — which is
// inside `#if defined(__ANDROID__)`.  A host compile (no NDK, no `-D__ANDROID__`)
// therefore had an anonymous namespace that never closed, and the `#else` branch's
// own main() was unreachable: "expected '}' at end of input".  Nothing caught it
// because the Makefile's app check compiles this file *with* `__ANDROID__`; the
// CMake host check, which compiles every source the APK compiles, found it.
#if defined(__ANDROID__)

namespace {

// The engine's own logging goes to stdout/stderr, which Android discards for an
// app with no Java.  core.cpp routes MLOG_* to logcat under __ANDROID__; this is
// the app-level equivalent for lines that must be visible even before the engine's
// logger is configured, and on a host it is a printf.
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

// -----------------------------------------------------------------------------
//  Where the camera starts: the Gateway of India, looking north up the harbour.
//  (World metres, from the pipeline's own anchors — the same numbers the tests
//  use.  E 6 878.8 / N 5 693.2, 60 m up.)
// -----------------------------------------------------------------------------
constexpr f64 kStartWorldX = -6878.8;
constexpr f64 kStartWorldZ = 5693.2;
constexpr f32 kStartAltitude = 60.0f;
constexpr f32 kFlySpeed = 45.0f;          // metres per second at full stick
constexpr f32 kLookPerPixel = 0.0035f;    // radians per pixel of drag
constexpr f32 kStreamRadius = 3000.0f;    // metres of world kept resident
constexpr u32 kStreamPerCall = 3;         // cells per streaming tick
constexpr u64 kStreamIntervalMs = 500;
constexpr u64 kInstallBytesPerTick = 4u << 20;
// The city download's per-frame budget.  A step fetches until the budget is spent
// and then stops, so this is what keeps the download from being the frame loop:
// half a megabyte is a fraction of a second on a phone's network, and the state
// machine continues mid-file on the next frame (world_download.h).
constexpr u64 kDownloadBytesPerTick = 512u << 10;

// -----------------------------------------------------------------------------
//  Input
// -----------------------------------------------------------------------------
struct Input {
    bool stickActive = false;
    i32 stickPointer = -1;
    f32 stickOriginX = 0, stickOriginY = 0;
    f32 stickX = 0, stickY = 0;           // -1..1, y is forward

    bool lookActive = false;
    i32 lookPointer = -1;
    f32 lookLastX = 0, lookLastY = 0;

    bool ascend = false, descend = false;
    u64 buttonTapMs = 0;
    i32 buttonPointer = -1;
};

// The panel's buttons, in the same pixel space the HUD draws in.  One definition,
// read by both the input handler and the HUD: a button whose hit test and whose
// drawing are written out twice is a button that stops working the day one of them
// is edited (the two altitude buttons and the mode button used to be literals in
// two places; the city button is what made that worth fixing).
struct Rect {
    f32 x = 0, y = 0, w = 0, h = 0;
    bool has(f32 px, f32 py) const {
        return px > x && px < x + w && py > y && py < y + h;
    }
};

Rect flyButton(f32 width) { return {width - 260.0f, 20.0f, 240.0f, 70.0f}; }
Rect upButton(f32 width) { return {width - 260.0f, 110.0f, 110.0f, 70.0f}; }
Rect downButton(f32 width) { return {width - 140.0f, 110.0f, 120.0f, 70.0f}; }
Rect cityButton(f32 width, f32 height) {
    (void)width;
    return {20.0f, height - 90.0f, 210.0f, 70.0f};
}

struct Client {
    render::Renderer renderer;
    bool ready = false;
    bool haveWorld = false;
    std::string dataDir;
    u32 width = 0, height = 0;
    u64 frames = 0;
    u64 lastStatMs = 0;
    u64 nextRetryMs = 0;
    u64 nextFallbackMs = 0;
    std::string lastError;

    // camera, in f64 world metres with a f32 render offset: the floating origin of
    // section 4, done the way the specification asks (int64 cell + local offset).
    f64 worldX = kStartWorldX, worldZ = kStartWorldZ;
    f32 altitude = kStartAltitude;
    f32 yaw = 0.0f, pitch = -0.08f;

    Input input;

    // The starter world travels inside the APK; first run copies it out.
    struct Installer {
        bool active = false;
        bool done = false;
        std::vector<std::string> files;
        size_t next = 0;
        u64 bytesTotal = 0, bytesDone = 0;
        std::string error;
    } installer;

    u64 lastStreamMs = 0;
    render::WorldLoadStats lastLoad;
    u32 resident = 0;
    f32 fps = 0.0f;

    // THE REST OF THE CITY, ON REQUEST (D-048).  The APK carries South Bombay; the
    // other 899 cells are a 1.3 GB download, and nothing starts it except a person
    // tapping the button, because it is somebody's data plan.  `downloadNote` is the
    // sentence the panel prints when there is nothing to offer: a build with no
    // published world says so, rather than reporting a network error that means
    // nothing.
    world::Fetcher* fetcher = nullptr;
    world::DownloadConfig downloadConfig;
    std::unique_ptr<world::WorldDownload> download;
    bool downloadWanted = false;
    bool downloadApplied = false;
    std::string downloadNote;
};

u64 nowMillis() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<u64>(ts.tv_sec) * 1000ull + static_cast<u64>(ts.tv_nsec) / 1000000ull;
}

bool readAsset(AAssetManager* manager, const std::string& path, std::string* out) {
    AAsset* asset = AAssetManager_open(manager, path.c_str(), AASSET_MODE_BUFFER);
    if (!asset) return false;
    const off_t size = AAsset_getLength(asset);
    out->resize(static_cast<size_t>(size));
    const int read = AAsset_read(asset, out->data(), static_cast<size_t>(size));
    AAsset_close(asset);
    if (read != size) {
        out->clear();
        return false;
    }
    return true;
}

bool makeDirsFor(const std::string& filePath) {
    // The APK lays its world out flat (cells/ and textures/ one level down), so
    // creating the one intermediate directory is enough — and a mkdir that fails
    // because it already exists is not an error.
    const size_t slash = filePath.find_last_of('/');
    if (slash == std::string::npos) return true;
    const std::string dir = filePath.substr(0, slash);
    if (dir.empty()) return true;
    ::mkdir(dir.c_str(), 0770);
    return true;
}

// THE CITY SUPERSEDES THE SLICE, and this is the file that says so.  The APK packs
// South Bombay so that a fresh install shows something (D-047); a person who has
// downloaded the whole city does not want an app update to copy 96 MiB of the slice
// back over it, and — worse — the slice carries its own 25-cell
// `world_manifest.json`, which would replace the full one and hide 899 cells that
// are sitting on the disk.  The downloader writes this marker when it finishes, and
// the installer defers to it.  A partial download leaves no marker, so the slice is
// still installed exactly as before.
constexpr const char* kCityCompleteMarker = "city_complete.txt";

bool cityIsComplete(const std::string& dataDir) {
    std::FILE* file = std::fopen((dataDir + "/" + kCityCompleteMarker).c_str(), "rb");
    if (!file) return false;
    std::fclose(file);
    return true;
}

bool markCityComplete(const std::string& dataDir, const std::string& note) {
    std::FILE* file = std::fopen((dataDir + "/" + kCityCompleteMarker).c_str(), "wb");
    if (!file) return false;
    const int written = std::fprintf(file, "%s\n", note.c_str());
    std::fclose(file);
    return written > 0;
}

// The index is a list of files to copy, written by the packaging tool.  Its first
// line is a version token: when it changes, the installed copy is refreshed, which
// is what makes this survive an app update that ships a newer slice.
//
// The total is measured here, not accumulated as the copy runs.  It used to be
// `bytesTotal = max(bytesTotal, bytesDone)`, which reads 100% from the first frame:
// a progress bar that is full before any work has happened is worse than none,
// because it says the install is finished while it has not started.
bool beginInstall(AAssetManager* manager, Client* client) {
    if (cityIsComplete(client->dataDir)) {
        client->installer.done = true;
        appLog("the whole city is installed; the APK's own slice is not copied over it");
        return true;
    }
    std::string index;
    if (!readAsset(manager, "world/INDEX.txt", &index)) {
        return false;   // no slice in this build: the renderer draws sky and says so
    }
    Client::Installer& inst = client->installer;
    inst.files.clear();
    inst.next = 0;
    inst.bytesDone = 0;
    inst.bytesTotal = 0;
    std::string version;
    size_t start = 0;
    while (start < index.size()) {
        size_t end = index.find('\n', start);
        if (end == std::string::npos) end = index.size();
        std::string line = index.substr(start, end - start);
        start = end + 1;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (version.empty()) version = line;
            continue;
        }
        inst.files.push_back(line);
    }
    if (inst.files.empty()) return false;

    // One open per listed file, to measure what the slice weighs.  A slice is a few
    // dozen files, and this runs once, on the frame the client starts.
    for (const std::string& rel : inst.files) {
        AAsset* asset = AAssetManager_open(manager, ("world/" + rel).c_str(), AASSET_MODE_BUFFER);
        if (!asset) continue;
        inst.bytesTotal += static_cast<u64>(AAsset_getLength(asset));
        AAsset_close(asset);
    }

    const std::string markerPath = client->dataDir + "/installed.txt";
    std::string marker;
    if (FILE* f = std::fopen(markerPath.c_str(), "rb")) {
        char buffer[256];
        const size_t n = std::fread(buffer, 1, sizeof(buffer) - 1, f);
        std::fclose(f);
        buffer[n] = 0;
        marker = buffer;
    }
    if (!marker.empty()) {
        // Compare the version token, not the whole index: the token is what the
        // packaging tool changes when the slice changes.
        const size_t newline = marker.find('\n');
        if (marker.substr(0, newline) == version) {
            inst.done = true;
            appLog("starter world already installed (%s)", version.c_str());
            return true;
        }
    }
    inst.active = true;
    appLog("installing the starter world: %zu files from the APK into %s", inst.files.size(),
           client->dataDir.c_str());
    return true;
}

// Copies a bounded amount per call so that the first frames arrive immediately and
// the panel can report progress.  A first run that blocks for ten seconds on a
// black screen is exactly what this project is trying not to ship.
u32 installStep(AAssetManager* manager, Client* client) {
    Client::Installer& inst = client->installer;
    if (!inst.active) return 0;
    u64 copied = 0;
    u32 files = 0;
    while (inst.next < inst.files.size() && copied < kInstallBytesPerTick) {
        const std::string& rel = inst.files[inst.next];
        const std::string dest = client->dataDir + "/" + rel;
        makeDirsFor(dest);
        AAsset* asset = AAssetManager_open(manager, ("world/" + rel).c_str(), AASSET_MODE_BUFFER);
        if (!asset) {
            inst.error = "missing in the APK: " + rel;
            appLog("install: %s", inst.error.c_str());
            inst.active = false;
            break;
        }
        const off_t size = AAsset_getLength(asset);
        std::vector<char> buffer(static_cast<size_t>(size));
        const int got = AAsset_read(asset, buffer.data(), static_cast<size_t>(size));
        AAsset_close(asset);
        if (got != size) {
            inst.error = "short read: " + rel;
            inst.active = false;
            break;
        }
        if (FILE* f = std::fopen(dest.c_str(), "wb")) {
            const size_t written = std::fwrite(buffer.data(), 1, static_cast<size_t>(size), f);
            std::fclose(f);
            if (written != static_cast<size_t>(size)) {
                inst.error = "short write: " + rel;
                inst.active = false;
                break;
            }
        } else {
            inst.error = "cannot write " + dest;
            inst.active = false;
            break;
        }
        copied += static_cast<u64>(size);
        inst.bytesDone += static_cast<u64>(size);
        // The measured total is a floor, not a promise: if a file in the APK is
        // larger than the measurement saw, progress stays at 100% instead of
        // reporting more than everything.
        if (inst.bytesDone > inst.bytesTotal) inst.bytesTotal = inst.bytesDone;
        ++inst.next;
        ++files;
    }
    if (inst.active && inst.next >= inst.files.size()) {
        inst.active = false;
        inst.done = true;
        // The version token is written last: an interrupted install is retried.
        std::string index;
        std::string version = "starter-world";
        if (readAsset(manager, "world/INDEX.txt", &index)) {
            const size_t newline = index.find('\n');
            version = index.substr(0, newline == std::string::npos ? index.size() : newline);
        }
        if (FILE* f = std::fopen((client->dataDir + "/installed.txt").c_str(), "wb")) {
            std::fprintf(f, "%s\n%s\n", version.c_str(), "installed by the client");
            std::fclose(f);
        }
        appLog("starter world installed: %zu files, %.1f MiB",
               inst.files.size(), static_cast<f64>(inst.bytesDone) / (1024.0 * 1024.0));
    }
    return files;
}

// -----------------------------------------------------------------------------
//  Starting and stopping the renderer
// -----------------------------------------------------------------------------
void startRenderer(android_app* app, Client* client) {
    if (client->ready || !app->window) return;
    client->width = static_cast<u32>(ANativeWindow_getWidth(app->window));
    client->height = static_cast<u32>(ANativeWindow_getHeight(app->window));

    std::string error;
    if (!client->renderer.init(app->window, client->width, client->height, client->dataDir, &error)) {
        // Not fatal: the next INIT_WINDOW (or the next frame) may succeed, and a
        // renderer that gives up on the first failure gives up on devices that hand
        // the surface over late, which is most of them.
        client->lastError = error;
        appLog("renderer did not start: %s", error.c_str());
        return;
    }

    render::Environment env;                 // golden hour, monsoon humidity
    client->renderer.setEnvironment(env);
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

// -----------------------------------------------------------------------------
//  Touch
// -----------------------------------------------------------------------------
i32 onInput(android_app* app, AInputEvent* event) {
    auto* client = static_cast<Client*>(app->userData);
    if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_KEY) {
        const i32 code = AKeyEvent_getKeyCode(event);
        if (code == AKEYCODE_BACK && AKeyEvent_getAction(event) == AKEY_EVENT_ACTION_UP) {
            ANativeActivity_finish(app->activity);
            return 1;
        }
        return 0;
    }
    if (AInputEvent_getType(event) != AINPUT_EVENT_TYPE_MOTION) return 0;

    const i32 action = AMotionEvent_getAction(event) & AMOTION_EVENT_ACTION_MASK;
    const size_t index = static_cast<size_t>(AMotionEvent_getAction(event) &
                                             AMOTION_EVENT_ACTION_POINTER_INDEX_MASK) >>
                         AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;
    const f32 x = AMotionEvent_getX(event, index);
    const f32 y = AMotionEvent_getY(event, index);
    const i32 id = AMotionEvent_getPointerId(event, index);
    const f32 w = static_cast<f32>(client->width);
    const f32 h = static_cast<f32>(client->height);
    Input& input = client->input;

    // The panel's buttons, in the same pixel space the HUD draws in: the rectangles
    // come from the shared definitions above, so a button cannot be drawn in one
    // place and hit-tested in another.
    const bool inFlyButton = flyButton(w).has(x, y);
    const bool inUpButton = upButton(w).has(x, y);
    const bool inDownButton = downButton(w).has(x, y);
    const bool inCityButton = cityButton(w, h).has(x, y);

    switch (action) {
        case AMOTION_EVENT_ACTION_DOWN:
        case AMOTION_EVENT_ACTION_POINTER_DOWN:
            if (inFlyButton) {
                client->altitude = client->altitude < 5.0f ? 60.0f : -client->altitude;
                // A negative altitude is a dive: the app has no ground query yet, so
                // the two altitude buttons are the honest way to go up and down.
                if (client->altitude < 0.0f) client->altitude = 12.0f;
            } else if (inUpButton) {
                input.ascend = true;
            } else if (inDownButton) {
                input.descend = true;
            } else if (inCityButton) {
                // The one thing in this app that costs somebody money, so it takes a
                // deliberate tap in both directions: the same button stops it again.
                if (client->download) client->downloadWanted = !client->downloadWanted;
                appLog("city download %s", client->downloadWanted ? "started" : "paused");
            } else if (x < w * 0.5f) {
                input.stickActive = true;
                input.stickPointer = id;
                input.stickOriginX = x;
                input.stickOriginY = y;
                input.stickX = input.stickY = 0.0f;
            } else {
                input.lookActive = true;
                input.lookPointer = id;
                input.lookLastX = x;
                input.lookLastY = y;
            }
            return 1;

        case AMOTION_EVENT_ACTION_MOVE:
            if (input.stickActive && input.lookPointer != id) {
                // Any pointer moving while the stick is held keeps the stick alive:
                // the finger that owns the stick is the one whose id matches.
            }
            for (size_t i = 0; i < AMotionEvent_getPointerCount(event); ++i) {
                const i32 pid = AMotionEvent_getPointerId(event, i);
                const f32 px = AMotionEvent_getX(event, i);
                const f32 py = AMotionEvent_getY(event, i);
                if (input.stickActive && pid == input.stickPointer) {
                    const f32 dx = px - input.stickOriginX;
                    const f32 dy = py - input.stickOriginY;
                    const f32 limit = 140.0f;
                    input.stickX = std::max(-1.0f, std::min(1.0f, dx / limit));
                    input.stickY = std::max(-1.0f, std::min(1.0f, -dy / limit));
                } else if (input.lookActive && pid == input.lookPointer) {
                    client->yaw -= (px - input.lookLastX) * kLookPerPixel;
                    client->pitch = std::max(-1.45f, std::min(1.45f,
                                            client->pitch - (py - input.lookLastY) * kLookPerPixel));
                    input.lookLastX = px;
                    input.lookLastY = py;
                }
            }
            // A horizontal drag on the upper half of the screen also turns: it is
            // the gesture a person tries first, and it costs nothing to accept.
            if (!input.lookActive && !input.stickActive && AMotionEvent_getPointerCount(event) == 1) {
                static f32 lastX = x;
                client->yaw -= (x - lastX) * kLookPerPixel;
                lastX = x;
            }
            return 1;

        case AMOTION_EVENT_ACTION_UP:
        case AMOTION_EVENT_ACTION_POINTER_UP:
        case AMOTION_EVENT_ACTION_CANCEL:
            if (input.stickActive && (id == input.stickPointer || action == AMOTION_EVENT_ACTION_CANCEL)) {
                input.stickActive = false;
                input.stickPointer = -1;
                input.stickX = input.stickY = 0.0f;
            }
            if (input.lookActive && (id == input.lookPointer || action == AMOTION_EVENT_ACTION_CANCEL)) {
                input.lookActive = false;
                input.lookPointer = -1;
            }
            if (action == AMOTION_EVENT_ACTION_UP || action == AMOTION_EVENT_ACTION_CANCEL) {
                input.ascend = input.descend = false;
                (void)h;
            }
            return 1;
        default:
            return 0;
    }
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
            client->width = static_cast<u32>(ANativeWindow_getWidth(app->window));
            client->height = static_cast<u32>(ANativeWindow_getHeight(app->window));
            if (client->ready) client->renderer.resize(client->width, client->height);
            break;
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

// -----------------------------------------------------------------------------
//  The panel: what the phone can tell a person without a cable
// -----------------------------------------------------------------------------
std::string format(const char* fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    return std::string(buffer);
}

// WHAT THE CLIENT SAYS ABOUT ITSELF, in one place.  The GPU panel and the CPU
// fallback screen (engine/app/fallback_screen.h) draw the same sentences at
// different times: the panel whenever there is a frame, the fallback while there
// is not.  Two lists would drift, and the list is the client's only voice.
std::vector<std::string> statusLines(Client* client) {
    std::vector<std::string> lines;
    if (client->ready) {
        const render::FrameStats& st = client->renderer.stats();
        lines.push_back(format("%.0f FPS   %u draws   %u triangles", client->fps, st.drawCalls,
                               st.triangles));
        lines.push_back(format("%u chunks resident   %.0f MiB GPU",
                               client->renderer.chunkCount(),
                               static_cast<f64>(st.gpuBytes) / (1024.0 * 1024.0)));
    } else {
        lines.push_back("renderer not started");
        if (!client->lastError.empty()) lines.push_back(client->lastError.substr(0, 46));
    }
    lines.push_back(format("E %.0f m   N %.0f m   alt %.0f m", client->worldX, client->worldZ,
                           client->altitude));
    if (client->installer.active) {
        const f64 pct = client->installer.bytesTotal > 0
                            ? 100.0 * static_cast<f64>(client->installer.bytesDone) /
                                  static_cast<f64>(client->installer.bytesTotal)
                            : 0.0;
        lines.push_back(format("installing starter world  %.0f%%   %.1f MiB", pct,
                               static_cast<f64>(client->installer.bytesDone) / (1024.0 * 1024.0)));
    } else if (!client->installer.error.empty()) {
        lines.push_back("install: " + client->installer.error.substr(0, 42));
    } else if (!client->installer.done) {
        lines.push_back("no starter world in this build");
    } else if (!client->haveWorld) {
        lines.push_back("world index empty");
    } else {
        lines.push_back(format("world %u cells indexed, %u in radius %.0f m",
                               client->renderer.worldCellCount(),
                               client->lastLoad.considered, kStreamRadius));
    }
    if (!client->installer.active && client->installer.done && client->ready) {
        const render::WorldLoadStats& load = client->lastLoad;
        lines.push_back(format("streaming: %u loaded, %u resident, %u failed", load.loaded,
                               client->renderer.chunkCount(), load.failed));
        if (!load.firstError.empty()) lines.push_back("chunk: " + load.firstError.substr(0, 42));
    }
    if (client->download) {
        lines.push_back(client->download->status());
    } else if (!client->downloadNote.empty()) {
        lines.push_back(client->downloadNote);
    }
    lines.push_back("left half: move    right half: look");
    return lines;
}

void updateHud(Client* client) {
    render::HudFrame frame;
    const f32 w = static_cast<f32>(client->width);
    const f32 scale = w >= 1400.0f ? 3.0f : (w >= 900.0f ? 2.5f : 2.0f);

    const std::vector<std::string> lines = statusLines(client);
    f32 y = 14.0f;
    f32 panelWidth = 0.0f;
    for (const std::string& text : lines)
        panelWidth = std::max(panelWidth, render::hudTextWidth(text, scale));
    frame.rects.clear();
    frame.rects.push_back({8.0f, 8.0f, panelWidth + 24.0f,
                           static_cast<f32>(lines.size()) * render::hudLineHeight(scale) + 20.0f,
                           render::kHudPanel});
    for (const std::string& text : lines) {
        const bool bad = text.rfind("FAIL", 0) == 0 || text.find("failed") != std::string::npos ||
                         text.rfind("!!", 0) == 0;
        const u32 colour = bad ? render::kHudRed : render::kHudWhite;
        frame.lines.push_back({20.0f, y, scale, colour, text});
        y += render::hudLineHeight(scale);
    }

    // The buttons, drawn where the input handler looks for them: the same rectangles.
    const f32 h = static_cast<f32>(client->height);
    const Rect fly = flyButton(w);
    const Rect up = upButton(w);
    const Rect down = downButton(w);
    const Rect city = cityButton(w, h);
    frame.rects.push_back({fly.x, fly.y, fly.w, fly.h, render::kHudPanel});
    frame.lines.push_back({fly.x + 10.0f, fly.y + 14.0f, scale, render::kHudGreen, "FLY"});
    frame.rects.push_back({up.x, up.y, up.w, up.h, render::kHudPanel});
    frame.lines.push_back({up.x + 10.0f, up.y + 14.0f, scale, render::kHudWhite, "UP"});
    frame.rects.push_back({down.x, down.y, down.w, down.h, render::kHudPanel});
    frame.lines.push_back({down.x + 10.0f, down.y + 14.0f, scale, render::kHudWhite, "DOWN"});
    // The city button, which is only a button when there is a city to fetch.
    frame.rects.push_back({city.x, city.y, city.w, city.h,
                           client->download && client->downloadWanted ? render::kHudGreen
                                                                     : render::kHudPanel});
    const u32 cityColour = client->download ? render::kHudWhite : render::kHudPanel;
    const char* cityLabel = "GET CITY";
    if (client->download) {
        const world::WorldDownload::State state = client->download->state();
        if (state == world::WorldDownload::State::Done) cityLabel = "CITY OK";
        else if (client->downloadWanted) cityLabel = "PAUSE CITY";
    }
    frame.lines.push_back({city.x + 10.0f, city.y + 14.0f, scale, cityColour, cityLabel});

    // The stick, drawn where the finger is: a HUD that hides the control teaches
    // nothing about which half of the screen does what.
    if (client->input.stickActive) {
        const f32 cx = client->input.stickOriginX;
        const f32 cy = client->input.stickOriginY;
        frame.rects.push_back({cx - 70.0f, cy - 70.0f, 140.0f, 140.0f, render::kHudPanel});
        frame.rects.push_back({cx - 8.0f + client->input.stickX * 55.0f,
                               cy - 8.0f - client->input.stickY * 55.0f, 16.0f, 16.0f,
                               render::kHudGreen});
    }
    client->renderer.setHud(frame);
}

// -----------------------------------------------------------------------------
//  The screen that cannot be black
// -----------------------------------------------------------------------------
// While the renderer is not up there is no frame to put a panel in, and the phone
// shows whatever the window had — which is how "nothing here" is what a person
// reports.  This paints the same status lines straight into the window buffer on
// the CPU, at two hertz, and never touches the window once Vulkan owns it.
void updateFallbackScreen(android_app* app, Client* client, u64 now) {
    if (client->ready || !app->window) return;
    if (now < client->nextFallbackMs) return;
    client->nextFallbackMs = now + 500;

    std::vector<std::string> lines;
    lines.push_back("MUMBAI 1:1 EXACT");
    for (const std::string& line : statusLines(client)) lines.push_back(line);
    if (!client->lastError.empty()) lines.push_back("!! " + client->lastError.substr(0, 52));
    lines.push_back("waiting for a surface and a graphics device");

    if (!app::drawFallbackScreen(app->window, lines.front(), lines)) {
        // Nothing left to draw on, and saying so once a second is enough.
        appLog("fallback screen could not be painted (no lockable window buffer)");
    }
}

// -----------------------------------------------------------------------------
//  The frame
// -----------------------------------------------------------------------------
void updateCamera(Client* client, f32 dt) {
    Input& input = client->input;
    const f32 speed = kFlySpeed * (input.lookActive ? 2.0f : 1.0f);
    const f32 forward = input.stickY * speed * dt;
    const f32 strafe = input.stickX * speed * dt;
    // yaw 0 faces north (+Z), and yaw grows to the east: the world convention.
    const f32 sinYaw = std::sin(client->yaw), cosYaw = std::cos(client->yaw);
    client->worldX += static_cast<f64>(forward * sinYaw + strafe * cosYaw);
    client->worldZ += static_cast<f64>(forward * cosYaw - strafe * sinYaw);
    if (input.ascend) client->altitude += 25.0f * dt;
    if (input.descend) client->altitude = std::max(2.0f, client->altitude - 25.0f * dt);
    if (input.stickActive && input.stickY != 0.0f) {
        // Moving forward also flies along the pitch, so a person can dive at the
        // harbour wall and climb over the Sea Link with the same stick.
        client->altitude += std::sin(client->pitch) * 60.0f * dt;
        client->altitude = std::max(2.0f, client->altitude);
    }
}

void renderFrame(android_app* app, Client* client, f32 dt) {
    // Floating origin: the render origin is the camera's own macro-cell, so the
    // camera's render-space coordinates never leave [0, 1024) and a 45 km world
    // renders with millimetre precision (section 4, D-042).
    const f64 cellMetres = static_cast<f64>(MACRO_CELL_SIZE);
    const f64 cellX = std::floor(client->worldX / cellMetres);
    const f64 cellZ = std::floor(client->worldZ / cellMetres);
    client->renderer.setRenderOrigin(cellX * cellMetres, cellZ * cellMetres);

    render::Camera camera;
    // The subtraction is done in f64 and only the result is narrowed: the camera's
    // render-space position is a function of the offset from its own cell origin,
    // and rounding the operands first is how a floating origin loses its point.
    camera.pos = Vec3{static_cast<f32>(client->worldX - cellX * static_cast<f64>(MACRO_CELL_SIZE)),
                      client->altitude,
                      static_cast<f32>(client->worldZ - cellZ * static_cast<f64>(MACRO_CELL_SIZE))};
    camera.yaw = client->yaw;
    camera.pitch = client->pitch;
    client->renderer.setCamera(camera);

    std::string error;
    const bool ok = client->renderer.drawFrame(&error);
    if (!ok) {
        if (error != client->lastError) {
            client->lastError = error;
            appLog("frame dropped: %s", error.c_str());
        }
        return;
    }
    client->lastError.clear();
    ++client->frames;
    (void)dt;
}

// -----------------------------------------------------------------------------
//  The city download: one bounded step per frame, and only while it is wanted
// -----------------------------------------------------------------------------
// The state machine is engine/world/world_download.cpp; this is the frame loop's
// half of it.  Two things matter here.  One: a step is bounded, so a download that
// is running never owns the frame.  Two: a finished download changes what is on
// disk, so the world index is opened again — the manifest the archive carried lists
// every cell of the city, and until it is re-read the client would keep streaming
// only the cells it knew about at startup.
void stepDownload(Client* client) {
    if (!client->download || !client->downloadWanted) return;
    using State = world::WorldDownload::State;
    if (client->download->state() == State::Failed) {
        // A tap after a failure is a retry, not a restart: the part of the world
        // that arrived before the failure is on disk and verified, and the boundary
        // file remembers where the archive had got to.
        appLog("city download failed (%s); trying again", client->download->error().c_str());
        client->download.reset(new (std::nothrow)
                                   world::WorldDownload(client->fetcher, client->downloadConfig));
        if (!client->download) return;
    }
    const State before = client->download->state();
    client->download->step(kDownloadBytesPerTick);
    const State after = client->download->state();
    if (after != before) {
        appLog("city download: %s", client->download->status().c_str());
    }
    if (after == State::Done && !client->downloadApplied) {
        client->downloadApplied = true;
        client->downloadWanted = false;
        // From here the client's own slice is history: the marker keeps an app
        // update from copying it back over the city (see kCityCompleteMarker).
        markCityComplete(client->dataDir,
                         std::to_string(client->download->filesWritten()) + " files, verified");
        std::string error;
        client->haveWorld = client->renderer.openWorld(client->dataDir, &error);
        appLog("city download complete: %u cells indexed%s%s", client->renderer.worldCellCount(),
               client->haveWorld ? "" : " (world not opened: ", client->haveWorld ? "" : error.c_str());
    }
}

// -----------------------------------------------------------------------------
//  Streaming: the nearest cells that are not resident yet, a few per tick
// -----------------------------------------------------------------------------
void streamWorld(Client* client, u64 now) {
    if (!client->haveWorld || !client->ready) return;
    if (now - client->lastStreamMs < kStreamIntervalMs) return;
    client->lastStreamMs = now;
    const Vec3 focus{static_cast<f32>(client->worldX), client->altitude,
                     static_cast<f32>(client->worldZ)};
    client->lastLoad = client->renderer.loadWorldNear(focus, kStreamRadius, kStreamPerCall);
}

}  // namespace

void android_main(android_app* app) {
    // The glue library must not be stripped: it is the only reference to
    // ANativeActivity_onCreate, which is how the platform enters this code.
    app_dummy();

    Client client;
    app->userData = &client;
    app->onAppCmd = onCommand;
    app->onInputEvent = onInput;

    logSetLevel(LogLevel::Info);
    if (app->activity && app->activity->internalDataPath) {
        client.dataDir = std::string(app->activity->internalDataPath) + "/world";
    } else {
        client.dataDir = "world";
    }
    appLog("MUMBAI 1:1 EXACT — client starting");
    appLog("world data dir: %s", client.dataDir.c_str());

    // The slice that travels inside the APK is copied into the app's own storage
    // once, so that a fresh install shows Mumbai rather than an explanation.
    AAssetManager* assets = app->activity ? app->activity->assetManager : nullptr;
    if (assets) {
        beginInstall(assets, &client);
    }
    client.haveWorld = false;

    // THE REST OF THE CITY IS A DOWNLOAD, AND IT IS NOT STARTED HERE.  What is set
    // up here is the ability to fetch it: a transport, and a state machine pointed at
    // the release this build was compiled against (engine/world/world_release.h, which
    // is empty for a build with no published world).  A person starts it by tapping
    // the button, because 1.3 GB over somebody's mobile data is their decision.
    const world::WorldRelease release = world::publishedWorldRelease();
    if (release.known()) {
        client.fetcher = app::createAndroidFetcher(app->activity);
        if (client.fetcher) {
            client.downloadConfig = world::configForRelease(release, client.dataDir);
            client.download.reset(
                new (std::nothrow) world::WorldDownload(client.fetcher, client.downloadConfig));
            appLog("the city download is ready: %s", client.downloadConfig.archiveUrl.c_str());
        } else {
            client.downloadNote = "city download: no transport in this build";
            appLog("no HTTP transport: the city download is unavailable");
        }
    } else {
        client.downloadNote = "city download: no published world for this build";
    }

    while (true) {
        int events = 0;
        android_poll_source* source = nullptr;
        // Block while there is nothing to draw; with a surface, poll with a zero
        // timeout so the loop runs at the display's rate instead of being woken by
        // input.
        const int timeout = client.ready ? 0 : -1;
        while (ALooper_pollOnce(timeout, nullptr, &events, reinterpret_cast<void**>(&source)) >= 0) {
            if (source) source->process(app, source);
            if (app->destroyRequested) {
                stopRenderer(&client);
                // The downloader is destroyed before its transport, because it holds
                // a reference to it and deletes the part files it was writing.
                client.download.reset();
                app::destroyAndroidFetcher(client.fetcher);
                client.fetcher = nullptr;
                appLog("%llu frames rendered", static_cast<unsigned long long>(client.frames));
                return;
            }
        }

        if (!client.ready) {
            // The platform may hand the surface over late, or the first device
            // created may have been one that could not present.  One retry a second,
            // so a device that recovers recovers, and one that cannot does not spin.
            const u64 now = nowMillis();
            updateFallbackScreen(app, &client, now);
            if (app->window && now >= client.nextRetryMs) {
                client.nextRetryMs = now + 1000;
                startRenderer(app, &client);
            }
            continue;
        }

        const u64 now = nowMillis();
        static u64 lastFrameMs = 0;
        const f32 dt = lastFrameMs ? static_cast<f32>(now - lastFrameMs) / 1000.0f : 0.016f;
        lastFrameMs = now;

        // The starter world, installed a few megabytes at a time so the first
        // frames are on screen while it copies.
        if (assets && client.installer.active) {
            installStep(assets, &client);
            if (!client.installer.active && client.installer.done && !client.haveWorld) {
                std::string error;
                client.haveWorld = client.renderer.openWorld(client.dataDir, &error);
                if (!client.haveWorld) appLog("no world to open: %s", error.c_str());
            }
        }
        if (client.installer.done && !client.haveWorld) {
            std::string error;
            client.haveWorld = client.renderer.openWorld(client.dataDir, &error);
            if (!client.haveWorld) appLog("world index: %s", error.c_str());
        }

        stepDownload(&client);
        updateCamera(&client, dt);
        streamWorld(&client, now);
        renderFrame(app, &client, dt);

        if (now - client.lastStatMs >= 1000) {
            const render::FrameStats& st = client.renderer.stats();
            const f64 seconds = client.lastStatMs ? static_cast<f64>(now - client.lastStatMs) / 1000.0
                                                  : 1.0;
            client.fps = static_cast<f32>(static_cast<f64>(client.frames) / seconds);
            appLog("%llu frames, %.1f fps, %u draws, %u triangles, %u instances, %u chunks, "
                   "%.1f MiB GPU, %ux%u",
                   static_cast<unsigned long long>(client.frames), client.fps, st.drawCalls,
                   st.triangles, st.instances, client.renderer.chunkCount(),
                   static_cast<f64>(st.gpuBytes) / (1024.0 * 1024.0), st.renderWidth,
                   st.renderHeight);
            client.lastStatMs = now;
            client.frames = 0;
        }
        updateHud(&client);
    }
}

#else   // !__ANDROID__

// A host build of this file exists so that its typos are found on a machine
// without an NDK.  It is not the Android client and does not pretend to be.
int main() {
    std::printf("engine/app/android_main.cpp is the Android entry point.\n");
    std::printf("Configure the app build with the NDK toolchain to produce libmumbai.so;\n");
    std::printf("on a host, `mumbai_render` (make render) is the renderer's harness, and\n");
    std::printf("`make host-check` verifies the shader inventory and the world manifest.\n");
    return 0;
}

#endif
