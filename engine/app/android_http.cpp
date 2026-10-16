// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_http.cpp
//
//  The JNI bridge between the downloader's `Fetcher` and the platform's HTTP stack.
//  See android_http.h for why the transport is Java, and world_download.h for what
//  the state machine above it does with the bytes.
//
//  THE SHAPE OF A CALL.  `get()` runs on the app's own thread — the one that drains
//  the looper and draws the frames — and blocks until the step's byte budget is
//  spent.  That is deliberate: the budget is the knob that keeps a step short, a
//  background thread would need a queue, a lock and a shutdown path for a download
//  that happens once, and a step that blocks for a quarter of a second every few
//  frames is a hitch this client can account for and a thread is a class of bug it
//  cannot.  Read and connect timeouts bound the worst case: 10 s and 15 s, after
//  which the state machine records the failure and the panel says why.
//
//  THE RESULT CODES ARE THE BRIDGE'S WHOLE VOCABULARY.  WorldFetch.get returns a
//  `long`:
//
//    >= 0   the range arrived, and the number is the resource's total size
//    -1     it failed; WorldFetch.lastError() says why
//    -2     the native callback said "enough for this step"; lastTotal() is the size
//    -3     the server ignored the Range header and started from zero, so the caller
//           must throw its stream away and ask again from the beginning
//
//  Every one of those is a case the downloader already has.  -2 exists because a
//  step is *supposed* to stop early, and a transport that cannot say "paused" would
//  make every step look like a failure.
// =============================================================================
#include "engine/app/android_http.h"

#include "engine/core/core.h"

#if defined(__ANDROID__)

#include <android/native_activity.h>   // the JavaVM the activity carries
#include <jni.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace mumbai::app {
namespace {

constexpr const char* kFetchClassName = "com.mumbai.exact.WorldFetch";
constexpr const char* kFetchMethodName = "get";
constexpr const char* kFetchMethodSignature = "(Ljava/lang/String;JJ)J";
constexpr const char* kErrorMethodName = "lastError";
constexpr const char* kErrorMethodSignature = "()Ljava/lang/String;";
constexpr const char* kTotalMethodName = "lastTotal";
constexpr const char* kTotalMethodSignature = "()J";
constexpr const char* kInfoMethodName = "systemInfo";
constexpr const char* kInfoMethodSignature = "(Landroid/content/Context;)Ljava/lang/String;";
constexpr const char* kLogMethodName = "writeLog";
constexpr const char* kLogMethodSignature =
    "(Landroid/content/Context;Ljava/lang/String;)Ljava/lang/String;";

// -----------------------------------------------------------------------------
//  FINDING AN APP'S OWN CLASS FROM A NATIVE THREAD
//
//  `FindClass` resolves a name through the class loader of the *current* native
//  frame, and the frame that creates any of this has none: android_native_app_glue
//  runs the app on a thread it created itself, so the lookup goes to the system
//  loader, which has never heard of `com.mumbai.exact.WorldFetch`.  On a phone that
//  is a *silent* failure — the transport comes back null and the client reports
//  "no transport in this build", which reads exactly like a machine that has none
//  (D-054).  The fix is to ask the activity's own loader, which is the one that
//  loaded the app's classes; `activity->clazz` is the NativeActivity instance and
//  every object knows its class, which knows its loader.
// -----------------------------------------------------------------------------
jclass findAppClass(JNIEnv* env, jobject activityObject, const char* dottedName) {
    if (!activityObject) return nullptr;
    jclass activityClass = env->GetObjectClass(activityObject);
    if (!activityClass) return nullptr;
    jclass result = nullptr;
    jmethodID getClassLoader =
        env->GetMethodID(activityClass, "getClassLoader", "()Ljava/lang/ClassLoader;");
    if (getClassLoader) {
        jobject loader = env->CallObjectMethod(activityObject, getClassLoader);
        if (env->ExceptionCheck()) {
            env->ExceptionClear();
            loader = nullptr;
        }
        if (loader) {
            jclass loaderClass = env->FindClass("java/lang/ClassLoader");
            jmethodID loadClass =
                loaderClass ? env->GetMethodID(loaderClass, "loadClass",
                                               "(Ljava/lang/String;)Ljava/lang/Class;")
                            : nullptr;
            if (loadClass) {
                // `ClassLoader.loadClass` takes the *dotted* name; the slashed form
                // belongs to JNI signatures, and passing it here is the classic way
                // to get a ClassNotFoundException that reads like a typo.
                jstring name = env->NewStringUTF(dottedName);
                result = static_cast<jclass>(env->CallObjectMethod(loader, loadClass, name));
                if (env->ExceptionCheck()) {
                    env->ExceptionClear();
                    result = nullptr;
                }
                if (name) env->DeleteLocalRef(name);
            }
            if (loaderClass) env->DeleteLocalRef(loaderClass);
            env->DeleteLocalRef(loader);
        }
    }
    env->DeleteLocalRef(activityClass);
    return result;
}

// The app's own Java class, held once per process, for the calls that are not the
// download: the device summary and the log file.  Thread-local attachment, like the
// fetcher's, because these are called from whichever thread asks.
struct AppJava {
    JavaVM* vm = nullptr;
    jclass appClass = nullptr;
    jmethodID info = nullptr;
    jmethodID log = nullptr;
    bool tried = false;
};

AppJava& appJava() {
    static AppJava bridge;
    return bridge;
}

bool environmentFor(JavaVM* vm, JNIEnv** env, bool* detached) {
    *detached = false;
    if (!vm) return false;
    const jint status = vm->GetEnv(reinterpret_cast<void**>(env), JNI_VERSION_1_6);
    if (status == JNI_OK && *env) return true;
    if (status != JNI_EDETACHED) return false;
    if (vm->AttachCurrentThread(env, nullptr) != JNI_OK || !*env) return false;
    *detached = true;
    return true;
}

std::string takeJavaString(JNIEnv* env, jstring text) {
    if (!text) return {};
    const char* chars = env->GetStringUTFChars(text, nullptr);
    std::string out = chars ? chars : "";
    if (chars) env->ReleaseStringUTFChars(text, chars);
    env->DeleteLocalRef(text);
    return out;
}

// Looks the app's class up once.  Returns nullptr with a reason logged when there is
// no VM, no activity, or no such class.
AppJava* appBridge(struct ANativeActivity* activity, JNIEnv** env, bool* detached) {
    if (!activity || !activity->vm) return nullptr;
    AppJava& bridge = appJava();
    if (!bridge.tried) {
        bridge.tried = true;
        bridge.vm = static_cast<JavaVM*>(activity->vm);
        JNIEnv* local = nullptr;
        bool localDetached = false;
        if (environmentFor(bridge.vm, &local, &localDetached)) {
            jclass found = findAppClass(local, activity->clazz, kFetchClassName);
            if (found) {
                bridge.appClass = static_cast<jclass>(local->NewGlobalRef(found));
                local->DeleteLocalRef(found);
            }
            if (bridge.appClass) {
                bridge.info =
                    local->GetStaticMethodID(bridge.appClass, kInfoMethodName, kInfoMethodSignature);
                bridge.log =
                    local->GetStaticMethodID(bridge.appClass, kLogMethodName, kLogMethodSignature);
                if (local->ExceptionCheck()) local->ExceptionClear();
            }
            if (!bridge.appClass) {
                MLOG_WARN("http: %s is not reachable through the activity's class loader",
                          kFetchClassName);
            }
            if (localDetached) bridge.vm->DetachCurrentThread();
        }
    }
    if (!bridge.appClass) return nullptr;
    if (!environmentFor(bridge.vm, env, detached)) return nullptr;
    return &bridge;
}

// The codes WorldFetch.get returns.  They are the Java file's contract too: the two
// are changed together or not at all.
constexpr jlong kResultError = -1;
constexpr jlong kResultPaused = -2;
constexpr jlong kResultRestartNeeded = -3;

// One fetch in flight.  Java holds it as a jlong and hands it back for every chunk,
// which is how the transport tells the callback where to put the bytes without a
// global: a global would be a lie the day somebody downloads two things at once.
struct ActiveCall {
    const std::function<bool(const u8*, size_t)>* onData = nullptr;
    bool stopped = false;
};

// The bytes Java hands over are copied out through this, once per chunk, because
// pinning a Java array across a call into the engine is a promise the engine cannot
// keep (it may allocate, and a GC is allowed to move it).
std::vector<u8>& chunkBuffer() {
    static std::vector<u8> buffer;
    return buffer;
}

class AndroidFetcher : public world::Fetcher {
  public:
    AndroidFetcher(JavaVM* vm, jclass fetchClass, jmethodID getMethod, jmethodID errorMethod,
                   jmethodID totalMethod)
        : m_vm(vm),
          m_class(fetchClass),
          m_get(getMethod),
          m_error(errorMethod),
          m_total(totalMethod) {}

    ~AndroidFetcher() override {
        JNIEnv* env = nullptr;
        if (m_vm && m_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK && env &&
            m_class) {
            env->DeleteGlobalRef(m_class);
        }
    }

    bool get(const std::string& url, u64 offset,
             const std::function<bool(const u8*, size_t)>& onData, u64* totalBytes,
             std::string* error) override {
        m_restartRequired = false;
        if (totalBytes) *totalBytes = 0;
        if (error) error->clear();
        if (!m_vm || !m_class || !m_get) {
            if (error) *error = "the platform's HTTP stack is not reachable";
            return false;
        }
        JNIEnv* env = nullptr;
        bool detach = false;
        if (!environment(&env, &detach)) {
            if (error) *error = "no JNI environment on this thread";
            return false;
        }

        ActiveCall call;
        call.onData = &onData;
        jstring jurl = env->NewStringUTF(url.c_str());
        if (!jurl) {
            env->ExceptionClear();
            if (detach) m_vm->DetachCurrentThread();
            if (error) *error = "the URL could not be passed to the platform";
            return false;
        }
        const jlong result = env->CallStaticLongMethod(
            m_class, m_get, jurl, static_cast<jlong>(offset),
            static_cast<jlong>(reinterpret_cast<intptr_t>(&call)));
        env->DeleteLocalRef(jurl);

        if (env->ExceptionCheck()) {
            // An exception in the Java side is a bug in it or a hostile URL; either
            // way the state machine gets a sentence, never a crash inside JNI.
            env->ExceptionDescribe();
            env->ExceptionClear();
            if (detach) m_vm->DetachCurrentThread();
            if (error) *error = "the platform's HTTP stack threw an exception";
            return false;
        }

        bool ok = false;
        if (result >= 0) {
            if (totalBytes) *totalBytes = static_cast<u64>(result);
            ok = true;
        } else if (result == kResultPaused) {
            // Not an error: the step's budget is spent, and the state machine knows
            // that because its own callback returned false.
            if (totalBytes) *totalBytes = readTotal(env);
            ok = false;
        } else if (result == kResultRestartNeeded) {
            m_restartRequired = true;
            if (error) *error = "the server ignored the byte range";
            ok = false;
        } else {
            if (error) *error = readError(env);
            if (error && error->empty()) *error = "the platform's HTTP stack failed";
            ok = false;
        }
        (void)kResultError;
        if (detach) m_vm->DetachCurrentThread();
        return ok;
    }

    // True when the last `get` found a server that answered the whole file to a
    // range request.  The downloader restarts rather than appending a prefix from a
    // different download to the bytes it already parsed (D-048).
    bool restartRequired() const override { return m_restartRequired; }

  private:
    bool environment(JNIEnv** env, bool* detached) {
        *detached = false;
        const jint status = m_vm->GetEnv(reinterpret_cast<void**>(env), JNI_VERSION_1_6);
        if (status == JNI_OK && *env) return true;
        if (status != JNI_EDETACHED) return false;
        if (m_vm->AttachCurrentThread(env, nullptr) != JNI_OK || !*env) return false;
        *detached = true;
        return true;
    }

    std::string readError(JNIEnv* env) {
        if (!m_error) return {};
        jstring text = static_cast<jstring>(env->CallStaticObjectMethod(m_class, m_error));
        return takeString(env, text);
    }

    u64 readTotal(JNIEnv* env) {
        if (!m_total) return 0;
        const jlong total = env->CallStaticLongMethod(m_class, m_total);
        return total > 0 ? static_cast<u64>(total) : 0;
    }

    std::string takeString(JNIEnv* env, jstring text) {
        if (!text) return {};
        const char* chars = env->GetStringUTFChars(text, nullptr);
        std::string out = chars ? chars : "";
        if (chars) env->ReleaseStringUTFChars(text, chars);
        env->DeleteLocalRef(text);
        return out;
    }

    JavaVM* m_vm = nullptr;
    jclass m_class = nullptr;
    jmethodID m_get = nullptr;
    jmethodID m_error = nullptr;
    jmethodID m_total = nullptr;
    bool m_restartRequired = false;
};

}  // namespace

// The static method Java calls for every chunk of the body.  Its name is the bridge:
// the mangled symbol has to match `com.mumbai.exact.WorldFetch.onChunk`, which is
// why the class and this function are renamed together or not at all.
JNIEXPORT jboolean JNICALL Java_com_mumbai_exact_WorldFetch_onChunk(JNIEnv* env, jclass, jlong handle,
                                                                   jbyteArray data, jint length) {
    auto* call = reinterpret_cast<ActiveCall*>(static_cast<intptr_t>(handle));
    if (!call || !call->onData || !data || length <= 0) return JNI_FALSE;
    std::vector<u8>& buffer = chunkBuffer();
    buffer.resize(static_cast<size_t>(length));
    env->GetByteArrayRegion(data, 0, length, reinterpret_cast<jbyte*>(buffer.data()));
    if (env->ExceptionCheck()) {
        env->ExceptionClear();
        call->stopped = true;
        return JNI_FALSE;   // stopping is the safe answer to anything unexpected
    }
    if (!(*call->onData)(buffer.data(), static_cast<size_t>(length))) {
        call->stopped = true;
        return JNI_FALSE;
    }
    return JNI_TRUE;
}

world::Fetcher* createAndroidFetcher(struct ANativeActivity* activity) {
    if (!activity || !activity->vm) return nullptr;
    auto* vm = static_cast<JavaVM*>(activity->vm);
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK || !env) {
        MLOG_WARN("http: no JNI environment on the app's thread");
        return nullptr;
    }
    // Through the activity's loader, not FindClass: see findAppClass above.  The
    // first version of this used FindClass and worked only in the imagination — on
    // the app's own thread it finds nothing, and the failure is a null transport and
    // a panel that blames the build.
    jclass local = findAppClass(env, activity->clazz, kFetchClassName);
    if (!local) {
        MLOG_ERR("http: %s is not reachable through the activity's class loader",
                 kFetchClassName);
        return nullptr;
    }
    auto* fetchClass = static_cast<jclass>(env->NewGlobalRef(local));
    env->DeleteLocalRef(local);
    if (!fetchClass) {
        MLOG_ERR("http: the fetch class could not be held");
        return nullptr;
    }
    const jmethodID getMethod =
        env->GetStaticMethodID(fetchClass, kFetchMethodName, kFetchMethodSignature);
    const jmethodID errorMethod =
        env->GetStaticMethodID(fetchClass, kErrorMethodName, kErrorMethodSignature);
    const jmethodID totalMethod =
        env->GetStaticMethodID(fetchClass, kTotalMethodName, kTotalMethodSignature);
    if (env->ExceptionCheck() || !getMethod || !errorMethod || !totalMethod) {
        env->ExceptionClear();
        env->DeleteGlobalRef(fetchClass);
        MLOG_ERR("http: WorldFetch does not have the methods the bridge expects");
        return nullptr;
    }
    MLOG_INFO("http: the platform's HTTP stack is available for the city download");
    return new AndroidFetcher(vm, fetchClass, getMethod, errorMethod, totalMethod);
}

void destroyAndroidFetcher(world::Fetcher* fetcher) { delete fetcher; }

std::string androidSystemInfo(struct ANativeActivity* activity) {
    JNIEnv* env = nullptr;
    bool detach = false;
    AppJava* bridge = appBridge(activity, &env, &detach);
    if (!bridge || !bridge->info) return std::string();
    jstring text = static_cast<jstring>(
        env->CallStaticObjectMethod(bridge->appClass, bridge->info, activity->clazz));
    if (env->ExceptionCheck()) env->ExceptionClear();
    const std::string result = takeJavaString(env, text);
    if (detach) bridge->vm->DetachCurrentThread();
    return result;
}

std::string androidWriteLog(struct ANativeActivity* activity, const std::string& text) {
    JNIEnv* env = nullptr;
    bool detach = false;
    AppJava* bridge = appBridge(activity, &env, &detach);
    if (!bridge || !bridge->log) return std::string();
    jstring payload = env->NewStringUTF(text.c_str());
    jstring where = static_cast<jstring>(env->CallStaticObjectMethod(
        bridge->appClass, bridge->log, activity->clazz, payload));
    if (env->ExceptionCheck()) env->ExceptionClear();
    const std::string result = takeJavaString(env, where);
    if (payload) env->DeleteLocalRef(payload);
    if (detach) bridge->vm->DetachCurrentThread();
    return result;
}

}  // namespace mumbai::app

#else

namespace mumbai::app {

world::Fetcher* createAndroidFetcher(struct ANativeActivity*) { return nullptr; }
void destroyAndroidFetcher(world::Fetcher* fetcher) { delete fetcher; }
// On a host there is no Java and no phone: both answer "nothing to report", and the
// client's panel says the same thing it always said on a desktop build.
std::string androidSystemInfo(struct ANativeActivity*) { return std::string(); }
std::string androidWriteLog(struct ANativeActivity*, const std::string&) { return std::string(); }

}  // namespace mumbai::app

#endif  // __ANDROID__
