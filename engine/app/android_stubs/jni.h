// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/jni.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule:
//  the client compiles on a machine with no NDK, against headers that declare the
//  same names with the same signatures, so that a typo costs a compile here rather
//  than a CI round trip.  This one covers JNI, which is where a typo is most
//  expensive: a wrong method name or a wrong `lastError` lookup does not fail to
//  compile, it fails at runtime, on a phone, as a download that never starts.
//
//  WHAT IS DIFFERENT FROM THE REAL HEADER, AND WHY IT DOES NOT MATTER.  The NDK's
//  <jni.h> declares JNIEnv as a struct holding a pointer to a table of function
//  pointers, with thousands of lines of C++ inline wrappers so that `env->FindClass`
//  is spelled the same from C and C++.  Here JNIEnv is a plain struct with those
//  same member functions, and JavaVM likewise.  Call sites are therefore checked
//  against the same names and the same argument types — which is all a compile check
//  is for — and nothing here is ever linked: the stub has no implementation, and a
//  host build cannot call it.  The variadic `Call*Method` forms are variadic here
//  too, because that is what the NDK has.
//
//  JNI IS NOT THE APP'S LANGUAGE.  Exactly one class of the app is Java
//  (android/app/src/main/java/com/mumbai/exact/WorldFetch.java), and it exists for
//  one reason: HTTPS.  See engine/app/android_http.h.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_JNI_H
#define MUMBAI_ANDROID_STUBS_JNI_H

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JNI_VERSION_1_6 0x00010006
#define JNI_OK 0
#define JNI_ERR (-1)
#define JNI_EDETACHED (-2)

#define JNI_FALSE 0
#define JNI_TRUE 1

typedef uint8_t jboolean;
typedef int8_t jbyte;
typedef uint16_t jchar;
typedef int16_t jshort;
typedef int32_t jint;
typedef int64_t jlong;
typedef float jfloat;
typedef double jdouble;
typedef jint jsize;

#ifdef __cplusplus
struct _jobject {};
typedef _jobject* jobject;
typedef _jobject* jclass;
typedef _jobject* jstring;
typedef _jobject* jthrowable;
typedef _jobject* jarray;
typedef _jobject* jbyteArray;
#else
typedef void* jobject;
typedef void* jclass;
typedef void* jstring;
typedef void* jthrowable;
typedef void* jarray;
typedef void* jbyteArray;
#endif

typedef struct _jmethodID* jmethodID;
typedef struct _jfieldID* jfieldID;

struct JNINativeMethod {
    const char* name;
    const char* signature;
    void* fnPtr;
};

struct JNIEnv {
    // ---- class and method lookup
    jclass FindClass(const char* name);
    jclass GetObjectClass(jobject obj);
    jmethodID GetStaticMethodID(jclass clazz, const char* name, const char* sig);
    jfieldID GetStaticFieldID(jclass clazz, const char* name, const char* sig);

    // ---- strings and references
    jstring NewStringUTF(const char* utf);
    const char* GetStringUTFChars(jstring str, jboolean* isCopy);
    void ReleaseStringUTFChars(jstring str, const char* chars);
    jobject NewGlobalRef(jobject obj);
    void DeleteGlobalRef(jobject obj);
    void DeleteLocalRef(jobject obj);

    // ---- arrays
    jsize GetArrayLength(jarray array);
    void GetByteArrayRegion(jbyteArray array, jsize start, jsize len, jbyte* buf);

    // ---- fields and calls
    jboolean GetStaticBooleanField(jclass clazz, jfieldID field);
    jint GetStaticIntField(jclass clazz, jfieldID field);
    jobject GetStaticObjectField(jclass clazz, jfieldID field);
    jlong CallStaticLongMethod(jclass clazz, jmethodID methodID, ...);
    jobject CallStaticObjectMethod(jclass clazz, jmethodID methodID, ...);
    jboolean CallStaticBooleanMethod(jclass clazz, jmethodID methodID, ...);
    jint ThrowNew(jclass clazz, const char* message);

    // ---- exceptions
    jboolean ExceptionCheck();
    void ExceptionDescribe();
    void ExceptionClear();
    void ExceptionOccurred();
};

struct JavaVMAttachArgs {
    jint version;
    const char* name;
    jobject group;
};

struct JavaVM {
    jint GetEnv(void** env, jint version);
    jint AttachCurrentThread(JNIEnv** env, void* thr_args);
    jint DetachCurrentThread();
};

typedef JNIEnv* (*jni_onload_t)(JavaVM* vm, void* reserved);

#ifdef __cplusplus
}
#endif

// The real header spells the exported entry points with these.  On the host they
// only have to make the declaration in the .cpp file look like the NDK's.
#ifndef JNIEXPORT
#define JNIEXPORT
#endif
#ifndef JNICALL
#define JNICALL
#endif

#endif  // MUMBAI_ANDROID_STUBS_JNI_H
