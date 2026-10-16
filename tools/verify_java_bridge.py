#!/usr/bin/env python3
# =============================================================================
#  MUMBAI 1:1 EXACT — tools/verify_java_bridge.py
#
#  THE JNI BRIDGE IS SPELLED TWICE, AND JNI DOES NOT COMPLAIN WHEN IT IS NOT.
#
#  engine/app/android_http.cpp looks the Java side up by *name*: a class name, three
#  method names, three signatures, and the mangled symbol that the runtime resolves
#  when Java calls back into native code.  None of that is checked by either compiler:
#  a wrong method name is a null method ID at runtime, a wrong signature is a
#  NoSuchMethodError caught by the bridge and reported on the panel as "the platform's
#  HTTP stack is not reachable", and a wrong symbol name is an UnsatisfiedLinkError on
#  the first chunk — on a phone, in the middle of somebody's download, with nothing in
#  this repository having said a word.  So the two files are compared here instead.
#
#  It is deliberately a *spelling* check: package, class, method names, argument
#  types, and the mangled symbol.  There is no JDK in the development sandbox, so the
#  Java file itself is compiled only by CI; this catches everything except a Java type
#  error, which is the part a compiler is actually good at.
#
#  Run: python3 tools/verify_java_bridge.py
# =============================================================================
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JAVA = os.path.join(ROOT, "android/app/src/main/java/com/mumbai/exact/WorldFetch.java")
BRIDGE = os.path.join(ROOT, "engine/app/android_http.cpp")

# The signature the bridge expects for each Java method, in JNI's own notation.  These
# four strings are the contract; the bridge's constants and this table are changed
# together or not at all.
EXPECTED = {
    "get": ("(Ljava/lang/String;JJ)J", ["String", "long", "long"], "long"),
    "lastError": ("()Ljava/lang/String;", [], "String"),
    "lastTotal": ("()J", [], "long"),
    "onChunk": ("(J[BI)Z", ["long", "byte[]", "int"], "boolean"),
}

failures = 0
checks = 0


def check(condition, what):
    global checks, failures
    checks += 1
    if condition:
        print(f"  ok   {what}")
    else:
        print(f"  FAIL {what}")
        failures += 1


def read(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def python_type_to_jni(text):
    table = {
        "byte": "B", "char": "C", "double": "D", "float": "F",
        "int": "I", "long": "J", "short": "S", "boolean": "Z", "void": "V",
    }
    text = text.strip()
    if text.endswith("[]"):
        return "[" + python_type_to_jni(text[:-2])
    return table.get(text, "L" + text.replace(".", "/") + ";")


def main():
    java = read(JAVA)
    bridge = read(BRIDGE)

    # --- 1. the package and the class name the bridge looks up -------------------
    package = re.search(r"^\s*package\s+([\w.]+)\s*;", java, re.M)
    klass = re.search(r"^\s*(?:public\s+)?(?:final\s+)?class\s+(\w+)", java, re.M)
    check(package is not None, "WorldFetch.java declares a package")
    check(klass is not None, "WorldFetch.java declares a class")
    if not package or not klass:
        return 1
    dotted = f"{package.group(1)}.{klass.group(1)}"
    print(f"  java: {dotted}")

    # The C++ side's own idea of the class name, and the two must agree.
    bridge_class = re.search(r'kFetchClassName\s*=\s*"([^"]+)"', bridge)
    check(bridge_class is not None, "android_http.cpp names the class it looks up")
    if bridge_class:
        check(bridge_class.group(1) == dotted,
              f"the bridge looks up {bridge_class.group(1)} and the Java file declares {dotted}")

    # --- 2. the methods, their names, argument types and return types ------------
    found = {}
    for match in re.finditer(
            r"((?:public|private|protected)\s+static\s+(?:native\s+)?[\w\[\]<>, .]+\s+)(\w+)\s*\(([^)]*)\)",
            java):
        name = match.group(2)
        # The *type* of each argument, not its name: `String url` is a String.
        args = [a.strip().split()[0] for a in match.group(3).split(",") if a.strip()]
        ret = re.sub(r"(public|private|protected|static|native)\s+", "", match.group(1)).strip()
        found[name] = (args, ret)
    for name, (signature, args, ret) in EXPECTED.items():
        check(name in found, f"WorldFetch.java declares {name}()")
        if name not in found:
            continue
        actual_args, actual_ret = found[name]
        check(actual_args == args,
              f"{name}({', '.join(actual_args)}) takes the declared argument types")
        check(actual_ret == ret, f"{name}() returns {actual_ret}")
        if name == "onChunk":
            # onChunk is `native` in Java: nothing looks it up by signature, the
            # runtime resolves the symbol and calls it.  What has to line up is the
            # argument list the C++ definition takes, and the JNI types that go with
            # it — a mismatch there is undefined behaviour, not an error message.
            check(re.search(r"jlong\s+\w+,\s*jbyteArray\s+\w+,\s*jint\s+\w+", bridge)
                  is not None,
                  "android_http.cpp's onChunk takes (jlong handle, jbyteArray data, jint length)")
        else:
            # Looked up by GetStaticMethodID, so the signature string has to describe
            # exactly what the Java file declares.
            check(signature in bridge,
                  f"android_http.cpp asks for {name} with the signature {signature}")

    # --- 3. the mangled symbol the runtime resolves ------------------------------
    mangled = "Java_" + dotted.replace(".", "_") + "_onChunk"
    check(f"Java_com_mumbai_exact_WorldFetch_onChunk" in bridge,
          "android_http.cpp defines the onChunk symbol with the class's own name")
    check(mangled == "Java_com_mumbai_exact_WorldFetch_onChunk",
          f"the symbol the bytes have to be named is {mangled}")
    # A native method declared in Java but never defined in C++ is an
    # UnsatisfiedLinkError on the first call, which is the whole point of this file.
    check(re.search(r"native\s+boolean\s+onChunk\s*\(", java) is not None,
          "WorldFetch.java declares onChunk as a native method")

    # --- 4. the library the Java side loads is the one the APK ships -------------
    library = re.search(r'System\.loadLibrary\("([^"]+)"\)', java)
    check(library is not None, "WorldFetch.java loads a native library")
    if library:
        check(library.group(1) == "mumbai",
              f"it loads lib{library.group(1)}.so, which is what CMake names the target")

    print("-" * 71)
    print(f"  {checks} checks, {failures} failures — {'PASS' if not failures else 'FAIL'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
