#!/usr/bin/env python3
# =============================================================================
#  MUMBAI 1:1 EXACT — tools/compile_shaders.py
#
#  Compiles every shader in engine/render/shaders to SPIR-V, however a machine
#  happens to be equipped:
#
#    glslc            (the Vulkan SDK's front end — what the Makefile prefers)
#    glslangValidator (the reference compiler — what Ubuntu's `glslang-tools`
#                      and the Khronos release binaries provide, and what the
#                      Android CI runner installs)
#    neither          -> the caller is told, loudly, and the embedder writes
#                        empty modules so a build without shaders still links
#                        and fails at pipeline creation with a name, not a
#                        black screen and a shrug.
#
#  It then checks what it produced: every module must start with the SPIR-V
#  magic word and be longer than the five-word header.  A zero-length or
#  truncated module is exactly what a shader that failed to compile leaves
#  behind when the compiler's exit status is ignored, and it is the failure this
#  script exists to prevent.
#
#      python3 tools/compile_shaders.py engine/render/shaders build/spv
# =============================================================================
import os
import shutil
import subprocess
import sys

STAGES = {'.vert': 'vertex', '.frag': 'fragment', '.comp': 'compute'}
SPIRV_MAGIC = 0x07230203


def find_compiler():
    for name, kind in (('glslc', 'glslc'), ('glslangValidator', 'glslang')):
        path = os.environ.get('MUMBAI_GLSL_COMPILER') if name == 'glslc' else None
        path = path or shutil.which(name)
        if path:
            return kind, path
    env = os.environ.get('MUMBAI_GLSL_COMPILER')
    if env and os.path.isfile(env):
        return ('glslc' if os.path.basename(env).startswith('glslc') else 'glslang'), env
    return None, None


def command_for(kind, compiler, source, out, include_dir):
    if kind == 'glslc':
        return [compiler, f'-I{include_dir}', '--target-env=vulkan1.1', '-O', source, '-o', out]
    # glslangValidator infers the stage from the file extension and writes SPIR-V
    # with -V.  -o is supported with -V in every version this project builds with.
    return [compiler, '-V', '--target-env', 'vulkan1.1', f'-I{include_dir}', '-o', out, source]


def check_module(path):
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 20 or len(data) % 4 != 0:
        return False, f'{len(data)} bytes'
    magic = int.from_bytes(data[0:4], 'little')
    if magic != SPIRV_MAGIC:
        return False, f'magic 0x{magic:08x}'
    return True, f'{len(data) // 4} words'


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    # A build that embeds empty shader modules is a build that produces an APK
    # which cannot draw anything, and the reason (no compiler) scrolls off the
    # top of a four-thousand-line log.  Required by default: pass
    # --allow-missing-compiler for a build that genuinely does not need shaders.
    require = '--allow-missing-compiler' not in sys.argv
    if len(args) < 2:
        print('usage: compile_shaders.py [--allow-missing-compiler] <shader-dir> <out-dir>',
              file=sys.stderr)
        return 2
    shader_dir, out_dir = args[0], args[1]
    os.makedirs(out_dir, exist_ok=True)

    sources = sorted(f for f in os.listdir(shader_dir) if os.path.splitext(f)[1] in STAGES)
    if not sources:
        print(f'  no shaders found in {shader_dir}', file=sys.stderr)
        return 1

    kind, compiler = find_compiler()
    if not compiler:
        print('  NOTE no glslc or glslangValidator on this machine: shaders are NOT compiled.')
        print('       The embedder will emit empty modules and every pipeline will refuse')
        print('       to build, by name, at runtime.')
        if require:
            print('  Get one with `make glslang` (builds glslangValidator from source into')
            print('  build/tools/glslang), or `apt-get install glslang-tools`, or pass')
            print('  --allow-missing-compiler if this build really does not need shaders.')
            return 1
        return 0

    print(f'  compiler: {compiler} ({kind})')
    failures = []
    total_bytes = 0
    for name in sources:
        out = os.path.join(out_dir, name + '.spv')
        cmd = command_for(kind, compiler, os.path.join(shader_dir, name), out, shader_dir)
        proc = subprocess.run(cmd, capture_output=True, text=True)
        # glslangValidator returns non-zero for warnings-as-errors; both compilers
        # print to stderr and stdout inconsistently, so both are reported.
        if proc.returncode != 0:
            failures.append((name, proc.stdout + proc.stderr))
            continue
        ok, detail = check_module(out)
        if not ok:
            failures.append((name, f'produced an invalid module ({detail})'))
            continue
        total_bytes += os.path.getsize(out)
        print(f'  {name:14s} -> {os.path.basename(out):18s} {detail}')

    # Cross-stage linking.  Each stage compiles on its own, but a vertex shader
    # declaring `flat out uint` against a fragment shader declaring `in uint` is
    # only caught when the two are linked, and the device-side symptom is not a
    # compile error — it is whatever the driver decides to hand the fragment shader.
    # glslc has no equivalent mode, so this runs under glslang only.
    linked = 0
    if kind == 'glslang' and not failures:
        pairs = {}
        for name in sources:
            stem, ext = os.path.splitext(name)
            pairs.setdefault(stem, {})[ext] = name
        for stem, stages in sorted(pairs.items()):
            if '.vert' not in stages or '.frag' not in stages:
                continue
            out = os.path.join(out_dir, stem + '.linked.spv')
            cmd = [compiler, '-V', '--target-env', 'vulkan1.1', f'-I{shader_dir}', '-l',
                   '-o', out, os.path.join(shader_dir, stages['.vert']),
                   os.path.join(shader_dir, stages['.frag'])]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            if proc.returncode != 0:
                failures.append((stem + ' (vertex+fragment link)', proc.stdout + proc.stderr))
            else:
                linked += 1
                os.remove(out)
        print(f'  linked:   {linked} vertex/fragment pairs agree on their interfaces')

    print('-' * 71)
    print(f'  {len(sources)} shaders, {len(failures)} failed, '
          f'{total_bytes / 1024.0:.1f} KiB of SPIR-V')
    if failures:
        for name, log in failures:
            print(f'  FAIL {name}')
            for line in log.strip().splitlines()[:40]:
                print(f'       {line}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
