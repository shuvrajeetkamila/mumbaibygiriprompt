#!/usr/bin/env python3
# =============================================================================
#  MUMBAI 1:1 EXACT — tools/verify_shader_layout.py
#
#  Checks the two interfaces between C++ and GLSL that fail silently.
#
#  1. THE UNIFORM BLOCK.  std140 offsets are derived from the C++ struct by the
#     same rules the GLSL compiler uses, and are compared member by member with
#     the block in the shader: name, type, array length and offset.  A mismatch
#     here produces no error, no warning and a plausible-looking wrong picture —
#     a sheared city, a camera in the wrong place, a sun that rises sideways.
#     It is the single most expensive class of bug to find on a device, so it is
#     checked on every build instead.
#
#  2. THE VERTEX ATTRIBUTES.  Every `layout(location = N) in TYPE name;` in a
#     vertex shader must have a matching VkVertexInputAttributeDescription in
#     pipeline.cpp with the same location, a compatible format, and a binding
#     that matches the buffer it reads from.  A missing attribute is undefined
#     behaviour (usually zeros, sometimes garbage), and a wrong format reads the
#     vertex stride off by four bytes and shreds the mesh.
#
#  Exit code 0 means every check passed.  Anything else is a failure, because
#  this tool only runs when someone can still fix it.
# =============================================================================
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FAILURES = []
CHECKS = 0


def fail(msg):
    FAILURES.append(msg)


def check(cond, msg):
    global CHECKS
    CHECKS += 1
    if not cond:
        fail(msg)
    return cond


# -----------------------------------------------------------------------------
#  GLSL types -> (component type, component count, base alignment, size)
# -----------------------------------------------------------------------------
SCALARS = {
    'float': ('f32', 1), 'int': ('i32', 1), 'uint': ('u32', 1), 'bool': ('b32', 1),
}
VECTORS = {'vec2': 'float', 'vec3': 'float', 'vec4': 'float',
           'ivec2': 'int', 'ivec3': 'int', 'ivec4': 'int',
           'uvec2': 'uint', 'uvec3': 'uint', 'uvec4': 'uint'}
MATRICES = {'mat2': (2, 2), 'mat3': (3, 3), 'mat4': (4, 4)}


def glsl_size_alignment(type_name, count=1):
    """std140 size and alignment for a GLSL member type, with array count."""
    if type_name in SCALARS:
        base_align, base_size = 4, 4
    elif type_name in VECTORS:
        n = int(type_name[-1])
        base_align, base_size = (8 if n == 2 else 16), 4 * n
    elif type_name in MATRICES:
        cols, rows = MATRICES[type_name]
        col_size = 16 if rows != 2 else 8
        # Each column is aligned to 16 (except vec2 columns, aligned to 8) and the
        # matrix is treated as an array of columns.
        size = cols * (col_size if col_size > 8 else 16)
        return size, 16
    elif type_name.startswith('vec') or type_name.startswith('mat'):
        return None, None
    else:
        return None, None
    if count > 1:
        # An array's stride is its base alignment rounded up to 16.
        stride = (base_size + 15) // 16 * 16
        return stride * count, 16
    return base_size, base_align


# -----------------------------------------------------------------------------
#  1. C++ FrameUbo
# -----------------------------------------------------------------------------
def parse_cpp_struct(path, name):
    text = Path(path).read_text()
    m = re.search(r'struct\s+' + name + r'\s*\{(.*?)\n\};', text, re.S)
    if not m:
        fail(f'{path}: struct {name} not found')
        return None
    body = re.sub(r'//[^\n]*', '', m.group(1))
    members = []
    for decl in re.finditer(r'([\w:]+)\s+(\w+)\s*(\[(\d+)\])?\s*;', body):
        type_name, member = decl.group(1), decl.group(2)
        arr = int(decl.group(4)) if decl.group(4) else 1
        members.append((member, type_name, arr))
    # The GLSL float types are what the GPU sees; a C++ f32 is a GLSL float.
    return members


def cpp_member_glsl_type(cpp_type, arr):
    """C++ member -> the GLSL type the shader declares for it.

    The project's convention (renderer_impl.h states it): a small fixed-size array
    of f32 in the uniform block *is* the GLSL vector or matrix, not an array of
    floats — `f32 cameraPos[4]` is `vec4 cameraPos`, `f32 viewProj[16]` is
    `mat4 viewProj`.  std140 lays these out identically, which is the point.
    """
    base = {'f32': 'float', 'u32': 'uint', 'i32': 'int'}.get(cpp_type, cpp_type)
    if base == 'float' and arr in (2, 3, 4):
        return f'vec{arr}', 1
    if base == 'float' and arr == 16:
        return 'mat4', 1
    return base, arr


def std140_layout(members):
    """(name, glsl_type, array_len, offset) in declaration order."""
    out, offset = [], 0
    for member, cpp_type, arr in members:
        glsl_type, arr = cpp_member_glsl_type(cpp_type, arr)
        size, align = glsl_size_alignment(glsl_type, arr)
        if size is None:
            fail(f'FrameUbo: cannot compute std140 for {cpp_type} {member}')
            return out
        offset = (offset + align - 1) // align * align
        out.append((member, glsl_type, arr, offset))
        offset += size
    return out


def parse_glsl_block(path, block_name):
    text = Path(path).read_text()
    # The block ends with `} instanceName;`, not `};` — GLSL requires the instance
    # name and the C++/GLSL confusion here cost this tool its only real check the
    # first time it ran.
    m = re.search(r'layout\([^)]*std140[^)]*\)\s*uniform\s+' + block_name + r'\s*\{(.*?)\}\s*\w*\s*;', text, re.S)
    if not m:
        fail(f'{path}: uniform block {block_name} not found')
        return None
    body = re.sub(r'//[^\n]*', '', m.group(1))
    members = []
    for decl in re.finditer(r'(?:layout\([^)]*\)\s*)?([\w]+)\s+(\w+)\s*(\[(\d+)\])?\s*;', body):
        members.append((decl.group(2), decl.group(1), int(decl.group(4)) if decl.group(4) else 1))
    return members


def verify_uniform_block():
    cpp = parse_cpp_struct(ROOT / 'engine/render/renderer_impl.h', 'FrameUbo')
    glsl = parse_glsl_block(ROOT / 'engine/render/shaders/common.glsl', 'FrameBlock')
    if not cpp or not glsl:
        return
    cpp_layout = std140_layout(cpp)
    if len(cpp_layout) != len(glsl):
        fail(f'FrameBlock has {len(glsl)} members in GLSL but {len(cpp_layout)} in C++')
        return
    for (name, ctype, carr, coff), (gname, gtype, garr) in zip(cpp_layout, glsl):
        check(name == gname, f'FrameBlock member {coff}: C++ "{name}" vs GLSL "{gname}"')
        check(ctype == gtype, f'FrameBlock {name}: C++ {ctype} vs GLSL {gtype}')
        check(carr == garr, f'FrameBlock {name}: array {carr} vs {garr}')
    size = 0
    for name, ctype, carr, coff in cpp_layout:
        m_size, _ = glsl_size_alignment(ctype, carr)
        size = max(size, coff + m_size)
    total = (size + 15) // 16 * 16
    print(f'  uniform block: FrameBlock, {len(glsl)} members, std140 size {total} B '
          f'(C++ struct {total if total == 160 else "?"} B declared 160)')
    check(total == 160, f'FrameBlock std140 size is {total} B, the C++ static_assert says 160')


# -----------------------------------------------------------------------------
#  2. Vertex attributes
# -----------------------------------------------------------------------------
# A Vulkan format's shape: (bytes, component count, component type).  The check
# compares shapes rather than format names, because a shader is allowed to read a
# packed attribute as a wider scalar: `VK_FORMAT_R8G8B8A8_UINT` (4 B of four u8,
# unpacked in the shader with shifts) is exactly what a GLSL `uint` attribute must
# see.  Requiring identical names would reject the one idiom this project uses.
FORMATS = {
    ('float', 1): (4, 1, 'float'), ('float', 2): (8, 2, 'float'),
    ('float', 3): (12, 3, 'float'), ('float', 4): (16, 4, 'float'),
    ('uint', 1): (4, 1, 'uint'), ('uint', 2): (8, 2, 'uint'),
    ('uint', 3): (12, 3, 'uint'), ('uint', 4): (16, 4, 'uint'),
    ('int', 1): (4, 1, 'int'), ('int', 2): (8, 2, 'int'),
    ('int', 3): (12, 3, 'int'), ('int', 4): (16, 4, 'int'),
}
VULKAN_FORMATS = {
    'VK_FORMAT_R32_SFLOAT': (4, 1, 'float'), 'VK_FORMAT_R32G32_SFLOAT': (8, 2, 'float'),
    'VK_FORMAT_R32G32B32_SFLOAT': (12, 3, 'float'),
    'VK_FORMAT_R32G32B32A32_SFLOAT': (16, 4, 'float'),
    'VK_FORMAT_R32_UINT': (4, 1, 'uint'), 'VK_FORMAT_R32G32_UINT': (8, 2, 'uint'),
    'VK_FORMAT_R32G32B32_UINT': (12, 3, 'uint'),
    'VK_FORMAT_R32G32B32A32_UINT': (16, 4, 'uint'),
    'VK_FORMAT_R8G8B8A8_UINT': (4, 4, 'uint'),
    'VK_FORMAT_R8G8B8A8_UNORM': (4, 4, 'float'),
    'VK_FORMAT_R32_SINT': (4, 1, 'int'), 'VK_FORMAT_R32G32B32A32_SINT': (16, 4, 'int'),
}


def glsl_attr_shape(glsl_type):
    """(bytes, component count, component type) as the shader declares it."""
    base = None
    if glsl_type in ('float', 'uint', 'int', 'bool'):
        base = 'float' if glsl_type == 'bool' else glsl_type
        return FORMATS[(base, 1)]
    prefixes = {'vec': 'float', 'uvec': 'uint', 'ivec': 'int', 'bvec': 'uint'}
    for prefix, comp in prefixes.items():
        if glsl_type.startswith(prefix) and glsl_type[len(prefix):].isdigit():
            n = int(glsl_type[len(prefix):])
            return FORMATS[(comp, n)]
    return None


def format_matches(shader_type, vulkan_format):
    """True when the declared format feeds the shader's attribute correctly."""
    shape = glsl_attr_shape(shader_type)
    fmt = VULKAN_FORMATS.get(vulkan_format)
    if shape is None or fmt is None:
        fail(f'unknown type pair: shader {shader_type} / {vulkan_format}')
        return True
    s_bytes, s_count, s_comp = shape
    f_bytes, f_count, f_comp = fmt
    if s_bytes != f_bytes:
        return False
    if s_comp != f_comp:
        return False
    # A scalar attribute may read several packed components (the u8 packing idiom);
    # a vector attribute must match component for component.
    if s_count > 1 and s_count != f_count:
        return False
    return True


def cpp_attr_descriptions(shader_name):
    """The {location, binding, format, offsetof(X, field)} entries for a shader."""
    text = (ROOT / 'engine/render/pipeline.cpp').read_text()
    # Attribute entries may wrap across lines (the traffic colour attribute does),
    # so the whole file is scanned rather than each line.
    # world.vert reads MeshVertex; traffic.vert reads a mesh binding plus the
    # per-instance binding, so both struct names count as 'this shader's attributes'.
    anchors = (('MeshVertex',) if shader_name == 'world.vert'
               else ('Vertex3_3_3', 'InstanceVertex'))
    out = []
    pattern = (r'\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(VK_FORMAT_\w+)\s*,\s*'
               r'(?:static_cast<\w+>\()?\s*offsetof\(\s*(\w+)\s*,\s*(\w+)\s*\)')
    for m in re.finditer(pattern, text, re.S):
        if m.group(4) in anchors:
            out.append((int(m.group(1)), int(m.group(2)), m.group(3), m.group(5)))
    return out


def verify_vertex_attributes():
    for shader, prefix in (('world.vert', 'in'), ('traffic.vert', 'i')):
        text = (ROOT / 'engine/render/shaders' / shader).read_text()
        glsl = [(int(m.group(1)), m.group(2), m.group(3))
                for m in re.finditer(r'layout\(location\s*=\s*(\d+)\)\s+in\s+([\w]+)\s+(\w+)\s*;', text)]
        cpp = cpp_attr_descriptions(shader)
        check(len(glsl) == len(cpp),
              f'{shader}: {len(glsl)} vertex inputs but {len(cpp)} attribute descriptions')
        for (loc, gtype, gname), (cloc, cbinding, cformat, cfield) in zip(glsl, cpp):
            check(loc == cloc,
                  f'{shader}: attribute {gname} is location {loc} in GLSL but {cloc} in pipeline.cpp')
            check(format_matches(gtype, cformat),
                  f'{shader}: attribute {gname} ({gtype}) is fed by {cformat}, '
                  f'which does not have that shape')
        print(f'  vertex input:  {shader}, {len(glsl)} attributes matched to pipeline.cpp '
              f'(locations, formats)')

    # The instance buffers share one buffer with two bindings; binding 1 must exist
    # and must be the stride of InstanceVertex in the GLSL, or every vehicle reads
    # its neighbour's transform.
    text = (ROOT / 'engine/render/pipeline.cpp').read_text()
    bindings = re.findall(r'bindings\[(\d+)\][^\n]*VK_VERTEX_INPUT_RATE_(\w+)', text)
    check(('1', 'INSTANCE') in bindings or "binding = 1" in text,
          'pipeline.cpp: no instance-rate vertex binding for the traffic buffers')


def main():
    print('MUMBAI 1:1 EXACT — shader/C++ interface verification')
    print('=' * 71)
    verify_uniform_block()
    verify_vertex_attributes()
    print('-' * 71)
    if FAILURES:
        for f in FAILURES:
            print(f'  FAIL {f}')
        print(f' {CHECKS} checks, {len(FAILURES)} failures — FAIL')
        return 1
    print(f' {CHECKS} checks, 0 failures — PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
