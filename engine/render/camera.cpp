// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/camera.cpp
//
//  The camera's matrices, deliberately in a translation unit of their own.
//
//  They lived in frame.cpp at first, which was fine until the harness had to be
//  linkable on a machine with no Vulkan loader: frame.cpp cannot link without the
//  device layer, and a build-time check that cannot run on a build machine is not
//  a check.  Camera maths touches no Vulkan type, so it does not belong in a file
//  that does — and that is a decision worth keeping rather than a detail of the
//  Makefile (D-043).
//
//  ---------------------------------------------------------------------------
//  WHICH WAY IS RIGHT?  (D-044 — read this before "fixing" the cross products)
//
//  The world's axes are x = east metres, y = up metres, z = north metres
//  (`TransverseMercator::forward` returns easting/northing, and the pipeline
//  writes them straight into x/z).  Those three directions are **left-handed**:
//  east x up = south, not north.  That is a property of the data convention this
//  project adopted in A0, not a choice made here, and it cannot be flipped now
//  without mirroring 1.4 GB of published world.
//
//  Standing in a left-handed world, a camera built the way every textbook
//  (gluLookAt, three.js, `Mat4::lookAt`) builds one renders the city
//  **mirrored**: facing north up Marine Drive, the Arabian Sea — which is west,
//  and therefore on your left — appears on the right of the screen.  Every
//  landmark is internally consistent, so nothing looks broken until you know
//  Mumbai: Chowpatty on the wrong side, the Sea Link curving the wrong way.
//
//  The camera therefore builds its basis from the physical question instead:
//  given forward `f` and world up, the direction on the **right of the frame**
//  is `up x f`.  With f = north and up = y, that is (0,1,0) x (0,0,1) = (1,0,0)
//  = east, which is where east belongs when you face north.
//
//      screen right  s = normalise(up x f)        (east when facing north)
//      screen up     u = normalise(f x s)
//      view forward  z = -f                       (the projection's convention)
//
//  `Mat4::lookAt` is left alone: it is the correct right-handed lookAt, it is
//  general maths with no opinion about Mumbai, and nothing in the engine calls
//  it (checked — this file was its only caller).
// =============================================================================
#include "engine/render/renderer.h"

#include <cmath>

namespace mumbai::render {

Vec3 Camera::forward() const {
    const f32 cp = std::cos(pitch);
    return Vec3{std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp};
}

Mat4 Camera::view() const {
    const Vec3 f = forward();
    Vec3 s = Vec3{0.0f, 1.0f, 0.0f}.cross(f);      // up x forward = physical right
    if (s.lengthSq() < 1e-10f) {
        // Looking straight up or down: the right direction is undefined, so pick
        // one deterministically rather than handing the projection a NaN and
        // letting it become a black frame with no explanation.
        s = Vec3{1.0f, 0.0f, 0.0f};
    }
    s = s.normalized();
    const Vec3 u = f.cross(s).normalized();        // already unit if f and s are orthogonal

    Mat4 r;
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;  r.m[12] = -s.dot(pos);
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;  r.m[13] = -u.dot(pos);
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z; r.m[14] =  f.dot(pos);
    r.m[3] = 0.0f; r.m[7] = 0.0f; r.m[11] = 0.0f; r.m[15] =  1.0f;
    return r;
}

Mat4 Camera::proj(f32 aspect) const {
    return Mat4::perspective(fovY, aspect, zNear, zFar);
}

}  // namespace mumbai::render
