// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_render.cpp
//
//  What can be tested about a renderer with no GPU in the room: the camera, the
//  lens, and the claim that the city comes out the right way round.
//
//  The orientation cases are the important ones and they are not academic.  The
//  world's axes are x = east, y = up, z = north, which is a **left-handed**
//  triple; a camera built the textbook way renders Mumbai mirrored, and the
//  symptom is not a crash or a black frame — it is the Arabian Sea on the wrong
//  side of Marine Drive, which only a resident would call a bug (D-044).  So the
//  test poses the resident's question directly: standing at Marine Drive facing
//  north, is the sea on the left?
// =============================================================================
#include "engine/core/geo.h"
#include "engine/render/renderer.h"
#include "tests/test_harness.h"

#include <cmath>

using namespace mumbai;

namespace {

// Project a world-space point through a camera and return NDC (x, y, z, w).
Vec4 projectNdc(const render::Camera& cam, const Vec3& world, f32 aspect) {
    const Vec4 clip = cam.proj(aspect) * (cam.view() * Vec4{world.x, world.y, world.z, 1.0f});
    return clip;
}

struct Screen {
    f32 x = 0, y = 0, z = 0, w = 0;
    bool inFront() const { return w > 0.0f; }
};

Screen onScreen(const render::Camera& cam, const Vec3& world, f32 aspect) {
    const Vec4 c = projectNdc(cam, world, aspect);
    Screen s;
    s.w = c.w;
    if (std::fabs(c.w) > 1e-12f) { s.x = c.x / c.w; s.y = c.y / c.w; s.z = c.z / c.w; }
    return s;
}

// The camera the acceptance shot uses: on the coast road at Marine Drive, at
// eye height, facing north up the bay.
render::Camera marineDriveFacingNorth() {
    const TransverseMercator proj;
    const Vec2d w = proj.forward(18.9433, 72.8230);      // Marine Drive, Chowpatty end
    render::Camera cam;
    cam.pos = Vec3{0.0f, 1.7f, 0.0f};                     // render space: see below
    cam.yaw = 0.0f;                                      // due north
    cam.pitch = 0.0f;
    cam.fovY = 1.0472f;                                  // 60 degrees
    cam.zNear = 0.25f;
    cam.zFar = 6000.0f;
    (void)w;
    return cam;
}

}  // namespace

// -----------------------------------------------------------------------------
//  The resident's test
// -----------------------------------------------------------------------------
TEST(cameraPutsTheArabianSeaOnTheLeftWhenFacingNorth) {
    const f32 aspect = 16.0f / 9.0f;
    const render::Camera cam = marineDriveFacingNorth();

    // Three probes, 300 m away: one dead ahead (north), one 200 m west (the sea),
    // one 200 m east (the city).  All three are well inside a 16:9 60-degree
    // frustum, so all three must project to finite NDC inside [-1, 1].
    const Screen north = onScreen(cam, Vec3{0.0f, 1.7f, 300.0f}, aspect);
    const Screen west  = onScreen(cam, Vec3{-200.0f, 1.7f, 300.0f}, aspect);
    const Screen east  = onScreen(cam, Vec3{ 200.0f, 1.7f, 300.0f}, aspect);

    CHECK(north.inFront());
    CHECK(west.inFront());
    CHECK(east.inFront());
    CHECK(std::fabs(north.x) < 1e-5f);          // straight ahead is the centre
    CHECK(west.x < -0.1f);                      // the sea is on the LEFT
    CHECK(east.x >  0.1f);                      // the city is on the RIGHT

    // And the mirror of that check, stated as the failure it prevents: if the
    // basis were built the textbook way (`forward x up`), west would come out on
    // the right.  This is the assertion that would have caught it.
    CHECK(west.x < east.x);
}

TEST(cameraKeepsTheSkyUpAndTheGroundDown) {
    const f32 aspect = 1.0f;
    const render::Camera cam = marineDriveFacingNorth();

    // Vulkan's clip-space +Y points down the framebuffer, so the projection flips
    // Y: a point *above* the camera must land at negative ndc.y (the top of the
    // frame) and one below at positive ndc.y.  Getting this backwards is the
    // classic first-port symptom: the city hangs from the sky.
    const Screen above = onScreen(cam, Vec3{0.0f, 30.0f, 100.0f}, aspect);
    const Screen below = onScreen(cam, Vec3{0.0f, -10.0f + 1.7f, 100.0f}, aspect);
    CHECK(above.inFront());
    CHECK(below.inFront());
    CHECK(above.y < -0.1f);
    CHECK(below.y >  0.05f);

    // The 60-degree vertical field of view: a point exactly on the frustum's
    // upper edge projects to the top of the frame.  tan(30 degrees) = 0.5774.
    const f32 d = 200.0f;
    const f32 h = d * std::tan(0.5f * cam.fovY);
    const Screen edge = onScreen(cam, Vec3{0.0f, 1.7f + h, d}, aspect);
    CHECK(edge.inFront());
    CHECK_NEAR(edge.y, -1.0, 2e-4);
}

TEST(cameraDepthRangeIsVulkansZeroToOne) {
    const render::Camera cam = marineDriveFacingNorth();
    const f32 aspect = 1.0f;

    // A point on the near plane must map to ndc.z = 0 and one on the far plane to
    // ndc.z = 1.  OpenGL's [-1, 1] convention here would put half the world in
    // front of the near plane and blank the frame after the depth test.
    const Screen atNear = onScreen(cam, Vec3{0.0f, 1.7f, cam.zNear}, aspect);
    const Screen atFar  = onScreen(cam, Vec3{0.0f, 1.7f, cam.zFar}, aspect);
    CHECK_NEAR(atNear.z, 0.0, 1e-4);
    CHECK_NEAR(atFar.z, 1.0, 1e-4);

    // A point behind the camera has negative w: it must be clipped, not drawn
    // upside down in front (the symptom of an inverted forward axis).
    const Screen behind = onScreen(cam, Vec3{0.0f, 1.7f, -50.0f}, aspect);
    CHECK_FALSE(behind.inFront());
}

TEST(cameraYawTurnsRightForPositiveAnglesAndMatchesTheShadersConvention) {
    // One convention, stated once and checked here: forward = (sin yaw, *, cos yaw),
    // so yaw 0 is north (+z), yaw pi/2 is east (+x), yaw -pi/2 is west (-x).  The
    // traffic vertex shader, the viewer's ride camera and the simulation's heading
    // all use the same formula; this is where it is pinned down.
    render::Camera cam = marineDriveFacingNorth();
    const struct { f32 yaw; f32 x; f32 z; } cases[] = {
        {0.0f,           0.0f,  1.0f},
        {1.57079633f,    1.0f,  0.0f},
        {-1.57079633f,  -1.0f,  0.0f},
        {3.14159265f,    0.0f, -1.0f},
    };
    for (const auto& c : cases) {
        cam.yaw = c.yaw;
        const Vec3 f = cam.forward();
        CHECK_NEAR(f.x, c.x, 2e-6);
        CHECK_NEAR(f.z, c.z, 2e-6);
        CHECK_NEAR(f.y, 0.0, 2e-6);
    }

    // Positive yaw is a RIGHT turn: facing east (yaw = pi/2), north is on the left.
    // The probe sits 30 degrees off-axis — inside the frustum, which a point exactly
    // 90 degrees to the side is not (it lands on the side plane, w = 0, and clipping
    // it is correct behaviour rather than a failure).
    cam.yaw = 1.57079633f;
    const Screen northOfEast = onScreen(cam, Vec3{170.0f, 1.7f, 100.0f}, 16.0f / 9.0f);
    CHECK(northOfEast.inFront());
    CHECK(northOfEast.x < -0.1f);

    // ...and the reverse: facing west, north is on the right.
    cam.yaw = -1.57079633f;
    const Screen northOfWest = onScreen(cam, Vec3{-170.0f, 1.7f, 100.0f}, 16.0f / 9.0f);
    CHECK(northOfWest.inFront());
    CHECK(northOfWest.x > 0.1f);
}

TEST(cameraIsRobustLookingStraightUpAndDown) {
    // The right-hand vector degenerates when forward is parallel to up.  A camera
    // that returns NaN there produces a black frame with no clue why; this one
    // picks an axis and stays finite.
    const f32 aspect = 1.0f;
    for (f32 pitch : {1.57079633f, -1.57079633f, 1.5f, -1.5f}) {
        render::Camera cam = marineDriveFacingNorth();
        cam.pitch = pitch;
        const Mat4 v = cam.view();
        for (int i = 0; i < 16; ++i) CHECK(std::isfinite(v.m[i]));
        const Vec4 clip = cam.proj(aspect) * (v * Vec4{0.0f, 1.7f, 5.0f, 1.0f});
        CHECK(std::isfinite(clip.x) && std::isfinite(clip.y) && std::isfinite(clip.w));
    }
}

TEST(cameraViewMatrixIsRigidAndMapsTheEyeToTheOrigin) {
    render::Camera cam = marineDriveFacingNorth();
    cam.pos = Vec3{123.5f, 62.25f, -840.75f};
    cam.yaw = 0.7f;
    cam.pitch = -0.12f;
    const Mat4 v = cam.view();

    // The eye maps to the view origin.
    const Vec4 eye = v * Vec4{cam.pos.x, cam.pos.y, cam.pos.z, 1.0f};
    CHECK_NEAR(eye.x, 0.0, 1e-4);
    CHECK_NEAR(eye.y, 0.0, 1e-4);
    CHECK_NEAR(eye.z, 0.0, 1e-4);

    // Distances are preserved: a rigid transform, so the ground never breathes as
    // the camera turns.  (A shear or a non-unit basis here is a stretched city.)
    const Vec3 a{10.0f, 5.0f, 20.0f}, b{40.0f, 5.0f, 90.0f};
    const Vec4 pa = v * Vec4{a.x, a.y, a.z, 1.0f};
    const Vec4 pb = v * Vec4{b.x, b.y, b.z, 1.0f};
    const Vec3 va{pa.x, pa.y, pa.z}, vb{pb.x, pb.y, pb.z};
    CHECK_NEAR((va - vb).length(), (a - b).length(), 1e-3);

    // A point 100 m ahead is 100 m ahead in view space (i.e. z = -100), never +100.
    const Vec3 f = cam.forward();
    const Vec3 ahead = cam.pos + f * 100.0f;
    const Vec4 va2 = v * Vec4{ahead.x, ahead.y, ahead.z, 1.0f};
    CHECK_NEAR(va2.z, -100.0, 1e-3);
}
