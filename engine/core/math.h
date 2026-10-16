// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/math.h
//  Vector/matrix/quaternion/AABB primitives used by every subsystem.
//
//  Precision policy (see DOCS/04_RUNTIME.md, decision D-004):
//    * World-space geography is f64 (metres) in the pipeline.
//    * Runtime entity geography is GlobalPosition (int64 cells + f32 local).
//    * Render-space and collision maths are f32 but are only ever handed
//      camera-local coordinates below 4096 m, so f32 ulp stays < 1 mm.
// =============================================================================
#pragma once
#include "engine/core/core.h"

namespace mumbai {

struct Vec2 {
    f32 x = 0, y = 0;
    Vec2() = default;
    Vec2(f32 x_, f32 y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(f32 s) const { return {x * s, y * s}; }
    Vec2 operator/(f32 s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    f32 dot(const Vec2& o) const { return x * o.x + y * o.y; }
    f32 cross(const Vec2& o) const { return x * o.y - y * o.x; }
    f32 length() const { return std::sqrt(x * x + y * y); }
    f32 lengthSq() const { return x * x + y * y; }
    Vec2 normalized() const { f32 l = length(); return l > 1e-12f ? Vec2{x / l, y / l} : Vec2{0, 0}; }
    Vec2 perpendicular() const { return {-y, x}; }   // left-hand 90 deg rotation
};

struct Vec2d {
    f64 x = 0, y = 0;
    Vec2d() = default;
    Vec2d(f64 x_, f64 y_) : x(x_), y(y_) {}
    Vec2d operator+(const Vec2d& o) const { return {x + o.x, y + o.y}; }
    Vec2d operator-(const Vec2d& o) const { return {x - o.x, y - o.y}; }
    Vec2d operator*(f64 s) const { return {x * s, y * s}; }
    Vec2d operator/(f64 s) const { return {x / s, y / s}; }
    f64 dot(const Vec2d& o) const { return x * o.x + y * o.y; }
    f64 cross(const Vec2d& o) const { return x * o.y - y * o.x; }
    f64 length() const { return std::sqrt(x * x + y * y); }
    f64 lengthSq() const { return x * x + y * y; }
    Vec2d normalized() const { f64 l = length(); return l > 1e-15 ? Vec2d{x / l, y / l} : Vec2d{0, 0}; }
    Vec2d perpendicular() const { return {-y, x}; }
    Vec2 toF32() const { return {static_cast<f32>(x), static_cast<f32>(y)}; }
};

struct Vec3 {
    f32 x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(f32 x_, f32 y_, f32 z_) : x(x_), y(y_), z(z_) {}
    explicit Vec3(f32 s) : x(s), y(s), z(s) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(f32 s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(f32 s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(f32 s) { x *= s; y *= s; z *= s; return *this; }
    f32 dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    f32 length() const { return std::sqrt(x * x + y * y + z * z); }
    f32 lengthSq() const { return x * x + y * y + z * z; }
    f32 lengthXZ() const { return std::sqrt(x * x + z * z); }
    Vec3 normalized() const { f32 l = length(); return l > 1e-12f ? Vec3{x / l, y / l, z / l} : Vec3{0, 0, 0}; }
    Vec2 xz() const { return {x, z}; }
};

struct Vec4 {
    f32 x = 0, y = 0, z = 0, w = 0;
    Vec4() = default;
    Vec4(f32 x_, f32 y_, f32 z_, f32 w_) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4 operator+(const Vec4& o) const { return {x + o.x, y + o.y, z + o.z, w + o.w}; }
    Vec4 operator-(const Vec4& o) const { return {x - o.x, y - o.y, z - o.z, w - o.w}; }
    Vec4 operator-() const { return {-x, -y, -z, -w}; }
    Vec4 operator*(f32 s) const { return {x * s, y * s, z * s, w * s}; }
    Vec4 operator/(f32 s) const { return {x / s, y / s, z / s, w / s}; }
    f32 dot(const Vec4& o) const { return x * o.x + y * o.y + z * o.z + w * o.w; }
};

// Column-major 4x4, matching the memory layout consumed by Vulkan/GLSL and by
// the viewer's WebGL path (column-major Float32Array upload).
struct Mat4 {
    f32 m[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    static Mat4 identity() { return Mat4{}; }
    static Mat4 translation(const Vec3& t) {
        Mat4 r; r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z; return r;
    }
    static Mat4 scale(const Vec3& s) {
        Mat4 r; r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z; return r;
    }
    static Mat4 perspective(f32 fovYRadians, f32 aspect, f32 nearZ, f32 farZ);
    static Mat4 orthographic(f32 l, f32 r, f32 b, f32 t, f32 n, f32 f);
    // The textbook right-handed lookAt (screen right = forward x up).  Correct
    // for a right-handed world; this project's world is left-handed (x east,
    // y up, z north), so the renderer's camera builds its own basis instead of
    // calling this — see engine/render/camera.cpp and D-044.  Kept because the
    // maths is right and something may want a right-handed view later.
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
    static Mat4 fromQuat(const Vec4& q);          // q = (x,y,z,w)
    static Mat4 rotationAxis(const Vec3& axis, f32 radians);
    static Mat4 trs(const Vec3& t, const Vec4& q, const Vec3& s);

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int row = 0; row < 4; ++row) {
                f32 sum = 0;
                for (int k = 0; k < 4; ++k) sum += m[k * 4 + row] * o.m[c * 4 + k];
                r.m[c * 4 + row] = sum;
            }
        return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0] * v.x + m[4] * v.y + m[8]  * v.z + m[12] * v.w,
            m[1] * v.x + m[5] * v.y + m[9]  * v.z + m[13] * v.w,
            m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w,
            m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w
        };
    }
    Vec3 transformPoint(const Vec3& p) const {
        Vec4 r = (*this) * Vec4{p.x, p.y, p.z, 1.0f};
        return {r.x, r.y, r.z};
    }
    Mat4 inverse() const;
    void toFloatArray(f32 out[16]) const { std::memcpy(out, m, sizeof(m)); }
};

// Quaternion helpers (x, y, z, w) — used for entity orientation and for the
// floating-origin shift, which must not introduce rotational drift.
Vec4 quatIdentity();
Vec4 quatFromAxisAngle(const Vec3& axis, f32 radians);
Vec4 quatMultiply(const Vec4& a, const Vec4& b);
Vec4 quatNormalize(const Vec4& q);
Vec4 quatFromEuler(f32 yaw, f32 pitch, f32 roll);
Vec3 quatRotate(const Vec4& q, const Vec3& v);
Vec4 quatSlerp(const Vec4& a, const Vec4& b, f32 t);
Vec4 quatLookRotation(const Vec3& forward, const Vec3& up);

struct Aabb {
    Vec3 min{ 1e30f,  1e30f,  1e30f};
    Vec3 max{-1e30f, -1e30f, -1e30f};
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }
    void expand(const Aabb& o) { expand(o.min); expand(o.max); }
    void expandBy(f32 r) { min -= Vec3(r); max += Vec3(r); }
    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return max - min; }
    f32 radius() const { return extent().length() * 0.5f; }
    bool valid() const { return max.x >= min.x && max.y >= min.y && max.z >= min.z; }
    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
    bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

// Frustum, extracted from a view-projection matrix for the culling system.
struct Plane { Vec4 p; f32 distance(const Vec3& v) const { return p.x * v.x + p.y * v.y + p.z * v.z + p.w; } };
struct Frustum {
    Plane planes[6];
    static Frustum fromViewProjection(const Mat4& vp);
    bool intersectsSphere(const Vec3& c, f32 r) const;
    bool intersectsAabb(const Aabb& b) const;
};

// Ray/geometry intersections used by world queries (ground height, picking,
// collision fallbacks in the physics layer).
struct Ray { Vec3 origin; Vec3 direction; };
bool rayAabb(const Ray& r, const Aabb& b, f32& tNear, f32& tFar);
bool rayTriangle(const Ray& r, const Vec3& a, const Vec3& b, const Vec3& c, f32& t);
bool rayPlane(const Ray& r, const Vec3& pointOnPlane, const Vec3& normal, f32& t);

// --- Spline evaluation (roads, rails, cables, routes) -----------------------
// Cubic Bezier, evaluated in f32 render space and in f64 pipeline space.
Vec3 bezierEval(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t);
Vec3 bezierTangent(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t);
Vec3 bezierSecondDerivative(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t);
// Catmull-Rom through control points, converted to a Bezier for evaluation.
Vec3 catmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t);

// Removes the NaN/Inf class of defect that the spec forbids outright.
inline bool isFinite(const Vec3& v) { return isFinite(v.x) && isFinite(v.y) && isFinite(v.z); }
inline bool isFinite(const Vec2& v) { return isFinite(v.x) && isFinite(v.y); }
inline bool isFinite(const Vec4& v) { return isFinite(v.x) && isFinite(v.y) && isFinite(v.z) && isFinite(v.w); }

// Geographic helpers (great-circle distances used for data validation against
// source coordinates and for the traffic demand model).
f64 haversineMetres(f64 lat1, f64 lon1, f64 lat2, f64 lon2);

} // namespace mumbai
