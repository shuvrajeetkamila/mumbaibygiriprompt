// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/math.cpp
// =============================================================================
#include "engine/core/math.h"

namespace mumbai {

// -----------------------------------------------------------------------------
//  Matrices
// -----------------------------------------------------------------------------
Mat4 Mat4::perspective(f32 fovYRadians, f32 aspect, f32 nearZ, f32 farZ) {
    Mat4 r;
    std::memset(r.m, 0, sizeof(r.m));
    // Vulkan convention: clip-space depth in [0,1], Y flipped by the projector
    // so that +Y is up in world space (see DOCS/04_RUNTIME.md).
    const f32 f = 1.0f / std::tan(fovYRadians * 0.5f);
    r.m[0]  = f / aspect;
    r.m[5]  = -f;
    r.m[10] = farZ / (nearZ - farZ);
    r.m[11] = -1.0f;
    r.m[14] = (farZ * nearZ) / (nearZ - farZ);
    return r;
}

Mat4 Mat4::orthographic(f32 l, f32 r_, f32 b, f32 t, f32 n, f32 f) {
    Mat4 o;
    std::memset(o.m, 0, sizeof(o.m));
    o.m[0]  = 2.0f / (r_ - l);
    o.m[5]  = -2.0f / (t - b);
    o.m[10] = 1.0f / (n - f);
    o.m[12] = -(r_ + l) / (r_ - l);
    o.m[13] = (t + b) / (t - b);
    o.m[14] = n / (n - f);
    o.m[15] = 1.0f;
    return o;
}

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = (center - eye).normalized();   // forward
    const Vec3 s = f.cross(up).normalized();      // right
    const Vec3 u = s.cross(f);                    // true up
    Mat4 r;
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;  r.m[12] = -s.dot(eye);
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;  r.m[13] = -u.dot(eye);
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z; r.m[14] =  f.dot(eye);
    r.m[3] = 0;    r.m[7] = 0;    r.m[11] = 0;    r.m[15] =  1.0f;
    return r;
}

Mat4 Mat4::fromQuat(const Vec4& q) {
    const f32 x = q.x, y = q.y, z = q.z, w = q.w;
    const f32 x2 = x + x, y2 = y + y, z2 = z + z;
    const f32 xx = x * x2, xy = x * y2, xz = x * z2;
    const f32 yy = y * y2, yz = y * z2, zz = z * z2;
    const f32 wx = w * x2, wy = w * y2, wz = w * z2;
    Mat4 r;
    r.m[0] = 1 - (yy + zz); r.m[1] = xy + wz;       r.m[2] = xz - wy;       r.m[3] = 0;
    r.m[4] = xy - wz;       r.m[5] = 1 - (xx + zz); r.m[6] = yz + wx;       r.m[7] = 0;
    r.m[8] = xz + wy;       r.m[9] = yz - wx;       r.m[10] = 1 - (xx + yy); r.m[11] = 0;
    r.m[12] = 0; r.m[13] = 0; r.m[14] = 0; r.m[15] = 1;
    return r;
}

Mat4 Mat4::rotationAxis(const Vec3& axis, f32 radians) {
    const Vec3 a = axis.normalized();
    const f32 c = std::cos(radians), s = std::sin(radians), t = 1 - c;
    Mat4 r;
    r.m[0] = t * a.x * a.x + c;       r.m[1] = t * a.x * a.y + s * a.z; r.m[2] = t * a.x * a.z - s * a.y;
    r.m[4] = t * a.x * a.y - s * a.z; r.m[5] = t * a.y * a.y + c;       r.m[6] = t * a.y * a.z + s * a.x;
    r.m[8] = t * a.x * a.z + s * a.y; r.m[9] = t * a.y * a.z - s * a.x; r.m[10] = t * a.z * a.z + c;
    return r;
}

Mat4 Mat4::trs(const Vec3& t, const Vec4& q, const Vec3& s) {
    Mat4 r = fromQuat(q);
    for (int c = 0; c < 3; ++c) {
        r.m[c * 4 + 0] *= s.x;
        r.m[c * 4 + 1] *= s.y;
        r.m[c * 4 + 2] *= s.z;
    }
    r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
    return r;
}

Mat4 Mat4::inverse() const {
    const f32* a = m;
    f32 inv[16];
    inv[0]  =  a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15] + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
    inv[4]  = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15] - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
    inv[8]  =  a[4]*a[9]*a[15]  - a[4]*a[11]*a[13] - a[8]*a[5]*a[15] + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
    inv[12] = -a[4]*a[9]*a[14]  + a[4]*a[10]*a[13] + a[8]*a[5]*a[14] - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
    inv[1]  = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15] - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
    inv[5]  =  a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15] + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
    inv[9]  = -a[0]*a[9]*a[15]  + a[0]*a[11]*a[13] + a[8]*a[1]*a[15] - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
    inv[13] =  a[0]*a[9]*a[14]  - a[0]*a[10]*a[13] - a[8]*a[1]*a[14] + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
    inv[2]  =  a[1]*a[6]*a[15]  - a[1]*a[7]*a[14]  - a[5]*a[2]*a[15] + a[5]*a[3]*a[14] + a[13]*a[2]*a[7]  - a[13]*a[3]*a[6];
    inv[6]  = -a[0]*a[6]*a[15]  + a[0]*a[7]*a[14]  + a[4]*a[2]*a[15] - a[4]*a[3]*a[14] - a[12]*a[2]*a[7]  + a[12]*a[3]*a[6];
    inv[10] =  a[0]*a[5]*a[15]  - a[0]*a[7]*a[13]  - a[4]*a[1]*a[15] + a[4]*a[3]*a[13] + a[12]*a[1]*a[7]  - a[12]*a[3]*a[5];
    inv[14] = -a[0]*a[5]*a[14]  + a[0]*a[6]*a[13]  + a[4]*a[1]*a[14] - a[4]*a[2]*a[13] - a[12]*a[1]*a[6]  + a[12]*a[2]*a[5];
    inv[3]  = -a[1]*a[6]*a[11]  + a[1]*a[7]*a[10]  + a[5]*a[2]*a[11] - a[5]*a[3]*a[10] - a[9]*a[2]*a[7]   + a[9]*a[3]*a[6];
    inv[7]  =  a[0]*a[6]*a[11]  - a[0]*a[7]*a[10]  - a[4]*a[2]*a[11] + a[4]*a[3]*a[10] + a[8]*a[2]*a[7]   - a[8]*a[3]*a[6];
    inv[11] = -a[0]*a[5]*a[11]  + a[0]*a[7]*a[9]   + a[4]*a[1]*a[11] - a[4]*a[3]*a[9]  - a[8]*a[1]*a[7]   + a[8]*a[3]*a[5];
    inv[15] =  a[0]*a[5]*a[10]  - a[0]*a[6]*a[9]   - a[4]*a[1]*a[10] + a[4]*a[2]*a[9]  + a[8]*a[1]*a[6]   - a[8]*a[2]*a[5];
    f32 det = a[0]*inv[0] + a[1]*inv[4] + a[2]*inv[8] + a[3]*inv[12];
    Mat4 out;
    if (std::fabs(det) < 1e-20f) return out;   // singular: identity, caller fault
    det = 1.0f / det;
    for (int i = 0; i < 16; ++i) out.m[i] = inv[i] * det;
    return out;
}

// -----------------------------------------------------------------------------
//  Quaternions
// -----------------------------------------------------------------------------
Vec4 quatIdentity() { return {0, 0, 0, 1}; }

Vec4 quatFromAxisAngle(const Vec3& axis, f32 radians) {
    const Vec3 a = axis.normalized();
    const f32 h = radians * 0.5f, s = std::sin(h);
    return {a.x * s, a.y * s, a.z * s, std::cos(h)};
}

Vec4 quatMultiply(const Vec4& a, const Vec4& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

Vec4 quatNormalize(const Vec4& q) {
    const f32 l = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (l < 1e-12f) return quatIdentity();
    const f32 inv = 1.0f / l;
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

Vec4 quatFromEuler(f32 yaw, f32 pitch, f32 roll) {
    const f32 cy = std::cos(yaw * 0.5f),   sy = std::sin(yaw * 0.5f);
    const f32 cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const f32 cr = std::cos(roll * 0.5f),  sr = std::sin(roll * 0.5f);
    Vec4 q;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.w = cr * cp * cy + sr * sp * sy;
    return q;
}

Vec3 quatRotate(const Vec4& q, const Vec3& v) {
    // v' = v + 2w(q x v) + 2(q x (q x v))   — cheaper and more stable than
    // building a matrix for the hot paths in physics and traffic.
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = qv.cross(v) * 2.0f;
    return v + t * q.w + qv.cross(t);
}

Vec4 quatSlerp(const Vec4& a, const Vec4& b, f32 t) {
    f32 d = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    Vec4 bb = b;
    if (d < 0.0f) { bb = {-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f) {
        return quatNormalize({lerp(a.x, bb.x, t), lerp(a.y, bb.y, t),
                              lerp(a.z, bb.z, t), lerp(a.w, bb.w, t)});
    }
    const f32 theta0 = std::acos(clamp(d, -1.0f, 1.0f));
    const f32 theta = theta0 * t;
    const f32 s0 = std::sin(theta0 - theta) / std::sin(theta0);
    const f32 s1 = std::sin(theta) / std::sin(theta0);
    return quatNormalize({a.x*s0 + bb.x*s1, a.y*s0 + bb.y*s1, a.z*s0 + bb.z*s1, a.w*s0 + bb.w*s1});
}

Vec4 quatLookRotation(const Vec3& forward, const Vec3& up) {
    const Vec3 f = forward.normalized();
    if (f.lengthSq() < 1e-12f) return quatIdentity();
    Vec3 r = f.cross(up);
    if (r.lengthSq() < 1e-8f) {                    // forward parallel to up
        r = f.cross(Vec3{1, 0, 0});
        if (r.lengthSq() < 1e-8f) r = f.cross(Vec3{0, 0, 1});
    }
    r = r.normalized();
    const Vec3 u = r.cross(f);
    // Build from basis (r, u, -f) then convert matrix -> quaternion.
    const f32 m00 = r.x,        m01 = u.x,        m02 = -f.x;
    const f32 m10 = r.y,        m11 = u.y,        m12 = -f.y;
    const f32 m20 = r.z,        m21 = u.z,        m22 = -f.z;
    const f32 trace = m00 + m11 + m22;
    Vec4 q;
    if (trace > 0.0f) {
        const f32 s = std::sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const f32 s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s; q.x = 0.25f * s;
        q.y = (m01 + m10) / s; q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const f32 s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s; q.x = (m01 + m10) / s;
        q.y = 0.25f * s;       q.z = (m12 + m21) / s;
    } else {
        const f32 s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s; q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s; q.z = 0.25f * s;
    }
    return quatNormalize(q);
}

// -----------------------------------------------------------------------------
//  Frustum
// -----------------------------------------------------------------------------
Frustum Frustum::fromViewProjection(const Mat4& vp) {
    Frustum f;
    const f32* m = vp.m;   // column-major: m[col*4+row]
    auto row = [&](int r) { return Vec4{m[r], m[4 + r], m[8 + r], m[12 + r]}; };
    const Vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
    auto norm = [](Vec4 p) {
        const f32 l = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (l > 1e-20f) { p.x /= l; p.y /= l; p.z /= l; p.w /= l; }
        return p;
    };
    f.planes[0].p = norm(r3 + r0);   // left
    f.planes[1].p = norm(r3 - r0);   // right
    f.planes[2].p = norm(r3 + r1);   // bottom
    f.planes[3].p = norm(r3 - r1);   // top
    f.planes[4].p = norm(r2);        // near  (Vulkan [0,1] depth: z >= 0)
    f.planes[5].p = norm(r3 - r2);   // far
    return f;
}

bool Frustum::intersectsSphere(const Vec3& c, f32 r) const {
    for (const Plane& p : planes)
        if (p.distance(c) < -r) return false;
    return true;
}

bool Frustum::intersectsAabb(const Aabb& b) const {
    for (const Plane& p : planes) {
        // Positive-vertex test: if the box is entirely behind any plane, cull.
        const Vec3 positive{
            p.p.x >= 0 ? b.max.x : b.min.x,
            p.p.y >= 0 ? b.max.y : b.min.y,
            p.p.z >= 0 ? b.max.z : b.min.z
        };
        if (p.distance(positive) < 0.0f) return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
//  Ray intersections
// -----------------------------------------------------------------------------
bool rayAabb(const Ray& r, const Aabb& b, f32& tNear, f32& tFar) {
    f32 t0 = -1e30f, t1 = 1e30f;
    for (int i = 0; i < 3; ++i) {
        const f32 o = (&r.origin.x)[i];
        const f32 d = (&r.direction.x)[i];
        const f32 mn = (&b.min.x)[i];
        const f32 mx = (&b.max.x)[i];
        if (std::fabs(d) < 1e-12f) {
            if (o < mn || o > mx) return false;
        } else {
            f32 a = (mn - o) / d, c = (mx - o) / d;
            if (a > c) std::swap(a, c);
            t0 = std::max(t0, a);
            t1 = std::min(t1, c);
            if (t0 > t1) return false;
        }
    }
    tNear = t0; tFar = t1;
    return t1 >= 0.0f;
}

bool rayTriangle(const Ray& r, const Vec3& a, const Vec3& b, const Vec3& c, f32& t) {
    // Möller–Trumbore
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 p = r.direction.cross(e2);
    const f32 det = e1.dot(p);
    if (std::fabs(det) < 1e-12f) return false;
    const f32 invDet = 1.0f / det;
    const Vec3 tv = r.origin - a;
    const f32 u = tv.dot(p) * invDet;
    if (u < -1e-6f || u > 1.0f + 1e-6f) return false;
    const Vec3 q = tv.cross(e1);
    const f32 v = r.direction.dot(q) * invDet;
    if (v < -1e-6f || u + v > 1.0f + 1e-6f) return false;
    t = e2.dot(q) * invDet;
    return t > 1e-6f;
}

bool rayPlane(const Ray& r, const Vec3& pointOnPlane, const Vec3& normal, f32& t) {
    const f32 denom = normal.dot(r.direction);
    if (std::fabs(denom) < 1e-9f) return false;
    t = (pointOnPlane - r.origin).dot(normal) / denom;
    return t >= 0.0f;
}

// -----------------------------------------------------------------------------
//  Splines
// -----------------------------------------------------------------------------
Vec3 bezierEval(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t) {
    const f32 u = 1.0f - t;
    const f32 b0 = u * u * u;
    const f32 b1 = 3.0f * u * u * t;
    const f32 b2 = 3.0f * u * t * t;
    const f32 b3 = t * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

Vec3 bezierTangent(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t) {
    const f32 u = 1.0f - t;
    return (p1 - p0) * (3.0f * u * u) + (p2 - p1) * (6.0f * u * t) + (p3 - p2) * (3.0f * t * t);
}

Vec3 bezierSecondDerivative(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t) {
    const f32 u = 1.0f - t;
    return (p2 - p1 * 2.0f + p0) * (6.0f * u) + (p3 - p2 * 2.0f + p1) * (6.0f * t);
}

Vec3 catmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, f32 t) {
    // Converted to the equivalent cubic Bezier so that downstream code has a
    // single evaluation path (important for road profile extrusion consistency).
    const Vec3 b0 = p1;
    const Vec3 b1 = p1 + (p2 - p0) * (1.0f / 6.0f);
    const Vec3 b2 = p2 - (p3 - p1) * (1.0f / 6.0f);
    const Vec3 b3 = p2;
    return bezierEval(b0, b1, b2, b3, t);
}

// -----------------------------------------------------------------------------
//  Geographic distance
// -----------------------------------------------------------------------------
f64 haversineMetres(f64 lat1, f64 lon1, f64 lat2, f64 lon2) {
    constexpr f64 R = 6371008.8;   // IUGG mean Earth radius
    const f64 p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    const f64 dp = (lat2 - lat1) * DEG2RAD;
    const f64 dl = (lon2 - lon1) * DEG2RAD;
    const f64 a = std::sin(dp * 0.5) * std::sin(dp * 0.5) +
                  std::cos(p1) * std::cos(p2) * std::sin(dl * 0.5) * std::sin(dl * 0.5);
    return 2.0 * R * std::asin(std::min(1.0, std::sqrt(a)));
}

} // namespace mumbai
