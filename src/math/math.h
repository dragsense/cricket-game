#pragma once
// Math layer. Thin wrapper over GLM: the rest of the codebase includes this
// header and uses the math:: names, never <glm/...> directly, so GLM stays an
// implementation detail.
//
// Conventions:
//   * Right-handed, +Y up, 1 unit = 1 metre.
//   * Character/bone "forward" is local +Z (see lookRotation).
//   * Matrices are column-major with column vectors (p' = M * p), same layout
//     as Filament, so they can be copied straight into the renderer.
//   * Quaternion composition (a * b) applies b first, then a.
//
// GLM_FORCE_* / GLM_ENABLE_EXPERIMENTAL are set by the `math` CMake target so
// every translation unit sees the same configuration.

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

namespace math {

using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;
using Quat = glm::quat;  // NOTE: GLM's constructor order is (w, x, y, z).
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 2.0f * kPi;
inline constexpr float kHalfPi = 0.5f * kPi;
inline constexpr float kEpsilon = 1e-6f;

inline constexpr Vec3 kUp{0, 1, 0};
inline constexpr Vec3 kRight{1, 0, 0};
inline constexpr Vec3 kForward{0, 0, 1};

using glm::clamp;
using glm::cross;
using glm::degrees;
using glm::distance;
using glm::dot;
using glm::inverse;
using glm::length;
using glm::length2;
using glm::radians;
using glm::slerp;

inline Quat identityQuat() { return Quat(1, 0, 0, 0); }

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

// Normalize that returns `fallback` for (near) zero-length input instead of NaN.
inline Vec3 safeNormalize(Vec3 v, Vec3 fallback = Vec3(0)) {
    float len2 = glm::length2(v);
    return len2 > kEpsilon * kEpsilon ? v / std::sqrt(len2) : fallback;
}
inline Quat normalize(Quat q) { return glm::normalize(q); }
inline Vec3 normalize(Vec3 v) { return glm::normalize(v); }

inline Quat fromAxisAngle(Vec3 unitAxis, float radians) { return glm::angleAxis(radians, unitAxis); }

// Intrinsic yaw (about Y), then pitch (about local X), then roll (about local Z).
inline Quat fromYawPitchRoll(float yaw, float pitch, float roll) {
    return fromAxisAngle(kUp, yaw) * fromAxisAngle(kRight, pitch) * fromAxisAngle(kForward, roll);
}

// Rotation mapping local +Z onto `forward`, keeping local +Y as close to `up`
// as possible. Parallel forward/up falls back to a stable alternative axis.
inline Quat lookRotation(Vec3 forward, Vec3 up = kUp) {
    Vec3 f = safeNormalize(forward, kForward);
    Vec3 r = cross(up, f);
    if (glm::length2(r) < 1e-8f) r = cross(std::fabs(f.y) < 0.99f ? kUp : kRight, f);
    r = glm::normalize(r);
    Vec3 u = cross(f, r);
    return glm::normalize(glm::quat_cast(Mat3(r, u, f)));
}

// Shortest-arc rotation taking unit vector `from` onto unit vector `to`.
inline Quat fromTo(Vec3 from, Vec3 to) {
    float c = dot(from, to);
    if (c < -1.0f + 1e-6f) {
        Vec3 axis = cross(kRight, from);
        if (glm::length2(axis) < 1e-6f) axis = cross(kUp, from);
        return fromAxisAngle(glm::normalize(axis), kPi);
    }
    Vec3 a = cross(from, to);
    return glm::normalize(Quat(1.0f + c, a.x, a.y, a.z));
}

// Normalized lerp along the shortest arc. Cheaper than slerp; fine for small
// steps such as per-tick blending.
inline Quat nlerp(Quat a, Quat b, float t) {
    if (dot(a, b) < 0.0f) b = -b;
    return glm::normalize(Quat(lerp(a.w, b.w, t), lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t)));
}

// Angle in radians of the rotation taking `a` to `b`, in [0, pi].
inline float angleBetween(Quat a, Quat b) {
    float c = std::fabs(dot(glm::normalize(a), glm::normalize(b)));
    return 2.0f * std::acos(c > 1.0f ? 1.0f : c);
}

// Splits q into q = swing * twist, where `twist` rotates about `unitAxis` and
// `swing` rotates about an axis perpendicular to it. Used for joint limits
// (e.g. forearm twist vs. elbow bend).
inline void swingTwist(Quat q, Vec3 unitAxis, Quat& swing, Quat& twist) {
    Vec3 v(q.x, q.y, q.z);
    Vec3 p = unitAxis * dot(v, unitAxis);  // project vector part onto the axis
    twist = Quat(q.w, p.x, p.y, p.z);
    float len2 = twist.w * twist.w + glm::length2(p);
    if (len2 < 1e-12f) {
        twist = identityQuat();  // 180 degree swing: twist is undefined, pick identity
    } else {
        twist = twist * (1.0f / std::sqrt(len2));
    }
    swing = q * glm::conjugate(twist);
}

inline bool nearlyEqual(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }
inline bool nearlyEqual(Vec3 a, Vec3 b, float eps = 1e-5f) {
    return glm::all(glm::lessThanEqual(glm::abs(a - b), Vec3(eps)));
}
// q and -q are the same rotation.
inline bool nearlyEqual(Quat a, Quat b, float eps = 1e-5f) {
    return std::fabs(std::fabs(dot(a, b)) - 1.0f) <= eps;
}
inline bool nearlyEqual(const Mat4& a, const Mat4& b, float eps = 1e-4f) {
    for (int c = 0; c < 4; ++c)
        if (!glm::all(glm::lessThanEqual(glm::abs(a[c] - b[c]), Vec4(eps)))) return false;
    return true;
}

inline Mat4 translation(Vec3 t) { return glm::translate(Mat4(1.0f), t); }
inline Mat4 rotation(Quat q) { return glm::mat4_cast(q); }

// Translation * Rotation * Scale.
inline Mat4 trs(Vec3 t, Quat r, Vec3 s) {
    Mat4 m = glm::mat4_cast(r);
    m[0] *= s.x;
    m[1] *= s.y;
    m[2] *= s.z;
    m[3] = Vec4(t, 1.0f);
    return m;
}

inline const float* data(const Mat4& m) { return glm::value_ptr(m); }

} // namespace math

#include "math/transform.h"
