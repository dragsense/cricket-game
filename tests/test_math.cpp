// Tests for our math helpers on top of GLM, plus the GLM conventions we rely on
// (so a GLM upgrade that changed them would be caught here).

#include "math/math.h"

#include <doctest/doctest.h>

using namespace math;

namespace {
constexpr float kTol = 1e-5f;

bool vecEq(Vec3 a, Vec3 b, float eps = kTol) { return nearlyEqual(a, b, eps); }
} // namespace

TEST_CASE("quaternion basics") {
    Quat q = fromAxisAngle(kUp, kHalfPi);
    CHECK(vecEq(q * Vec3(1, 0, 0), Vec3(0, 0, -1)));  // right-handed: +X -> -Z about +Y

    SUBCASE("composition applies right operand first") {
        Quat a = fromAxisAngle(kUp, kHalfPi);
        Quat b = fromAxisAngle(kRight, kHalfPi);
        Vec3 v(0, 1, 0);
        CHECK(vecEq((a * b) * v, a * (b * v)));
    }
    SUBCASE("inverse and conjugate undo the rotation") {
        Quat r = normalize(Quat(0.3f, 0.5f, -0.2f, 0.7f));
        Vec3 v(1.0f, 2.0f, 3.0f);
        CHECK(vecEq(glm::inverse(r) * (r * v), v));
        CHECK(vecEq(glm::conjugate(r) * (r * v), v));
    }
    SUBCASE("yaw-pitch-roll order") {
        Quat ypr = fromYawPitchRoll(0.4f, 0.3f, 0.2f);
        Quat manual = fromAxisAngle(kUp, 0.4f) * fromAxisAngle(kRight, 0.3f) * fromAxisAngle(kForward, 0.2f);
        CHECK(nearlyEqual(ypr, manual));
    }
    SUBCASE("q and -q are equal rotations") {
        Quat r = fromAxisAngle(kForward, 1.0f);
        CHECK(nearlyEqual(r, -r));
    }
}

TEST_CASE("slerp and nlerp") {
    Quat a = identityQuat();
    Quat b = fromAxisAngle(kUp, kHalfPi);

    CHECK(nearlyEqual(slerp(a, b, 0.0f), a));
    CHECK(nearlyEqual(slerp(a, b, 1.0f), b));
    CHECK(nearlyEqual(slerp(a, b, 0.5f), fromAxisAngle(kUp, kHalfPi * 0.5f)));
    CHECK(angleBetween(a, slerp(a, b, 0.25f)) == doctest::Approx(kHalfPi * 0.25f).epsilon(1e-4));

    SUBCASE("takes the shortest path when quaternions are in opposite hemispheres") {
        Quat bNeg = -b;
        CHECK(nearlyEqual(nlerp(a, bNeg, 0.5f), fromAxisAngle(kUp, kHalfPi * 0.5f)));
        CHECK(angleBetween(a, slerp(a, bNeg, 0.5f)) == doctest::Approx(kHalfPi * 0.5f).epsilon(1e-4));
    }
    SUBCASE("nearly identical inputs stay normalized") {
        Quat c = fromAxisAngle(kUp, 1e-5f);
        CHECK(length(slerp(a, c, 0.5f)) == doctest::Approx(1.0f));
    }
}

TEST_CASE("lookRotation") {
    SUBCASE("identity for +Z forward") {
        CHECK(nearlyEqual(lookRotation(kForward, kUp), identityQuat()));
    }
    SUBCASE("maps +Z to forward and keeps +Y up") {
        Vec3 f = normalize(Vec3(1, 0, 1));
        Quat q = lookRotation(f, kUp);
        CHECK(vecEq(q * kForward, f));
        CHECK(vecEq(q * kUp, kUp));
    }
    SUBCASE("tilted forward keeps up in the forward/up plane") {
        Vec3 f = normalize(Vec3(0, 0.5f, -1));
        Quat q = lookRotation(f, kUp);
        CHECK(vecEq(q * kForward, f));
        CHECK(dot(q * kRight, kUp) == doctest::Approx(0.0f).epsilon(1e-5));
        CHECK((q * kUp).y > 0.0f);
    }
    SUBCASE("forward parallel to up does not produce NaN") {
        Quat q = lookRotation(kUp, kUp);
        CHECK(vecEq(q * kForward, kUp));
        CHECK(length(q) == doctest::Approx(1.0f));
    }
}

TEST_CASE("fromTo") {
    Vec3 a = normalize(Vec3(1, 2, 3));
    Vec3 b = normalize(Vec3(-2, 0.5f, 1));
    CHECK(vecEq(fromTo(a, b) * a, b));
    CHECK(vecEq(fromTo(a, -a) * a, -a));  // opposite vectors
    CHECK(nearlyEqual(fromTo(a, a), identityQuat()));
}

TEST_CASE("swing-twist decomposition") {
    Vec3 axis = kUp;
    Quat twistIn = fromAxisAngle(axis, 0.7f);
    Quat swingIn = fromAxisAngle(normalize(Vec3(1, 0, 1)), 0.4f);  // perpendicular to Y
    Quat q = swingIn * twistIn;

    Quat swing, twist;
    swingTwist(q, axis, swing, twist);

    CHECK(nearlyEqual(swing * twist, q));
    CHECK(nearlyEqual(twist, twistIn));
    CHECK(nearlyEqual(swing, swingIn));
    // Twist rotates only about the axis: it leaves the axis unchanged.
    CHECK(vecEq(twist * axis, axis));

    SUBCASE("pure swing has identity twist") {
        Quat s, t;
        swingTwist(swingIn, axis, s, t);
        CHECK(nearlyEqual(t, identityQuat()));
    }
    SUBCASE("180 degree swing is handled") {
        Quat s, t;
        Quat flip = fromAxisAngle(kRight, kPi);
        swingTwist(flip, axis, s, t);
        CHECK(nearlyEqual(s * t, flip));
    }
}

TEST_CASE("matrices") {
    Quat r = fromAxisAngle(normalize(Vec3(1, 1, 0)), 0.8f);
    Vec3 t(1, 2, 3), s(2.0f);
    Mat4 m = trs(t, r, s);
    Vec3 p(0.5f, -1.0f, 2.0f);
    CHECK(vecEq(Vec3(m * Vec4(p, 1.0f)), t + r * (s * p), 1e-4f));
    CHECK(nearlyEqual(m * glm::inverse(m), Mat4(1.0f)));
    CHECK(nearlyEqual(trs(t, r, Vec3(1.0f)), translation(t) * rotation(r)));
    // Column-major layout: translation sits in elements 12..14, as Filament expects.
    const float* d = data(m);
    CHECK(d[12] == doctest::Approx(1.0f));
    CHECK(d[13] == doctest::Approx(2.0f));
    CHECK(d[14] == doctest::Approx(3.0f));
}

TEST_CASE("transform composition") {
    Transform parent{{1, 0, 0}, fromAxisAngle(kUp, kHalfPi), Vec3(2.0f)};
    Transform child{{0, 1, 1}, fromAxisAngle(kRight, 0.3f), Vec3(0.5f)};
    Vec3 p(0.2f, -0.4f, 1.5f);

    Transform world = parent * child;
    CHECK(vecEq(world.transformPoint(p), parent.transformPoint(child.transformPoint(p)), 1e-4f));
    CHECK(nearlyEqual(world.toMat4(), parent.toMat4() * child.toMat4()));

    SUBCASE("inverse") {
        Transform inv = inverse(world);
        CHECK(vecEq(inv.transformPoint(world.transformPoint(p)), p, 1e-4f));
        Transform id = world * inv;
        CHECK(vecEq(id.position, Vec3(0.0f), 1e-4f));
        CHECK(nearlyEqual(id.rotation, identityQuat()));
    }
    SUBCASE("lerp endpoints") {
        CHECK(vecEq(lerp(parent, child, 0.0f).position, parent.position));
        CHECK(nearlyEqual(lerp(parent, child, 1.0f).rotation, child.rotation));
    }
}
