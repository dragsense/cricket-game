// Tests for body repositioning: the body follows the feet, legs stay within
// reach, and the stance leg wins when the feet are too far apart.

#include "motion/body_reposition.h"

#include <doctest/doctest.h>

#include <initializer_list>

using namespace motion;
using math::Vec3;

namespace {

constexpr float kLeg = 0.9f;

float reach(Vec3 pelvis, const LegSupport& leg) { return math::distance(pelvis + leg.hipOffset, leg.foot); }

LegSupport leg(float side, Vec3 foot) { return {Vec3(side * 0.1f, -0.05f, 0.0f), foot}; }

} // namespace

TEST_CASE("clampToLegs keeps both hips within leg reach, no squat limit at minReach 0") {
    motion::RepositionSettings s;
    s.minReach = 0.0f;
    s.softBand = 0.0f;
    const motion::LegSupport left{{0.1f, 0, 0}, {0.1f, 0, 0}}, right{{-0.1f, 0, 0}, {-0.1f, 0, 0}};
    const math::Vec3 far = motion::clampToLegs({0, 0.8f, 0.6f}, left, right, 0.9f, s);  // pushed forward
    CHECK(math::distance(far + left.hipOffset, left.foot) <= s.maxReach * 0.9f + 1e-4f);
    CHECK(math::distance(far + right.hipOffset, right.foot) <= s.maxReach * 0.9f + 1e-4f);
    CHECK(far.z > 0.0f);
    const math::Vec3 low = motion::clampToLegs({0, 0.2f, 0}, left, right, 0.9f, s);  // deep squat: allowed
    CHECK(low.y == doctest::Approx(0.2f));
}

TEST_CASE("settles at standing height above both feet") {
    RepositionSettings s;
    LegSupport l = leg(1, {0.1f, 0, 0}), r = leg(-1, {-0.1f, 0, 0});
    Vec3 p(0, 0.3f, 0);
    for (int i = 0; i < 600; ++i) p = repositionBody(p, l, r, kLeg, 0.5f, s, 1.0f / 120.0f);
    CHECK(reach(p, l) == doctest::Approx(s.standReach * kLeg).epsilon(1e-3));
    CHECK(p.x == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("legs stay within reach in a single tick") {
    RepositionSettings s;
    LegSupport l = leg(1, {0.1f, 0, 0}), r = leg(-1, {-0.1f, 0, 0});
    // Body far above (feet dropped) and far below (jump lifting the feet).
    for (Vec3 p : {Vec3(0, 3, 0), Vec3(0, -1, 0), Vec3(2, 0.9f, 0)}) {
        Vec3 q = repositionBody(p, l, r, kLeg, 0.5f, s, 1.0f / 120.0f);
        for (const LegSupport* x : {&l, &r}) {
            CHECK(reach(q, *x) <= s.maxReach * kLeg + 1e-3f);
            CHECK(reach(q, *x) >= s.minReach * kLeg - 1e-3f);
        }
    }
}

TEST_CASE("body follows sliding feet") {
    RepositionSettings s;
    Vec3 p(0, 0.9f, 0);
    for (int i = 0; i < 600; ++i) {
        float z = 2.0f * static_cast<float>(i) / 600.0f;
        p = repositionBody(p, leg(1, {0.1f, 0, z}), leg(-1, {-0.1f, 0, z}), kLeg, 0.5f, s, 1.0f / 120.0f);
    }
    CHECK(p.z == doctest::Approx(2.0f).epsilon(0.05));
}

TEST_CASE("stance leg wins when feet are too far apart") {
    RepositionSettings s;
    LegSupport l = leg(1, {3, 0, 0}), r = leg(-1, {-0.1f, 0, 0});
    Vec3 q = repositionBody(Vec3(0, 0.9f, 0), l, r, kLeg, 1.0f, s, 1.0f / 120.0f);
    CHECK(reach(q, r) <= s.maxReach * kLeg + 1e-3f);
}
