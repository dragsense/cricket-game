#include "motion/reach.h"

#include <doctest/doctest.h>

using motion::clampToReach;
using motion::swingToward;

namespace {

math::Vec3 rotateAbout(math::Vec3 pivot, math::Vec3 p, const motion::Swing& s) {
    return pivot + math::fromAxisAngle(s.axis, s.angle) * (p - pivot);
}

} // namespace

TEST_CASE("swingToward: within reach needs no swing") {
    const motion::Swing s = swingToward({0, 0, 0}, {0, 1, 0}, {0.3f, 1.2f, 0}, 0.6f);
    CHECK(s.angle == 0.0f);
}

TEST_CASE("swingToward: swung end lands exactly at reach (law of cosines)") {
    const math::Vec3 pivot(0, 0, 0), end(0, 0.5f, 0), target(0, 0.2f, 1.0f);
    const float reach = 0.7f;
    const motion::Swing s = swingToward(pivot, end, target, reach);
    REQUIRE(s.angle > 0.0f);
    const math::Vec3 moved = rotateAbout(pivot, end, s);
    CHECK(math::distance(moved, target) == doctest::Approx(reach).epsilon(1e-4));
    CHECK(math::length(moved) == doctest::Approx(0.5f));  // rotation keeps the segment length
}

TEST_CASE("swingToward: target too far points the end straight at it") {
    const math::Vec3 pivot(0, 0, 0), end(0, 0.5f, 0), target(2.0f, 0, 0);
    const motion::Swing s = swingToward(pivot, end, target, 0.5f);
    CHECK(s.angle == doctest::Approx(math::kHalfPi));
    const math::Vec3 moved = rotateAbout(pivot, end, s);
    CHECK(math::nearlyEqual(math::normalize(moved), math::Vec3(1, 0, 0), 1e-4f));
}

TEST_CASE("clampRotationVector limits each axis") {
    const math::Vec3 lo(math::radians(-10.0f), math::radians(-30.0f), math::radians(-20.0f));
    const math::Vec3 hi(math::radians(80.0f), math::radians(30.0f), math::radians(20.0f));
    const math::Quat small = math::fromAxisAngle(math::kRight, math::radians(40.0f));
    CHECK(math::angleBetween(motion::clampRotationVector(small, lo, hi), small) < 1e-3f);
    const math::Quat tooFar = math::fromAxisAngle(math::kRight, math::radians(100.0f));
    CHECK(math::angleBetween(motion::clampRotationVector(tooFar, lo, hi),
                             math::fromAxisAngle(math::kRight, math::radians(80.0f))) < 1e-3f);
    const math::Quat back = math::fromAxisAngle(math::kRight, math::radians(-45.0f));
    CHECK(math::angleBetween(motion::clampRotationVector(back, lo, hi),
                             math::fromAxisAngle(math::kRight, math::radians(-10.0f))) < 1e-3f);
    const math::Quat turn = math::fromAxisAngle(math::kUp, math::radians(60.0f));
    CHECK(math::angleBetween(motion::clampRotationVector(turn, lo, hi),
                             math::fromAxisAngle(math::kUp, math::radians(30.0f))) < 1e-3f);
}

TEST_CASE("clampToReach keeps near targets and pulls far ones in") {
    CHECK(math::nearlyEqual(clampToReach({0, 0, 0}, {0, 0.3f, 0}, 0.5f), math::Vec3(0, 0.3f, 0)));
    CHECK(math::nearlyEqual(clampToReach({0, 0, 0}, {0, 0, 2.0f}, 0.5f), math::Vec3(0, 0, 0.5f)));
}
