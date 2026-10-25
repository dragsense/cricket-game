// Tests for foot planting and stepping: feet stay locked without input, one
// foot swings at a time, holding a direction alternates feet, and steps land
// where expected.

#include "motion/foot_stepper.h"

#include <doctest/doctest.h>

#include <algorithm>

using namespace motion;
using math::Vec2;
using math::Vec3;

namespace {

constexpr float kDt = 1.0f / 120.0f;

FootStepper standing() {
    FootStepper s;
    s.reset(Vec3(0.1f, 0, 0), Vec3(-0.1f, 0, 0));
    return s;
}

// Ticks until the current step lands; returns which foot moved.
Foot finishStep(FootStepper& st, Vec2 dir, const StepSettings& s) {
    Vec3 l = st.foot(Foot::Left);
    st.tick(kDt, dir, s);
    while (st.stepping()) st.tick(kDt, dir, s);
    return math::distance(l, st.foot(Foot::Left)) > 1e-4f ? Foot::Left : Foot::Right;
}

} // namespace

TEST_CASE("stepTo moves one foot to the spot, the other stays") {
    motion::FootStepper stepper;
    stepper.reset({0.1f, 0, 0}, {-0.1f, 0, 0});
    REQUIRE(stepper.stepTo(motion::Foot::Left, {0.3f, 0.5f, 0.6f}));
    CHECK_FALSE(stepper.stepTo(motion::Foot::Right, {0, 0, 0}));  // one at a time
    motion::StepSettings s;
    for (int i = 0; i < 120 && stepper.stepping(); ++i) stepper.tick(1.0f / 120.0f, math::Vec2(0.0f), s);
    CHECK_FALSE(stepper.stepping());
    CHECK(math::nearlyEqual(stepper.foot(motion::Foot::Left), math::Vec3(0.3f, 0.0f, 0.6f), 1e-4f));  // ground height kept
    CHECK(math::nearlyEqual(stepper.foot(motion::Foot::Right), math::Vec3(-0.1f, 0, 0), 1e-4f));
}

TEST_CASE("feet stay planted without input") {
    StepSettings s;
    FootStepper st = standing();
    for (int i = 0; i < 240; ++i) st.tick(kDt, Vec2(0.0f), s);
    CHECK_FALSE(st.stepping());
    CHECK(math::nearlyEqual(st.foot(Foot::Left), Vec3(0.1f, 0, 0)));
    CHECK(math::nearlyEqual(st.foot(Foot::Right), Vec3(-0.1f, 0, 0)));
}

TEST_CASE("holding forward alternates feet and moves ahead") {
    StepSettings s;
    FootStepper st = standing();
    Foot a = finishStep(st, Vec2(0, 1), s);
    Foot b = finishStep(st, Vec2(0, 1), s);
    Foot c = finishStep(st, Vec2(0, 1), s);
    CHECK(a != b);
    CHECK(b != c);
    // Three steps: the last foot is one step ahead of the other.
    float lead = std::max(st.foot(Foot::Left).z, st.foot(Foot::Right).z);
    float trail = std::min(st.foot(Foot::Left).z, st.foot(Foot::Right).z);
    CHECK(lead - trail == doctest::Approx(s.stepLength).epsilon(1e-3));
    CHECK(st.foot(Foot::Left).x - st.foot(Foot::Right).x == doctest::Approx(0.2f).epsilon(1e-3));
}

TEST_CASE("only the swing foot moves, and it lifts") {
    StepSettings s;
    FootStepper st = standing();
    float maxLift = 0.0f;
    st.tick(kDt, Vec2(0, 1), s);
    while (st.stepping()) {
        CHECK(math::nearlyEqual(st.foot(Foot::Right), Vec3(-0.1f, 0, 0)));  // first step is the left foot
        maxLift = std::max(maxLift, st.foot(Foot::Left).y);
        st.tick(kDt, Vec2(0, 1), s);
    }
    CHECK(maxLift == doctest::Approx(s.stepHeight).epsilon(0.02));
    CHECK(st.foot(Foot::Left).y == doctest::Approx(0.0f));
}

TEST_CASE("side step leads with the foot on that side, then closes") {
    StepSettings s;
    FootStepper st = standing();
    CHECK(finishStep(st, Vec2(1, 0), s) == Foot::Left);  // +X is left
    CHECK(finishStep(st, Vec2(1, 0), s) == Foot::Right);
    CHECK(st.foot(Foot::Right).x == doctest::Approx(-0.1f + s.sideStepLength).epsilon(1e-3));
    CHECK(st.foot(Foot::Left).x - st.foot(Foot::Right).x == doctest::Approx(0.2f).epsilon(1e-3));
}
