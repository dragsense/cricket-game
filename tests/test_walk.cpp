// Tests for walking: gait timing matches the biomechanics norms, the walk
// controller eases speed and turns at a limited rate, and gait-mode stepping
// alternates feet, keeps planted feet fixed in the world, and settles to
// stance when stopped.

#include "motion/foot_stepper.h"
#include "motion/walk_controller.h"

#include <doctest/doctest.h>

#include <cmath>

using namespace motion;
using math::Vec2;
using math::Vec3;

namespace {

constexpr float kDt = 1.0f / 120.0f;

} // namespace

TEST_CASE("gait timing at the reference speed") {
    WalkSettings s;
    GaitTiming g = gaitTiming(s.refSpeed, 1.8f, s);
    CHECK(g.stepLength == doctest::Approx(0.41f * 1.8f).epsilon(1e-3));
    CHECK(g.period == doctest::Approx(g.stepLength / s.refSpeed).epsilon(1e-3));
    CHECK(g.stance / (2.0f * g.period) == doctest::Approx(0.6f).epsilon(1e-3));  // 60 % stance
    CHECK(60.0f / g.period > 100.0f);  // cadence in steps per minute
    CHECK(60.0f / g.period < 120.0f);
}

TEST_CASE("walk controller eases speed and turns at a limited rate") {
    WalkSettings s;
    WalkController w;
    w.reset(0.0f);
    w.tick(kDt, Vec2(0, 1), true, 0.0f, s);
    CHECK(w.speed() == doctest::Approx(s.acceleration * kDt));
    for (int i = 0; i < 240; ++i) w.tick(kDt, Vec2(0, 1), true, 0.0f, s);
    CHECK(w.speed() == doctest::Approx(s.speed));
    CHECK(w.velocity().z == doctest::Approx(s.speed));

    // Face a heading at a right angle: yaw moves at most turnRate per tick.
    w.tick(kDt, Vec2(0, 1), true, math::kHalfPi, s);
    CHECK(w.yaw() == doctest::Approx(s.turnRate * kDt));
    for (int i = 0; i < 240; ++i) w.tick(kDt, Vec2(0, 1), true, math::kHalfPi, s);
    CHECK(w.yaw() == doctest::Approx(math::kHalfPi).epsilon(1e-3));

    for (int i = 0; i < 240; ++i) w.tick(kDt, Vec2(0.0f), false, 0.0f, s);
    CHECK(w.speed() == doctest::Approx(0.0f));
}

TEST_CASE("sidestep and back up without turning") {
    WalkSettings s;
    WalkController w;
    w.reset(0.0f);
    for (int i = 0; i < 240; ++i) w.tick(kDt, Vec2(1, 0), false, 0.0f, s);  // +X is left
    CHECK(w.yaw() == doctest::Approx(0.0f));
    CHECK(w.velocity().x == doctest::Approx(s.sideSpeed));
    for (int i = 0; i < 480; ++i) w.tick(kDt, Vec2(0, -1), false, 0.0f, s);
    CHECK(w.yaw() == doctest::Approx(0.0f));
    CHECK(w.velocity().z == doctest::Approx(-s.backSpeed));
}

TEST_CASE("gait stepping alternates feet, keeps the planted foot fixed, then settles") {
    WalkSettings s;
    FootStepper st;
    const Vec3 restL(0.1f, 0, 0), restR(-0.1f, 0, 0);
    st.reset(restL, restR);

    // Walk forward at full speed for a few seconds, moving the root.
    math::Transform root;
    const GaitTiming g = gaitTiming(s.speed, 1.8f, s);
    int swings = 0, last = -1;
    for (int i = 0; i < 480; ++i) {
        const int planted = st.swingFoot() == 0 ? 1 : 0;
        const Vec3 plantedWorld = root.transformPoint(st.foot(static_cast<Foot>(planted)));
        math::Transform next = root;
        next.position.z += s.speed * kDt;
        st.rebase(root, next);
        root = next;
        if (st.stepping()) CHECK(math::nearlyEqual(root.transformPoint(st.foot(static_cast<Foot>(planted))), plantedWorld, 1e-4f));
        st.tickGait(kDt, Vec3(0, 0, s.speed), g, s.stepHeight, s.settleDistance);
        if (st.swingFoot() >= 0 && st.swingFoot() != last) {
            last = st.swingFoot();
            ++swings;
        }
    }
    CHECK(swings > 6);

    // Stop: a far-off foot closes back to stance; near ones stay where they landed.
    for (int i = 0; i < 480; ++i) st.tickGait(kDt, Vec3(0.0f), gaitTiming(0.0f, 1.8f, s), s.stepHeight, s.settleDistance);
    CHECK_FALSE(st.stepping());
    CHECK(math::distance(st.foot(Foot::Left), restL) < s.settleDistance);
    CHECK(math::distance(st.foot(Foot::Right), restR) < s.settleDistance);
}

TEST_CASE("gait blend interpolates every setting") {
    WalkSettings a, b;
    b.speed = 5.0f;
    b.flightFraction = 0.4f;
    const WalkSettings m = blendGait(a, b, 0.5f);
    CHECK(m.speed == doctest::Approx(0.5f * (a.speed + 5.0f)));
    CHECK(m.flightFraction == doctest::Approx(0.2f));
    CHECK(m.stepK == doctest::Approx(a.stepK));
}

TEST_CASE("running has a flight phase and lands both feet on stop") {
    WalkSettings s;
    s.speed = s.refSpeed = 4.0f;
    s.stepK = 0.69f;
    s.swingFraction = 1.0f;
    s.flightFraction = 0.3f;
    s.flightLift = 0.05f;
    s.minSwing = 0.2f;
    const GaitTiming g = gaitTiming(s.speed, 1.8f, s);
    CHECK(g.flight > 0.0f);
    CHECK(g.stance / (2.0f * g.period) < 0.5f);  // less contact than walking

    GaitStyle style;
    style.flightLift = s.flightLift;
    style.heelRise = 0.5f;
    style.toeOffPitch = 0.5f;
    FootStepper st;
    st.reset(Vec3(0.1f, 0, 0), Vec3(-0.1f, 0, 0));
    math::Transform root;
    int airborne = 0;
    for (int i = 0; i < 480; ++i) {
        math::Transform next = root;
        next.position.z += s.speed * kDt;
        st.rebase(root, next);
        root = next;
        st.tickGait(kDt, Vec3(0, 0, s.speed), g, s.stepHeight, s.settleDistance, style);
        if (st.foot(Foot::Left).y > 0.01f && st.foot(Foot::Right).y > 0.01f) ++airborne;
    }
    CHECK(airborne > 0);

    for (int i = 0; i < 480; ++i) st.tickGait(kDt, Vec3(0.0f), gaitTiming(0.0f, 1.8f, s), s.stepHeight, s.settleDistance, style);
    CHECK_FALSE(st.stepping());
    CHECK(st.foot(Foot::Left).y == doctest::Approx(0.0f));
    CHECK(st.foot(Foot::Right).y == doctest::Approx(0.0f));
    CHECK(std::fabs(st.footPitch(Foot::Left)) < 0.1f);  // landed level
}
