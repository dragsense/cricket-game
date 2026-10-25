#include "motion/walk_controller.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kMinGaitSpeed = 0.1f;  // m/s; slower walking uses this speed's timing
constexpr float kRunShareFlight = 0.1f; // flight share at which a gait counts fully as running

float wrapAngle(float a) {
    return std::remainder(a, math::kTwoPi);
}

} // namespace

WalkSettings blendGait(const WalkSettings& a, const WalkSettings& b, float t) {
    WalkSettings r;
    r.speed = math::lerp(a.speed, b.speed, t);
    r.backSpeed = math::lerp(a.backSpeed, b.backSpeed, t);
    r.sideSpeed = math::lerp(a.sideSpeed, b.sideSpeed, t);
    r.acceleration = math::lerp(a.acceleration, b.acceleration, t);
    r.deceleration = math::lerp(a.deceleration, b.deceleration, t);
    r.turnRate = math::lerp(a.turnRate, b.turnRate, t);
    r.stepK = math::lerp(a.stepK, b.stepK, t);
    r.refSpeed = math::lerp(a.refSpeed, b.refSpeed, t);
    r.swingFraction = math::lerp(a.swingFraction, b.swingFraction, t);
    r.minSwing = math::lerp(a.minSwing, b.minSwing, t);
    r.maxSwing = math::lerp(a.maxSwing, b.maxSwing, t);
    r.stepHeight = math::lerp(a.stepHeight, b.stepHeight, t);
    r.settleDistance = math::lerp(a.settleDistance, b.settleDistance, t);
    r.bob = math::lerp(a.bob, b.bob, t);
    r.sway = math::lerp(a.sway, b.sway, t);
    r.leanPerSpeed = math::lerp(a.leanPerSpeed, b.leanPerSpeed, t);
    r.leanPerAccel = math::lerp(a.leanPerAccel, b.leanPerAccel, t);
    r.turnLean = math::lerp(a.turnLean, b.turnLean, t);
    r.maxLean = math::lerp(a.maxLean, b.maxLean, t);
    r.leanRate = math::lerp(a.leanRate, b.leanRate, t);
    r.armSwing = math::lerp(a.armSwing, b.armSwing, t);
    r.armBackBias = math::lerp(a.armBackBias, b.armBackBias, t);
    r.elbowBias = math::lerp(a.elbowBias, b.elbowBias, t);
    r.elbowSwing = math::lerp(a.elbowSwing, b.elbowSwing, t);
    r.armRate = math::lerp(a.armRate, b.armRate, t);
    r.headStabilise = math::lerp(a.headStabilise, b.headStabilise, t);
    r.headLead = math::lerp(a.headLead, b.headLead, t);
    r.headRate = math::lerp(a.headRate, b.headRate, t);
    r.flightFraction = math::lerp(a.flightFraction, b.flightFraction, t);
    r.flightLift = math::lerp(a.flightLift, b.flightLift, t);
    r.heelRise = math::lerp(a.heelRise, b.heelRise, t);
    r.toeOffPitch = math::lerp(a.toeOffPitch, b.toeOffPitch, t);
    r.landPitch = math::lerp(a.landPitch, b.landPitch, t);
    r.swingPeakAt = math::lerp(a.swingPeakAt, b.swingPeakAt, t);
    r.widthScale = math::lerp(a.widthScale, b.widthScale, t);
    r.pelvisTwist = math::lerp(a.pelvisTwist, b.pelvisTwist, t);
    r.chestCounter = math::lerp(a.chestCounter, b.chestCounter, t);
    r.pelvisDrop = math::lerp(a.pelvisDrop, b.pelvisDrop, t);
    r.sideFollow = math::lerp(a.sideFollow, b.sideFollow, t);
    r.flightCarry = math::lerp(a.flightCarry, b.flightCarry, t);
    return r;
}

GaitTiming gaitTiming(float speed, float height, const WalkSettings& s) {
    const float v = std::max(speed, kMinGaitSpeed);
    GaitTiming g;
    g.stepLength = s.stepK * height * std::pow(v / std::max(s.refSpeed, kMinGaitSpeed), 0.42f);
    const float fraction = math::clamp(s.swingFraction, 0.1f, 1.0f);
    g.swing = math::clamp(fraction * g.stepLength / v, s.minSwing, s.maxSwing);
    g.period = g.swing / fraction;
    g.flight = math::clamp(s.flightFraction, 0.0f, 0.9f) * g.swing;
    g.stance = 2.0f * g.period - g.swing - g.flight;
    return g;
}

float gaitBob(float t, float flightShare) {
    const float walk = std::sin(math::kPi * t) - 0.5f;
    const float f = math::clamp(flightShare, 0.0f, 0.9f);
    if (f <= 0.0f) return walk;
    const float stance = 1.0f - f;
    const float run = t < stance ? -0.5f * std::sin(math::kPi * t / stance)
                                 : 0.5f * std::sin(math::kPi * (t - stance) / f);
    return math::lerp(walk, run, runShare(f));
}

float runShare(float flightShare) {
    return math::clamp(flightShare / kRunShareFlight, 0.0f, 1.0f);
}

void WalkController::reset(float yaw) {
    yaw_ = yaw;
    local_ = math::Vec2(0.0f);
    accel_ = yawRate_ = gaitSpeed_ = 0.0f;
}

void WalkController::tick(float dt, math::Vec2 move, bool face, float faceYaw, const WalkSettings& s) {
    if (dt <= 0.0f) return;
    if (math::length2(move) > 1.0f) move /= math::length(move);  // diagonals

    yawRate_ = 0.0f;
    float turnSlow = 1.0f;
    if (face) {
        const float diff = wrapAngle(faceYaw - yaw_);
        const float turn = math::clamp(diff, -s.turnRate * dt, s.turnRate * dt);
        yaw_ = wrapAngle(yaw_ + turn);
        yawRate_ = turn / dt;
        turnSlow = std::max(0.0f, std::cos(diff));  // slow down for sharp turns
    }
    const math::Vec2 target(move.x * s.sideSpeed, move.y > 0.0f ? move.y * s.speed * turnSlow : move.y * s.backSpeed);
    if (math::length2(target) > math::kEpsilon) gaitSpeed_ = math::length(target);
    const math::Vec2 delta = target - local_;
    const float rate = math::length2(target) > math::length2(local_) ? s.acceleration : s.deceleration;
    const float len = math::length(delta);
    const math::Vec2 step = len > rate * dt ? delta * (rate * dt / len) : delta;
    accel_ = step.y / dt;
    local_ += step;
}

math::Vec3 WalkController::velocity() const {
    // Character +X (left) is world (cos, 0, -sin); +Z (forward) is (sin, 0, cos).
    const float c = std::cos(yaw_), sn = std::sin(yaw_);
    return math::Vec3(c, 0.0f, -sn) * local_.x + math::Vec3(sn, 0.0f, c) * local_.y;
}

} // namespace motion
