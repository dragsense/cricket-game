#include "motion/walk_controller.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kMinGaitSpeed = 0.1f;  // m/s; slower walking uses this speed's timing

float wrapAngle(float a) {
    return std::remainder(a, math::kTwoPi);
}

} // namespace

GaitTiming gaitTiming(float speed, float height, const WalkSettings& s) {
    const float v = std::max(speed, kMinGaitSpeed);
    GaitTiming g;
    g.stepLength = s.stepK * height * std::pow(v / std::max(s.refSpeed, kMinGaitSpeed), 0.42f);
    const float fraction = math::clamp(s.swingFraction, 0.1f, 1.0f);
    g.swing = math::clamp(fraction * g.stepLength / v, s.minSwing, s.maxSwing);
    g.period = g.swing / fraction;
    g.stance = 2.0f * g.period - g.swing;
    return g;
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
