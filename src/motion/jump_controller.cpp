#include "motion/jump_controller.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

float smooth(float t) {
    t = math::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

math::Vec2 lerp2(math::Vec2 a, math::Vec2 b, float t) {
    return a + (b - a) * t;
}

} // namespace

void JumpController::start() {
    if (active()) return;
    stage_ = JumpStage::Load;
    released_ = false;
    time_ = load_ = 0.0f;
    bodyY_ = feetY_ = arms_ = 0.0f;
    velocity_ = lean_ = math::Vec2(0.0f);
}

void JumpController::release(math::Vec2 direction) {
    if (!loading()) return;
    released_ = true;
    const float len = math::length(direction);
    direction_ = len > 1.0f ? direction / len : direction;
}

void JumpController::tick(float dt, const JumpSettings& s) {
    if (stage_ == JumpStage::None) return;
    time_ += dt;
    const float forwardLean = s.crouchLean;
    switch (stage_) {
    case JumpStage::Load: {
        load_ = std::min(1.0f, load_ + dt / std::max(s.loadTime, 0.01f));
        const float c = smooth(load_);
        bodyY_ = -s.crouchDepth * c;
        arms_ = -c;
        lean_ = math::Vec2(forwardLean * c, 0.0f);
        if (released_ && load_ >= s.minLoad) {
            // Take-off speed for the rise; the flight is symmetric (lands at
            // take-off height), so its time is 2 v / g.
            share_ = load_;
            upSpeed_ = std::sqrt(2.0f * s.gravity * s.height * share_);
            flightTime_ = 2.0f * upSpeed_ / s.gravity;
            const float along = direction_.y >= 0.0f ? s.forwardDistance : s.backDistance;
            travel_ = math::Vec2(direction_.x * s.sideDistance, direction_.y * along) * share_;
            fromY_ = bodyY_;
            fromArms_ = arms_;
            stage_ = JumpStage::Push;
            time_ = 0.0f;
        }
        break;
    }
    case JumpStage::Push: {
        // Ease in: the legs extend fastest at take-off, near the flight's speed.
        const float u = math::clamp(time_ / std::max(s.pushTime, 0.01f), 0.0f, 1.0f);
        bodyY_ = math::lerp(fromY_, s.pushLift, u * u);
        arms_ = math::lerp(fromArms_, 1.0f, smooth(u));
        const math::Vec2 toward(-direction_.x, direction_.y);  // lean y is toward the right
        lean_ = lerp2(math::Vec2(forwardLean * share_, 0.0f), toward * s.lean, smooth(u));
        if (u >= 1.0f) {
            stage_ = JumpStage::Flight;
            time_ = 0.0f;
            velocity_ = travel_ / std::max(flightTime_, 0.01f);
        }
        break;
    }
    case JumpStage::Flight: {
        const float t = std::min(time_, flightTime_);
        const float rise = upSpeed_ * t - 0.5f * s.gravity * t * t;
        bodyY_ = s.pushLift + rise;
        feetY_ = rise + s.tuck * std::sin(math::kPi * t / std::max(flightTime_, 0.01f));
        arms_ = math::lerp(1.0f, 0.3f, t / std::max(flightTime_, 0.01f));
        if (time_ >= flightTime_) {
            stage_ = JumpStage::Land;
            time_ = 0.0f;
            feetY_ = 0.0f;
            velocity_ = math::Vec2(0.0f);
            fromY_ = s.pushLift;
            fromArms_ = arms_;
        }
        break;
    }
    case JumpStage::Land: {
        // Down into the absorb fast (it starts at the landing speed), then up.
        const float down = s.landDepth * share_;
        if (time_ < s.landTime) {
            const float u = time_ / std::max(s.landTime, 0.01f);
            bodyY_ = math::lerp(fromY_, -down, std::sin(0.5f * math::kPi * u));
        } else {
            const float u = (time_ - s.landTime) / std::max(s.recoverTime, 0.01f);
            bodyY_ = -down * (1.0f - smooth(u));
            if (u >= 1.0f) {
                stage_ = JumpStage::None;
                bodyY_ = arms_ = 0.0f;
                lean_ = math::Vec2(0.0f);
                break;
            }
        }
        const float u = math::clamp(time_ / (s.landTime + s.recoverTime), 0.0f, 1.0f);
        arms_ = fromArms_ * (1.0f - smooth(u));
        // Absorbing, the trunk bends forward a little, then straightens.
        lean_ = math::Vec2(forwardLean * share_ * std::sin(math::kPi * u), 0.0f);
        break;
    }
    case JumpStage::None:
        break;
    }
}

} // namespace motion
