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

JumpArm lerpArm(const JumpArm& a, const JumpArm& b, float t) {
    return {math::lerp(a.flex, b.flex, t), math::lerp(a.abduct, b.abduct, t), math::lerp(a.elbow, b.elbow, t)};
}

} // namespace

void JumpController::start(bool moving, bool driveLeft, float speedShare) {
    if (active()) return;
    stage_ = JumpStage::Load;
    released_ = false;
    moving_ = moving;
    driveLeft_ = driveLeft;
    speedShare_ = math::clamp(speedShare, 0.0f, 1.0f);
    time_ = load_ = 0.0f;
    bodyY_ = feetY_ = 0.0f;
    velocity_ = lean_ = math::Vec2(0.0f);
    direction_ = math::Vec2(0.0f);
    arms_[0] = arms_[1] = JumpArm{};
    armWeight_ = moving ? 0.0f : 1.0f;
}

void JumpController::release(math::Vec2 direction, float minShare) {
    if (!loading()) return;
    released_ = true;
    minShare_ = minShare;
    const float len = math::length(direction);
    direction_ = len > 1.0f ? direction / len : direction;
}

void JumpController::armTargets(const JumpSettings& s) {
    for (int i = 0; i < 2; ++i) {
        const bool left = i == 0;
        JumpArm& top = top_[i];
        JumpArm& land = land_[i];
        if (moving_) {
            // The gait's opposed swing, grown: the arm opposite the driving
            // knee punches forward-up, the other drives back.
            const float drive = math::lerp(s.moveMin, 1.0f, speedShare_);
            const bool forward = left != driveLeft_;
            top = {forward ? s.moveDrive * drive : -s.moveBack * drive, 0.0f, s.elbowMove};
            land = {s.landFlex * 0.6f, 0.0f, s.elbowLand};
            continue;
        }
        // Direction weights: straight up, forward, back, side (they sum to 1).
        const float side = std::fabs(direction_.x);
        const float fwd = std::max(direction_.y, 0.0f), back = std::max(-direction_.y, 0.0f);
        const float up = std::max(0.0f, 1.0f - math::length(direction_));
        const float sum = std::max(up + fwd + back + side, 0.001f);
        const float wu = up / sum, wf = fwd / sum, wb = back / sum, ws = side / sum;
        const bool lead = side > 0.0f && (direction_.x > 0.0f) == left;  // +x is left
        const float sagittal = wu * s.armUp + wf * s.armUpForward + wb * s.armUpBack;
        const float scale = math::lerp(0.6f, 1.0f, share_);
        if (lead || ws == 0.0f) top = {(sagittal + ws * s.armUpSide) * scale, ws * s.sideOut, s.elbowTop};
        else top = {(sagittal + ws * s.acrossFlex) * scale, -ws * s.across, s.elbowTop};
        land = {(1.0f - ws) * s.landFlex + ws * s.landSideFlex, ws * s.landSideOut * (lead ? 1.0f : 0.6f), s.elbowLand};
    }
}

void JumpController::tick(float dt, const JumpSettings& s) {
    if (stage_ == JumpStage::None) return;
    time_ += dt;
    const float forwardLean = s.crouchLean;
    switch (stage_) {
    case JumpStage::Load: {
        // Moving, the dip stays small while it waits for the take-off foot.
        load_ = std::min(moving_ ? s.moveLoad : 1.0f, load_ + dt / std::max(s.loadTime, 0.01f));
        const float c = smooth(load_);
        bodyY_ = -s.crouchDepth * c;
        lean_ = math::Vec2(forwardLean * c, 0.0f);
        if (moving_) {
            // No back swing: the walk's arms hand over to the take-off pose.
            armTargets(s);
            armWeight_ = smooth(load_ / std::max(minShare_, 0.01f));
            arms_[0] = top_[0];
            arms_[1] = top_[1];
        } else {
            // Both arms back (further when a forward jump is being asked for).
            const float back = math::lerp(s.armBack, s.armBackForward, std::max(direction_.y, 0.0f));
            arms_[0] = arms_[1] = JumpArm{-back * c, 0.0f, s.elbowLoad * c};
        }
        // Moving: no deep crouch, just the dip the stride gives (minShare).
        const float need = moving_ ? std::min(minShare_, 1.0f) : std::max(s.minLoad, std::min(minShare_, 1.0f));
        if (released_ && load_ >= need) {
            // Take-off speed for the rise; the flight is symmetric (lands at
            // take-off height), so its time is 2 v / g.
            share_ = load_;
            // Moving: a low bound; the run's own speed carries it (no added travel).
            upSpeed_ = std::sqrt(2.0f * s.gravity * (moving_ ? s.moveHeight : s.height * share_));
            flightTime_ = 2.0f * upSpeed_ / s.gravity;
            const float along = direction_.y >= 0.0f ? s.forwardDistance : s.backDistance;
            travel_ = moving_ ? math::Vec2(0.0f) : math::Vec2(direction_.x * s.sideDistance, direction_.y * along) * share_;
            fromY_ = bodyY_;
            armTargets(s);
            from_[0] = arms_[0];
            from_[1] = arms_[1];
            stage_ = JumpStage::Push;
            time_ = 0.0f;
        }
        break;
    }
    case JumpStage::Push: {
        // Ease in: the legs extend fastest at take-off, near the flight's speed.
        const float u = math::clamp(time_ / std::max(moving_ ? s.movePushTime : s.pushTime, 0.01f), 0.0f, 1.0f);
        bodyY_ = math::lerp(fromY_, s.pushLift, u * u);
        for (int i = 0; i < 2; ++i) arms_[i] = lerpArm(from_[i], top_[i], smooth(u));
        armWeight_ = 1.0f;
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
        const float share = t / std::max(flightTime_, 0.01f);
        const float rise = upSpeed_ * t - 0.5f * s.gravity * t * t;
        bodyY_ = s.pushLift + rise;
        // Moving, no tuck: the legs stay long (the take-off leg straight behind).
        feetY_ = rise + (moving_ ? 0.0f : s.tuck * std::sin(math::kPi * share));
        // The arms start toward the landing pose.
        for (int i = 0; i < 2; ++i) arms_[i] = lerpArm(top_[i], land_[i], s.flightFall * smooth(share));
        if (time_ >= flightTime_) {
            stage_ = JumpStage::Land;
            time_ = 0.0f;
            feetY_ = 0.0f;
            velocity_ = math::Vec2(0.0f);
            fromY_ = s.pushLift;
            from_[0] = arms_[0];
            from_[1] = arms_[1];
        }
        break;
    }
    case JumpStage::Land: {
        // Down into the absorb fast (it starts at the landing speed), then up;
        // the arms reach the landing pose, then hang again (or hand back to
        // the walk's swing).
        const float down = s.landDepth * share_;
        if (time_ < s.landTime) {
            const float u = time_ / std::max(s.landTime, 0.01f);
            bodyY_ = math::lerp(fromY_, -down, std::sin(0.5f * math::kPi * u));
            for (int i = 0; i < 2; ++i) arms_[i] = lerpArm(from_[i], land_[i], smooth(u));
        } else {
            const float u = (time_ - s.landTime) / std::max(s.recoverTime, 0.01f);
            bodyY_ = -down * (1.0f - smooth(u));
            for (int i = 0; i < 2; ++i) arms_[i] = lerpArm(land_[i], JumpArm{}, smooth(u));
            if (moving_) armWeight_ = 1.0f - smooth(u);
            if (u >= 1.0f) {
                stage_ = JumpStage::None;
                bodyY_ = 0.0f;
                lean_ = math::Vec2(0.0f);
                arms_[0] = arms_[1] = JumpArm{};
                armWeight_ = 0.0f;
                break;
            }
        }
        const float u = math::clamp(time_ / (s.landTime + s.recoverTime), 0.0f, 1.0f);
        // Absorbing, the trunk bends forward a little, then straightens.
        lean_ = math::Vec2(forwardLean * share_ * std::sin(math::kPi * u), 0.0f);
        break;
    }
    case JumpStage::None:
        break;
    }
}

} // namespace motion
