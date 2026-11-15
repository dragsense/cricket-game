#include "motion/jump_planner.h"

#include <algorithm>

namespace motion {

namespace {
constexpr float kMinTime = 0.05f;

float share(float time, float duration) { return std::clamp(time / std::max(duration, kMinTime), 0.0f, 1.0f); }
} // namespace

void JumpPlanner::start(float standingHips) {
    phase_ = Phase::Crouch;
    held_ = true;
    time_ = bottom_ = 0.0f;
    standing_ = standingHips;
}

void JumpPlanner::release() { held_ = false; }

void JumpPlanner::touchdown() {
    if (phase_ != Phase::Flight) return;
    phase_ = Phase::Land;
    time_ = 0.0f;
}

bool JumpPlanner::tick(float dt, const JumpPlannerSettings& s, TargetSet& t) {
    if (phase_ == Phase::None) return false;
    time_ += dt;
    // Only the hips' height is given; across, the decider keeps them
    // between the feet.
    float y = standing_;
    switch (phase_) {
    case Phase::Crouch:
        bottom_ = s.crouchDrop * math::minimumJerk(share(time_, s.crouchTime));
        y = standing_ - bottom_;
        if (!held_ && time_ > dt) {
            phase_ = Phase::Push;
            time_ = 0.0f;
        }
        break;
    case Phase::Push: {
        // Faster and faster: the speed peaks at take-off.
        const float u = share(time_, s.pushTime);
        y = standing_ - bottom_ + (bottom_ + s.pushLift) * u * u;
        if (u >= 1.0f) {
            phase_ = Phase::Flight;
            time_ = 0.0f;
        }
        break;
    }
    case Phase::Flight:
        if (time_ >= s.maxFlight) touchdown();
        break;
    case Phase::Land: {
        const float down = math::minimumJerk(share(time_, s.landTime));
        const float up = math::minimumJerk(share(time_ - s.landTime, s.recoverTime));
        // As deep as the jump was big: a small hop lands with a small give
        // (the crouch's share of its full depth).
        const float size = std::clamp(bottom_ / std::max(s.crouchDrop, 0.01f), 0.0f, 1.0f);
        y = standing_ - s.landDrop * size * down * (1.0f - up);
        if (time_ >= s.landTime + s.recoverTime) phase_ = Phase::None;
        break;
    }
    case Phase::None:
        break;
    }
    t.hips.mode = TargetMode::Driven;
    t.hips.position = math::Vec3(0.0f, y, 0.0f);
    t.hips.directions[0] = math::kUp;  // up only: across stays over the feet
    t.hips.directionCount = 1;
    return phase_ != Phase::None;
}

} // namespace motion
