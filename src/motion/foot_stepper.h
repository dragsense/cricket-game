#pragma once
// Foot planting and stepping: both feet stay locked where they were planted
// until a step is requested; then one foot at a time swings along an arc to
// its next spot. The next spot is relative to the planted foot (stance width
// to the side, plus the step in the requested direction), so holding a
// direction alternates the feet. Character space (+X left, +Y up, +Z forward).
// No allocations.

#include "math/math.h"

namespace motion {

struct StepSettings {
    float stepLength = 0.6f;      // forward, ahead of the planted foot
    float backStepLength = 0.35f;
    float sideStepLength = 0.3f;
    float stepDuration = 0.4f;    // seconds per swing
    float stepHeight = 0.08f;     // swing arc apex above the ground
};

enum class Foot : int { Left = 0, Right = 1 };

class FootStepper {
public:
    // Plants both feet; stance width comes from their current spacing.
    void reset(math::Vec3 left, math::Vec3 right);

    // `dir` is the requested step direction on the ground (x = left, y =
    // forward), zero for none. A step in progress always finishes.
    void tick(float dt, math::Vec2 dir, const StepSettings& s);

    math::Vec3 foot(Foot f) const { return feet_[static_cast<int>(f)]; }
    bool stepping() const { return swing_ >= 0; }

private:
    bool startStep(math::Vec2 dir, const StepSettings& s);

    math::Vec3 feet_[2]{};    // current positions (swing foot moves along the arc)
    math::Vec3 from_{0.0f}, to_{0.0f};
    float stanceWidth_ = 0.2f;
    float t_ = 0.0f;          // swing progress [0, 1]
    int swing_ = -1;          // swinging foot index, -1 when both are planted
    int last_ = static_cast<int>(Foot::Right);
};

} // namespace motion
