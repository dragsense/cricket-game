#pragma once
// Foot planting and stepping: both feet stay locked where they were planted
// until a step is requested; then one foot at a time swings along an arc to
// its next spot. The next spot is relative to the planted foot (stance width
// to the side, plus the step in the requested direction), so holding a
// direction alternates the feet. Gait mode (walking) instead places each
// swing foot ahead of its stance spot by the body's velocity, alternating feet
// on the gait's timing, and settles back to stance when the velocity is zero.
// Character space (+X left, +Y up, +Z forward). No allocations.

#include "math/math.h"

namespace motion {

struct StepSettings {
    float stepLength = 0.6f;      // forward, ahead of the planted foot
    float backStepLength = 0.35f;
    float sideStepLength = 0.3f;
    float stepDuration = 0.4f;    // seconds per swing
    float stepHeight = 0.08f;     // swing arc apex above the ground
};

struct GaitTiming;

// Swing shape and foot roll for gait mode; the defaults are walking (flat
// feet, centred arc, standing width).
struct GaitStyle {
    float flightLift = 0.0f;    // m the toes leave the ground at push-off (running flight)
    float heelRise = 0.0f;      // share of the other foot's swing the stance heel rises over
    float toeOffPitch = 0.0f;   // rad, plantarflexion at toe-off (positive = toes down)
    float landPitch = 0.0f;     // rad, foot pitch at landing
    float peakAt = 0.5f;        // swing time share of the arc apex
    float ballDistance = 0.1f;  // m, ankle to the ball of the foot (heel-rise pivot)
    float widthScale = 1.0f;    // share of standing width the feet land at
    float flightCarry = 0.0f;   // share of body speed the back foot moves forward at after toe-off
};

enum class Foot : int { Left = 0, Right = 1 };

class FootStepper {
public:
    // Plants both feet; stance width comes from their current spacing, and
    // these spots are the gait mode's stance spots.
    void reset(math::Vec3 left, math::Vec3 right);

    // The character root moved from `from` to `to` (world): keeps the feet
    // where they are in the world, re-expressed in the new character space.
    void rebase(const math::Transform& from, const math::Transform& to);

    // `dir` is the requested step direction on the ground (x = left, y =
    // forward), zero for none. A step in progress always finishes.
    void tick(float dt, math::Vec2 dir, const StepSettings& s);

    // Gait mode. `velocity` is the body's velocity in character space. When
    // stopped, a foot only steps back to stance if it is more than
    // `settleDistance` from its stance spot (no fidgety tiny steps).
    // Sideways movement is a side shuffle (the feet never cross). Running:
    // the stance heel rises (the foot rolls over its ball) late in the other
    // foot's swing, and with g.flight > 0 the toes push off before the swing
    // foot lands, so both feet are in the air.
    void tickGait(float dt, math::Vec3 velocity, const GaitTiming& g, float stepHeight, float settleDistance,
                  const GaitStyle& style = {});

    math::Vec3 foot(Foot f) const { return feet_[static_cast<int>(f)]; }
    // Starts a step of foot `f` to `spot` (character space; lands at the
    // foot's current height); tick() with no direction advances it. False if
    // a step is already in progress.
    bool stepTo(Foot f, math::Vec3 spot);
    // Puts foot `f` at `spot` (character space) at once, flat, without a step.
    void place(Foot f, math::Vec3 spot);

    bool stepping() const { return swing_ >= 0; }
    int swingFoot() const { return swing_; }     // -1 when both are planted
    float swingProgress() const { return t_; }  // [0, 1] while stepping
    // Running: share of the body's weight on the right foot from the gait
    // phase (the stance foot, handing over to the swing foot through the
    // flight); -1 when both feet are planted.
    float rightWeight(float flightShare) const;
    // Foot pitch in character space (rad, positive = toes down) and toe bend
    // (rad, negative = toes up, keeping them on the ground while the heel rises).
    float footPitch(Foot f) const { return pitch_[static_cast<int>(f)]; }
    float toeBend(Foot f) const { return toe_[static_cast<int>(f)]; }

private:
    bool startStep(math::Vec2 dir, const StepSettings& s);
    void advanceSwing(float dt, float duration, float height, float peakAt = 0.5f);

    math::Vec3 feet_[2]{};    // current positions (swing foot moves along the arc)
    math::Vec3 from_{0.0f}, to_{0.0f};
    math::Vec3 launch_{0.0f};     // swing foot's starting velocity relative to the step (m/s)
    math::Vec3 carry_[2]{};       // gait mode: back foot's forward velocity after toe-off
    math::Vec3 stance_[2]{};  // gait mode's standing spots
    float pitch_[2]{}, toe_[2]{};
    float wait_ = 0.0f;       // gait mode: time both feet have been planted
    float dur_ = 0.0f;        // gait mode: this swing's duration
    bool idle_ = true;        // gait mode: standing (next walk starts fresh)
    int start_ = static_cast<int>(Foot::Left);  // gait mode: foot that started the last walk
    float stanceWidth_ = 0.2f;
    float t_ = 0.0f;          // swing progress [0, 1]
    int swing_ = -1;          // swinging foot index, -1 when both are planted
    int last_ = static_cast<int>(Foot::Right);
};

} // namespace motion
