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

#include <algorithm>

namespace motion {

struct StepSettings {
    float stepLength = 0.6f;      // forward, ahead of the planted foot
    float backStepLength = 0.35f;
    float sideStepLength = 0.3f;
    float stepDuration = 0.4f;    // seconds per swing
    float stepHeight = 0.08f;     // swing arc apex above the ground
    float peakAt = 0.5f;          // swing time share of the arc apex (early: knee bends, heel up)
    float liftAt = 0.0f;          // step time share the foot stays down first (weight moves off it)
};

struct GaitTiming;

// Swing height share, 0 -> 1 at `peakAt` -> 0 over the swing time share t.
// Eases in and out on both sides, so the foot never jumps up at lift-off
// however early the apex is.
float swingArc(float t, float peakAt);

// Swing shape and foot roll for gait mode; the defaults are walking (flat
// feet, centred arc, standing width).
struct GaitStyle {
    float toeOffPitch = 0.0f;   // rad, plantarflexion at toe-off (positive = toes down)
    float landPitch = 0.0f;     // rad, foot pitch at landing
    float peakAt = 0.5f;        // swing time share of the arc apex
    float pushTime = 0.12f;     // s, a foot pushes off over this long before it lifts (pre-swing)
    float liftOffSpeed = 0.0f;  // forward speed at lift-off, shares of the average swing speed (0: from rest)
    float widthScale = 1.0f;    // share of standing width the feet land at
    float landAhead = 0.5f;     // share of a foot's ground travel that is ahead of the hips at landing
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
    // The deciding part of a step, for a caller that moves the foot itself:
    // which foot a step in `dir` moves now (-1: none, too short), and the
    // spot foot `f` lands on for `dir` (relative to the other, planted foot;
    // at its ground height). Asked every tick, a step re-aims as `dir` changes.
    int pickFoot(math::Vec2 dir, const StepSettings& s) const;
    math::Vec3 stepSpot(Foot f, math::Vec2 dir, const StepSettings& s) const;
    // The last foot that stepped (a caller moving the feet itself sets it).
    void setLastStepped(Foot f) { last_ = static_cast<int>(f); }

    bool stepping() const { return swing_ >= 0; }
    // Foot `f` is off the ground (swinging).
    bool inAir(Foot f) const { return swing_ == static_cast<int>(f); }
    int swingFoot() const { return swing_; }     // -1 when both are planted
    float swingProgress() const { return t_; }  // [0, 1] while stepping
    // Foot pitch in character space (rad, positive = toes down) and toe bend
    // (rad, negative = toes up, keeping them on the ground while the heel rises).
    float footPitch(Foot f) const { return pitch_[static_cast<int>(f)]; }
    float toeBend(Foot f) const { return toe_[static_cast<int>(f)]; }
    // Gait mode: share [0, 1] of the push-off a foot is in: rises over the
    // last pushTime before it lifts, then fades over the first half of its
    // swing (the ankle comes back up for clearance).
    float pushOff(Foot f) const { return push_[static_cast<int>(f)]; }

private:
    bool startStep(math::Vec2 dir, const StepSettings& s);
    void advanceSwing(float dt, float duration, float height, float peakAt = 0.5f, float liftAt = 0.0f,
                      float liftOffSpeed = 0.0f);

    math::Vec3 feet_[2]{};    // current positions (swing foot moves along the arc)
    math::Vec3 from_{0.0f}, to_{0.0f};
    math::Vec3 stance_[2]{};  // gait mode's standing spots
    float pitch_[2]{}, toe_[2]{}, push_[2]{};
    float wait_ = 0.0f;       // gait mode: time both feet have been planted
    float dur_ = 0.0f;        // gait mode: this swing's duration
    bool idle_ = true;        // gait mode: standing (next walk starts fresh)
    int start_ = static_cast<int>(Foot::Left);  // gait mode: foot that started the last walk
    float stanceWidth_ = 0.2f;
    float t_ = 0.0f;          // swing progress [0, 1]
    float liftAt_ = 0.0f;     // single steps: share of the step before the foot lifts
    int swing_ = -1;          // swinging foot index, -1 when both are planted
    int last_ = static_cast<int>(Foot::Right);
};

} // namespace motion
