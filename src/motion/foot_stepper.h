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

// Swing shape and foot roll for gait mode; the defaults are walking (flat
// feet, centred arc, standing width).
struct GaitStyle {
    float toeOffPitch = 0.0f;   // rad, plantarflexion at toe-off (positive = toes down)
    float landPitch = 0.0f;     // rad, foot pitch at landing
    float peakAt = 0.5f;        // swing time share of the arc apex
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
    // Foot `last` has just landed (e.g. from a jump): any step in progress
    // ends and the gait carries on with the other foot next, as running (the
    // stance spots are kept; reset() would move them to where the feet are).
    void landed(Foot last) {
        swing_ = -1;
        t_ = wait_ = 0.0f;
        idle_ = false;
        last_ = static_cast<int>(last);
    }
    // Gait mode: the swing in progress takes `scale` times as long (from
    // now), so it lands that much further on and rises that much higher.
    void stretchSwing(float scale) {
        if (swing_ < 0) return;
        t_ /= scale;
        dur_ *= scale;
        stretch_ = scale;
    }
    // Gait mode: the next swing to start is stretched by `scale` (as above).
    void stretchNext(float scale) { nextStretch_ = scale; }
    float swingTimeLeft() const { return swing_ >= 0 ? (1.0f - t_) * dur_ : 0.0f; }
    // Gait mode: the stance spots are turned by `angle` (rad, about +Y) from
    // the character's facing, so the feet keep stepping along a run line
    // while the body turns.
    void setStanceTurn(float angle) { stanceTurn_ = angle; }
    // Puts foot `f` at `spot` (character space) at once, flat, without a step.
    void place(Foot f, math::Vec3 spot);

    bool stepping() const { return swing_ >= 0; }
    // Foot `f` is off the ground (swinging).
    bool inAir(Foot f) const { return swing_ == static_cast<int>(f); }
    int swingFoot() const { return swing_; }     // -1 when both are planted
    float swingProgress() const { return t_; }  // [0, 1] while stepping
    // Foot pitch in character space (rad, positive = toes down) and toe bend
    // (rad, negative = toes up, keeping them on the ground while the heel rises).
    float footPitch(Foot f) const { return pitch_[static_cast<int>(f)]; }
    float toeBend(Foot f) const { return toe_[static_cast<int>(f)]; }

private:
    bool startStep(math::Vec2 dir, const StepSettings& s);
    void advanceSwing(float dt, float duration, float height, float peakAt = 0.5f, float liftAt = 0.0f,
                      float liftOffSpeed = 0.0f);

    math::Vec3 feet_[2]{};    // current positions (swing foot moves along the arc)
    math::Vec3 from_{0.0f}, to_{0.0f};
    math::Vec3 stance_[2]{};  // gait mode's standing spots
    float pitch_[2]{}, toe_[2]{};
    float wait_ = 0.0f;       // gait mode: time both feet have been planted
    float dur_ = 0.0f;        // gait mode: this swing's duration
    float stretch_ = 1.0f, nextStretch_ = 1.0f;  // gait mode: this / the next swing's stretch
    float stanceTurn_ = 0.0f;  // gait mode: stance spots' turn from the facing
    bool idle_ = true;        // gait mode: standing (next walk starts fresh)
    int start_ = static_cast<int>(Foot::Left);  // gait mode: foot that started the last walk
    float stanceWidth_ = 0.2f;
    float t_ = 0.0f;          // swing progress [0, 1]
    float liftAt_ = 0.0f;     // single steps: share of the step before the foot lifts
    int swing_ = -1;          // swinging foot index, -1 when both are planted
    int last_ = static_cast<int>(Foot::Right);
};

} // namespace motion
