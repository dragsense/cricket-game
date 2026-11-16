#pragma once
// Where each foot is planted (character space: +X left, +Y up, +Z forward):
// the limb test's feet. The decider's balance steps move them (place); the
// root moving keeps them where they are in the world (rebase). Also the swing
// arc a step's foot follows. No allocations.

#include "math/math.h"

#include <algorithm>

namespace motion {

// A balance step's shape (locomotion.json "step").
struct StepSettings {
    float stepDuration = 0.4f;    // seconds per step
    float stepHeight = 0.08f;     // swing arc apex above the ground
    float peakAt = 0.5f;          // swing time share of the arc apex (early: knee bends, heel up)
    float liftAt = 0.0f;          // step time share the foot stays down first (weight moves off it)
};

// Swing height share, 0 -> 1 at `peakAt` -> 0 over the swing time share t.
// Eases in and out on both sides, so the foot never jumps up at lift-off
// however early the apex is.
float swingArc(float t, float peakAt);

enum class Foot : int { Left = 0, Right = 1 };

class FootStepper {
public:
    // Plants both feet.
    void reset(math::Vec3 left, math::Vec3 right);
    // The character root moved from `from` to `to` (world): keeps the feet
    // where they are in the world, re-expressed in the new character space.
    void rebase(const math::Transform& from, const math::Transform& to);
    math::Vec3 foot(Foot f) const { return feet_[static_cast<int>(f)]; }
    // Puts foot `f` at `spot` (character space) at once.
    void place(Foot f, math::Vec3 spot) { feet_[static_cast<int>(f)] = spot; }

private:
    math::Vec3 feet_[2]{};
};

} // namespace motion
