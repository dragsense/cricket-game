#pragma once
// The planner's orders to the body's decider (DECISIONS 2026-11-10 "Whole-body
// decider on targets"): the seven targets (hands, feet, hips, chest, head),
// each with a pose, a mode, weights and (feet) the ground contact. An action
// (idle, step, walk, reach, dive, ...) only places these over time; the
// decider knows nothing of actions. Character space (+X left, +Y up, +Z
// forward), the root being the player's capsule.
//
// Modes:
//  Free: the target is not given; the decider uses the part's own frame:
//   feet world-pinned where they last were on the ground, hips over the
//   weighted support feet, chest upright on the hips, head level in the
//   world, hands at rest in the chest's frame.
//  Driven: the pose is followed, by its weights, from wherever the part is
//   now (a path starts from the current plan; the decider blends the mode in).
//  Pinned (feet): on the ground at the given spot, flat to `contactNormal`,
//   carrying weight; it never slides.
// Mode and weight changes never switch at once: the decider blends them
// over its blendTime.

#include "math/math.h"

namespace motion {

enum class TargetMode { Free, Driven, Pinned };

struct Target {
    TargetMode mode = TargetMode::Free;
    math::Vec3 position{0.0f};
    math::Quat rotation = math::identityQuat();  // the part's orientation (identity: upright, facing forward)
    math::Vec3 velocity{0.0f};                   // m/s, the target's own motion
    math::Vec3 angularVelocity{0.0f};            // rad/s
    float weightPosition = 1.0f;  // 0..1: how fully the position is followed when Driven
    float weightRotation = 1.0f;  // 0..1: how fully the rotation is followed when Driven
    // Feet: a timed swing's share gone, toe-off 0 to heel strike 1 (negative:
    // no timing, a foot just held up), and the ground's normal under it.
    float swing = -1.0f;
    math::Vec3 contactNormal = math::kUp;
};

struct TargetSet {
    Target hand[2];  // left, right: the wrist
    Target foot[2];  // left, right: the ankle
    Target hips;     // the pelvis
    Target chest;    // rotation only
    Target head;     // rotation only (gaze)
    // The hips' position weight split: up/down and across (both 0..1).
    float hipsWeightVertical = 1.0f, hipsWeightHorizontal = 1.0f;
    float dt = 0.0f;        // s, this tick (the fixed physics step)
    float strength = 1.0f;  // 0..1, the whole body's: hits, falls, get-up (scales motors and the pin)
};

} // namespace motion
