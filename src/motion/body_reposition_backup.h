#pragma once
// Backup (the user's call, 2026-11-03: body_reposition, motion::BodyReposition, takes over the body from the feet).
// Body repositioning from the foot targets (a "late update" after the feet are
// set). One formula, whatever moved a foot:
//  - where: between the feet by the weight each carries;
//  - how high: as high as every loaded leg reaches straight (standReach), the
//    lowest deciding; a lifted foot sets nothing;
//  - on one foot the lifted side drops stanceDrop (BIOMECHANICS 4.3);
//  - a damped spring moves it there; the leg clamp keeps both feet reachable.
// Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct RepositionSettings {
    float standReach = 0.95f;  // comfortable hip-to-foot distance (fraction of leg length)
    float minReach = 0.5f;     // deepest crouch (the knee fully bent; set from the joint limits)
    float maxReach = 0.98f;    // legs never fully straight
    float softBand = 0.02f;    // the max reach eases in over this share of leg length (0: hard)
    float followRate = 10.0f;  // 1/s, the spring's natural frequency
    float damping = 1.0f;      // spring damping ratio (1: settles without overshoot)
    float sideFollow = 1.0f;   // share of the sideways shift over the stance foot the body takes
    float forwardFollow = 1.0f; // share of the forward/back shift over the feet (0: stay at restZ)
    float restZ = 0.0f;        // pelvis forward position when not following the feet
    float stanceDrop = 0.07f;  // radians, the unloaded side's hip drop with all weight on one leg
    float standShare = 1.0f;   // share of the standing rules (straight-leg height, hip drop); 0 moving
    int iterations = 4;
};

struct LegSupport {
    math::Vec3 hipOffset{0.0f};  // hip relative to the pelvis (rest, unrotated, for repositionBody)
    math::Vec3 foot{0.0f};       // foot (ankle) target
};

// The pelvis kept between ticks: position and hip drop, each with its spring
// velocity, and the position it is heading for.
struct BodyState {
    math::Vec3 position{0.0f};
    math::Vec3 velocity{0.0f};
    math::Vec3 goal{0.0f};
    float drop = 0.0f, dropRate = 0.0f;  // about +Z; positive raises the left hip

    math::Quat rotation() const;
};

// The leg clamp on its own: moves `pelvis` so each hip is within reach of its
// foot: `first` up to maxReach, `second` within [minReach, maxReach]
// (x legLength, soft top), `first` then `second`, s.iterations times.
math::Vec3 clampToLegs(math::Vec3 pelvis, const LegSupport& first, const LegSupport& second, float legLength,
                       const RepositionSettings& s);

// Knee-limited reach: hip-to-ankle distance with the knee bent `kneeFlex`
// radians (law of cosines over thigh and shin).
float bentReach(float thigh, float shin, float kneeFlex);

// Advances `body` one tick toward the feet. `stanceWeight` in [0, 1]: 0 = left
// leg carries the body, 1 = right leg. All points in the same space, +Y up.
void repositionBody(BodyState& body, const LegSupport& left, const LegSupport& right, float legLength,
                    float stanceWeight, const RepositionSettings& s, float dt);


} // namespace motion
