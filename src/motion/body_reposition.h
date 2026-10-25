#pragma once
// Body repositioning (a "late update" after the foot targets are set): moves
// the pelvis so both feet stay reachable. The body eases toward a comfortable
// height above the support point (both feet, weighted by stance), then each
// leg's hip-to-foot distance is clamped to [minReach, maxReach] * leg length,
// lighter leg first so the stance leg wins when both can't be satisfied.
// Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct RepositionSettings {
    float standReach = 0.95f;  // comfortable hip-to-foot distance (fraction of leg length)
    float minReach = 0.5f;     // deepest crouch
    float maxReach = 0.98f;    // legs never fully straight
    float softBand = 0.02f;    // the max reach eases in over this share of leg length (0: hard)
    float followRate = 10.0f;  // 1/s, how fast the body settles toward the comfortable height
    float sideFollow = 1.0f;   // share of the sideways shift over the stance foot the body takes
    float forwardFollow = 1.0f; // share of the forward/back shift over the feet (0: stay at restZ)
    float restZ = 0.0f;        // pelvis forward position when not following the feet
    int iterations = 4;
};

struct LegSupport {
    math::Vec3 hipOffset{0.0f};  // hip relative to the pelvis
    math::Vec3 foot{0.0f};       // foot (ankle) target
};

// Returns the new pelvis position. `stanceWeight` in [0, 1]: 0 = left leg
// carries the body, 1 = right leg. All points in the same space, +Y up.
math::Vec3 repositionBody(math::Vec3 pelvis, const LegSupport& left, const LegSupport& right, float legLength,
                          float stanceWeight, const RepositionSettings& s, float dt);

} // namespace motion
