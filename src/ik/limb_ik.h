#pragma once
// Two-bone and aim IK on a rig pose, using ozz-animation's IKTwoBoneJob and
// IKAimJob. Inputs are model-space joint matrices (from Rig::localToModel);
// the solvers write their corrections into the pose's local rotations.
// ozz types stay inside limb_ik.cpp. No allocations.

#include "math/math.h"
#include "rig/rig.h"

#include <vector>

namespace ik {

struct TwoBoneChain {
    int start = -1, mid = -1, end = -1;  // e.g. thigh, shin, foot
    math::Vec3 midAxis{1, 0, 0};         // mid joint's bend axis, in its local space
};

// Rotates `start` and `mid` so `end` reaches `target` (model space), bending
// toward `pole` (model-space direction). Returns false if the job is invalid.
bool solveTwoBone(const TwoBoneChain& chain, const std::vector<math::Mat4>& models, math::Vec3 target,
                  math::Vec3 pole, float soften, rig::Pose& pose);

// Rotates `joint` so its local `forward` axis points at `target` (model
// space), keeping its local `up` toward `pole`. `weight` in [0, 1].
bool solveAim(int joint, const std::vector<math::Mat4>& models, math::Vec3 target, math::Vec3 forward,
              math::Vec3 up, math::Vec3 pole, float weight, rig::Pose& pose);

} // namespace ik
