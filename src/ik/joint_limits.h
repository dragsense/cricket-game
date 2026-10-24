#pragma once
// Joint limits: hinge and swing-twist (elliptical cone + twist range), applied
// to local joint rotations after every IK solve. Rotations are relative to the
// rest pose, whose joint frames are aligned with the character (see
// config/fielder_joints.json). Our own code: no listed library provides it.

#include "math/math.h"
#include "rig/rig.h"

#include <vector>

namespace core {
class Config;
}

namespace ik {

enum class LimitType { Free, Hinge, SwingTwist };

// Angles in radians.
struct JointLimit {
    LimitType type = LimitType::Free;
    int axis = 1;  // hinge axis or twist axis: 0 = x, 1 = y, 2 = z
    float twistMin = 0.0f, twistMax = 0.0f;    // hinge range uses these too
    float swing1Min = 0.0f, swing1Max = 0.0f;  // first remaining axis (x, y, z order)
    float swing2Min = 0.0f, swing2Max = 0.0f;  // second remaining axis
};

// Clamps a local rotation to the limit. Returns the clamped rotation.
math::Quat clampRotation(math::Quat q, const JointLimit& limit);

// Limit mirrored for the opposite side (rotations about Y and Z flip sign).
JointLimit mirrored(const JointLimit& limit);

// One limit per rig joint.
class JointLimits {
public:
    // Reads config/fielder_joints.json "joints"; names without a side suffix
    // apply to the left bone and, mirrored, to the right. Load-time only.
    bool load(const core::Config& config, const rig::Rig& rig);

    // Clamps every joint's local rotation in place. No allocations.
    void apply(rig::Pose& pose) const;

    const JointLimit& limit(int joint) const { return limits_[static_cast<size_t>(joint)]; }

private:
    std::vector<JointLimit> limits_;
};

} // namespace ik
