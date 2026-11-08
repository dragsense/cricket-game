#pragma once
// Joint limits: hinge and swing-twist (elliptical cone + twist range), applied
// to local joint rotations after every IK solve. Rotations are relative to the
// rest pose, whose joint frames are aligned with the character (see
// config/human_joints.json). Our own code: no listed library provides it.

#include "math/math.h"
#include "rig/rig.h"

#include <vector>

namespace core {
class Config;
}

namespace ik {

enum class LimitType { Free, Hinge, SwingTwist };

// One range bound set by another joint's angle (a two-joint muscle: the
// gastrocnemius limits the ankle's dorsiflexion while the knee is straight).
// The bound goes linearly from value0 at angle0 to value1 at angle1 of the
// coupled hinge, held outside that span (the measured points only).
struct LimitCoupling {
    enum class Bound { TwistMin, TwistMax, Swing1Min, Swing1Max, Swing2Min, Swing2Max };
    int joint = -1;  // the coupled hinge (rig joint), -1: none
    Bound bound = Bound::Swing1Min;
    float angle0 = 0.0f, angle1 = 0.0f;  // the coupled hinge's angle, rad
    float value0 = 0.0f, value1 = 0.0f;  // the bound there, rad
};

// Angles in radians.
struct JointLimit {
    LimitType type = LimitType::Free;
    int axis = 1;  // hinge axis or twist axis: 0 = x, 1 = y, 2 = z
    float twistMin = 0.0f, twistMax = 0.0f;    // hinge range uses these too
    float swing1Min = 0.0f, swing1Max = 0.0f;  // first remaining axis (x, y, z order)
    float swing2Min = 0.0f, swing2Max = 0.0f;  // second remaining axis
    LimitCoupling coupling;                    // optional: one bound set by another joint
};

// A hinge limit's angle of a local rotation (about its axis, rad).
float hingeAngle(math::Quat local, const JointLimit& hinge);

// `limit` with its coupled bound set for the coupled hinge at `angle` (rad).
JointLimit coupledLimit(const JointLimit& limit, float angle);

// Clamps a local rotation to the limit, measured the way Jolt's joints do
// (hinge: the angle about its axis; swing-twist: twist about the twist axis,
// then a pyramid swing: the angle about each swing axis clamped on its own),
// so the pose and the physics body read the ranges alike. Returns the
// clamped rotation.
math::Quat clampRotation(math::Quat q, const JointLimit& limit);

// Limit mirrored for the opposite side (rotations about Y and Z flip sign).
JointLimit mirrored(const JointLimit& limit);

// One limit per rig joint.
class JointLimits {
public:
    // Reads config/human_joints.json "joints"; names without a side suffix
    // apply to the left bone and, mirrored, to the right. Load-time only.
    bool load(const core::Config& config, const rig::Rig& rig);

    // Clamps every joint's local rotation in place (a coupled bound by its
    // coupled joint in the same pose). No allocations.
    void apply(rig::Pose& pose) const;

    // The joint's range as configured (a coupled bound at its first value).
    const JointLimit& limit(int joint) const { return limits_[static_cast<size_t>(joint)]; }
    // The joint's range in `pose` (a coupled bound by its coupled joint there).
    JointLimit limitIn(int joint, const rig::Pose& pose) const;

private:
    std::vector<JointLimit> limits_;
};

} // namespace ik
