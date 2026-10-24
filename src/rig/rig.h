#pragma once
// Runtime skeleton: builds an ozz-animation Skeleton from a SkeletonDef at a
// given height (RawSkeleton -> SkeletonBuilder) and converts local poses to
// model space with ozz's LocalToModelJob. ozz types stay inside rig.cpp.
//
// Joint indices are ozz's (depth-first) order, which differs from the config
// order; use findJoint() / boneDefIndex() to map between them.
// build() allocates; localToModel() does not.

#include "math/math.h"
#include "rig/skeleton_def.h"

#include <memory>
#include <string>
#include <vector>

namespace rig {

// Local (parent-relative) transform per joint, in joint order.
struct Pose {
    std::vector<math::Transform> locals;
};

class Rig {
public:
    Rig();
    ~Rig();
    Rig(Rig&&) noexcept;
    Rig& operator=(Rig&&) noexcept;

    // (Re)builds the skeleton with every ratio scaled by `height` metres.
    bool build(const SkeletonDef& def, float height);
    bool valid() const;

    float height() const { return height_; }
    int numJoints() const { return static_cast<int>(parents_.size()); }
    int parent(int joint) const { return parents_[static_cast<size_t>(joint)]; }
    const std::string& jointName(int joint) const { return names_[static_cast<size_t>(joint)]; }
    int findJoint(const std::string& name) const;  // -1 if missing
    int boneDefIndex(int joint) const { return defIndex_[static_cast<size_t>(joint)]; }

    // Rest (neutral standing) pose, scaled to the current height.
    const Pose& restPose() const { return rest_; }
    // Thigh + shin length of one leg, in metres.
    float legLength() const { return legLength_; }

    // Model-space (relative to the character root) matrices for every joint.
    // `out` must already hold numJoints() entries. Returns false on error.
    bool localToModel(const Pose& pose, std::vector<math::Mat4>& out) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    float height_ = 0.0f;
    float legLength_ = 0.0f;
    std::vector<int> parents_;
    std::vector<int> defIndex_;
    std::vector<std::string> names_;
    Pose rest_;
};

} // namespace rig
