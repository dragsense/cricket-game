#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. M1 Step 1.1: holds the neutral standing pose.

#include "math/math.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

namespace agents {

class Fielder {
public:
    // Builds the skeleton at `height` metres and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;

private:
    rig::SkeletonDef def_;
    rig::Rig rig_;
    rig::Pose previous_, current_;
    math::Transform root_;
};

} // namespace agents
