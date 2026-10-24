#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, move the pelvis,
// aim chest and head, solve two-bone IK for legs and arms, keep the feet
// flat, then clamp every joint to its limits.

#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

// IK goals in world space (the pelvis offset is in character space).
struct IkTargets {
    math::Vec3 footL{0.0f}, footR{0.0f};
    math::Vec3 handL{0.0f}, handR{0.0f};
    math::Vec3 lookAt{0.0f};
    math::Vec3 pelvisOffset{0.0f};
};

struct IkSettings {
    math::Vec3 kneePole{0, 0, 1};
    math::Vec3 elbowPole{0.3f, 0, -1};
    float soften = 0.97f;
    float chestAimWeight = 0.3f;
    float headAimWeight = 1.0f;
};

struct IkDemoSettings {
    float period = 6.0f;
    float handReach = 0.9f;
    float footLift = 0.35f;
    float footSwing = 0.5f;
    float pelvisDrop = 0.35f;
    float lookRadius = 3.0f;
};

class Fielder {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/fielder_joints.json) and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    // Debug: targets follow paths that drive every limb to and past its limits.
    void setIkDemo(bool enabled) { ikDemo_ = enabled; }
    bool ikDemo() const { return ikDemo_; }
    const IkTargets& targets() const { return targets_; }

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;

private:
    void updateDemoTargets();
    void solveIk();

    struct Joints {
        int pelvis = -1, chest = -1, head = -1;
        ik::TwoBoneChain legL, legR, armL, armR;
    };

    rig::SkeletonDef def_;
    rig::Rig rig_;
    ik::JointLimits limits_;
    const core::Config* jointsConfig_ = nullptr;
    IkSettings ik_;
    IkDemoSettings demo_;
    Joints joints_;
    IkTargets targets_;
    bool ikDemo_ = false;
    double demoTime_ = 0.0;
    rig::Pose previous_, current_;
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_;
};

} // namespace agents
