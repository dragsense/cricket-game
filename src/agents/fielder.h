#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, step the feet,
// reposition the body over the foot targets (late update), solve two-bone
// IK for the legs, keep the feet flat, then clamp every joint to its limits.

#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/body_reposition.h"
#include "motion/foot_stepper.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

// IK goals in world space.
struct IkTargets {
    math::Vec3 footL{0.0f}, footR{0.0f};
};

struct IkSettings {
    math::Vec3 kneePole{0, 0, 1};
    math::Vec3 elbowPole{0.3f, 0, -1};
    float soften = 0.97f;
    float chestAimWeight = 0.3f;
    float headAimWeight = 1.0f;
};

class Fielder {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/fielder_joints.json) and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    const IkTargets& targets() const { return targets_; }
    // Requested step direction in character space (x = left, y = forward),
    // zero for none. Holding a direction keeps stepping, alternating feet.
    void setStepInput(math::Vec2 dir) { stepInput_ = dir; }

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;

private:
    void plantFeet();
    void repositionBody(float dt);
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
    motion::RepositionSettings reposition_;
    float stanceBlendHeight_ = 0.1f;
    motion::StepSettings steps_;
    motion::FootStepper stepper_;
    math::Vec2 stepInput_{0.0f};
    Joints joints_;
    IkTargets targets_;
    math::Vec3 pelvis_{0.0f};  // pelvis position in character space, kept between ticks
    rig::Pose previous_, current_;
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_;
};

} // namespace agents
