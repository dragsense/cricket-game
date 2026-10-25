#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, walk (move and
// turn the root, step the feet on the gait), reposition the body over the
// foot targets (late update), add pelvis bob, sway, lean and twist, solve two-bone
// IK for the legs, keep the feet flat, swing the arms opposite the legs,
// steady the head, then clamp every joint to its limits.

#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/body_reposition.h"
#include "motion/foot_stepper.h"
#include "motion/walk_controller.h"
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
    // (config/fielder_joints.json) and walk/run settings (config/locomotion.json),
    // and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion);
    // Re-reads config/locomotion.json without moving the fielder.
    void loadLocomotion(const core::Config& locomotion);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    const IkTargets& targets() const { return targets_; }
    // Requested walk in character space (x = left, y = forward), length 0..1
    // scales the speed; zero stops. With `face`, turns toward world heading
    // `faceYaw` (radians about +Y).
    void setMoveInput(math::Vec2 move, bool face, float faceYaw) {
        moveInput_ = move;
        face_ = face;
        faceYaw_ = faceYaw;
    }
    // Pace (config/locomotion.json "walk", "jog", "run"). The speed eases to
    // the new pace and the gait shape blends with the actual speed.
    void setGait(motion::Gait gait) { gait_ = gait; }
    motion::Gait gait() const { return gait_; }
    float speed() const { return walker_.speed(); }  // m/s

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }
    math::Transform interpolatedRoot(float alpha) const { return math::lerp(previousRoot_, root_, alpha); }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;

private:
    void plantFeet();
    void walk(float dt);
    void repositionBody(float dt);
    void addGaitMotion(float dt);
    void swingArms(float dt);
    void stabiliseHead(float dt);
    void solveIk();
    motion::GaitStyle gaitStyle() const;

    struct Joints {
        int pelvis = -1, chest = -1, head = -1, toeL = -1, toeR = -1;
        ik::TwoBoneChain legL, legR, armL, armR;
    };

    rig::SkeletonDef def_;
    rig::Rig rig_;
    ik::JointLimits limits_;
    const core::Config* jointsConfig_ = nullptr;
    IkSettings ik_;
    motion::RepositionSettings reposition_;
    float stanceBlendHeight_ = 0.1f;
    float ballDistance_ = 0.1f;  // ankle to ball of the foot (m), from the skeleton
    void blendShape();

    motion::WalkSettings walkSettings_, jogSettings_, runSettings_;  // jog starts from walk, run from jog
    motion::WalkSettings control_;  // chosen pace: target speeds, acceleration, turn rate
    motion::WalkSettings shape_;    // gait shape blended by the actual speed
    motion::Gait gait_ = motion::Gait::Walk;
    motion::WalkController walker_;
    motion::FootStepper stepper_;
    math::Vec2 moveInput_{0.0f};
    bool face_ = false;
    float faceYaw_ = 0.0f;
    math::Vec2 lean_{0.0f};  // smoothed (forward, into-turn) lean, radians
    float accel_ = 0.0f;     // smoothed forward acceleration for the lean, m/s^2
    float legPhase_ = 0.0f;  // left leg fore-aft position, -1 back .. +1 forward
    float armAmount_ = 0.0f; // smoothed arm swing scale 0..1
    float headYaw_ = 0.0f;   // smoothed head turn ahead of the body, radians
    Joints joints_;
    IkTargets targets_;
    math::Vec3 pelvis_{0.0f};  // pelvis position in character space, kept between ticks
    rig::Pose previous_, current_;
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_, previousRoot_;
};

} // namespace agents
