#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, walk (move and
// turn the root, step the feet on the gait), reposition the body over the
// foot targets (late update), add pelvis bob, sway, lean and twist, solve two-bone
// IK for the legs, keep the feet flat, swing the arms opposite the legs,
// steady the head, reach for the hand target if there is one (standing
// only; docs/BIOMECHANICS.md section 9), then clamp every joint to its limits.

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

// Hand-driven reaching (config/fielder_joints.json "reach").
struct ReachSettings {
    float armReach = 0.97f;         // share of full arm length the hand reaches; the elbow never locks
    float wristAboveTarget = 0.08f; // m; wrist goal above the target (the palm is on the ball)
    float maxSpeed = 0.05f;         // m/s; reaching runs only below this (standing)
    // Spine stage: share of the bend and of the twist taken by spine_01,
    // spine_02 and chest (each sums to 1); iterations absorb clamping.
    math::Vec3 spineBend{0.6f, 0.2f, 0.2f};
    math::Vec3 spineTwist{0.2f, 0.4f, 0.4f};
    int spineIterations = 2;
    // Pelvis stage limits (radians), as a rotation vector in character space:
    // x = forward tilt (hip flexion; + forward), y = turn, z = side tilt.
    math::Vec3 pelvisMin{math::radians(-10.0f), math::radians(-30.0f), math::radians(-20.0f)};
    math::Vec3 pelvisMax{math::radians(80.0f), math::radians(30.0f), math::radians(20.0f)};
    int bodyIterations = 16;  // body stage: push toward the target, slide along the legs
    float footTolerance = 0.01f;  // m; an ankle further than this from its target counts as moved
    int plantChecks = 6;          // bisection steps that scale the pelvis/body back to keep the feet
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

    // Hand target in world space (e.g. a ball on the ground). The nearer hand
    // reaches for it while the fielder stands; walking pauses the reach.
    void setReachTarget(math::Vec3 world) {
        reachTarget_ = world;
        reaching_ = true;
    }
    void clearReach() { reaching_ = false; }
    bool reaching() const { return reaching_; }
    // Distance (m) the hand is still short of the target after the last tick;
    // 0 when reached. Later stages (spine, pelvis, body, step) take it on.
    float reachError() const { return reachError_; }

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
    // Reaching, split around the leg IK: spine, then pelvis and body before it
    // (the feet stay planted), clavicle and arm after the walk arm swing.
    void reachSpine();
    // Pelvis rotation (scaled by `scale` in [0, 1]), then the body translation.
    void reachHips(float scale);
    // reachHips + leg IK, scaled back (bisection) until the joint limits keep
    // both ankles on their planted foot targets.
    void reachHipsWithPlantedFeet();
    bool feetPlanted();
    void reachArm();
    math::Vec3 reachGoal() const;  // wrist goal, character space
    // Applies a model-space rotation to `joint` (about its own position),
    // clamped to its limit. `models_` must be current.
    void rotateInModel(int joint, math::Quat rotation);
    motion::GaitStyle gaitStyle() const;

    struct Joints {
        int pelvis = -1, chest = -1, head = -1, toeL = -1, toeR = -1, clavicleL = -1, clavicleR = -1;
        int spine[3] = {-1, -1, -1};  // spine_01, spine_02, chest
        ik::TwoBoneChain legL, legR, armL, armR;
    };

    rig::SkeletonDef def_;
    rig::Rig rig_;
    ik::JointLimits limits_;
    const core::Config* jointsConfig_ = nullptr;
    IkSettings ik_;
    ReachSettings reach_;
    float armLength_ = 0.6f;  // shoulder to wrist (m), from the skeleton
    math::Vec3 reachTarget_{0.0f};
    bool reaching_ = false;
    bool reachLeft_ = true;  // this tick's reaching hand
    float reachError_ = 0.0f;
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
    rig::Pose reachBase_;  // scratch: pose after the spine stage, sized at build
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_, previousRoot_;
};

} // namespace agents
