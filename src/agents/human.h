#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, walk (turn the
// root, step the feet), reposition the body over the foot targets, solve
// two-bone IK for the legs, keep the feet flat, swing the arms opposite the
// legs, steady the head, then clamp every joint to its limits. With the
// physics body, the decider plans the whole body from the targets (steps,
// jump, limb test) and the muscles pull toward that plan.

#include "agents/human_body.h"
#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/foot_stepper.h"
#include "motion/jump_planner.h"
#include "motion/target_set.h"
#include "motion/body_decider.h"
#include "motion/walk_controller.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

// The old formula repositioning (moved here from motion/body_reposition_backup,
// 2026-11-06), the pose without the physics body.
// One formula, whatever moved a foot:
//  - where: between the feet by the weight each carries;
//  - how high: as high as every loaded leg reaches straight (standReach), the
//    lowest deciding; a lifted foot sets nothing;
//  - on one foot the lifted side drops stanceDrop (BIOMECHANICS 4.3);
//  - a damped spring moves it there; the leg clamp keeps both feet reachable.
struct RepositionSettings {
    float standReach = 0.95f;  // comfortable hip-to-foot distance (fraction of leg length)
    float minReach = 0.5f;     // deepest crouch (the knee fully bent; set from the joint limits)
    float maxReach = 0.98f;    // legs never fully straight
    float softBand = 0.02f;    // the max reach eases in over this share of leg length (0: hard)
    float followRate = 10.0f;  // 1/s, the spring's natural frequency
    float damping = 1.0f;      // spring damping ratio (1: settles without overshoot)
    float sideFollow = 1.0f;   // share of the sideways shift over the stance foot the body takes
    float forwardFollow = 1.0f; // share of the forward/back shift over the feet (0: stay at restZ)
    float restZ = 0.0f;        // pelvis forward position when not following the feet
    float stanceDrop = 0.07f;  // radians, the unloaded side's hip drop with all weight on one leg
    float standShare = 1.0f;   // share of the standing rules (straight-leg height, hip drop); 0 moving
    int iterations = 4;
};

struct LegSupport {
    math::Vec3 hipOffset{0.0f};  // hip relative to the pelvis (rest, unrotated)
    math::Vec3 foot{0.0f};       // foot (ankle) target
};

// The pelvis kept between ticks: position and hip drop, each with its spring
// velocity, and the position it is heading for.
struct BodyState {
    math::Vec3 position{0.0f};
    math::Vec3 velocity{0.0f};
    math::Vec3 goal{0.0f};
    float drop = 0.0f, dropRate = 0.0f;  // about +Z; positive raises the left hip

    math::Quat rotation() const { return math::fromAxisAngle(math::kForward, drop); }
};

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

class Human {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/human_joints.json) and walk settings (config/locomotion.json),
    // and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion);
    // The body in the shared physics world (DECISIONS 2026-11-09 "Full
    // physics body on the capsule"): a capsule (Jolt CharacterVirtual)
    // carries the player over the ground; the body is a full-physics Jolt
    // Ragdoll whose muscles (Jolt's pose drive) pull it toward this tick's
    // pose, held up by a spring from the capsule to the pelvis. Leaning past
    // the release angles lets go: the body falls, then recovers. The feet
    // stand on the world's real surface (Jolt ground query). After init();
    // again after the world is rebuilt. False if the world is full.
    bool attachPhysics(physics::PhysicsWorld& world);
    // The capsule holds the body (not let go after a lean).
    bool supported() const { return supportOn_; }
    // The step under way: its foot (-1: none) and its clock share [0, 1].
    int stepFoot() const { return decider_.stepFoot(); }
    float stepPhase() const { return decider_.stepShare(); }
    // The whole body's strength (0..1): scales every muscle and the capsule's
    // hold (a hit, a fall, a get-up).
    float strength() const { return strength_; }
    void setStrength(float strength) { strength_ = std::clamp(strength, 0.0f, 1.0f); }
    // The real pelvis off the hold's target (physics_world.json
    // "character.pelvisError"): Small is nothing, Medium sustained asks the
    // planner for a recovery step, Large lets go (when the release is on).
    enum class PelvisError { Small, Medium, Large };
    PelvisError pelvisError() const { return pelvisError_; }
    float pelvisErrorDistance() const { return pelvisErrorNow_; }
    // The targets given to the decider this tick and its answer, for debug
    // drawing (character space of root()).
    const motion::TargetSet& bodyTargets() const { return bodyTargets_; }
    // Debug drawing: the capsule (its feet, height and radius; world) and
    // the line the release reads, from the centre of the feet to the hips
    // (the physics body, world). False without a body.
    bool capsuleDebug(math::Vec3& feet, float& height, float& radius, math::Vec3& feetCentre, math::Vec3& hips) const;
    const HumanBody& humanBody() const { return humanBody_; }
    HumanBody& humanBody() { return humanBody_; }
    const ik::JointLimits& jointLimits() const { return limits_; }
    // Re-reads config/locomotion.json without moving the fielder.
    void loadLocomotion(const core::Config& locomotion);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    const IkTargets& targets() const { return targets_; }
    // The body decider's last answer (its plan: valid until the next tick).
    const motion::BodyQueue& bodyQueue() const { return queue_; }
    // Turning: with `face`, turns toward world heading `faceYaw` (radians
    // about +Y). Moving is by steps only (setStepInput).
    void setFacing(bool face, float faceYaw) {
        face_ = face;
        faceYaw_ = faceYaw;
    }
    // Steps (WASD; character space, x = left, y = forward; zero for none):
    // each step swings one foot (locomotion.json "step"), the body follows
    // the feet; held, the steps go on (walking).
    void setStepInput(math::Vec2 dir) { stepInput_ = dir; }
    // The jump key (Space), held: the body crouches; let go: it jumps
    // straight up (the physics body, standing; locomotion.json "jump").
    void setJumpInput(bool held);
    bool jumping() const { return jumpPlanner_.running(); }
    motion::JumpPlanner::Phase jumpPhase() const { return jumpPlanner_.phase(); }
    // Walking pace: +1 faster, -1 slower (locomotion.json "step.speedStep");
    // the next step takes it.
    void changeWalkSpeed(int direction);
    float walkSpeedScale() const { return speedScale_; }
    float speed() const { return walker_.speed(); }  // m/s
    // Stands the fielder at `position` (world) facing `yaw` radians, feet at
    // rest, any jump cleared.
    void placeAt(math::Vec3 position, float yaw);

    // Limb test (physics only, the user's call): a wrist goal for one hand or
    // an ankle goal for one foot (world space) for the physics body; only the
    // limb moved last has a target, the rest holds and balances. Nothing
    // while physics is off. clearPose() drops the goals.
    void setPoseHand(bool left, math::Vec3 world);
    void setPoseFoot(motion::Foot foot, math::Vec3 world);
    void clearPose();
    // Every part back at the planned standing pose, at rest, the hold on
    // again; the muscles pull from there.
    void resetPhysicsBody();
    bool posing() const { return phase_ == Phase::Pose; }
    // Wrist of one hand in world space, from a model-space pose.
    math::Vec3 wrist(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.armL : joints_.armR).end)][3]));
    }
    // Ankle of one foot in world space, from a model-space pose.
    math::Vec3 ankle(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.legL : joints_.legR).end)][3]));
    }
    // Whole-body centre of mass in world space, from the segment masses.
    math::Vec3 centreOfMass();

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }
    math::Transform interpolatedRoot(float alpha) const { return math::lerp(previousRoot_, root_, alpha); }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;
    // The drawn skeleton from the physics body (character space of the
    // interpolated root; `models` sized to the rig). False without a body.
    bool physicalModels(const physics::PhysicsWorld& world, float alpha, std::vector<math::Mat4>& models) const;

private:
    void plantFeet();
    void walk(float dt);
    // The root's move this tick (from `from`, where walk() found it) goes
    // through the capsule (Jolt CharacterVirtual): it slides along what it
    // hits and keeps to the ground; the root takes where it ends up, the
    // planted feet staying where they are in the world.
    void followCapsule(math::Vec3 from, float dt);
    void repositionBody(float dt);
    void swingArms(float dt);
    void stabiliseHead(float dt);
    void solveIk();
    math::Vec3 modelCentreOfMass();  // uses models_, which must be current
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
    enum class Phase { Idle, Pose };
    Phase phase_ = Phase::Idle;
    RepositionSettings reposition_;
    float stanceBlendHeight_ = 0.1f;
    // Limb test: the hand with a wrist goal (its side) and the goal, the foot
    // last placed (motion::Foot, -1 none) and its ankle goal (world space).
    bool poseHand_ = false;
    bool poseLeft_ = false;
    math::Vec3 poseGoal_{0.0f};
    int posedFoot_ = -1;
    bool poseHandLast_ = false;  // the hand was moved after the foot (only it has a target)
    math::Vec3 poseFootGoal_{0.0f};
    // A lifted foot's ankle stays at least this high (character space): its
    // standing height plus the minimum toe clearance (locomotion.json
    // "step.toeClearance"), so its target never runs along or into the ground.
    float toeClearance_ = 0.012f;
    float liftedAnkleMin_ = 0.0f;
    float ballReach_ = 0.0f;  // m, ankle to the ball of the foot, level (rest): push-off rolls over it
    // Physics: a hand's or foot's commanded point travels from where it was
    // to its goal in Fitts' time on a minimum-jerk path (Flash & Hogan 1985);
    // the IK solves for it (character space). A goal moved mid-way retargets.
    struct GoalPath {
        math::Vec3 from{0.0f}, to{0.0f}, at{0.0f};
        float time = 0.0f, duration = 0.0f;
        int end = -1;  // the rig joint it moves (wrist or ankle)
        bool on = false;
    };
    GoalPath handPath_, footPath_;
    math::Vec2 stepInput_{0.0f};    // single steps: requested direction (character space)
    bool stepWalk_ = false;         // single steps in use (standing)
    motion::StepSettings stepSettings_;
    motion::StepSettings baseStep_;  // as loaded: the pace scale's 1
    float speedScale_ = 1.0f, speedStep_ = 1.2f, speedMin_ = 0.5f, speedMax_ = 3.0f;
    float heelLift_[2] = {0.0f, 0.0f};         // radians, smoothed heel lift past the ankle's range (solveIk)
    static constexpr float kHeelRate = 15.0f;  // 1/s, how fast the heel lift follows (no flicker)
    float tickSeconds_ = 1.0f / 120.0f;        // this tick's step
    HumanBody humanBody_;                  // the full-physics body (Jolt Ragdoll)
    physics::PhysicsWorld* physics_ = nullptr;
    physics::CharacterId capsule_ = physics::kInvalidCharacter;  // carries the root over the ground
    float restComHeight_ = 0.0f;               // m, the centre of mass above the root at rest
    float restPelvisHeight_ = 1.0f;            // m, the pelvis above the root at rest
    // The capsule's hold of the body: a hidden kinematic body riding the
    // capsule's feet, sprung to the pelvis (physics_world.json "character").
    physics::BodyId anchor_ = physics::kInvalidBody;
    physics::JointId support_ = physics::kInvalidJoint;
    bool supportOn_ = true;
    float supportShare_ = 1.0f;  // 0..1, eased in after a recovery
    float restTime_ = 0.0f;      // s the let-go body has been at rest
    bool armed_ = true;          // the lean check is on (off after a fall until the body is back up)
    // Each tick, after the pose: the muscles pull toward `target` (character
    // space), the hold follows the capsule, the capsule takes the pelvis's
    // height; a lean past the release angles lets go, rest brings it back.
    void physicsTick(const std::vector<math::Mat4>& target, float dt);
    // The targets (steps, jump, limb test) through the decider: its plan
    // (character space), or null without one.
    const std::vector<math::Mat4>* limbTestPlan(float dt);
    motion::BodyDecider decider_;
    motion::BodyDeciderJoints deciderJoints_;
    motion::BodyDeciderSettings deciderSettings_;
    motion::BodyQueue queue_;
    motion::TargetSet bodyTargets_;
    MuscleTiming muscleTiming_;
    std::vector<math::Mat4> physicsModels_;   // the physics body now (character space)
    rig::Pose physicsPose_;                   // scratch: the physics body's locals (a get-up seeds the plan from it)
    // The body's strength and the real pelvis's error from the hold's target.
    float strength_ = 1.0f, appliedStrength_ = 1.0f;
    PelvisError pelvisError_ = PelvisError::Small;
    float pelvisErrorNow_ = 0.0f, pelvisErrorTime_ = 0.0f;
    float ballDistance_ = 0.1f;  // ankle to ball of the foot (m), from the skeleton
    float ballDrop_ = 0.03f;     // ball of the foot below the ankle (m), from the skeleton

    motion::WalkSettings walkSettings_;  // the only pace (jog and run removed 2026-11-09)
    float postureLean_ = 0.087f;  // rad, walking posture: trunk held slightly forward
    motion::WalkController walker_;
    motion::FootStepper stepper_;
    bool face_ = false;
    float faceYaw_ = 0.0f;
    // A standing jump with the physics body (JumpPlanner); the decider's
    // bounds while it runs.
    motion::JumpPlanner jumpPlanner_;
    motion::JumpPlannerSettings jumpPlan_;
    // Each action's passive shares (locomotion.json "step.passive", "jump.passive").
    motion::PassiveShares stepPassive_, jumpPassive_;
    bool jumpHeld_ = false;
    // WASD's hips target (world): it moves at the walker's velocity.
    math::Vec3 hipsGoal_{0.0f};
    math::Vec3 hipsDirection_ = math::kForward;  // world, level: the way it moves (its active direction)
    bool hipsDriven_ = false;
    bool holdOff_ = false;  // the hold's joint off for the jump's push and flight
    float legPhase_ = 0.0f;  // left leg fore-aft position, -1 back .. +1 forward
    float armAmount_ = 0.0f; // smoothed arm swing scale 0..1
    float headYaw_ = 0.0f;   // smoothed head turn ahead of the body, radians
    Joints joints_;
    IkTargets targets_;
    BodyState body_;  // pelvis position and foot-driven turn/drop in character space, kept between ticks
    rig::Pose previous_, current_;
    // Segment mass model (built with the skeleton): mass share per joint, its
    // single child (segment end) or -1, and the leaf tip in joint space.
    std::vector<float> segmentMass_;
    std::vector<int> segmentEnd_;
    std::vector<math::Vec3> segmentTip_;
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_, previousRoot_;
};

} // namespace agents
