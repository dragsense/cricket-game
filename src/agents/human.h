#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. The limb test only (the user's call, 2026-11-16: walking,
// running and the jump removed): with the physics body, the decider plans the
// whole body from the limb test's targets (its balance steps too) and the
// muscles pull toward that plan; without it, the fielder stands.

#include "agents/human_body.h"
#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/foot_stepper.h"
#include "motion/target_set.h"
#include "motion/body_decider.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

struct IkSettings {
    math::Vec3 kneePole{0, 0, 1};  // knees point this way (the decider's legs)
};

class Human {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/human_joints.json) and the step settings (config/locomotion.json),
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
    // The step under way: its foot (-1: none).
    int stepFoot() const { return decider_.stepFoot(); }
    // The real pelvis off the hold's target (physics_world.json
    // "character.pelvisError"): Small is nothing, Medium sustained, Large
    // lets go (when the release is on).
    enum class PelvisError { Small, Medium, Large };
    // The targets given to the decider this tick and its answer, for debug
    // drawing (character space of root()).
    const motion::TargetSet& bodyTargets() const { return bodyTargets_; }
    // Debug drawing: the capsule (its feet, height and radius; world) and
    // the line the release reads, from the centre of the feet to the hips
    // (the physics body, world). False without a body.
    bool capsuleDebug(math::Vec3& feet, float& height, float& radius, math::Vec3& feetCentre, math::Vec3& hips) const;
    const HumanBody& humanBody() const { return humanBody_; }
    HumanBody& humanBody() { return humanBody_; }
    // Re-reads config/locomotion.json without moving the fielder.
    void loadLocomotion(const core::Config& locomotion);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    // The body decider's last answer (its plan: valid until the next tick).
    const motion::BodyQueue& bodyQueue() const { return queue_; }
    const motion::BodyDecider& decider() const { return decider_; }
    // Stands the fielder at `position` (world) facing `yaw` radians, feet at rest.
    void placeAt(math::Vec3 position, float yaw);

    // Limb test (physics only, the user's call): a goal for any one of the
    // seven targets (motion::Goal: a hand's wrist, a foot's ankle, the hips,
    // the chest, the head), where it goes (world; hands, feet and hips) and,
    // once `turned`, how it is turned (world; every part). Only the target
    // set last drives; the rest holds and balances. Nothing while physics is
    // off. clearPose() drops the goals.
    void setPoseGoal(motion::Goal goal, math::Vec3 world, math::Quat rotation, bool turned);
    void setPoseHand(bool left, math::Vec3 world);
    void setPoseFoot(motion::Foot foot, math::Vec3 world);
    void clearPose();
    // The rig joint a target moves (its bone: wrist, ankle, pelvis, chest, head).
    int goalJoint(motion::Goal goal) const;
    // Every part back at the planned standing pose, at rest, the hold on
    // again; the muscles pull from there.
    void resetPhysicsBody();
    // Wrist of one hand in world space, from a model-space pose.
    math::Vec3 wrist(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.armL : joints_.armR).end)][3]));
    }
    // Ankle of one foot in world space, from a model-space pose.
    math::Vec3 ankle(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.legL : joints_.legR).end)][3]));
    }
    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
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
    // The capsule (Jolt CharacterVirtual) follows the real hips: it slides
    // along what it hits and keeps to the ground; the root takes where it
    // ends up, the planted feet staying where they are in the world.
    void followCapsule(float dt);

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
    // Limb test: the target set last (motion::Goal, -1 none: only it drives),
    // and each target's goal (world space): where, how turned, turned yet.
    int poseLast_ = -1;
    math::Vec3 posePosition_[7]{};
    math::Quat poseRotation_[7]{};
    bool poseTurned_[7]{};
    // A lifted foot's ankle stays at least this high (character space): its
    // standing height plus the minimum toe clearance (locomotion.json
    // "step.toeClearance"), so its target never runs along or into the ground.
    float toeClearance_ = 0.012f;
    float liftedAnkleMin_ = 0.0f;
    float ballReach_ = 0.0f;  // m, ankle to the ball of the foot, level (rest): push-off rolls over it
    // Physics: a hand's or foot's commanded point travels from where it was
    // to its goal in Fitts' time on a minimum-jerk path (Flash & Hogan 1985);
    // kept in the world (the root moves under it); a new goal (a press)
    // starts a new path from where the point is.
    struct GoalPath {
        math::Vec3 from{0.0f}, to{0.0f}, at{0.0f};
        float time = 0.0f, duration = 0.0f;
        int end = -1;  // the rig joint it moves (wrist or ankle)
        bool on = false;
    };
    GoalPath handPath_, footPath_, hipsPath_;
    // The decider's balance step's shape (locomotion.json "step").
    motion::StepSettings stepSettings_;
    float toeOffPitch_ = 0.0f;  // rad, the foot's plantarflexion at toe-off (step.toeOffPitchDeg)
    HumanBody humanBody_;                  // the full-physics body (Jolt Ragdoll)
    physics::PhysicsWorld* physics_ = nullptr;
    physics::CharacterId capsule_ = physics::kInvalidCharacter;  // carries the root over the ground
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
    // The real pelvis's error from the hold's target.
    PelvisError pelvisError_ = PelvisError::Small;
    float pelvisErrorNow_ = 0.0f, pelvisErrorTime_ = 0.0f;

    // Where each foot is planted (character space): the limb test's feet.
    motion::FootStepper stepper_;
    // A balance step's passive shares (locomotion.json "step.passive").
    motion::PassiveShares stepPassive_;
    Joints joints_;
    rig::Pose previous_, current_;
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_, previousRoot_;
};

} // namespace agents
