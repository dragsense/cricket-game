#pragma once
// The human body in the shared physics world, built only from generic parts
// (PhysicsWorld; DECISIONS 2026-11-05): one Jolt body per bone that has mass
// (shape from the skeleton's visual, mass = the bone's de Leva share of the
// body mass), joined at each bone's joint by a Jolt joint with the real ranges
// from human_joints.json (the same ranges BodyReposition uses): hinge for the
// hinge joints (knee, elbow, toes), SixDOF for the rest (uneven ranges). A
// bone without mass of its own (neck, clavicles, fingers) rides its parent.
// The parts of one body do not collide with each other; they collide with
// the ground and with everything else.
// Muscles (BIOMECHANICS 14; DECISIONS 2026-11-06): each joint's velocity
// motor turns it the way the decider's queue says, no harder than that
// joint's real strength, after holding the weight beyond it (gravity
// compensation). Nothing balances the body yet: it can tip over.
// Load-time allocation only.

#include "ik/joint_limits.h"
#include "math/math.h"
#include "motion/body_decider.h"
#include "physics/core/physics_world.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace agents {

// config/human_joints.json "muscles" (BIOMECHANICS 14).
struct MuscleSettings {
    float activationTime = 0.03f;  // s, muscle excitation to force (10-50 ms): how fast a motor pulls
    float damping = 1.0f;          // ratio; 1 = reaches the pose without overshoot
    // Strongest torque per joint (N m), by bone name without its side
    // ("thigh" for thigh_l and thigh_r). A joint not listed has no muscle.
    std::vector<std::pair<std::string, float>> maxTorque;
    // A limb joint following its target pose: the spring's stiffness (N m/rad)
    // and damping (N m s/rad) by bone name (human_joints.json "muscles").
    std::vector<std::pair<std::string, float>> stiffness, jointDamping;
};

// How fast a muscle closes its target's gap: Fitts' law movement time
// fittsA + fittsB log2(2D / fittsWidth) (human_joints.json "reach";
// BIOMECHANICS 9.6). A turn target has no Fitts' law: it closes in fittsA.
struct MuscleTiming {
    float fittsA = 0.1f, fittsB = 0.1f, fittsWidth = 0.1f;
};

class HumanBody {
public:
    // Builds the body standing in the rig's rest pose at `root` (world), with
    // its muscles on. `group` (> 0) keeps this body's parts from colliding
    // with each other. False if the world is full.
    bool init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
              const ik::JointLimits& limits, const MuscleSettings& muscles, const math::Transform& root,
              uint32_t group);
    bool ready() const { return !parts_.empty(); }

    struct Part {
        int joint = -1;                    // rig joint the body belongs to
        physics::BodyId body = physics::kInvalidBody;
        float mass = 0.0f;                 // kg
        int parent = -1;                   // index of the part it is joined to (-1: none)
        physics::JointId link = physics::kInvalidJoint;  // its joint to the parent part
        math::Transform shape;             // the body (its shape) in the bone's frame
        physics::ShapeType shapeType = physics::ShapeType::Sphere;  // its Jolt shape and size (for drawing)
        float radius = 0.0f, halfHeight = 0.0f;  // sphere, capsule (half the straight part, local +Y)
        math::Vec3 halfExtents{0.0f};            // box
        bool hinge = false;                // hinge (else SixDOF)
        math::Vec3 frameX{1, 0, 0}, frameY{0, 1, 0}, frameZ{0, 0, 1};  // the joint's axes in the bone frame
        bool muscle = false;               // its motor is on
        float strength = 0.0f;             // N m, its muscle's strongest torque (human_joints.json "muscles")
        float stiffness = 0.0f;            // N m/rad, its spring toward a target pose (0: none)
        float damping = 0.0f;              // N m s/rad, that spring's damping
    };
    const std::vector<Part>& parts() const { return parts_; }
    // Per rig joint (`numJoints`): its part's mass (kg, 0 if none of its own)
    // and that mass's centre in the bone's frame. Load time.
    void jointMasses(int numJoints, std::vector<float>& mass, std::vector<math::Vec3>& centre) const;
    float totalMass() const { return totalMass_; }

    // The planned pose (`models`, character space, the rig's joints) as each
    // muscle's target: the bone's rotation relative to its parent part's bone.
    // Kinematic parts (for now): each part moves to its bone in the planned
    // pose (`models`, character space of `root`) this step.
    void follow(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root,
                float dt) const;

    void setTargets(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models) const;

    // Physics on (every part dynamic: gravity, muscles, contacts) or off
    // (kinematic, placed by follow), keeping each part's velocity. Idle
    // muscles take their held angles afresh.
    void setPhysics(physics::PhysicsWorld& world, bool on);
    // Every part placed at its bone in `models` (character space of `root`),
    // at rest; idle muscles take their held angles afresh.
    void reset(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root);

    // Each muscle holds the weight of every part beyond it, away from the
    // pelvis (a torque on the part and the opposite on its parent), no harder
    // than its strength. Right for limbs held off the trunk; standing legs
    // carry the body from the feet (the balance's job, later). Call before
    // drive. `owned` (one entry per part, may be null): parts whose joint
    // the balancer drives (a stance ankle), left out. No allocations.
    void compensateGravity(physics::PhysicsWorld& world, const uint8_t* owned = nullptr) const;

    // The muscles from the decider's queue. Each active turn order turns its
    // joint along its direction at its share of the speed that closes its
    // target's gap in Fitts' time plus the target's own motion, as that joint
    // sees it (the end's needed velocity across the joint's lever; a turn
    // target: its angular gap). A joint with no order holds the angle it had
    // when its last order ended (its motor toward that angle).
    // Each motor pulls no harder than the strength left after holding the
    // weight. A joint riding its parent (clavicle, neck) has no muscle of its
    // own; the hips' moves are the legs' and the balance's. `models`: the
    // physics body now (character space). `driven` (one entry per part, may
    // be null): joints with an axis the balancer drives this step
    // (Balancer::driven); one it lets go of holds the angle it has then. No
    // allocations.
    void drive(physics::PhysicsWorld& world, const motion::BodyQueue& queue, const motion::BodyGoals& goals,
               const motion::BodyDeciderJoints& joints, const std::vector<math::Mat4>& models,
               const MuscleTiming& timing, const uint8_t* driven = nullptr);

    // The joints that cannot go further the way their orders pull: at their
    // limit (measured the way Jolt's joints are, ik::clampRotation), with no
    // muscle of their own, or with the muscle at its full strength in the last
    // step of `dt` s. `out`: one entry per rig joint. No allocations.
    void blockedJoints(const physics::PhysicsWorld& world, const motion::BodyQueue& queue,
                       const std::vector<math::Mat4>& models, const ik::JointLimits& limits, float dt,
                       std::vector<uint8_t>& out) const;

    // The drawn skeleton from the bodies (character space of `root`; `alpha`
    // between the last two steps): a bone with a body takes the body's pose,
    // a bone without one its planned local rotation (`pose`) on its parent.
    void boneModels(const physics::PhysicsWorld& world, float alpha, const rig::Rig& rig, const rig::Pose& pose,
                    const math::Transform& root, std::vector<math::Mat4>& models) const;

    // Centre of mass of all parts (world) after the last step; its height.
    math::Vec3 centreOfMass(const physics::PhysicsWorld& world) const;
    float height(const physics::PhysicsWorld& world) const { return centreOfMass(world).y; }
    // Velocity of the centre of mass (world, m/s) after the last step.
    math::Vec3 centreOfMassVelocity(const physics::PhysicsWorld& world) const;
    // The part a rig joint's bone belongs to (-1: it rides its parent).
    int partOf(int joint) const { return joint >= 0 && joint < static_cast<int>(partOf_.size()) ? partOf_[static_cast<size_t>(joint)] : -1; }

private:
    std::vector<Part> parts_;
    std::vector<int> partOf_;  // per rig joint: its part, -1 if it rides
    float totalMass_ = 0.0f;
    // Per part: holding an angle (no order since it was taken) and the angle
    // (joint frame as created: SixDOF rotation, hinge angle).
    std::vector<uint8_t> holding_;
    std::vector<uint8_t> wasDriven_;  // per part: the balancer drove an axis of it last step
    std::vector<math::Quat> heldRotation_;
    std::vector<float> heldAngle_;
    // Scratch, sized at init (no per-step allocation; mutable: not the body's state).
    mutable std::vector<math::Vec3> omega_;         // per part: its muscle's angular velocity (character space)
    std::vector<uint8_t> ordered_;                  // per part: an order turns it this step
    std::vector<uint8_t> targeted_;                 // per part: it pulls to a target pose this step
    std::vector<math::Quat> targetLocal_;           // per part: that target (its bone on its parent's)
    mutable std::vector<float> hold_;               // per part: the torque its muscle spends holding the weight (N m)
    mutable std::vector<float> beyondMass_;         // per part: mass of it and every part beyond it
    mutable std::vector<math::Vec3> beyondMoment_;  // per part: their mass x centre of mass (world)
};

} // namespace agents
