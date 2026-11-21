#pragma once
// The human body in the shared physics world: a Jolt Ragdoll (PhysicsWorld;
// DECISIONS 2026-11-09 "Full physics body on the capsule"). One dynamic body
// per bone that has mass (shape from the skeleton's visual, mass = the bone's
// de Leva share of the body mass), joined at each bone's joint by a Jolt
// SixDOF joint with the plan's own ranges from human_joints.json, in the
// joint's rest frame (ik::limitFrame: the same set ik::clampRotation keeps
// the pose in; a hinge turns only about its axis; a range set by another
// joint's angle follows the real body's joint each tick). Each axis's motor
// is its muscle (human_joints.json "muscles": strength, activation time),
// pulling the joint toward the pose. A bone without mass of its own (neck,
// clavicles, fingers) rides its parent. The parts of one body do not collide
// with each other; they collide with the ground and with everything else.
// Load-time allocation only.

#include "ik/joint_limits.h"
#include "math/math.h"
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
    // A strength per direction about one axis of the joint (its two muscles,
    // e.g. knee extensors and flexors), by bone name without its side: the
    // axis (0 x, 1 y, 2 z, the joint limits' frame) and the strongest torque
    // toward + and - rotation about it, left side (the right side mirrors).
    // The joint's other axes keep maxTorque.
    struct Directional {
        std::string bone;
        int axis = 0;
        float plus = 0.0f, minus = 0.0f;
    };
    std::vector<Directional> directional;
};

// How fast a muscle closes its target's gap: Fitts' law movement time
// fittsA + fittsB log2(2D / fittsWidth) (human_joints.json "reach";
// BIOMECHANICS 9.6).
struct MuscleTiming {
    float fittsA = 0.1f, fittsB = 0.1f, fittsWidth = 0.1f;
    // A goal that moves more than goalJump (m) in one tick jumped (a press: a
    // new path); one moving more than goalMoved after its path ended starts one.
    float goalJump = 0.02f, goalMoved = 1e-3f;
};

class HumanBody {
public:
    // Builds the ragdoll standing in the rig's rest pose at `root` (world).
    // `group` (> 0) keeps this body's parts from colliding with each other.
    // False if the world is full.
    bool init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
              const ik::JointLimits& limits, const MuscleSettings& muscles, const math::Transform& root,
              uint32_t group);
    bool ready() const { return !parts_.empty(); }

    struct Part {
        int joint = -1;                    // rig joint the body belongs to
        physics::BodyId body = physics::kInvalidBody;
        float mass = 0.0f;                 // kg
        int parent = -1;                   // index of the part it is joined to (-1: none, the pelvis)
        math::Transform shape;             // the body (its shape) in the bone's frame
        physics::ShapeType shapeType = physics::ShapeType::Sphere;  // its Jolt shape and size (for drawing)
        float radius = 0.0f, halfHeight = 0.0f;  // sphere, capsule (half the straight part, local +Y)
        math::Vec3 halfExtents{0.0f};            // box
        bool hinge = false;                // turns about frameX only
        math::Vec3 frameX{1, 0, 0}, frameY{0, 1, 0}, frameZ{0, 0, 1};  // the joint's axes in the bone frame (X twist)
        bool muscle = false;               // its motor has strength
        int coupledPart = -1;              // its range follows this part's joint angle (-1: fixed range)
    };
    const std::vector<Part>& parts() const { return parts_; }
    // Per rig joint (`numJoints`): its part's mass (kg, 0 if none of its own)
    // and that mass's centre in the bone's frame. Load time.
    void jointMasses(int numJoints, std::vector<float>& mass, std::vector<math::Vec3>& centre) const;
    float totalMass() const { return totalMass_; }

    // Every muscle carries the weight of the parts beyond it (away from the
    // pelvis), as real muscles hold posture: a torque on the part and the
    // opposite on its parent. The hold and Jolt's motor (drivePose) share
    // the muscle's strength: drivePose gives the hold within it and the motor
    // corrects with what is left. Call each tick before drivePose. No
    // allocations.
    void holdWeight(physics::PhysicsWorld& world) const;
    // A part's hold from the last holdWeight (world, N m) and the ragdoll (for measuring).
    math::Vec3 holdTorque(int part) const { return hold_[static_cast<size_t>(part)]; }
    physics::RagdollId ragdoll() const { return ragdoll_; }
    // Every muscle pulls its joint toward the pose `models` (character space,
    // the rig's joints; a range set by another joint first follows that
    // joint in the real body, as the plan's limitIn); the head is
    // held in space: its muscle's target is the pose's head in the world
    // (`root`: character to world) seen from the real body it hangs from, not
    // from the posed one (the vestibulo-collic reflex keeps the head stable
    // in space, Pozzo et al. 1990), within its range. `headJoint`: the rig
    // joint of the head.
    // No allocations.
    void drivePose(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root,
                   int headJoint) const;
    // A part's joint range now: the plan's, a coupled bound set by the real
    // body's coupled joint (as JointLimits::limitIn reads the pose's).
    ik::JointLimit limitNow(const physics::PhysicsWorld& world, int part) const;
    // The body (character space of `root`) a part has in the pose `models`.
    math::Transform partPose(int part, const std::vector<math::Mat4>& models) const;
    // Every part placed at its bone in `models` (character space of `root`), at rest.
    void reset(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root) const;

    // The drawn skeleton from the bodies (character space of `root`; `alpha`
    // between the last two steps): a bone with a body takes the body's pose,
    // a bone without one its planned local rotation (`pose`) on its parent.
    void boneModels(const physics::PhysicsWorld& world, float alpha, const rig::Rig& rig, const rig::Pose& pose,
                    const math::Transform& root, std::vector<math::Mat4>& models) const;

    // Velocity of the centre of mass (world, m/s) after the last step.
    math::Vec3 centreOfMassVelocity(const physics::PhysicsWorld& world) const;
    // A part's bone (its joint) in the world after the last step.
    math::Transform boneWorld(const physics::PhysicsWorld& world, int part) const;
    // The part a rig joint's bone belongs to (-1: it rides its parent).
    int partOf(int joint) const { return joint >= 0 && joint < static_cast<int>(partOf_.size()) ? partOf_[static_cast<size_t>(joint)] : -1; }

private:
    std::vector<Part> parts_;
    std::vector<int> partOf_;  // per rig joint: its part, -1 if it rides
    float totalMass_ = 0.0f;
    physics::RagdollId ragdoll_ = physics::kInvalidRagdoll;
    const ik::JointLimits* limits_ = nullptr;  // the ranges (owned by Human, outlives the body)
    mutable std::vector<math::Quat> rotations_;  // scratch: the pose drive's targets, sized at init
    mutable std::vector<float> beyondMass_;         // scratch per part: mass of it and every part beyond it
    mutable std::vector<math::Vec3> beyondMoment_;  // scratch per part: their mass x centre of mass (world)
    mutable std::vector<math::Vec3> hold_;          // scratch per part: the torque its weight needs (world)
};

} // namespace agents
