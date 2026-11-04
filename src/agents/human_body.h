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
// Muscles (BIOMECHANICS 14): each joint's motor drives it toward the planned
// pose (setTargets), no harder than that joint's real strength, switching on
// as fast as muscle activation allows. Nothing balances the body yet: it
// keeps its shape but can tip over.
// Load-time allocation only.

#include "ik/joint_limits.h"
#include "math/math.h"
#include "physics/core/physics_world.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

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
        bool hinge = false;                // hinge (else SixDOF)
        math::Vec3 frameX{1, 0, 0}, frameY{0, 1, 0}, frameZ{0, 0, 1};  // the joint's axes in the bone frame
        bool muscle = false;               // its motor is on
    };
    const std::vector<Part>& parts() const { return parts_; }
    float totalMass() const { return totalMass_; }

    // The planned pose (`models`, character space, the rig's joints) as each
    // muscle's target: the bone's rotation relative to its parent part's bone.
    void setTargets(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models) const;

    // The drawn skeleton from the bodies (character space of `root`; `alpha`
    // between the last two steps): a bone with a body takes the body's pose,
    // a bone without one its planned local rotation (`pose`) on its parent.
    void boneModels(const physics::PhysicsWorld& world, float alpha, const rig::Rig& rig, const rig::Pose& pose,
                    const math::Transform& root, std::vector<math::Mat4>& models) const;

    // Centre of mass of all parts (world) after the last step; its height.
    math::Vec3 centreOfMass(const physics::PhysicsWorld& world) const;
    float height(const physics::PhysicsWorld& world) const { return centreOfMass(world).y; }

private:
    std::vector<Part> parts_;
    std::vector<int> partOf_;  // per rig joint: its part, -1 if it rides
    float totalMass_ = 0.0f;
};

} // namespace agents
