#pragma once
// The human body in the shared physics world, built only from generic parts
// (PhysicsWorld; DECISIONS 2026-11-05): one Jolt body per bone that has mass
// (shape from the skeleton's visual, mass = the bone's de Leva share of the
// body mass), joined at each bone's joint by a Jolt joint with the real ranges
// from human_joints.json (the same ranges BodyReposition uses): hinge for the
// hinge joints (knee, elbow, toes), SixDOF for the rest (uneven ranges). A
// bone without mass of its own (neck, clavicles, fingers) rides its parent.
// The parts of one body do not collide with each other; they collide with
// the ground and with everything else. No muscles yet (motors off): the body
// falls and folds under gravity within its joint ranges.
// Load-time allocation only.

#include "ik/joint_limits.h"
#include "math/math.h"
#include "physics/core/physics_world.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace agents {

class HumanBody {
public:
    // Builds the body standing in the rig's rest pose at `root` (world).
    // `group` (> 0) keeps this body's parts from colliding with each other.
    // False if the world is full.
    bool init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
              const ik::JointLimits& limits, const math::Transform& root, uint32_t group);
    bool ready() const { return !parts_.empty(); }

    struct Part {
        int joint = -1;                    // rig joint the body belongs to
        physics::BodyId body = physics::kInvalidBody;
        float mass = 0.0f;                 // kg
        int parent = -1;                   // index of the part it is joined to (-1: none)
        physics::JointId link = physics::kInvalidJoint;  // its joint to the parent part
        math::Transform shape;             // the body (its shape) in the bone's frame
    };
    const std::vector<Part>& parts() const { return parts_; }
    float totalMass() const { return totalMass_; }

    // Centre of mass of all parts (world) after the last step; its height.
    math::Vec3 centreOfMass(const physics::PhysicsWorld& world) const;
    float height(const physics::PhysicsWorld& world) const { return centreOfMass(world).y; }

private:
    std::vector<Part> parts_;
    float totalMass_ = 0.0f;
};

} // namespace agents
