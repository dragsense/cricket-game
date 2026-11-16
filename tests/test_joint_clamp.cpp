// The plan's joint clamp (ik::clampRotation) and the physics body's (Jolt's
// SixDOF clamp of a motor's target) hold the same set: for many random
// rotations a Jolt SixDOF joint built from ik::limitFrame clamps its target
// to the rotation ik::clampRotation gives, for the fielder's real ranges.

#include "core/config.h"
#include "ik/joint_limits.h"
#include "physics/core/physics_world.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

namespace {

// A rotation given in the bone's frame, in a joint frame (columns x, y, z).
math::Quat toFrame(math::Quat q, const ik::LimitFrame& f) {
    const math::Vec3 v(q.x, q.y, q.z);
    return math::Quat(q.w, math::dot(v, f.x), math::dot(v, f.y), math::dot(v, f.z));
}

// The angle (rad) between two rotations, precise near 0 (from the
// difference's vector part, not acos of the dot).
float apart(math::Quat a, math::Quat b) {
    const math::Quat d = math::inverse(math::normalize(a)) * math::normalize(b);
    return 2.0f * std::atan2(math::length(math::Vec3(d.x, d.y, d.z)), std::abs(d.w));
}

} // namespace

TEST_CASE("joint clamp: Jolt's SixDOF clamps a target exactly as the plan's clampRotation") {
    core::Config skel, joints, pw;
    REQUIRE(skel.load(CRICKET_CONFIG_DIR "/human_skeleton.json"));
    REQUIRE(joints.load(CRICKET_CONFIG_DIR "/human_joints.json"));
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    rig::SkeletonDef def;
    REQUIRE(rig::loadSkeletonDef(skel, def));
    rig::Rig rig;
    REQUIRE(rig.build(def, def.height));
    ik::JointLimits limits;
    REQUIRE(limits.load(joints, rig));

    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    // Shoulder, hip, ankle (straight knee and fully coupled), wrist, elbow,
    // knee, both sides.
    const char* names[] = {"upper_arm_l", "upper_arm_r", "thigh_l", "thigh_r", "foot_l", "foot_r",
                           "hand_l",      "hand_r",      "forearm_l", "forearm_r", "shin_l", "toe_r"};
    float y = 1.0f;
    for (const char* name : names) {
        const int j = rig.findJoint(name);
        REQUIRE(j >= 0);
        for (const float knee : {0.0f, math::radians(30.0f)}) {
            const ik::JointLimit lim = ik::coupledLimit(limits.limit(j), knee);
            const ik::LimitFrame f = ik::limitFrame(lim);
            physics::BodyDesc anchor;
            anchor.shape = physics::ShapeType::Sphere;
            anchor.radius = 0.02f;
            anchor.dynamic = false;
            anchor.collides = false;
            anchor.pose.position = math::Vec3(0.0f, y, 0.0f);
            physics::BodyDesc part = anchor;
            part.dynamic = true;
            part.mass = 1.0f;
            part.pose.position = math::Vec3(0.0f, y - 0.2f, 0.0f);
            const physics::BodyId a = world.createBody(anchor), b = world.createBody(part);
            physics::SixDofDesc d;
            d.pivot = anchor.pose.position;
            d.axisX = f.x;
            d.axisY = f.y;
            d.minAngles = f.min;
            d.maxAngles = f.max;
            const physics::JointId joint = world.createSixDof(a, b, d);
            REQUIRE(joint != physics::kInvalidJoint);
            y += 1.0f;
            float worst = 0.0f;
            for (int i = 0; i < 2000; ++i) {
                const math::Quat q = math::normalize(math::Quat(u(rng), u(rng), u(rng), u(rng)));
                world.setSixDofTarget(joint, toFrame(q, f));
                worst = std::max(worst, apart(world.sixDofTarget(joint), toFrame(ik::clampRotation(q, lim), f)));
            }
            const std::string what = std::string(name) + " knee " + std::to_string(knee) + ": worst " + std::to_string(worst) + " rad";
            MESSAGE(what);
            CHECK(worst < 1e-3f);
        }
    }
}
