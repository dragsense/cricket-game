// Generic object physics on simple objects (no human): capsule shapes,
// hinge and swing-twist joints, and joint motors with a torque limit.

#include "physics/core/physics_world.h"

#include "core/config.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr float kTick = 1.0f / 120.0f;
constexpr float kPi = 3.14159265f;

bool makeWorld(physics::PhysicsWorld& w) {
    core::Config c;
    return c.load(CRICKET_CONFIG_DIR "/physics_world.json") &&
           w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c));
}

// A fixed point to hang things from: static and touching nothing.
physics::BodyId anchorAt(physics::PhysicsWorld& w, math::Vec3 p) {
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.02f;
    d.dynamic = false;
    d.collides = false;
    d.pose.position = p;
    return w.createBody(d);
}

// A 2 kg, 0.5 m arm (capsule along X) from `pivot` out to +X.
physics::BodyId armFrom(physics::PhysicsWorld& w, math::Vec3 pivot) {
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Capsule;
    d.radius = 0.04f;
    d.halfHeight = 0.21f;  // 0.5 m tip to tip with the rounded ends
    d.mass = 2.0f;
    d.collides = false;  // only the joint holds it
    d.pose.position = pivot + math::Vec3(0.25f, 0.0f, 0.0f);
    d.pose.rotation = math::fromAxisAngle(math::Vec3(0, 0, 1), -0.5f * kPi);  // capsule +Y along +X
    return w.createBody(d);
}

void run(physics::PhysicsWorld& w, float seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 120.0f); ++i) w.step(kTick);
}

} // namespace

TEST_CASE("physics joints: a capsule lying on the ground rests on its side") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Capsule;
    d.radius = 0.05f;
    d.halfHeight = 0.2f;
    d.mass = 3.0f;
    d.restitution = 0.0f;
    d.pose.position = math::Vec3(0.0f, 1.0f, 0.0f);
    d.pose.rotation = math::fromAxisAngle(math::Vec3(0, 0, 1), 0.5f * kPi);  // lying
    const physics::BodyId id = w.createBody(d);
    REQUIRE(id != physics::kInvalidBody);
    run(w, 3.0f);
    CHECK(w.transform(id).position.y == doctest::Approx(0.05f).scale(0.0).epsilon(0.05));
}

TEST_CASE("physics joints: a free hinge swings under gravity and stays inside its limits") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    const math::Vec3 pivot(0.0f, 2.0f, 0.0f);
    const physics::BodyId a = anchorAt(w, pivot), arm = armFrom(w, pivot);
    physics::HingeDesc h;
    h.pivot = pivot;
    h.axis = math::Vec3(0, 0, 1);    // swings in the X-Y plane
    h.normal = math::Vec3(1, 0, 0);  // angle 0: the arm level
    h.minAngle = -0.5f;
    h.maxAngle = 0.5f;
    const physics::JointId j = w.createHinge(a, arm, h);
    REQUIRE(j != physics::kInvalidJoint);
    float lowest = 0.0f;
    for (int i = 0; i < 2 * 120; ++i) {
        w.step(kTick);
        lowest = std::min(lowest, w.hingeAngle(j));
    }
    MESSAGE("free hinge: lowest angle ", lowest);
    CHECK(lowest < -0.4f);   // gravity pulled it down
    CHECK(lowest > -0.55f);  // the limit stopped it
}

TEST_CASE("physics joints: a strong motor holds the arm level; a weak one cannot") {
    // Gravity's torque on the arm at the pivot: m g d = 2 * 9.81 * 0.25 = 4.9 N m.
    for (const float maxTorque : {50.0f, 1.0f}) {
        physics::PhysicsWorld w;
        REQUIRE(makeWorld(w));
        const math::Vec3 pivot(0.0f, 2.0f, 0.0f);
        const physics::BodyId a = anchorAt(w, pivot), arm = armFrom(w, pivot);
        physics::HingeDesc h;
        h.pivot = pivot;
        h.axis = math::Vec3(0, 0, 1);
        h.normal = math::Vec3(1, 0, 0);
        const physics::JointId j = w.createHinge(a, arm, h);
        physics::MotorDesc m;
        m.frequency = 4.0f;
        m.damping = 1.0f;
        m.maxTorque = maxTorque;
        w.setJointMotor(j, m, true);
        w.setHingeTarget(j, 0.0f);
        run(w, 2.0f);
        MESSAGE("motor ", maxTorque, " N m: angle ", w.hingeAngle(j));
        if (maxTorque > 4.9f) {
            // A spring-driven motor gives under load by load / stiffness;
            // Jolt's stiffness is the arm's inertia about its centre times
            // (2 pi f)^2: 2 * 0.5^2 / 12 * (8 pi)^2 = 26.3 N m/rad, so it
            // holds 4.9 N m about 0.19 rad below level.
            const float k = 2.0f * 0.25f / 12.0f * std::pow(2.0f * kPi * m.frequency, 2.0f);
            CHECK(w.hingeAngle(j) == doctest::Approx(-4.905f / k).scale(0.0).epsilon(0.1));
        } else
            CHECK(w.hingeAngle(j) < -1.0f);  // too weak: the arm drops
    }
}

TEST_CASE("physics joints: a swing-twist motor turns the body to its target rotation") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    const math::Vec3 pivot(0.0f, 2.0f, 0.0f);
    const physics::BodyId a = anchorAt(w, pivot);
    // A 2 kg arm hanging straight down from the pivot.
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Capsule;
    d.radius = 0.04f;
    d.halfHeight = 0.21f;
    d.mass = 2.0f;
    d.collides = false;
    d.pose.position = pivot - math::Vec3(0.0f, 0.25f, 0.0f);
    const physics::BodyId arm = w.createBody(d);
    physics::SwingTwistDesc s;
    s.pivot = pivot;
    s.twistAxis = math::Vec3(0, -1, 0);  // along the arm
    s.planeAxis = math::Vec3(1, 0, 0);
    s.normalHalfCone = s.planeHalfCone = 1.0f;
    s.minTwist = -0.5f;
    s.maxTwist = 0.5f;
    const physics::JointId j = w.createSwingTwist(a, arm, s);
    REQUIRE(j != physics::kInvalidJoint);
    physics::MotorDesc m;
    m.frequency = 4.0f;
    m.damping = 1.0f;
    m.maxTorque = 50.0f;
    w.setJointMotor(j, m, true);
    // 0.5 rad of swing about the joint's plane axis.
    const math::Quat target = math::fromAxisAngle(math::Vec3(0, 0, 1), 0.5f);
    w.setSwingTwistTarget(j, target);
    run(w, 2.0f);
    const math::Quat now = w.swingTwistRotation(j);
    const float off = 2.0f * std::acos(std::min(1.0f, std::fabs(math::dot(now, target))));
    MESSAGE("swing-twist: off target by ", off, " rad");
    CHECK(off < 0.1f);
}

namespace {

// Rotation about the joint's Z axis (rad) for a mostly-Z rotation.
float aboutZ(math::Quat q) { return 2.0f * std::atan2(q.z, q.w); }

// A level 2 kg arm on a SixDOF joint at the pivot, reaching to side (+1: +X,
// -1: -X); turning about Z is limited to [-0.3, 1.2] rad, the other axes to
// +-0.05 rad.
physics::JointId unevenArm(physics::PhysicsWorld& w, math::Vec3 pivot, float side) {
    const physics::BodyId a = anchorAt(w, pivot);
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Capsule;
    d.radius = 0.04f;
    d.halfHeight = 0.21f;
    d.mass = 2.0f;
    d.collides = false;
    d.pose.position = pivot + math::Vec3(0.25f * side, 0.0f, 0.0f);
    d.pose.rotation = math::fromAxisAngle(math::Vec3(0, 0, 1), -0.5f * kPi);
    const physics::BodyId arm = w.createBody(d);
    physics::SixDofDesc s;
    s.pivot = pivot;
    s.minAngles = math::Vec3(-0.05f, -0.05f, -0.3f);
    s.maxAngles = math::Vec3(0.05f, 0.05f, 1.2f);
    return w.createSixDof(a, arm, s);
}

} // namespace

TEST_CASE("physics joints: a SixDOF joint keeps uneven real ranges (little one way, far the other)") {
    // Arm to +X: gravity turns it about -Z, where the joint allows only 0.3.
    // Arm to -X: gravity turns it about +Z, where the joint allows 1.2.
    for (const float side : {1.0f, -1.0f}) {
        physics::PhysicsWorld w;
        REQUIRE(makeWorld(w));
        const physics::JointId j = unevenArm(w, math::Vec3(0.0f, 2.0f, 0.0f), side);
        REQUIRE(j != physics::kInvalidJoint);
        run(w, 2.0f);
        const float z = aboutZ(w.sixDofRotation(j));
        MESSAGE("SixDOF arm to ", side > 0 ? "+X" : "-X", ": rests at ", z, " rad about Z");
        if (side > 0)
            CHECK(z == doctest::Approx(-0.3f).scale(0.0).epsilon(0.1));
        else
            CHECK(z == doctest::Approx(1.2f).scale(0.0).epsilon(0.1));
    }
}

TEST_CASE("physics joints: a SixDOF motor drives the joint to its target inside the range") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    const physics::JointId j = unevenArm(w, math::Vec3(0.0f, 2.0f, 0.0f), 1.0f);
    physics::MotorDesc m;
    m.frequency = 6.0f;
    m.damping = 1.0f;
    m.maxTorque = 100.0f;
    w.setJointMotor(j, m, true);
    w.setSixDofTarget(j, math::fromAxisAngle(math::Vec3(0, 0, 1), 0.8f));  // raise it 0.8 rad
    run(w, 2.0f);
    const float z = aboutZ(w.sixDofRotation(j));
    MESSAGE("SixDOF motor: at ", z, " rad (target 0.8; gives a little under the arm's weight)");
    CHECK(z == doctest::Approx(0.8f).scale(0.0).epsilon(0.1));
}

TEST_CASE("physics joints: a kinematic body turned dynamic falls under gravity") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.1f;
    d.mass = 1.0f;
    d.kinematic = true;
    d.pose.position = math::Vec3(0.0f, 3.0f, 0.0f);
    const physics::BodyId b = w.createBody(d);
    run(w, 0.5f);
    CHECK(w.transform(b).position.y == doctest::Approx(3.0f));  // kinematic: held
    w.setKinematic(b, false);
    run(w, 0.5f);
    // Dynamic: falls g t^2 / 2 = 1.23 m in 0.5 s.
    CHECK(w.transform(b).position.y == doctest::Approx(3.0f - 0.5f * 9.81f * 0.25f).epsilon(0.03));
}

TEST_CASE("physics joints: a hinge velocity motor turns the arm at its speed; a weak one cannot lift it") {
    // Gravity's torque on the arm at the pivot: 4.9 N m (see above).
    for (const float maxTorque : {50.0f, 1.0f}) {
        physics::PhysicsWorld w;
        REQUIRE(makeWorld(w));
        const math::Vec3 pivot(0.0f, 2.0f, 0.0f);
        const physics::BodyId a = anchorAt(w, pivot), arm = armFrom(w, pivot);
        physics::HingeDesc h;
        h.pivot = pivot;
        h.axis = math::Vec3(0, 0, 1);
        h.normal = math::Vec3(1, 0, 0);
        const physics::JointId j = w.createHinge(a, arm, h);
        physics::MotorDesc m;
        m.maxTorque = maxTorque;
        w.setJointMotor(j, m, true);
        w.setHingeVelocity(j, 0.5f);  // raise it at 0.5 rad/s
        run(w, 1.0f);
        MESSAGE("velocity motor ", maxTorque, " N m: angle after 1 s ", w.hingeAngle(j));
        if (maxTorque > 4.9f)
            CHECK(w.hingeAngle(j) == doctest::Approx(0.5f).epsilon(0.05));
        else
            CHECK(w.hingeAngle(j) < 0.0f);  // too weak: the arm drops
    }
}

TEST_CASE("physics joints: a SixDOF velocity motor turns at its speed and holds at zero") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    const physics::JointId j = unevenArm(w, math::Vec3(0.0f, 2.0f, 0.0f), 1.0f);
    physics::MotorDesc m;
    m.maxTorque = 100.0f;
    w.setJointMotor(j, m, true);
    w.setSixDofVelocity(j, math::Vec3(0.0f, 0.0f, 0.4f));  // raise it at 0.4 rad/s
    run(w, 1.0f);
    const float raised = aboutZ(w.sixDofRotation(j));
    MESSAGE("SixDOF velocity motor: ", raised, " rad after 1 s");
    CHECK(raised == doctest::Approx(0.4f).epsilon(0.05));
    w.setSixDofVelocity(j, math::Vec3(0.0f));  // hold
    run(w, 1.0f);
    CHECK(aboutZ(w.sixDofRotation(j)) == doctest::Approx(raised).epsilon(0.05));
}

TEST_CASE("physics joints: a torque spins a free body by torque / inertia") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.1f;
    d.mass = 1.0f;
    d.collides = false;
    d.pose.position = math::Vec3(0.0f, 5.0f, 0.0f);
    const physics::BodyId b = w.createBody(d);
    const float inertia = 0.4f * 1.0f * 0.1f * 0.1f;  // solid sphere: 2/5 m r^2
    for (int i = 0; i < 60; ++i) {  // 0.5 s at I rad/s^2 worth of torque: 0.5 rad/s
        w.addTorque(b, math::Vec3(0.0f, inertia, 0.0f));
        w.step(kTick);
    }
    CHECK(w.angularVelocity(b).y == doctest::Approx(0.5f).epsilon(0.05));  // less a little angular damping
}

TEST_CASE("physics joints: a SixDOF axis spring holds a load at k theta = torque") {
    // The 2 kg arm (gravity's torque at the pivot 2 x 9.81 x 0.25 = 4.9 N m),
    // its Z axis on a 100 N m/rad spring aimed level every step: it rests
    // 4.9 / 100 = 0.049 rad low.
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    const physics::JointId j = unevenArm(w, math::Vec3(0.0f, 2.0f, 0.0f), 1.0f);
    for (int i = 0; i < 3 * 120; ++i) {
        w.setSixDofAxisSpring(j, 2, -aboutZ(w.sixDofRotation(j)), 100.0f, 10.0f, 1000.0f);
        w.step(kTick);
    }
    const float z = aboutZ(w.sixDofRotation(j));
    MESSAGE("SixDOF axis spring 100 N m/rad under 4.9 N m: rests at ", z, " rad (expected -0.049)");
    CHECK(z == doctest::Approx(-0.049f).epsilon(0.1));
}
