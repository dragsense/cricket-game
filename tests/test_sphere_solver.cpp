#include "physics/ball/ball_setup.h"
#include "physics/sim/sphere_solver.h"

#include "core/config.h"

#include <doctest/doctest.h>

#include <cmath>

using physics::BodyState;
using physics::BoxCollider;
using physics::SphereSolver;

namespace {

constexpr float kTick = 1.0f / 120.0f;
const physics::GravityForce kGravity;
const physics::DragForce kDrag;

// A cricket-ball-sized body with gravity (and optionally drag); no colliders.
SphereSolver makeSolver(bool drag) {
    SphereSolver s;
    physics::SphereBody body;
    body.mass = 0.16f;
    body.radius = 0.036f;
    body.dragCoefficient = 0.45f;
    s.configure(body, physics::WorldSettings{}, physics::SolverSettings{});
    s.addForce(&kGravity);
    if (drag) s.addForce(&kDrag);
    return s;
}

BoxCollider ground(float restitution) {
    BoxCollider g;
    g.pose.position = math::Vec3(0, -1, 0);
    g.halfExtents = math::Vec3(100, 1, 100);
    g.restitution = restitution;
    return g;
}

void run(const SphereSolver& s, BodyState& b, float seconds) {
    for (int i = 0, n = static_cast<int>(std::lround(seconds / kTick)); i < n; ++i) s.step(b, kTick);
}

} // namespace

TEST_CASE("sphere solver: free fall matches the analytic result") {
    SphereSolver s = makeSolver(false);
    BodyState b;
    b.position = math::Vec3(0, 100, 0);
    run(s, b, 1.0f);
    CHECK(b.velocity.y == doctest::Approx(-9.81f).epsilon(1e-4));
    CHECK(b.position.y == doctest::Approx(100.0f - 0.5f * 9.81f).epsilon(1e-3));
}

TEST_CASE("sphere solver: drag slows a fast ball and caps the fall speed") {
    SphereSolver s = makeSolver(true);
    BodyState b;
    b.position = math::Vec3(0, 1000, 0);
    b.velocity = math::Vec3(0, 0, 40);
    run(s, b, 20.0f / 40.0f);  // about the 20 m from release to the batter
    // k = rho Cd A / 2m = 0.0070/m: ~13 % lost over 20 m (BALL_PHYSICS 3).
    CHECK(b.velocity.z == doctest::Approx(40.0f * std::exp(-0.0070f * b.position.z)).epsilon(0.01));
    CHECK(b.velocity.z < 36.0f);

    BodyState fall;
    fall.position = math::Vec3(0, 5000, 0);
    run(s, fall, 30.0f);
    CHECK(-fall.velocity.y == doctest::Approx(37.4f).epsilon(0.02));  // terminal sqrt(g/k)
}

TEST_CASE("sphere solver: a bounce keeps restitution of the normal speed") {
    SphereSolver s = makeSolver(false);
    s.colliders().add(ground(0.5f));
    BodyState b;
    b.position = math::Vec3(0, 1.0f + 0.036f, 0);
    // Track the apex after the first bounce.
    bool bounced = false;
    float apex = 0.0f;
    for (int i = 0; i < 240; ++i) {
        const float vy = b.velocity.y;
        s.step(b, kTick);
        if (vy < 0.0f && b.velocity.y > 0.0f) bounced = true;
        if (bounced) apex = std::max(apex, b.position.y - 0.036f);
    }
    REQUIRE(bounced);
    CHECK(apex == doctest::Approx(0.25f).epsilon(0.05));  // e^2 * drop height
}

TEST_CASE("sphere solver: bounces die out and the ball comes to rest on the ground") {
    SphereSolver s = makeSolver(false);
    s.colliders().add(ground(0.5f));
    BodyState b;
    b.position = math::Vec3(0, 2, 0);
    run(s, b, 5.0f);
    CHECK(b.position.y == doctest::Approx(0.036f).epsilon(0.01));
    CHECK(std::fabs(b.velocity.y) < 0.1f);
    CHECK(s.touching(b));
}

TEST_CASE("sphere solver: no tunnelling through a thin wall at 45 m/s") {
    SphereSolver s = makeSolver(false);
    BoxCollider wall;
    wall.pose.position = math::Vec3(0, 50, 5);
    wall.halfExtents = math::Vec3(5, 5, 0.005f);  // 1 cm thick
    wall.restitution = 0.5f;
    s.colliders().add(wall);
    BodyState b;
    b.position = math::Vec3(0, 50, 0);
    b.velocity = math::Vec3(0, 0, 45);
    run(s, b, 0.5f);
    CHECK(b.position.z < 5.0f);
    CHECK(b.velocity.z == doctest::Approx(-22.5f).epsilon(0.01));
}

TEST_CASE("sphere solver: sphere and rotated box colliders push along their normal") {
    SphereSolver s = makeSolver(false);
    physics::SphereCollider sphere;
    sphere.centre = math::Vec3(0, 0, 0);
    sphere.radius = 1.0f;
    sphere.restitution = 1.0f;
    s.colliders().add(sphere);
    BodyState b;
    b.position = math::Vec3(0, 3, 0);
    run(s, b, 1.0f);
    CHECK(b.position.y > 1.0f);  // bounced off the top, never inside

    // 45° ramp: a dropped ball leaves sideways (+X here) with no friction.
    SphereSolver r = makeSolver(false);
    BoxCollider ramp;
    ramp.pose.rotation = math::fromAxisAngle(math::Vec3(0, 0, 1), -0.25f * math::kPi);
    ramp.halfExtents = math::Vec3(10, 0.1f, 10);
    ramp.restitution = 0.0f;
    r.colliders().add(ramp);
    BodyState c;
    c.position = math::Vec3(0, 1, 0);
    run(r, c, 0.6f);
    CHECK(c.velocity.x > 1.0f);
}

TEST_CASE("sphere solver: a sliding ball spins up and rolls at 5/7 of its speed") {
    SphereSolver s = makeSolver(false);
    BoxCollider g = ground(0.3f);
    g.friction = 0.5f;
    g.rollingResistance = 0.0f;
    s.colliders().add(g);
    BodyState b;
    b.position = math::Vec3(0, 0.036f, 0);
    b.velocity = math::Vec3(5, 0, 0);  // no spin: slides first
    run(s, b, 1.0f);
    CHECK(b.velocity.x == doctest::Approx(5.0f * 5.0f / 7.0f).epsilon(0.01));  // solid sphere, k = 0.4
    CHECK(-b.angularVelocity.z * 0.036f == doctest::Approx(b.velocity.x).epsilon(0.01));  // rolling: v = w r
}

TEST_CASE("sphere solver: rolling resistance stops a rolling ball at v^2 / (2 C_rr g)") {
    SphereSolver s = makeSolver(false);
    BoxCollider g = ground(0.3f);
    g.rollingResistance = 0.15f;
    s.colliders().add(g);
    BodyState b;
    b.position = math::Vec3(0, 0.036f, 0);
    b.velocity = math::Vec3(5, 0, 0);
    b.angularVelocity = math::Vec3(0, 0, -5.0f / 0.036f);  // already rolling
    run(s, b, 6.0f);
    CHECK(b.position.x == doctest::Approx(25.0f / (2.0f * 0.15f * 9.81f)).epsilon(0.03));
    CHECK(math::length(b.velocity) == doctest::Approx(0.0f));
    CHECK(math::length(b.angularVelocity) == doctest::Approx(0.0f));
}

TEST_CASE("ball setup: size and mass from scene.json, drag and ground from ball.json") {
    core::Config ball, scene;
    REQUIRE(ball.load(CRICKET_CONFIG_DIR "/ball.json"));
    REQUIRE(scene.load(CRICKET_CONFIG_DIR "/scene.json"));
    SphereSolver s;
    physics::configureBallSolver(s, ball, scene, physics::WorldSettings{});
    CHECK(s.body().mass == doctest::Approx(0.160f));
    CHECK(s.body().radius == doctest::Approx(0.036f).epsilon(0.01));
    CHECK(s.body().dragCoefficient == doctest::Approx(0.45f));
    REQUIRE(s.colliders().boxCount() >= 1);
    CHECK(s.colliders().box(0).pose.position.y + s.colliders().box(0).halfExtents.y == doctest::Approx(0.0f));
}
