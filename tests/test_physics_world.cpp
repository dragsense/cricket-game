#include "physics/core/physics_world.h"

#include "core/config.h"

#include <doctest/doctest.h>

namespace {

constexpr float kTick = 1.0f / 120.0f;

} // namespace

TEST_CASE("physics world: settings come from config/physics_world.json") {
    core::Config c;
    REQUIRE(c.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    const physics::WorldSettings w = physics::readWorldSettings(c);
    CHECK(w.gravity.y == doctest::Approx(-9.81f));
    CHECK(w.airDensity == doctest::Approx(1.225f));
}

TEST_CASE("physics world: a body falls freely under the shared gravity") {
    core::Config c;
    REQUIRE(c.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld w;
    REQUIRE(w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c)));

    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.036f;
    d.mass = 0.16f;
    d.pose.position = math::Vec3(0, 100, 0);
    const physics::BodyId id = w.createBody(d);
    REQUIRE(id != physics::kInvalidBody);

    for (int i = 0; i < 120; ++i) w.step(kTick);  // 1 s
    // Jolt integrates semi-implicit Euler: y = y0 - g t (t + dt) / 2.
    CHECK(w.transform(id).position.y == doctest::Approx(100.0f - 0.5f * 9.81f * (1.0f + kTick)).epsilon(1e-4));
    CHECK(w.linearVelocity(id).y == doctest::Approx(-9.81f).epsilon(1e-3));
}

TEST_CASE("physics world: interpolation blends the last two ticks; reset teleports") {
    core::Config c;
    REQUIRE(c.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld w;
    REQUIRE(w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c)));
    physics::BodyDesc d;
    d.pose.position = math::Vec3(0, 10, 0);
    const physics::BodyId id = w.createBody(d);
    for (int i = 0; i < 10; ++i) w.step(kTick);

    const float before = w.interpolated(id, 0.0f).position.y;
    const float after = w.interpolated(id, 1.0f).position.y;
    CHECK(before > after);
    CHECK(w.interpolated(id, 0.5f).position.y == doctest::Approx(0.5f * (before + after)));

    math::Transform home;
    home.position = math::Vec3(1, 2, 3);
    w.resetBody(id, home);
    CHECK(w.interpolated(id, 0.0f).position == home.position);
    CHECK(w.linearVelocity(id) == math::Vec3(0.0f));
}

TEST_CASE("physics world: a mass on a push-only spring settles where gravity and the spring balance") {
    core::Config c;
    REQUIRE(c.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld w;
    REQUIRE(w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c)));

    // A kinematic anchor on the ground and a mass 1 m above it that touches
    // nothing; the spring only pushes (slack beyond 1 m, like a leg).
    physics::BodyDesc anchor;
    anchor.shape = physics::ShapeType::Sphere;
    anchor.radius = 0.01f;
    anchor.kinematic = true;
    anchor.collides = false;
    const physics::BodyId a = w.createBody(anchor);
    physics::BodyDesc body = anchor;
    body.kinematic = false;
    body.lockRotation = true;
    body.mass = 70.0f;
    body.pose.position = math::Vec3(0, 1, 0);
    const physics::BodyId b = w.createBody(body);
    REQUIRE(a != physics::kInvalidBody);
    REQUIRE(b != physics::kInvalidBody);
    const float hz = 2.0f;
    const physics::SpringId s = w.createSpring(a, math::Vec3(0), b, math::Vec3(0), 1.0f, 100.0f, hz, 1.0f);
    REQUIRE(s != physics::kInvalidSpring);

    for (int i = 0; i < 5 * 120; ++i) w.step(kTick);  // 5 s, critically damped
    // F = k x with k = m (2 pi f)^2: the spring shortens by g / (2 pi f)^2.
    const float omega = 2.0f * 3.14159265f * hz;
    CHECK(w.transform(b).position.y == doctest::Approx(1.0f - 9.81f / (omega * omega)).epsilon(0.01));
    CHECK(w.transform(b).position.x == doctest::Approx(0.0f));

    // Disabled, the spring holds nothing: the mass falls freely.
    w.setSpringEnabled(s, false);
    const float y0 = w.transform(b).position.y;
    for (int i = 0; i < 12; ++i) w.step(kTick);
    CHECK(w.transform(b).position.y < y0 - 0.04f);
}
