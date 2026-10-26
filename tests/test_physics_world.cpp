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
