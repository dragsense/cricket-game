// The body's mass in the shared physics world: gravity from
// config/physics_world.json, held up by push-only spring legs.

#include "agents/body_physics.h"
#include "core/config.h"

#include <doctest/doctest.h>

namespace {

constexpr float kTick = 1.0f / 120.0f;

bool makeWorld(physics::PhysicsWorld& w) {
    core::Config c;
    return c.load(CRICKET_CONFIG_DIR "/physics_world.json") &&
           w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c));
}

} // namespace

TEST_CASE("body physics: standing on two spring legs, the body settles by m g / 2 k") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    agents::BodyPhysics body;
    agents::BodyPhysicsSettings s;
    s.legDamping = 1.0f;  // settle for the check (the model itself has none)
    const math::Vec3 com(0.0f, 1.0f, 0.0f);
    const math::Vec3 ankles[2] = {com + math::Vec3(0.1f, -0.9f, 0.0f), com + math::Vec3(-0.1f, -0.9f, 0.0f)};
    // Rest: each leg's real length (centre of mass to ankle) as it stands.
    const float rest[2] = {math::distance(com, ankles[0]), math::distance(com, ankles[1])}, mass = 73.0f;
    REQUIRE(body.init(w, mass, com, rest, s));

    const bool down[2] = {true, true};
    for (int i = 0; i < 5 * 120; ++i) {
        body.setFeet(w, com, ankles, down, kTick);
        w.step(kTick);
    }
    // Two legs in parallel: 2 k x = m g. Each leg is tilted 6.3 deg (0.1 m out
    // over 0.9 m): it shortens by x cos(a) and pushes up by its force times
    // cos(a), so the body sinks m g / (2 k cos^2 a).
    const float c = 0.9f / rest[0];
    const float sink = mass * 9.81f / (2.0f * s.legStiffness * c * c);
    MESSAGE("sink ", com.y - body.height(w), " m, expected ", sink);
    CHECK(com.y - body.height(w) == doctest::Approx(sink).scale(0.0).epsilon(0.02));
}

TEST_CASE("body physics: with both feet off the ground the body falls freely") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    agents::BodyPhysics body;
    const float rest[2] = {0.9f, 0.9f};
    const math::Vec3 com(0.0f, 1.0f, 0.0f);
    REQUIRE(body.init(w, 73.0f, com, rest, agents::BodyPhysicsSettings{}));
    const math::Vec3 ankles[2] = {com - math::Vec3(0.0f, 0.9f, 0.0f), com - math::Vec3(0.0f, 0.9f, 0.0f)};
    const bool down[2] = {false, false};
    for (int i = 0; i < 12; ++i) {
        body.setFeet(w, com, ankles, down, kTick);
        w.step(kTick);
    }
    CHECK(body.verticalSpeed(w) == doctest::Approx(-9.81f * 0.1f).scale(0.0).epsilon(0.01));
}
