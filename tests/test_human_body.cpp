// The human body's mass in the shared physics world: gravity from
// config/physics_world.json, stopped by the world's one ground. Nothing holds
// it up yet (muscles come with the ragdoll).

#include "agents/human_body.h"
#include "core/config.h"

#include <doctest/doctest.h>

#include <cmath>

namespace {

constexpr float kTick = 1.0f / 120.0f;

bool makeWorld(physics::PhysicsWorld& w) {
    core::Config c;
    return c.load(CRICKET_CONFIG_DIR "/physics_world.json") &&
           w.init(physics::readWorldSettings(c), physics::readPhysicsSettings(c));
}

} // namespace

TEST_CASE("human body: the mass falls under the world's gravity") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    agents::HumanBody body;
    REQUIRE(body.init(w, 73.0f, math::Vec3(0.0f, 1.0f, 0.0f)));
    for (int i = 0; i < 12; ++i) w.step(kTick);
    CHECK(body.verticalSpeed(w) == doctest::Approx(-9.81f * 0.1f).scale(0.0).epsilon(0.01));
}

TEST_CASE("human body: the world's ground stops the falling mass") {
    physics::PhysicsWorld w;
    REQUIRE(makeWorld(w));
    agents::HumanBody body;
    REQUIRE(body.init(w, 73.0f, math::Vec3(0.0f, 1.0f, 0.0f)));
    for (int i = 0; i < 3 * 120; ++i) w.step(kTick);
    // Rests on the ground (its 5 cm sphere on top of it), not through it.
    CHECK(body.height(w) > w.groundHeightAt(math::Vec3(0.0f)));
    CHECK(body.height(w) < w.groundHeightAt(math::Vec3(0.0f)) + 0.1f);
    CHECK(std::fabs(body.verticalSpeed(w)) < 0.05f);
}
