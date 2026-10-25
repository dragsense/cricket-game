#include "game/ball_feeder.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

TEST_CASE("feeder: the ball travels straight at the fielder, passes beside and stops") {
    game::FeederSettings s;
    s.distance = 10.0f;
    s.speedMin = s.speedMax = 5.0f;
    s.passDistance = 4.0f;
    s.sideSpread = 1.0f;
    game::BallFeeder feeder;
    feeder.setSettings(s);
    const math::Transform root;  // at the origin, facing +Z
    feeder.throwAt(root, 0.036f);
    REQUIRE(feeder.active());
    const math::Vec3 start = feeder.position();
    CHECK(start.z == doctest::Approx(10.0f));
    CHECK(std::fabs(start.x) <= 1.0f);

    float minDistance = 1e9f;
    int ticks = 0;
    while (feeder.active() && ticks < 10 * 120) {
        feeder.tick(1.0f / 120.0f);
        const math::Vec3 p = feeder.position();
        minDistance = std::min(minDistance, std::sqrt(p.x * p.x + p.z * p.z));
        CHECK(p.y == doctest::Approx(start.y));  // straight line, constant height
        ++ticks;
    }
    CHECK_FALSE(feeder.active());
    CHECK(feeder.position().z == doctest::Approx(-4.0f).epsilon(1e-3));  // stopped past the fielder
    CHECK(minDistance == doctest::Approx(std::fabs(start.x)).epsilon(1e-3));  // passed at its side offset
    CHECK(ticks == doctest::Approx(14.0f / 5.0f * 120.0f).epsilon(0.01));
}
