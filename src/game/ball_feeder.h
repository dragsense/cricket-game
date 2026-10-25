#pragma once
// Test driver for picking: throws the ball toward the fielder in a straight
// line at constant speed (no physics), rolling on the ground or at a fixed
// height, passing beside them. Speed, height and side offset are random
// within config/engine.json "feeder". Advanced by fixed sim ticks; keeps the
// previous position so rendering can interpolate. No allocations per tick.

#include "math/math.h"

#include <cstdint>
#include <random>

namespace core {
class Config;
}

namespace game {

struct FeederSettings {
    float distance = 12.0f;     // m in front of the fielder the ball starts
    float speedMin = 3.0f, speedMax = 6.0f;      // m/s
    float heightMin = 0.4f, heightMax = 1.8f;    // m, ball centre for air balls
    float groundShare = 0.5f;   // share of throws that roll on the ground
    float sideSpread = 1.0f;    // m; passes up to this far to either side
    float passDistance = 5.0f;  // m beyond the fielder where the ball stops
};

class BallFeeder {
public:
    void configure(const core::Config& engine);
    void setSettings(const FeederSettings& s) { settings_ = s; }

    // Starts a throw at the fielder standing at `root` (world). `ballRadius`
    // puts ground balls on the ground.
    void throwAt(const math::Transform& root, float ballRadius);
    void stop() { active_ = false; }
    bool active() const { return active_; }

    void tick(float dt);
    math::Vec3 position() const { return current_; }
    math::Vec3 velocity() const { return active_ ? velocity_ : math::Vec3(0.0f); }
    math::Vec3 position(float alpha) const { return math::lerp(previous_, current_, alpha); }

private:
    FeederSettings settings_;
    std::mt19937 rng_{20261026u};
    math::Vec3 previous_{0.0f}, current_{0.0f};
    math::Vec3 velocity_{0.0f};
    float travelled_ = 0.0f, length_ = 0.0f;
    bool active_ = false;
};

} // namespace game
