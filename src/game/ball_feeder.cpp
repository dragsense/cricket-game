#include "game/ball_feeder.h"

#include "core/config.h"

#include <algorithm>

namespace game {

void BallFeeder::configure(const core::Config& c) {
    FeederSettings& s = settings_;
    s.distance = c.getFloat("feeder.distance", s.distance);
    s.speedMin = c.getFloat("feeder.speedMin", s.speedMin);
    s.speedMax = c.getFloat("feeder.speedMax", s.speedMax);
    s.heightMin = c.getFloat("feeder.heightMin", s.heightMin);
    s.heightMax = c.getFloat("feeder.heightMax", s.heightMax);
    s.groundShare = c.getFloat("feeder.groundShare", s.groundShare);
    s.sideSpread = c.getFloat("feeder.sideSpread", s.sideSpread);
    s.passDistance = c.getFloat("feeder.passDistance", s.passDistance);
}

void BallFeeder::throwAt(const math::Transform& root, float ballRadius) {
    const FeederSettings& s = settings_;
    auto uniform = [&](float lo, float hi) { return std::uniform_real_distribution<float>(lo, std::max(lo, hi))(rng_); };
    const bool ground = uniform(0.0f, 1.0f) < s.groundShare;
    const float height = ground ? ballRadius : uniform(s.heightMin, s.heightMax);
    const float side = uniform(-s.sideSpread, s.sideSpread);

    // Start ahead of the fielder, travel straight back past them at `side`.
    const math::Vec3 forward = root.transformDirection(math::kForward);
    const math::Vec3 left = root.transformDirection(math::kRight);  // +X is left
    const math::Vec3 ground0(root.position.x, 0.0f, root.position.z);
    current_ = previous_ = ground0 + forward * s.distance + left * side + math::kUp * height;
    velocity_ = -forward * uniform(s.speedMin, s.speedMax);
    travelled_ = 0.0f;
    length_ = s.distance + s.passDistance;
    active_ = true;
}

void BallFeeder::tick(float dt) {
    previous_ = current_;
    if (!active_) return;
    const float step = std::min(math::length(velocity_) * dt, length_ - travelled_);
    current_ += math::safeNormalize(velocity_) * step;
    travelled_ += step;
    if (travelled_ >= length_) active_ = false;  // went past: it stays where it stopped
}

} // namespace game
