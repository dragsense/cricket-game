#pragma once
// The two wickets (Law 8) as Jolt bodies: three stumps and two bails at each
// end. The layout comes from config/scene.json (sizes) and the bodies from
// config/physics_world.json "wicket" (mass, friction). The same layout places
// the visuals, so what is drawn matches what collides.

#include "math/math.h"
#include "physics/core/physics_world.h"

#include <array>

namespace core {
class Config;
}

namespace match {

constexpr int kStumpsPerWicket = 3;
constexpr int kBailsPerWicket = 2;
constexpr int kPiecesPerWicket = kStumpsPerWicket + kBailsPerWicket;
constexpr int kWicketPieces = 2 * kPiecesPerWicket;  // index: end * 5 + (stumps 0-2, bails 3-4)

struct WicketLayout {
    float stumpHeight = 0.7112f;
    float stumpRadius = 0.0178f;
    float bailLength = 0.1095f;
    float bailRadius = 0.0065f;
    // Rest pose of each piece's centre of mass; a cylinder's axis is its local +Y
    // (bails are turned to lie along X).
    std::array<math::Transform, kWicketPieces> rest;

    bool isBail(int piece) const { return piece % kPiecesPerWicket >= kStumpsPerWicket; }
    float halfLength(int piece) const { return 0.5f * (isBail(piece) ? bailLength : stumpHeight); }
    float radius(int piece) const { return isBail(piece) ? bailRadius : stumpRadius; }
};

// Stumps stand on the ground (y = 0) at z = ±pitchLength/2; bails rest on the
// stump tops, meeting over the middle stump.
WicketLayout makeWicketLayout(const core::Config& scene, const core::Config& physics);

class Wickets {
public:
    // Creates the ten bodies (load time). Rebuilding replaces the old ones.
    bool build(physics::PhysicsWorld& world, const core::Config& scene, const core::Config& physics);
    void destroy(physics::PhysicsWorld& world);
    // Puts every piece back in its rest pose.
    void reset(physics::PhysicsWorld& world) const;

    bool built() const { return built_; }
    const WicketLayout& layout() const { return layout_; }
    physics::BodyId body(int piece) const { return bodies_[static_cast<size_t>(piece)]; }

private:
    WicketLayout layout_;
    std::array<physics::BodyId, kWicketPieces> bodies_{};
    bool built_ = false;
};

} // namespace match
