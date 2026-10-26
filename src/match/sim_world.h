#pragma once
// Simulation state, advanced only by fixed ticks. Knows nothing about
// rendering. For Milestone 0 it holds a single "probe" that moves along a
// path so fixed-step + interpolation can be checked visually; real match
// state replaces it in later milestones.
//
// Every tick copies `current` into `previous`; the renderer blends the two
// with the loop's alpha.

#include "agents/fielder.h"
#include "match/wickets.h"
#include "math/math.h"
#include "physics/sim/sphere_solver.h"
#include "physics/core/physics_world.h"

namespace core {
class Config;
}

namespace match {

struct ProbeState {
    math::Transform transform;
};

class SimWorld {
public:
    void configure(const core::Config& config);
    void tick(double dt);

    // Creates the fielder from its skeleton description. Load-time only.
    bool initFielder(const rig::SkeletonDef& def, const core::Config& joints, const core::Config& locomotion);
    // Creates the Jolt world from config/physics_world.json and the wickets
    // (sizes from config/scene.json). Load time; calling again rebuilds them.
    bool initPhysics(const core::Config& physicsWorld, const core::Config& scene);
    // Rebuilds the wickets only (scene.json changed).
    bool rebuildWickets(const core::Config& physicsWorld, const core::Config& scene);
    const Wickets& wickets() const { return wickets_; }
    physics::PhysicsWorld& physicsWorld() { return physics_; }
    const physics::PhysicsWorld& physicsWorld() const { return physics_; }

    // Sets up the ball solver (config/ball.json, size and mass from scene.json)
    // with gravity and drag, using the physics world's shared settings. Call
    // after initPhysics, and again when any of those files change.
    void initBall(const core::Config& ball, const core::Config& scene);
    // Starts a physics flight from this state; stopBall() ends it.
    void launchBall(math::Vec3 position, math::Vec3 velocity);
    void stopBall() { ballInFlight_ = false; }
    bool ballInFlight() const { return ballInFlight_; }
    const physics::BodyState& ball() const { return ball_; }
    math::Vec3 ballPosition(float alpha) const { return math::lerp(ballPrevious_.position, ball_.position, alpha); }
    math::Transform ballTransform(float alpha) const {
        return {ballPosition(alpha), math::slerp(ballPrevious_.orientation, ball_.orientation, alpha), math::Vec3(1.0f)};
    }

    agents::Fielder& fielder() { return fielder_; }
    const agents::Fielder& fielder() const { return fielder_; }
    // The bowler (M5): the same body, standing at `position` facing `yaw`
    // radians, holding a ball. Load-time only.
    bool initBowler(const rig::SkeletonDef& def, const core::Config& joints, const core::Config& locomotion,
                    math::Vec3 position, float yaw);
    agents::Fielder& bowler() { return bowler_; }
    const agents::Fielder& bowler() const { return bowler_; }

    // Interpolated probe transform for rendering.
    math::Transform probeTransform(float alpha) const {
        return math::lerp(previous_.transform, current_.transform, alpha);
    }

    double time() const { return time_; }
    void setProbeEnabled(bool enabled) { probeEnabled_ = enabled; }

private:
    ProbeState previous_, current_;
    agents::Fielder fielder_;
    agents::Fielder bowler_;
    physics::PhysicsWorld physics_;
    Wickets wickets_;
    physics::GravityForce gravity_;
    physics::DragForce drag_;
    physics::SphereSolver ballSolver_;
    physics::BodyState ball_, ballPrevious_;
    bool ballInFlight_ = false;
    double time_ = 0;
    bool probeEnabled_ = true;
    float pathRadius_ = 12.0f;  // metres
    float pathSpeed_ = 0.5f;    // radians per second
    float pathHeight_ = 1.0f;
};

} // namespace match
