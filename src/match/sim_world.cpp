#include "match/sim_world.h"

#include "core/config.h"
#include "physics/ball/ball_setup.h"

namespace match {

void SimWorld::configure(const core::Config& config) {
    pathRadius_ = config.getFloat("sim.probe.radius", pathRadius_);
    pathSpeed_ = config.getFloat("sim.probe.angularSpeed", pathSpeed_);
    pathHeight_ = config.getFloat("sim.probe.height", pathHeight_);
    tick(0.0);
    previous_ = current_;
}

bool SimWorld::initFielder(const rig::SkeletonDef& def, const core::Config& joints, const core::Config& locomotion) {
    return fielder_.init(def, def.height, joints, locomotion);
}

bool SimWorld::initBowler(const rig::SkeletonDef& def, const core::Config& joints, const core::Config& locomotion,
                          math::Vec3 position, float yaw) {
    rig::SkeletonDef placed = def;
    placed.spawnPosition = position;
    placed.spawnYaw = yaw;
    return bowler_.init(placed, placed.height, joints, locomotion);
}

bool SimWorld::initPhysics(const core::Config& physicsWorld, const core::Config& scene) {
    wickets_.destroy(physics_);
    if (!physics_.init(physics::readWorldSettings(physicsWorld), physics::readPhysicsSettings(physicsWorld))) return false;
    return wickets_.build(physics_, scene, physicsWorld);
}

bool SimWorld::rebuildWickets(const core::Config& physicsWorld, const core::Config& scene) {
    return physics_.ready() && wickets_.build(physics_, scene, physicsWorld);
}

void SimWorld::initBall(const core::Config& ball, const core::Config& scene) {
    physics::configureBallSolver(ballSolver_, ball, scene, physics_.world());
    ballSolver_.clearForces();
    ballSolver_.addForce(&gravity_);
    ballSolver_.addForce(&drag_);
}

void SimWorld::launchBall(math::Vec3 position, math::Vec3 velocity) {
    ball_.position = position;
    ball_.velocity = velocity;
    ball_.angularVelocity = math::Vec3(0.0f);
    ballPrevious_ = ball_;
    ballInFlight_ = true;
}

void SimWorld::tick(double dt) {
    previous_ = current_;
    time_ += dt;
    fielder_.tick(dt);
    if (bowler_.rig().numJoints() > 0) bowler_.tick(dt);
    ballPrevious_ = ball_;
    if (ballInFlight_) ballSolver_.step(ball_, static_cast<float>(dt));
    physics_.step(static_cast<float>(dt));
    if (!probeEnabled_) return;

    float a = static_cast<float>(time_) * pathSpeed_;
    math::Vec3 pos(pathRadius_ * std::cos(a), pathHeight_ + 0.5f * std::sin(3.0f * a), pathRadius_ * std::sin(a));
    // Face along the direction of travel (tangent of the circle).
    math::Vec3 tangent(-std::sin(a), 0.0f, std::cos(a));
    current_.transform.position = pos;
    current_.transform.rotation = math::lookRotation(tangent);
}

} // namespace match
