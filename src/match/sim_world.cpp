#include "match/sim_world.h"

#include "core/config.h"

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

bool SimWorld::initPhysics(const core::Config& physicsWorld) {
    return physics_.init(physics::readWorldSettings(physicsWorld), physics::readPhysicsSettings(physicsWorld));
}

void SimWorld::tick(double dt) {
    previous_ = current_;
    time_ += dt;
    fielder_.tick(dt);
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
