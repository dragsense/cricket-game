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

bool SimWorld::initFielder(const rig::SkeletonDef& def) { return fielder_.init(def, def.height); }

void SimWorld::tick(double dt) {
    previous_ = current_;
    time_ += dt;
    fielder_.tick(dt);
    if (!probeEnabled_) return;

    float a = static_cast<float>(time_) * pathSpeed_;
    math::Vec3 pos(pathRadius_ * std::cos(a), pathHeight_ + 0.5f * std::sin(3.0f * a), pathRadius_ * std::sin(a));
    // Face along the direction of travel (tangent of the circle).
    math::Vec3 tangent(-std::sin(a), 0.0f, std::cos(a));
    current_.transform.position = pos;
    current_.transform.rotation = math::lookRotation(tangent);
}

} // namespace match
