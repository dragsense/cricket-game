#include "agents/fielder.h"

namespace agents {

bool Fielder::init(const rig::SkeletonDef& def, float height) {
    def_ = def;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);
    return setHeight(height);
}

bool Fielder::setHeight(float height) {
    if (!rig_.build(def_, height)) return false;
    current_ = rig_.restPose();
    previous_ = current_;
    return true;
}

void Fielder::tick(double /*dt*/) {
    previous_.locals = current_.locals;  // same size: no allocation
    // Neutral standing: the rest pose. Steps 1.2+ drive the pose from IK.
}

void Fielder::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
