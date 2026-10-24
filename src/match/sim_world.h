#pragma once
// Simulation state, advanced only by fixed ticks. Knows nothing about
// rendering. For Milestone 0 it holds a single "probe" that moves along a
// path so fixed-step + interpolation can be checked visually; real match
// state replaces it in later milestones.
//
// Every tick copies `current` into `previous`; the renderer blends the two
// with the loop's alpha.

#include "agents/fielder.h"
#include "math/math.h"

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
    bool initFielder(const rig::SkeletonDef& def);
    agents::Fielder& fielder() { return fielder_; }
    const agents::Fielder& fielder() const { return fielder_; }

    // Interpolated probe transform for rendering.
    math::Transform probeTransform(float alpha) const {
        return math::lerp(previous_.transform, current_.transform, alpha);
    }

    double time() const { return time_; }
    void setProbeEnabled(bool enabled) { probeEnabled_ = enabled; }

private:
    ProbeState previous_, current_;
    agents::Fielder fielder_;
    double time_ = 0;
    bool probeEnabled_ = true;
    float pathRadius_ = 12.0f;  // metres
    float pathSpeed_ = 0.5f;    // radians per second
    float pathHeight_ = 1.0f;
};

} // namespace match
