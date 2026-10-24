#pragma once
// Main loop skeleton. Knows nothing about windows or rendering: subclasses
// plug those in through the virtual hooks. Each loop iteration:
//   pumpEvents -> N x fixedUpdate(step) -> renderFrame(alpha)

#include "core/fixed_timestep.h"

namespace core {

struct LoopStats {
    float fps = 0;              // rendered frames per second (averaged)
    float frameMs = 0;          // average frame time
    float simTicksPerSecond = 0;
    double simTime = 0;         // total simulated seconds
    long long simTicks = 0;
};

struct LoopSettings {
    double simHz = 120.0;
    int maxStepsPerFrame = 8;
    double maxFrameSeconds = 0.25;  // clamp after hitches (debugger, window drag)
};

class App {
public:
    explicit App(const LoopSettings& settings);
    virtual ~App() = default;

    int run();

protected:
    virtual bool init() = 0;
    virtual void shutdown() = 0;
    // Process OS events. Return false to quit.
    virtual bool pumpEvents() = 0;
    // Advance the simulation by exactly `dt` seconds.
    virtual void fixedUpdate(double dt) = 0;
    // Render; `alpha` interpolates between the previous and current sim state.
    virtual void renderFrame(double frameSeconds, float alpha) = 0;

    const LoopStats& stats() const { return stats_; }
    void requestQuit() { quit_ = true; }

private:
    LoopSettings settings_;
    FixedTimestep timestep_;
    LoopStats stats_;
    bool quit_ = false;
};

} // namespace core
