#include "core/app.h"

#include "core/log.h"

#include <chrono>

namespace core {

App::App(const LoopSettings& settings)
    : settings_(settings), timestep_(1.0 / settings.simHz, settings.maxStepsPerFrame) {}

int App::run() {
    if (!init()) {
        LOG_ERROR("initialisation failed");
        shutdown();
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();

    // Stats are averaged over a short window so the overlay is readable.
    double statWindow = 0;
    int statFrames = 0;
    int statTicks = 0;

    while (!quit_) {
        auto now = Clock::now();
        double frameSeconds = std::chrono::duration<double>(now - last).count();
        last = now;
        if (frameSeconds > settings_.maxFrameSeconds) frameSeconds = settings_.maxFrameSeconds;

        if (!pumpEvents()) break;

        int steps = timestep_.advance(frameSeconds);
        for (int i = 0; i < steps; ++i) {
            fixedUpdate(timestep_.step());
            stats_.simTime += timestep_.step();
            ++stats_.simTicks;
        }

        renderFrame(frameSeconds, timestep_.alpha());

        statWindow += frameSeconds;
        ++statFrames;
        statTicks += steps;
        if (statWindow >= 0.5) {
            stats_.fps = static_cast<float>(statFrames / statWindow);
            stats_.frameMs = static_cast<float>(1000.0 * statWindow / statFrames);
            stats_.simTicksPerSecond = static_cast<float>(statTicks / statWindow);
            statWindow = 0;
            statFrames = statTicks = 0;
        }
    }

    shutdown();
    return 0;
}

} // namespace core
