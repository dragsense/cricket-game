#pragma once
// Fixed-timestep accumulator ("Fix Your Timestep"). The sim always advances
// in steps of exactly `stepSeconds`; rendering interpolates between the last
// two sim states with alpha = leftover / step.

namespace core {

class FixedTimestep {
public:
    FixedTimestep(double stepSeconds, int maxStepsPerFrame)
        : step_(stepSeconds), maxSteps_(maxStepsPerFrame) {}

    // Adds real elapsed time; returns how many sim steps to run now. If the
    // sim cannot keep up, excess time is dropped (no spiral of death).
    int advance(double frameSeconds) {
        accumulator_ += frameSeconds;
        int steps = static_cast<int>(accumulator_ / step_);
        if (steps > maxSteps_) {
            steps = maxSteps_;
            accumulator_ = 0.0;
        } else {
            accumulator_ -= steps * step_;
        }
        return steps;
    }

    // Interpolation factor in [0, 1) for rendering.
    float alpha() const { return static_cast<float>(accumulator_ / step_); }
    double step() const { return step_; }

private:
    double step_;
    int maxSteps_;
    double accumulator_ = 0.0;
};

} // namespace core
