#pragma once
// Debug cameras built on Filament's camutils::Manipulator (orbit and
// free-flight modes). We only translate our input into manipulator calls and
// expose the resulting eye pose in our own math types, so callers never see
// camutils or Filament types.
//
// Orbit:    right-drag rotates, middle-drag pans, wheel zooms.
// Free-fly: hold right mouse to look, WASD/QE to move, Shift for speed,
//           wheel changes the base speed.

#include "core/input.h"
#include "math/math.h"

#include <memory>

namespace core {
class Config;
}

namespace camera {

enum class CameraMode { Orbit, FreeFly };

struct CameraPose {
    math::Vec3 eye{0.0f};
    math::Vec3 target{0, 0, -1};
    math::Vec3 up{0, 1, 0};
};

class DebugCamera {
public:
    DebugCamera();
    ~DebugCamera();

    void configure(const core::Config& config, int viewportWidth, int viewportHeight);
    void setViewport(int width, int height);

    void setMode(CameraMode mode);
    CameraMode mode() const { return mode_; }

    // Returns true while the mouse is captured (free-fly look), so the app can
    // hide the cursor.
    bool update(const core::InputState& input, const core::ActionMap& actions, float dt);

    CameraPose pose() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    CameraMode mode_ = CameraMode::Orbit;
};

} // namespace camera
