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

// Live-tunable feel (Debug panel sliders; defaults from config/engine.json).
struct CameraSpeeds {
    float orbitRotateDeg = 0.5f;     // orbit: degrees per mouse pixel
    float orbitZoomStep = 0.15f;     // orbit: fraction of the view distance per wheel notch
    float flyLookDeg = 0.3f;         // free-fly: degrees per mouse pixel
    float flyMoveSpeed = 20.0f;      // free-fly: m/s
    float flyFastMultiplier = 4.0f;  // free-fly: Shift speed-up

    bool operator==(const CameraSpeeds&) const = default;
};

CameraSpeeds readCameraSpeeds(const core::Config& config);

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

    const CameraSpeeds& speeds() const { return speeds_; }
    void setSpeeds(const CameraSpeeds& speeds);  // applies immediately

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    CameraMode mode_ = CameraMode::Orbit;
    CameraSpeeds speeds_;
};

} // namespace camera
