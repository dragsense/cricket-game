#pragma once
// Debug orbit camera built on Filament's camutils::Manipulator. We only
// translate mouse input into manipulator calls and expose the resulting eye
// pose in our own math types, so callers never see camutils or Filament types.
// Right-drag rotates, middle-drag pans, wheel zooms. (Free-fly was removed:
// scripted camera paths come with the broadcast cameras, M8.)

#include "core/input.h"
#include "math/math.h"

#include <memory>

namespace core {
class Config;
}

namespace camera {

// Live-tunable feel (Debug panel sliders; defaults from config/engine.json).
struct CameraSpeeds {
    float orbitRotateDeg = 0.5f;     // orbit: degrees per mouse pixel
    float orbitZoomStep = 0.15f;     // orbit: fraction of the view distance per wheel notch

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

    void update(const core::InputState& input, float dt);

    CameraPose pose() const;

    const CameraSpeeds& speeds() const { return speeds_; }
    void setSpeeds(const CameraSpeeds& speeds);  // applies immediately
    // Ground point the orbit follows (e.g. the player's root); the orbit
    // keeps its configured offset above it. Rotate and zoom stay relative.
    void setFollowPoint(math::Vec3 point) { follow_ = point; }
    // Mouse-look: mouse motion rotates without a button (pair with the
    // window's relative mouse mode).
    void setMouseLook(bool on) { mouseLook_ = on; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    CameraSpeeds speeds_;
    math::Vec3 follow_{0.0f};
    bool mouseLook_ = false;
};

} // namespace camera
