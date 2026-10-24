#include "camera/debug_camera.h"

#include "core/config.h"

#include <camutils/Manipulator.h>

namespace camera {

using Manip = filament::camutils::Manipulator<float>;
using filament::camutils::Mode;

struct DebugCamera::Impl {
    std::unique_ptr<Manip> orbit;
    // camutils has no "fast" key, so Shift switches to a second free-flight
    // manipulator with a higher top speed; bookmarks carry the pose across.
    std::unique_ptr<Manip> fly;
    std::unique_ptr<Manip> flyFast;
    Manip* activeFly = nullptr;
    int speedSteps = 80;

    int width = 1, height = 1;
    // Virtual cursor fed to grabUpdate. Built from relative deltas so it keeps
    // working while SDL's relative mouse mode pins the real cursor.
    float cursorX = 0, cursorY = 0;
    bool grabbing = false;
    bool keyState[static_cast<int>(Manip::Key::COUNT)] = {};

    Manip* active(CameraMode m) { return m == CameraMode::Orbit ? orbit.get() : activeFly; }
};

DebugCamera::DebugCamera() : impl_(std::make_unique<Impl>()) {}
DebugCamera::~DebugCamera() = default;

void DebugCamera::configure(const core::Config& c, int width, int height) {
    Impl& s = *impl_;
    s.width = width;
    s.height = height;

    math::Vec3 target = c.getVec3("camera.orbit.target", {0, 0.5f, 0});
    math::Vec3 home = c.getVec3("camera.orbit.home", {22, 12, 26});
    float fov = c.getFloat("camera.fovY", 50.0f);
    float farPlane = c.getFloat("camera.far", 2000.0f);

    s.orbit.reset(Manip::Builder()
                      .viewport(width, height)
                      .targetPosition(target.x, target.y, target.z)
                      .orbitHomePosition(home.x, home.y, home.z)
                      .zoomSpeed(c.getFloat("camera.orbit.zoomSpeed", 0.05f))
                      .fovDegrees(fov)
                      .farPlane(farPlane)
                      .build(Mode::ORBIT));

    math::Vec3 start = c.getVec3("camera.freeFly.start", {0, 1.7f, 16});
    float pitch = math::radians(c.getFloat("camera.freeFly.pitch", -5.0f));
    float yaw = math::radians(c.getFloat("camera.freeFly.yaw", 0.0f));
    float maxSpeed = c.getFloat("camera.freeFly.maxSpeed", 12.0f);
    float fast = c.getFloat("camera.freeFly.fastMultiplier", 5.0f);
    s.speedSteps = c.getInt("camera.freeFly.speedSteps", 80);

    auto buildFly = [&](float speed) {
        Manip* m = Manip::Builder()
                       .viewport(width, height)
                       .flightStartPosition(start.x, start.y, start.z)
                       .flightStartOrientation(pitch, yaw)
                       .flightMaxMoveSpeed(speed)
                       .flightSpeedSteps(s.speedSteps)
                       .flightMoveDamping(c.getFloat("camera.freeFly.damping", 15.0f))
                       .fovDegrees(fov)
                       .farPlane(farPlane)
                       .build(Mode::FREE_FLIGHT);
        // Speed is pow(maxSpeed, wheel position in [-1, 1]); start at the top
        // so the configured max speed is the default.
        m->scroll(0, 0, static_cast<float>(s.speedSteps));
        return m;
    };
    s.fly.reset(buildFly(maxSpeed));
    s.flyFast.reset(buildFly(maxSpeed * fast));
    s.activeFly = s.fly.get();
}

void DebugCamera::setViewport(int width, int height) {
    Impl& s = *impl_;
    s.width = width;
    s.height = height;
    for (Manip* m : {s.orbit.get(), s.fly.get(), s.flyFast.get()})
        if (m) m->setViewport(width, height);
}

void DebugCamera::setMode(CameraMode mode) {
    if (mode == mode_) return;
    Impl& s = *impl_;
    Manip* old = s.active(mode_);
    if (s.grabbing) old->grabEnd();
    s.grabbing = false;
    for (int k = 0; k < static_cast<int>(Manip::Key::COUNT); ++k)
        if (s.keyState[k]) old->keyUp(static_cast<Manip::Key>(k));
    std::fill(std::begin(s.keyState), std::end(s.keyState), false);
    mode_ = mode;
}

bool DebugCamera::update(const core::InputState& in, const core::ActionMap& actions, float dt) {
    Impl& s = *impl_;
    using core::Action;

    // Shift in free-fly: hand the pose over to the other speed manipulator.
    if (mode_ == CameraMode::FreeFly) {
        Manip* want = actions.down(in, Action::MoveFast) ? s.flyFast.get() : s.fly.get();
        if (want != s.activeFly) {
            for (int k = 0; k < static_cast<int>(Manip::Key::COUNT); ++k)
                if (s.keyState[k]) s.activeFly->keyUp(static_cast<Manip::Key>(k));
            std::fill(std::begin(s.keyState), std::end(s.keyState), false);
            if (s.grabbing) s.activeFly->grabEnd();
            want->jumpToBookmark(s.activeFly->getCurrentBookmark());
            s.activeFly = want;
            if (s.grabbing) want->grabBegin(static_cast<int>(s.cursorX), static_cast<int>(s.cursorY), false);
        }
    }
    Manip* m = s.active(mode_);

    // Mouse: right button rotates/looks, middle button pans (orbit only).
    bool rotate = in.down(core::MouseButton::Right);
    bool pan = mode_ == CameraMode::Orbit && in.down(core::MouseButton::Middle);
    bool wantGrab = (rotate || pan) && (s.grabbing || !in.uiWantsMouse);
    if (wantGrab && !s.grabbing) {
        // camutils expects a bottom-left origin, like Filament viewports.
        s.cursorX = in.mouseX;
        s.cursorY = static_cast<float>(s.height) - in.mouseY;
        m->grabBegin(static_cast<int>(s.cursorX), static_cast<int>(s.cursorY), pan);
        s.grabbing = true;
    } else if (!wantGrab && s.grabbing) {
        m->grabEnd();
        s.grabbing = false;
    }
    if (s.grabbing && (in.mouseDx != 0.0f || in.mouseDy != 0.0f)) {
        s.cursorX += in.mouseDx;
        s.cursorY -= in.mouseDy;
        m->grabUpdate(static_cast<int>(s.cursorX), static_cast<int>(s.cursorY));
    }

    if (!in.uiWantsMouse && in.wheel != 0.0f) {
        // Orbit: zoom. Free-fly: adjust the base speed. Negative wheel = zoom out.
        float delta = mode_ == CameraMode::Orbit ? -in.wheel : in.wheel;
        m->scroll(static_cast<int>(in.mouseX), static_cast<int>(s.height - in.mouseY), delta);
    }

    // Keyboard movement (free-flight; orbit ignores these keys).
    const std::pair<Action, Manip::Key> keyMap[] = {
        {Action::MoveForward, Manip::Key::FORWARD}, {Action::MoveBack, Manip::Key::BACKWARD},
        {Action::MoveLeft, Manip::Key::LEFT},       {Action::MoveRight, Manip::Key::RIGHT},
        {Action::MoveUp, Manip::Key::UP},           {Action::MoveDown, Manip::Key::DOWN},
    };
    for (const auto& [action, key] : keyMap) {
        bool down = actions.down(in, action);
        bool& was = s.keyState[static_cast<int>(key)];
        if (down != was) {
            if (down) m->keyDown(key);
            else m->keyUp(key);
            was = down;
        }
    }

    m->update(dt);
    return s.grabbing && mode_ == CameraMode::FreeFly;
}

CameraPose DebugCamera::pose() const {
    filament::math::float3 eye, target, up;
    const Impl& s = *impl_;
    const Manip* m = mode_ == CameraMode::Orbit ? s.orbit.get() : s.activeFly;
    m->getLookAt(&eye, &target, &up);
    return {{eye.x, eye.y, eye.z}, {target.x, target.y, target.z}, {up.x, up.y, up.z}};
}

} // namespace camera
