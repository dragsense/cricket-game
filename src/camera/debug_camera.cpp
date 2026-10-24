#include "camera/debug_camera.h"

#include "core/config.h"

#include <camutils/Manipulator.h>

#include <algorithm>

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
    // Each manipulator's settings. camutils settings are fixed after build, so a
    // speed change rebuilds the manipulator and restores its pose via a bookmark.
    Manip::Config orbitCfg{}, flyCfg{}, flyFastCfg{};

    static std::unique_ptr<Manip> rebuild(const std::unique_ptr<Manip>& old, const Manip::Config& cfg, Mode mode) {
        Manip::Builder b;
        b.details = cfg;
        std::unique_ptr<Manip> m(b.build(mode));
        if (old) m->jumpToBookmark(old->getCurrentBookmark());
        return m;
    }

    int width = 1, height = 1;
    // Virtual cursor fed to grabUpdate. Built from relative deltas so it keeps
    // working while SDL's relative mouse mode pins the real cursor.
    float cursorX = 0, cursorY = 0;
    bool grabbing = false;
    bool keyState[static_cast<int>(Manip::Key::COUNT)] = {};

    Manip* active(CameraMode m) { return m == CameraMode::Orbit ? orbit.get() : activeFly; }
};

CameraSpeeds readCameraSpeeds(const core::Config& c) {
    CameraSpeeds s;
    s.orbitRotateDeg = c.getFloat("camera.orbit.rotateDegPerPixel", s.orbitRotateDeg);
    s.orbitZoomStep = c.getFloat("camera.orbit.zoomStep", s.orbitZoomStep);
    s.flyLookDeg = c.getFloat("camera.freeFly.lookDegPerPixel", s.flyLookDeg);
    s.flyMoveSpeed = c.getFloat("camera.freeFly.maxSpeed", s.flyMoveSpeed);
    s.flyFastMultiplier = c.getFloat("camera.freeFly.fastMultiplier", s.flyFastMultiplier);
    return s;
}

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

    speeds_ = readCameraSpeeds(c);

    Manip::Builder ob;
    ob.viewport(width, height)
        .targetPosition(target.x, target.y, target.z)
        .orbitHomePosition(home.x, home.y, home.z)
        .zoomSpeed(1.0f)  // 1 m per scroll unit; update() scales the scroll by view distance
        .fovDegrees(fov)
        .farPlane(farPlane);
    s.orbitCfg = ob.details;

    math::Vec3 start = c.getVec3("camera.freeFly.start", {0, 1.7f, 16});
    float pitch = math::radians(c.getFloat("camera.freeFly.pitch", -5.0f));
    float yaw = math::radians(c.getFloat("camera.freeFly.yaw", 0.0f));
    s.speedSteps = c.getInt("camera.freeFly.speedSteps", 80);

    Manip::Builder fb;
    fb.viewport(width, height)
        .flightStartPosition(start.x, start.y, start.z)
        .flightStartOrientation(pitch, yaw)
        .flightSpeedSteps(s.speedSteps)
        .flightMoveDamping(c.getFloat("camera.freeFly.damping", 15.0f))
        .fovDegrees(fov)
        .farPlane(farPlane);
    s.flyCfg = fb.details;
    s.flyFastCfg = fb.details;

    s.orbit.reset();
    s.fly.reset();
    s.flyFast.reset();
    setSpeeds(speeds_);  // builds all three manipulators
}

void DebugCamera::setSpeeds(const CameraSpeeds& sp) {
    Impl& s = *impl_;
    speeds_ = sp;
    float orbitRad = math::radians(sp.orbitRotateDeg);
    float lookRad = math::radians(sp.flyLookDeg);

    s.orbitCfg.orbitSpeed = {orbitRad, orbitRad};
    s.flyCfg.flightPanSpeed = {lookRad, lookRad};
    s.flyFastCfg.flightPanSpeed = {lookRad, lookRad};
    s.flyCfg.flightMaxSpeed = std::max(1.01f, sp.flyMoveSpeed);
    s.flyFastCfg.flightMaxSpeed = std::max(1.01f, sp.flyMoveSpeed * sp.flyFastMultiplier);

    // Release any held keys or drag on the manipulators being replaced; update()
    // re-sends key presses on the next frame.
    if (Manip* old = s.active(mode_); old && s.grabbing) old->grabEnd();
    s.grabbing = false;
    std::fill(std::begin(s.keyState), std::end(s.keyState), false);

    bool fastActive = s.activeFly && s.activeFly == s.flyFast.get();
    s.orbit = Impl::rebuild(s.orbit, s.orbitCfg, Mode::ORBIT);
    s.fly = Impl::rebuild(s.fly, s.flyCfg, Mode::FREE_FLIGHT);
    s.flyFast = Impl::rebuild(s.flyFast, s.flyFastCfg, Mode::FREE_FLIGHT);
    s.activeFly = fastActive ? s.flyFast.get() : s.fly.get();
    // Free-flight speed is pow(maxSpeed, wheel position in [-1, 1]); put the
    // wheel at the top so maxSpeed is the actual speed.
    s.fly->scroll(0, 0, static_cast<float>(s.speedSteps));
    s.flyFast->scroll(0, 0, static_cast<float>(s.speedSteps));
}

void DebugCamera::setViewport(int width, int height) {
    Impl& s = *impl_;
    s.width = width;
    s.height = height;
    for (Manip::Config* cfg : {&s.orbitCfg, &s.flyCfg, &s.flyFastCfg}) {
        cfg->viewport[0] = width;
        cfg->viewport[1] = height;
    }
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
        int sx = static_cast<int>(in.mouseX), sy = static_cast<int>(s.height - in.mouseY);
        if (mode_ == CameraMode::Orbit) {
            // camutils zooms a fixed distance per notch, which crawls when far
            // away. Scale the step by the distance to the ground point being
            // looked at, so each notch is a constant fraction of the view.
            filament::math::float3 eye, target, up;
            m->getLookAt(&eye, &target, &up);
            math::Vec3 gaze = math::safeNormalize(math::Vec3(target.x - eye.x, target.y - eye.y, target.z - eye.z),
                                                  math::Vec3(0, -1, 0));
            float viewDist = gaze.y < -0.05f ? eye.y / -gaze.y : math::length(math::Vec3(eye.x, eye.y, eye.z));
            float metres = speeds_.orbitZoomStep * math::clamp(viewDist, 1.0f, 1000.0f);
            m->scroll(sx, sy, -in.wheel * metres);  // zoomSpeed is 1 m/unit; wheel up = zoom in
        } else {
            m->scroll(sx, sy, in.wheel);   // free-fly: wheel adjusts the base speed
        }
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
