#include "camera/debug_camera.h"

#include "core/config.h"

#include <camutils/Manipulator.h>

#include <algorithm>
#include <cmath>

namespace camera {

using Manip = filament::camutils::Manipulator<float>;
using filament::camutils::Mode;

struct DebugCamera::Impl {
    std::unique_ptr<Manip> orbit;
    // camutils settings are fixed after build, so a speed change rebuilds the
    // manipulator and restores its pose via a bookmark.
    Manip::Config orbitCfg{};

    static std::unique_ptr<Manip> rebuild(const std::unique_ptr<Manip>& old, const Manip::Config& cfg, Mode mode) {
        Manip::Builder b;
        b.details = cfg;
        std::unique_ptr<Manip> m(b.build(mode));
        if (old) m->jumpToBookmark(old->getCurrentBookmark());
        return m;
    }

    // Orbit limits (config/engine.json camera.orbit): the eye never goes
    // under the ground, through the target or too far out.
    math::Vec3 target{0.0f};
    float minEyeHeight = 0.3f, minDistance = 1.5f, maxDistance = 300.0f;

    int width = 1, height = 1;
    // Virtual cursor fed to grabUpdate, built from relative mouse deltas.
    float cursorX = 0, cursorY = 0;
    bool grabbing = false;
};

CameraSpeeds readCameraSpeeds(const core::Config& c) {
    CameraSpeeds s;
    s.orbitRotateDeg = c.getFloat("camera.orbit.rotateDegPerPixel", s.orbitRotateDeg);
    s.orbitZoomStep = c.getFloat("camera.orbit.zoomStep", s.orbitZoomStep);
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
    s.target = target;
    s.minEyeHeight = c.getFloat("camera.orbit.minEyeHeight", s.minEyeHeight);
    s.minDistance = c.getFloat("camera.orbit.minDistance", s.minDistance);
    s.maxDistance = c.getFloat("camera.orbit.maxDistance", s.maxDistance);

    Manip::Builder ob;
    ob.viewport(width, height)
        .targetPosition(target.x, target.y, target.z)
        .orbitHomePosition(home.x, home.y, home.z)
        .zoomSpeed(1.0f)  // 1 m per scroll unit; update() scales the scroll by view distance
        .fovDegrees(fov)
        .farPlane(farPlane);
    s.orbitCfg = ob.details;

    s.orbit.reset();
    setSpeeds(speeds_);  // builds all three manipulators
}

void DebugCamera::setSpeeds(const CameraSpeeds& sp) {
    Impl& s = *impl_;
    speeds_ = sp;
    float orbitRad = math::radians(sp.orbitRotateDeg);
    s.orbitCfg.orbitSpeed = {orbitRad, orbitRad};

    // Release any drag on the manipulator being replaced.
    if (s.orbit && s.grabbing) s.orbit->grabEnd();
    s.grabbing = false;
    s.orbit = Impl::rebuild(s.orbit, s.orbitCfg, Mode::ORBIT);
}

void DebugCamera::setViewport(int width, int height) {
    Impl& s = *impl_;
    s.width = width;
    s.height = height;
    s.orbitCfg.viewport[0] = width;
    s.orbitCfg.viewport[1] = height;
    if (s.orbit) s.orbit->setViewport(width, height);
}

void DebugCamera::update(const core::InputState& in, float dt) {
    Impl& s = *impl_;
    Manip* m = s.orbit.get();

    // Mouse: mouse-look or the right button rotates. No pan: the orbit
    // follows the fielder and its pivot is pinned (constrain()).
    bool rotate = mouseLook_ || in.down(core::MouseButton::Right);
    bool pan = false;
    bool wantGrab = (rotate || pan) && (s.grabbing || mouseLook_ || !in.uiWantsMouse);
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
        // Zoom toward the screen centre (the target), not the cursor: a ray
        // under the cursor can carry the eye past the target and underground.
        int sx = s.width / 2, sy = s.height / 2;
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
    }

    m->update(dt);
    constrain();
}

void DebugCamera::constrain() {
    Impl& s = *impl_;
    filament::math::float3 e, t, up;
    s.orbit->getLookAt(&e, &t, &up);
    const math::Vec3 eye(e.x, e.y, e.z), target(t.x, t.y, t.z);

    // Keep the configured pivot, the distance in range and the eye above the
    // ground (orbit space: the ground is y = 0 under the follow point).
    math::Vec3 offset = eye - s.target;
    const float dist = math::clamp(math::length(offset), s.minDistance, s.maxDistance);
    offset = math::safeNormalize(offset, math::Vec3(0, 1, 1)) * dist;
    const float minY = s.minEyeHeight - s.target.y;
    if (offset.y < minY) {
        // Same heading and distance, raised to the minimum height.
        const float y = std::min(minY, dist * 0.99f);
        const float flat = std::sqrt(std::max(dist * dist - y * y, 0.0f));
        const math::Vec3 dir = math::safeNormalize(math::Vec3(offset.x, 0.0f, offset.z), math::Vec3(0, 0, 1));
        offset = dir * flat + math::Vec3(0.0f, y, 0.0f);
    }
    const math::Vec3 fixedEye = s.target + offset;
    if (math::distance(fixedEye, eye) < 1e-3f && math::distance(target, s.target) < 1e-3f) return;

    // camutils keeps its orbit state private: rebuild at the corrected pose
    // (only when a limit is hit), keeping any drag going.
    const bool grabbing = s.grabbing;
    if (grabbing) s.orbit->grabEnd();
    Manip::Builder b;
    b.details = s.orbitCfg;
    b.targetPosition(s.target.x, s.target.y, s.target.z).orbitHomePosition(fixedEye.x, fixedEye.y, fixedEye.z);
    s.orbit.reset(b.build(Mode::ORBIT));
    if (grabbing) s.orbit->grabBegin(static_cast<int>(s.cursorX), static_cast<int>(s.cursorY), false);
}

CameraPose DebugCamera::pose() const {
    filament::math::float3 eye, target, up;
    const Impl& s = *impl_;
    s.orbit->getLookAt(&eye, &target, &up);
    // camutils orbits a fixed pivot; the whole view is carried with the
    // follow point, so grabs and zoom keep working wherever the player is.
    const math::Vec3 f = follow_;
    return {{eye.x + f.x, eye.y + f.y, eye.z + f.z}, {target.x + f.x, target.y + f.y, target.z + f.z}, {up.x, up.y, up.z}};
}

} // namespace camera
