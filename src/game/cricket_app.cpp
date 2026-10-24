#include "game/cricket_app.h"

#include "core/log.h"
#include "platform/imgui_platform.h"

#include <imgui.h>

#include <chrono>

namespace game {

namespace {

core::LoopSettings loopSettings(const core::Config& c) {
    core::LoopSettings s;
    s.simHz = c.getFloat("sim.tickHz", 120.0f);
    s.maxStepsPerFrame = c.getInt("sim.maxStepsPerFrame", 8);
    return s;
}

render::LightingDesc readLighting(const core::Config& c) {
    render::LightingDesc l;
    l.sunDirection = c.getVec3("lighting.sunDirection", l.sunDirection);
    l.sunColor = c.getVec3("lighting.sunColor", l.sunColor);
    l.sunIlluminance = c.getFloat("lighting.sunIlluminance", l.sunIlluminance);
    l.skyColor = c.getVec3("lighting.skyColor", l.skyColor);
    l.horizonColor = c.getVec3("lighting.horizonColor", l.horizonColor);
    l.groundColor = c.getVec3("lighting.groundColor", l.groundColor);
    l.iblIntensity = c.getFloat("lighting.iblIntensity", l.iblIntensity);
    return l;
}

} // namespace

CricketApp::CricketApp(AppConfigs configs) : core::App(loopSettings(configs.engine)), cfg_(std::move(configs)) {}

bool CricketApp::init() {
    const core::Config& e = cfg_.engine;

    platform::WindowDesc wd;
    wd.title = e.getString("window.title", wd.title);
    wd.width = e.getInt("window.width", wd.width);
    wd.height = e.getInt("window.height", wd.height);
    wd.resizable = e.getBool("window.resizable", true);
    if (!window_.create(wd)) return false;

    vsync_ = e.getBool("window.vsync", true);
    if (vsync_) targetFrameSeconds_ = 1.0f / window_.refreshRate();

    // Dear ImGui context first: the renderer registers itself as ImGui's backend.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;  // no imgui.ini next to the exe; layout is code-defined
    ImGui::StyleColorsDark();

    render::RendererDesc rd;
    rd.nativeWindow = window_.nativeHandle();
    rd.width = window_.pixelWidth();
    rd.height = window_.pixelHeight();
    rd.backend = e.getString("render.backend", rd.backend);
    rd.vsync = vsync_;
    rd.shadowMapSize = e.getInt("render.shadowMapSize", rd.shadowMapSize);
    rd.shadowCascades = e.getInt("render.shadowCascades", rd.shadowCascades);
    rd.shadowFar = e.getFloat("render.shadowFar", rd.shadowFar);
    rd.debugLineCapacity = static_cast<size_t>(e.getInt("render.debugLineCapacity", 65536));
    rd.assetDir = e.getString("render.assetDir", rd.assetDir);
    renderer_ = render::createFilamentRenderer();
    if (!renderer_->init(rd)) return false;
    backendName_ = rd.backend;

    renderer_->setLighting(readLighting(e));

    if (!platform::imguiPlatformInit(window_)) return false;
    imguiReady_ = true;

    actions_.load(cfg_.input);
    camera_.configure(e, rd.width, rd.height);
    sim_.configure(e);

    scene_ = buildTestScene(world_, *renderer_, cfg_.scene);

    debugSettings_.shadows = e.getBool("render.shadows", true);
    debugSettings_.debugDraw = e.getBool("debug.debugDraw", true);
    debugSettings_.showOverlay = e.getBool("debug.showOverlay", true);
    appliedSettings_ = debugSettings_;
    appliedSettings_.shadows = !debugSettings_.shadows;  // force the first apply
    appliedSettings_.showProbe = !debugSettings_.showProbe;
    return true;
}

void CricketApp::shutdown() {
    if (imguiReady_) platform::imguiPlatformShutdown();
    imguiReady_ = false;
    world_.registry().clear();
    if (renderer_) renderer_->shutdown();  // releases ImGui textures, so before DestroyContext
    renderer_.reset();
    if (ImGui::GetCurrentContext()) ImGui::DestroyContext();
    window_.destroy();
}

bool CricketApp::pumpEvents() {
    input_.beginFrame();
    window_.pumpEvents(input_, events_);

    core::Event ev;
    bool running = true;
    while (events_.pop(ev)) {
        switch (ev.type) {
            case core::EventType::Quit: running = false; break;
            case core::EventType::WindowResized:
                renderer_->resize(ev.width, ev.height);
                camera_.setViewport(ev.width, ev.height);
                break;
            default: break;
        }
    }

    if (imguiReady_) {
        const ImGuiIO& io = ImGui::GetIO();
        input_.uiWantsKeyboard = io.WantCaptureKeyboard;
        input_.uiWantsMouse = io.WantCaptureMouse;
    }

    if (actions_.pressed(input_, core::Action::Quit)) running = false;
    if (actions_.pressed(input_, core::Action::ToggleOverlay)) debugSettings_.showOverlay = !debugSettings_.showOverlay;
    if (actions_.pressed(input_, core::Action::ToggleDebugDraw)) debugSettings_.debugDraw = !debugSettings_.debugDraw;
    if (actions_.pressed(input_, core::Action::ToggleCamera)) debugSettings_.cameraMode ^= 1;
    return running;
}

void CricketApp::fixedUpdate(double dt) { sim_.tick(dt); }

void CricketApp::renderFrame(double frameSeconds, float alpha) {
    auto frameStart = std::chrono::steady_clock::now();

    // --- Cameras (render-side, real frame time) ---
    camera_.setMode(debugSettings_.cameraMode == 0 ? camera::CameraMode::Orbit : camera::CameraMode::FreeFly);
    bool captureMouse = camera_.update(input_, actions_, static_cast<float>(frameSeconds));
    window_.setRelativeMouse(captureMouse);
    camera::CameraPose pose = camera_.pose();
    render::CameraDesc cam;
    cam.eye = pose.eye;
    cam.target = pose.target;
    cam.up = pose.up;
    cam.fovYDegrees = cfg_.engine.getFloat("camera.fovY", 50.0f);
    cam.nearPlane = cfg_.engine.getFloat("camera.near", 0.1f);
    cam.farPlane = cfg_.engine.getFloat("camera.far", 2000.0f);
    renderer_->setCamera(cam);

    // --- Apply toggles that changed ---
    if (debugSettings_.shadows != appliedSettings_.shadows) renderer_->setShadowsEnabled(debugSettings_.shadows);
    if (debugSettings_.debugDraw != appliedSettings_.debugDraw) renderer_->setDebugDrawEnabled(debugSettings_.debugDraw);
    if (debugSettings_.showProbe != appliedSettings_.showProbe) {
        sim_.setProbeEnabled(debugSettings_.showProbe);
        renderer_->setObjectVisible(world_.registry().get<Renderable>(scene_.probe).object, debugSettings_.showProbe);
    }
    appliedSettings_ = debugSettings_;

    // --- Interpolated sim state -> entities -> renderer scene ---
    math::Transform probe = sim_.probeTransform(alpha);
    for (auto e : world_.registry().view<FollowsProbe>()) world_.setTransform(e, probe);
    world_.syncToRenderer(*renderer_);

    if (debugSettings_.debugDraw) drawDebugGeometry(alpha);

    // --- Debug UI ---
    platform::imguiPlatformNewFrame();
    ImGui::NewFrame();
    debug::OverlayInfo info;
    info.stats = &stats();
    info.simHz = cfg_.engine.getFloat("sim.tickHz", 120.0f);
    info.renderAlpha = alpha;
    info.debugLines = renderer_->debugDraw().vertexCount() / 2;
    info.debugLinesDropped = renderer_->debugDraw().droppedLines();
    info.backend = backendName_.c_str();
    debug::drawOverlay(debugSettings_, info);
    ImGui::Render();

    renderer_->render(ImGui::GetDrawData());

    // Frame limiter backs up the swap chain's vsync (and caps the rate if the
    // driver ignores it), keeping the GPU from rendering hundreds of frames.
    if (targetFrameSeconds_ > 0.0f) {
        double spent = std::chrono::duration<double>(std::chrono::steady_clock::now() - frameStart).count();
        platform::preciseSleep(targetFrameSeconds_ - spent - 0.0005);
    }
}

void CricketApp::drawDebugGeometry(float alpha) {
    render::DebugDraw& dd = renderer_->debugDraw();
    if (debugSettings_.showGrid) dd.grid(math::Vec3(0, 0.02f, 0), 30.0f, 5.0f, render::rgba(255, 255, 255, 60));
    dd.axes(math::Transform::identity(), 2.0f);

    // Pitch outline and stump lines.
    float halfLen = 0.5f * scene_.pitchLength;
    dd.box(math::Vec3(0, 0.36f, halfLen), math::Vec3(0.115f, 0.36f, 0.02f), render::colors::kYellow);
    dd.box(math::Vec3(0, 0.36f, -halfLen), math::Vec3(0.115f, 0.36f, 0.02f), render::colors::kYellow);

    if (debugSettings_.showProbe) {
        math::Transform probe = sim_.probeTransform(alpha);
        dd.axes(probe, 0.6f);
        dd.sphere(probe.position, 0.4f, render::colors::kCyan);
    }
}

} // namespace game
