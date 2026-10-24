# Design

Living document: update it whenever the architecture changes.

## Goals

- Full 3D cricket for PC (Windows first), then Android and iOS from the same code.
- **All gameplay character motion is procedural**: bones driven by IK, joint constraints,
  balance and physics. Clips are allowed only for non-gameplay characters and moments
  (umpire, crowd, celebrations, cutscenes), blended with procedural IK on the same
  skeleton.
- No game engine: our own thin engine on proven open-source libraries.
- Built for mobile performance from day one.

## Library policy

Use a popular, maintained, permissively licensed library (MIT/BSD/Apache/zlib, works on
Windows/Android/iOS) for anything general-purpose. Our own code is only for what no library
provides: procedural motion, ball physics, fielding AI, match rules. IK and constraints follow
standard published algorithms (analytic two-bone IK, FABRIK, swing-twist joint limits).

All versions are pinned in [cmake/dependencies.cmake](cmake/dependencies.cmake).

| Library | Use | Status |
|---|---|---|
| SDL3 | window, input, (later) audio | M0 |
| Filament | PBR rendering, scene/transform/light system | M0 |
| Filament camutils | orbit and free-flight debug cameras | M0 |
| Filament gltfio | glTF model loading (`Renderer::loadModel`) | M0 (wired, no models yet) |
| GLM | math, wrapped by `src/math` | M0 |
| Dear ImGui (docking) | debug UI | M0 |
| nlohmann/json | config files | M0 |
| spdlog | logging | M0 |
| EnTT | game objects (entities/components) | M0 |
| doctest | unit tests | M0 |
| ozz-animation | skeletons, two-bone/aim IK, blending, skinning, clip playback | M1 |
| ImGuizmo | dragging IK targets in the viewport | M1 |
| Jolt Physics | player body collisions | later |
| RmlUi | game menus / HUD | later |

Anything else: ask first.

## Layers

Each layer depends only on the layers below it.

```
match      game rules, sim world                      (M0: SimWorld placeholder)
ai         fielding / bowling / batting decisions     (empty)
agents     player bodies combining motion + ai        (empty)
motion     balance, locomotion, procedural actions    (empty)
ik         FABRIK, swing-twist limits (ozz for 2-bone/aim)  (empty, M1)
rig        skeletons (ozz-animation)                  (empty, M1)
math       GLM wrapper: types, lookRotation, fromTo, swingTwist, Transform
core       app loop, fixed timestep, config, input mapping, events, logging
```

Side modules used by the app shell (`src/game`), never by the sim:

```
platform   SDL3 window, input, ImGui platform glue    (only place with SDL)
render     Renderer interface, mesh generation, debug draw; render/filament = backend
camera     debug cameras (camutils)
debug      ImGui overlay
physics    Jolt integration (later)
ui         game UI (later)
game       CricketApp: wires everything; EnTT World; test scene
```

## Folder layout

```
CMakeLists.txt, CMakePresets.json
cmake/dependencies.cmake   all pinned library versions
cmake/Filament.cmake       prebuilt Filament import + matc material compile
scripts/                   setup_windows.ps1, build.ps1
third_party/filament/      prebuilt SDK (git-ignored)
config/                    engine.json, input.json, scene.json
assets/materials/          Filament .mat sources
src/main.cpp
src/core math platform render camera debug game match
src/rig ik motion agents ai physics ui      (empty until their milestone)
tests/                     doctest
```

## Data flow (sim → render)

```
SDL events ─► platform::Window ─► core::InputState ─► core::ActionMap
                                        │
             ┌──────────────────────────┴───────────────┐
             ▼ fixed 120 Hz                             ▼ per rendered frame
   match::SimWorld::tick(dt)                  camera::DebugCamera::update
   (previous ← current; advance)                         │
             │ interpolate(alpha)                        │
             ▼                                           ▼
   game::World (EnTT) TransformComponent ──► render::Renderer (interface)
             └── syncToRenderer (dirty only)    └── FilamentRenderer: TransformManager,
                                                    scene view + ImGui view → swap chain
```

## Main loop and timing

`core::App::run()` drives a fixed-timestep loop (Glenn Fiedler's "Fix Your Timestep"):

```
frame:
  pumpEvents()                         SDL -> InputState / EventQueue
  n = accumulator / step (capped)      config: sim.tickHz (120), sim.maxStepsPerFrame
  n x fixedUpdate(step)                SimWorld::tick — the only place sim state changes
  renderFrame(alpha)                   render interpolates previous/current sim state
```

- The sim keeps `previous` and `current` state; rendering shows `lerp/slerp(previous,
  current, alpha)`, so motion is smooth at any frame rate and the sim is deterministic per tick.
- Frame time is clamped (0.25 s) and excess steps are dropped, so hitches never spiral.
- The sim never calls the renderer. `CricketApp::renderFrame` copies interpolated sim
  state into EnTT components, and `World::syncToRenderer` pushes dirty transforms.
- Vsync: the swap chain presents at the display rate, and a frame limiter at the display
  refresh rate backs it up (`window.vsync`).

## Game objects (EnTT) and the scene

- Every game object is an EnTT entity in `game::World`.
- `TransformComponent` holds the local `math::Transform` plus a dirty flag;
  `Renderable` holds a `render::ObjectId`.
- Each `ObjectId` maps to Filament entities inside the renderer. Transforms go to
  Filament's `TransformManager`, so hierarchies (glTF nodes, later attachments) are
  native. Lights are Filament light components.
- Game code never includes Filament headers; only `src/render/filament` and
  `src/camera` (camutils, a header-light helper library) do.

## Rendering (Filament)

- `render::Renderer` is the interface; `render/filament/filament_renderer.cpp` is the only
  implementation. Replacing Filament means writing a new implementation of that interface.
- Two views per frame: the 3D scene, then an ImGui view (orthographic, no post-processing,
  translucent blend).
- Lighting: a sun light with cascaded PCF shadows, plus image-based lighting. The irradiance
  is 3-band spherical harmonics computed from a procedural sky gradient, and the skybox is a
  solid colour. Physical units: the sun is in lux, with the camera's default exposure.
- Materials are `.mat` sources in `assets/materials`, compiled by Filament's `matc` at
  build time into `<bin>/assets/materials/*.filamat` (OpenGL + Vulkan).
- Debug draw (`render::DebugDraw`): a fixed-capacity line buffer (lines, circles,
  spheres, axes, boxes, grids). It is uploaded once per frame as a single LINES renderable
  and cleared. Bones and IK targets will use it.
- ImGui is drawn with our own small Filament backend (`imgui_filament.cpp`), using the
  ImGui 1.92 dynamic-texture protocol. Filament's `filagui` helper would be the library
  choice, but it is not shipped in the prebuilt Windows release, and it bundles its own
  older ImGui, which conflicts with the docking branch. This backend is glue code where the
  libraries stop; revisit it if Filament starts shipping filagui.
- Backend: OpenGL by default; `render.backend = "vulkan"` switches.

## Conventions

- Right-handed, +Y up, **1 unit = 1 metre**, seconds, radians in code, degrees in config.
- The pitch lies along Z, centred at the origin, with stumps at z = ±10.06.
- Bone/character forward is local +Z (`math::lookRotation`). Cameras look down -Z.
- Matrices are column-major (GLM and Filament share the layout).
- `math::Quat` is `glm::quat`. Its constructor order is **(w, x, y, z)**.

## Data-driven tunables

`config/*.json` (comments allowed) is read at startup through `core::Config` by dotted
path with an in-code fallback. Values are cached at init, never looked up in the sim loop.

## Memory and performance rules

- No heap allocations per sim tick. `SimWorld`, `FixedTimestep`, `EventQueue` and
  `InputState` are fixed-size.
- Render-side per-frame uploads use malloc'd staging copies that Filament frees after
  consumption (debug lines, ImGui). This is acceptable at the current scale; move to ring
  buffers if profiling shows it.
- Assets and meshes are created at load time only.

## Platform code

Only `src/platform` touches SDL or OS APIs. `platform::Window` exposes the native handle
the renderer needs (HWND on Windows; ANativeWindow / CAMetalLayer later).

## Systems

One section per system, filled in as each is built.

### Rig (M1): not started
ozz-animation runtime skeleton, loaded from the player glTF. Body sizes come from config.

### IK (M1): not started
ozz two-bone and aim IK. Our own FABRIK chains and swing-twist joint limits (published
algorithms), driven by targets that can be dragged with ImGuizmo.

### Motion (M1+): not started
Balance, locomotion and procedural actions built on rig and IK.

### Ball physics (M2): not started
Our own code: flight with drag, bounce off pitch and outfield, rolling. Fixed-step.

### Agents / AI (M3+): not started
Player bodies (agents) driven by fielding, bowling and batting decisions (ai).

### Match (M7): not started
Rules, scoring, innings. `match::SimWorld` is currently a placeholder probe.

### Broadcast cameras (M8): not started
Game cameras imitating TV coverage. Debug cameras stay on camutils.

### Game UI (M9): not started
RmlUi for menus and HUD. ImGui stays debug-only.

## Milestones

Status and checklists live in [PROGRESS.md](PROGRESS.md).

- **M0 (done):** this skeleton.
- **M1:** procedural fielder rig: ozz-animation skeleton, ozz two-bone/aim IK, our
  FABRIK + swing-twist limits, ImGuizmo target dragging, debug-drawn bones.
- **M2:** basic ball physics (ours): flight with drag, bounce off the pitch and outfield,
  and rolling. Fielders need a ball to chase.
- **M3:** fielding: chase, intercept, pick-up, throw and catch, using M1 + M2.
- Then batting and bowling reuse the same rig, IK and motion systems.

## Decisions still open

- **Player body mesh** (decide before M1): a skinned humanoid exported as glTF and loaded
  with gltfio. Candidates are MakeHuman (open source, CC0 assets) or a custom Blender
  model.
- **Mobile:** Android builds from Windows. iOS builds need a Mac with Xcode.

## Workflow

- Git from day one: commit after every working step, so any change can be rolled back.
- Procedural motion is judged by eye. Expect the user's visual feedback on every motion
  milestone ("the knee bends too early") and plan iteration time for it.
