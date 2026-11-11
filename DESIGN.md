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
| Jolt Physics | generic rigid bodies (player bodies, bat; later), behind `src/physics/core` | M3 |
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
physics    core: Jolt behind PhysicsWorld (M3); ball: our ball solver (M3)
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

### Test scene (M0): built
`game/test_scene.cpp` builds everything from `config/scene.json`, using Laws of
Cricket dimensions:
- **Ground:** round outfield disc to the boundary radius, a boundary-rope torus, a
  run-off ring and an outer surround.
- **30-yard circle:** two semicircles around each middle stump joined by straight lines,
  drawn as merged painted dots.
- **Pitch and creases:** a pitch strip plus bowling, popping and return creases (one
  merged mesh).
- **Wickets:** removed 2026-11-04 (the user's call, before the body physics); they come
  back later as Jolt bodies.
- **Ball:** a sphere using the `CricketBall` material kind (procedural seam shader).

Small parts are merged with `render::appendMesh` to keep draw calls low.

### Rig (M1): skeleton built (Step 1.1)
- `rig::SkeletonDef` (`src/rig/skeleton_def.*`) parses `config/human_skeleton.json`.
  Every offset is a **ratio of standing height** (Drillis & Contini / Winter ratios,
  de Leva 1996 masses; tables and derivations in
  [docs/BIOMECHANICS.md](docs/BIOMECHANICS.md) section 1). Left-side bones are defined
  once and mirrored to `_r`.
- `rig::Rig` (`src/rig/rig.*`) builds an ozz `RawSkeleton` at a given height, runs
  `SkeletonBuilder`, and converts AoS `rig::Pose` locals to model matrices with
  `LocalToModelJob`. Joint indices are ozz's depth-first order; ozz types stay in
  `rig.cpp`.
- 52 bones: pelvis (root), spine_01, spine_02, chest, neck, head; per side clavicle,
  upper_arm, forearm, hand, thigh, shin, foot, toe, plus fingers (thumb, index, middle,
  ring and pinky, `_01`–`_03` each; thumb_01 is the metacarpal). Finger segments are
  fractions of hand length (Buchholz et al. 1992). The names map 1:1 to Mixamo's
  skeleton for the planned textured body (Step 1.1b).
- **Rest pose = neutral standing pose**, with identity local rotations. Every joint
  frame is aligned with character space (+Y up, +Z forward, +X = character's left), so
  joint-limit axes are the same for every bone. Key landmarks at H: hip 0.530, knee
  0.285, ankle 0.039, shoulder 0.818; leg length (thigh + shin) 0.491 H.
- `agents::Fielder` owns the rig, the previous and current pose, and the root
  transform. It is ticked in the sim; rendering interpolates the locals.
- Body visual: `game::FielderVisual` creates one capsule or box per bone (shape in
  config). Meshes are in height-ratio units, scaled by the height at draw time, so the
  height slider needs no new meshes. A skinned glTF replaces this layer later.
- Debug: `debug::drawSkeleton` (bones, joints, axes, centre of mass with its ground
  projection), toggled from the main Debug window (the separate "Fielder" panel was
  removed).

### IK (M1): joint limits and limb IK built (Step 1.2)
- `ik::JointLimits` (`src/ik/joint_limits.*`, our own code): per-joint limits from
  `config/human_joints.json`.
  - **Hinge:** twist about one axis, clamped.
  - **Swing-twist:** `math::swingTwist` decomposition. The twist is clamped to a range;
    the swing rotation vector is clamped to an asymmetric ellipse (a radius for each
    direction).
  - The left side is in config; the right side is mirrored (rotations about Y and Z
    flip sign).
- `ik::solveTwoBone` / `ik::solveAim` (`src/ik/limb_ik.*`) wrap ozz `IKTwoBoneJob` /
  `IKAimJob`. The mid axis opens the joint: the knee uses −X and the elbow +X. Poles
  are set so the knees point forward and the elbows back and out.
- `agents::Fielder::tick` order:
  1. rest pose, then body repositioning over the foot targets (`motion::repositionBody`);
  2. (chest and head aim: not driven yet);
  3. two-bone IK for the legs (arms: not driven yet);
  4. limits;
  5. feet flattened to character space;
  6. limits.
  No allocations.
- The IK demo was removed after Step 1.3 (the user's choice); feet are driven by
  `motion::FootStepper` (Step 1.4, WASD via `Fielder::setStepInput`). ImGuizmo was not added (the user's choice).

### Motion (M1): repositioning, stepping, walk, jog, run built
- The formula repositioning (`moveBodyToFeet` in `src/agents/human.cpp`; was
  `motion::repositionBody` in `src/motion/body_reposition_backup.*` until 2026-11-06;
  `motion::BodyReposition` in `src/motion/body_reposition.*` replaced it for standing, walking,
  running and the limb test): every tick after the foot
  targets are set, the pelvis eases toward standing height above the stance-weighted
  feet, and each hip-to-ankle distance is clamped to [min, max] × leg length
  (`config/human_joints.json` "reposition").
- `motion::WalkController` (`src/motion/walk_controller.*`): camera-relative WASD →
  desired velocity in character space, eased start and stop, limited turn rate, slowing
  on sharp turns. W turns to the camera heading; A/D sidestep and S backs up without
  turning. `gaitTiming` gives step length (k · H · (v/v_ref)^0.42), swing, stance and
  flight time.
- **Paces:** `motion::Gait` Walk / Jog / Run, from `config/locomotion.json` "walk",
  "jog" (inherits walk) and "run" (inherits jog). Input: walk by default, Ctrl jogs,
  Shift runs. Sidesteps and backing up cap at the jog. The chosen pace sets the target
  speeds; the gait *shape* is `motion::blendGait` of the presets by the actual speed
  (against each pace's speed in the direction of travel), so pace changes never snap.
- `motion::FootStepper` (`src/motion/foot_stepper.*`): feet are world-locked while
  planted (`rebase` when the root moves). Gait mode places each swing foot ahead of its
  stance spot by the velocity (it passes under the hip mid-stance), alternates feet on
  the gait timing, side-shuffles without crossing, and settles to stance on stop.
  `motion::GaitStyle`:
  - foot roll for every gait: heel strike (toes up), flat, heel rise over the ball (toe
    bend keeps the toes down), toe-off;
  - flight phase for jog and run (the toes push off before the swing foot lands);
  - arc apex time (early = heel recovery);
  - step width.
  The 1.4 single-step mode (`tick`) stays, tested.
- `agents::Fielder::tick` order:
  1. rest pose;
  2. walk (move and turn the root, step the feet);
  3. body repositioning;
  4. pelvis bob (flipped when running), sway, lean (speed, acceleration, turns), pelvis
     twist with chest counter-rotation;
  5. leg IK and limits;
  6. feet at the stepper's pitch, plus toe bend;
  7. arm swing opposite the legs (elbows near 90° when running);
  8. head stabilisation (upright, leads turns);
  9. limits.
  No allocations.
- Jumping was dropped from M1 (the user's choice). It comes back with the fielding
  actions that need it (M3: catches, dives).

### Reaching (M2): in progress
A hand target drives the body like foot targets do in `repositionBody`: arm, clavicle,
spine, pelvis, then body translation (summed with the foot offset), then one lunge step;
beyond that the body holds maximum reach. See docs/BIOMECHANICS.md "Reaching".
Built: `motion::swingToward` (src/motion/reach.h) and `Fielder::reach()` (clavicle +
arm, Step 2.1); left click places the target (the ball), R clears it.

### Physics (M3): in progress
`physics::PhysicsWorld` (src/physics/core) wraps Jolt: init from config/physics_world.json
(shared gravity, air density, wind), stepped once per 120 Hz tick by `match::SimWorld`,
previous/current transform per body for interpolation, preallocated temp allocator
(no per-tick heap use), single-threaded job system by default. Jolt stays inside the .cpp.

### Physics body and balance (M6): whole-body decider on seven targets (2026-11-10)
The body moves only by forces. Nothing places a part while its physics is on. Two
systems, strictly apart (DECISIONS 2026-11-10 "Whole-body decider on targets"):
```
Planner (an action: idle, step, walk, reach, dive; `motion::StepPlanner` is the example)
   knows the action; only places the 7 targets over time (`motion::TargetSet`):
   pose, mode (Free / Driven / Pinned), weights, feet's swing share and ground normal;
   gives the capsule its intent velocity; reads the decider's reports
        ▼
Decider (`motion::BodyDecider`; knows no action; never reads the physics body per tick)
   1 Pinned feet: anchors (never slide)       2 Support weights (blended) → hips reference
   3 Driven targets (blended weights)          4 Reach: limbs clamped to 96 % length; trunk
     queue in real body order; short → reported 5 Posture: chest upright, head level, free
     arms swing (Collins 2009) and settle in the chest's frame
   → output pose: hips speed/acceleration-bounded, joint rates bounded, warm-started from
     last tick; hard constraint: pelvis below every
     grounded leg's reach; grounded legs solved last on the output pelvis
        ▼
Muscles (`agents::HumanBody`): holdWeight x strength, Jolt Ragdoll::DriveToPoseUsingMotors
   toward the output pose (head world-stabilised from the real chest), torque limits x strength
        ▼
Capsule + pin (`agents::Human::followCapsule` / `physicsTick`): the capsule LEADS: it moves by
   the planner's intent (+ a slow re-centre toward the support while both feet are planted,
   the plan rebased so the pelvis's world target does not move); the pin (SixDOF spring from
   the capsule's feet) pulls the real pelvis to the plan's pelvis, force x strength x support
   under the feet (none in flight). Ragdoll (let go): the capsule follows the real pelvis;
   at rest it gets up from the landed pose (the decider re-seeded once).
        ▼
Jolt step; then the real pelvis's error from the pin target is classed: small (ignore),
   medium sustained (planner: recovery step), large (let go, when the release is on).
```
**Tick order** (one fixed 120 Hz step, `Human::tick`): walk/turn → planner (step clock,
intent) → capsule moves → root = capsule → targets → `decide` → `holdWeight`,
`drivePose`, pin target → Jolt step → error readback (next tick). No render dt anywhere.

**Why not a self-balancing controller:** every attempt (Peterka ankles, centre-of-mass
Jᵀ, torque legs, QP) fell or drifted, measured 2026-11-06..08; the plan read the lagging
body back and jumped. Balance is by construction (support-weighted hips reference, the pin),
and stepping is the planner's answer to a reported error or short reach, as in sports
games' physical animation (PuppetMaster, Unreal Physical Animation). Capture-point
stepping can come later inside the planner, without touching the decider.

**Edge cases and how the design meets them:** out of reach / past limits: clamped at 96 %,
reported (`BodyQueue::reach`), never snapped. Both feet unpinned: support 0, the hips'
reference carries on ballistically, the pin fades to nothing. A foot moving while it is
the support: its weight ramps off over supportBlendTime; the hips shift before it lifts.
Re-target or cancel mid-step: paths continue from the current target (`StepPlanner::
retarget` / `cancel`), the output pose from the current plan. A hand needing the trunk:
the reach queue, limited to what the hips (if driven) leave; still short → reported. Leg
near straight: never (reachClamp). Slopes / stairs: feet pin with the ground normal, the
hips' height follows the weighted feet. A hit during a step: the body deviates, the plan
does not read it; the error classes decide. Activation mid-motion: blended over blendTime,
from the current plan; deactivation: feet re-pin where they are, others blend to rest. Big
dt: the app's fixed step only. Ragdoll in/out: strength and the pin scale; get-up re-seeds
the plan from the landed pose. A wall: the capsule stops; the plan's pelvis stays within
its bound (`offsetClamped`), the feet targets clamp to reach and report short.

**Tuning guide** (`human_joints.json` "decider", `physics_world.json` "character"):
- Snapping hips: lower `hipsMaxAccel` (8) / `hipsMaxSpeed` (2); too sluggish: raise them.
- Weight shift too abrupt / too slow before a step: `supportBlendTime` (0.12 s) and the
  planner's `step.liftAt`.
- Targets switching on/off visibly: `blendTime` (0.1 s).
- Knees snapping straight: `reachClamp` (0.96) lower; legs too bent standing: higher.
- Head over-rotating: `headMaxRate` (6 rad/s) lower; `passiveInfluence.head` toward 1.
- Trunk whipping in reaches: `trunkMaxRate` (4 rad/s).
- Falls / recovery: `pelvisError` thresholds and `release.on`; muscles and pin through
  `Human::setStrength`.

**Debug drawing** (`CricketApp::drawCapsule`): pinned feet as white rings sized by support
weight, driven targets magenta, the plan's pelvis cyan with its offset line from the
capsule (red when clamped), short reaches as red lines, the capsule white/red (held / let
go). Numbers: `Human::pelvisError()` / `pelvisErrorDistance()`, `BodyQueue::reach`.

**Test checklist** (tests/test_body_decider.cpp, tests/test_human.cpp "stepping on the
physics body"): no snapping: the plan's pelvis speed and acceleration under their bounds
through a sudden stance change and a 0.5 m landing; no head over-rotation: the real chest
swing under 10 deg through steps; no foot sliding: a pinned foot's plan position within
2 mm through a step, the real ankle's slide measured; no capsule drift: the capsule under
the real pelvis every tick (DECISIONS 2026-11-11 night); no switches: support
weights and activations change by at most dt/blendTime per tick; a far target reports
short; a target let go blends back without a jump.

**Never:** outside forces on the body (one exception, the user's call 2026-11-08: the
assist string, off by default), placing parts while physics is on, locked axes, tuned
strategy thresholds (physical limits decide), per-action balance code, the capsule
following the real pelvis while it holds the body.

### Own sphere physics (M3): in progress
Generic, reusable by any object we simulate ourselves (ball now; e.g. players' contact
queries later), in `src/physics/sim`: force generators (gravity, drag; spin and swing
later as new generators), box and sphere colliders with sphere contact tests
(`ColliderSet`, fixed capacity, a surface per shape), and `SphereSolver` (substepped
semi-implicit Euler, restitution bounce, grip-slip friction to rolling, rolling
resistance to rest; spin and orientation). (Stumps and bails were Jolt bodies until
2026-11-04; removed for now.) `src/physics/ball` only configures it
for the cricket ball. The predictor (3.4) will call the same `step()`.

### Agents / AI (M4+): not started
Player bodies (agents) driven by fielding, bowling and batting decisions (ai).

### Match (M8): not started
Rules, scoring, innings. `match::SimWorld` is currently a placeholder probe.

### Broadcast cameras (M8): not started
Game cameras imitating TV coverage. Debug cameras stay on camutils.

### Game UI (M9): not started
RmlUi for menus and HUD. ImGui stays debug-only.

### Debug UI and demos (policy)
- No demo scripts or demo code (target sweeps, showcase paths, test drivers in the game)
  unless the user asks for them. Features are verified with unit tests and real
  gameplay code.
- No new checkboxes, toggles, panels or readouts in the debug UI unless the user asks.
  The existing toggles stay in the single main Debug window, default off; anything the
  user requests goes there too.

## Milestones

Status and checklists live in [PROGRESS.md](PROGRESS.md).

- **M0 (done):** this skeleton.
- **M1 (done):** procedural fielder rig: ozz-animation skeleton, ozz two-bone/aim IK,
  our swing-twist limits, body repositioning, stepping, walk, jog, run (no jump, no
  ImGuizmo, no demos).
- **M2:** hand-driven reaching and body repositioning (no ball, no game logic).
- **M3:** basic ball physics (ours): flight with drag, bounce off the pitch and outfield,
  and rolling; dive (needs the ball's arrival time).
- **M4:** fielding: chase, intercept, pick-up, throw and catch, using M1–M3.
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
