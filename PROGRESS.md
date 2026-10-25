# Progress

## Current milestone: M2 Hand-Driven Body Repositioning (Reaching) — IN PROGRESS

Goal: a hand target (a ball on the ground, placed by mouse click) drives the whole body,
the same way foot targets drive the body in `repositionBody`. Solved continuously every
tick (the body re-adjusts while the target moves). Recruitment follows real
biomechanics: arm → clavicle → spine → pelvis rotation/tilt → body translation in any
direction (no squat/drop limit yet, to observe behaviour) → one lunge step. If still out
of reach, the body holds its maximum reach toward the target and the leftover error is
kept (the dive trigger, M3). Hand and foot repositioning offsets are added together;
neither overrides the other. Hand mode: one hand (nearest picked automatically) or both.
Standing only (walking cancels the reach). Out of scope: ball physics, dive, reaching
while walking, game logic. One step at a time; stop for a visual check after each.

- [x] **2.0 Research** (awaiting the user's review) → docs/BIOMECHANICS.md section 9 "Reaching" (recruitment order, clavicle,
  spine, pelvis ranges, squat/kneel, balance, timing). Stop for review.
- [x] **2.1 Arm and clavicle** (built; `motion_tests` pass; awaiting the user's visual
  check): left-click the ground to place the ball there; it becomes the hand target
  (R clears it). The nearer hand reaches while standing (walking pauses it).
  `motion::swingToward` (law-of-cosines swing about a pivot, reused by 2.2/2.3) turns
  the clavicle toward the target within its swing-twist limit, then ozz two-bone IK
  places the hand at most `armReach` (0.97 × arm length) from the shoulder, so the
  elbow never locks. The leftover distance is kept in `Fielder::reachError()`.
  Settings in `config/fielder_joints.json` "reach". No smoothing yet (2.6), so the arm
  snaps to a new target. `test_reach.cpp` added.
- [x] **2.2 Spine rotation** (built; tests pass; awaiting the user's visual check): runs
  before the clavicle. One `motion::swingToward` swing about the spine base, split into
  bend and twist (about the character's up), each shared over spine_01 / spine_02 /
  chest by `reach.spineBend` [0.6, 0.2, 0.2] and `reach.spineTwist` [0.2, 0.4, 0.4]
  (BIOMECHANICS 9.3), every bone clamped to its swing-twist limit; `spineIterations` 2
  absorbs clamping. `Fielder::rotateInModel` is shared with the clavicle. agents_tests:
  a ground ball ahead tilts the chest ~58° forward (error left for 2.3–2.4); a target
  behind-left twists the chest toward it.
- [ ] **2.3 Pelvis rotation and tilt**
- [ ] **2.4 Body repositioning toward the hand** (offset added to the foot repositioning)
- [ ] **2.5 Lunge step / max reach hold** (leftover error kept for the dive)
- [ ] **2.6 Balance and natural timing** (CoM, counter-balance arm, springs, two-hand)

## Previous milestone: M1 Procedural Fielder Body and Basic Locomotion — DONE (2026-10-25), awaiting the user's visual review

Goal: a single fielder with an ozz skeleton that stands, walks, runs and jumps with fully
procedural motion. Jumping was dropped (user's choice; it returns with catches and
dives in M3). Out of scope: game logic, teams, ball interaction, skinned mesh. The
M0 ground, pitch and ball stay as they are. Work one step at a time; stop for a visual
check after each step.

- [x] **1.0 Biomechanics research** → [docs/BIOMECHANICS.md](docs/BIOMECHANICS.md)
  (proportions, ROM, balance, walk, run, jump, movement quality, fielding preview).
  Awaiting user review.
- [x] **1.1 Skeleton and body** (built, 2 test suites passing; awaiting the user's visual
  check). Finger bones added (15 per hand, Mixamo-compatible names): 52 bones.
- [ ] **1.1b Textured Mixamo body** (deferred: waiting for the user's model; carried
  forward): the user supplies `assets/models/fielder.glb`
  (Mixamo FBX → Blender → glb). Skinned through gltfio, bone-name mapping in config,
  per-bone retarget correction; capsules kept as a debug toggle.: ozz RawSkeleton built at runtime from
  `config/fielder_skeleton.json` (height ratios); per-bone capsule/box visual kept
  separate; debug bones, joints and axes; ImGui height slider and bone readouts;
  neutral standing pose.
- [x] **1.2 Joint limits and IK** (unit-tested; the IK demo was removed after 1.3; ImGuizmo dropped at the user's request): `config/fielder_joints.json` (hinge and
  swing-twist); our own swing-twist clamp after every solve; ozz IKTwoBoneJob for the
  legs and arms with pole targets; IKAimJob for the head and chest; ImGuizmo targets.
- [x] **1.3 Body repositioning** (built, `motion_tests`; awaiting the user's visual
  check): `motion::repositionBody` runs every tick after the foot targets are set (a
  "late update"; no toggle). It generalises the user's RepositionBody: the pelvis eases
  toward standing height above the stance-weighted feet, then each hip-to-ankle distance
  is clamped to [min, max] × leg length (from the skeleton), stance leg last. Stance
  weight comes from foot heights (the lower foot carries the body) until 1.4 sets it.
  Tunables in `config/fielder_joints.json` "reposition". The IK demo is removed; arm
  and aim IK stay in `ik::` but are not driven until an action needs them.
- [x] **1.4 Foot planting and stepping** (built, `motion_tests`; awaiting the user's
  visual check): `motion::FootStepper` keeps both feet locked; `W A S D` request a
  step (character space; held = keep stepping, alternating feet).
  The next spot is relative to the planted foot (stance width + step from
  `config/fielder_joints.json` "stepping"); one foot swings at a time on a smoothstep
  arc with a sine lift; the body follows via 1.3. Free-fly camera removed
  (user's choice: only the orbit camera is user-controlled; scripted camera paths come
  with M8), freeing WASD/QE/Shift/C. No debug draw (user policy). Since 1.5, WASD walks
  instead of single-stepping.
- [x] **1.5 Walk** (committed `7c63405`; arm swing and head awaiting a visual check):
  `motion::WalkController` (camera-relative WASD → desired velocity; eased start and
  stop, limited turn rate, slows on sharp turns); `motion::gaitTiming` (step = 0.41 H ×
  (v/1.35)^0.42, 60 % stance); `FootStepper::tickGait` places each swing foot ahead of
  its stance spot by the velocity and settles to stance on stop; the root moves and is
  interpolated for rendering; pelvis bob, sway and lean (speed, acceleration, turns).
  Tunables in `config/locomotion.json` "walk" (hot-reloads). WASD now walks; the 1.4
  "stepping" config block is gone (`FootStepper::tick` stays, tested). `motion_tests`
  adds `test_walk.cpp`. W turns to the camera heading; A/D sidestep and S backs up
  without turning. Steps start full size (0.6 m); on stop, feet stay unless more than
  `settleDistance` off; a walk from standing picks its lead foot (no forced swap);
  sidesteps never cross the feet. User-checked: walking works. Arm swing (opposite the
  legs, fading for sidesteps/backing up) and head stabilisation (upright, leads W turns)
  added, awaiting a visual check. Walk-to-point dropped (the fielding AI drives the
  controller later). Gamepad deferred (user: later). Sideways is a side shuffle (feet
  never cross; the crossover comes with running, 1.6), per docs/BIOMECHANICS.md 4.7.
  The orbit camera now follows the fielder (`DebugCamera::setFollowPoint`).
  `test_walk.cpp` not yet run (watch builds skip tests).
- [x] **1.6 Run** (awaiting the user's visual check): paces walk / jog / run, picked by
  input (`config/input.json`): walk by default, hold Shift to run, hold Ctrl to jog
  (user's choice; Shift wins if both). The debug "Fielder run" checkbox
  is removed. `config/locomotion.json` has "walk", "jog" (3 m/s, from walk) and "run"
  (5 m/s, from jog). The chosen pace sets the target speed. The gait shape
  (`motion::blendGait`) blends walk -> jog -> run by the actual speed, so pace changes
  never snap. `motion::GaitStyle` (FootStepper): foot roll for every gait (heel strike
  toes-up, flat, heel rise over the ball with toe bend, toe-off), flight phase, early
  heel-recovery arc apex, step width (running feet nearer the midline but apart). The
  fielder adds pelvis twist, chest counter-rotation and a flipped bob when running.
  Backing up and sidesteps cap at the jog (user's choice): the run preset has no
  back/side speeds, and the shape blend measures speed in the direction of travel.
  `test_walk.cpp` adds blend and flight-phase tests.
  Camera: right-drag rotates by default; Tab toggles mouse-look (the mouse turns the
  orbit camera, cursor hidden via relative mouse mode). Later: sprint (to 9 m/s), crossover, turn-and-run (M3).
- [x] **1.6b Run review fixes (2026-10-25, pre-M2)** (awaiting the user's visual check):
  research review of the run against docs/BIOMECHANICS.md section 5, then fixes:
  - Bob phase: `motion::gaitBob` puts the pelvis lowest mid-stance and highest mid-flight
    (was highest at touchdown). `motion::runShare` blends walk-only/run-only behaviour.
  - Cadence: jog `stepK` 0.6 (~167 steps/min at 3 m/s), run 0.92 (~181 at 5 m/s).
  - `pelvisDrop` (jog 2 cm, run 3.5 cm) for stance knee flexion.
  - Back foot leaves the ground at toe-off and moves forward in flight (`flightCarry`,
    higher `flightLift`); it was planted a whole step, giving a 1.6 m lunge.
  - Pelvis no longer chases the stance foot fore/aft while moving (`forwardFollow`
    from speed); sideways follow per gait (`sideFollow`, run 0.25); wider running
    feet (`widthScale` jog 0.85, run 0.8) so the feet keep their lanes.
  - Smoothness: swing arc eases in at lift-off (old power curve jumped ~8 cm), swing
    keeps its launch velocity (Hermite), soft max leg reach (`reposition.softBand`
    0.02, must stay above standReach), running weight from gait phase
    (`FootStepper::rightWeight`), smoothed acceleration for the lean, continuous idle bob.
  - Input: W takes the camera heading only while stopped, so the camera can orbit a
    moving fielder.
  - Side sway root cause: the min-reach clamp also acted on the lifted leg, so the
    tucked heel pushed the pelvis ±28 cm sideways; the unloaded leg now only limits
    max reach (pelvis now ±0.4 cm). This also caused the crossed-over look.
  Tests: `test_walk.cpp` additions and new `agents_tests` (`test_fielder.cpp`, whole
  fielder at run speed: pelvis sway and foot lanes). User confirmed fixed 2026-10-25.
- [ ] Cross-cutting: config hot-reload key; slow motion (0.1–1x) and pause in ImGui;
  unit tests for joint limits, swing-twist and repositioning.

## Earlier milestone: M0 Foundation — DONE (2026-10-24), awaiting user review

Scope:
- Windows build with one command, and a setup script.
- SDL3 window (1600x900, resizable, vsync) with Filament PBR, at real scale.
- Field, 22-yard pitch, sun with shadows, sky/IBL lighting, stumps and capsules.
- camutils orbit and free-fly cameras.
- Debug lines, spheres and axes.
- ImGui overlay showing fps, frame time and ticks, with toggles.
- 120 Hz fixed timestep with interpolation.
- EnTT entities linked to Filament renderables.
- Math helpers with tests.
- README.


## Checklist (M0)
- [x] CMake + Ninja + MSVC presets; `scripts/build.ps1`, `scripts/setup_windows.ps1`
- [x] Dependencies pinned in `cmake/dependencies.cmake`
- [x] SDL3 window and input → `core::InputState` / `ActionMap` (config/input.json)
- [x] Filament renderer behind `render::Renderer` (scene + UI views)
- [x] Test scene at real scale, with sun shadows and SH sky lighting
- [x] camutils orbit and free-fly cameras (`C` switches)
- [x] `render::DebugDraw` (lines, circles, spheres, axes, boxes, grid)
- [x] ImGui overlay (fps, frame ms, ticks/s, shadows/debug toggles)
- [x] Fixed-timestep loop (`core::App`) with interpolated `SimWorld` probe
- [x] EnTT `game::World` → Filament TransformManager
- [x] GLM wrapper + doctest (56 assertions passing)
- [x] Debug and release build warning-free, game runs
- [x] Real-measurement ground (config/scene.json):
  - round outfield (70 m), boundary rope, run-off, 30-yard circle markers;
  - pitch 20.12 × 3.05 m with bowling, popping and return creases (Law 7);
  - wickets with stumps and bails (Law 8);
  - ball 22.65 cm circumference with a procedural leather, seam and quarter-seam
    shader (Law 4).
- [ ] User visual check of cameras and scene (pending)

## Roadmap
| # | Milestone | Status |
|---|---|---|
| M0 | Foundation: window, render, cameras, debug tools | done |
| M1 | Procedural fielder body and basic locomotion: ozz skeleton, IK, joint limits, stepping, walk, jog, run | done |
| M2 | Hand-driven body repositioning (reaching) | in progress |
| M3 | Basic ball flight (drag), bounce, roll; dive (needs ball arrival time) | not started |
| M4 | Fielding: chase, intercept, pick-up, throw, catch | not started |
| M5 | Wicketkeeper | not started |
| M6 | Bowling | not started |
| M7 | Batting | not started |
| M7 | Match rules | not started |
| M8 | Broadcast cameras | not started |
| M9 | Game UI (RmlUi) | not started |
| M10 | Mobile (Android, then iOS on a Mac) | not started |

Before M1, decide the player body mesh (MakeHuman CC0 or Blender → glTF).

## Known issues / pending
- **Camera gray screen (fixed 2026-10-25):** zoom moved the eye along the ray under the
  cursor and the orbit pitch was unclamped, so the eye went under the field (seen from
  below, the ground is invisible). Zoom now aims at the screen centre, and the orbit is
  clamped (min eye height, min/max distance; config/engine.json camera.orbit).
- **Namespace migration:** M0 code uses bare layer namespaces (`core`, `render`, …).
  The convention is now `cricket::<layer>`. This needs approval, as a mechanical rename.
- **filagui** is not shipped in the prebuilt Filament release, so a small custom ImGui
  backend is used (`src/render/filament/imgui_filament.cpp`). See DECISIONS.md.
- **IBL:** the skybox is a flat colour, with no specular reflection cubemap yet. This is
  fine for rough materials, and can be revisited if visuals need it.
- **Render uploads:** debug-line and ImGui uploads malloc a staging copy per frame. This
  is render-side, not sim; move to ring buffers if profiling shows a cost.
- **Unchecked by eye:** camera feel and vsync behaviour on the OpenGL backend.
