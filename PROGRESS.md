# Progress

## Current milestone: M1 Procedural Fielder Body and Basic Locomotion — IN PROGRESS

Goal: a single fielder with an ozz skeleton that stands, walks, runs and jumps with fully
procedural motion. Out of scope: game logic, teams, ball interaction, skinned mesh. The
M0 ground, pitch and ball stay as they are. Work one step at a time; stop for a visual
check after each step.

- [x] **1.0 Biomechanics research** → [docs/BIOMECHANICS.md](docs/BIOMECHANICS.md)
  (proportions, ROM, balance, walk, run, jump, movement quality, fielding preview).
  Awaiting user review.
- [x] **1.1 Skeleton and body** (built, 2 test suites passing; awaiting the user's visual
  check). Finger bones added (15 per hand, Mixamo-compatible names): 52 bones.
- [ ] **1.1b Textured Mixamo body**: the user supplies `assets/models/fielder.glb`
  (Mixamo FBX → Blender → glb). Skinned through gltfio, bone-name mapping in config,
  per-bone retarget correction; capsules kept as a debug toggle.: ozz RawSkeleton built at runtime from
  `config/fielder_skeleton.json` (height ratios); per-bone capsule/box visual kept
  separate; debug bones, joints and axes; ImGui height slider and bone readouts;
  neutral standing pose.
- [ ] **1.2 Joint limits and IK**: `config/fielder_joints.json` (hinge and
  swing-twist); our own swing-twist clamp after every solve; ozz IKTwoBoneJob for the
  legs and arms with pole targets; IKAimJob for the head and chest; ImGuizmo targets.
- [ ] **1.3 Body repositioning**: port of the user's RepositionBody, with the stance
  foot as a parameter and leg length from the skeleton (or a better approach).
- [ ] **1.4 Foot planting and stepping**: locked feet, a step trigger from the ideal
  position, a swing arc from config, one swing foot at a time, debug draw.
- [ ] **1.5 Walk**: desired-velocity controller (WASD, gamepad, walk-to-point), gait
  from `config/locomotion.json`, pelvis bob and sway, lean, arm swing, head
  stabilisation, smooth start, stop and turn.
- [ ] **1.6 Run**: speed-driven blend, flight phase, sprint up to realistic speed.
- [ ] **1.7 Jump**: crouch, push, ballistic flight, tuck, absorbing landing, recovery;
  standing and running jumps.
- [ ] Cross-cutting: config hot-reload key; slow motion (0.1–1x) and pause in ImGui;
  unit tests for joint limits, swing-twist and repositioning.

## Previous milestone: M0 Foundation — DONE (2026-10-24), awaiting user review

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

Next milestone: **M1 Rig and IK**. Start it with the `start-milestone` skill.

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
| M1 | Procedural fielder body and basic locomotion: ozz skeleton, IK, joint limits, stepping, walk, run, jump | in progress |
| M2 | Basic ball flight (drag), bounce, roll | not started |
| M3 | Fielding: chase, intercept, pick-up, throw, catch | not started |
| M4 | Wicketkeeper | not started |
| M5 | Bowling | not started |
| M6 | Batting | not started |
| M7 | Match rules | not started |
| M8 | Broadcast cameras | not started |
| M9 | Game UI (RmlUi) | not started |
| M10 | Mobile (Android, then iOS on a Mac) | not started |

Before M1, decide the player body mesh (MakeHuman CC0 or Blender → glTF).

## Known issues / pending
- **Namespace migration:** M0 code uses bare layer namespaces (`core`, `render`, …).
  The convention is now `cricket::<layer>`. This needs approval, as a mechanical rename.
- **filagui** is not shipped in the prebuilt Filament release, so a small custom ImGui
  backend is used (`src/render/filament/imgui_filament.cpp`). See DECISIONS.md.
- **IBL:** the skybox is a flat colour, with no specular reflection cubemap yet. This is
  fine for rough materials, and can be revisited if visuals need it.
- **Render uploads:** debug-line and ImGui uploads malloc a staging copy per frame. This
  is render-side, not sim; move to ring buffers if profiling shows a cost.
- **Unchecked by eye:** camera feel and vsync behaviour on the OpenGL backend.
