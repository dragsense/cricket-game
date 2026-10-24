# CLAUDE.md

Procedural 3D cricket game on a thin custom engine. Read DESIGN.md for the architecture.

## Approach: libraries first
- Never write what a popular, maintained, permissively licensed (MIT/BSD/Apache/zlib),
  cross-platform (Windows/Android/iOS) library already does. Use the library.
- Write our own code only where libraries stop: procedural motion, ball physics,
  fielding AI, match rules, game cameras, and thin glue between libraries.
- IK and constraints follow standard published algorithms (two-bone from ozz, FABRIK,
  swing-twist limits), not invented approaches.
- Approved: SDL3, Filament (+camutils, gltfio), GLM, EnTT, nlohmann/json, spdlog,
  Dear ImGui (docking), doctest. Planned: ozz-animation + ImGuizmo (M1), Jolt (later).
  Ask the user before adding any other library, and explain why.
- Pin every version in `cmake/dependencies.cmake`.

## Rules
- Layers: core → math → rig/ik → motion → agents → ai → match; only depend downward.
- Sim runs on a 120 Hz fixed step and never calls the renderer. Render interpolates.
- Game code uses `src/render`, never Filament headers. SDL only in `src/platform`.
- Tunables live in `config/*.json`. No per-tick heap allocations in the sim.
- Commit after every working step.
- Build ONLY the current milestone, then stop and report.

## Build
- `powershell -ExecutionPolicy Bypass -File scripts\build.ps1` (debug: build, test, run).
  Add `-Config release`, `-NoRun` or `-NoTests` as needed.
- First-time setup: `scripts\setup_windows.ps1` (downloads Filament, ~850 MB).
- After a build, fix all warnings in our own code (`/W4`).
