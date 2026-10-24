# CLAUDE.md

## Project
Procedural cricket game in C++20, with no game engine. PC (Windows) first, later Android
and iOS. Gameplay characters move procedurally by default: IK, joint limits, balance and
physics-based motion. Clips are allowed only for the umpire, crowd, celebrations and
cutscenes. The goal is real TV-broadcast camera work and action, not photoreal humans.

Docs: [DESIGN.md](DESIGN.md) (architecture), [PROGRESS.md](PROGRESS.md) (milestone and
status), [DECISIONS.md](DECISIONS.md) (decision log).

## Stack (fixed; do not substitute)
- SDL3
- Filament: scene, camutils, gltfio, filagui (not in the prebuilt release, see
  DECISIONS.md)
- GLM, EnTT, nlohmann/json, spdlog, Dear ImGui (docking), doctest
- Planned: ozz-animation and ImGuizmo (M1), Jolt Physics (later), RmlUi (game menus,
  later)
- All versions are pinned in `cmake/dependencies.cmake`.

## Library-first rule
- Never write code that a popular, maintained, permissively licensed library already
  provides. Use the library.
- Our own code is only for cricket-specific logic (procedural motion, ball physics,
  fielding AI, match rules, broadcast cameras) and thin glue.
- Adding a library not in the stack requires asking first: use the `add-library` skill.

## Architecture rules
- Layers: core → (math) → rig/ik → motion → agents → ai → match. Each layer depends only
  on layers below it.
- The simulation runs on a 120 Hz fixed timestep, and rendering is interpolated.
- Game code never calls Filament directly; it uses the `src/render` interface.
- Tunables live in `config/*.json`, never hardcoded.
- Platform code goes only in `src/platform`.
- No per-frame heap allocations in the simulation. Keep mobile performance in mind
  always.
- Everything is created from code. No editor, no scene files.

## Rules for EVERY prompt
1. Before writing code, read PROGRESS.md and the relevant DESIGN.md sections.
2. Before creating any function, class or file, search the codebase for existing code
   that does the same thing. Reuse or extend it; never duplicate.
3. Stay inside the current milestone's scope (see PROGRESS.md). If a request goes beyond
   it, say so and ask before doing it.
4. Follow existing naming and style. If a decision isn't covered here, check
   DECISIONS.md. If it isn't there either, ask.
5. After changes: build, run the tests, and fix all errors and warnings.
6. Update PROGRESS.md, and DECISIONS.md if a decision was made.
7. Commit to Git with a clear message after each working step.
8. End with a short summary: what changed, what's next.

## Conventions
- **Files:** `snake_case.h` / `.cpp`, one main type per pair, in its layer folder
  (`src/<layer>/`). Filament-specific code goes in `src/render/filament/`.
- **Names:** types are `PascalCase`; functions and variables are `camelCase`; members
  have a trailing underscore (`count_`); constants are `kPascalCase`; enums are
  `enum class` with `PascalCase` values.
- **Namespaces:** top level is `cricket`, with one sub-namespace per layer
  (`cricket::core`, `cricket::render`, …). The M0 code still uses bare layer namespaces;
  the migration is pending in PROGRESS.md.
- **Headers:** `#pragma once`, then a short comment on what the file is for, then
  includes in this order: own project, third-party, std. Forward-declare where possible.
  Third-party headers stay out of public headers when a forward declaration works.
- **Errors:** no exceptions in game code. Init functions return `bool` and log the cause.
  Invalid handles are sentinel values (e.g. `kInvalidObject`). Use `assert` only for
  programmer errors.
- **Logging:** spdlog through `LOG_INFO` / `LOG_WARN` / `LOG_ERROR` (`core/log.h`),
  with std::format syntax. Never log per tick in the simulation.
- **Units:** 1 unit = 1 metre, seconds, radians internally (degrees only in config).
  Right-handed, +Y up. Character forward is local +Z; cameras look down -Z.
- **Math:** use the `math::` aliases and helpers (`src/math`), never `glm::` directly
  outside `src/math`. Note that `math::Quat` is constructed as (w, x, y, z).

## Never
- Prebuilt animations for gameplay characters.
- Rewriting working systems without being asked.
- Changing the stack, layers or folder layout without asking.
- Downloading anything not in the stack without asking.

## Build
- `powershell -ExecutionPolicy Bypass -File scripts\build.ps1`: debug build, then tests,
  then run.
- Options: `-Config release`, `-NoRun`, `-NoTests`.
- First-time setup: `scripts\setup_windows.ps1`, which downloads Filament (~850 MB).
