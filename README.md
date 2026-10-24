# Cricket Game

A 3D cricket game with fully procedural character motion, built on a small custom engine
(SDL3 + Filament + GLM + Dear ImGui + EnTT). Windows first; Android and iOS later from the
same codebase. See [DESIGN.md](DESIGN.md) for the architecture.

**Current state: Milestone 0** — window, Filament PBR rendering of a cricket field at real
scale, sun shadows + sky lighting, orbit and free-fly debug cameras, debug line renderer,
ImGui stats overlay, fixed-timestep simulation loop, math wrapper with unit tests.

## 1. Install the tools (once)

You need Windows 10/11 x64 and these tools. From a PowerShell window:

```powershell
winget install Git.Git
winget install Kitware.CMake          # 3.28 or newer
winget install Ninja-build.Ninja
winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```

The last one installs the MSVC C++ compiler and the Windows SDK (about 6 GB). If you
prefer the GUI: download *Build Tools for Visual Studio 2022* from
<https://visualstudio.microsoft.com/visual-cpp-build-tools/> and tick
**Desktop development with C++**.

Close and reopen PowerShell afterwards so `PATH` picks up the new tools.

## 2. Set up (once)

```powershell
cd <repo>
powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1
```

This checks the tools above (and prints install commands for anything missing), downloads
the pinned Filament release (~850 MB) into `third_party/filament`, and configures the debug
build. The first configure also clones SDL3, GLM, ImGui, nlohmann/json, spdlog, EnTT and doctest once into `third_party/deps/` (shared by debug and release; never re-downloaded) (pinned in [cmake/dependencies.cmake](cmake/dependencies.cmake)).

## 3. Build and run

Easiest: from the project folder, run one of these (or double-click `run.cmd`).
In PowerShell (the VS Code terminal) type the `.\` prefix; in cmd, plain `run` works.

```
.\run            debug build + tests + play
.\run fast       debug build + play (skip tests)
.\run release    optimized build + tests + play
```

Only changed files are recompiled. If the game is already open, it is closed
automatically so the new build can replace it. You can edit code while the game runs.

The underlying script (more options):

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build.ps1                    # debug: build, test, run
powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Config release    # optimized
powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -NoRun             # build + tests only
```

`build.ps1` loads the MSVC environment itself, so a normal PowerShell window works (no
"Developer Command Prompt" needed). The game ends up at `build/<config>/bin/CricketGame.exe`,
with `config/` and the compiled `assets/` next to it.

From an MSVC developer prompt you can also use the presets directly:

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

## Controls

| Input | Action |
|---|---|
| `C` | switch between orbit and free-fly camera |
| Orbit: right-drag / middle-drag / wheel | rotate / pan / zoom |
| Free-fly: hold right mouse | look around |
| Free-fly: `W A S D`, `Q`/`E` | move, down/up |
| Free-fly: `Shift` / wheel | fast / change base speed |
| `F1` | toggle the debug overlay |
| `F2` | toggle debug drawing |
| `Esc` | quit |

Key bindings live in [config/input.json](config/input.json); engine, lighting and camera
tunables in [config/engine.json](config/engine.json); field and pitch dimensions in
[config/scene.json](config/scene.json). They are copied next to the exe on every build.

## Layout

```
cmake/            dependency pins + Filament import
config/           JSON tunables
assets/materials/ Filament materials (.mat, compiled by matc at build time)
scripts/          setup_windows.ps1, build.ps1
src/              engine + game code, one folder per layer (see DESIGN.md)
tests/            doctest unit tests
third_party/      Filament SDK (downloaded, git-ignored)
```
