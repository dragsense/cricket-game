# Decisions

Dated log. Format: decision, reason, alternatives rejected.

## 2026-10-24: Custom stack instead of an off-the-shelf engine
- **Decision:** build on libraries (SDL3, Filament, GLM, EnTT…) with no game engine.
- **Reason:** full control over procedural motion and the sim loop. The custom code stays
  small and cricket-specific. The stack is cross-platform for mobile.
- **Rejected:**
  - Unity and Unreal: heavy, licensing terms, and their animation systems are built
    around clips.
  - Godot: less control over low-level animation and the loop.
  - Bevy: Rust ecosystem and an immature mobile story.

## 2026-10-24: Libraries first
- **Decision:** use a popular, maintained, permissive (MIT/BSD/Apache/zlib),
  cross-platform library for anything general-purpose. Our own code only for cricket
  logic and glue.
- **Reason:** less code to maintain, and fewer subtle bugs (e.g. in quaternion math that
  IK depends on).
- **Rejected:** writing our own math, JSON, logging and camera controllers. These were
  briefly drafted in M0, then replaced.

## 2026-10-24: Procedural gameplay motion, with optional clips
- **Decision:** fielders, keeper, bowler and batter are procedural (IK, joint limits,
  balance, physics). Clips only for the umpire, crowd, celebrations and cutscenes,
  blendable with IK on the same skeleton.
- **Reason:** cricket actions vary continuously (ball position, timing). Procedural
  motion adapts where clips can't.
- **Rejected:** mocap or keyframe clips for gameplay characters.

## 2026-10-24: Broadcast camera realism over photorealism
- **Decision:** aim for TV-broadcast camera work and believable action, not photoreal
  humans.
- **Reason:** that is what sells cricket visually, and it is achievable for a small
  team.
- **Rejected:** a photoreal character pipeline (scanning, hair, skin shading).

## 2026-10-24: GLM instead of our own math
- **Decision:** GLM, wrapped by `src/math` (aliases plus helpers: lookRotation, fromTo,
  swingTwist, Transform).
- **Reason:** widely used and well tested. Bugs in our own quaternion code would
  silently break IK.
- **Rejected:** an in-house math library.

## 2026-10-24: Filament as renderer, behind our own interface
- **Decision:** Google Filament (prebuilt release) for PBR and its scene, transform and
  light system. Game code uses only `render::Renderer`.
- **Reason:** mobile-first PBR on Vulkan, OpenGL and Metal. The interface keeps it
  replaceable.
- **Rejected:** bgfx (lower level, so more to write), and a custom renderer.

## 2026-10-24: Debug cameras from Filament camutils
- **Decision:** `camutils::Manipulator` provides orbit and free-flight. Shift-for-speed
  uses a second manipulator, with the pose carried across by bookmarks, because camutils
  has no speed key.
- **Rejected:** hand-written camera controllers.

## 2026-10-24: Custom ImGui backend instead of filagui (for now)
- **Decision:** a small Filament ImGui renderer, `render/filament/imgui_filament.cpp`.
- **Reason:** filagui is not shipped in the prebuilt Filament Windows release (checked
  in v1.77.2 `lib/`). It also vendors its own older ImGui, which conflicts with the
  docking branch.
- **Rejected:**
  - Building filagui from Filament source: needs the full Filament build, plus the
    ImGui version clash.
- **Revisit:** if Filament ships filagui, or builds from source.

## 2026-10-24: Game objects in EnTT, linked to Filament entities
- **Decision:** an EnTT registry in `game::World`. `Renderable` holds a
  `render::ObjectId`, which maps to Filament entities. Local transforms are pushed to
  Filament's TransformManager.
- **Reason:** an ECS for game state, plus Filament's native hierarchy, culling and
  lights, with no Filament types in game code.

## 2026-10-24: OpenGL as the default Filament backend on Windows
- **Decision:** `render.backend = "opengl"` by default; Vulkan can be selected in config.
- **Reason:** widest driver compatibility for development. Vulkan is the target on
  Android.

## 2026-10-24: Procedural ball texture in a shader
- **Decision:** the ball's leather, seam stitching and quarter seam are generated in
  `assets/materials/ball.mat` from the object-space direction. There are no image files
  and no UVs.
- **Reason:** "everything created from code". It stays sharp at any distance and is
  easily tuned via config (colours, clear coat).
- **Rejected:** an image texture with sphere UVs, which needs an asset pipeline and
  shows seams and pole distortion.

## 2026-10-24: Dependencies cloned once into third_party/deps
- **Decision:** `cmake/dependencies.cmake` clones each library once, with git, into
  `third_party/deps/<name>-<version>/`. FetchContent is pointed at that folder through
  `FETCHCONTENT_SOURCE_DIR_*`, so it never downloads anything itself.
- **Reason:** FetchContent re-cloned dependencies whenever its hidden download
  sub-project changed (e.g. a different git on PATH in another terminal), causing slow
  downloads and full rebuilds. Now debug and release share one copy, and deleting
  `build/` never re-downloads.
- **Rejected:**
  - `FETCHCONTENT_UPDATES_DISCONNECTED`: did not prevent the re-clones.
  - Archive URLs: re-extraction still touches timestamps.
## 2026-10-24: Milestone order: ball physics before fielding
- **Decision:** M1 rig and IK, M2 basic ball flight and bounce, M3 fielding.
- **Reason:** fielders need a ball to chase and catch.
