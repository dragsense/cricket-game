# Decisions

Dated log. Format: decision, reason, alternatives rejected.

## 2026-10-24: Custom stack instead of an off-the-shelf engine
- **Decision:** build on libraries (SDL3, Filament, GLM, EnTTâ€¦) with no game engine.
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
## 2026-10-24: Fielder skeleton built at runtime from height ratios (M1)
- **Decision:** ozz-animation 0.16.0 (runtime plus offline `RawSkeleton` and
  `SkeletonBuilder`) builds the skeleton from `config/fielder_skeleton.json`. The config
  stores ratios of height, not metres. All rest rotations are identity, so the rest pose
  is the standing pose, and the joint axes match the character axes.
- **Reason:** one height value rescales the body. Identical joint frames keep joint
  limits and IK axes simple and uniform.
- **Rejected:**
  - Loading the skeleton from glTF: there is no body mesh yet; it comes later.
  - Bone-aligned frames (+Y along the bone): different per-bone limit axes for no gain
    now.

## 2026-10-24: Milestone order: ball physics before fielding
- **Decision:** M1 rig and IK, M2 basic ball flight and bounce, M3 fielding.
- **Reason:** fielders need a ball to chase and catch.

## 2026-10-24: Orbit is the only user-controlled camera
- **Decision:** the free-fly camera, its keys (WASD/QE/Shift, `C`) and its Debug window
  settings are removed. The user controls only the orbit camera; WASD steps the
  fielder.
- **Reason:** keys stay free for gameplay. Moving cameras in cricket follow defined shots
  (show a player, sweep the ground), so they come as scripted paths with the broadcast
  cameras (M8), not as a manual fly camera.
- **Rejected:** keeping free-fly on separate keys: it clashed with gameplay input and
  isn't needed to check the fielder.

## 2026-10-25: Paces walk / jog / run with a speed-blended gait shape (M1)
- **Decision:** three presets in `config/locomotion.json` ("jog" inherits "walk", "run"
  inherits "jog"). The input picks the pace (walk by default, Ctrl jog, Shift run),
  which sets the target speed. The gait shape is blended between presets by the actual
  speed. Sidesteps and backing up cap at the jog.
- **Reason:** follows docs/BIOMECHANICS.md section 5: no mode switch, so nothing snaps.
  Real players backpedal and shuffle; they do not sprint backwards or sideways.
- **Rejected:**
  - A Debug-window run checkbox (removed).
  - Double-tap Shift to run (the user preferred Ctrl jog / Shift run).

## 2026-10-25: Mouse-look orbit camera (M1)
- **Decision:** right-drag rotates the orbit camera by default. Tab toggles mouse-look,
  where the mouse turns the camera without a button (SDL relative mouse mode, cursor
  hidden).
- **Reason:** the user asked for third-person-style camera control while moving the
  fielder.

## 2026-10-25: Jump dropped from M1
- **Decision:** Step 1.7 (jump) is removed from M1. Jumps return with the fielding
  actions that need them (M3).
- **Reason:** the user's choice.

## 2026-10-25: Run review fixes before M2
- **Decision:** keep the kinematic approach (foot placement + IK + gait-phase pelvis
  motion); fix bob phase, cadence, stance-leg compression, flight-phase back foot, pelvis
  following and swing smoothness rather than switching to a physics (spring-mass) body.
- **Reason:** the research review found the approach matches standard procedural
  locomotion; the problems were tuning and phase bugs. A physics body waits for Jolt.
- **Also:** W uses the camera heading only while the fielder is stopped (user's choice,
  so the camera can orbit a running fielder).

## 2026-10-25: M2 becomes hand-driven reaching; roadmap shifted
- **Decision:** M2 is hand-driven reaching and body repositioning. Ball physics moves to
  M3, fielding to M4 (later milestones shift by one). The dive moves to M3 because it
  needs the ball's arrival time.
- **Reach mechanism:** solved continuously like `repositionBody`; recruitment arm →
  clavicle → spine → pelvis → body translation → one lunge step; beyond that the body
  holds maximum reach and keeps the leftover error (dive trigger later). No squat/drop
  limit at first, to observe behaviour. Hand and foot body offsets are summed, never
  overriding each other. Target placed by mouse click on the ground; one hand (nearest
  automatic) or both. Standing only.
- **Reason:** the user's choice; fielding needs reaching before ball interaction.
- **Rejected:** multi-step approach to the target (that is M4 fielding); logging only
  when out of reach.
