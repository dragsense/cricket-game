# Progress

## Current milestone: M5 Bowling — Arm Action — in progress (branch milestone-5)

Goal: a standing bowler (no run-up) delivers the ball. The bowling hand makes a full
circle around the shoulder (front, down, back, over the top), releases near the top on
the way forward along the arc's tangent at a fixed speed (config), then follows through
across the body; the elbow stays near-straight (Law 21, 15° limit). The user picks the
release point and the arc's tilt (as in M4) and clicks to bowl; the ball flies with our
physics. Reuses the M4 throw code (`motion::ThrowArc`, `makeSwingArc`, Fielder Throw
phase, `takeRelease` -> `SimWorld::launchBall`, reach chain). Out of scope: run-up,
seam/spin, batter, new tests or collisions not asked for. One step at a time; stop after
each.

- [x] **5.0 Research** (awaiting the user's review): docs/BIOMECHANICS.md section 11
  "Bowling arm action" (Law 21 elbow limit, arm circle, timing, release point and height,
  speeds, sources; planned `config/bowling.json` values).
- [x] **5.1 Arc** (built; awaiting the user's check): a bowler (a second `agents::Fielder`,
  `SimWorld::initBowler`) stands at the bowling crease (`config/bowling.json` bowler).
  B switches control between fielder and bowler (WASD moves, the camera follows). With
  the bowler: `motion::makeBowlingArc` draws a full circle around the bowling shoulder
  (radius `arc.radiusFactor` 0.97 x arm length; 0 top, +90 deg in front); the mouse picks
  the release point (red; default `aim.releaseDeg` 15); after a click on the circle the
  wheel leans it from vertical (`aim.tiltStepDeg`, -15..90 deg) instead of zooming; R or
  B gives the wheel back. The ball still stays with the fielder (until 5.3).
- [x] **5.2 Arm circle** (built; body mechanics need work, see below): a second click
  (`Fielder::startBowl`, phase Bowl) takes the hand round the full circle: eases to the
  front (`circle.startDeg` 135), down and back (`downBackTime` 0.5 s), over the top to the
  release point speeding up (`overTime` 0.18 s), then slows across the body
  (`followTime` 0.4 s, `followAcross` 0.6) and returns. Elbow bend held at
  `arm.elbowBendDeg` 10 (sets the radius; no IK softening while bowling). The bowler
  starts with the ball; the fielder's hand starts empty (T sends it to the fielder).
- [x] **5.3 Release** (built): the ball leaves the solved hand at the release point along
  the circle's tangent at `release.speed` (22 m/s) and flies with our physics; after
  `release.returnDelay` (3 s) it is back in the bowler's hand unless the fielder has it.
- [ ] **Next (the user's call): fix the body mechanics** of the bowling action before
  going on.

## Previous milestone: M4 Fielding — Throw — built, awaiting the user's visual review (branch milestone-4)

Goal: after a pick, the fielder throws. The hand follows an arc (circle around the
throwing shoulder); the user picks the release point on it (underarm … sidearm …
overarm), the hand winds back along the arc, swings forward through that point and
releases the ball tangent to the arc at a fixed speed (config) in the facing direction;
the ball then flies with our physics. Procedural only; the existing reach chain follows
the hand. Out of scope: chase, intercept, catch, aiming at targets, variable speed, new
tests or collisions not asked for. One step at a time; stop after each.

- [x] **4.0 Research** (awaiting the user's review): docs/BIOMECHANICS.md section 10
  "Throwing" (arm slots, phases and timing, arm path and release point, ball speeds,
  sources; planned `config/throw.json` values).
- [x] **4.1 Arc** (built; the user checked it): `motion::ThrowArc` (src/motion/throw_arc)
  is a circle around the holding shoulder, square to the facing direction (angle -90°
  underarm, 0° sidearm, +90° overarm), radius `config/throw.json` arc.radiusFactor (0.95)
  x arm length. While the ball is held, the mouse ray picks the nearest point on it; the
  circle (cyan) and point (magenta) are drawn. Angle not limited yet.
- [x] **4.2 Wind-up and swing** (built; the user checked it). Two half circles around the
  shoulder (the user's design): the slot arc (above) fixes a slot point on a first click;
  the swing arc passes through it, centred on the shoulder, and the mouse wheel turns it
  about the shoulder-to-slot line (`aim.tiltStepDeg`; the camera does not zoom
  meanwhile). The hand eases to the swing arc's start (behind, elbow bent:
  `swing.cockReach` 0.6), swings forward speeding up (`windupTime` 0.3 s, `swingTime`
  0.12 s) while the elbow extends, straight at the release point, follows through to the
  arc's end and returns; the body follows via the reach chain (`Fielder` Throw phase).
  R goes back to choosing the slot.
- [x] **4.3 Release** (built; the user checked it): the second click picks the release
  point on the swing arc; the ball leaves the solved hand there along the arc's tangent at
  `release.speed` (25 m/s) and flies with our physics (`Fielder::takeRelease` ->
  `SimWorld::launchBall`).

M4 code done (to improve later: aiming controls, a target, body sequencing).

## Earlier milestone: M3 Physics and Ball Physics — done to 3.3 (branch milestone-3)

Goal: general ball physics: gravity, air drag, bounce, rolling, hitting the stumps. Jolt
Physics (wrapped in src/physics/core) for stumps, bails and later bodies; our own ball
solver (src/physics/ball) with force generators (gravity, drag), shared by the
predictor. World settings in config/physics_world.json. Out of scope: spin, swing,
pitch-specific behaviour, players, bat, fielding, dive, game logic. One step at a time;
stop after each.

- [x] **3.0 Research** (awaiting the user's review) → docs/BALL_PHYSICS.md (ball size and
  mass, drag, bounce and friction on pitch and grass, rolling resistance, stumps, speeds,
  integrator notes, sources).
- [x] **3.1 Jolt setup** (built; tests pass): Jolt v5.3.0 pinned (`cmake/dependencies.cmake`,
  DLL CRT, no samples/tests), wrapped by `physics::PhysicsWorld` (src/physics/core; Jolt
  only in the .cpp), created from `config/physics_world.json` ("world": gravity, air
  density, wind, shared with the ball solver; "jolt": limits, temp allocator, threads,
  slop) and stepped by `match::SimWorld::tick` at 120 Hz; F5/live reload rebuilds it.
  Bodies keep previous/current transforms for interpolation. No bodies, ground or
  collisions yet (the user's choice: stumps and bails wait for 3.3). `physics_tests`:
  settings from config, free fall under the shared gravity, interpolation and reset.
- [x] **3.2 Ball flight, collision and bounce** (built; tests pass; awaiting the user's
  visual check). Scope set by the user: gravity, collision detection with simple boxes and
  spheres, bounce; no friction yet; everything generic, so other objects (e.g. players)
  can reuse it. Generic code in `src/physics/sim`: `BodyState`/`SphereBody`,
  `ForceGenerator` with `GravityForce` and `DragForce` (shared world settings),
  `BoxCollider`/`SphereCollider` with sphere contact tests in a fixed-capacity
  `ColliderSet`, and `SphereSolver` (sums generators, semi-implicit Euler in substeps of at
  most 0.5 radius of travel, so no tunnelling; normal restitution bounce, slow bounces
  absorbed so it settles). Ball-only setup in `src/physics/ball/ball_setup` (size/mass
  from scene.json, drag, solver and colliders from `config/ball.json`; the ground is one
  box, top at y = 0, restitution 0.45). `SimWorld` steps the ball each tick; rendering
  interpolates. Keys 1-5 launch the presets of `config/ball_launch.json` (fast and slow
  delivery, ground shot, lofted shot, high catch); a left click puts the ball down, T
  still throws with the feeder, Space can pick a flying ball. Without friction the ball
  slides on after its bounces (3.3). Tests (`test_sphere_solver.cpp`): free fall, drag
  (~13 % over 20 m, terminal 37 m/s), bounce apex e², comes to rest, no tunnelling through
  a 1 cm wall at 45 m/s, sphere and rotated box colliders, ball config.
  Follow-up (user's request): clicking to place the ball and E/Q are removed; **T**
  releases the ball with the physics from the batting position toward the fielder
  (`config/ball_launch.json` "toFielder"): random elevation (10-55°) and first landing
  (0.2-1.6 of the way), so throws vary between many bounces, one bounce, in the air and
  over the fielder; Space picks it. The straight-line `BallFeeder` is no longer used by
  the game (kept for its fielder test).
- [x] **3.3 Ball contacts** (built; tests pass; awaiting the user's visual check). Generic
  solver: each collider has a surface (restitution, sliding friction, rolling resistance);
  grip-slip Coulomb friction spins the body up until it rolls (5/7 of the speed for a
  solid sphere), rolling resistance (C_rr x normal gravity) slows it and it stops below
  `restSpeed`; the body has spin and orientation (the ball visibly rolls). `config/ball.json`:
  pitch box (e 0.5, mu 0.3, C_rr 0.05) listed before the outfield box (e 0.38, mu 0.5,
  C_rr 0.15). Stumps and bails are Jolt bodies (`match::Wickets`, sizes from scene.json,
  masses in physics_world.json "wicket", Jolt ground restored) that stand on the ground;
  their visuals follow the bodies. No ball-stump collision and no stump tests yet (the
  user's choice: later). Tests: slide to roll at 5/7, roll distance v^2/(2 C_rr g) and
  stop. Known issue (later): untouched bails/stumps creep up to ~1 cm (round bails roll on
  flat stump tops; real ones sit in grooves).
- [-] **3.4 Prediction and boundary**: moved out of M3 (user's choice); later.

M3 code done (user's choice: only up to 3.3); next is M4 on a new branch.

## Previous milestone: M2 Hand-Driven Body Repositioning (Reaching) — built, awaiting the user's visual review (not finished)

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
- [x] **2.3 Pelvis rotation and tilt** (built; tests pass; awaiting the user's visual
  check): `reach()` is split around the leg IK: `reachBody()` (spine, then pelvis)
  runs before it so the feet stay planted; `reachArm()` (clavicle, arm) runs last.
  The pelvis swings about the hip centre for the spine's leftover, clamped per axis by
  `motion::clampRotationVector` to `reach.pelvisMinDeg/MaxDeg` [-10, -30, -20] /
  [80, 30, 20] (forward tilt = hip flexion, straight knees). Test: a ground ball
  0.5 m ahead is now reached with straight legs (pelvis ~64° forward), feet unmoved.
- [x] **2.4 Body repositioning toward the hand** (reworked 2026-10-25 after the user's
  visual check; tests pass; awaiting a new visual check). The first version lay the body
  flat (no balance), missed balls (the arm was clamped after the IK and the error was
  measured before it) and used retry guards. Now one formula pass per stage, like
  `repositionBody`: spine and pelvis swings (clamped to limits); body += dir × leftover;
  hips shift back against the centre of mass's overhang beyond the feet (gain, max
  0.3 m); `motion::clampToLegs` (the leg clamp factored out of `repositionBody`, no squat
  limit) keeps both feet planted; after the leg IK, `rebalance()` measures the real
  centre of mass (segment masses, plus the reaching arm) and moves the pelvis to cancel
  any overhang (`reach.balancePasses` 2; a deep squat swings the knees forward, which no
  estimate sees). Knees follow the pelvis turn; the elbow pole is made perpendicular to
  the shoulder-to-target line (`reach.elbowPole`), so the elbow bends away from the body
  for ground, side and air targets. The wrist goal sits back from the ball toward the
  shoulder (`reach.wristOffset`). `reachError()` is measured after the joint limits.
  Ball: hold the left button to drag it at its height, E/Q raise/lower it, R clears the
  reach. Tests: all 27 ground spots in front of and beside the feet (≤ 0.6 m) reached;
  balls behind the heels or beyond need 2.5; far balls stay balanced.
- [x] **2.5 Pick, lunge, grip, hand modes** (built; tests pass; awaiting the user's visual
  check; also covers 2.6, which the user folded in): placing the ball (left-drag, E/Q
  height) no longer reaches; **Space** picks: the wrist goal eases (smoothstep,
  `reach.reachTime`) from its rest spot to the ball, every stage follows the goal, so the
  hand leads and the body follows; on the ball for `holdTime` the fingers and thumb
  curl (grip grows within `gripRadius`) and the ball is carried back (`returnTime`) to
  the standing pose; the ball rides in the hand until moved again. Out of reach at full
  stretch: the foot nearer the ball lunges to land beside it (`lungeShort` short, at
  most `maxLunge`; BIOMECHANICS 8) via the new `FootStepper::stepTo`; still out of reach
  after `giveUpTime`, it lets go and logs the dive case. Back to standing, the gait's
  settle steps the lunged foot home. **H** cycles the hand: auto (nearest; unchanged),
  left, right, both (a hand each side of the ball). **R** abandons a pick. Walk input
  waits during a pick. Tests: pick + carry + stand again, lunge pick of a 1.2 m ball,
  right-hand mode, `stepTo`.
  Follow-up (user's review): the fielder starts with the ball in the right hand (carried
  while walking/running); a left click takes it from the hand and puts it down there;
  E/Q ignored while held. Fingers and thumb close fully once the hand is on the ball and
  stay closed while held, relaxed (`relaxedCurl`) otherwise; the thumb now flexes toward
  the palm about Z like the fingers (thumb limits moved to Z). While reaching, the hand
  turns so the palm faces the ball; a held ball sits in front of the palm
  (`palmCentre`, `palmThickness`), not inside the fingers.
  Second follow-up: finger and thumb curl axes come from the rest geometry (each finger
  bone turns toward the palm, each thumb bone toward the held ball), so the thumb now
  closes on the ball (thumb limits allow its twist). Step first: a ball further than
  `reach.stepBeyond` (0.65 m) from the middle of the feet gets the lunge before the
  reach; while that foot is in the air the body follows the feet through
  `repositionBody`, and the reach starts once it lands (the at-full-stretch lunge stays
  as a fallback).
  Test controls (user's request): 1/2/3 pick with the left/right/both hands (default
  right; H still cycles, auto kept for later). **T** throws the ball at the fielder
  (`game::BallFeeder`, config/engine.json "feeder"): straight line, no physics, rolling
  or at a random height, random speed and side offset, passing beside them; Space in
  time picks it, otherwise it stops behind them; T again throws another. A click
  stops a throw (manual placement). Test: `test_ball_feeder.cpp`.
  Human timing (user's request): reach and return take Fitts' law time for the hand's
  travel (`motion::movementTime`, BIOMECHANICS 9.6). Space no longer acts at once: a
  new Wait phase finds the meeting point (`motion::timeToClosest`: where the ball's
  path passes closest to the feet; where a still ball lies). Beyond `reach.pickRange`
  (1.5 m) it is not tried; a still ball in range is reached at once; a moving one is
  waited for, the reach (plus a lunge if needed, stepped first to the meeting point)
  starting so the hand arrives as it passes; a moving ball is grabbed on contact.
  Already past: tried at once if in range. No dive. Tests: Fitts, timeToClosest, out of
  range, still ball sooner when nearer, moving ball waited for (~2.5 s) then caught.
  Simplified (user's request): two-hand picking and the hand keys (H, 1/2/3) are
  removed; the hand on the ball's side picks (auto). Left/Right stay in the API.
- [-] **2.6** folded into 2.5 (user's choice): timing via the eased goal, balance from 2.4.

## Earlier milestone: M1 Procedural Fielder Body and Basic Locomotion — DONE (2026-10-25), awaiting the user's visual review

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
| M4 | Fielding: throw (chase, intercept, catch later) | in progress |
| M5 | Bowling: arm action (standing bowler) | in progress |
| M6 | Wicketkeeper | not started |
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
