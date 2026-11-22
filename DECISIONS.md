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
  `SkeletonBuilder`) builds the skeleton from `config/human_skeleton.json`. The config
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

## 2026-10-25: Reach solve is one formula pass per stage (no retries)
- **Decision:** the reach solve mirrors `repositionBody`: each stage (spine, pelvis,
  body, clavicle, arm) runs once with a closed-form step (law-of-cosines swing, clamp,
  body += dir × error, leg clamp, centre-of-mass overhang cancel); only fixed iteration
  counts, as in the reposition leg clamp. Balance is measured after the leg IK.
- **Reason:** the user's direction ("use biomechanics and math properly, fewer checks,
  formulas like reposition"). The earlier guards (bisection, retries, special cases)
  fixed one case and broke another.
- **Rejected:** bisection guards on feet/balance, iterative hip-sliding helpers,
  knees-out pole hack.

## 2026-10-26: M3 physics architecture
- **Decision:** Jolt Physics handles all generic physics (gravity, rigid bodies,
  collisions, materials): stumps, bails, later player bodies and dives. It is wrapped in
  `src/physics/core` behind our own thin interface. The ball uses our own solver in
  `src/physics/ball`, built from reusable force generators (gravity, drag; spin and swing
  added later as new generators without changing the solver). The trajectory predictor
  calls the exact same step code. Gravity, air density and wind live in
  `config/physics_world.json`, shared by Jolt and the ball solver. Values come from
  docs/BALL_PHYSICS.md.
- **Reason:** the user's direction; prediction must match the simulation exactly, and
  cricket-specific forces need to plug into the ball code.
- **Also:** M3 starts on branch `milestone-3` while M2 still waits for the user's visual
  review. The dive leaves M3 (no players in this milestone) and will be scheduled later.

## 2026-10-26: Jolt Physics v5.3.0 added (M3 Step 3.1)
- **Decision:** Jolt Physics v5.3.0 (MIT), cloned through `cricket_dependency` (CMake in
  `Build/`), DLL CRT, no IPO, samples and tests off. Linked only to `cricket_physics`.
  Default job system is single-threaded (tiny scene, mobile); `jolt.threads` enables the
  thread pool.
- **Reason:** the user's M3 brief named it; it was already the planned physics library
  (popular, maintained, Windows/Android/iOS, deterministic option).
- **Rejected:** Bullet (older API, heavier), PhysX (larger, more complex licence/build),
  writing rigid bodies ourselves (library-first rule).
- **Also:** Step 3.1 has no bodies or collisions (the user's choice); stumps and bails
  as Jolt bodies move to 3.3 with ball contacts.

## 2026-10-26: Generic sphere physics; Step 3.2 scope
- **Decision:** forces, colliders, contact tests and the substepped solver live in
  `src/physics/sim` as generic code (`SphereSolver`, `ColliderSet`, `ForceGenerator`), not
  tied to the ball, so other objects (e.g. players) can reuse them. `src/physics/ball` only
  holds the ball's configuration. Step 3.2 covers gravity, drag, collision detection
  (boxes, spheres; the ground is a box) and restitution bounce; no friction until 3.3.
- **Reason:** the user's direction.
- **Rejected:** Jolt for the ball (prediction must run the same code); a separate ground
  plane type (a large box covers it).

## 2026-10-26: M3 ends at 3.3; no ball-stump contact yet
- **Decision:** M3 stops after 3.3 (friction, rolling, stopping; stumps and bails as
  standing Jolt bodies). The ball does not collide with the stumps yet, and there are no
  stump tests; 3.4 (prediction, boundary) moves out of M3. Next: M4 on a new branch.
- **Reason:** the user's choice; details later.

## 2026-10-26: M5 is Bowling (arm action); Wicketkeeper moves to M6
- **Decision:** swap M5 and M6 in the roadmap. M5 is the bowling arm action with a
  standing bowler (no run-up), reusing the M4 throw arc and release code; the
  wicketkeeper becomes M6.
- **Reason:** the user's choice; bowling reuses the M4 arc work directly.

## 2026-10-26: M5 standing-delivery estimates
- **Decision:** keep the research estimates for the standing bowler: release speed
  ~22 m/s (`config/bowling.json` release.speed), arm circle ~0.5 s down and back,
  ~0.18 s over the top, ~0.4 s follow-through, release ~5–25° before vertical.
- **Reason:** no measured data for a standing delivery (docs/BIOMECHANICS.md section 11);
  35–45 m/s needs a run-up. The user left the choice to me; all values stay tunable.

## 2026-10-26: Controls are body-level intent, never "classic" aiming
- **Decision:** the player controls what a real player decides with their body (where the
  hand is, how far the trunk winds up, where the ball leaves the hand) through on-body
  circles and clicks; the procedural body (reach chain, trunk twist, stepper, IK, joint
  limits) does the movement, and the ball's direction, height and speed come out of that
  geometry and physics. No target markers, predicted flight lines, power bars or solved
  "hit the target" speeds.
- **Reason:** the user's design goal for the whole game: nothing classic, everything from
  real mechanics. A target + prediction line was built and rejected; a free mouse hand was
  rejected (no control over the angle).
- **Rejected:** target/prediction aiming, mouse-as-hand, pull/flick power gestures.

## 2026-10-26: The throw is driven by the trunk on three circles
- **Decision:** height circle around the chest (slot point, hand goes there); throw circle
  around the shoulder (full arm length, through the slot direction and the facing
  direction, wheel-tilted) for the force point (trunk wind-up, the hand riding the trunk)
  and the release point (hand blends out onto the circle, ball along its tangent, speed
  from the wind-up); stride by force. Lives in `Fielder` (Throw phase) and
  `config/throw.json`.
- **Reason:** the user's design, iterated live; trunk rotation is where real throwing
  speed comes from (BIOMECHANICS 10.2).


## 2026-10-26: M6 is the bowling rework; Wicketkeeper moves to M7
- **Decision:** M6 (branch `milestone-6`) reworks the bowler's delivery as player-driven
  body intent on circles, like the throw. The wicketkeeper becomes M7; later milestones
  shift by one.
- **Reason:** the user's choice.

## 2026-11-02: The standing body comes from one solve of real constraints, our own code
- **Decision:** in the limb test, body and legs come from one solve
  (`motion::StanceSolver`, damped least squares over the leg and pelvis joints): foot
  contact, joint limits, balance, reach and comfort together (ground contact later, with
  collisions and gravity), with no patch-up
  conditions after the IK. Walking, running and picking keep their own code for now.
- **Reason:** the user's call. Step-by-step fixes kept breaking each other. No whole-body
  IK library (PlaCo and Pinocchio were looked at), although the library-first rule would
  prefer one; the solver is small.

## 2026-11-03: The body follows its targets in real body order, no optimiser
- **Decision:** `motion::BodyReposition` (was StanceSolver) is a simple pass, not an
  optimiser: the pelvis starts over the root; each part reaches with its own joints first
  (two-bone IK, knee forward, elbow back and out) and the next part only takes what that
  left inside its joint limits (feet: hip range, then pelvis tilt over the standing hip,
  then pelvis moves; hands: arm, clavicle, chest, spine_02, spine_01, pelvis, body). Only
  feet on the ground hold the body. Design target: seven targets (hips, chest, head, two
  feet, two hands) plus IK hints; feet and hands done.
- **Why:** the user's rule: real body constraints only, no checks or tuned weights. The
  damped-least-squares solver needed patches (balance switch, upright and knee-hint
  weights, knee-as-length steps) and still leaned sideways and bent knees wrongly.
- **Also:** the foot stepper only moves targets along paths (weight shares, standing heel
  rise, no-cross rule and run flight carry removed). Running: no jog pace; Shift runs,
  Shift + mouse wheel sets the pace (run.minSpeed .. run.speed); in flight the standing
  foot starts its next swing early (each swing lasts swing + flight); `landAhead` (walk
  0.5, run 0.35). Walk hands swing opposite the other leg's thigh angle (`armLegRatio`).
- **Deferred to gravity/forces (later milestone, Jolt ragdoll recommended):** loaded knee
  bend, pelvis sway, dip and turn, run lowest at mid-stance.
- **Kept:** `body_reposition_backup.*` (formula version, still used by pick, throw, bowl,
  jump) and `body_reposition_lm_backup.*.txt` (the optimiser, not compiled).

## 2026-11-04: Body physics through Jolt (one place for gravity, mass, forces)
- **Decision:** the body gets mass, gravity and leg forces from `physics::PhysicsWorld`
  (Jolt), the same place every other object uses (bat and others later). The body's mass
  is one Jolt dynamic body at the centre of mass (skeleton masses, rotation locked); each
  foot on the ground pushes it through a Jolt distance-constraint spring (push only, the
  spring-mass model); `BodyReposition` fits the joints to where Jolt puts the body. First
  vertical only; forward forces and push-off later. Stumps and bails removed first.
- **Reason:** the user's rules: gravity/forces/mass only from one place, library first,
  nothing invented; the body without mass snaps at landing; running needs the spring leg.

## 2026-11-05: Generic joints first; our ball solver removed
- **Decision:** generic object physics comes first in `physics::PhysicsWorld` (Jolt):
  capsule shapes, hinge and swing-twist joints (limits), joint motors (frequency,
  damping, torque limit), tested on simple objects. The human body is built from these
  later (one shape per bone, skeleton masses, joint limits from human_joints.json, the
  same ranges BodyReposition uses). Our own ball solver (`src/physics/sim`,
  `src/physics/ball`, config/ball.json) is removed: the ball, bat and stumps become
  generic Jolt bodies later. Replaces the ball part of the 2026-10-26 M3 decision.
- **Reason:** the user's call: general things first, one physics for every object.
- **Also:** until then a released ball does not fly (SimWorld keeps the ball interface).
  The spring-leg stand-in for muscles was removed the same day (no jugaad).

## 2026-11-06: The body moves by physics only; one decider, one balancer
- **Decision:** while a player's physics is on, every part moves only by Jolt muscle
  forces; joint limits apply in Jolt.
  - `BodyReposition` becomes a decider (new file next to it; the old one is deleted
    once it works). The backups were deleted the same day (the user's call): the
    formula repositioning that jog, run, jump, pick, throw and bowl still use moved
    into `agents::Human` unchanged (its test went with it); those actions move to the
    decider later, in M6. Balance is part of M6 (the user's call). It
    keeps today's order and direction math, outputs a fixed-size queue (part,
    direction, order) from the real physics pose, and decides no amounts, applies no
    limits, moves nothing. Speed comes from the target's own motion and timing.
  - Removed from it: limit clamps, pelvis min/max, writing the pose, `bodyDrop`,
    `keepLeg`, the geometric foot roll, passes, `legReach` / `armReach`. Push-off stays
    as a decision (a muscle action).
  - Balance is real human balance (Horak & Nashner 1986; Winter 1995; Maki & McIlroy
    1997; Hof 2008; Peterka 2002): capture point with ankle, hip (blended) and step,
    planned ahead from the known steps and target paths (anticipation; Kajita 2003,
    Englsberger 2015), feedback for surprises. It uses only the free parts and
    **always wins**: the active target gives way if the free parts cannot hold it. A
    far reach brings a step; when nothing is left the body is released to free physics.
  - Torques: capture point + centre-of-mass spring (Jᵀ) + gravity compensation (Coros
    2010), no new library; the whole-body optimiser (QP) can replace the torque step
    later. Foot placement details: later.
  - Physics on only for players in the play (bowler from the delivery start, batter
    at release, keeper and fielders when the ball comes); standing = kinematic.
- **Reason:** the user's rules (real mechanics, nothing faked, physics in one place).
  The 2026-11-05 muscles sagged (Jolt's frequency mode scales stiffness by each part's
  own inertia, ~19 N m/rad at the shoulder against ~10.6 N m of arm weight: 0.56 rad,
  as measured) and the body tipped (no balance).
- **Rejected:** external "balloon" forces (Gang Beasts, TABS), AI controllers (PHC,
  MaskedMimic, DeepMimic: mocap, GPU training, a network runtime), stiff joints alone
  (ankle stiffness < mgh), the hybrid (planned pose with physics only for dives).
- **Later the same day (the user's call):** the parts a target needs move together from
  the start, not one after another; each part's share comes from its place in the queue
  (the first the most, the last very little; `decider.queueShare`), and a blocked part's
  share goes to the rest. The queue's order depends on where the target is (below the
  hips: the hips lead; behind: the chest; ahead or to the side: the clavicle).
  Where research measured a split it replaces the place share (the user's call): the
  lumbar spine and the hips by the lumbopelvic rhythm (Esola et al. 1996).

## 2026-11-06: Standing balance (step 3a): ankles by Peterka, idle muscles hold their angle
- **Decision:**
  - The stance ankles' torque is the measured human ankle feedback T = -(KP θ + KD θ̇)
    (Peterka 2002: KP 18.6 N m/deg, KD 6.1 N m s/deg; KD/KP 0.33 s ≈ 1/ω₀, so it acts on
    the capture point), θ the body's sway about the stance ankles (the inverted
    pendulum's pivot; not the contact centre, which jumped and sat 14 cm ahead of them).
  - Ahead/back only: side to side the hips load and unload the legs (Winter 1995); a
    5.5 cm foot tips under side torques at the ankle.
  - The balancer owns only the ankle's plantar/dorsiflexion; its other axes stay with
    its muscle (releasing them let the body yaw and rock).
  - An idle muscle (no order) holds the angle it had when its last order ended (the
    user's call), the gap closed in fittsA by the velocity motor at full strength. A
    velocity servo at 0 alone crept (spine 0.28 rad in 4.5 s); Jolt's position spring
    collapsed the legs (frequency mode scales by the part's own inertia).
- **Rejected:** Peterka gains on both horizontal axes at the ankles; free ankle axes;
  Jolt's position motor for holding.
- **Later the same day:** side to side on two feet the hips' ab/adduction balances (the
  legs and pelvis turn as a parallelogram, ankle inversion free); on one foot the ankle's
  inversion. The balance acts as Jolt stiffness-and-damping springs on those joint axes
  (Peterka's KP/KD), not as added torques (the feet chattered). Toes have a muscle
  (8 N m). The limb test is physics only (the user's call); only the limb moved last has
  a target; R resets the body to its standing pose. Bones without a body ride their
  parent as built while physics is on.
- **2026-11-07:** Peterka's damping is split as he fit it (Eq. 15): the balance springs
  keep only the joint's passive damping B; the active KD acts on the body's sway rate
  through the spring's aim (sway + KD/KP sway rate, the capture point). All of KD in the
  joint damper fought the contacts on the light foot and left the body 6 cm forward.


## 2026-11-07: Limb joints close the two-bone solve's angle (the decider gives it)
- **Decision:** a limb joint's order (shoulder, elbow, hip, knee) carries its own turn
  to the decider's two-bone solve (`PartOrder::angle`), and its muscle closes that angle
  in Fitts' time for the end's gap (plus the target's own motion across its lever).
  This amends 2026-11-06 ("the decider decides no amounts") for the limb joints only;
  the trunk and hips joining a far reach still take the end's needed velocity across
  their lever, and limits and strength still stop any joint (blocked). The decider gets
  the game's poles: `ik.kneePole` and `reach.elbowPole`, as BodyReposition did.
- **Reason:** measured (the user's report of the arm turning the wrong way): from a
  straight limb, bending the middle joint first moves the end away from a straight-line
  path, so the lever projection drove the knee straight into its limit and the elbow
  the wrong way. With the solve's angles the limb's joints turn the solve's way and
  arrive together.
- **Rejected:** the whole chain's Jacobian for the speeds (singular at a straight limb:
  the same failure); clamping the projected speed's sign (a patch).

## 2026-11-07 (later): Limbs follow their IK pose with Jolt position motors
- **Decision (the user's call: "IK decides like the brain, Jolt moves everything"):**
  the decider solves a hand's or foot's two-bone IK from the limb's rest rotations on
  the body's real parents and sends each limb joint (shoulder, elbow, hip, knee) its
  target rotation. Its muscle is Jolt's position motor in stiffness-and-damping mode
  (N m/rad and N m s/rad per joint, `muscles.stiffness` / `jointDamping`), capped at
  the strength left after holding the weight; gravity compensation stays. Timing:
  the commanded point travels from the end to the goal in Fitts' time on a
  minimum-jerk path (Flash & Hogan 1985); a goal moved mid-way retargets the path.
  Replaces the earlier 2026-11-07 entry (the order's angle closed by a velocity
  motor) and the bend-axis twist patch.
- **Reason:** measured: a velocity motor has no target, so axes without an order
  drifted (the forearm's twist crept to its limit) and a solve from the drifting pose
  fed the drift back (the shoulder chased it; the arm flew 0.21 m off). The standard
  way to drive a physics body to a pose is a spring toward each joint's target angle
  (PD tracking: Jolt's `Ragdoll::DriveToPoseUsingMotors`; SIMBICON; Zordan & Hodgins);
  muscles act as springs toward an equilibrium angle the brain sets (Feldman; Bizzi).
  The 2026-11-06 rejection of the position motor was its frequency mode (stiffness
  scaled by each part's inertia); stiffness mode has real units.
- **Not yet:** idle joints and the trunk joining a far reach keep velocity motors;
  the stiffness and damping values are [E] estimates to check against the papers.
- **Later (the user's screenshot):** the arm's IK pole is no fixed direction: the
  elbow's side follows the arm's shortest swing from hanging relaxed off the chest
  (no added twist, the shoulder's neutral rotation); exactly overhead the swing goes
  through the front. A fixed pole twists a straight arm toward it and flips the elbow
  where the arm points along it. The decider's elbow pole setting is removed
  (`reach.elbowPole` stays for the kinematic reach).

## 2026-11-08: One centre-of-mass controller for balance
- **Decision (the user's call, planned 2026-11-07):** the balancer is one controller of
  the centre of mass, no per-case joints: capture point -> centre of pressure (Peterka's
  KP/KD written as the pressure they make, T = m g (p - aim); kept inside the touching
  soles' hull) -> ground force (LIPM: m omega0^2 (x - p) across, m g up) -> per-foot
  share (by where p lies between the ankles) -> J^T joint torques through each stance
  leg (ankle, knee, hip, minus the leg's weight below the joint; Coros 2010's support
  torque). One law in every direction (the side gains, [E] copies, are removed).
  - The ankle's torque goes through its Jolt springs (slope KP, aim giving the torque),
    solved with the contacts; knee and hip torques are added torques over their holds.
  - The support is the soles of the touching parts, not the corners touching this step.
  - A foot lifts once the pressure the feedback asks for (for the position, and for
    position and motion) lies inside the other foot; a lifting foot carries nothing.
- **Reason:** measured. Explicit ankle torques made the stance foot chatter and spin on
  one foot; a contact-corner support let a tilted foot press its own edge and stay
  tilted; lifting with the centre of mass at the foot's edge left no pressure range to
  bring it back. The single law also gives Winter's load/unload on two feet without
  hip springs.
- **Rejected:** keeping per-case springs (ankle, hip parallelogram, one-foot hip);
  lift on "centre of mass and capture point inside the foot" (lifted at the edge).


## 2026-11-08 (night): Whole-body QP tried and dropped; our Balancer stays
- **Decision (the user's call, after the agreed two-session timebox):** the whole-body
  program (`agents::WholeBodyController`: an RBDL tree of the Jolt body, one OSQP
  program per tick with the equations of motion, strength and friction pyramids; costs
  soles still, centre of mass by the Balancer's pressure point, joints to the plan) is
  removed with RBDL v3.3.1, OSQP v1.0.0, QDLDL v0.1.8 and Eigen 3.4.0. Balance stays on
  `agents::Balancer` (capture point -> pressure -> ground force -> J^T, 05a7db3). The
  trial is kept as `wip/whole_body_qp_trial_v1.patch` / `_v2.patch`.
- **Reason (measured):** the tree itself matched Jolt (rotations and ball-joint spin
  exact, positions within mm), but the program's plan is not what Jolt does with it:
  - Torque delivery: the torques go through Jolt springs aimed from where the joint is;
    the joint turns within the step, so Jolt gives the ankles about half the planned
    torque (26.5 -> 11.5 N m), spine 65-90 %, hips and knees 80-100 %. Exact torques
    with springs aimed at the plan's end of step fell at once.
  - Contact split: Jolt's feet share the ground force differently from the program's
    contacts (toes worst; toes made rigid with the foot in the tree: no change).
  - Session 1 (balance and held joints at equal weight): the program missed its own
    centre-of-mass goal from tick 1 (asked -0.008, got +0.075 m/s^2); stand, one foot
    and push all fell after ~1-1.5 s.
  - Session 2 (balance first: comWeight 1e6, held 1): it met its own goal every tick and
    stood 5 s, but sank 8 cm (0.99 -> 0.91 m), bent its trunk forward and ended with the
    centre of mass ~14 cm ahead of the ankles, tipping; one foot and the side push fell.
- **Would need, to retry:** torques applied as Jolt sees them (the program and the
  solver on the same contacts and the same step), i.e. a controller built around Jolt's
  own constraint solve, not a separate tree. Not planned.

## 2026-11-08 (night, later): Joint solver 40 steps (the ankle's torque reaches the ground); torso hold rejected
- **Decision (the user's call: "apply first and see"):** `jolt.jointVelocitySteps` 20 -> 40
  (config/physics_world.json). `setSixDofAxisSpring` now builds its target the way Jolt
  measures a motor's error (now^-1 * target, in the joint's frame).
- **Reason (measured, one-foot stance, per joint axis with Jolt's motor lambdas):** the
  body leaned out over a flat foot (no foot roll, 4 contacts over 10 cm) because the
  ankle's roll spring delivered ~80 % of the asked torque: effective stiffness ~853 N
  m/rad against the body's own m g h ~709, nearly neutral, so it drifted. The cause is
  the solver's convergence through the light foot (1 kg between the shin and the ground
  contacts), not our maths: 20 steps 80 %, 60 96 %, 200 99.5 %. The target-frame fix
  was real but small (no measurable change alone). One foot: 3.8 cm -> 2.7 cm at the end
  (worst side 3.3 -> 1.9 cm), passes. Cost per human per tick (release): 0.22 -> 0.30 ms
  (60 steps: 0.37). Mobile: to re-measure on the device.
- **Rejected (measured):** the stance hip holding the pelvis at its world aim
  (SIMBICON's torso control, Yin et al. 2007), saved in `wip/torso_hold_trick2.patch`.
  The pelvis stayed more upright (tilt 2.35 -> 1.4 deg), but the hip's reaction leaned
  the stance leg out: one foot 3.8 -> 7.0 cm at 20 steps, 2.5 -> 3.95 cm at 60. In
  SIMBICON a stiff ankle absorbs that reaction and stepping balances; our ankle is the
  balancer's torque source, so holding the trunk uses the same narrow pressure range
  that keeps the centre of mass over the foot. The trunk's moment has to be budgeted in
  the balancer (the hip-strategy plan: pressure point vs force line split), not added.

## 2026-11-08 (night): The assist string
- **Decision (the user's idea and call):** a rope from straight above a physics human's
  hips (`PhysicsWorld` ropes: tension only, spring + damping beyond its length, capped;
  applied in `step`, its pull readable). Anchored 1 m above the hips when physics turns
  on or R resets; slack while standing. Settings in physics_world.json "string" (off by
  default; on live by saving). Drawn while on: grey slack, yellow to red toward its cap.
- **Reason:** a measuring tool: its pull shows how much the body's own muscles and
  balance miss, case by case. Capped at half the body's weight so it never carries it.
- **Exception** to DESIGN's "no outside forces" (recorded there).

## 2026-11-08 (night, last): Balance by visible assists (replaces the centre-of-mass controller)
- **Decision (the user's call):** the 2026-11-05 rule "no outside forces, real values only" is
  reversed for balance. `agents::Balancer` now stands the physics body with visible assists,
  scaled by one balance strength (0..1): hover (70 % of the weight + a height spring on the
  hips, at most 95 %, only with a foot on the ground), upright torque on the pelvis (tilt only,
  by the body's tilt inertia), a horizontal pull of the centre of mass over the stance ankles,
  locked planted feet (a fixed Jolt SixDOF to a hidden kinematic anchor, made at load time,
  enabled on plant; the ankle of a locked foot is left free so the body pivots over it), and a
  step when the capture point leaves the support (Human moves the foot). Gains are feel
  tunables in human_joints.json "balance" (Hz / damping ratio), not body values. Drawn in game.
- **Removed:** the J^T stance torques, Peterka ankle springs, hip damping, owned/driven/held,
  `HumanBody::springTorque`, Jolt axis-spring helpers, `jointMotorTorqueAxes`, the assist string
  and the rope API (and their tests), the TEMP tests. Feet: friction 1.0, restitution 0 (they had
  0.3 bounce from the defaults). Masses unchanged (real values kept).

## 2026-11-09: Capsule carries the player; walk only
- **Decision (the user's call):** a Jolt `CharacterVirtual` capsule per player carries the
  root over the ground (upright always; slides, steps, slopes, other capsules). The
  procedural body is posed on it. Jog, run and the run-up delivery are removed (walk only;
  U bowls standing). The Jolt Ragdoll for dives and falls is planned on top later.
- **Reason:** a week of self-balancing physics-body attempts gave no solid result; a
  capsule keeps the player standing by construction, which is how sports games do it.
- **Kept, unused by the capsule:** `agents::Balancer` and the physics limb test (still
  run when physics is on).

## 2026-11-09 (later): Full physics body on the capsule
- **Decision (the user's calls):** the body is a full-physics Jolt Ragdoll driven by Jolt's
  `DriveToPoseUsingMotors`, with gravity holding (each joint carries the weight beyond it, at
  most its real strength). The capsule holds it up by a spring to the pelvis; the spring lets
  go when the hips lean past 4 angles from the centre of the feet, and comes back at rest.
- **Why:** Jolt does the joint physics; only the pose (our cricket logic) and thin glue are
  ours. Jolt's motors alone sag under gravity (a spring needs an error to push), hence the
  gravity holding, as real muscles hold posture.
- **Joints:** Jolt's pose drive needs SwingTwist joints; uneven ranges are centred on the
  parent's joint frame so the symmetric limits cover them.
- **Removed:** `Balancer`, the old velocity-muscle drive, `JumpController`, `BodyReposition`.

## 2026-11-10: Whole-body decider on targets; the capsule leads
- **Decision (the user's approval of the plan, 2026-11-10):** two systems, strictly
  apart. A planner (an action) only places the seven targets over time
  (`motion::TargetSet`: pose, mode Free / Driven / Pinned, weights, a foot's swing share
  and ground normal) and gives the capsule its intent; `motion::StepPlanner` is the
  example (one step: Start, Up, Land). The decider (`motion::BodyDecider`) knows no
  action: from the targets alone it decides the whole body each tick in priority order
  (pinned feet, blended support weights and the hips over them, driven targets, reach
  with the trunk queue, posture) and carries its own plan forward (warm start, seeded
  from the real pose only at reset and get-up); its output pose is bounded in speed and
  acceleration (hips) and joint rate (trunk, head, limbs), limbs reach at most 96 % of
  their length, grounded legs are solved last on the output pelvis, a short reach is
  reported, never snapped. Every mode and weight change blends over time.
- **The capsule leads** (reverses item D of 2026-11-09, "the capsule goes under the real
  hips"): it moves by the planner's intent, re-centres slowly toward the support while
  both feet are planted (the plan rebased so the pelvis's world target stays), and the
  plan's pelvis stays within a bound of it; the pin pulls the real pelvis to the plan's
  pelvis, at the body's strength and the support under the feet. The real pelvis's error
  from the pin is classed small / medium sustained (recovery step reported) / large (let
  go, with the release on). One strength (0..1) scales muscles and the pin.
- **Why:** everything measured 2026-11-06..09 that read the lagging physics body back into
  the plan jumped; every torque-level balancer fell; the capsule chasing the real hips
  while the hold pulled the hips to the capsule was a loop. Bounded commanded motion is
  how muscles work (activation), not a patch: no target change can make the plan jump.
- **Kept from the committed walk:** through a timed swing the hips stay between the
  feet's targets with the measured sway (`hipsSway` 2.7 cm, Orendurff 2004); without it
  the hips went fully over the stance foot (side sway 12.2 cm, real 3.9-7).
- **Removed:** `BodyGoals` (replaced by `TargetSet`), the hips' Fitts path, the joint
  `blocked` flags (never set by the game), the hips raised by bisection for the ankle's
  range (the ankle's range is Jolt's limit now), the per-limb turn orders.
- **Balance:** by construction (support-weighted hips reference, the pin) plus the
  planner's stepping on reports; no self-balancing controller. Capture-point stepping can
  come later inside the planner.

## 2026-11-11: Reach trunk queue in the documented order; lowest hips from the knees
- **Decision (the user's approval, 2026-11-11):** in `motion::BodyDecider` a hand target
  below the hips bends first (lumbar spine with the pelvis tilt, Esola split, then the
  upper spine, chest, clavicle) and lowers the hips with the legs last, as BIOMECHANICS
  9.1 orders it (the queue had the legs first). The hips go no lower than puts a grounded
  knee at 90° on two feet (semi-squat) or 67° on one (largest mean stance knee of a
  maximal one-leg foot reach, 9.7); this replaces "no squat or drop limit" (2026-10-25).
  The reach's pelvis tilt keeps to `reach.pelvisMinDeg` / `pelvisMaxDeg` (80° forward).
  Past the bounds the reach is reported short (a step is a planner's job).
- **Why:** measured before (real Human, 3 s holds): a hand at the toes squatted the knees
  to 150-164° with 26° of trunk and hip bend; a foot 0.8 m ahead bent the stance knee to
  127°. Measured humans: self-chosen ground lifts 48-71° knee, stoop the common default;
  one-leg foot reach 24-67° stance knee with the trunk upright (BIOMECHANICS 9.1, 9.7).
  After: hand at the toes knees 46-64°, pelvis tilt 51°, trunk 62°; foot reaches hold the
  stance knee at 67°. Walking unchanged (stepping test numbers identical; the stance knee
  peaks at 45° in a step, under the bound).
- **Rejected:** hips travelling toward a foot target in the air (measured: they stay over
  the stance foot until the foot takes weight).

## 2026-11-11 (later): Limbs reach to a 5° knee/elbow bend; limb test jumps and stands on the ground
- **Decision (the user's approval and game check, 2026-11-11):** the decider's limb reach
  is the limb's span with the knee or elbow bent `decider.limbMinBendDeg` (5°, quiet
  standing 0-5°, BIOMECHANICS 2), from its own two segment lengths; it replaces
  `reachClamp` 0.96, which bent every knee and elbow 32.5° (the old note "~16 deg" was
  wrong). General: every action's hips height and reach come from it.
- **Limb test:** an arrow press jumps the target `debug.limbTestStep` (0.1 m), no slide;
  the target never goes under the ground (y = 0); a foot target at the ground puts the
  foot there at once, pinned (it carries weight, the hips go between the feet), above
  the ground it is held up (Driven, no weight). The old test forced a foot target 1.2 cm
  above the ground and always Driven, so the body always stood on one leg.
- **Why:** measured: standing knees 32.5° (real 0-5°), the straighter knee while walking
  never under 33° (real ~5° mid-stance). After: standing 5.1°, walking x1 7-31°; the
  stepping test's foot slide 3.3 -> 2.0 cm, chest swing 2.2 -> 1.7°.
- **Rejected:** stepping the limb-test foot with the step planner (the user: the limb test
  only jumps the target).

## 2026-11-11 (night): The capsule follows the real hips; the hold rides the plan
- **Decision (the user's call, 2026-11-11):** the capsule goes under the physics pelvis
  every tick (moved, so it still collides); the root rides it. The capsule's hold (the
  hidden kinematic anchor sprung to the pelvis) rides the plan's pelvis, yaw from the
  root, target offset 0. Reverses "the capsule leads" (2026-11-10).
- **Removed:** the intent velocity, `character.recentreRate`, `decider.hipsOffsetMax`
  (its bound only existed to drag the plan along with a leading capsule; the legs' reach
  bounds the hips); `passiveInfluence.hips` 0 (it pulled the plan toward the root, now
  the real hips).
- **Why:** measured with W held: the capsule ran 19.5 cm ahead of the real hips (its pace
  a step per step, the hips half the foot's way), then the recentre pulled it back. Now
  0.6 cm; real pelvis accel 20.6 -> 9.4 m/s^2, foot slide 2.0 -> 1.4 cm.
- **No loop** (the 2026-11-09 one came from a hold riding a capsule that follows the real
  hips): the anchor rides the plan, and the plan stays put in the world as the root moves
  (rebase).
- **Rejected:** the anchor at the feet's centre: its damper damps the hips toward the
  feet's speed (still with both feet down), the real hips 8.8 cm behind the plan.

## 2026-11-13: Two muscles per joint axis (strength per direction)
- **Decision (the user's call):** a joint's muscle has one strength toward each way about
  its main axis (`human_joints.json` `muscles.directional`: knee, hip, ankle, shoulder,
  elbow, wrist; holding values, BIOMECHANICS 14). The joint's other axes, the gravity
  hold, and the trunk, neck and toes keep `maxTorque`.
- **How:** Jolt keeps a min and a max torque on the twist motor, so a hinge (knee, toe)
  gets its exact pair. Both swing axes share one limit, so on a swing axis
  `PhysicsWorld::driveRagdollToPose` picks, each tick, the strength of the muscle that the
  motor pulls with about that axis. That pull is the spring's push toward the target less
  the damping of the turn, so braking uses the opposite muscle.
- **Measured:** stepping, reach and limb-test cases are unchanged within 0.1-2 cm
  (nothing reaches its limit there; the capsule and the gravity hold carry the weight).
  A check with a 5 N m knee flexor slowed the knee (59 vs 70 deg at 0.3 s). A 5 N m hip
  flexor slowed the lift (67 vs 82 deg at 0.5 s), and a 5 N m hip extensor could not stop
  it (91 deg, target 82.5). So the right muscle acts each way.
- **Rejected:** turning Jolt's twist axis onto the main axis of the swing joints (exact
  limits, but it changes how Jolt bounds the ranges and could part them from the plan's
  limits).
- **Later the same day (the user's yes):** the gravity hold and the motor share one
  limit. `HumanBody::holdWeight` gives each part's need to `PhysicsWorld::setRagdollHold`.
  The drive keeps the hold within the muscle on each axis, at the body's strength, and
  the motor gets what is left. Before, each went up to the strength, so a joint could give
  twice it. Measured: stepping, reach and limb cases unchanged (they never come near a
  limit). Arm held straight ahead: 0.1 cm below the plan; with an 8 N m shoulder flexor
  35.5 cm (the old hold's own 120 N m limit carried it).

## 2026-11-14: Standing jump: the hips target alone
- **Decision (the user's design):** the jump is one active target, the hips: down, then
  pushed up past standing; every other part follows by its passive share (no hand or
  foot targets). In the push the capsule's hold carries the hips with the jump's hips
  target (not the decider's plan, which stops at the legs' reach); in the air the
  hold's joint is off (its down limit held the body up otherwise) and the body flies
  on the speed it has; it eases back in from the landing.
- **Why:** measured: with the hold off in the push (the legs alone) the body rose at
  1.0 m/s and never left the ground: the stance legs' muscles are only as stiff and
  hold only as much as the parts hanging below each joint, not the body above.
- **Rejected:** hand and foot targets (arm swing, tuck, feet under the body in the air):
  the user's call, simple first.

## 2026-11-15: The balance step: the hips move, the feet follow (capture point)
- **Decision (the user's design):** the body's decider steps on its own whenever the body's
  motion would take it out of its support, whatever moved it. WASD moves only the hips target
  (the walker's eased velocity); the decider computes the extrapolated centre of mass
  (XCoM = CoM + v / w0, w0 = sqrt(g / l), Hof, Gazendam & Sinke 2005) and, once it will be
  out of the feet's outline by the time a step could land, asks for a step
  (`BodyQueue::step`): the foot whose step leaves it best inside the new support (feet
  never cross, at most `maxStep` from the other), the two taking turns on a tie; the heel
  lands `landingBehind` short of where the XCoM will be, under its own hip. The decider
  makes the step itself (the user's call: it owns the StepPlanner, which shapes the foot's
  path); a foot moves only by a step, never slides. The step's time is
  the gait's for the body's speed (`motion::gaitTiming`), so its length is the speed times
  its time. Stopped, the hips target holds until the last step has landed.
- **Measured (W, 0.9 m/s):** steady steps 45.9 cm in 0.52 s (gaitTiming: 0.45 m, 0.5 s);
  stopping: a 3.4 cm closing step; S the same backward; A/D a side shuffle (lead foot out,
  the other closes to `minFootGap`); limb test unchanged (no steps).
- **Rejected:** the walk's fixed WASD step spots (`FootStepper::stepSpot`, kept unused);
  predicting from the plan's hips speed (it overshoots when the target stops and stepped
  on the seed's settling); free hips' support reference moving with the swing foot (the
  spot would chase itself).
- **Not used now (files kept, the user's call):** `motion::JumpPlanner` (Space no longer
  jumps); reach and dive will come from the same push on the hips.

## 2026-11-15: Active directions on a target
- **Decision (the user's design):** a Driven target can drive its part along chosen
  directions only (`Target::directions`, up to three at right angles; none: the whole
  position). Along the others the part's place is the body's own (support, legs, other
  targets). WASD gives the hips the way it moves; the jump (unused) gave up only.
- **Why:** with the hips driven across too, the support's sway to the standing foot was
  overridden (0.2 cm); now 4.5 cm (real 3.9-7 cm).
- **Rejected:** the separate hips weights across / up (`hipsWeightHorizontal/Vertical`):
  forward and across could not be split.

## 2026-11-15: One move controller; the decider makes walk, run and flight
- **Decision (the user's design):** the walk controller is the move controller
  (`motion::MoveController`, config "move"). It gives only the hips target's eased speed.
  The body's decider makes walking, running and the air from that speed (how far the hips
  are pushed decides the steps; a bigger push, running's flight). The same push will
  drive jump and dive: a small push steps, a large one, or the hips turning the legs off
  the ground, dives.
- **Rejected:** separate walk / run controllers or modes.

## 2026-11-16: A step's swing by Fitts' law; the walking step timing removed
- **Decision (the user's call: no walking values):** every step's swing is the foot's aimed
  move, Fitts' law a + b log2(2D / W) with the same values as the limb test's hand and foot
  goals (`muscles.fitts*`), at least the leg's swing; the weight shift by the capture point
  (entry below), bounded by `weightShiftMost` 0.5 s (natural APA) and none when the capture
  point starts outside the feet (the body falls: the step goes at once). `stepDuration` and
  `liftAt` are gone; a path's time stays the target's (the swing first, the shift the rest).
- **Why:** one swing time (0.45 s) for every step was a walking value; foot movements follow
  Fitts' law (foot pointing; pedal studies, Drury: ~0.19 + 0.055 s/bit, close to ours).
- **Rejected:** separate foot Fitts values (no stepping regression found; ours lie within
  ~0.05 s of Drury's over 0.1-0.6 m); the walking pendulum swing (steps here are aimed).

## 2026-11-16: A step's hips follow the capture point's path (DCM); the weight shift from it
- **Decision (the user's call: general, no walking values):** through a step the hips
  follow the divergent component of motion (Englsberger 2015; capture point, Hof 2005)
  over the standing foot to the new stance's centre, x' = -w0 (x - xi), w0 = sqrt(g / l) of
  the plan's centre of mass. The weight shift before the lift lasts the shortest time that
  keeps the centre of pressure within the feet (minimum-jerk DCM: 1.875 D / (w0 T)), at
  least the foot's unloading time, at most a whole step's; a hips path keeps its time (sync
  formula) and gives the shift at most the step shape's share.
- **Why:** `hipsSway` (2.7 cm, Orendurff walking) and "the hips travel half the foot's way"
  were walking fits; the pendulum gives every step its own sway (a walking-like step: 2.5
  cm each way, as measured) and the old 0.15 s shift needed the centre of pressure 11.5 cm
  outside the feet.
- **Rejected:** fixed sway values per step type; keeping 0.15 s (physically impossible).

## 2026-11-16: The support keeps the body upright only; weight and push from the legs (chosen, not built)
- **Decision (the user's call):** the capsule's support spring (physics_world.json
  "character.support") is to keep only the body's tilt and turn (it never tips). Its
  up/down and fore-aft pull goes: the stance legs carry the weight (the gravity hold must
  hold the whole body above a stance leg, not only the leg below each joint), and the push
  for steps, runs and jumps comes from the legs. Built one piece at a time, each checked in
  the game; the first piece: a stance leg holds the body's weight.
- **Why:** today the spring carries the body (jog: ~700 N, the legs almost nothing), its
  upward stop (anchor never above standing, an uncapped limit spring) blocks a jump, and it
  pulls the physics back onto the plan. A self-balancing body without it failed
  2026-11-06..08.
- **Rejected:** the capsule driving the hips' path (jump height from the plan, not the
  legs); no support at all (fell, 2026-11-06..08).

## 2026-11-16: The limb test only (walk, jog/run, jump removed)
- **Decision (the user's call):** the fielder keeps only the limb test. Removed: WASD
  walking (the hips target at the walker's velocity), E/Q pace, Ctrl/Shift jog and run,
  the Space jump (JumpPlanner), turning to the camera's heading and free view (V), the
  MoveController and gait timing, the old pose without the physics body (repositioning,
  leg IK, arm swing, head steadying), the walking FootStepper (it only keeps where each
  foot is planted), their configs (`move`, `jump`, `reposition`, walking `step` keys,
  unused `ik` keys) and tests. The decider's balance steps stay (the limb test uses
  them), timed by `step` alone.
- **Why:** walking and running are to be built again on one general mechanism; until
  then nothing half-working stays in the code.

## 2026-11-16: No running model in the decider (replaces the next entry)
- **Decision (the user's call):** the decider is general only: no gait-specific model. The
  spring-mass running (run share, spring legs, take-off, neutral point, flight timing,
  config `decider.run`) is removed; the hips' height is the legs' bounds and steps take the
  gait's timing at every speed. Running is to come back as the same general mechanism with
  parameters by speed (the user's 2026-11-15 design: a bigger push gives the flight), its
  push from the physics legs, not a spring in the plan.
- **Why:** it was a separate gait model inside the decider; measured, its spring gave the
  jog plan a ~60 deg stance knee and 6-27 x body weight spikes at landings.

## 2026-11-15: Running as spring legs in the decider (removed 2026-11-16, see above)
- **Decision:** running is the spring-mass model in the decider, not a gait mode: the
  hips ride the grounded leg as a spring (11.2 kN/m, rest = leg reach) and fly on
  gravity; the run share (0..1) comes from the body's own speed by Froude 0.5. A leg
  back at its length behind the hips takes off; the foot lands at the neutral point
  (Raibert 1986) when the hips come down to it. Each foot has its own StepPlanner, so
  the next foot can lift while the other is in the air.
- **Rejected:** a fixed 68 deg attack angle (with these legs the stance was uneven, the
  hips never rose back, flights were cut short); a fixed 0.45 s running swing (the foot
  waited on the ground and landed behind the hips); the push as g T / 2 alone (the cycle
  sank into a crouch at 5 m/s).
- **Status:** working (PROGRESS B2), see the next entry.

## 2026-11-15: The decider plans each swing ahead (no per-tick timing)
- **Decision (the user's design):** the decider plans a swing's landing time when a foot
  lands or lifts, or the hips target's motion changes, from its own prediction of the hips
  (a walking stance ends at its leg's reach, a spring stance when the spring is back at
  rest length; a flight by gravity), and then lets it run. A re-time keeps the swing's
  progress and changes only the rest of its clock. A stance is a spring from its own
  touchdown, so a leg that landed walking holds the walking height (the walk -> run start's
  first push comes from it). The height's run share is not commanded, so it is out of the
  hips' acceleration budget. One mechanism for every step; nothing gait-specific.
- **Why:** re-timing every tick from guesses fed on itself (a quintic restarted each tick
  at zero acceleration dragged the feet; a 0.02 s floor and a stance-end guess made feet
  hover; the hips fell, then the spring launched them). A Python sim of the whole loop
  showed the planned version stable (3 m/s 171/min, bob 8 cm; 5 m/s 189/min).
- **Rejected:** per-tick swing timing (`runSwingTiming`) and its quintic re-plan; time floors;
  a run-only stride planner (the user: nothing specific in the decider).

## 2026-11-15: The sync formula: every part follows an active target's path in its time
- **Decision (the user's design):** an active target has a path (A to B, its time). Every
  part that follows it gets its own A to B in the same time: same start, same end, its own
  shape inside (a foot: the weight shift, then the swing's arc). One rule for every part;
  several active targets: each gives its share on its own clock, the shares add up, the
  balance correction runs on the combined pose. Active always wins.
- **Steps:** a step happens when the hips' path (driven, derived from another target, or a
  push) takes the centre of mass out of the support. Planned at the path's start (an
  event), not each tick: the foot lands as the hips arrive. Where the hips stop they end
  between the feet; moving on, the capture point (Hof 2005) as before, blended by the
  hips' speed there (`stopBlendSpeed`; the mid-stance rule alone made steps alternate
  long and short while walking). Part 1 built (PROGRESS B2c).
- **Agreed for the next parts:** a hand within the arm's reach brings the trunk in by
  research shares (from ~0.8-0.9 of the reach, smoothly); a clavicle body with muscles;
  parts no target needs relaxed (passive joint stiffness only), physics moving them.
- **Rejected:** the step on its own gait clock (it landed 0.2 s after the hips arrived);
  stepping on every hips move (replaced by the centre of mass rule, the user's call).

## 2026-11-15: Pelvic list and one-leg balance (sync formula part 2)
- **Decision (the user's OK):** the pelvis drops on the side of the leg with less weight,
  4 deg with all the weight on one leg (BIOMECHANICS 4.3), by the weight's share eased, so
  it comes with the weight shift and leads a lift (Massion 1992). General: walking and
  running get it too. The trunk turns back from it so the chest stays upright. A foot held
  up keeps the centre of mass over the standing foot each tick (the foot's path moves the
  hips: a leg forward takes them back).
- **How:** the list turns the output pelvis exactly about the loaded hip, its place inside
  the hips' speed and acceleration bounds.
- **Rejected:** the list in the desired pose (the output's turn and place arrive at their
  own rates: the stance knee went straight, 1.5 deg); a height clamp on the output (it
  snapped the hips 13 cm in one tick when a foot was pinned far ahead); the list after the
  bounds (accel 8.906 over the 8.9 bound); a pivot between the hips by weight (the
  standing hip rose).

## 2026-11-15: Standing reach by research shares; the body steps after a far hand
- **Decision (the user's design, research numbers):** shares are decided from where the
  target is, never a queue's leftover. On both feet the centre of mass may lie anywhere
  over the feet (the hips go back only to keep it inside); a hand further than the arm and
  0.23 of the height past it (Moriyama 2022, functional reach) takes the hips toward it by
  the rest, so the centre of mass leaves the feet and the balance step steps. A foot never
  slides: a foot given a new spot (standing or held up) steps there in its path's time.
- **Rejected:** the hips last in the hand's trunk queue (1.6 % share, never moved: the trunk
  reached 80 cm with the hips 20 cm back); the trunk's leftover to the hips (the trunk
  reached on its own, the hips still went back); the centre of mass held over the ankles.

## 2026-11-16: The muscle plan (P / B), frozen
- **Decision (the user's OK):** the body is fixed in this order, each piece committed when
  its measured numbers pass (PROGRESS "The muscle plan"): P1 Jolt joints with the plan's
  exact limits (SixDOF), P2 clavicle body, P3 drive feedforward (velocity lead and inertia
  torque), P4 stance hold, P5 research stiffness and damping (ankle by Peterka), P6 the
  support keeps tilt and turn only; then B1 hand shares by minimum effort, B2 the pelvis
  turn from a foot target, B3 the chest's turn from the trunk's stiffness, B4 free arms by
  physics. Nothing for the limb test itself; no walking numbers in the checks.
- **Why:** measured: the physics body cannot follow the plan (Jolt clamps targets into
  limits that are not the plan's; a moving target lags by the damper; the ankle is limp),
  so no body-mechanism piece could show right until the drive does.

## 2026-11-17: Ragdoll joints are Jolt SixDOF with the plan's own ranges (muscle plan P1)
- **Decision:** every ragdoll joint is a `JPH::SixDOFConstraint` (pyramid swing,
  translation fixed) in the joint's rest frame from `ik::limitFrame` (X = twist or hinge
  axis, Y = first swing axis, Z = X x Y, the second swing's range flipped when Z is its
  opposite), with the plan's asymmetric ranges per axis (no centring, no half cones). A
  locked axis has no motor. `ik::clampRotation` copies Jolt's `ClampSwingTwist` (twist,
  pyramid, locked and single-swing paths, the side Jolt picks outside a range), so the plan
  and the physics hold one set (unit test `joint clamp`). Each free axis has a position
  motor (stiffness and damping as before) with its own torque limits: the directional pair
  on its axis, `maxTorque` on the others, the gravity hold applied within them and the
  motor given the rest. A coupled range (the ankle by the knee) is set each tick from the
  real coupled joint (`HumanBody::limitNow`, the same mapping as `limitIn`). The drive is
  ours (`SetTargetOrientationBS`), not `Ragdoll::DriveToPoseUsingMotors` (SwingTwist only).
  The decider clamps every limb joint it writes except a grounded leg's (the pinned foot
  wins); a clamped limb is reported short; a free arm's swing stops at its range.
- **Why:** measured (PROGRESS D1): Jolt clamps a motor's target into its limits, and the
  centred symmetric cones were not the plan's pyramid: the shoulder was clamped 31 deg,
  the free arm's target 126 deg (it flipped), the squat's ankle sat on a wrong limit.
- **Rejected:** keeping SwingTwist with centred half cones (a different set for large
  ranges); clamping grounded legs too (the plan's feet slid up to 44 cm in deep reaches);
  clamping every drive target in HumanBody (it would hide plan joints outside their range).

## 2026-11-17 Pelvis turn by the step's length
- **Decision:** the decider turns the pelvis by `pelvisTurnPerStep` (0.216 rad/m = 12.4 deg)
  times the fore-aft distance between the feet, not by a share of the feet line's
  `atan2` turn (`pelvisTurnShare` 0.11, fitted at one speed).
- **Why:** Yang 2013's 11 and 19 deg peak to peak at 0.40 and 1.47 m/s, with step length
  0.41 H (v / 1.35)^0.42 (BIOMECHANICS 4.1), are the same 12.4 deg per metre each way;
  the `atan2` form saturated for long steps. No table of turn against step length exists
  (BIOMECHANICS 4.3), so this is the two points' ratio; approximate (women, H 1.65).
- **Rejected:** keeping the fitted share.

## 2026-11-21: Both feet in the air: the support lets go
- **Decision:** when neither foot carries weight (the decider's `supportWeight` sum
  < 1e-3), `Human::physicsTick` disables the capsule's support joint exactly as the lean
  release does, whatever `releaseOn` says; the body falls by physics alone and the
  existing at-rest path takes it back.
- **Why:** the user's rule. Scaling only the motor strength by the weight left the
  joint's hard upper limit and its `freeDown` spring active, so the body was never free.
- **Rejected:** zeroing the motor alone (the limits still hold the body).

## 2026-11-21: The shoulder girdle is a body (muscle plan P2)
- **Decision:** the `clavicle` bone (clavicle + scapula as one body) has mass 1.2 % per side
  (Veeger et al. 1991: 0.156 + 0.705 kg), taken from the chest (de Leva's upper trunk 15.96
  -> 13.56 %, the total unchanged); shape, ranges and stiffness as every other part (its
  existing joint limits, K from the inertia beyond it); `muscles.maxTorque` 45 N m from
  isometric protraction / retraction 13-24 kg at the arm (Williams 2009, women) times ~0.21 m.
  `planTrunk` turns it like the spine joints (no clavicle-only code).
- **Why:** without a body the plan's clavicle turn landed in the shoulder (PROGRESS D1).
- **Rejected:** per-direction girdle strengths (one value is enough for now).

## 2026-11-21: The drive leads the pose (muscle plan P3, lead only); W pushes the hips
- **Decision (the user's OK):** each part's drive target is set D / K (its motor's damping
  over stiffness, 0.06 s) ahead along the pose's own joint velocity this tick
  (`HumanBody::drivePose`); not the head (a reflex from the real body).
  W pushes the hips target forward (locomotion.json "move"); the decider makes the steps.
- **Why:** measured walking at 1.2 m/s: the real swing foot touched down 0.1 s early and
  27 cm behind the plan, then slid 27 cm: the damper pulls toward no motion, so a target
  moving at w is followed D / K x w behind (PROGRESS D2). With the lead: 3-5 cm.
- **Rejected:** the velocity eased over the activation time (it led on after the pose
  stopped: 8 cm past the spot, knee +6 deg); the inertia torque part of P3 (not needed);
  a walking-only fix.

## 2026-11-21: Walk every way by input; two general decider fixes
- **Decision (the user's OK):** W / S / A / D push the hips target forward / back / sideways
  and E / Q change the forward speed within 0.4..1.6 m/s (locomotion.json "move"); the
  decider makes every step. In the decider, generally: (1) a grounded leg's reach counts
  from its ankle as solved (the heel risen by the toe-off), not the ground under it; (2) a
  weight shift's capture point starts at the body's own speed (a quintic: that velocity
  at the start, at rest at the lift; standing it is the minimum-jerk path as before), the
  shift the shortest time that keeps the centre of pressure inside the feet the whole
  way, none fitting: the step goes at once; `stepHips` runs the same path.
- **Why:** measured: (1) the hips dropped 1.3 cm after each landing of a slow walk; (2) a
  capture point taken at rest made the shift 0.5 s or 0.12 s on a 3 mm difference (a pause
  every other step). The top speed is our body's measured limit (hip muscle at its torque
  limit from 1.8 m/s with steps capped at 0.6 m), not the walk-run transition (2.16 m/s).
- **Rejected:** the shift by the LIP's fastest time (constant centre of pressure at the
  feet's edge): the long shift moved to the other foot; maxStepForward 0.9 (every other
  step landed 45-73 cm short); the swing's arc by the way gone (slide 6.5 -> 30 cm).

## 2026-11-21: The hips' bounded motion lands on its target (standing plan still)
- **Decision (the user's OK):** in `BodyDecider::limitOutput` the hips' speed toward the
  desired pelvis is also at most d / dt (it lands on the target this tick), besides
  hipsMaxSpeed and the stopping law sqrt(2 a d).
- **Why:** measured standing: the reference stood still but the output pelvis chattered
  +-0.17 mm in a 6-tick cycle (the continuous stopping law overshoots within a dt^2 of the
  target); the knee flipped 4.5..5.5 deg, the drive's lead made a 5.5 deg target gap and
  the support spring held 53.7 N m against the leg motors. After: plan still, 0.5 deg,
  10.7 N m; walking unchanged.
- **Rejected:** applying the stance hold first (computed, it is right, but the spring was
  fighting the chatter, not gravity); a stance hold split 50/50 between the feet without
  centres of pressure (measured worse: 105 N m, reverted).

## 2026-11-21: The support spring is a slack rope (up and down)
- **Decision (the user's design and OK):** the hold's anchor rides the plan's pelvis
  height too (the clamp to standing height gone); up/down is free from `freeDown` (0.03 m)
  below the target to `freeUp` (0.05 m, [E], physics_world.json character.support) above
  it, and Jolt's soft limit (the linear spring) catches only outside. The band keeps both
  sizes when the target moves (the target's y, unused by the off motor, marks its centre).
- **Why:** the hold blocked every rise (the clamp, and the band's top at the target: above
  the plan the ~46 kN/m limit pulled the hips down), so jog and run could not lift the
  hips. A slack rope catches a fall but lets the legs carry the body. Measured: walking
  the spring catches far less (1.2 m/s 202 -> 13 ticks), slides unchanged; squat spring
  torque 122.5 -> 135.2 N m (open).
- **Rejected:** the anchor on the real hips (no gap, holds nothing: the 11-09 loop; the
  body falls, CRICKET_NOHOLD); a tight hold at the plan's hips (the spring would carry the
  weight, not the legs).

## 2026-11-22: A hand further than standing: the step first, then the bend
- **Decision (the user's OK):** on a hand path's event (it starts or its end moves more than
  1 cm) the decider compares the way from the upright shoulder (hips between the feet) to
  the path's end with how far a body reaches standing (BIOMECHANICS 9.8: floor 0.63 m;
  shoulder height, the arm and 0.23 H fore-aft, 0.13 H sideways; in proportion between).
  The rest is how far the hips must end from the feet's middle; the foot lands mirrored
  about it (`stepSpot`), less what its pelvis turn (`pelvisTurnPerStep`) carries the shoulder
  on; the foot is one that gets there in a step, then the hand's side. The step takes its own
  time (weight shift and Fitts), the hand's path's if longer. The trunk's target goes from
  where it was to the hand's, less the way the hips still come, by the feet's progress. A step
  of the reach landed, the hips go on to their end themselves; another step only when the
  centre of mass there leaves the feet or a leg is left behind. `reachShift_` removed.
- **Why:** the body leaned first (trunk 79 deg) and stepped 160-360 ms after the press once
  the centre of mass was out. The hips' end counts from the feet's middle, not from the hips
  of the moment (leaning, they are 13 cm behind the ankles: a spot where the foot stood).
- **Rejected (measured):** asking the trunk solve how far it reaches (low it reaches further
  than people: pelvis 70 deg, lean 105); the step in the hand path's 0.2 s; the hips going
  after any shortfall (hand down: 16 cm the wrong way and a step); no pacing of the trunk (it
  bent 103 deg, the centre of mass 9 cm past the toes, before the foot lifted).

## 2026-11-22: The hips turn when the trunk is at its ranges, a leg goes round with them
- **Decision (the user's formula):** on a hand path's event, no far step needed and the trunk's
  own solve (pelvis and spine at their ranges, the feet as they stand) still short of the
  path's end by more than `stepMargin`: the hips turn about the vertical toward the trunk's
  bend, as little as lets the body reach it (eighths of the way there). On both feet the pelvis
  turns within its range (`pelvisMin/Max` 30 deg); further, or a standing foot past its hip's
  twist range: a leg lifts and goes round with its hip about the other foot (a turn to the left,
  the left leg back), landing turned, the other foot turning in place if its hip needs it.
  Feet keep their turn (`anchorYaw_`); the pelvis's range counts from the feet's mean turn; a
  step of anything else lands facing ahead again.
- **Also (2026-11-22):** the bend sideways past the pelvis's and spine's side range turns the
  hips (on both feet) until what is left is within it, coming in with how short the hand would
  stay (all at 1 cm): the turn at once flipped the trunk from a side bend to a forward one.
- **Why:** a hand down beside or behind the body stopped 10-18 cm short: the trunk bent to the
  side and back into its ranges, and the hips could not turn the bend to the front.
- **Rejected (measured):** the turn from the ranges alone (it stepped for balls the body already
  reached); a fixed 90 deg.

## 2026-11-22: A hand down beside the body: face it, then bend forward
- **Decision (the user's ask):** on a hand path's event the body turns until the target is in
  front (between the midline and the hand's shoulder), coming in with how far below the hanging
  hand it is (`facingDrop`); past 30 deg a foot goes round, the one that takes the hips toward
  the hand (target ahead: the turn's far foot forward); still further than standing reach, the
  step goes on toward it. The trunk of a hand down turns each joint the easiest way for its
  room per axis (`easiestSwing`) instead of the shortest, in by the same ramp. It replaces the
  hips' turn once the side bend was past the trunk's range.
- **Why:** right under the shoulder the shortest swing is the shoulder's 18 cm side offset, so
  the trunk bent sideways (pelvis 20, spine 14-37); people face the target and bend forward
  (Lavender 1995, Marras 2000, Konz 1974).
- **Rejected (measured in scratch):** the easiest swing without the facing (a ball 0.9 m left
  21 cm short, jerks); one least-effort solve for pelvis, spine and knees together (balls up to
  15 cm short, a step on every ball, 782 deg/s); lifting a leg behind as a balance (the user: no).

## 2026-11-22: Jog and run are an action of their own (RunPlanner)
- **Decision (the user's rule):** nothing of the run is in the decider. `motion::RunPlanner`
  (own file) only places the feet's and the hips' targets; `Human` uses it from 1.7 m/s of
  the hips' move (W, E / Q) and the decider's own steps wait meanwhile (`balanceSteps`).
- A leg's swing takes its own time, a stance ends once the hips have gone so far over the
  foot: at speed the stance is the shorter and the flight comes of itself. Hips: a free fall
  in the flight, the standing leg's half-sine push in the stance (`StanceFeet::load`).
- In a run's flight the hold does not let go (an exception to 2026-11-21 "both feet in the
  air"): no push, it only keeps the body upright. A fall that is no run lets go as before.

## 2026-11-22: Turning to the camera's heading, free view (V) again
- **Decision (the user's ask):** while it moves the player's root turns to the camera's
  heading (`Human::turnToHeading`); V holds the heading. Removed 2026-11-16, built anew.
- Three general lines in the decider (asked first): `rebase` keeps each foot's turn on the
  ground, a foot in the air on a step lands turned as its target is, and the free arms
  cancel the body's turning against its heading (not the meant turn).
- Arms and trunk lean of the run are the run planner's hand and chest targets.
