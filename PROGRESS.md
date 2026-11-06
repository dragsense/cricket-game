# Progress

## Current milestone: M6 Bowling rework — in progress (branch milestone-6)

Goal: the bowler's delivery in the spirit of the M5 chest throw: the player gives body
intent on circles around the bowler (trunk coil, arm-circle tilt, release point); the
body does the motion; line, length and speed come from it. Reworked 2026-10-28 (the
user's yes): two clicks on one arm circle; the body is never controlled, it follows the
hand through the reach stages (reverse IK). Old `startBowl` kept (unused).

- [x] **6.1 Action point and release** (the user: "feels better"; commit dbcbde0 has the
  full version with trunk coil and lean, back-foot step, stride, hips-then-shoulders and
  front arm).
- [x] **6.1d Hand only** (the user's call, 2026-10-28; built in watch mode, awaiting the
  user's check): rebuilt step by step from the bowling hand (dbcbde0 has the earlier
  full version). B, then:
  1. Action circle (yellow, full, `Fielder::actionCircle`: round the bowling shoulder,
     seen from the side, 0 down, 90 ahead; the wheel turns it about the vertical up to
     180 deg either way, Ctrl + wheel sizes it, at most the arm length): the mouse picks the
     action point (white) anywhere on it; the hand's way there shows in cyan
     (`Fielder::actionPathPoint`: in the plane of the shoulder, the resting hand and the
     point, always up through the front, never backward; the radius growing to the point's + the gap: `action.gap` 0.1 m to start,
     Shift + wheel changes it). Click (`Fielder::actionBowl`): the hand goes that way in
     `action.time`, past the point, then is pulled back onto it in `action.pullTime`
     (the wrist loads).
  2. Release circle (red, `bowlArc`: round the bowling shoulder, arm length, square to
     the batter; the wheel leans it out to the bowling-arm side only); the mouse picks
     the release point (magenta). Click (`releaseBowl`): the front foot steps straight
     ahead (`swing.strideLength`) once the hand is nearly down (`swing.strideAt`); the hand goes round the shoulder from the action point
     down to the fixed bottom point (`Fielder::bottomPoint`, straight below the shoulder
     by the arm; `Fielder::arcBetween`, `back.time`, speeding up), then onto the red
     circle by a quarter turn and round it (back, over) in `swing.time`; ball off its
     tangent at the release point, along the facing before the action (toward the batter). Follow-through on the red circle (`follow.*`); return.
  Chest and body (2026-10-29, the user's call; no hip twist): at the action point the
  chest turns side-on (chest bone first, spine_02 and spine_01 taking what its limit
  leaves): `coil.chestDeg` 20 always, growing once the point is less than
  `coil.aheadStart` 0.2 m ahead of the shoulder, to `coil.maxDeg` 35 an arm length
  behind; held to the bottom point, unwound by the release. With the hand from the action
  point the whole player turns side-on (`walk()`: the walker's facing, the feet stepping
  round until the stride): `swing.bodyDeg` 60 always, plus the angle at the shoulder
  between the bottom point and the red circle's point furthest across the body
  (`swing.leanSign`); full at the bottom point, back to straight by the release. The front
  foot strides, and points, along the facing before the action at `swing.strideAt` 0.9 of the way
  down (it lands in the swing, the body nearly straight); the foot turns straight while
  in the air. Values from BIOMECHANICS 11.5.
  Front hand (`Fielder::frontPathPoint`, blue, drawn in full): its way runs from rest
  round its own shoulder (via the front) to the end of the bowling hand's way, touching
  it; the hand starts `action.frontDelay` after the bowling hand and goes only
  `action.frontShare` 0.8 of it, then holds; after the release click back down the same
  way (`Fielder::frontDownPoint`); at rest in the swing.
  The circles, ways and points follow the shoulders as the chest carries them
  (`Fielder::shoulderNow`, from the chest bone each tick; the action point rides the
  shoulder).
  No reverse IK while bowling or throwing (2026-10-29, the user's call): the arm IK alone takes the
  hand (elbow bends as needed).
  R cancels. Real speed by default; I toggles slow motion (`slowMotion.factor` 0.2).
  U: a random action at once (`bowling.json` "random"; the release at the top of the red
  circle follows once the hand holds), the camera following; the mouse does not aim
  meanwhile. (A run-up and gather jump were tried and removed, the user's call,
  2026-10-29: M6 is a standing delivery.)
- [x] **6.2 Standing jump** (2026-10-29; the user's visual check passed): groundwork for
  the bowler's jump. `motion::JumpController`: crouch (held J deepens it, trunk forward,
  arms back), push-off (legs extend, arms forward-up), ballistic flight (feet tuck),
  landing absorb and recovery. WASD on release jumps forward / left / back / right
  (camera-relative; none: straight up). Standing only; picking waits. Tunables:
  `locomotion.json` "jump". Next: jumps from walk, jog and run (one-foot take-off,
  speed carried).

- [ ] **6.3 Body from the feet** (2026-11-02, the user's call: fix the basics first;
  awaiting the user's visual check). Simplified to three formulas after several layered
  attempts (centre-of-mass offset, hip-tilt levelling, iterative heel roll with body drops)
  made the upper body roll and the planted foot sink; all removed (the user: "simple").
  1. Body (`motion::repositionBody`): between the feet by the weight each carries; as high
     as every loaded leg reaches straight (`standReach` 0.998), the lowest deciding; on
     one foot the lifted side drops `stanceDropDeg` 4; a damped spring (`followRate`,
     `damping`); the leg clamp keeps both feet reachable (deepest crouch: a shin length). Straight standing legs: `maxReach` 1.0, `softBand` 0.0015, `ik.soften` 0.999.
  2. Limb test with a placed foot: one solve from real body constraints only
     (`motion::StanceSolver`, our own damped least squares; the user's call: no library,
     no added conditions): planted ball on its spot with toes flat (the body hangs from
     it), every joint inside its limit on every step, centre of mass over the planted
     foot (no ground contact yet: collisions and gravity later, the user's call), reaching ankle as near its target as these allow, joints
     near neutral otherwise (`human_joints.json` "stance"). Heel lift comes from the
     toe joint. Replaced: the undo on a failed reach, the unloaded-leg lowering switch,
     the out-of-reach foot placement, the shin-length crouch floor and the weight-first
     lift delay. Measured (left foot 0.8 m ahead / 0.8 m back / 0.6 m aside / 0.4 m up):
     reaching ankle 4.6 / 13 / 1.6 / 0.7 cm from target; planted toes at standing height
     (measured with the ground check, since removed). Too far (1.6 m ahead, 1.4 m aside): the foot stops where
     the balanced body reaches. The limb-test target goes anywhere (air or below the
     ground; the user's call); the foot gets as near as the body allows.
  3. Heel and foot elsewhere (`Fielder::solveIk`, walking, picking): heel lift = how far
     the ankle's limit tips the flat foot's toes down (smoothed `kHeelRate`); not yet on
     the solver.
  **2026-11-03 rework (the user: "walk looks good"; running continues next session):**
  `motion::BodyReposition` (renamed from StanceSolver, rewritten simply: each part reaches
  with its own joints, the next part takes the rest; see DECISIONS 2026-11-03) now does the
  body for standing, walking, running, WASD single steps (limb test) and the limb test's
  foot and hand targets (F cycles both feet and both hands). Hand targets swing opposite
  the other leg while walking. Foot stepper is path-only. Run: Shift + mouse wheel pace,
  flight overlap, `landAhead`. Measured (5 m/s): feet land 0.45-0.55 m ahead and leave
  0.55-0.7 m behind; pelvis 0.85-0.98 m (lowest at contact ends, not mid-stance: waits on
  gravity). Pick, throw, bowl and jump still use the backup repositioning and reach*.
  Next: running fixes (mid-stance low point, running elbows ~90 deg), then tests. Tests not
  run this session; test_walk.cpp lost the run-flight test with the removed flight carry.
  **2026-11-03 later:** running taken out for now (it read as a fast walk): Shift is a fast
  walk (`Gait::FastWalk`, walk shape), Shift + mouse wheel sets 0.9-2.2 m/s; the run flight
  overlap is removed from the stepper (back next session with a proper run). Walk: 0.9 m/s,
  0.45 m steps (~120 steps/min, `refSpeed` 0.9), swing 0.18 m peaking at 0.4. Chest target
  in BodyReposition (spine first, then pelvis); walking holds a 5 deg posture lean
  (`postureLeanDeg`). Measured: the walk dips ~4-5 cm in double support quickly (straight
  legs, no heel-toe roll); next: heel-to-toe foot roll in the step path, then running.
  **2026-11-04 foot roll (awaiting the user's visual check):** one foot mechanism in
  `BodyReposition` for every movement (BIOMECHANICS 12, the user's call: real body
  mechanics, no per-gait fixes). The shin's lean past the ankle's loaded range
  (`dorsiflexionDeg` / `plantarflexionDeg` 10) turns the foot about its heel (toes up) or
  ball (heel up, toes bent back), the contact point staying put; the ankle moves round it
  and the body is solved again on the rolled ankles (`rollPasses` 2). Replaces "feet
  flat". Not yet measured (dip, heel-off timing). Later, with the user's OK: remove the old
  heel lift in `Fielder::solveIk` and the stepper's foot pitch / toe bend.
  **2026-11-04 later:** heel and ball contacts come from the skeleton's shoe (foot box back
  edge, under the toe joint, on the sole; no guessed numbers on the body side). The roll
  only lifts the ankle where the heel/ball meets the ground (air foot just turns). Push-off:
  the stepper gives each foot a push share over `pushTime` before lift-off (target side);
  the body moves the ankle range toward `pushOffDeg` plantarflexion, so the heel rises and
  the knee bends before toe-off (lift-off knee snap gone, measured). A foot in the air no
  longer pulls the body. Swing foot leaves moving forward (`liftOffSpeed`, low arc).
  Open (measured): the front foot lands 3-4 cm beyond the leg's reach, so the body drops
  in one tick at landing (back knee ~45 deg jump). The body has no mass/damping yet
  (inertia proposed; the user: later, with gravity and forces). Rule (user): targets may
  take any path; the body following them is real mechanics only, no patches.
  **Physics plan (later, the user's call 2026-11-04):** the landing snap is the body having
  no mass. Real walking: the body vaults over a near-straight stance leg (inverted
  pendulum); stance heel rise and heel-strike ankle limit the drop; the step-to-step
  transition (back-leg push-off + front-leg collision, ~20-27 % of the stride) redirects
  the body, smoothly because of its mass. Nothing steers the hips. Steps: (1) mass only:
  the body keeps its momentum, legs constrain it (no snaps); (2) mass + gravity + leg
  forces: push-off and loading from real load instead of the stepper's clock (needs
  balance control). Running (researched 2026-11-04: stance 35-40 %, flight, spring leg with
  knee ~40 deg and the body lowest mid-stance, foot under the hips, elbows ~90 deg) needs
  that physics; without it a run is a fast walk. The fast walk (Shift + wheel pace) is
  removed: the player only walks. Next: the body's physics (mass, gravity, forces), then
  running on it (old run code is not the reference).
  Stumps and bails removed (code, scene, config; the user's call so they do not confuse the
  body physics). Rule (user): gravity, forces and mass always from one place
  (`physics::PhysicsWorld` + physics_world.json, Jolt), library first, nothing invented;
  body, bat and later objects use the same. Body physics plan approved (DECISIONS
  2026-11-04): 1 PhysicsWorld springs, 2 body mass, 3 leg springs, 4 pelvis follows,
  5 vertical only first.
- [x] **Physics 1** `PhysicsWorld`: kinematic bodies (`moveKinematic`), bodies that touch
  nothing, locked rotation, push-only springs (Jolt `DistanceConstraint` soft limits:
  `createSpring`, `setSpringDistance`, `setSpringEnabled`; `jolt.maxSprings`). Test: a
  70 kg mass on a 2 Hz spring settles at g/(2 pi f)^2 below rest; disabled, it falls.
- [x] **Physics 2-3** `agents::BodyPhysics`: the body's mass (73 kg, `bodyMass` in the
  skeleton config, de Leva 1996) as one Jolt body at the centre of mass (vertical only),
  world gravity; a kinematic anchor and a push-only Jolt spring per leg, on only while
  that foot is down (20 kN/m, BIOMECHANICS 13). Tests: sinks 18.1 mm on two legs
  (m g / 2 k cos^2 a), falls at g with both feet up. Not yet in the fielder.
- [x] **One ground** (the user's rule: every mover stays above one ground, no checks of
  its own): `world.groundHeight` in physics_world.json is the only ground. Jolt's ground
  slab sits at it; colliding Jolt bodies stop on it (the body mass now collides);
  `PhysicsWorld::groundHeightAt` / `aboveGround` (a Jolt ray onto the static layer) for
  movers that are not bodies: the camera uses it (its own y = 0 assumption removed); the
  ball's colliders are placed from it. Limbs, the ball vs Jolt bodies and the camera vs
  objects do not collide yet. Tests: query, lift, a body resting on a moved ground.
- [x] **Generic joints** (2026-11-05): `PhysicsWorld` capsule shape, `createHinge`,
  `createSwingTwist` (limits), `setJointMotor` (Jolt MotorSettings: frequency, damping,
  torque limit), hinge / swing-twist targets; `jolt.maxJoints`. Tests on simple objects
  (tests/test_physics_joints.cpp): capsule rests on its side; free hinge stops at its
  limit; a 50 N m motor holds a 2 kg arm (sags by load / stiffness, 0.19 rad as Jolt's
  spring predicts), a 1 N m one cannot; a swing-twist motor reaches its target.
- [~] **Physics body held, drawn; stiffness researched** (2026-11-06): the 19 parts are
  kinematic for now (the user: the player must not drop) and follow the planned pose each
  tick (`HumanBody::follow`, `PhysicsWorld::moveKinematic` with rotation); the motors are
  idle meanwhile. Debug draw shows each part's Jolt shape (orange), hinges (yellow, axis),
  SixDOF joints (cyan, X/Y/Z axes) and muscles (red ring) (`CricketApp::drawHumanBody`).
  Research (BIOMECHANICS 14): joint stiffness depends on activation and load; intrinsic
  ankle stiffness is below the toppling stiffness mgh (~650 N m/rad), so even quiet
  standing needs the balance controller. Agreed direction (talks 2026-11-05/06): one
  balance system for every activity, from the capture point (Hof: xi = x + v / w0;
  inside the feet: ankle/hip strategies, Peterka 2002; outside: step to xi + margin;
  beyond reach: controlled fall); the walk's foot placement should come from xi (the
  gait timer stays as the rhythm). **Designed 2026-11-06 (DESIGN "Physics body and
  balance", DECISIONS 2026-11-06):** physics only moves the body; BodyReposition becomes
  a decider (queue + directions from the real physics pose, speed from the target's
  timing); a balancer (capture point: ankle/hip blended, step, release; planned ahead;
  always wins); physics on only for players in the play. **Next:** 1 the decider next to
  BodyReposition (exact keep/remove list agreed); 2 muscles: velocity motors with real
  strength + gravity compensation; 3 balancer; 4 free fall and recovery from the landed
  pose. **Step 1 built (2026-11-06):** `motion::BodyDecider` (src/motion/body_decider.*):
  7 targets (`BodyGoals`: hands, feet, hips, chest, head, each with its own velocity),
  per-joint `blocked` flags in, a fixed-size `BodyQueue` out (joint, turn or move,
  direction, the target's velocity, active, share); hand: arm, then (out of reach) every
  part of the queue moves at once, each with a share by its place (the user's rule:
  `decider.queueShare` 0.5, a starting value, not measured; a blocked part's share goes
  to the rest); the queue is led by where the target is (below the hips: pelvis tilt,
  hips lower, spine, chest, clavicle; behind: chest first; ahead/side: clavicle first;
  BIOMECHANICS 9.1/9.3); chest and head share the same way; foot, push-off, hips. Not
  wired into the game yet. Test: tests/test_body_decider.cpp (target ahead, ball at the
  feet, shares, blocked parts, near target arm only). Measured ratios replace the place
  share where research has them: in a bend spine_01 : pelvis tilt by the lumbopelvic
  rhythm at the trunk's flexion (Esola 1996: 1.9 / 0.9 / 0.4 per 30 deg band;
  `decider.lumbarHipRatio`).
  **Step 2a built (2026-11-06):** generic physics for the muscles (`PhysicsWorld`):
  `setKinematic` (a body switches kinematic/dynamic keeping mass and velocity: physics
  on/off), velocity motors `setHingeVelocity` / `setSixDofVelocity` (SixDOF: joint frame
  on body b; torque limit kept, 0 holds), `addTorque`, `angularVelocity`. Tests on simple
  objects: kinematic→dynamic falls g t²/2; hinge velocity motor 0.5 rad/s → 0.500 rad
  in 1 s, a 1 N m one cannot lift the arm; SixDOF 0.4 rad/s → 0.400 rad, holds at 0;
  torque / inertia spin. The game keeps the body kinematic until the balancer (a
  dynamic body without balance falls). **Next, 2b:** HumanBody muscles from the
  decider's queue (velocity per joint from the shares and the target's motion, capped
  at real strength) + gravity compensation; blocked joints from Jolt; test with the
  pelvis held by the test only.
  **Step 2b built (2026-11-06):** `HumanBody::compensateGravity` (each
  muscle holds the weight beyond its joint, away from the pelvis, capped at its strength),
  `drive` (each active turn order: its share of the speed that closes the target's gap in
  Fitts' time plus the target's own motion, across the joint's lever; turn targets close
  in fittsA; joints without orders hold at 0; motor limit = strength left after holding),
  `blockedJoints` (at its limit by ik::clampRotation, no own muscle, or the motor at full
  strength by Jolt's lambda), `setPhysics`; `Rig::modelToLocal`;
  `PhysicsWorld::jointMotorTorque`. Tests (tests/test_human_body.cpp, pelvis held by the
  test): no target, 2 s: worst joint 0 rad off rest (was 0.56 rad sag) ✓; hand target
  0.32 m away: the wrist ends 0.7 mm from it after 2 s ✓ (0.05 m at 0.5 s, 0.01 m at
  0.75 s). **Fixed 2026-11-06** (it ended 0.82 m away): measured commanded vs. actual
  joint velocity in each joint's frame: Jolt's motors were right (body 2 relative to
  body 1, body 2's constraint axes), but the decider gave the elbow the forearm's
  world turn, which includes the shoulder's: the elbow was ordered about its locked
  axis at full strength, the motor-vs-limit leak spun the shoulder and forearm twists
  past their ranges. `BodyDecider::limbOrders` now orders each joint's own turn (its
  parent's turn in the solve taken out; the shoulder's is unchanged). **Next:** step 3
  (the balancer; wire into the game with physics on).
  **Step 3a in progress (2026-11-06; the user's slicing: 3a stand, 3b step, 3c release,
  3d game wiring):** `agents::Balancer` (src/agents/balancer.*): stance = feet meant to be
  down that touch the ground (`PhysicsWorld::groundContacts`, Jolt narrow phase: the
  touching face's corners); the stance ankles' plantar/dorsiflexion is the balancer's
  (`PhysicsWorld::setSixDofFreeAxes`), torque T = -(KP sway + KD sway rate) about the
  stance ankles, Peterka 2002 (KP 18.6 N m/deg, KD 6.1 N m s/deg; human_joints.json
  "balance"), ahead/back only (side to side is the hips', Winter 1995); a torque pair on
  shin and foot. Muscles: an idle muscle now holds the angle it had when its last order
  ended (the user's call), its gap closed in fittsA by the velocity motor at full
  strength (Jolt's position spring was too soft again). Measured (tests, nothing held):
  ahead/back the centre of mass stays within 5 mm of the ankles, joints within 0.01 rad;
  a forward reach moves the hips back. Failing (marked may_fail): side to side nothing
  balances yet: the body rocks on its foot edges (ankle inversion giving way, the other
  foot rolling), growing, falls after ~3 s. **Next:** the side-to-side control (hip
  load/unload, Winter 1995) and the hip strategy: the user's decision (hips are
  velocity muscles; both need hip torques).
  Balance is part of M6 (the user's call). The backups were deleted; the formula
  repositioning that jog, run, jump, pick, throw and bowl still use now lives in
  `agents::Human` (unchanged; `tests/test_reposition.cpp` removed with it).
- [~] **Muscles** (2026-11-05, in progress): each joint's Jolt motor drives it toward
  BodyReposition's planned pose (`HumanBody::setTargets`) with the joint's real strength
  as max torque (human_joints.json "muscles", BIOMECHANICS 14); the planned pose no longer
  follows the falling mass; the game draws each human from its physics body
  (`Human::physicalModels`). Measured: joints sag badly (shoulder 0.56 rad, the body
  slumps 0.99 -> 0.45 m) and, with no balance, it tips over (falls on its face, knees
  folded). Cause: motor stiffness from activation time (wrong: in Jolt it scales with the
  part's own inertia). Its test is marked may_fail. **Next session:** 1 research measured
  joint stiffness (N m/rad) per joint and use Jolt's stiffness mode (short); 2 balance
  (long: centre of mass over the feet, ankle/hip strategies, stepping; then the walk
  moving the physics body). Option offered: a debug toggle between the planned-pose and
  physics display so the walk is usable meanwhile. The walk in the game does not work
  while the physics body is drawn (the root moves, the body does not travel yet).
- [x] **Human from generic parts** (2026-11-05): `HumanBody` builds 19 Jolt bodies (each
  bone with mass: shape from the skeleton visual, de Leva share of 73 kg; neck, clavicles
  and fingers ride their parents) joined at each joint with the human_joints.json ranges
  (hinge: knee, elbow, toe; SixDOF: the rest); one collision group per human (its parts
  skip each other, Jolt group filter). Motors off: it folds onto the ground (centre of
  mass 0.99 -> 0.11 m). `ik::clampRotation` now measures ranges the way Jolt does (twist,
  then pyramid swing, per axis), so the pose and the physics read them alike (the user's
  OK); test_ik's ellipse check became a per-axis check. Joint solver iterations 20/5
  (`jolt.jointVelocitySteps` / `jointPositionSteps`): the muscle-less crash overshoots
  0.21 rad (default 0.30). The stepped pick of a ball 1.2 m ahead now ends 3.4 cm short
  (trunk twist range); its test is skipped (the user: no ball work now). The drawn pose's
  pelvis follows the falling centre of mass (expected until motors and display).
- [x] **SixDOF joint** (the user's call: real, uneven ranges): `createSixDof` (Jolt
  SixDOF, pyramid swing, per-axis min/max, motors, `setSixDofTarget`). Real joints bend
  further one way (hip, ankle); Jolt's swing-twist only has even cones. Tests: the same
  joint stops an arm at -0.30 one way and 1.20 the other; its motor reaches 0.73 of a 0.8
  target under the arm's weight. Fingers (0 mass in de Leva) will ride the hand.
- [x] **Ball solver removed** (the user's call): `src/physics/sim`, `src/physics/ball`,
  config/ball.json, their tests. A released ball stays where it is let go until the ball
  is a generic Jolt body. Next: the human from generic parts (shapes per bone, joints with
  the human_joints.json ranges), then muscles (motors) and balance.
- [x] **Leg springs removed** (2026-11-05, the user's call: no jugaad): the spring legs,
  their damping, the standing/walking switch and the hold-length on stopping were a
  stand-in for muscles and are gone. `HumanBody` is only the body's mass (73 kg, world
  gravity, vertical only, collides): nothing holds it up, so it falls until Jolt's ground
  stops it and the pelvis follows it (to watch the fall). Real support comes with muscles:
  Jolt Ragdoll (one body per segment, skeleton masses), joint motors driving toward the
  BodyReposition pose, balance (edge cases listed 2026-11-05; plan via start-milestone).
- [x] **Rename** (the user's call): one shared human body for every person (player,
  fielder, batter, bowler, crowd): `agents::Fielder` -> `agents::Human` (human.h/.cpp),
  `BodyPhysics` -> `HumanBody` (human-specific use of PhysicsWorld; generic physics stays
  in PhysicsWorld), `FielderVisual` -> `HumanVisual`, config `human_skeleton.json` /
  `human_joints.json` ("humanBody"). Roles (SimWorld fielder()/bowler()) keep their names.
- [x] **Physics 4-5** (vertical only): `Fielder::attachPhysics` puts the body's mass in
  the world (SimWorld, also after a world rebuild); feet stand on the real surface under
  them (Jolt ground query); the body's height from its mass on the legs sets the pelvis
  (`BodyTargets::bodyDrop`); human_joints.json "bodyPhysics" (20 kN/m, damping 0).
  Measured (walk 0.9 m/s): pelvis per-tick jump 0.45 mm (was 7-26 mm), worst knee jump
  15 deg (was 45 deg), pelvis bob 2.9 cm (real ~4-5 cm). Standing: pelvis 1.8 cm lower,
  knee 26 deg (real quiet standing 0-5 deg): the walking leg stiffness on a straight leg.
  Limb test on (`engine.json`): bowler hidden, no walking, jumping, balls or throws; F
  picks a foot, arrows move its target, Shift + Up/Down raise/lower it, R resets. Tests
  not rerun since the simplification (watch running).

## Previous milestone: M5 Bowling — Arm Action — in progress (branch milestone-5)

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
- [x] **Limb test** (built; awaiting the user's check; a tool for studying the body
  mechanics): `engine.json` `debug.limbTest` (on) hides the bowler, the throw and
  bowling arcs and WASD walking (code kept). F chooses the limb (right foot, left foot,
  right hand, left hand; yellow marker); a left click grabs its target, which follows
  the mouse on a level plane (Shift: an upright plane facing the camera, for height);
  another click lets go and it stays. Feet are placed directly (`FootStepper::place`),
  the body follows by `repositionBody`; a hand goal drives the reach stages
  (`Fielder::setPoseHand`, phase Pose). One hand goal at a time. R resets the pose.
  Later (the user's call): the arrows move the target (camera-relative, Shift: up/down;
  `debug.limbTestSpeed`) instead of the mouse; WASD walking is back on.
- [x] **Pick rework** (built; tests pass; awaiting the user's check): the old pick logic
  is removed (wait for a moving ball, pick range, hold time, auto lunge, give-up).
  Space reaches at once (Fitts' law hand speed; the hand follows a moving ball) and
  grips on contact; Alt+Space steps toward the ball first (the lunge), then reaches.
  Whether to wait, walk up or step is the player's choice. The controlled player
  always faces the camera's heading, standing, walking or running.
- [x] **Throw rework: player-driven chest throw** (built; the user checked it: "perfect").
  C puts the ball in the fielder's hand. Three clicks, each acting at once:
  1. Height circle (cyan, full, around the chest, radius `slot.radiusFactor` 1.0 x arm
     length): the hand goes to the clicked point (arm slot).
  2. Throw circle (yellow, `Fielder::chestArc`: around the throwing shoulder, radius
     `chest.radiusFactor` 1.0 x arm length, in the plane of the slot point and the facing
     direction: horizontal for sidearm, upright for over/underarm). The wheel leans it
     (+-`aim.maxTiltDeg` 30) before and after the force click. Force click anywhere on it
     (behind): the trunk winds back (`Fielder::twistChest`: `turnShare` 0.5 of the angle,
     pelvis `pelvisShare` 0.4, spine the rest, clamped to joint limits); the hand keeps its
     place on the trunk (`heldHand`); the circle follows the turning shoulder.
  3. Release click on the yellow circle: with force >= `stride.minForce` the foot opposite
     the throwing hand strides along the throw (0.3..0.9 m by force) and the swing waits
     for it to land; the chest swings forward speeding up, the hand blends out onto the
     yellow circle (`timing.blendShare` 0.6) with a push (`timing.push`), the ball leaves
     along the tangent at 10..31 m/s by force (full at `chest.maxBackDeg` 120), then
     follow-through and return; the chest turns back square.
  Test: `test.pauseAtRelease` holds the hand at the release point until the next click.
  The throw also works in the limb test. Tried and dropped (the user's call): pull-back
  along an arc, mouse pull/flick, a target with a predicted flight (too "classic"), a free
  mouse hand, a separate inner circle. Open: hips leading the chest in time, the glove
  arm, the back foot in the follow-through.
- [ ] **Next: bowling rework** in the same spirit (the user's call): see the bowling
  prompt in the chat of 2026-10-26.

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
  Settings in `config/human_joints.json` "reach". No smoothing yet (2.6), so the arm
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
  `config/human_skeleton.json` (height ratios); per-bone capsule/box visual kept
  separate; debug bones, joints and axes; ImGui height slider and bone readouts;
  neutral standing pose.
- [x] **1.2 Joint limits and IK** (unit-tested; the IK demo was removed after 1.3; ImGuizmo dropped at the user's request): `config/human_joints.json` (hinge and
  swing-twist); our own swing-twist clamp after every solve; ozz IKTwoBoneJob for the
  legs and arms with pole targets; IKAimJob for the head and chest; ImGuizmo targets.
- [x] **1.3 Body repositioning** (built, `motion_tests`; awaiting the user's visual
  check): `motion::repositionBody` runs every tick after the foot targets are set (a
  "late update"; no toggle). It generalises the user's RepositionBody: the pelvis eases
  toward standing height above the stance-weighted feet, then each hip-to-ankle distance
  is clamped to [min, max] × leg length (from the skeleton), stance leg last. Stance
  weight comes from foot heights (the lower foot carries the body) until 1.4 sets it.
  Tunables in `config/human_joints.json` "reposition". The IK demo is removed; arm
  and aim IK stay in `ik::` but are not driven until an action needs them.
- [x] **1.4 Foot planting and stepping** (built, `motion_tests`; awaiting the user's
  visual check): `motion::FootStepper` keeps both feet locked; `W A S D` request a
  step (character space; held = keep stepping, alternating feet).
  The next spot is relative to the planted foot (stance width + step from
  `config/human_joints.json` "stepping"); one foot swings at a time on a smoothstep
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
  Tests: `test_walk.cpp` additions and new `agents_tests` (`test_human.cpp`, whole
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
| M6 | Bowling rework: player-driven delivery (standing bowler) | in progress |
| M7 | Wicketkeeper | not started |
| M8 | Batting | not started |
| M8 | Match rules | not started |
| M9 | Broadcast cameras | not started |
| M10 | Game UI (RmlUi) | not started |
| M11 | Mobile (Android, then iOS on a Mac) | not started |

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
