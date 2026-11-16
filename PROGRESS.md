# Progress

## Current milestone: M6 Bowling rework — in progress (branch milestone-6)

Goal: the bowler's delivery in the spirit of the M5 chest throw: the player gives body
intent on circles around the bowler (trunk coil, arm-circle tilt, release point); the
body does the motion; line, length and speed come from it. Reworked 2026-10-28 (the
user's yes): two clicks on one arm circle; the body is never controlled, it follows the
hand through the reach stages (reverse IK). Old `startBowl` kept (unused).

**2026-11-14 cleanup (the user's call): only walk and jump stay.** Removed: throw and
bowling (`motion/throw_arc`, the bowler, the aim circles, `throw.json`, `bowling.json`),
hand reach and picking (`motion/reach`, grip, ball in hand), the ball gameplay (launch,
feeder, `ball_launch.json`; the scene's ball stays as a prop) and `wip/` patches. Kept:
walk, jump, the limb test, slow motion (`engine.json debug.slowMotionFactor`). The
decider's reach math moved into `body_decider.cpp`; the pelvis limits are now
`decider.pelvisMinDeg/MaxDeg`, the limb test's Fitts timing `muscles.fitts*`. The items
6.1 and 6.1d below describe code that is gone (history only).

**2026-11-15: branch `body-mechanism` (from milestone-6, the user's call): one body
mechanism instead of separate walk / jump.** The hips move, the body follows by real
balance: the decider's balance step (DECISIONS 2026-11-15).

**2026-11-16: the muscle plan, P / B (frozen, the user's OK; built in this order, each
piece committed once its numbers pass; no game checks, no limb-test-only fixes, no
walking numbers; DECISIONS "The muscle plan").** The question every piece answers: a
target moves from A to B, how does the body (its muscles, in physics) move.

Decoded first (measured, `measure: where the physics body misses the plan`,
`measure: the drive with a moving goal`, `measure: stance legs carry the weight`):
- D1 Jolt clamps a motor's target into its joint limits (`SwingTwistConstraint::
  SetTargetOrientationCS`), and Jolt's limits are not the plan's (Jolt: the parent's
  frame turned to each range's middle, symmetric half cones; the plan: the asymmetric
  pyramid in the rest frame; the ankle's knee-coupled dorsiflexion -25..-39 deg only in
  the plan; the clavicle has no body, so the plan's clavicle turn lands in the shoulder).
  Measured: shoulder 31.3 deg clamped (hand across, real hand 29 cm off), elbow 22 deg
  (hand up), wrist 4-5 deg, the free right arm's target 126 deg outside (it flips 177
  deg), squat right ankle on its limit (133 N m), that foot off the ground.
- D2 A moving target lags by the motor's damper (it pulls toward no motion): tracking
  = D / K x the plan's joint speed (shoulder 4.7 vs 5.0 deg predicted, spine 12.2 vs
  13.7, knee 6.4 vs 6.3, hip 4.6 vs 4.5); no muscle at its strength. Hand 0.5 m/s:
  forward 4.9 cm, across 8.3, up 10.0, down 4.6 cm behind.
- D3 The ankle is limp (K 17 vs ~570): feet 8-13 deg off the plan, motor 1.6 N m.
- Known: stiffness K = I (1/0.03)^2 (hip ~10x, ankle ~1/30 of research); the hold
  pelvis-rooted (a stance leg holds only what hangs below each joint); the support spring
  carries balance (one foot: tilt 67 N m, side 71 N; squat: 201 N down, 77 N m).

Pieces (pass numbers in brackets):
- [x] P0 Measures kept (`PhysicsWorld::supportLoad`, joint clamp / tracking angles,
  `Human::supportJoint`, `HumanBody::holdTorque`, `ragdoll`); the stance-hold try
  removed (comes back in P4).
- [ ] P1 Ragdoll joints as Jolt SixDOF with the plan's exact asymmetric pyramid limits
  (`SetRotationLimits`), the ankle's knee coupling each tick, a motor per axis; the plan
  keeps every joint it writes in the same limits [clamp < 1 deg static and moving; squat:
  both feet on the ground; free arm never flips].
- [ ] P2 Clavicle body and muscle (mass, strength from research) [hand across: real off
  plan 29 -> < 5 cm].
- [ ] P3 Drive feedforward (internal model; equilibrium point leads, Feldman, Flash 1987;
  Wolpert & Kawato 1998): the target led by D / K x the plan's joint velocity, plus each
  joint's inertia torque for the plan's acceleration [moving tracking < 1.5 deg; hand lag
  forward / across / up / down < 2 cm].
- [ ] P4 Stance hold: the plan's statics, the ground push inside the foot's functional
  base of support (Sloot 2025) [spring vertical ~0 in standing, one foot, squat; static
  leg motors < 5 N m].
- [ ] P5 Every joint's stiffness and damping from research (BIOMECHANICS 14: relaxed +
  slope x |torque|); the ankle by Peterka 2002 (KP 18.6 N m/deg, KD 6.1 N m s/deg) [K in
  the research ranges; feet within 2 deg of the plan; tracking <= 3 deg].
- [ ] P6 The support spring loses up/down, then fore-aft (tilt and turn stay) [hips
  within 1 cm of the plan; centre of mass inside the feet's functional support; no fall].
- [ ] B1 Hand shares by minimum effort (share ~ lever^2 / inertia moved, Khatib 1987),
  within range and strength, the rest by the same weights; the trunk joins from 0.8-0.9
  of the arm's reach (Levin 2001); Esola's lumbar : hip and 2:1 scapulohumeral kept;
  `queueShare` gone [trunk small within reach; Esola ratio; 2:1].
- [ ] B2 A foot target turns the pelvis by the same share rule (hip rotators);
  `pelvisTurnShare` gone [star excursion: trunk 1-5 deg, pelvis small].
- [ ] B3 The chest's turn against the pelvis by the trunk's axial stiffness (research)
  and the chest's inertia; `trunkTurn` gone [w = sqrt(K / I) from research].
- [ ] B4 Free arms by physics (relaxed shoulder 10.8 N m/rad, Gomi & Osu 1998), not
  driven by the plan; `armCancel` gone [a hanging arm within 2 deg of vertical, no flip].

- [x] **B1 Balance step** (built; awaiting the user's check in the game): WASD moves the
  hips target; the decider steps by the capture point (any direction, bigger push = bigger
  or more steps); `human_joints.json` "decider.balanceStep". Measure:
  `agents_tests --no-skip -tc="measure: balance steps*"`. Space no longer jumps.
  The decider makes the step itself (it owns the StepPlanner: start, re-aim, the foot's
  path, timing by the gait); Human only gives the hips target and keeps a landed foot's
  spot (`BodyQueue::landedFoot`). Numbers unchanged by that move.
- [x] **B1b Active directions** (the user's design): a Driven target may drive its part
  along some directions only (`Target::directions`, `motion::drivenPart`; hips, hands and
  feet). W drives the hips forward only; across and up stay the body's own, so the hips
  sway over the standing foot again: side sway 4.5 cm walking (was 0.2 cm; real 3.9-7 cm,
  Orendurff 2004). Replaces `hipsWeightHorizontal/Vertical`.
- [x] **B1c-1 Pelvis turns with the legs** (BIOMECHANICS 4.3, research done): the decider
  turns the pelvis by `decider.pelvisTurnShare` 0.11 of the turn of the line between the
  feet (a swinging foot counts at its target). Walking 0.9 m/s: pelvis 15.8 deg peak to
  peak in the plan, 15.2 real (real people 11 at 0.40 m/s, 19 at 1.47 m/s). Hand and
  chest targets still turn the pelvis on top (their reach queue composes with it).
  Measured too: the chest turns with the pelvis exactly (15.2 deg, correlation +1.00):
  the physics trunk does not counter-turn on its own (real ~8 deg and opposite), so B1c-2
  needs the trunk's spring-damper in the plan (Prins 2019).
- [x] **B1c-2 Chest turns through the trunk's spring-damper** (Prins 2019): the chest's
  turn against the pelvis is a rotational spring-damper driven by the pelvis's turn
  (`decider.trunkTurn` frequency 2.5 rad/s, damping 0.5; [E] fitted roughly, the user's
  rule: a close number is enough), shared evenly by the three spine joints; a hand or
  chest target driving the trunk takes over. Walking 0.9 m/s, steady (last 2 of 4.5 s):
  pelvis 15.1 deg, chest 13.0 deg, about a quarter cycle behind (correlation -0.06; was
  +1.00 locked to the pelvis). Real: chest 12 deg slow to 6 fast, timing -20 to -150 deg.
  Also seen over 4.5 s: A/D at 0.8 m/s the hips run 52 cm ahead, chest lean 19 deg (the
  side shuffle's limit, see B1).
- [x] **B1c-3 No fixed passive shares** (the user's design): `passiveInfluence` and
  `step.passive` hands, chest, hips 0; the head is a reflex (level, facing the heading,
  Pozzo 1990), no share. The arms swing by their pendulum and the turn-cancel alone:
  left shoulder 36 deg peak to peak walking at 0.9 m/s (was 18; real 30-40). Limb test:
  feet unchanged, hands within ~1 cm of before. `walk.sideSpeed` 0.8 -> 0.55 (the side
  shuffle's limit): over 4.5 s of A the hips 39 cm ahead (was 52), chest lean 8 deg (was
  19).
- [x] **B1c-4 Side shuffle's lean fixed** (measure: `measure: chest lean, W then A, by
  cause`, config variants in a temp copy): the stepping test's chest lean (10.4 deg) was
  not the arms (hands 0.5: 10.0) nor the plan (its pelvis level, no foot out of reach):
  the feet opened 90 cm in the shuffle and the real pelvis rolled ~10 deg on them.
  `decider.balanceStep.maxStep` 0.9 -> 0.6 m, `walk.sideSpeed` 0.55 -> 0.45 (the shuffle's
  limit with it). Now: A chest 2.6 deg, pelvis 1.0; the stepping test chest 2.6 deg and
  foot slide 7.0 cm (was 14.6); W foot slide 4.0 cm (was 18), first step 59 cm (was 76).
- [x] **B1c-5 Side shuffle's hips sinking fixed**: a step's reach is an ellipse
  (`maxStepForward` 0.6, `maxStepSide` 0.5; the hip flexes far more than it abducts),
  `walk.sideSpeed` 0.35 (the shuffle's limit with it); 0.5 every way had capped walking's
  steps (bob 3 -> 11 cm). A from standing: hips bob 17.7 -> 9.4 cm, foot slide 1.2 cm,
  chest lean 1.9 deg; after W: bob 4.1 cm. W unchanged (bob 3.1 cm, foot slide 4.0 cm).
  Added the "foot left behind steps" rule (`reachDrop` 0.04 m: a leg at its reach would
  hold the hips lower than that); it does not fire in steady walking (the steps chain),
  kept for reaches and pushes.
- [x] **B1d Walk controller renamed** (the user's call): `motion::MoveController`,
  `MoveSettings`, `src/motion/move_controller.*`, `tests/test_move.cpp`, config block
  `locomotion.json` "move" (was "walk"). It only gives the hips target's speed; walking,
  running and flight are the decider's, from the speed.
- [ ] **B1c Shares from the targets** (the rest): passive config shares to 0;
  a stepping foot turns the pelvis (not pushing it on: the hips target already does), the
  pelvis's turn turns the chest back, the arms cancel; posture (chest upright, head level)
  a decider rule. Every share from real numbers.
  Known, not fixed yet:
  - the real (physics) swing foot lags the plan at landing by 18-49 cm in forward walking
    at 0.9 m/s (0.45 m steps: the swing foot travels ~0.9 m in 0.4 s); the stepping
    test's foot slide is 26 cm (was 1.4-2 cm with 0.3 m steps): the muscles, next;
  - the side shuffle can keep up only to about (maxStep - minFootGap) / (2 x step time)
    ~0.62 m/s; `walk.sideSpeed` is 0.8, so the hips run ahead (lead step 0.9 m);
  - the first step from standing is long (76 cm): the walker reaches full speed in
    0.18 s (`walk.acceleration` 5);
  - turning on the spot takes no steps yet (feet stay); no hips sway with the hips driven.
- [x] **B2 Running from the speed (spring legs)** (built 2026-11-15; awaiting the user's check
  in the game). In the decider, one mechanism for walking and running:
  - Two steps at once (`steps_[2]`, one StepPlanner per foot); `BodyQueue::landed[2]`.
  - Run share 0..1 from the plan hips' level speed (Froude 0.5 = 2.16 m/s, +-0.3 band).
  - Height: a stance is a spring from its own touchdown (`springLeg_`: a foot that lands at
    run share >= 0.5; 11.2 kN/m, rest = leg reach); a leg that landed walking keeps the
    walking height; gravity in flight; inside the legs' bounds. In `limitOutput` the
    height's run share is out of the muscles' acceleration budget (a fall or push no longer
    starves the level motion: the start reached only 1.86 m/s before).
  - Take-off (`runTakeoff`): a grounded leg back at its rest length behind the hips lifts;
    the push sets the flight so the hips come down to the next touchdown height; its swing
    planned at once: the other foot's flight, its predicted stance (`springStance`) and a
    flight.
  - Swing times planned, not chased (`timeSwings`, the user's design "the decider plans
    ahead"): only on a foot's event or a change of the hips target's motion, from the
    body's prediction (`stanceLeft`: a walking leg ends at its reach, reachDrop below
    standing; a spring leg when back at rest length). `StepPlanner::setTimeLeft` keeps the
    swing's progress and changes only the rest of its clock (the per-tick quintic re-plan
    restarted at zero acceleration each tick: the feet dragged behind). No time floors
    (`touchdownTime` 0.02 s > a tick never landed: both feet hovered, the hips fell).
  - Python sim of the whole loop first (scratchpad; walk, walk -> jog, walk -> run).
  - Measured (release): jog 3 m/s 180/min, 1.05 m, contact 0.24 s, flight 12 %, plan bob
    6.0 cm, real speed 2.7-3.2 m/s (was bob 46.5, 0-6.4 m/s); run 5 m/s 210/min, 1.58 m,
    contact 0.17 s, flight 32 %, bob 7.0 cm, real 4.0-5.6 m/s (was 53.2, 1.1-8.9). W walking:
    pelvis turn 15.8 deg, bob 3.1 cm (unchanged), foot slide 3.2 cm (was 4.0: the first step
    lands before the stance leg runs out). Limb test: no steps. All tests pass.
  - Open: 5 m/s cadence 210/min is above the 175-185 of recreational runners (the sim had
    189); next: leg push for B3.
- [x] **B2b Limb test: every target, turned too** (2026-11-15, the user's ask): F picks any of
  the seven targets (right/left foot, right/left hand, hips, chest, head); arrows move it
  (Shift: up/down), Ctrl+arrows turn it (`debug.limbTestTurnDeg` 15 per press; Ctrl+Shift
  Left/Right roll). The decider, generally: a driven hips target turns the pelvis too
  (within pelvisMin/MaxDeg); a chest target's place bends the trunk from the lumbar spine
  (each joint within its range). The jump drives the hips up only (no turn).
  - A balance step that would land the foot where it stands (within stepMargin) is no
    step: a hand far out of reach re-stepped one foot on the spot every ~0.4 s forever.
    Measure `measure: hand far ahead, the steps settle` (CRICKET_PRESSES): 12-15 presses
    5-6 steps while held -> 0. Walking, limb test, reach shape unchanged.
  - Found: the run measure is chaotic (normalizing one quaternion moved the 5 m/s plan bob
    7.0 -> 11.1 cm): the running loop is not robust yet.
- [x] **B2c Sync formula, part 1: the hips' path plans the step** (2026-11-15, the user's
  design, DECISIONS "The sync formula"; awaiting the user's check in the game). A target
  carries its path (`Target::pathEnd`, `pathLeft`; the limb test fills it for hips, hands,
  feet). On a driven hips path the decider plans the step at the path's start from its end:
  a step when the centre of mass there is out of the support; the foot lands as the hips
  arrive (`pathStepTime`: the path's time left, no shorter than the hips' most speed or the
  leg's swing); where the hips stop they end between the feet (`stopBlendSpeed` 0.3 m/s:
  moving on, the capture point as before); a way past a step's reach is shared by more
  steps. Measure `measure: hips pressed once*` (limb test hips, one press):
  - fwd 0.20: foot lands 0.30 s, hips arrive 0.28 s (was no step); foot 42 cm, hips end
    1.8 cm from the feet's middle (was 19.3);
  - fwd 0.40: two steps, the last lands 0.40 s with the hips 0.40 s (was 0.62 vs 0.41);
  - back 0.20: lands 0.31 s, hips 0.31 s (was 0.48 vs 0.31); foot 39 cm (was 17), hips
    0.9 cm from the middle (was 12.3).
  W, jog, run, limb test arrows, hand far ahead: unchanged (W bob 3.1 cm, pelvis turn 15.8
  deg; jog 180/min, run 210/min). W's widest foot slide 5.5 cm at the stop (was 3.2):
  the stopping step now ends with the hips between the feet.
  fwd 0.10 and left 0.15 take no step: the centre of mass stays inside the whole foot's
  outline (toes 16 cm ahead of the ankle); the user: that is right (kept).
- [x] **B2d Sync formula, part 2: a foot held up moves the hips** (2026-11-15, awaiting the
  user's check in the game). Measure `measure: foot pressed once*` (limb test left foot,
  one press, 15 cm up):
  - Pelvic list (new, general: walking too): the side of the leg with less weight drops
    `decider.pelvisListDeg` 4 deg (BIOMECHANICS 4.3) by the weight's share, eased; turned
    on the output pelvis exactly about the loaded hip (so it never rises past its leg),
    its place within the hips' bounds. The trunk turns back from it (three spine joints)
    so the chest stays upright. Foot held up: 4.0 deg, reached by the foot's 50 % (the
    weight shift leads the lift, Massion 1992; was 0).
  - One-leg standing balance: the centre of mass over the standing foot each tick (as
    two-leg standing), so it follows the foot's path: foot 30 cm forward, hips 5.1 cm
    back (was 0.6 forward), CoM 0.2 cm from the ankle fore-aft (was 4.5 ahead).
  - The pelvis turn with the foot (6.6 deg at 30 cm) was already there.
  Walking W: chest lean 2.2 deg, bob plan 2.2 cm / real 3.0 (was 3.1 / 4.1), pelvis turn
  15.4 deg, foot slide 6.2 cm at the stop; jog and run: steps 1.05 / 1.56 m at the same
  speeds. All tests pass (decider: hips speed 1.36 m/s, accel 8.0 m/s^2; stance knee
  never under 3.7 deg).
- [x] **B2d-2 Limb test fixes** (2026-11-15, the user's report: side moves, odd steps):
  - The limb test's paths are kept in the world (the root moves under them); a press (the
    goal jumps over 2 cm) starts a new path in Fitts' time from where the point is. Before,
    the root's drift restarted tiny paths and the next press reused their tiny time (the
    hips target took a 10 cm press in ~0.07 s, the step landed before the hips moved).
  - More steps share a hips path only if this one leaves the centre of mass out of the
    support (a wide side step catches it); a path that has ended times no step (the foot
    left behind follows in the gait's time).
  - Measure `measure: hips pressed 0.1 m at a time` (as in the game): fwd and left, the
    step lands with the hips (0.71 s, hips 19.9 cm). Single press left 0.25: lands 0.33 s,
    hips 0.34 s (was 0.18 vs 0.34). `measure: hand pressed 0.1 m at a time`: no steps.
- [x] **B2d-3 A hand reaching low and far: no steps back and forth** (2026-11-15, the
  user's report). Measure `measure: hand down then forward, the steps` (7 presses down,
  6 forward): was 5 steps (a step back, then the left foot creeping 40 -> 56 cm and the
  right to 91 cm, past the hand's goal at 58); now none: the hips 23 cm back, the hand
  2.7 cm short at 58 cm. Two causes: the "hips end between the feet" spot (B2c) also
  served free hips, whose place follows the feet (a loop: the spot crept on with the
  reach's lean): now only where a hips target sets the place; and the foot-left-behind
  rule took a deliberate crouch for a leg at its reach: it now measures at the plan's hip
  height when that is lower.
- [x] **B2d-4 A foot held up to the side** (2026-11-15, the user's report: the leg went
  forward). The physics leg followed the plan (1 cm); the plan bent the knee forward. Now a
  foot held up (no step's timing) turns its knee toward the foot's way
  (`decider.kneeForwardBias` 0.25 of the leg ahead), as far as the thigh stays within the
  hip's abduction range (45 deg); a foot further out is held at that range (its height
  kept, its way out cut), and the pelvis hikes about the standing hip for what the hip
  cannot give (`footHike`, within pelvisMaxDeg). A step's swing keeps the knee forward
  (walking unchanged: the first try turned walking knees out, the user saw it). Measure
  `measure: foot out to the side, plan against real`: 20 cm out, 15 up: real 18.7 / 13.9
  cm, thigh at the range. Open: 60 cm out (past the range) ends with the thigh 81 deg
  forward and out, the foot 28 cm up: to look at.
- [x] **B2d-5 Limb test: a foot moved on the ground steps** (2026-11-15, the user: "foot
  moves, the hips move with it: a mini step"). Before, the limb test pinned the foot at
  its new spot at once (it slid; the plan's hips went on, the real foot stayed: the body
  fell back into a split). Now a pinned foot given a new spot (past stepMargin) steps
  there, in its path's Fitts time (the sync formula); the hips go between the feet with
  the swing as in walking. Measure `measure: foot moved on the ground, it steps` (4
  presses of 0.1 m): each a 0.2 s step, hips half the way (4.9 .. 18.6 cm), 1 cm from the
  feet's middle; the real ankle 2.4-3.1 cm ahead of the plan. Walking, running, the hips'
  and hand's cases unchanged.
  A foot held up and put down elsewhere steps down too (from where it is in the air), the
  user's rule "a foot never slides": `measure: foot held up` (15 cm up, then 20 cm ahead
  on the ground): a 0.3 s step, the plan ankle at most 2.3 cm a tick, the real 22.1 cm.
- [x] **B2d-6 A hand far out takes the body after it** (2026-11-15, the user: the hips
  move by their share, decided from where the target is, not a queue's leftover; a step
  comes by itself). Research (Moriyama 2022, functional reach, young adults): standing, a
  hand reaches 0.22-0.24 H past the arm with the pelvis +-0.01 H and the pressure 0.05 H
  forward. Now (a) on both feet balance keeps the centre of mass anywhere within the feet
  (the hips go back only to keep it inside; it was pulled over the ankles: the hips went
  20 cm back); (b) a hand further than the arm and `decider.standingReachShare` 0.23 H
  (level, from the upright shoulder) takes the hips toward it by the rest, decided from
  the hand's target each tick (so with its path), and balance keeps that: the centre of
  mass leaves the feet and the balance step steps. Measure `measure: hand pressed 0.1 m
  at a time` and `measure: hand down then forward` (CRICKET_DOWNS / CRICKET_FWDS): hand
  1.18 m ahead at shoulder height: one step, reached (was 10 cm short); 0.8 m: no step,
  hips 0 (was 19 cm back); down then 58 cm ahead: hips 7 cm back (was 23), reached.
  Walking, the foot and hips cases unchanged (S's widest foot slide 7.9 cm, was 5.5).
- [ ] **The limb test only** (2026-11-16, the user's call; DECISIONS): walking, jog/run,
  the jump, turning to the camera and the old non-physics pose removed; the decider has
  no running model (commit f2ff75d). Release build clean, all tests pass, the arrow-key
  limb test measures as before. Not committed; awaiting the user's check in the game.
  Walking and running come back later as one general mechanism (B2f below is on hold).
  Measured before (2026-11-16, test-only, reverted): walk's legs carry the weight (support
  up 0 N, knee within 1 deg) but its balance is the support's (level ~158 N, ground force
  line 20-35 cm off the centre of mass); jog: plan stance knee ~60 deg, the support carried
  ~700 N; the plan's fore-aft push came from the swing leg's motion (trunk ~0).
- [x] **A3 A step's swing by Fitts' law; no walking timing** (2026-11-16; the user checked
  it in the game: "perfect"; DECISIONS). The swing is the foot's aimed move:
  Fitts' law for its way with the limb test's values (`muscles.fitts*`; foot pointing and
  pedal studies, Drury ~0.19 + 0.055 s/bit), at least the leg's swing; `math::fittsTime`
  shared by Human and the decider. `stepDuration` 0.6 and `liftAt` 0.25 removed (config,
  `StepSettings`, the planner's defaults): each step is timed by the decider (shift by the
  capture point, swing by Fitts). A path keeps its time: the swing takes its own first, the
  shift what is left. The shift's bound is `weightShiftMost` 0.5 s (natural APA), and none
  when the capture point starts outside the feet (falling: it steps at once). The balance
  step's decision estimates the least shift + Fitts to the capture point. Predicted swing:
  0.1 m 0.20 s, 0.3 m 0.36 s, 0.6 m 0.46 s (was 0.45 for all). Measured: `hips fwd 0.40`
  trailing step 0.67 s (was 0.79), `hips left 0.35` second step 0.51 s (predicted 0.49);
  arrow-key limb test unchanged.
- [x] **A2 A step's hips by the capture point's path** (2026-11-16; the user checked it in the
  game: "perfect"; DECISIONS). Through a step of the decider's own the hips
  follow the DCM (Englsberger 2015): on the standing foot xi = p + (xi_end - p) e^(-w0 t_left),
  xi_end the new stance's centre; the hips x' = -w0 (x - xi), and settle the same way after
  the landing. The weight shift before the lift takes the shortest time that keeps the centre
  of pressure within the feet (at least supportBlendTime 0.12 s; at most a step's 0.6 s; a
  path step keeps its shape's share of the path's time). Gone: `hipsSway` and the hips going
  "half the foot's way". Predicted (sim): forward 0.3 m shift 0.27 s, hips 4.6 cm toward the
  stance foot; side out 0.3 m 0.14 s, 1.5 cm (real APA ~0.3 s fastest, ~0.5 s natural).
  Measured: `hips fwd 0.40` trailing step 0.79 s (0.34 shift + 0.45 swing); arrow-key limb test
  unchanged; nothing dropped. The decider test "hips never exceed their bounds" now runs with
  the body's masses and gravity (`Fixture::giveMassesAndGravity`), its landing its own.
  Next: step timing (`stepDuration` 0.6, `liftAt` 0.25 still estimate the balance step's
  decision and the gait swing), then the pelvis/chest turn values fitted to walking.
- [ ] **A1 Muscle stiffness from research** (2026-11-16, measured, design awaiting the user's
  answers). `measure: muscle stiffness, built against research`: the built K (inertia
  beyond x (1 / 0.03 s)^2, w 33 rad/s) against BIOMECHANICS 14 at the hold's torque:
  shoulder 548 vs 11-42, elbow 68 vs 9-23, hip 3069 vs 183-312, knee 447 vs 77-104,
  trunk 4842 vs 590, ankle 17 vs ~570 (0.8 mgh): the ankle is nearly limp (balance is the
  capsule spring's). The arrow-key sweep now prints the real end off the plan: hand
  fwd/back/out 4.3/4.3/3.8 cm, across 41, up 16; feet moved on the ground 65-74 cm (the
  real foot stays: an old issue, to look at). Predicted with research K and today's hold:
  hand lag 10-20 cm, standing knees fold to ~30 deg (the hold lifts only the shank and
  foot; the body above outweighs K 77) until the support's freeDown catches.
- [ ] **B2f Feet slide at fast walk, jog, run** (2026-11-15, measured, NOT fixed; next
  session starts here). `measure: gaits by speed`: foot slide walk 1 cm, 1.56 m/s 20 cm,
  jog 45 cm, run 46 cm. Chain found (`measure: swing foot*`, `measure: hip through a
  swing` with CRICKET_PACE / CRICKET_STRENGTH / CRICKET_KNEE):
  - the plan is right (sync formula, timing); the real pelvis follows it within 1-2 cm;
  - the real swing foot lands 50 cm behind and 11 cm high, then is dragged to the plan's
    pinned spot = the slide;
  - cause: the real KNEE stays bent the whole cycle (jog: lift-off 46-50 deg vs plan 5,
    landing 67-72 vs 5, stance 70-80 vs 35-50). The knee muscle's stiffness and weight
    hold are built from the parts beyond it (shank + foot): right for a swing, far too
    soft for a stance carrying the body; the decider's running leg is a 11.2 kN/m spring,
    the physics leg is much softer (real bob 12 cm vs plan 5);
  - tried, not kept: muscle feedforward (lead 2 x damping x activationTime = 0.06 s): the
    thigh then tracks with 5x strength, but the knee still bends; strength x5 alone: the
    thigh motor saturates at 1x (177 N m) but the lag is the damper (its pull is toward no
    turn, not the pose's turn) and the knee.
  - Also the user's report (not measured yet): from a crouch (knees bent) the leg
    straightens with a sudden jerk and the head shakes (not in normal walking). Likely the
    same root: the soft stance knee lets the real hips sink below the plan, then the
    pelvis hold and the knee motor at its limit snap the leg straight. Measure first: plan
    vs real knee, the pelvis height gap and the hold's force at the jerk.
  - Next (the user's OK pending): stance-leg muscle stiffness from the load it carries so
    the physics leg is the decider's legStiffness spring; then the user's "strength by
    speed" for all muscles and leg feedforward if still needed.
- [ ] **B2e Sync formula, the rest:** 3: a hand's trunk within reach by research shares,
  derived hips step; 4: the head's small share; 5: clavicle bodies and muscles; 6: free
  arms relaxed (physics). Plan agreed with the user 2026-11-15.
- [ ] **B3 Reach**: how far the ball is decides step, hop / jump or dive (one push).

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
  speed carried). (That controller was removed on 2026-11-09 with the physics body.)
  **2026-11-13:** jump research refreshed for the physics body (BIOMECHANICS 6: standing
  CMJ timings, poses, joint torques, arm swing; one-foot running take-off; landing; our
  strength, decider bounds and capsule hold against them). Next: the user's decisions
  (legs push vs. hold carries, per-direction strength), then one plan for the whole
  chain, then code.

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
  **Step 3a built, physics on in the limb test (2026-11-06; the user's slicing: 3a stand,
  3b step, 3c release, 3d game wiring):** `agents::Balancer` (src/agents/balancer.*):
  stance = feet meant to be down that touch the ground (`PhysicsWorld::groundContacts`,
  Jolt narrow phase); sway = the body's angle about the stance ankles; Peterka 2002's
  measured feedback (KP 18.6 N m/deg, KD 6.1 N m s/deg; human_joints.json "balance") as
  Jolt springs (`setSixDofAxisSpring`, stiffness-and-damping motor, solved with the
  contacts; `addTorque` made the feet chatter): ahead/back on the stance ankles'
  plantar/dorsiflexion; side to side on two feet the hips' ab/adduction (Winter 1995's
  load/unload, the legs and pelvis a parallelogram, ankle inversion free), on one foot
  its ankle's inversion (side gains: Peterka's sagittal values, [E]). Muscles: an idle
  muscle holds the angle it had when its last order ended (the user's call), the gap
  closed in fittsA by the velocity motor. Toes got a muscle (8 N m: toe flexors 1.54 +
  0.95 N/kg at ~4.5 cm; they flapped free). Tests (nothing held): stands 5 s, side to side
  too; a 0.15 m/s push no longer knocks it down; a forward reach moves the hips back; the
  game's own loop with physics on stands 5 s. Fixed 2026-11-07: it rested ~6 cm ahead of
  the ankles because all of Peterka's KD sat in the joint damper, which fought the
  contacts on the light foot each step (Jolt's spring alone is exact: new physics test).
  Now Peterka's own split (Eq. 15): the joint keeps its passive damping B (0.43 N m s/deg)
  and the active KD (5.8) acts on the body's sway rate through the spring's aim (sway +
  KD/KP sway rate: the capture point). Measured: standing within 5 mm of the ankles, a
  0.15 m/s push back to 4 mm in 3 s.
  **Game (2026-11-06, the user's call):** the limb test is physics only: the fielder's
  body is dynamic (`Human::setPhysicsOn`: decider -> balancer -> muscles each tick); only
  the limb moved last has a target, the rest holds and balances; the old kinematic limb
  test is gone (nothing while physics is off); R puts the body back at its standing pose
  (`Human::resetPhysicsBody`). Bones without a body (clavicles, neck, fingers) ride their
  parent as built (rest) while physics is on (they were drawn from the planned pose and
  the shoulder came apart on screen; the physical joints hold within 0.7 mm).
  **Next:** 3b stepping (one-foot stance falls).
  **Fixed 2026-11-07, limb IK in physics (the user's report: the arm turned the wrong
  way first and the elbow never bent):** the muscles took each limb joint's speed from
  the hand's or foot's straight-line gap across the joint's lever; from a straight limb
  bending the elbow or knee first moves the end away, so the joint turned against the
  IK (measured: the knee pushed straight into its -3 deg limit, the foot 0.3 m short).
  Now the decider's order carries the two-bone solve's angle per joint and the muscle
  closes it in Fitts' time (`PartOrder::angle`; the trunk joining a far reach is
  unchanged). The game also gave the decider the default elbow pole (back only), not
  `reach.elbowPole` (down, out, back): the elbow went over the head for a hand at the
  face. Measured, pelvis held, 2 s: knee-up foot 0.03 mm (knee 97 deg), heel back 0.03
  mm, hand ahead 0.5 mm, to the chest 2.7 mm, at the face 2.4 cm (elbow below the
  hand); the balanced forward reach 1.9 -> 0.7 cm.
  **Then (the user's report: the hand does not stop at its target, it turns away):**
  through the game's own loop the hand reached a target at the face, then the forearm's
  twist crept to its -80 deg limit and the shoulder was ordered 78 deg off: the arm flew
  0.21 m away (spin up to 8 rad/s); a side target stopped 3.6 cm short. The two-bone
  solve read the elbow's bend axis in the forearm's frame, twist included. Now it
  leaves the forearm's twist out (pronation is below the elbow's hinge). Measured: face
  0.7 mm held 2-4 s (spin 0.02 rad/s), side 1.2 mm, rig face 0.13 mm, forward reach
  0.4 mm. Left: the forearm's twist still creeps ~1 deg/s while its elbow is ordered
  (an ordered joint's other axes do not hold); it no longer moves the hand. The arm
  rising for a low target near the hip (the user's screenshot) was not reproduced in
  tests (6 spots around the hip reach within 5 mm); the user's steps are needed.
  **Then the IK -> Jolt handover redone (the user's call, DECISIONS 2026-11-07 "Limbs
  follow their IK pose"):** the decider (the brain) solves the limb from its rest
  rotations on the body's real parents and sends each limb joint its target rotation;
  the muscle is Jolt's position motor (stiffness mode, `human_joints.json`
  `muscles.stiffness` / `jointDamping`, [E]) capped at its strength, gravity held as
  before; the hand's or foot's commanded point travels to the goal in Fitts' time on a
  minimum-jerk path (`Human::GoalPath`). The speed-per-joint code and the bend-axis
  patch are gone. Measured through the game loop (right hand, 8 targets): within 1 cm
  in 0.67-0.98 s (just out of reach 1.7 s), held 1-8 mm from 2 s to 4 s, the forearm's
  twist no longer creeps (< 0.4 deg); a goal dragged mid-way follows. **Known:**
  straight overhead is slow (3.7 s, 2.7 cm): the clavicle has no body to lift the
  shoulder; idle joints and the trunk joining a far reach still use velocity motors;
  the stiffness and damping values are estimates to check.
  **Elbow's side (the user's screenshot: the hand turned over beside the thigh):** the
  fixed pole `reach.elbowPole` (out, down, back) twisted a nearly straight arm 30 deg
  inward. Now the elbow's side follows the arm's shortest swing from hanging off the
  chest (no added twist; exactly overhead through the front). The decider no longer
  has an elbow pole setting. Measured, both arms through the game loop: thigh 1.2 mm,
  twist 0.03 deg, palm to the thigh; hanging behind, face, side, low behind, overhead
  forward within 2 mm; a sweep front-low to back-low through straight down does not
  flip the elbow. The user checked it in the game: good. **Known:** straight overhead
  and overhead out stop 6-7 cm short with the shoulder's twist at its 90 deg limit
  (the clavicle has no body to lift the shoulder); across the body at chest height
  stops 25 cm short (the shoulder's swing range: flexion and adduction together; to
  check with the joint numbers).
  **Whole-body target pose (part B, tried 2026-11-07, saved, not applied):**
  `wip/whole_body_plan.patch` (untracked; against d1ac11f; also holds the 2026-11-06
  balance WIP hunks): the decider keeps one planned pose (trunk from upright with the
  queue's shares and the lumbopelvic split as amounts, legs by IK from the planned hips
  to the planted feet, the plan's centre of mass kept over the feet, the legs within
  their real ranges; the arm from the planned shoulder); every planned joint follows it
  by Jolt's spring; stance knees and hips get support torque for the body above them
  (Coros 2010). Measured: far ahead stands (6.8 cm short), the trunk straightens after;
  ground / beside-foot / low-behind reaches fall after 1.5-3 s (they stood before) and
  the face hold went 0.7 mm -> 1.6 cm. Cause: the body passes through unbalanced states
  on its way to the plan; holding a big posture change needs dynamic balance (hips
  back as you go, hip strategy, steps). **Next (the user's call):** the balance redesign
  (one COM controller: capture point -> ground force -> joint torques, Jᵀ, with the
  stance support torque), then part B on top of it. Kept from part B (committed): a
  SixDOF one-axis spring keeps the other axes' targets (the balancer's hip spring wiped
  the thigh's planned bend).
  **Talks 2026-11-07:** balance may be redesigned as one centre-of-mass controller
  (capture point -> ground force -> joint torques through the stance legs, Jᵀ) instead
  of per-case joints; the user's call pending. Yesterday's uncommitted one-foot work
  (lift gating, unloading pivot) is not committed: a lift measured that the weight
  shift overshoots (the hips' parallelogram tips about the feet's middle, not the
  stance ankle).
  **Balance redesign built (2026-11-08, the user's call; DECISIONS 2026-11-08 "One
  centre-of-mass controller"):** `agents::Balancer` is one controller for every stance:
  support = the soles of the touching foot parts (hull); pressure point p = aim + (KP (x
  - aim) + KD v) / (m g h) (Peterka's gains as a centre of pressure, on the capture
  point), kept in the support; ground force m omega0^2 (x - p) across, m g up; two feet
  share it by where p lies between the ankles (Winter's load/unload falls out); J^T
  torques through each stance leg (ankle, knee, hip; minus the leg's own weight below
  the joint: the support torque). The ankle's part goes through its Jolt springs
  (slope KP, aim giving the torque; an added torque made the light foot chatter and
  spin on one foot); the hips' ab/adduction on two feet is passive (B). Lift: the aim
  goes over the other ankle; the foot lifts once the pressure the feedback asks for
  (position alone, and with motion) lies inside the other foot; a lifting foot carries
  nothing. A joint the balancer lets go of holds its angle then (`HumanBody::drive`
  `driven`). Removed: the per-case ankle/hip springs, the parallelogram's free
  inversion, `sideStiffness` / `sideDamping`. Measured: standing 1.5 mm off the ankles
  (was 5 mm); push 0.15 m/s: 2.4 cm furthest, 1 mm after 3 s (was 4 mm); forward reach
  hips back 1.6 cm; **one foot (new test):** lift after 1.09 s, foot up 10 cm, stands
  3 s on the other, centre of mass at most 2.8 cm side / 4.2 cm ahead-back of the ankle
  (over the sole), 2.2 cm at the end. Not yet: hip strategy, stepping, release.
  **Part B re-applied on it, then reverted (2026-11-08, the user's call):** the far
  reaches stood, but in the limb test the hand moved backward as its target moved
  across the body (wrist 0.33 -> 0.26 m while the target went 0.5 -> 0.75 m): the
  plan's chest turn jumped tick to tick (-0.3, -5.8, -17.8, -2.4 deg) and the body's
  shoulder swung inward chasing it. Back to the arm aiming from the real body.
  **Slice 2, WIP not committed (2026-11-08): the body follows a hand target and the
  reach completes.** Saved as `wip/whole_body_plan_v4.patch` (the working tree minus
  `config/engine.json` and `src/main.cpp`, which are the user's). It contains: the plan
  made independent of the real body (planted-foot anchors, trunk reset once a tick, no
  physics blocked flags), a completion loop (`planTrunk` + `planLegs`, 16 rounds, 1 mm),
  toe support only at the ball edge, and `Balancer::setPosture` (virtual model control:
  the plan's pelvis held through torque-driven stance legs). Its TEMP tests ("TEMP
  arrows and reaches", "TEMP ground plan vs body") are not to be committed.
  **What failed:** torque-driven legs. For the ground reach ahead the plan is steady
  (hips 0.89 m up, 0.11 m back, knee 61 deg, lean 92 deg, wrist 1 mm off), but the body
  keeps its knee straight (3 deg; a force through a straight knee has no lever), its
  hips run back (-0.29 -> -1.13 m) and it falls at about 2.5 s; the leg springs also
  saturate the muscles. Before the posture, with the legs following the plan's angles by
  springs, the body tracked well (knee 51/51 deg, lean 61-64/65 deg) and stood; it fell
  only once the plan "completed" the low reach with a 92 deg lean and 9 cm hip drop.
  **Done (2026-11-08, the user's yes; still WIP, saved as `wip/whole_body_plan_v5.patch`):**
  torque-driven legs removed (`Balancer::setPosture`, `legStiffness`); the legs follow
  the plan by springs again. A low target's queue is led by the hips' drop (hips lower,
  pelvis tilt, spine_01 ...), and the drop is the one that alone brings the shoulder
  within reach (`motion::dropToward`; it closed only dir.y^2 of its share before). A
  planted leg's knee tracks its foot (pole carried with the foot from standing): the
  knees forward over toed-out feet put the ankles 18 deg out (range 15) and the plan
  raised the hips. Measured, game loop (TEMP arrows, both sides alike): far ahead
  1.1-1.6 cm standing; low behind 4-5 cm standing; hand forward 12 cm, back 25, out 16,
  up 22-24 (overhead known), down-forward 45 cm (lean 72), feet 0.1-42 cm, all
  standing; across L 1.1 m (hips 55 cm down) / R 0.55 m standing. **Still falling:**
  ground ahead, ground beside, hand straight down: backward, sitting. Cause measured
  (MuscleRig, goal set at once): the knees reach the plan's bend in ~0.1 s while the
  trunk lags (15 deg of a planned 61 at 0.125 s) and the ankle is the balancer's only
  (no plan ankle angle): the shank stays upright, the hips go 10-15 cm behind the plan,
  the centre of mass passes the heels. Compared: tilt-first order (also falls across
  and low behind) and the old drop (also falls across): both worse. `motion_tests`:
  2 decider cases fail from the WIP's design (blocked parts no longer zero a share; the
  plan starts upright, not from the bent pose): to update or restore with the user.
  **Real values (2026-11-08, the user's call: no estimates):** muscles.stiffness is
  now base + stiffnessPerTorque x the torque held (gravity hold + the balancer's stance
  torques; Gomi & Osu 1998, Huang 2020, Granata 2002), trunk 590 N m/rad whole
  (Cholewicki 2010), measured damping (0.63-100), and the ankle's dorsiflexion follows
  the knee (25 deg straight, 39 from 20 deg bend, weight-bearing, Baumbach 2014;
  `ik::LimitCoupling`, Jolt's SixDOF range updated each tick). Measured: worse; most
  hand cases and every low reach now fall or sit (hips 0.8 m down); far ahead 13 cm.
  Each change alone also worse (config swaps): the deeper ankle lets the plan squat to
  the ground, which the body cannot hold; the measured (low) damping rings; the
  stiffness alone falls too. Reading: the measured values are a body with reflexes
  and whole-body control around them; ours has neither yet. Not committed.
  **Speed (the user: the limb test is slow):** measured, ground reach: decide 0.59 ms
  per tick release, 8.0 ms debug (worst 26 ms; the 120 Hz budget is 8.3 ms) against
  0.003 ms with no hand goal: the plan's loops (16 completion rounds x trunk passes x
  the legs' 24 raise steps) each recompute the whole rig (localToModel). Debug runs
  slower than real time.
  **Speed (cont.):** `Rig::localToModel(pose, out, from)` (ozz's from/to: one joint's
  subtree), the plan's centre of mass by matrix x point, a pelvis move without a turn
  shifts the planned body (`movePlannedPelvis`, no skeleton pass): decide 0.52-0.59 ->
  0.36-0.38 ms release (ground reach). Same maths; float rounding grows only in falls.
  **Foot reach on the plan (the user's yes):** a lifted foot is an end like a hand
  (`footTrunk`: the stance leg lowers the hips, the pelvis tilts on the stance hip,
  Robinson & Gribble 2008; the leg solved from the planned hip; the old pelvis orders,
  which did nothing on the physics body, removed). Measured with the real values:
  foot forward / out / back now fall (stance leg sits), foot up 0.5 m short; the pinned
  pelvis test "a raised foot target bends the knee" fails (12 cm: the plan shifts its
  pelvis over the stance foot, the pinned real pelvis does not follow).
  **Next (to decide):** the whole body balanced at every tick, not only the end pose:
  either the plan's ankle angle as the balance loop's reference, or a whole-body
  controller (task-space QP: centre of mass, hand, joint limits together; a solver
  library would need the add-library ask). The user's check in the game first.
  **Then:** slice 3, stepping (a far foot target: the body moves and the other foot
  follows). Later, the same rule (put a support under the capture point) carried on to
  falls: hands or knees go where the body is heading; in the air nothing pushes (plan
  and ask first).
  **Whole-body QP trial, session 1 of 2 (2026-11-08, the user's call; not committed,
  saved as `wip/whole_body_qp_trial_v1.patch`):** libraries approved (add-library: RBDL
  v3.3.1 zlib, OSQP v1.0.0 Apache-2.0 with its QDLDL v0.1.8, Eigen 3.4.0 MPL-2.0; TSID
  rejected: always links LGPL eiquadprog, Pinocchio needs compiled Boost), pinned and
  cloned into third_party/deps (no configure-time download). `agents::WholeBodyController`:
  an RBDL tree of the Jolt body (free pelvis, ball joints, hinges, Jolt masses/inertias;
  `PhysicsWorld::inertia`, `groundFriction`), one OSQP program per tick (equations of
  motion, strength, friction pyramid at the Balancer's sole corners; costs: soles still,
  the centre of mass by the Balancer's unclamped pressure point, joints to the plan),
  torques through each joint's Jolt spring at its measured stiffness (ankle: Peterka KP).
  `balance.controller` "qp" / "jt" (set back to "jt"). Measured: the tree matches Jolt
  (rotations and ball-joint spin exact, positions mm); OSQP solves every tick; **it falls
  after ~1-1.5 s** (stand, one foot, push all fail). Leads, in order: (1) the program
  misses its own centre-of-mass goal from tick 1 (asked -0.008, gives +0.075 m/s^2): the
  costs' weights conflict (held hands/head 100 = centre of mass 100, soles 1e4) -> try
  strict priorities (balance first, posture in what is left; Lee & Goswami 2012);
  (2) Jolt's feet respond differently from the tree (toes worst; toes now rigid with the
  foot in the tree, no change). Session 2: (1) then re-measure; if it does not stand, drop
  the QP. With "jt" the old balancer passes stand/push/game loop; the one-foot test ends
  3.8 cm off the ankle (limit 3 cm), believed from the uncommitted real values (not
  verified).
  **Session 2 (2026-11-08, saved as `wip/whole_body_qp_trial_v2.patch`):** lead (1)
  confirmed: with the centre of mass far above the held joints (comWeight 1e6,
  heldWeight 1) the program meets its own balance goal every tick and the body stands
  5 s, but sinks 8 cm (0.99 -> 0.91 m), its trunk bends forward and its centre of mass
  ends ~14 cm ahead of the ankles, tipping. One foot and the side push still fall.
  Measured cause: Jolt gives the ankles about half the program's torque (spring aimed
  from where the joint is; the joint turns within the step; 26.5 -> 11.5 N m), spine
  65-90 %, hips and knees 80-100 %. Tried and reverted: exact torque + spring aimed at
  the plan's end of step (fell at once). Controller set back to "jt".
  **QP dropped (2026-11-08 night, the user's call; DECISIONS "Whole-body QP tried and
  dropped"):** WholeBodyController, its test, RBDL / OSQP / QDLDL / Eigen (pins, clones,
  build folders), `balance.controller` / `balance.qp`, `PhysicsWorld::inertia` /
  `groundFriction()`, the Balancer's wantedPressure / supportPoints / supportPart and
  HumanBody::drive's `external` / `commanded()` removed; `springTorque` kept; the trial
  stays in `wip/whole_body_qp_trial_v1/v2.patch`. Release build clean. Balance tests:
  standing 0.17 cm off the ankles (0.992 -> 0.992 m); push 0.15 m/s 2.5 cm furthest,
  0.18 cm after 3 s; forward reach hips back 2.0 cm, wrist 5.5 mm; game loop lowest
  0.988 m; **one foot fails:** lift at 1.09 s, foot up 11.6 cm, off the stance ankle
  0.9 cm at 1.5 s growing steadily to 3.77 cm at 4 s (side 3.2, ahead/back 3.1; limit
  3 cm). Next: the hip strategy + stepping plan (for the user's approval).
  **Balance by visible assists (2026-11-08 night, the user's call; DECISIONS same name), not
  committed yet:** measured (release): standing 0.19 cm off the ankles; push 0.15 m/s 0.7 cm
  furthest, 0.14 cm after 3 s; forward reach hips back 2.1 cm, wrist 1 cm; one foot 2.2 cm worst,
  0.45 cm at the end; 1 m/s push: steps and stands; locked foot slides 0.04 mm under a side push;
  knock -> feet let go, strength back to 1 in 1 s; game loop stands. **Still failing:** raised
  foot target 8.8 cm short (knee 91 deg); game-loop hand at the face 12 mm (limit 10). Next: the
  user's in-game check (limb-test arrows, see the drawn assists), then those two, then commit.
  **Delivery fix (2026-11-08 night, DECISIONS "Joint solver 40 steps"):** measured
  (TEMP one-foot drift test, Jolt's motor torque per axis, `jointMotorTorqueAxes`): the
  foot stays flat; the stance ankle's roll spring gave ~80 % of its asked torque (Jolt's
  solver not converging through the light foot). `jolt.jointVelocitySteps` 20 -> 40,
  SixDOF axis spring target in Jolt's own error frame. One foot now passes: 2.7 cm at
  the end (worst side 1.9, ahead/back 3.1). Standing 1.2 mm, push 2.3 cm / 1.3 mm after
  3 s, forward reach hips back 1.9 cm, game loop lowest 0.991 m. Cost 0.22 -> 0.30 ms
  per human per tick. **Tried and rejected:** the stance hip holding the pelvis upright
  in the world (SIMBICON torso control): one foot 7.0 cm (worse); saved in
  `wip/torso_hold_trick2.patch`. Still failing: "a raised foot target bends the knee"
  (9.6 cm, known), the 2 decider tests (known); the arrow-key limb cases are poor in the
  WIP state at both 20 and 40 steps (hand fwd, hand out, hand down and ground reaches
  fall or end 0.4-2 m off; 40 vs 20: 3 cases now stand, 4 now fall). Committed alone on
  HEAD as 3643ebd (all suites passed there; one foot 1.4 cm).
  **Assist string (2026-11-08 night, DECISIONS "The assist string"):** PhysicsWorld
  ropes + the hips' string in Human, drawn in the game; physics_world.json "string" (off
  by default). Rope test passes. Not committed (it sits on the WIP). Next: arrow-key
  cases (measured with the string), then relaxed idle muscles + release on a fall.
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

- [~] **Capsule + ragdoll** (2026-11-09, the user's call; DECISIONS 2026-11-09):
  - [x] **A. Walk only** (b55c2ff): jog, run and the U run-up delivery (bound, run-on,
    check stops) removed; U bowls a random action standing. Shift/Ctrl stay as held
    modifiers (`ModShift`/`ModCtrl`).
  - [x] **B. Capsule** (1973a0f): Jolt `CharacterVirtual` per player (`PhysicsWorld`
    createCharacter/moveCharacter, physics_world.json "character"); `Human::followCapsule`
    moves the root through it each tick. Measured: 1.5 m/s for 1 s = 1.5 m on the ground;
    stops at a wall (face - radius - padding) and slides along it; lands from a 2 m drop.
  - [~] **C. Full-physics body on the capsule** (2026-11-09, the user's calls; DECISIONS
    2026-11-09 "Full physics body on the capsule"; awaiting the user's test):
    - `HumanBody` is a Jolt Ragdoll (`PhysicsWorld::createRagdoll`): one dynamic body per bone
      with mass, Jolt SwingTwist joints with the real ranges (hinge: swings locked; uneven
      swing ranges centred on the parent's joint frame, symmetric half cones).
    - Muscles: `holdWeight` (each joint carries the weight beyond it, capped at `maxTorque`)
      + Jolt `Ragdoll::DriveToPoseUsingMotors` toward the pose (measured stiffness/damping in
      Jolt's stiffness mode where listed, else activation-time frequency).
    - The capsule holds the body: a hidden kinematic anchor on the capsule's feet, a SixDOF
      spring to the pelvis on all six axes (stiffness = whole-body mass / inertia x (2 pi 4
      Hz)^2). The capsule's height follows the pose's pelvis (shapes 5 cm apart).
    - Release: the hips lean past forward/back/left/right degrees seen from the centre of both
      feet (physics_world.json "character.release"): the spring is off, the body falls; at
      rest 1 s the capsule goes under the pelvis and the spring eases back in over 1 s.
    - Limb test: the goals go through the body decider; its plan is the muscles' target.
    - Removed: `Balancer`, the old custom drive, tests/test_human_body.cpp, `JumpController`
      (J key, "jump" config), `BodyReposition` (stance now `repositionBody` + `solveIk`).
    - Walking input is off for now (the legs lag the walking pose).
    - Drawn: each capsule (white held, red let go) and the feet-centre-to-hips line.
    - Measured (temporary tests, not committed): standing worst joint 1.3 cm for 5 s; a
      chest shove holds; after a fall it stands back up (pelvis 7 mm).
    - **Open:** every limb-test move drops the body: the decider's plan leans the hips 5-10
      deg back (past backDeg 4.5). Proposed: release on actual minus planned lean, the
      capsule following the planned pelvis. Walking: the measured (relaxed) stiffness is too
      soft for fast moves (knee lags); proposed: stiffness from the limb's inertia x the
      activation frequency. Test files to decide: test_physics_joints.cpp (its joint/motor
      /spring API is now unused), test_body_decider.cpp (2 failing), the skipped pick-step case.
  - [~] **D. Stepping on the physics body** (2026-11-09 evening, the user's calls; committed, being tested):
    - Capsule: goes under the real hips and turns with the pelvis yaw; the root no longer reads the
      hips (that looped: runaway). Hold: up/down free 3 cm below the plan (support.freeDown, Jolt
      SixDOF soft limit), never pulled above standing (restPelvisHeight_). Release off (release.on).
    - One input: WASD = steps (held: walking); setMoveInput/addGaitMotion removed; WalkController
      only turns now.
    - Stepper decides only (pickFoot/stepSpot, re-aimed every tick); the foot target travels in
      Human::limbTestPlan (stages: weight shift liftAt, up to the top, down onto the spot).
    - Decider (general): a foot down with a goal is pinned there; the planned hips travel to a new
      place on a minimum-jerk path in Fitts' time (BodyGoals.dt); rebase() when the root moves.
      Physics body in Idle/Pose: always the decider's plan, feet without a goal pinned.
    - Muscles: stiffness = inertia beyond the joint x (2 pi f)^2 (activationTime), critical damping,
      capped at maxTorque (relaxed measured values no longer drive).
    - Measured (TEMP trace in tests/test_human.cpp; configs must stay alive, see memory): standing
      ankle 0.2 cm; step foot 2-8 cm from its target, chest 0-4 deg, lands within 2 cm; after landing
      the real chest swings 7-15 deg and the foot slides ~15 cm for ~0.5 s.
    - **Next:** one clock per step (foot + hips targets from the same phase) for a smooth landing;
      why planLegs raises the hips so often; turn the TEMP trace into a real test; hand sways in the limb test (check with the new muscles).
  - [~] **E. Smoother walk, seven targets** (2026-11-09/10 night, the user's calls; UNCOMMITTED, user testing):
    - Decider: hips go between the feet's targets (a moving foot carries them half its travel); through a
      timed swing they sway toward the standing foot and back (hipsSway 2.7 cm x sin(pi x swing), Orendurff
      2004; BodyGoals::Point::swing from the step clock). A foot held up without timing: all weight on the other.
      Removed the step's load share and hips deadline (hipsTime). While a foot moves the static centre-of-mass
      loop is off (walking is not statically balanced); standing reaches keep it.
    - Seven targets always present (standing place, moving with the root): an active one is followed 100 %,
      the others by human_joints.json decider.passiveInfluence {head 0.9, hands 0.5, hips 0.3, chest 0.5}.
    - Head held in space: its muscle target is the plan's head in the world seen from the REAL chest
      (vestibulo-collic reflex, Pozzo 1990; HumanBody::drivePose overload).
    - E / Q: walking pace (locomotion.json step.speedStep/Min/Max; length x sqrt(k), duration / sqrt(k)).
    - Measured (stepping test, 4 s): plan pelvis accel 70 -> 10 m/s^2, real 42 -> 13.5, chest 11 -> 3.7 deg,
      foot slide 4.5 -> 1.5 cm, side sway 4.5 cm (real 3.9-7), landings within 0.3-3.8 cm.
    - **Open (next):** late in a quick swing the real knee lags the plan 30-58 deg (muscle damping toward zero
      speed + torque cap); this blocks faster walking (breaks at x1.2, real 1.2-1.4 m/s), an even-paced hips
      path, and the head sway (real +-12-15 deg; neck stiffness ~55 N m/rad is already real: 22-41, Simoneau
      2008 / Tierney 2005). Measure the knee's torque parts per tick first. Then: capture-point balance (#4)
      or just rename the feet-moving condition. speedMax 3 is above what the body manages now.
    - Before commit: arrow-key limb cases (a foot held up now puts all weight on the other), PROGRESS/DECISIONS.
  - [~] **F. Whole-body decider on seven targets** (2026-11-10, the user's approval; DECISIONS
    2026-11-10; UNCOMMITTED, the user checking in the game):
    - `motion::TargetSet` (planner -> decider interface: pose, mode Free/Driven/Pinned, weights,
      swing share, ground normal, strength); `motion::StepPlanner` (one step: Start, Up, Land;
      retarget, cancel); `motion::BodyDecider` rewritten: warm-started plan (never reads the
      physics body per tick; seeded at reset and get-up), priorities pinned feet -> support
      weights (blended) and hips reference -> driven targets (blended) -> reach (96 % clamp,
      trunk queue, short reported) -> posture; output bounded (hips 2 m/s, 8 m/s^2; joint rates).
    - Capsule leads (intent velocity, re-centres toward the support with both feet planted);
      pin strength x body strength x support; pelvis error classes; `Human::setStrength`
      (Jolt motor torque limits + gravity hold); targets drawn in the game.
    - Measured (stepping test, 4 s; committed walk in brackets): plan pelvis accel 11.0 (10)
      m/s^2, real 20.4 (13.5), chest swing 2.2 (3.7) deg, foot slide 3.3 (1.5) cm, side sway
      2.4 (4.5; real 3.9-7) cm, no drop. Decider tests 27/27, agents 15/15 (1 skipped).
    - **Open:** real pelvis accel and foot slide worse than the committed walk; side sway a
      little low; a hand let go returns in ~0.1 s (blendTime); arrow-key limb cases not yet
      run (the user's check before commit).
  - [x] **G. Reach in the documented order; lowest hips from the knees** (2026-11-11, the
    user's approval and game check; DECISIONS 2026-11-11, BIOMECHANICS 9.1, 9.4, 9.7):
    - Low hand target: lumbar spine and pelvis tilt first, legs last; the hips no lower
      than a grounded knee at 90° (two feet) / 67° (one foot); pelvis tilt within
      `reach.pelvis*Deg`. Config `decider.twoLegKneeMaxDeg` / `oneLegKneeMaxDeg`.
    - Measured (plan; before in brackets): hand at the toes knees 64/46 (152/149) deg,
      pelvis tilt 51 (8), trunk bend 62 (18), hips drop 12 (69) cm; foot 0.8 m ahead stance
      knee 67 (127) deg, short 12 cm (reported). Stepping test unchanged.
    - Measurement cases in `tests/test_human.cpp` ("measure: ...", skipped by default; run
      with `--no-skip -tc="measure*"`).
    - **Open:** the real body misses a foot target 0.5 m behind by 50 cm (it did before
      too: the physics body, not the plan).
  - [x] **H. Limbs reach to a 5° bend; limb test jumps and stands on the ground** (2026-11-11,
    the user's check; DECISIONS 2026-11-11 later): `decider.limbMinBendDeg` 5 replaces
    `reachClamp` 0.96 (it bent every knee and elbow 32.5°). Standing knees 32.5 -> 5.1°,
    walking x1 straighter knee 33-46 -> 7-31°. Limb test: a press jumps the target 0.1 m,
    never under the ground; a foot target on the ground is pinned there (weight on it).
    - **Open:** at pace x2.49 a knee reaches 0.0° at some moment (the bound is 5°):
      not measured yet which leg or when.
  - [~] **I. The capsule follows the real hips; the hold rides the plan** (2026-11-11, the
    user's call; DECISIONS 2026-11-11 night; committed, the user's game check pending):
    the capsule moves under the physics pelvis every tick (`moveCharacter`, it still
    collides); the hold's anchor rides the plan's pelvis (target offset 0, so the damper
    damps toward the plan's speed); removed: intent velocity, `character.recentreRate`,
    `decider.hipsOffsetMax`; `passiveInfluence.hips` 0.
    - Measured (W 3 s; before -> anchor on the feet' centre -> anchor on the plan): capsule
      from the real hips 19.5 -> 0.5 -> 0.6 cm (mean); real hips behind the plan 2.4 -> 8.8
      -> 1.2 cm; real pelvis accel 20.6 -> 15.1 -> 9.4 m/s^2 (real walking ~2-4); foot slide
      2.0 -> 2.2 -> 1.4 cm; side sway 2.4 -> 3.5 -> 4.3 cm (real 2.5-6); side accel 1.6 ->
      1.8 -> 3.4 m/s^2. All tests pass. The stepping test's side sway is now measured in the
      world (from the root it read 0.2 cm, as the root follows the hips).
    - **Open:** the hips 13.7 cm behind the feet's centre (the plan: half the swing foot's
      way); the pelvis neither turns (0.1°, real ±4.5°) nor drops (0.5°, real ±4°): the
      foot's queue moves the pelvis only past the leg's reach and the leg IK ignores the
      hip's limits. Next (the user's idea): the lifted leg moves the pelvis by its share
      (forward, side, turn) once the hip joint has its part; research the hip-pelvis rhythm
      first (BIOMECHANICS 4.3 has the gait timing, added today).

  - [x] **J. Two muscles per joint axis** (2026-11-13, the user's call; DECISIONS
    2026-11-13; the user's game check pending): `muscles.directional` gives knee, hip,
    ankle, shoulder, elbow and wrist a strength each way about their main axis (holding
    values, BIOMECHANICS 14). Knee: Jolt's twist min/max. Swing joints: the muscle
    pulling that way, picked each tick. Measured: stepping, reach and arrow-key limb cases
    are unchanged (nothing reaches its limit there). Weak-muscle checks show the right
    muscle acting each way. All tests pass (agents 15, physics 15).
    - [x] The gravity hold and the motor share one limit (2026-11-13, the user's yes):
      before, each could give the full strength. Cases unchanged; a weak (8 N m) shoulder
      flexor now lets the straight arm sink 35.5 cm, as it should.
    - **Next:** the jump plan (BIOMECHANICS 6), the whole chain first.
  - [~] **K. Standing jump on the physics body** (2026-11-14, the user's design; DECISIONS
    2026-11-14; the user's game check pending): Space held crouches, let go jumps
    (Pick moved to P). `motion::JumpPlanner` drives only the hips target (down 0.3 m in
    0.7 s, pushed up 0.1 m past standing in 0.3 s, faster and faster); the rest follows
    by its passive share; the feet stay pinned. In the push the capsule's hold carries
    the hips with the jump's target (past standing too); in the air its joint is off and
    the body flies; landing (the real ankles back on the ground) pins the feet where they
    came down and the hips sink 0.15 m and rise. The decider's hips bounds while jumping:
    3.2 m/s, 15 m/s^2 (`locomotion.json` "jump").
    - Measured: pelvis 29 cm above standing, feet ~29 cm off the ground, chest lean at
      most 16 deg, no fall, standing again by 3 s. Walking unchanged; all tests pass.
    - **Open:** the body drifts forward in the air, the feet land ~26 cm behind the hips
      and a small recovery step follows; legs alone (hold off in the push) only reached
      1.0 m/s (the stance legs' muscles carry only the hanging parts' weight).

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
