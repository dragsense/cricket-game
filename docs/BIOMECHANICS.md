# Biomechanics reference (M1 Step 1.0)

Research notes on how the human body moves, written as the basis for the procedural
fielder (M1) and later fielding work (M3+). Each section ends with **Implementation
notes** that map the findings onto our skeleton, config files and IK targets.

**How to read the numbers**
- `H` = standing height. Our default is H = 1.80 m.
- Values are for healthy young adult males unless stated. Real people vary a lot, often
  ±10–20 %, so every number here is a *default* for config, not a hard rule.
- **Confidence tags:** **[P]** means the value comes from a primary or peer-reviewed
  source that was found and checked. **[T]** means it is a standard textbook value
  (Winter, Perry) that is widely reproduced, but the original page was not checked online
  for this document. **[E]** means it is our own estimate or derivation, used where no
  source gave a number.
- All text is paraphrased. Sources are listed at the end of each section.

---

## 1. Body proportions

### 1.1 Segment lengths as a fraction of height
These are the Drillis & Contini ratios, as tabulated in Winter's *Biomechanics and Motor
Control of Human Movement* (Fig. 4.1) **[T]**. Validation work found these ratios
correlate highly with measured lengths (r 0.95–0.98). It also found they slightly
under-predict arm and forearm length and over-predict shoulder width
([Konstanz ISBS](https://ojs.ub.uni-konstanz.de/cpa/article/view/292)).

| Landmark / segment | Fraction of H | At H = 1.80 m |
|---|---|---|
| Eye height | 0.936 | 1.685 m |
| Shoulder (acromion) height | 0.818 | 1.472 m |
| Chest (nipple) height | 0.720 | 1.296 m |
| Elbow height | 0.630 | 1.134 m |
| Hip joint (greater trochanter) height | 0.530 | 0.954 m |
| Wrist height | 0.485 | 0.873 m |
| Fingertip height (arm hanging) | 0.377 | 0.679 m |
| Knee height | 0.285 | 0.513 m |
| Ankle height | 0.039 | 0.070 m |
| Thigh (hip → knee) | 0.245 | 0.441 m |
| Shank (knee → ankle) | 0.246 | 0.443 m |
| Upper arm (shoulder → elbow) | 0.186 | 0.335 m |
| Forearm (elbow → wrist) | 0.146 | 0.263 m |
| Hand (wrist → fingertip) | 0.108 | 0.194 m |
| Foot length | 0.152 | 0.274 m |
| Foot width | 0.055 | 0.099 m |
| Shoulder width (bi-acromial) | 0.259 | 0.466 m |
| Hip width | 0.191 | 0.344 m |
| Head height (chin → vertex) | 0.130 | 0.234 m |

### 1.2 Segment masses and centres of mass
These are the de Leva (1996) adjustments of Zatsiorsky–Seluyanov's gamma-scan data, for
males (reference stature 1.741 m, mass 73 kg). The CoM position is given as a fraction
of segment length from the proximal joint centre **[P]**
([de Leva 1996, J Biomech 29(9)](https://ebm.ufabc.edu.br/wp-content/uploads/2013/12/Leva-1996.pdf);
tables reproduced by [Visual3D wiki](https://wiki.has-motion.com/Adjusted_Zatsiorsky-Seluyanov%27s_segment_inertia_parameters)
and [Duarte's BMC notebook](https://nbviewer.org/github/demotu/BMC/blob/master/notebooks/BodySegmentParameters.ipynb)).

| Segment | Mass (% body) | CoM from proximal end |
|---|---|---|
| Head + neck | 6.94 | 59.8 % (from vertex) |
| Trunk (whole) | 43.46 | 44.9 % (from the neck end) |
| Upper arm | 2.71 | 57.7 % |
| Forearm | 1.62 | 45.7 % |
| Hand | 0.61 | 79.0 % |
| Thigh | 14.16 | 41.0 % |
| Shank | 4.33 | 44.6 % |
| Foot | 1.37 | 44.2 % |

The masses sum to 100 % with both arms and both legs counted. In quiet standing, the
whole-body CoM sits roughly at 0.55 H, just in front of the sacrum **[T]**.

### Implementation notes
- `config/fielder_skeleton.json` stores **ratios, never metres**. Every bone length is
  `ratio × height`, so the height slider rescales the whole skeleton.
- Mapping to our bones (H = 1.80):
  - `thigh` = 0.245 H; `shin` = 0.246 H; the ankle sits 0.039 H above the ground.
    Leg length (`thigh + shin`), used by body repositioning in Step 1.3, is
    0.491 H = 0.884 m.
  - `upper_arm` = 0.186 H; `forearm` = 0.146 H. The `hand` bone runs from the wrist to
    the knuckles, about 0.5 of the 0.108 H hand length; fingers are not modelled.
  - `foot` runs from the ankle to the ball of the foot, about 0.72 of the 0.152 H foot
    length; `toe` covers the remaining 0.28 **[E]**.
  - The hip joints are offset laterally by about ±0.5 × 0.191 H × 0.5 ≈ ±0.048 H
    **[E]**. Hip *joint centres* sit closer together than the hip *width*, which
    includes soft tissue.
  - The spine runs from the pelvis (0.530 H) to the base of the neck (about 0.818 H),
    0.288 H in total. It is split into `spine_01`, `spine_02` and `chest` at roughly
    0.10, 0.10 and 0.088 H **[E]**. `neck` is about 0.052 H, and the head extends to
    the vertex (H).
  - Each `clavicle` runs from the sternum to the shoulder joint, about
    0.5 × 0.259 H ≈ 0.13 H laterally **[E]**.
- The segment masses go in the same JSON (`mass_fraction`, `com_fraction`). They are used
  later to compute the whole-body CoM for balance (section 3) and to draw the CoM in
  debug.
- Capsule radii for the visual body are rough proportions of H and are purely visual.
  They also live in config.

---

## 2. Joint ranges of motion

The clinical "normal ROM" values below follow the AAOS / AMA-guide style tables
**[T]**, cross-checked against a published table of normal joint motions
([PMC9218764](https://pmc.ncbi.nlm.nih.gov/articles/PMC9218764/table/T1)), a workers'
compensation summary ([Enlyte](https://www.enlyte.com/insights/article/adjuster/understanding-normal-range-motion-joint-functionality)),
a ROM norms sheet ([SNCC](https://www.sncc.ca/resources/Documents/Dragon%20Boat/2024%20CCWCs/range%20of%20motion%20normal%20values.pdf))
and measured hip, knee and ankle data that show large individual spread (hip flexion
90–150°) ([Acta Orthop.](https://actaorthop.org/actao/article/download/28654/33535/82106)).
The spine values come from a 300-subject normative study
([İKU 2023](https://avesis.iku.edu.tr/yayin/e6a38307-12a9-4526-9d9f-efa478dba5e1/normative-values-for-cervical-and-lumbar-range-of-motion-in-healthy-young-adults))
and a lumbar normative database ([Brighton](https://research.brighton.ac.uk/en/publications/c27e00fc-7633-44ca-aff9-f307acc1ae79)) **[P]**.
Sources disagree by 5–15° on many joints. Where they do, the more conservative value is
used.

| Joint (our bone) | Type | Axis ranges (degrees) |
|---|---|---|
| Hip (`thigh`) | Ball | flexion 120, extension 30; abduction 45, adduction 30; internal rotation 40, external rotation 50 |
| Knee (`shin`) | Hinge (with small twist) | flexion 0–140 (up to 150 passive); hyperextension 0–5; tibial rotation about ±10, only when flexed |
| Ankle (`foot`) | Ball, limited (ankle plus subtalar lumped) | dorsiflexion 20, plantarflexion 50; inversion 30, eversion 15; twist about ±10 |
| Toe (MTP, `toe`) | Hinge | extension 70, flexion 40 |
| Shoulder (`upper_arm`) | Ball | flexion 180, extension 60; abduction 180; cross-body adduction 40; internal rotation 70, external rotation 90 |
| Clavicle (SC joint, `clavicle`) | Ball, small | elevation about 40, depression 5; protraction 15, retraction 25 **[T]** |
| Elbow (`forearm`) | Hinge | flexion 0–145. Pronation and supination of 80 each are expressed as forearm twist. |
| Wrist (`hand`) | Ball (2-axis) | flexion 80, extension 70; radial deviation 20, ulnar deviation 30; no twist (handled by the forearm) |
| Lumbar (`spine_01`) | Ball | flexion 60–70, extension 25–40; lateral bend 25–35 per side; axial rotation small (about 5–10 per side) |
| Thoracic (`spine_02`, `chest`) | Ball | flexion about 50 total; rotation about 30 per side; lateral bend about 25 total **[T]**. Split between the two bones. |
| Neck (`neck`, `head`) | Ball | flexion about 55, extension about 60; lateral bend about 40 per side; rotation about 70 per side. Split about 50/50 between neck and head. |

The *total* hip flexion of 120° assumes the knee is bent. With a straight knee, the
hamstrings limit hip flexion to about 80–90° **[T]**. This cross-joint coupling is ignored
in M1.

### Implementation notes
- `config/fielder_joints.json` stores one entry per bone.
  - Hinge entries: `{ "type": "hinge", "axis": [1,0,0], "min_deg": .., "max_deg": .. }`.
  - Swing-twist entries: `{ "type": "swing_twist", "swing_x_deg", "swing_z_deg",
    "twist_min_deg", "twist_max_deg" }`, where the swing limits form an elliptical cone.
- Hinge joints are the **knee, elbow and toe**. Everything else uses swing-twist.
- The knee and elbow need a small hyperextension allowance (0–5°) so that a straight
  leg does not jitter at the limit.
- Forearm pronation and supination is modelled as *twist* on the `forearm` bone, not on
  the wrist. This matches the anatomy (the radius rolls over the ulna).
- Spine and neck ranges are **divided between the bones in the chain**, so the total over
  the chain matches the table.
- Gameplay should stay well inside these limits. These are clinical maxima. In walking,
  the knee peaks at about 60°, and in sprinting at about 120–130° (sections 4 and 5).

---

## 3. Balance

- **Base of support (BoS):** the convex hull of the feet's contact areas. The body is
  statically stable while the CoM's ground projection stays inside it
  ([Horak-framework summary, PMC5144705](https://pmc.ncbi.nlm.nih.gov/articles/PMC5144705)) **[P]**.
- **Strategies (Horak & Nashner):**
  - The **ankle strategy** treats the body as an inverted pendulum about the ankles. It
    is used for small, slow disturbances on firm ground.
  - The **hip strategy** bends at the hips (the upper body moves opposite to the legs).
    It is used for larger or faster disturbances, or a narrow support.
  - The **stepping strategy** is used when the CoM leaves the limits of stability: take a
    step to create a new BoS.
  - These form a continuum, not separate modes
    ([Jilk et al. 2014](https://scholarblogs.emory.edu/neuromechanicslab/files/2024/06/Jilk-et-al-2014.pdf);
    [PMC5144705](https://pmc.ncbi.nlm.nih.gov/articles/PMC5144705)) **[P]**.
- **Reaching and leaning:** when an arm or the torso moves the CoM forward, the hips
  shift *backward* and the free arm or leg extends opposite as a counterweight. This
  keeps the combined CoM over the feet **[T]**.
- **Dynamic balance (walking and running):** the CoM is *outside* the BoS for most of
  each step. Balance comes from where the next foot lands, not from keeping the CoM over
  the stance foot. This is why foot placement (section 4) is the main balance control in
  locomotion **[T]**.

### Implementation notes
- Compute the whole-body CoM each tick from segment masses (section 1.2), as a cheap
  weighted sum over 20-odd bones. Debug-draw it, with its ground projection and the BoS
  polygon.
- **Standing:** pelvis offset = −k × (CoM projection − BoS centre), with `k` from
  config, clamped to a few centimetres. This is the hip strategy. Reaching (Step 1.2
  hand targets) then automatically shifts the hips backward.
- **Stepping trigger (Step 1.4):** besides the "foot far from ideal" rule, also step
  when the CoM projection leaves the BoS by more than a margin. That is a stepping
  strategy.
- Config keys: `balance.hip_shift_gain`, `balance.max_hip_shift_m`,
  `balance.step_margin_m`.

---

## 4. Walking gait

### 4.1 Phases and timing
These are percentages of one stride (heel strike to the next heel strike of the *same*
foot). Sources: ([Wikipedia gait cycle](https://en.wikipedia.org/wiki/Bipedal_gait_cycle);
[Univ. Missouri normal-gait handout](https://www.medicine.missouri.edu/sites/default/files/pm-r/2020_Normal_Gait.pdf);
[UBC KIN366](https://wiki.ubc.ca/Course:KIN366/ConceptLibrary/Gait)) **[P/T]**.

| Event / phase | % of stride |
|---|---|
| Initial contact (heel strike) | 0 |
| Loading response (first double support) | 0–10 |
| Mid stance (single support) | 10–30 |
| Terminal stance (heel rise) | 30–50 |
| Pre-swing (second double support) | 50–60 |
| Toe-off | about 60 |
| Swing (initial, mid, terminal) | 60–100 |

- Stance takes about 60 % of the stride and swing about 40 %. Total double support is
  about 20 %, and it **shrinks with speed**, reaching 0 at the walk–run transition.
- **Sagittal joint angles (Perry / Winter norms) [T]:**
  - Hip: about 30° flexion at heel strike, about 10° extension in late stance.
  - Knee: about 15–20° flexion during loading (shock absorption), nearly straight in
    mid-stance, a peak of about 60–65° flexion in mid-swing for toe clearance, then
    nearly straight before heel strike.
  - Ankle: about 10° dorsiflexion in late stance, about 15–20° plantarflexion at
    toe-off.

### 4.2 Speed, stride and cadence
- Speed = cadence × step length. A stride is two steps.
- Typical adult comfortable walking speed is about 1.2–1.4 m/s, with a cadence of about
  100–120 steps/min **[T]**. The Missouri handout quotes 60–80 m/min and 80–110
  steps/min as clinical ranges.
- Step length scales with height. A commonly used estimate is a step of about 0.41 H
  ([step-length calculator, secondary](https://foot.it.com/step-length-calculator)) **[E]**,
  giving about 0.74 m at 1.80 m.
- Height predicts stride length, but **not** cadence
  ([PubMed 11292147](https://pubmed.ncbi.nlm.nih.gov/11292147/)) **[P]**.
- As speed rises, people lengthen their step *and* raise cadence. Grieve's data fit
  step length ∝ v^0.42 ([arXiv 1608.03583](https://arxiv.org/pdf/1608.03583),
  citing [Grieve & Gear 1966](https://pubmed.ncbi.nlm.nih.gov/5976536)) **[P]**, so
  cadence ∝ v^0.58.

### 4.3 Pelvis, CoM and trunk
- **Vertical CoM bob:** about 5 cm peak-to-peak, two cycles per stride. The highest
  point is mid-stance and the lowest is double support
  ([OUHSC gait kinematics](https://ouhsc.edu/bserdac/dthompso/web/gait/KNMATICS/gait.htm)) **[P]**.
  Normalised data give about 2.2 % H vertical and 1.3 % H lateral
  ([Design Society](https://www.designsociety.org/download-publication/29655/Trajectory+of+the+Human+Body+Mass+Centre+During+Walking+at+Different+Speed)).
  Vertical bob grows with speed.
- **Lateral sway:** one cycle per stride, toward the stance leg. It is about 2.5–6 cm
  peak-to-peak (sources differ), and it gets *smaller* at faster speeds.
- **Pelvic angles at normal speed**
  ([PMC3040131 table](https://pmc.ncbi.nlm.nih.gov/articles/PMC3040131/table/T3)) **[P]**:
  - Transverse rotation: about 9° range (±4.5°). The pelvis rotates forward on the
    swing-leg side.
  - Obliquity (frontal drop): about 8° range (±4°). The swing-side hip drops.
  - Sagittal tilt: about 5° range.
- Pelvic rotation contributes little (under 3 %) to step length
  ([VU Amsterdam](https://research.vu.nl/en/publications/pelvic-step-the-contribution-of-horizontal-pelvis-rotation-to-ste/)),
  so it is mostly a visual detail, not a gait driver.
- **Thorax** counter-rotates against the pelvis, which is why the arms swing opposite to
  the legs **[T]**.

### 4.4 Arms
- The arms swing opposite to the same-side leg. Arm swing lowers metabolic cost and
  helps balance ([Meyns commentary, JNER 2021](https://jneuroengrehab.biomedcentral.com/articles/10.1186/s12984-021-00844-0)) **[P]**.
- Shoulder sagittal swing is about 30–40° total at normal speed, with more extension
  than flexion **[T]**. The elbow stays slightly flexed (about 20–30° mean), with a
  further 20–30° of oscillation that peaks on the forward swing
  ([elbow arm-swing study](https://scholarworks.gnu.ac.kr/item/8216f53a-cfe0-4451-956e-65be607dc759)) **[P]**.
  The arms lag the shoulders slightly, which is a passive pendulum effect.

### 4.5 Foot clearance
- **Minimum toe clearance** in mid-swing is only about 1.1–1.3 cm on flat ground
  ([Winter via Clinical Gait Analysis](https://clinicalgaitanalysis.com/teach-in/clearance);
  [VU thesis](https://vuir.vu.edu.au/21295/)) **[P]**.
- Clearance about doubles when obstacles are visible, and increases with speed
  ([PMC5375113](https://pmc.ncbi.nlm.nih.gov/articles/PMC5375113/)) **[P]**.
- The heel lifts much higher, peaking at about 25–30 cm in early swing, because the knee
  is bending **[T]**.

### 4.6 Starting, stopping, turning
- **Starting (gait initiation):** *before* the first step, the centre of pressure moves
  *backward and toward the swing leg*. This pushes the CoM forward and onto the stance
  leg. The anticipatory phase lasts a few hundred milliseconds (the CoM starts moving
  about 300 ms after the tibialis anterior activates). Steady speed is reached in about
  2–3 steps
  ([PMC4234748](https://pmc.ncbi.nlm.nih.gov/articles/PMC4234748);
  [J Rehabil Med](https://medicaljournals.se/jrm/content/abstract/10.2340/16501977-2856)) **[P]**.
- **Stopping (gait termination):** braking force increases in the final stance and
  push-off is reduced. The final step is shortened, and the CoM is brought over the feet.
  A planned stop takes about 1–2 steps. Ankle braking is the key element
  ([PMC4842454](https://pmc.ncbi.nlm.nih.gov/articles/PMC4842454);
  [CMU bipedal model](https://ri.cmu.edu/?p=109917)) **[P]**.
- **Turning:** a *step turn* pivots on the outside leg with a wide base, which is stable.
  Young adults use it in over 80 % of 60° turns. A *spin turn* pivots on the inside
  leg's foot, with a narrow base
  ([Golyski 2017](https://sites.gatech.edu/p-golyski/wp-content/uploads/sites/755/2018/01/Golyski-2017-A-computational-algorithm-for-classifying-step-and-spin-turns-using-pelvic-center-of-mass-trajectory-and-foot-position.pdf);
  [PMC6151057](https://pmc.ncbi.nlm.nih.gov/articles/PMC6151057)) **[P]**.
- **Head and trunk lead turns:** the head turns toward the new direction first, then the
  trunk, then the feet **[T]**.

### Implementation notes (`config/locomotion.json`, `walk` block)
- Timing: `stance_fraction` 0.60, `double_support_fraction` 0.10 per transition. Both
  shrink linearly toward 0 at `walk_run_speed`.
- Step length: `step_length = step_k × H × (v / v_ref)^0.42`, with `step_k` 0.41 and
  `v_ref` 1.35 m/s. Cadence then follows as v / step_length. This drives the Step 1.4
  stepping timer.
- Swing arc: `toe_clearance_m` 0.015 and `heel_lift_m` 0.25. The swing foot's ankle
  target follows an arc whose peak height lands in early swing (heel up), not
  mid-swing.
- Pelvis:
  - `bob_amplitude_m` 0.025, at 2× the step frequency, lowest in double support;
  - `sway_amplitude_m` 0.02, toward the stance foot;
  - `rotation_deg` ±4.5, `obliquity_deg` ±4 and `tilt_deg` ±2.5. All are scaled by speed.
- Knee *stance* flexion (15–20°) comes from the pelvis being below full leg length
  during loading. Step 1.3 repositioning plus bob produces it naturally, with a pole
  target forward.
- Arms: `arm_swing_deg` 35 total at `v_ref`, scaled with speed; `elbow_bias_deg` 25;
  phase opposite to the same-side leg. Drive the hand IK target or the shoulder
  rotation, with a small lag (critically damped spring).
- Start: a short CoM-shift phase toward the stance foot (`start_shift_s` ≈ 0.3) before
  the first lift.
- Stop: shorten the final step and bring the second foot alongside.
- Turn: head yaw leads (`head_lead_s`), then the chest, then the pelvis. Stepping
  re-aims through the "ideal foot position" rule.

---

## 5. Running gait

- **Flight phase:** running has a flight phase (both feet off the ground) and no double
  support. Stance falls from about 60 % of the stride in walking, to about 30–40 % in
  jogging, to about 20–22 % in sprinting
  ([Konstanz ISBS sprint prosthesis paper](https://ojs.ub.uni-konstanz.de/cpa/article/view/223/183);
  [Podiatry Mgmt](https://www.podiatrym.com/pdf/2017/9/Kennedy917web.pdf)) **[P]**.
- **Walk → run transition:** at a Froude number v²/(gL) ≈ 0.5
  ([Wikipedia: walk–run transition](https://en.wikipedia.org/wiki/Transition_from_walking_to_running);
  [PMC1617162](https://pmc.ncbi.nlm.nih.gov/articles/PMC1617162)) **[P]**.
  - With our L = 0.954 m (hip height), v ≈ √(0.5 × 9.81 × 0.954) ≈ **2.16 m/s** **[E]**.
  - Usherwood shows walking becomes mechanically impossible at Fr ≈ 1, about 3.06 m/s.
- **Foot strike:** distance runners land on the rearfoot or midfoot. Sprinters land on
  the forefoot, under or slightly ahead of the CoM
  ([Novacheck 1998, Gait & Posture 7:77–95](https://sponet.de/sponet/Record/3043409)) **[P/T]**.
- **Stride and cadence vs. speed:** up to about 7 m/s, speed rises mainly through
  **longer steps**. Above that, it rises mainly through **higher cadence** (Schache et
  al. 2014, summarised by [e3rehab](https://e3rehab.com/running-cadence/)) **[P]**.
  - Distance-running cadence is about 160–190 steps/min, with wide individual spread
    ([Marathon Handbook](https://marathonhandbook.com/running-cadence-demystified-why-180-isnt-magic-and-what-actually-matters/);
    [Hanley, Leeds Beckett](https://eprints.leedsbeckett.ac.uk/id/eprint/7491/6/TheBiomechanicsOfDistanceRunningAM-HANLEY.pdf)).
- **Top speed (sprint):** faster runners do not move their legs faster through the air,
  since swing time is similar across runners. They push harder into the ground, which
  shortens contact (Weyand et al. 2000, J Appl Physiol 89(5)) **[P]**
  ([Scientific American](https://www.scientificamerican.com/article/the-secret-to-human-speed);
  [Harvard Magazine](https://www.harvardmagazine.com/node/545)).
- **Elite reference (Bolt, Berlin 2009):** top speed about 12.3 m/s, step length about
  2.6–2.8 m, step frequency about 4.4–4.5 Hz (about 270 steps/min), contact time about
  0.09 s ([PMC10669785](https://pmc.ncbi.nlm.nih.gov/articles/PMC10669785/);
  [Essex](https://repository.essex.ac.uk/2473/)) **[P]**.
  - Team-sport athletes such as cricketers reach about 8–9.5 m/s **[E]**. That range
    is the realistic sprint ceiling for fielders.
- **Lean:** the trunk is nearly upright (about 5–10° forward) at steady speed. Strong
  forward lean occurs only while accelerating (about 45° or more on the first steps
  from a start), and it rises progressively over about the first 10–20 m. Elite
  sprinters actually keep the trunk *more* upright than sub-elite sprinters in the first
  4 steps
  ([ISBS 2020](https://commons.nmu.edu/isbs/vol38/iss1/188);
  [Walker et al. 2021](https://researchportal.bath.ac.uk/en/publications/kinematic-factors-associated-with-start-performance-in-world-clas/)) **[P]**.
- **Knee lift and heel recovery:** swing-knee flexion grows from about 90° when jogging
  to about 120–130° when sprinting (the heel comes up toward the buttock). Hip flexion at
  peak thigh lift rises from about 50° to about 70–80° **[T]** (Novacheck).
- **Arms:** the swing is larger and more sagittal. The elbow is held at about 90°, and
  the hands go from about chin height to behind the hip in a sprint. The arms balance
  the angular momentum of the legs **[T]**.
- **Vertical oscillation:** the CoM is *lowest* at mid-stance in running, which is the
  opposite of walking (spring-mass model). It moves about 6–10 cm when jogging and less
  when sprinting **[T]**.

### Implementation notes (`config/locomotion.json`, `run` block)
- `walk_run_speed` ≈ 2.1 m/s (Fr 0.5) with a blend band of ±0.3 m/s. In the band,
  `stance_fraction` crosses below 0.5, which creates the flight phase. There is no mode
  switch, so nothing snaps.
- `stance_fraction` vs. speed: about 0.60 at 1.4 m/s, 0.40 at 3 m/s, 0.30 at 5 m/s and
  0.22 at sprint speed. Interpolate from a table in config.
- Step length vs. speed table (H = 1.80): about 1.0 m at 3 m/s, 1.5 m at 5 m/s,
  1.9–2.1 m at 8–9 m/s **[E]**. Cadence = v / step.
- `max_sprint_speed` 9.0 m/s (fielder default), with `accel_max` from the
  acceleration section below.
- Pelvis bob *phase flips*, with the lowest point at mid-stance.
- `trunk_lean_deg` = base (5–8) + `k_accel` × forward acceleration, clamped to about 45.
- Swing-foot arc: `heel_recovery_height` grows with speed (the heel goes toward the
  buttock), and `knee_lift_deg` grows with speed.
- Arms: `elbow_run_deg` 90; amplitude grows with speed; shoulder flexion range about
  60–90° in a sprint.

---

## 6. Jumping

- **Phases (counter-movement jump, CMJ):**
  1. Unweighting: triple flexion, where the body starts to drop.
  2. Braking: the downward motion is stopped at the bottom of the crouch.
  3. Propulsion: triple extension.
  4. Flight.
  5. Landing.

  ([Sport SPA](https://sportspa.ftos.untz.ba/index.php/sportspa/article/download/195/182/181);
  [PMC11208864](https://pmc.ncbi.nlm.nih.gov/articles/PMC11208864)) **[P]**
- **Sequencing:** extension proceeds **proximal → distal** (the hip first, then the knee,
  then the ankle). Better jumpers show cleaner sequencing
  ([MDPI 2025](https://www.mdpi.com/2673-7078/5/4/95);
  [Int J Sports Med 2019](https://thieme-connect.de/products/ejournals/html/10.1055/a-0970-9104)) **[P]**.
  An arm swing slows hip extension, which strengthens the sequence and raises the jump
  height ([Sci Rep 2024](https://doaj.org/article/c00920ab1ec848139765528f00b94ab4)) **[P]**.
- **Crouch depth:** the self-selected crouch is typically about 70–90° of knee flexion
  **[T]**. Simply instructing a *deeper* crouch did **not** raise jump height
  ([UPO study](https://investiga.upo.es/documentos/5eb2893929995203e2410887?lang=en)) **[P]**.
- **Heights:**
  - The counter-movement adds about 24 % over a squat jump, from the stretch–shortening
    effect ([Niš, FU Phys Ed](https://casopisi.junis.ni.ac.rs/index.php/FUPhysEdSport/article/view/1572)).
  - Elite basketball players reach a CMJ of about 48 cm; a two-step approach jump
    reaches about 53 cm, about 12 % higher
    ([BISp](https://www.bisp-surf.de/Record/PU201304002431)) **[P]**.
  - Fit non-specialists reach about 35–45 cm **[E]**.
- **Flight time** from height h: t = 2√(2h/g). A 0.45 m jump gives about 0.61 s in the
  air.
- **Landing:**
  - The toes touch first, then the heels.
  - The hip, knee and ankle flex together to absorb the impact over about 0.1–0.3 s.
  - Peak vertical force is about 2–5 × body weight for drops of 30–100 cm
    ([PMC6364912](https://pmc.ncbi.nlm.nih.gov/articles/PMC6364912)) **[P]**.
  - More hip and knee flexion means lower peak force
    ([gymnastics landing study](https://lida.sport-iat.de/ta/Record/4035347?lng=en)) **[P]**.
  - Peak knee flexion of about 90° is the conventional boundary between "stiff" (less)
    and "soft" (more) landings (DeVita & Skelly 1992, cited by
    [Konstanz ISBS](https://ojs.ub.uni-konstanz.de/cpa/article/view/1011/924)) **[P]**.
- **Running jump:** the run-up adds horizontal speed, and on a one-leg take-off the
  final (penultimate) step lowers the CoM. The plant leg then converts horizontal speed
  into vertical speed **[T]**. Search results did not give numbers for the penultimate
  lowering; to research in M3 if needed.
- **Legs in the air:** the legs tuck (the hips and knees flex) near the apex, then
  extend before touchdown to meet the ground **[T]**.

### Implementation notes (`config/jump.json`)
- Phase durations:
  - `crouch_s` about 0.25–0.35, `crouch_knee_deg` about 80;
  - `push_s` about 0.20–0.25;
  - `land_absorb_s` about 0.20, `land_knee_deg` about 60 (soft, but under 90);
  - `recover_s` about 0.3.
- `jump_height_m` 0.45 by default. The take-off velocity is v₀ = √(2gh); the airborne
  pelvis follows the ballistic arc exactly, and the IK only *places* the limbs.
- Push-off sequencing: stagger the hip, knee and ankle extension by `push_stagger_s`
  (about 30–50 ms each).
- Arm swing: down and back during the crouch, forcefully up during the push.
- Tuck: `tuck_knee_deg` about 60 at the apex, reduced back toward straight about 0.1 s
  before predicted touchdown.
- Running jump: `approach_gain` about 1.12 × the standing height, and the penultimate
  step lowers the pelvis by `penultimate_drop_m` (an estimated 0.05–0.08).

---

## 7. Movement quality: human vs. robotic

These come mainly from Lasseter's 1987 SIGGRAPH paper on the Disney principles in 3D
animation ([EVL PDF](https://www.evl.uic.edu/aej/527/papers/lasseter.pdf);
[SIGGRAPH HyperGraph](https://education.siggraph.org/static/HyperGraph/animation/character_animation/principles/prin_trad_anim.htm)),
grounded in the biomechanics above.

- **Anticipation:** every action is prepared. People crouch before jumping, shift weight
  before stepping (section 4.6) and wind up before throwing. Without it, motion reads as
  weightless.
- **Weight shift:** the pelvis moves over the stance foot *before* the other foot lifts.
  A robotic gait lifts a foot with the body still centred.
- **Follow-through and overlapping action:** distal parts (forearms, hands, head) keep
  moving after the proximal part stops, and settle with a slight overshoot.
- **Joint lag / proximal-to-distal timing:** motion starts at the core and travels
  outward (section 6). This applies to arm swing, turning and throwing.
- **Slow in / slow out (damping):** joints accelerate and decelerate smoothly. Use
  critically damped or slightly under-damped springs, never linear interpolation.
- **Arcs:** limbs move along curves, not straight lines (for example the swing foot and
  the hands).
- **Asymmetry and variation:** real gait is never perfectly symmetric. Step length and
  timing vary by a few percent from step to step. Toe clearance alone has an
  interquartile range of about 4–5 mm
  ([VU thesis](https://vuir.vu.edu.au/21295/)) **[P]**.
- **Head stabilisation:** the head stays nearly level, and the gaze stays fixed while the
  body bobs. The neck counteracts the trunk motion.

### Implementation notes
- A shared **spring helper** (critically damped, with frequency and damping ratio from
  config) for the pelvis offset, arm swing, head aim and trunk lean. Check for an
  existing helper in `src/math` before writing one.
- `variation.step_length_pct` about 2–3 and `variation.timing_pct` about 2, as
  per-step noise from a deterministic seeded RNG (so replays are reproducible).
  `variation.left_right_bias` gives per-fielder asymmetry.
- Lag: arm and head targets are driven through springs that are a little slower than
  the pelvis, so overlap appears automatically.
- Head: aim IK (Step 1.2) at a point about 5–10 m ahead along the travel direction, with
  the head's world pitch and roll damped toward level.

---

## 8. Cricket fielding preview (for M3+)

This section is brief by design. Detailed research belongs to the fielding milestone.

- **Ready stance:** as the bowler delivers, fielders walk in (or "set") on the balls of
  their feet. The feet are about shoulder-width apart, the knees flexed, the hips back,
  the hands low and in front, and the head still and level. This is coaching convention
  **[T]**, not measured data.
- **Reaction time:** skilled cricketers show about 200 ms, similar to laboratory reaction
  times ([South African J Sports Med](https://journals.assaf.org.za/index.php/sajsm/article/view/366/304)) **[P]**.
  - A study of slip catching with balls at 20 m/s found no effect of ball colour or light
    on movement initiation ([Sheffield Hallam](https://shura.shu.ac.uk/3287/)) **[P]**.
  - First slip watches the ball, while wider slips watch the bat
    ([PitchVision summary of Hughes](https://cdn0.pitchvision.com/slips-study), a blog).
- **First-step quickness:**
  - The average fielding sprint is only about 15 ± 4 m, so acceleration matters more
    than top speed.
  - Infielders and outfielders show similar sprint kinematics.
  - Greater first-step hip and knee flexion goes with longer first steps
    ([Lockie et al. 2014, JSSM 13:128](https://jssm.org/13-1-128.p_d_f)) **[P]**.
  - A rolling start makes the first and second steps 12 % and 8 % longer
    ([Callaghan et al.](https://www.researchgate.net/profile/Samuel_Callaghan/publication/261761156_The_acceleration_kinematics_of_cricket-specific_starts_when_completing_a_quick_single/links/0c960535751958ee98000000/The-acceleration-kinematics-of-cricket-specific-starts-when-completing-a-quick-single.pdf)) **[P]**.
- **Low pickups:** these bend at the hips and knees, with the stance foot beside the
  ball. Pickups on the run are one-handed, and the body is lowered by a long final step
  rather than by bending the back **[T]**.
- **Dives:** no biomechanical dive study was found. They are to research in M3 (likely
  modelled as a ballistic CoM arc plus a staged landing on the forearm, hip and
  shoulder).
- **Catching positions:** "fingers up" for balls above the chest, "fingers down" (cupped)
  for balls below. The hands give with the ball (elbow flexion), which spreads the
  impulse over time, much like a soft landing **[T]**.

### Implementation notes
- The fielder state machine later reuses everything from M1: ready stance is a pose
  preset plus balance, reaction is a delay of about 0.2 s from config, and the first step
  is the Step 1.6 acceleration with high lean.
- Config placeholders (M3): `reaction_time_s` 0.2; `ready_knee_deg` about 30;
  `ready_hip_flex_deg` about 30; `first_step_length_m`.
- Catching "give" reuses the hand IK target driven through a spring. This needs no new
  system.

---

## Gaps and follow-ups
- These numbers were not found online and rely on standard textbook values: the exact
  Winter / Perry joint-angle curves, arm-swing shoulder ranges, and running hip and knee
  peaks. They should be checked against the books if motion looks off.
- Diving and penultimate-step data are deferred to M3.
- All values are male-referenced. Female and other body types can be added later by
  swapping the ratio and mass tables in config.
