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
- `config/human_skeleton.json` stores **ratios, never metres**. Every bone length is
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
- **Fingers** (added for ball pick-up and catching): 3 segments per finger. Phalanx
  lengths are fractions of hand length L = 0.108 H **[T]**: thumb (metacarpal, proximal,
  distal) 0.251 / 0.196 / 0.158; index 0.265 / 0.143 / 0.097; middle
  0.277 / 0.170 / 0.108; ring 0.259 / 0.165 / 0.107; little 0.206 / 0.117 / 0.093. The
  knuckles sit about 0.37 L from the wrist. Values follow Buchholz, Armstrong & Goldstein
  1992, *Ergonomics* 35(3):261–273
  ([catalogue record](https://documentacion.fundacionmapfre.org/documentacion/publico/i18n%20/bib/47249.do);
  companion kinematic model, [Buchholz & Armstrong 1992](https://deepblue.lib.umich.edu/handle/2027.42/30224)).
  The table itself is paywalled; the numbers are as commonly reproduced in hand-modelling
  work. About 0.02 L is added to each fingertip for the pad. Finger mass is included in
  the hand's 0.61 %.
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
- `config/human_joints.json` stores one entry per bone.
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

### 3.1 Standing: how the body follows the feet
- **Weight first:** before a foot lifts, the CoM moves sideways over the stance foot
  (the mediolateral anticipatory postural adjustment). The shift unloads the leg so it
  can lift; it is feedforward and starts a few hundred ms before the lift. Only rushed
  (reactive) steps lift with little shift
  ([PMC5483205](https://pmc.ncbi.nlm.nih.gov/articles/PMC5483205/);
  [PMC5015477](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5015477/);
  [Frontiers 2021](https://www.frontiersin.org/journals/human-neuroscience/articles/10.3389/fnhum.2021.709780/full)) **[P]**.
- **Balance is the CoM, not the pelvis:** statically, the CoM projection stays in the
  base of support (section 3); the support centre is between the loaded feet.
- **Single-leg stance:** the pelvis shifts toward the stance hip and the unloaded side
  drops a few degrees (obliquity, about 4 deg, section 4.3); the drop lowers the CoM a
  little ([PMC5545133](https://pmc.ncbi.nlm.nih.gov/articles/PMC5545133/)) **[P]**.
- **Wider feet:** people stand about shoulder width by choice; wider stances stiffen the
  legs-pelvis and increase hip abduction
  ([Henry et al. 2001](https://journals.physiology.org/doi/pdf/10.1152/jn.2001.85.2.559);
  [PLOS One 2025](https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0315851)) **[P]**.
  The hips lower by the legs' angle; past what the hips allow, the stance knee bends
  (a side lunge) **[T]**.
- **Limits are the joints:** a foot goes no further than its leg reaches within the
  hip's range (section 2); the body sinks no lower than the stance knee's range.

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
  - Sample and speed (added 2026-11-11): healthy controls, n = 20, walking slowly
    (0.71 m/s); the paper gives no pelvic data at their own speed (1.28 m/s) **[P]**.
- **When the pelvis turns and drops (added 2026-11-11):**
  - Turn: the pelvis is turned furthest forward on a side at that foot's strike and
    furthest back at the opposite foot's strike, about 4° each side of straight
    ([Musculoskeletal Key, Biomechanics](https://musculoskeletalkey.com/biomechanics-2/)) **[T]**;
    it follows the swing leg and turns it inward through the hip
    ([OUHSC pelvic rotation](https://ouhsc.edu/bserdac/dthompso/web/gait/knmatics/pelrot1.htm)) **[T]**.
    So it is largest when the feet are furthest apart fore-aft and straight at midstance.
  - Drop: the side whose foot is off the ground falls, because nothing holds it up;
    the stance hip's abductors (gluteus medius and minimus) stop it at about 4° and
    lift it back, with a slight overcorrection
    ([O&P Library, Normal Gait](https://www.oandplibrary.org/alp/chap13-01.asp);
    [StatPearls, Trendelenburg gait](https://www.ncbi.nlm.nih.gov/books/NBK555987/)) **[T]**.
    The drop peaks just after the opposite toe-off and lasts under ~20 % of the cycle
    (secondary sources; the exact % from a normative dataset was not found).
  - Clinical line: a drop over 4° in single stance counts as a weak abductor
    (Westhoff 2005, cited in StatPearls) **[P]**.
  - All three pelvic motions are about 4° and follow the swing limb (O&P Library) **[T]**.
  - Speed: as speed rises, the pelvis turns more in phase with the thigh (Liang 2014,
    20 men, 1-5 km/h); no table of turn amplitude against step length was found.
- Pelvic rotation contributes little (under 3 %) to step length
  ([VU Amsterdam](https://research.vu.nl/en/publications/pelvic-step-the-contribution-of-horizontal-pelvis-rotation-to-ste/)),
  so it is mostly a visual detail, not a gait driver.
- **Thorax** counter-rotates against the pelvis, which is why the arms swing opposite to
  the legs **[T]**.
- **Pelvis and thorax turn: numbers and mechanism (added 2026-11-15, for the decider's
  shares):**
  - Amplitudes, peak to peak, 9 young women on a treadmill
    ([Yang et al. 2013, J Appl Biomech](https://journals.humankinetics.com/view/journals/jab/29/3/article-p261.xml);
    full numbers in the [thesis](http://hdl.handle.net/2065/40309)) **[P]**: pelvis 11° at
    0.40 m/s, 19° at 1.47 m/s (+80 %); thorax 12° slow, 6° fast (−50 %); at slow speed
    all trunk levels turn about the same. The turn peaks in order pelvis, L5, L3, L1,
    thorax: the pelvis leads, the thorax follows.
  - Other data: pelvis 4.3° ± 1.1 total at preferred speed in one lab, 3–14° across
    studies (marker sets differ) **[T]**; Liang et al. 2014 (20 men, 1–5 km/h): pelvis
    amplitude U-shaped with speed with normal steps (least near 3 km/h), larger with big
    steps ([VU](https://research.vu.nl/ws/files/42834063/Liang_G_P_2014.pdf)) **[P]**.
  - Timing: thorax–pelvis relative phase about −20° at 1 km/h (in phase), about −150° at
    5.4 km/h (nearly opposite) (cited in
    [Prins et al. 2019, Sci Rep](https://www.nature.com/articles/s41598-018-37549-9)) **[P]**.
    The shift comes from the pelvis turning in phase with the swinging thigh at higher
    speeds while the thorax keeps turning against the thigh
    ([Bruijn et al. 2008](https://www.academia.edu/5620440/Coordination_of_leg_swing_thorax_rotations_and_pelvis_rotations_during_gait_The_organisation_of_total_body_angular_momentum)) **[P]**.
  - Angular momentum: pelvis and thorax give under 10 % of the body's angular momentum
    about the vertical, legs and arms about 90 % (Bruijn 2008) **[P]**. So the thorax's
    counter-turn is not mainly there to cancel the legs; the arms do that.
  - Mechanism (Prins 2019, forward dynamic model, 30 subjects at 4 km/h, 83 % of the
    thorax turn explained) **[P]**: the thorax turn = the pelvis turn passed up through the
    trunk's axial stiffness and damping (a spring-damper at L5/S1), plus the arm swing's
    moment on the shoulders. Stiffer trunk: more in phase, smaller thorax turn. More arm
    swing: more out of phase, larger thorax turn. Trunk stiffness changes with speed
    (Kubo et al. 2006).

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

### 4.7 Sideways and backward walking
- **Two sideways patterns.** A *sidestep* (side shuffle): the lead foot steps out, the
  trail foot closes toward it, and the feet never cross. A *crossover step*: the trail
  foot crosses over the lead leg, either in front or behind. Both occur in daily life and
  in balance recovery. In young adults, long sidesteps measured about 60 cm wide; long
  crossovers were slightly narrower (about 56 cm) with a longer fore-aft component
  ([PMC4362666 table](https://pmc.ncbi.nlm.nih.gov/articles/PMC4362666/table/T2)) **[P]**.
- **A crossover is never in line.** The crossing foot must pass in front of or behind the
  stance leg, so it lands offset fore-aft from the other foot. The base of support gets
  very narrow, so a crossover is less stable than a sidestep
  ([PMC6501204](https://pmc.ncbi.nlm.nih.gov/articles/PMC6501204/figure/F3);
  [Thies, Salford](https://salford-repository.worktribe.com/output/1475113/what-causes-a-crossover-step-when-walking-on-uneven-ground-a-study-in-healthy-young-women)) **[P]**.
- **First step from standing:** the foot on the side of travel steps out first; stepping
  with the far foot first would force an immediate crossover **[T]**.
- **Body:** the trunk stays facing forward. Arm swing is small or absent, and the pelvis
  shifts sideways over each new stance foot instead of bobbing much **[T]**.
- **Backward walking:** speed, cadence and stride length are all lower than forward
  walking, and stance is longer. Knee range of motion is smaller. Hip and ankle moment
  patterns are roughly mirror images of forward walking, with much less joint power. The
  forefoot tends to touch down first instead of the heel
  ([Salford knee study](https://salford-repository.worktribe.com/output/1396748/comparison-of-biomechanical-characteristics-of-the-knee-joint-during-forward-walking-and-backward-walking);
  [PMC4723164](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4723164/)) **[P]**. The
  evidence is from small studies (8–13 people).

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
- Sideways (section 4.7): a **side shuffle**, the pattern athletes and fielders use for
  controlled lateral movement: the lead foot steps out and the trail foot closes beside
  it, never crossing ([Coaches Insider](https://coachesinsider.com/baseball/agility-drills-to-improve-baseball-fielding/);
  [CricConnect, keeper footwork](https://cricconnect.com/question/80/what-is-the-right-footwork-for-a-wicket-keeper-standing-back-to-fast-bowlers-is-it-having-the-feet-cross-over-behind-each-other-as-the-keeper-moves-sideways-or-a-shuffle-where-the-feet-following-each)).
  `sideSpeed` 0.8 m/s. From standing, the foot on the side of travel leads (A: left,
  D: right). The trunk stays forward and the arm swing fades out. The **crossover**
  (hips turn about 45°, trail foot crosses in front) is for longer or faster lateral
  moves and belongs with running (Step 1.6).
- Backward: `backSpeed` 0.9 m/s (slower than forward), arm swing fades out. Shorter
  steps and a forefoot-first landing are a later refinement.
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

### What Step 1.6 implements (first pass, `config/locomotion.json` "run")
Three paces: walk (default), jog (hold Ctrl, 3 m/s) and run (hold Shift, 5 m/s).
The gait shape blends walk → jog → run by the actual speed, so there is no mode switch
(as the notes above ask). The numbers below are for the run preset. The jog preset sits
between walk and run (1.0 m steps, flight 20 % of the swing, stance about 40 %, heel
recovery 0.22 m). The walk now has foot roll too: heel strike with the toes about 10°
up, then flat, then the heel rises and the foot rolls to about 18° plantarflexion at
toe-off (section 4.1). Sprint and the crossover come later.
- **Timing:** step 1.5 m at 5 m/s (H = 1.8), about 200 steps/min. No double support.
  Each foot swings for one step period. The last 40 % of each swing is flight, so
  contact is about 0.18 s (about 30 % of the stride), matching the table above.
- **Stance foot (foot roll):** flat at landing and mid-stance. Over the second half of
  the other foot's swing the heel rises and the foot rolls over its ball (the ankle rises
  by ball distance × sin(pitch); the toes bend up and stay on the ground) to about 35°
  plantarflexion. In the flight window the toes push off and leave the ground.
- **Swing foot:** pointed after toe-off, then levels to about 5° toes-up for a
  midfoot landing. The arc apex comes early (38 % of the swing) and is about 0.36 m
  high. This is the heel recovery toward the buttock, which gives the jogging knee
  flexion of about 90°. The foot lands about half a contact length ahead of the hip, so
  it passes under the hip at mid-stance.
- **Width:** feet land nearer the midline but still apart: 75 % (jog) and 65 % (run)
  of standing width.
- **Pelvis and trunk:** the bob is flipped (lowest at mid-stance, 7 cm when jogging, 6 cm when
  running). Lean is about 5–7° at steady speed, plus more while accelerating (capped at 20°). The pelvis
  turns about ±6° toward the forward leg, and the chest turns back past neutral against
  it.
- **Arms:** elbows near 85–90°, and an 80° shoulder swing (hands from about chin height
  to behind the hip), opposite the legs.

### Step 1.6b review (2026-10-25)
A check against the literature found the approach right but several details off. Fixed:
- **Bob phase:** the pelvis was highest at touchdown. Now lowest mid-stance and highest
  mid-flight (spring-mass model;
  [Geyer et al. 2005](https://www.ri.cmu.edu/app/uploads/2017/11/GeyerEA05JTheorBiol.pdf)).
- **Cadence:** was 180 and 200 steps/min at 3 and 5 m/s. Recreational runners are about
  160–170 and 175–185 ([Hanley](https://eprints.leedsbeckett.ac.uk/id/eprint/7491/6/TheBiomechanicsOfDistanceRunningAM-HANLEY.pdf))
  **[P/E]**. Steps are now about 1.08 m and 1.66 m.
- **Stance knee:** about 20° at contact and 40–45° mid-stance **[T]**; the pelvis now
  drops 2–3.5 cm when jogging and running.
- **Back foot in flight:** it leaves the ground at toe-off and starts forward. It was
  planted for the whole step, which made a lunge with both feet down.
- **Pelvis:** it stays over the root fore/aft while moving, and shifts only partly
  sideways when running (lateral CoM motion in running is small, about 1–2 cm **[E]**).

---

## 6. Jumping (refreshed 2026-11-13)

### 6.1 Standing jump (two feet, counter-movement jump, CMJ)
- **Phases:**
  1. Unweighting: hip, knee and ankle flex together; the body briefly falls on gravity.
     The ground force drops to **43 ± 24 %** of body weight (shallow crouch 58 %).
  2. Braking: the downward motion is stopped at the bottom of the crouch.
  3. Propulsion: triple extension, from the low point to take-off.
  4. Flight.
  5. Landing (section 6.3).

  ([McHugh et al. 2024, PMC11208864](https://pmc.ncbi.nlm.nih.gov/articles/PMC11208864);
  [Sport SPA](https://sportspa.ftos.untz.ba/index.php/sportspa/article/download/195/182/181);
  [McMahon 2018, NSCA SCJ](https://journals.lww.com/nsca-scj/fulltext/2018/08000/understanding_the_key_phases_of_the.10.aspx)) **[P]**
- **Timing and depth** (McHugh 2024: 83.5 kg, 1.81 m men; shallow / self-selected / deep) **[P]**:

  | | Shallow | Self-selected | Deep |
  |---|---|---|---|
  | COM drop (m) | 0.27 | **0.36** | 0.47 |
  | Countermovement (s) | 0.77 | **0.70** | 0.95 |
  | Propulsion (s) | 0.27 | **0.30** | 0.43 |
  | Jump height (m) | 0.345 | **0.373** | 0.378 |
  | Peak propulsive force (× BW) | 2.46 | **2.24** | 2.25 |

  A deeper crouch does **not** raise the jump: it costs 36–42 % more time. A shallow one
  loses a little height (less unweighting). This matches the earlier finding that
  instructing a deeper crouch did not help
  ([UPO study](https://investiga.upo.es/documentos/5eb2893929995203e2410887?lang=en)) **[P]**.
- **Pose at the bottom** (included joint angles, 180° = straight; McErlain-Naylor, King
  & Pain 2014) **[P]**: knee 81 ± 16° (about 100° flexion), hip 75 ± 15°, ankle
  84 ± 9°, shoulder −67 ± 26° (arms back). The earlier self-selected range of 70–90°
  knee flexion is **[T]**.
- **Pose at take-off** (same study) **[P]**: knee 174°, hip 172°, ankle 137° (heels
  high), shoulder flexion 103 ± 37° (arms up). Greater shoulder flexion and ankle
  plantarflexion at take-off went with higher jumps.
- **Sequencing:**
  - Extension runs **proximal → distal** (hip, then knee, then ankle); better jumpers
    show cleaner sequencing ([MDPI 2025](https://www.mdpi.com/2673-7078/5/4/95);
    [Int J Sports Med 2019](https://thieme-connect.de/products/ejournals/html/10.1055/a-0970-9104)) **[P]**.
  - Measured (McHugh 2024, self-selected) **[P]**: the hip and knee peak moments come
    almost together (knee 27 ± 47 ms after the hip). The ankle peak moment comes
    135 ± 55 ms after the knee. Peak knee power comes 239 ms after the low point and peak
    ankle power 251 ms after it, just before take-off (propulsion 301 ms).
- **Joint moments and powers needed** (peak, per leg, per kg of body mass) **[P]**:

  | Joint | Moment (N m/kg) | Power (W/kg) | Source |
  |---|---|---|---|
  | Hip | 3.80 ± 0.86 / 2.20 ± 0.44 | 11.2 / 9.8 | McHugh 2024 / McErlain-Naylor 2014 |
  | Knee | 3.44 ± 0.44 / 3.31 ± 0.62 | 24.2 / 22.0 | same |
  | Ankle | 2.39 ± 0.32 / 2.79 ± 0.40 | ~16–19 / 18.0 | same |

  The share of the positive work differs between studies: knee 49 % (Hubley & Wells
  1983), hip 51 % (Fukashiro & Komi 1987), hip 42 % and knee 30 % (Vanezis & Lees 2005),
  as cited by McErlain-Naylor 2014. A one-leg CMJ also needs hip abduction torque
  ([Biology Open, bio.062434](https://journals.biologists.com/bio/article/15/4/bio062434/371549)) **[P]**.
- **Arm swing:**
  - It raises the jump by **12–38 %** across studies (Feltner 1999, Hara 2008, Lees 2004,
    Mosier 2017/2019, Vanrenterghem 2008; summary in
    [PAAH, paah.263](https://paahjournal.com/articles/10.5334/paah.263)). In that study
    it added 18.9 %: the peak force was the same, but the push lasted longer (more
    impulse) **[P]**.
  - Lees et al. 2004: +8.6 cm, of which 72 % comes from a higher take-off velocity and
    28 % from a higher COM at take-off (arms up). Cheng 2008 (simulation): about 2/3
    velocity and 1/3 height ([PubMed 18514208](https://pubmed.ncbi.nlm.nih.gov/18514208/)) **[P]**.
  - The arms go down and back during the crouch and swing forward and up hard during the
    push. Swinging the arms slows hip extension, which strengthens the sequence
    ([Sci Rep 2024](https://doaj.org/article/c00920ab1ec848139765528f00b94ab4)) **[P]**.
- **Heights:**
  - The counter-movement adds about 24 % over a squat jump
    ([Niš, FU Phys Ed](https://casopisi.junis.ni.ac.rs/index.php/FUPhysEdSport/article/view/1572)) **[P]**.
  - Elite basketball players reach about 48 cm in a CMJ and about 53 cm (+12 %) with a
    two-step approach ([BISp](https://www.bisp-surf.de/Record/PU201304002431)) **[P]**.
  - Fit non-specialists reach about 35–45 cm **[E]**; McHugh's subjects reached 37 cm
    **[P]**.
- **Take-off velocity and flight:** v₀ = √(2gh). For 0.37 m that is about 2.7 m/s, and
  the flight time t = 2v₀/g is about 0.55 s (0.45 m: 3.0 m/s, 0.61 s) **[E]**.
- **Legs in the air:** the hips and knees flex (tuck) near the apex and extend again
  before touchdown to meet the ground **[T]**.

### 6.2 Running jump (one-foot take-off)
- **Mechanism:** the take-off leg is planted well **ahead** of the body, with the body
  leaning back. The leg acts like a pole that turns horizontal speed into vertical
  speed. The COM is low at touchdown and high at take-off, which makes the push longer.
  The penultimate step lowers the COM; the last step is shorter
  ([Dapena, ISBS](https://ojs.ub.uni-konstanz.de/cpa/article/view/1421/1283)) **[P]**.
- **Measured take-offs** **[P]**:

  | | High jump, elite (Nicholson 2024) | Basketball, 6 m run-up (Sugiyama 2014) |
  |---|---|---|
  | Approach speed | ~7.1 m/s (Dapena) | 7.9 m/s |
  | Foot contact | **0.17–0.18 s** | **0.23 s** (braking 0.054 + push 0.179) |
  | Knee at touchdown | 163–167° | up to 175° |
  | Knee, most bent | 136–140° (only **~40–45°** flexion) | 137–139° |
  | Hip, most bent | – | 85–87° |
  | Ankle, most bent | – | 94–97° |
  | COM height | 0.90 m at touchdown → 1.35 m at take-off | – |
  | Speed through the take-off | horizontal 7.1 → 3.8–4.3 m/s; vertical −0.3 → 4.4–4.6 m/s | – |
  | Last two steps | 2.16 m, then 1.95 m | – |
  | Result | – | 0.72 m |

  Sources: [Nicholson et al. 2024, PMC10876886](https://pmc.ncbi.nlm.nih.gov/articles/PMC10876886);
  [Sugiyama et al. 2014, PMC4234967](https://pmc.ncbi.nlm.nih.gov/articles/PMC4234967);
  Dapena (above).
- Compared with the standing jump, a one-foot running jump has a **shallower** hip, knee
  and ankle bend and a much **shorter** contact. One leg may carry up to about 3 × body
  weight
  ([Sportsmith](https://www.sportsmith.co/articles/using-the-approach-vertical-jump-for-athlete-monitoring-and-training/)) **[T]**.
- **Coaching guidance, not measured:** the penultimate step is the longest of the
  approach; in it the shin is at about 45° and the knee at about 90° of flexion
  (Sportsmith) **[T]**.
- **Two-foot approach (step-close or hop):** after the approach steps both feet plant,
  then a short crouch and the push
  ([Coutts 1982, PubMed 7070259](https://pubmed.ncbi.nlm.nih.gov/7070259/)) **[P]**.
  It gains about 12 % over a standing CMJ (BISp, above) **[P]**.

### 6.3 Landing
- The toes touch first, then the heels (section 12).
- Measured (Heebner et al. 2017, [PMC5763249](https://pmc.ncbi.nlm.nih.gov/articles/PMC5763249)) **[P]**:

  | Task | Knee at contact | Ankle PF at contact | Peak knee | Peak hip | Time to peak knee | Peak force (× BW) |
  |---|---|---|---|---|---|---|
  | Two-leg drop landing | 22° | 21° | 92° | 61° | 0.23 s | 3.6 |
  | Two-leg stop jump | 32° | 8° | 102° | 78° | 0.24 s | 1.9 |
  | Forward jump, one-leg landing | 15° | 23° | 58° | 45° | 0.18 s | 4.4 |
  | One-leg drop landing | 15° | 28° | 69° | 43° | 0.23 s | 4.9 |

- Peak vertical force is about 2–5 × body weight for drops of 30–100 cm
  ([PMC6364912](https://pmc.ncbi.nlm.nih.gov/articles/PMC6364912)) **[P]**. More hip and knee
  flexion means a lower peak force
  ([gymnastics landing study](https://lida.sport-iat.de/ta/Record/4035347?lng=en)) **[P]**.
- About 90° of peak knee flexion is the conventional line between "stiff" (less) and
  "soft" (more) landings (DeVita & Skelly 1992, cited by
  [Konstanz ISBS](https://ojs.ub.uni-konstanz.de/cpa/article/view/1011/924)) **[P]**.

### 6.4 Our body against these numbers (checked 2026-11-13)
- **Strength:** `human_joints.json` `maxTorque` is shin 160, thigh 197 and foot 125 N m,
  one value for both directions. A jump needs, per leg × 73 kg: knee about 240–285, hip
  160–277 and ankle 153–204 N m. The knee is at about 60 % of that, so a jump would come
  out much lower. Knee extension alone is about 205 N m (section 14); separate caps per
  direction would match the real values better. **Since 2026-11-13** the muscles have a
  strength per direction (`muscles.directional`): knee extension 247, hip extension 197,
  ankle plantarflexion 125 N m. These are holding (isometric) values, still below a
  jump's peaks.
- **Decider bounds:** the plan's hips are bounded to 2 m/s and 8 m/s². A jump needs a
  take-off of about 2.7 m/s and an upward acceleration of about (2.24 − 1) g ≈ 12 m/s²
  **[E]**.
- **Capsule hold** (`physics_world.json` character.support: 2000 N, 4 Hz) holds the pelvis
  to the plan. If it holds in the air, the hold does the jump, not the legs. The foot
  locks also hold the feet down. In the air the capsule already falls on world gravity
  (`PhysicsWorld`, character update).

### Implementation notes (planned; no code yet)
The old kinematic `JumpController` and `config/jump.json` were removed on 2026-11-09.
The numbers to build from, all from 6.1–6.3:
- Standing: drop the COM about 0.36 m in about 0.70 s, push for about 0.30 s, hip first,
  knee about 30 ms later, ankle last (its peak about 50 ms before take-off). Arms back
  to about −67° at the bottom and up to about 100° at take-off.
- Running, one foot: plant the take-off foot ahead of the hips, the knee nearly straight
  (163–175°) at touchdown, bending only about 40° in a 0.17–0.23 s contact. Penultimate
  step longer than the last.
- Two feet from steps: step-close, then a short CMJ.
- Air: tuck near the apex, extend before touchdown.
- Landing: toes first, knee 15–32° at contact, bending to 60–100° over about 0.2 s.
- Open: how far the COM drops in the penultimate step (no measured value found; the
  earlier guess was 0.05–0.08 m **[E]**); no data for jumps at walking speed
  (0.9–2.5 m/s). The planner, the hold release, strength and the decider bounds need a
  design decision before coding.

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

## 9. Reaching (M2)

How a standing person brings a hand to a target that may be beyond arm's length, here a
ball on the ground. Joint ranges are in section 2; this section adds the order in which
segments are recruited, how they depend on each other, and the working limits used
while reaching.

### 9.1 Recruitment order (which segment joins when)
- **Arm first, trunk when needed.** Targets within arm's length are reached with the
  shoulder and elbow and almost no trunk motion. For targets beyond it, trunk
  displacement grows with target distance, first by forward trunk inclination, then by
  raising or moving the whole body
  ([Levin et al., Exp Brain Res 2001](https://link.springer.com/doi/10.1007/s00221-001-0976-6))
  **[P]**. The trunk is recruited when the target is too far for shoulder–elbow
  coordination alone, because it is massive and costly to start and stop
  ([PMC8626510](https://pmc.ncbi.nlm.nih.gov/articles/PMC8626510)) **[P]**. The trunk
  also places the hand, not only stabilising posture
  ([PMC3773211](https://pmc.ncbi.nlm.nih.gov/articles/PMC3773211)) **[P]**.
- **Shoulder girdle.** The first ~30° of arm elevation is mostly glenohumeral; after
  that the scapula and clavicle join at an overall ratio of about 2:1 (humerus :
  scapula), so the clavicle is the first "extra reach" after the arm
  ([ABMP, scapulohumeral rhythm](https://www-b.abmp.com/massage-and-bodywork-magazine/issues/marchapril-2021/scapulohumeral-rhythm))
  **[T]**.
- **Standing reach uses the whole chain.** In the functional reach test, spinal motion
  is forward flexion plus lateral flexion plus thoracolumbar rotation plus lower-body
  rotation ([Cavanaugh et al. 1999](https://scholars.duke.edu/publication/665672);
  [Duncan et al. 1990](https://researchers.mq.edu.au/en/publications/functional-reach-a-new-clinical-measure-of-balance/))
  **[P]**.
- **Lumbar vs. hip in forward bending (lumbopelvic rhythm).** The lumbar spine leads,
  then the hips (pelvis tilting on the femurs) take over: lumbar : hip ratio ≈ 1.9 for
  0–30° of bending, 0.9 for 30–60°, 0.4 for 60–90°
  ([Esola et al. 1996](https://pubmed.ncbi.nlm.nih.gov/9122766/)) **[P]**. Tight
  hamstrings shift this toward more lumbar motion
  ([PMC7064356](https://pmc.ncbi.nlm.nih.gov/articles/PMC7064356)) **[P]**.
- **Going lower: stoop vs. squat.** A stoop (straight knees) lowers the hand with the
  lumbar spine and hips; a squat adds knee flexion. The largest difference between the
  two is at the knee (model peaks ~16° vs. ~52°)
  ([arXiv 1910.01184](https://arxiv.org/pdf/1910.01184);
  [BMC Musculoskelet Disord 10:15](https://link.springer.com/doi/10.1186/1471-2474-10-15))
  **[P]**. Real pickups mix both.
- **The three named postures** (definitions, not measurements): stoop, knee < 45° and
  trunk ~90°; semi-squat, knee ~90° and trunk ~45°; squat, knee ~135° and trunk < 30°
  ([Straker 2003](https://www.sciencedirect.com/science/article/abs/pii/S0169814102001919);
  [review PMC8720246](https://pmc.ncbi.nlm.nih.gov/articles/PMC8720246/)) **[T]**.
- **What people choose (added 2026-11-11).** Many default to the stoop, the cheapest in
  metabolic cost (review above) **[T]**. Measured self-chosen box lifts (6–15 kg) from the
  ground: knee flexion ~48° (novices), ~71° (experts); asked to change technique, people
  change mostly the knees and keep their own lumbar bend
  ([Dundee, intrinsic lumbar curvature](https://discovery.dundee.ac.uk/en/publications/variation-in-lifting-kinematics-related-to-individual-intrinsic-l/))
  **[P]**. The lower the target, the more trunk, hip and knee flexion all grow
  ([OSU 2002](https://spine.osu.edu/sites/default/files/uploads/Publications/2002/OccupationalErgonomics_2002_3-2_99-103.pdf))
  **[P]**. Not found: measured angles for a light object (the 39-subject
  [Burgess-Limerick 1995](https://www.researchgate.net/publication/15557341_Self-Selected_Manual_Lifting_Technique_Functional_Consequences_of_the_Interjoint_Coordination)
  study is paywalled).
- **Fielders (section 8):** a pickup on the run lowers the body by a long final step
  rather than by bending the back **[T]**. That is a step (a planner's job), not a deeper
  squat on planted feet.

**Order used by the game.** Each stage takes only the error the previous stage could not
remove: arm (shoulder, elbow) → clavicle → spine (lumbar first for bending, thoracic for
twist) → pelvis turn and forward tilt (hip flexion) → body translation in any direction
(legs bend through leg IK) → lunge step → hold maximum reach (the dive comes later).

### 9.2 Dependencies between segments
- The **shoulder position depends on** clavicle, spine and pelvis. Proximal stages are
  solved first and the arm IK runs last, on the final shoulder position.
- **Pelvis tilt is hip flexion**, so it is limited by the hips: 120° with knees bent but
  only ~80–90° with straight knees (hamstrings, section 2). Lowering the body (knees
  bending) therefore *unlocks* more pelvis tilt.
- **Body translation is limited by the legs.** Each hip-to-ankle distance stays within
  leg length (the `repositionBody` rule); planted feet do not move.
- **Balance couples everything.** Any forward reach moves the CoM; the hips shift back
  to keep it over the feet (section 3, 9.5).

### 9.3 Working ranges while reaching
Reaching uses comfortable working ranges, not the clinical maxima of section 2. Spine
values vary widely between sources and with age
([Brighton lumbar database](https://research.brighton.ac.uk/en/publications/a-normative-database-of-lumbar-spine-ranges-of-motion/);
[VA thoracolumbar exam template](https://vivian.worldvista.org/dox/Routine_DVBCWNS5_source.html)).

| Segment (our bones) | Motion | Working range (deg) | Note |
|---|---|---|---|
| Clavicle (`clavicle`) | protraction / elevation | 15 / 30 | elevation up to ~40–60 clinically, mostly with the arm overhead **[T]** |
| Lumbar (`spine_01`) | flexion / side bend / twist | 50 / 20 / 7 | lumbar twist is small (median ~7° each way) **[P]** |
| Thoracic (`spine_02` + `chest`) | flexion / side bend / twist | 30 / 20 / 35 | thoracic rotation 35–50° **[T]** |
| Whole thoracolumbar | flexion / side bend / twist | ~90 / ~30 / ~30–45 | VA template **[T]** |
| Pelvis on the hips | forward tilt (hip flexion) | 80 straight knees, 110 knees bent | hamstring coupling **[T]** |
| Pelvis | axial turn (hip rotation, feet planted) | ~30 | from hip rotation ranges **[E]** |

Split rule **[E]**: twist mostly thoracic (~80 %); forward bend lumbar-heavy early and
hip-heavy late (9.1); side bend shared evenly.

### 9.4 Squat and kneeling
- Squat depth by knee angle: partial ~40°, half 70–100°, deep >100° (NSCA review).
  Measured "horizontal" squats average knee ~113°, hip ~128°, ankle dorsiflexion ~23°;
  half squats ~71° / 88° / 16°
  ([CEJSSM](https://wnus.usz.edu.pl/cejssm/hr/issue/1244/article/19713);
  [J Exerc Sci Fit 2021](https://doaj.org/article/87e2425e7e374edf892fe6d1c38cd85f))
  **[P]**. These are means, not hard limits.
- **Kneeling:** no kinematic source was found for half-kneeling pickups. The cricket
  "long barrier" puts one knee down beside the other foot's heel **[T]**.
- M2 started with **no squat or drop limit** (the user's choice), so the solve could be
  observed. Measured 2026-11-11 without it: a hand at the toes squatted the stance knees
  to 150–164° with only 26° of trunk and hip bend, past every posture above. **Limit
  since 2026-11-11 (the user's approval):** the hips go no lower than puts a grounded
  knee at **90°** on two feet (the semi-squat; measured self-chosen ground lifts 48–71°,
  above) or **67°** on one foot (the largest mean stance knee of a maximal one-leg foot
  reach, 9.7). Past that the reach is reported short (a step is the planner's job). The
  one-knee drop comes later.

### 9.5 Balance compensation
- A forward reach moves the CoM forward. The body responds with a backward shift of the
  centre of pressure and the hips, and a counterweight from the free arm or leg
  ([Liu et al., APAs in forward reaching](https://pure.lib.cgu.edu.tw/en/publications/within-session-acquisition-of-anticipatory-postural-adjustments-d-3/);
  section 3) **[P]/[T]**.
- A reach done mainly by bending at the hips moves the centre of pressure less than one
  done by leaning from the ankles
  ([Aino repository abstract](https://aino.repo.nii.ac.jp/records/648), secondary) **[T]**.
- Static rule: the CoM's ground projection stays inside the base of support (section 3).
  If it cannot, a step is needed (stepping strategy).

### 9.6 Timing
- **Posture before the arm.** Postural muscles fire about 50–100 ms before the arm's
  prime mover (anticipatory postural adjustments). Standing, soleus is inhibited and
  tibialis anterior fires first; hamstrings and erector spinae then fire with the
  rotator cuff, before the anterior deltoid
  ([Silva et al.](https://recipp.ipp.pt/bitstreams/95a6bb60-98da-4bee-8793-ddf7dbd24aca/download))
  **[P]**.
- **The trunk spans the hand movement.** When used, it starts with or before the hand
  and stops after it ([Levin et al. 2001](https://link.springer.com/doi/10.1007/s00221-001-0976-6))
  **[P]**.
- So the hand *looks* like it leads (it moves fastest), but the trunk and hips start at
  the same time or slightly earlier and settle later. This refines the M2 plan's "arm
  leads, body follows": onset is near-simultaneous, the body settles later.
- **Movement time (Fitts' law).** Aimed movements take MT = a + b·log2(2D/W) for
  distance D and target width W; a stylus pointing study measured a ≈ 13 ms and
  b ≈ 95 ms per bit ([MacKenzie 1992](https://yorku.ca/mack/GI92.html)) **[P]**. We use
  a = 0.1 s (whole-body onset), b = 0.1 s/bit, W = 0.1 m (hand around the ball)
  **[E]**: a 0.2 m reach takes ~0.3 s, 1 m ~0.53 s.
- **Intercepting a moving ball** **[E]**: the hand meets it where its straight path
  passes closest to the fielder (where it crosses the line of control square to its
  path); the reach starts one movement time (plus a lunge's time) before that, so it
  arrives as the ball does; a moving ball is grabbed on contact.

### 9.7 Foot reach on one leg (added 2026-11-11)
The star excursion balance test: standing on one leg, the other foot reaches as far as
it can along 8 lines and touches down lightly. Change from standing at maximum reach,
healthy controls (n = 18), mean in degrees
([PLOS ONE 2019, PMC6347240](https://pmc.ncbi.nlm.nih.gov/articles/PMC6347240/)) **[P]**:

| Reach direction | Stance knee flexion | Stance hip flexion | Trunk flexion | Trunk turn to stance side |
|---|---|---|---|---|
| Forward | 55 | 9 | 5 | 1 |
| Forward-across | 61 | 12 | 8 | 3 |
| Across (behind the stance foot's side) | 67 | 32 | 6 | 5 |
| Back-across | 59 | 46 | 1 | 5 |
| Back | 50 | 53 | 0 | 1 |
| Back-out | 35 | 47 | 1 | 2 |
| Out to the side | 24 | 38 | 1 | 5 |
| Forward-out | 45 | 13 | 2 | −5 |

- **Hip and knee flexion make the reach:** they explain 62–95 % of the reach distance in
  all 8 directions ([Robinson & Gribble 2008](https://meridian.allenpress.com/jat/article/50/4/358/112375/Trunk-Rotation-Differences-at-Maximal-Reach-of-the),
  cited there) **[P]**. The trunk stays nearly upright (0–8°).
- So the stance leg **does lower the hips**, with the knee at most ~55–67°; reaching
  back, the pelvis tilts forward on the stance hip (hip flexion ~50°) while the trunk
  stays up.
- **The hips stay over the stance foot** while the foot is in the air (it is a balance
  test: the foot only touches). They carry over to the reaching foot only once it lands
  and takes weight (a step, section 3.1 and 4).
- Limits: small sample; a maximal test, so everyday foot reaches use less. Not found:
  lunge pelvis travel (only the preferred forward lunge step, ~90 cm,
  [PubMed 23584319](https://pubmed.ncbi.nlm.nih.gov/23584319/)) **[P]**.

### 9.8 Reaching with a step (added 2026-11-22)
- **Which foot.** Reaching further forward (arm's length + 5-20 % H), trunk and pelvis
  turn the same way (right hand: both to the left; the pelvis at the furthest targets), and
  hip flexion and pelvic tilt join
  ([Bohannon 2020](https://doi.org/10.1016/j.jbmt.2020.03.003)) **[P]**; so the hand-side
  foot ahead turns the pelvis with the reach. People preferred the support leg on the
  hand's side while picking up
  ([Cockell 1995](https://doi.org/10.1177/003151259508100232)) **[P]**; a later study of
  walking found no clear preference
  ([Bellinger 2020](https://doi.org/10.1123/mc.2018-0053)) **[P]**.
- **When.** Pointing with a rapid step (the opposite leg by instruction): the arm's
  acceleration peaks around foot-off
  ([Yiou 2007](https://doi.org/10.1016/j.humov.2007.03.003)) **[P]**.
- **How far without a step.** Shoulder height: 0.22-0.24 H past the arm forward (Moriyama
  2022, DECISIONS 2026-11-15). Sideways 19.8 cm, young adults, n = 240
  ([Tedla 2019](https://pmc.ncbi.nlm.nih.gov/articles/PMC6686713/)) and ~28 cm, n = 62
  ([Azukizawa 2021](https://doi.org/10.1080/07853890.2021.1968483)) **[P]**. On the floor:
  NIOSH's largest lifting distance, 63 cm from the ankles' middle, further the balance is
  lost ([NIOSH](https://stacks.cdc.gov/view/cdc/110725/cdc_110725_DS1.pdf)) **[T]** (two
  hands, a load). Forward reach in the same studies: 25-32 cm.
- **A target to the side.** Feet free to move, people bend sideways to a target at 90 deg no
  more than to one straight ahead ([Lavender 1995](https://doi.org/10.1518/001872095778995562))
  **[P]**; stepping to a load 65 cm away at 60 deg: 3.9 deg sideways (1.4 straight ahead), 3.9
  deg of twist ([Marras 2000](https://doi.org/10.1080/001401300184314)) **[P]**. Turning, the
  foot on the side opposite the turn is ahead
  ([Konz & Bhasin 1974](https://doi.org/10.1080/0002889748507100)) **[P]** (abstract). Boxes, two hands.
- **Picking from the floor:** people use several strategies, no single one
  ([Gilles 2018](https://doi.org/10.1016/j.apergo.2018.02.002)) **[P]**.
- **Not found:** the step length, trunk and pelvis angles of a standing step-and-reach; the
  foot used for a low one-hand pickup; the reach limit between floor and shoulder height.
  Several papers were read as abstracts only.

### Implementation notes (`config/human_joints.json` "reach")
- Each tick (late update, after foot repositioning), every stage reduces the error and
  passes the rest on: arm → clavicle → spine → pelvis → body translation → lunge → hold.
  Proximal segments are posed first; the arm IK runs last.
- `armReach` = 0.97 × arm length (the elbow never locks).
- Spine weights: twist lumbar 0.2 / thoracic 0.8; bend lumbar 0.6 / thoracic 0.4. Each
  bone is clamped by its swing-twist limit (`human_joints.json`).
- Pelvis tilt limit from 9.3, raised with knee flexion (80° straight, 110° bent).
- Body translation is an offset *added* to the foot repositioning offset; leg length
  clamps it. The lowest hips come from the grounded knees (9.4: 90° on two feet, 67° on
  one).
- The decider (`motion::BodyDecider`, 2026-11-11): for a hand target below the hips the
  trunk queue is lumbar spine, pelvis tilt, upper spine, chest, clavicle, and the legs
  last (9.1's order; the earlier legs-first queue squatted instead of bending). The
  pelvis tilt keeps to `reach.pelvisMinDeg` / `pelvisMaxDeg` (80° forward, 9.3).
- Timing: all stages start together; each has a spring, stiffest for the arm and softest
  for the pelvis and body, so the hand arrives first and the body settles last (9.6).
- Balance: the hips shift opposite to the CoM excursion; the free arm extends backward.
  Gains in config.

---

## 10. Throwing (M4)

How a fielder throws after a pick-up: arm slots, the phases of the throw, where the hand
travels, where it lets go, how long each part takes and how fast the ball leaves. Cricket
throwing is much less studied than baseball pitching, so phase timings and arm-slot
definitions come mostly from baseball **[P]** and are adapted **[E]**.

### 10.1 Arm slots
- Arm angle is measured in the frontal plane at release. Statcast: **90° = straight over
  the top, 0° = pure sidearm, negative = submarine** (e.g. -19°)
  ([MLB, arm angles 2024](https://www.mlb.com/news/the-range-of-pitcher-arm-angles-in-the-2024-mlb-postseason))
  **[P]**. Clock terms: overhand 12:00, three-quarter 1:30, sidearm 3:00, submarine 4:30;
  the most common slot is "high three-quarter"
  ([FanGraphs, playing the slots](https://tht.fangraphs.com/a-pitching-model-playing-the-slots1/))
  **[T]**. Slots are a continuum; categories overlap.
- Sidearm and three-quarter slots load the elbow more than overhand
  ([OJSM 2023, 10.1177/23259671221147874](https://journals.sagepub.com/doi/10.1177/23259671221147874))
  **[P]**: visually the same arm path, so no extra modelling.
- **Underarm** (hand below the hip, swinging forward like a bowling action): fielders use
  it for short flicks at the stumps. Speed comes from trunk-to-hand sequencing, with the
  forearm and hand adding most of the release speed; windmill pitches release next to the
  hip ([Auburn ETD, overhand vs underhand](https://etd.auburn.edu/handle/10415/136))
  **[P]**.
- Physics of the choice: for near, low targets underarm is more accurate; for far or high
  targets overarm is
  ([Venkadesan & Mahadevan, arXiv 1008.1442](https://arxiv.org/pdf/1008.1442)) **[P]**.

### 10.2 Phases and timing
Baseball phases (wind-up, stride, arm cocking, acceleration, deceleration, follow-through);
a cricket throwing study uses the same split: wind-up, late cocking, arm acceleration,
release ([Brunel, cricket throwing](https://bura.brunel.ac.uk/handle/2438/16239)) **[P]**.

| Phase | Hand path | Duration | Source |
|---|---|---|---|
| Wind-up / cocking | back and up behind the shoulder until maximum shoulder external rotation | long, variable (pitch: ~1.5 s incl. leg lift) | [Pogo Physio](https://www.pogophysio.com.au/blog/bio-mechanics-throwing/) **[T]** |
| Acceleration | forward past the shoulder to release | **~50 ms** | [PMC3445080](https://pmc.ncbi.nlm.nih.gov/articles/PMC3445080) **[P]** |
| Follow-through | across the body, decelerating | ~350 ms | same **[P]** |

- Peak shoulder internal rotation ~6,000–7,000 °/s, elbow extension ~4,600 °/s at the end
  of acceleration (pitchers) **[P]**. The hand moves fastest at release.
- A fielder's throw has no leg lift: after the pick-up the wind-up is short. We estimate
  **0.25–0.4 s wind-up** and **0.08–0.15 s forward swing** for a quick throw (slower than
  a pitcher's 50 ms, because the game's arc is a simple circle, not a whip) **[E]**.
- **Proximal to distal:** pelvis, then trunk, then upper arm, forearm, hand; both overhand
  and underhand throws follow it
  ([Auburn ETD](https://etd.auburn.edu/handle/10415/136)) **[P]**. In the game the hand
  target leads and the reach chain follows (section 9), which matches the look but not
  the cause; a later step can add pelvis/trunk rotation leading the arm.

### 10.3 Arm path and release point
- The hand path is close to an arc around the shoulder in the slot's plane; the elbow
  flexes during cocking and extends through release, so the real radius varies **[T]**.
  We use a circle of radius about arm reach (0.9–0.97 × arm length) around the throwing
  shoulder **[E]**.
- Release is **ahead of the shoulder**, not at the top: dart throwers release 17–37°
  before the arm is vertical
  ([Venkadesan & Mahadevan](https://arxiv.org/pdf/1008.1442)) **[P]**. The ball leaves
  along the hand's velocity, i.e. tangent to the arc at that point **[P]** (physics).
- Underarm releases are near or in front of the hip; sidearm at about shoulder height,
  out to the side; overarm above and in front of the head **[T]**.

### 10.4 Ball speeds
- Elite-programme cricketers: **30.5 ± 2.4 m/s** from a set position, **31.8 ± 2.1 m/s**
  after a 3 m shuffle; elite seniors are faster still, baseball pros faster again
  ([Freeston et al. 2007, EJSS](https://onlinelibrary.wiley.com/doi/10.1080/17461390701733793);
  [Throwing speed and accuracy, baseball and cricket](https://www.researchgate.net/publication/264390731_Throwing_Speed_and_Accuracy_in_Baseball_and_Cricket_Players))
  **[P]**. Radar over-reads by ~1.7 m/s head-on
  ([Sports Biomech 2021](https://www.tandfonline.com/doi/abs/10.1080/14763141.2021.1887343))
  **[P]**.
- **Accuracy is best at 75–85 % of maximum speed** (Freeston et al.) **[P]**, so a
  normal return is ~23–27 m/s.
- Maximum-distance throws are slower than maximum-speed throws (higher launch)
  ([Murdoch thesis 2021](https://researchportal.murdoch.edu.au/esploro/outputs/graduate/Determinants-of-throwing-speed-in-elite/991005542162807891))
  **[P]**. Stride length and shoulder internal rotation speed predict throw speed **[P]**.
- Underarm flicks at the stumps: no measured data; ~8–15 m/s **[E]**.

### Implementation notes (`config/throw.json`)
- Two half circles around the throwing shoulder, radius `arc.radiusFactor` (0.95) x arm
  length. The slot arc is square to the facing direction (-90° underarm, 0° sidearm,
  +90° overarm); the user picks the slot point. The swing arc passes through it and the
  facing direction and is turned about the shoulder-to-slot line by the mouse wheel.
- The hand eases to the swing arc's start (behind) with the elbow bent (`cockReach` 0.6,
  cocking), swings forward speeding up (`windupTime` 0.3 s, `swingTime` 0.12 s **[E]**)
  while the elbow extends, straight at the chosen release point, then follows through.
- Release: the ball leaves the hand along the arc's tangent at `release.speed` (25 m/s,
  ~80 % of ~31 m/s).
- The body follows through the existing reach chain (section 9); no new body system.

## 11. Bowling arm action (M5)

How a bowler's arm delivers the ball: the legal limit on the elbow, the arm circle, where
the ball is released, how long it takes and how fast the ball leaves. M5 has a standing
bowler (no run-up), so run-up and foot-contact data are listed for later milestones only.
Fast bowling is well studied; spin less so, and wrist spin hardly at all **[P]**.

### 11.1 Law: the straight arm
- MCC Law 21.2 (fair delivery, the arm) forbids throwing; the ICC regulations put a number
  on it: **elbow extension may not exceed 15°**, measured **from the moment the bowling
  arm reaches the horizontal until release**; hyperextension is discounted, and the limit
  is the same for all bowlers
  ([ICC, illegal bowling actions](https://www.icc-cricket.com/about/cricket/rules-and-regulations/illegal-bowling-actions);
  [ECB Bowling Review Regulations 2026](https://resources.ecb.co.uk/ecb/document/2026/03/24/b21ef749-7520-4734-9f40-1aa692cff18f/Bowling-Review-Regulations-2026.pdf);
  [Wikipedia, Throwing (cricket)](https://en.wikipedia.org/wiki/Throwing_(cricket)))
  **[P]**. 15° was chosen because larger extensions become visible to the naked eye and
  look like a throw **[P]**.
- So, unlike the throw (section 10), the bowling arm has **no cocked elbow and no
  whip**: the elbow is nearly straight from the horizontal to release. Most legal actions
  carry a small fixed bend (a few degrees) that changes far less than 15° **[T]**. For the
  game: elbow bend at most ~10° (inside the limit, with margin), roughly constant **[E]**.

### 11.2 Arm circle (circumduction)
- The hand starts low in front of the body (in a run-up delivery, after the gather/jump),
  goes **down and back** past the hip, **up behind** the shoulder, **over the top** and
  releases on the way forward, then **follows through down across the body** to the
  opposite hip **[T]**. The path is close to a circle around the bowling shoulder in a
  plane near vertical, through the target line; the trunk's side-flexion tilts it
  **[T]**.
- Studies time the arm by its angle at events, not by duration: how far back the arm is
  at front-foot contact (later arm in elite men goes with more speed; the opposite was
  found in elite women)
  ([Felton et al. 2019, male vs female](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2019-felton-male-vs-female-elite-bowlers/);
  [Felton 2015, female technique](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2015-felton-female-technique-ball-speed/))
  **[P]**.
- Timing (elite men): back-foot contact to front-foot contact **~192 ms**, front-foot
  contact to release **~103 ms** (women ~172 / 128 ms)
  ([Felton et al. 2019](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2019-felton-male-vs-female-elite-bowlers/))
  **[P]**. The arm goes from about horizontal-behind to release in roughly the last
  ~0.1 s **[E]** from these numbers.
- For a standing bowler with no run-up the circle is slower: we estimate **~0.5 s for
  the down-and-back part** and **~0.15–0.2 s from behind (horizontal) to release**, then
  a **~0.4 s follow-through** **[E]**.
- Proximal to distal, as in throwing: hips, trunk (flexion and rotation), then the arm
  ([Lyons 2023](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2023-lyons-female-kinematic-anthropometric-brs/))
  **[P]**. In the game the hand target leads and the reach chain follows (section 9).

### 11.3 Release point and height
- Shoulder angle (arm relative to the trunk) at release is **~173–194°**, i.e. the arm is
  about in line with the trunk, straight up; each extra degree of "arm lag" (arm still
  behind) is worth ~0.22 m/s ball speed; an arm already past vertical pushes the ball
  rather than slinging it
  ([Lyons 2023](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2023-lyons-female-kinematic-anthropometric-brs/))
  **[P]**. Because the trunk leans forward at release, in world terms the hand is
  slightly **ahead of the vertical over the shoulder**: we use **~5–25° before vertical**
  (pace nearer vertical, spin a bit further forward) **[E]**. Like the throw, the ball
  leaves along the hand's velocity, i.e. tangent to the arc **[P]** (physics).
- Release height is mostly the sum of knee flexion, trunk flexion and upper-arm abduction
  ([Manawadu 2022/2023](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-variations-and-loading/2023-manawadu-phd-fast-bowling-variations/))
  **[P]**. Academy fast bowlers release at **~107 % (bouncer) to ~112 % (yorker) of
  standing height**; thoracic posture explains 55–60 % of the variance **[P]**. A 1.85 m
  bowler releases at ~2.0–2.1 m.
- Taller bowlers release higher; bowling-shoulder angle at release was the best
  predictor of speed in female pace bowlers (89 % of variance, n = 11)
  ([Cardiff Met](https://pure.cardiffmet.ac.uk/en/publications/anthropometric-and-kinematic-influences-on-release-speed-in-mens-/))
  **[P]**.
- Front-foot vs back-foot: legally the front foot lands with some part behind the popping
  crease (Law 21.5) **[P]**; with no run-up, M5's bowler stands still with feet apart and
  the front foot forward **[E]**.

### 11.4 Ball speeds
- Pace: elite men **34.9 ± 1.7 m/s**, elite women **27.9 ± 1.4 m/s**
  ([Felton et al. 2019](https://www.whit.com.au/cleanbowled/cricket-biomechanics/fast-bowling-technique-and-context/2019-felton-male-vs-female-elite-bowlers/))
  **[P]**; the fastest internationals reach ~40–44 m/s (145–160 km/h) **[T]**.
- Finger spin: elite men **17.7–23.4 m/s, mean ~20.4 m/s**, spin 1,430–2,140 rpm
  ([Sanders 2018](https://www.whit.com.au/cleanbowled/cricket-biomechanics/spin-bowling-and-batting/2018-sanders-kinematic-parameters-spin/))
  **[P]**. Wrist spin: little data
  ([UNESA wrist spin study](https://journal.unesa.ac.id/index.php/jses/article/view/53480/17357))
  **[P]**; similar speed range **[T]**.
- A run-up adds a large share of pace-bowling speed; a standing delivery is clearly
  slower. No measured value found; we estimate ~20–25 m/s for a standing pace delivery
  **[E]**.

### 11.5 Trunk: side-on, counter-rotation, separation (M6)
- Around back-foot contact the bowler is turned side-on to the batter (shoulder angle
  under ~210° counts as side-on in one classification); front-on bowlers sit ~20° more
  open. The back foot's angle largely sets how side-on the shoulders get
  ([ISB 2005 abstract](https://media.isbweb.org/images/conf/2005/abstracts/0842.pdf))
  **[P]**.
- Shoulder counter-rotation (the shoulders turning further away from the batter during
  the delivery stride) measures ~10–40° (side-on ~11°, front-on ~22° at back-foot
  contact; ~38–42° mean in a mixed group); above ~30° is linked to lower-back injury
  ([ISB 2005](https://media.isbweb.org/images/conf/2005/abstracts/0842.pdf);
  [Senington, Lee & Williams, St Mary's](https://research.stmarys.ac.uk/id/eprint/1976/1/Are%20shoulder%20counter%20rotation%20Repository.pdf))
  **[P]**.
- Hip-shoulder separation (shoulders vs pelvis) peaks at ~27–30° in fast bowlers, ~21° at
  front-foot contact in spinners
  ([Senington et al.](https://research.stmarys.ac.uk/id/eprint/1976/1/Are%20shoulder%20counter%20rotation%20Repository.pdf);
  [Sanders 2018, spin](https://www.whit.com.au/cleanbowled/cricket-biomechanics/spin-bowling-and-batting/2018-sanders-kinematic-parameters-spin/))
  **[P]**. Angle definitions differ between labs, so the numbers are ranges, not targets.
- In the game (M6, standing delivery): the whole player turns ~60° side-on as the hand
  goes down (back-foot contact), the chest a further ~20–35° (separation and
  counter-rotation together), both unwinding to square by the release **[E]**.

### Implementation notes (planned `config/bowling.json`)
- Reuse the M4 arc (`motion::ThrowArc`, `makeSwingArc`) as a **full circle** around the
  bowling shoulder, radius `arc.radiusFactor` (~0.97 x arm length, near-straight arm);
  the user picks the release point and tilts the arc with the wheel, as in M4.
- Elbow: `arm.maxElbowBendDeg` ~10 (inside Law 21's 15°), no cocking.
- Timing **[E]**: `circle.downBackTime` ~0.5 s, `circle.overTime` ~0.18 s (horizontal
  behind to release), `followThroughTime` ~0.4 s.
- Release: fixed `release.speed` (default ~22 m/s for a standing delivery **[E]**; spin
  ~20 m/s, pace 35 m/s with a run-up later), tangent to the arc, flown by our physics.
- The body follows through the existing reach chain (section 9); no new body system.

---

## 12. Foot roll: the three rockers (M6, every gait)

One foot mechanism for walking, running, jumping, bowling and fielding. The foot turns
about whichever part of it carries the load; the leg moving over the foot drives the
roll (no per-gait timing). Sources: rocker model (Perry; [PMC2922483](https://pmc.ncbi.nlm.nih.gov/articles/PMC2922483);
[UoMus gait lecture](https://www.uomus.edu.iq/uploads/lectures/41/5d0af0a5a4524d60a2310788248e7291.pdf));
MTP joint ([Konstanz CPA 6369](https://ojs.ub.uni-konstanz.de/cpa/article/view/6369);
[Acta Orthop.](https://actaorthop.org/actao/article/download/29015/33895/82474);
[QUT 1st MTP ROM](https://podiatry-anatomy-app.qut.edu.au/content/joints/1st-mtp-joint-rom.html)) **[P/T]**.

### 12.1 The rockers (pivot follows the load)
| Rocker | Pivot | When (walk, % stride) | What turns |
|---|---|---|---|
| 1. Heel | heel | 0–10 (contact to foot flat) | foot lowers toes-down about the heel; ankle about 5° PF, controlled |
| 2. Ankle | ankle | 10–30 (foot flat) | shin rolls forward over the fixed foot; ankle to about 10° DF |
| 3. Forefoot | metatarsal heads (ball) | 30–60 (heel rise to toe-off) | heel rises once the ankle stops dorsiflexing; toes bend (MTP DF) |
| (4. Toe) | toe tip | last part of pre-swing | final push before the foot leaves |

- **Trigger for heel rise:** not a clock. The heel lifts when the shin has leaned past
  the ankle's loaded dorsiflexion range (about 10° walk) while the body keeps moving
  forward; the calf then holds the ankle and the pivot moves to the ball.
- **MTP (toe) dorsiflexion:** about 38–60° at push-off barefoot walking (windlass effect
  stiffens the foot); passive range 60–90°.
- **Ankle range:** DF about 8–16°, PF about 48–60° (non-weight-bearing).
- **Effect on the body:** rolling over heel then ball moves the ankle up and forward
  along the roll, so the hip path is smoother (less double-support dip than a flat,
  straight-legged foot).

### 12.2 The same mechanism in each movement
- **Walk:** all three rockers; heel strike about 10° toes-up; toe-off 15–20° PF.
- **Run:** rearfoot/midfoot strike shortens or skips rocker 1; forefoot strike (sprint)
  starts at rocker 3. Larger shin lean (ankle DF up to about 20° mid-stance **[T]**) and a
  stronger push (toe-off PF 25–35°). Same code; landing point on the foot comes from
  where it touches first.
- **Jump:** crouch keeps feet flat (rocker 2, deep DF); push-off goes rocker 3 → toe
  (heels up, toes bent); landing is the reverse: toes first, then heels (section 6).
- **Bowling / fielding:** back foot rolls to the ball and toe as the body passes over
  it; lunges and stops land heel first (rocker 1).

### Implementation notes
- Each planted foot reads its load and the shin angle over it; the pivot (heel, ankle,
  ball, toe) is picked from those, not from gait phase. Gait config only tunes numbers
  (landing pitch, DF limit, toe bend).
- Replaces the stance-foot flatten timer and the ankle-limit heel-lift patch.
- Gap: no measured MTP data for running or jumping; running mid-stance DF is textbook
  **[T]**.

---

## 13. The body as a mass on spring legs (M6 physics)

- **Body mass:** 73.0 kg, the mean of de Leva 1996's male subjects (the segment mass
  percentages in `human_skeleton.json` are of this) **[P]**.
- **Leg stiffness:** running about 11.2 kN/m below 5 m/s, 17.0 kN/m above, nearly constant
  across speeds ([review of 14 studies, PMC10368467](https://pmc.ncbi.nlm.nih.gov/articles/PMC10368467));
  stiffness scales with mass^0.67 across animals (Farley, Glasheen & McMahon 1993,
  [PMC1689165](https://pmc.ncbi.nlm.nih.gov/articles/PMC1689165)). Walking: no measured
  value found; the spring-mass walking model (Geyer, Seyfarth & Blickhan 2006) uses about
  20 kN/m for 80 kg **[T]**.
- **Damping:** the spring-mass model has none; real legs lose some energy (gap).
- The spring-mass model fits running better than walking (walking's energies are out of
  phase).

### Implementation notes
- `agents::BodyPhysics`: one Jolt body (73 kg) at the centre of mass, world gravity, a
  push-only Jolt spring from each foot on the ground (rest = that leg's real length).
  Test: on two legs it sinks m g / (2 k cos^2 a), 18.1 mm at 20 kN/m.

---

## 14. Muscles: joint strength, activation, stiffness (M6 physics)

- **Strength (max torque, adult men):** hip extension ~197 N m; knee extension 2.3-3.4
  N m/kg, flexion 1.3-1.9 N m/kg (isokinetic); trunk extension 268-283 N m; shoulder
  flexion 77, extension 164 N m; elbow flexion 51-56 N m; wrist flexion 21-25, extension
  11-13 N m; neck 28-41 N m; ankle plantarflexion 70-86 N m (isokinetic, untrained;
  isometric likely higher). Sources: MVIC reference values (PubMed 17364436), 178-subject
  strength study, Danneskiold-Samsoe 2009, trunk and neck strength studies, PMC8783437,
  PMC5105238, PMC5200854 **[P]**. Gaps: toes (no data), one value per joint for both
  directions (means where flexion and extension differ).
- **Strength per direction (added 2026-11-13; holding, isometric, men)** **[P]**:

  | Joint | One way | Other way | Source |
  |---|---|---|---|
  | Knee | extension 247 (isokinetic 185) | flexion 95 isokinetic (isometric not found) | Harbo 2012 (178 subjects, 15-83 y) |
  | Hip | extension 197 (isokinetic 178) | flexion 156 (isokinetic 138) | MVIC reference values; Harbo 2012 |
  | Ankle | plantarflexion 117-131 | dorsiflexion 40-42 (20-29 y), 34 (30-39 y) | Moraux 2013 |
  | Shoulder | extension 164 | flexion 77 (abduction 61, adduction 83) | MVIC reference values; Harbo 2012 |
  | Elbow | extension 48 | flexion 77 (Harbo 2012: 51) | systematic review PMC5960876 |
  | Wrist | extension 11 (isokinetic only) | flexion 25 | Harbo 2012 |

  Gaps: trunk and neck per direction (not found), knee flexion and wrist extension
  isometric, hip and shoulder rotations. In a jump the muscles give more than these
  (section 6.4).
- **Activation:** muscle excitation to force takes about 10-50 ms. It is how fast a
  muscle switches on, **not** how stiff a joint is (mistake found 2026-11-05).
- **Stiffness (researched 2026-11-06):** joint stiffness is **not a fixed number per
  joint**: it rises with muscle activation and load, and falls with the size of the
  movement (Brown & McGill trunk; in vivo knee studies; hip ~5 N m/rad per N of limb
  force). Measured values:
  - **Ankle, quiet standing (intrinsic):** 91 ± 23 % of the toppling stiffness mgh
    (Loram & Lakie 2002); 64 ± 8 % for ~1 deg pushes (Casadio 2005); 101 → 19 % as the
    rotation grows from 0.03 to 7 deg (Loram 2007); 93 → 44 % (Vlutters 2015). Some
    groups put it at ~15 % mgh. For our body, mgh ≈ 73 kg × 9.81 × ~0.9 m ≈ 650 N m/rad.
    Winter's ~859 N m/rad includes the nervous system's control, not only the muscle.
  - **Key finding:** intrinsic ankle stiffness is **below** what stops the body tipping
    (< 100 % mgh). Loram & Lakie: "not quite enough... neural modulation is needed".
    **A real person cannot stand still on muscle stiffness alone; even quiet standing
    needs the balance controller.**
  - **Hip and knee, swing phase of walking:** hip 34-66, knee 0-3.5 N m/rad (2022 IEEE,
    9 subjects, 0.5 m/s): low when relaxed.
  - **Shoulder,** near neutral at < 5 % activation: 30.9 N m/rad, damping 3.0 N m s/rad.
  - **Elbow, passive:** 0.4-1.8 N m/rad (19 subjects; scales with upper-arm volume).
  - **Gaps:** wrist (only a 3×3 matrix study, values not found), neck, trunk (Watt 2022
    lumbar meta-analysis not read yet), knee and hip in stance under load.
  - **Stiffness rising with the torque held (moved here from human_joints.json,
    2026-11-16; not used by the code yet)** **[P]**: K = relaxed + slope × |torque|.
    Shoulder 10.80 + 3.18 |t|, elbow 8.67 + 6.18 |t| (Gomi & Osu 1998, J Neurosci 18,
    Table 2 means, no co-contraction); hip 169.4 N m/rad relaxed + 5.15 N m/rad per N at
    the ankle over a 0.87 m mean leg = 5.95 per N m (Huang et al. 2020, IEEE TNSRE,
    Table III, straight leg); knee 75 relaxed (cited in Huang 2020) + 6.6 per N m of
    extension moment (Granata, Wilson & Padua 2002; 3.3 in flexion); trunk effective 590
    N m/rad about L4-L5 upright (Cholewicki et al. 2010: +160 = 27 %), 785 ± 580 unloaded
    up to 2200 under loads (Cholewicki 2000); no per-torque slope found for the trunk.
  - **Damping (N m s/rad, measured):** shoulder 0.63, elbow 0.76 (Gomi & Osu 1998, Table
    3 intercepts); knee ~2 relaxed (cited in Huang 2020); hip 2-21 relaxed (Huang 2020)
    and 0.5-10 (Koopman et al. 2016), no mean published; trunk 33.4 in extension
    (Cholewicki 2010).
  - Jolt 5.3 supports this directly: `ESpringMode::StiffnessAndDamping` (k in N m/rad,
    T = -k θ - c ω). Sources: e-space.mmu.ac.uk/13645, PMC2277155, PMC2277144,
    iris.unige.it/handle/11567/245370, arxiv.org/pdf/1908.07800, PMC2562882,
    par.nsf.gov/servlets/purl/10311862, PMC1029052, core.ac.uk/download/523258782.pdf
    **[P]**.
- **Balance (open):** muscles hold shape only; keeping the centre of mass over the feet
  (ankle and hip strategies, stepping) is its own step.

---

## Gaps and follow-ups
- These numbers were not found online and rely on standard textbook values: the exact
  Winter / Perry joint-angle curves, arm-swing shoulder ranges, and running hip and knee
  peaks. They should be checked against the books if motion looks off.
- Diving and penultimate-step data are deferred to M3.
- Reaching: no kinematic source for half-kneeling pickups; the pelvis axial turn and the
  spine split are estimates **[E]**.
- Throwing: phase timings and arm slots are from baseball; the fielder wind-up and
  swing times, the circular arc and underarm speeds are estimates **[E]**.
- Bowling: no measured arm-circle duration or standing-delivery speed; the release angle
  in world terms and the standing timings are estimates **[E]**. Spinner release height
  not found.
- All values are male-referenced. Female and other body types can be added later by
  swapping the ratio and mass tables in config.
