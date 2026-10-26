# Ball Physics Research (M3 Step 3.0)

Reference numbers for the M3 ball solver and the Jolt stumps. Every value in
`config/physics_world.json`, the ball config, `config/ball_materials.json` and
`config/ball_launch.json` should cite a section here. Confidence: **H** = rule or several
agreeing measurements; **M** = one study or a consensus range; **L** = estimate or
tuning starting point, to be checked against video.

Units: SI (m, kg, s, m/s). Speeds are also given in km/h because cricket reports them.

## 1. World

| Quantity | Value | Conf. | Source |
|---|---|---|---|
| Gravity g | 9.81 m/s² (down, −Y) | H | standard |
| Air density ρ | 1.225 kg/m³ (15 °C, sea level); 1.15–1.20 on a hot day | H | ISA standard atmosphere |
| Air kinematic viscosity ν | 1.5 × 10⁻⁵ m²/s | H | standard |
| Wind | 0 by default; 0–10 m/s is a realistic range | L | — |

Jolt and the ball solver share gravity, so the stumps and the ball fall the same way.

## 2. Ball

| Quantity | Value | Conf. | Source |
|---|---|---|---|
| Mass (men's) | 155.9–163.0 g; use **0.160 kg** | H | MCC Laws of Cricket, Law 4 |
| Circumference (men's) | 22.4–22.9 cm → radius 35.6–36.4 mm; use **r = 0.036 m** | H | MCC Law 4 |
| Women's / junior | 140–151 g, 21.0–22.5 cm | H | MCC Law 4 |
| Moment of inertia | ≈ 0.4 m r² (near-solid cork-and-twine core) ≈ 8.3 × 10⁻⁵ kg·m² | M | solid-sphere approximation; Cross (2005) |
| Cross-section A = πr² | 4.07 × 10⁻³ m² | H | derived |

## 3. Air drag

Drag force F = ½ ρ C_d A v², opposing the velocity relative to the air (ball velocity
minus wind).

- Reynolds number at bowling speeds (25–45 m/s): Re = v·2r/ν ≈ 1.2–2.2 × 10⁵. That
  is near the drag crisis of a smooth sphere. The seam and stitches make the ball's
  drag crisis much milder than a smooth sphere's: C_d falls slowly with speed rather
  than dropping sharply (Mehta 1985, 2005; Mehta & Wood 1980).
- Wind-tunnel C_d for a new ball is about **0.4–0.5** at subcritical speeds, falling
  toward ~0.3–0.35 at the highest speeds. Measurements scatter a lot with seam angle
  and wear (Mehta 2005; Scobie et al. 2013, J. Mech. Eng. Sci.). Drag and side force
  both peak at a seam angle of about 15° (BUET wind-tunnel study).
- **Model for M3:** constant **C_d = 0.45** (M). Speed-dependent C_d(Re) can be added
  later as another force generator if the trajectories look wrong.
- Effect: the drag deceleration is k·v², with k = ρ C_d A / 2m ≈ 0.0070 /m. At
  40 m/s that is about 11 m/s², more than gravity. Over the 20 m from release to the
  batter, a 40 m/s delivery loses about 13 % of its speed. Measured release-to-bat
  losses are 10–15 % (Hawk-Eye reports), which checks the model.
- Terminal velocity (falling): √(g/k) ≈ 37 m/s, so a high catch falling from 40 m
  lands at about 22 m/s (vacuum: 28 m/s).

## 4. Bounce on the pitch

Model each contact with a normal coefficient of restitution e (vertical speed after /
before) and a tangential law: Coulomb friction μ while sliding, switching to rolling
(spin-coupled, using the moment of inertia) once the contact point stops sliding (Cross
2002/2005; Carré & Haake 2000, 2004).

| Quantity | Value | Conf. | Source |
|---|---|---|---|
| Normal e, hard dry pitch (fast delivery) | 0.45–0.55 | M | Carré, James & Haake (2004), oblique impacts on clay pitches |
| Normal e, slow / damp pitch | 0.35–0.45 | M | Carré et al. 2004; Howard (Cambridge 2012), water content lowers e |
| Drop-test e (low speed, 2.4 m drop) | ~0.55–0.65 | M | Mondal et al. 2025: ball construction matters more than surface |
| Sliding friction μ, pitch | 0.25–0.35 (use **0.3**) | M | Carré et al. 2004 |
| Horizontal speed kept after a fast delivery bounces | ~0.65–0.75 | M | Hawk-Eye: 40 m/s release → ~29–33 m/s at the bat |
| Bounce height off a length (fast) | stumps to chest | H | broadcast footage |

Normal e falls with impact speed, because the ball deforms more. The solver takes e
from config as a constant per surface for now (L: e(v) later if needed).

## 5. Bounce and roll on the outfield (grass)

| Quantity | Value | Conf. | Source |
|---|---|---|---|
| Normal e, outfield turf | 0.30–0.45 (use **0.38**) | L–M | sports-turf drop tests (Baker et al.; Thorpe & Canaway); Penn State turf bounce data |
| Sliding friction μ, grass | 0.4–0.6 (use **0.5**) | L | turf studies |
| Rolling resistance coefficient C_rr | 0.08–0.25 (use **0.15**) | L | soccer/golf ball-roll studies scaled to cricket; tune by roll distance |
| Ball-roll deceleration | a = C_rr·g ≈ 0.5–1.5 m/s² | L | derived |

Roll distance d = v₀² / (2 C_rr g). With C_rr = 0.15, a ball rolling at 20 m/s goes
about 136 m before stopping, and a firm 30 m/s drive still crosses a 70 m boundary.
A soft push at 8 m/s stops after about 22 m. Real balls also lose speed in the first
skidding bounces (section 4 friction), so the pure-roll value may need to be lower;
tune it from broadcast roll times. A damp or long outfield is nearer 0.25 (82 m from
20 m/s). Groundsmen's guides say cut height, moisture and
hardness all change roll speed (GMA Cricket Outfield Performance Quality Standard).
In soccer turf studies, roll distance fell by 160 mm for each extra 1 mm of sward
height.

Stopping: once the speed is below ~0.05 m/s and the ball is on the ground, it comes to
rest (avoids endless creep). Config: `restSpeed`.

## 6. Ball against the stumps

| Quantity | Value | Conf. | Source |
|---|---|---|---|
| Stump height above ground | 0.7112 m (28 in) | H | MCC Law 8 |
| Wicket width (3 stumps) | 0.2286 m (9 in) | H | MCC Law 8 |
| Stump diameter | 34.9–38.1 mm (use 0.036 m) | H | MCC Law 8 |
| Stump mass (ash) | ~0.6–0.8 kg each | L | ash density ~700 kg/m³ × ~0.73 m × π(0.018)² ≈ 0.52 kg plus the spike |
| Bail length | 0.1095 m (4 5/16 in); stand 13 mm above stumps | H | MCC Law 8 |
| Bail mass | ~20–30 g | L | wooden bail volume estimate |
| Ball–wood restitution | ~0.5 | L | cricket bat studies (willow, low speed) |
| Stump in ground | ~0.2 m spike; held by soil, falls back on a hit | M | Law 8; footage |

The stumps are Jolt dynamic bodies, held upright by a soft constraint to the ground (the
soil), which a hit breaks. The ball pushes them through a one-way impulse: the ball
solver resolves its own contact, then applies the equal and opposite impulse to the Jolt
body.

## 7. Speeds (launch presets)

| Event | Speed | Conf. | Source |
|---|---|---|---|
| Fast bowling release | 38–42 m/s (135–150 km/h); record 44.7 m/s (161.3 km/h, Akhtar 2003) | H | broadcast speed guns |
| Medium pace | 33–37 m/s (120–135 km/h) | H | broadcast |
| Spin bowling | 20–25 m/s (70–90 km/h) | H | broadcast |
| Release height | 2.0–2.3 m (fast); 1.9–2.1 m (spin) | M | Hawk-Eye analyses |
| Release angle | ~−5° to −8° (fast, downward); slightly upward (spin, about +3°) | M | Hawk-Eye; Justham et al. |
| Ball off the bat (drive, six) | 35–45 m/s; up to ~50 m/s for the hardest hits | M | bat-ball studies (Stretch et al.; Smith & Singh) |
| Lofted six launch angle | 25–40° | M | trajectory analyses |
| Fielder throw | 25–35 m/s (elite ~40) | M | throwing studies (Cook & Strike) |
| High catch | launch 25–30 m/s at 70–80°, apex 30–40 m, ~5–6 s in the air | M | derived and broadcast |

## 8. Integrator notes (used in 3.2)

- 120 Hz tick, and the ball substeps each tick: at 45 m/s a tick moves the ball 0.375 m,
  about 5 ball radii. Substep so that each step moves at most ~0.5 r (about 16 substeps
  at top speed), and use swept contact tests against the ground and the stumps, so the
  ball never tunnels.
- Semi-implicit (symplectic) Euler, or RK2 for the drag term. The predictor calls the
  exact same step function with the same substep rule, so it matches the simulation
  bit for bit.

## Sources

- MCC, *Laws of Cricket*, Law 4 (The ball) and Law 8 (The wickets). lords.org/mcc/the-laws.
- Mehta, R.D. & Wood, D.H. (1980). Aerodynamics of the cricket ball. *New Scientist*.
- Mehta, R.D. (1985). Aerodynamics of sports balls. *Annu. Rev. Fluid Mech.* 17.
- Mehta, R.D. (2005). An overview of cricket ball swing. *Sports Engineering* 8.
- Scobie, J.A. et al. (2013). Fluid dynamics of cricket ball swing. *Proc IMechE C*.
  https://journals.sagepub.com/doi/xml/10.1243/09544062JMES1973
- RMIT/Melbourne: wear effects on cricket ball aerodynamics.
  https://people.eng.unimelb.edu.au/imarusic/proceedings/17/177_Paper.pdf
- BUET seam-angle wind-tunnel study.
  https://me.buet.ac.bd/public/old/icme/icme2001/cdfiles/Papers\Fluid\8.%20FM-29%20Final%20(41-44).pdf
- Carré, M.J. & Haake, S.J. (2000). Cricket ball impacts on soil pitches. https://shura.shu.ac.uk/2216/
- Carré, M.J., James, D.M. & Haake, S.J. (2004). Impact of a non-homogeneous sphere on a
  rigid surface. *Proc IMechE C* 218.
- Cross, R. (2002). Grip-slip behaviour of a bouncing ball. *Am. J. Phys.* 70.
- Cross, R. (2005). Bounce of a spin ball near normal incidence. *Am. J. Phys.* 73.
- Howard (2012). Cricket pitch bounce, Cambridge undergraduate project.
  https://www-geo.eng.cam.ac.uk/sites/default/files/howard.pdf
- Mondal et al. (2025). Coefficient of restitution of sports balls on different surfaces.
  https://www.hrpub.org/journals/article_info.php?aid=15572
- Penn State turf bounce study.
  https://plantscience.psu.edu/research/centers/ssrc/documents/pennbounce.pdf/@@download/file/pennbounce.pdf
- GMA, Cricket Outfield Performance Quality Standard.
  https://thegma.org.uk/services/interactive-tools/cricket/cricket-outfield-performance-quality-standard
- Turf ball-roll and sward height (ISHS). https://ishs.org/ishs-article/661_37/
- Michigan State thesis on rolling and ball roll. https://d.lib.msu.edu/etd/19502

Open items to check against video in 3.3: the pitch e at 40 m/s, the outfield C_rr (roll
time to the boundary), and the stump soil hold.
