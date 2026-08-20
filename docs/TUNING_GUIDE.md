# Geometric Controller Tuning Plan — X500v2 Interceptor (2.5 kg, Jetson Orin NX)

Master plan for safely computing, validating, and auto-tuning the gains of the
`mav_controllers_ros` geometric attitude controller
(`geometric_attitude_control_node` + mavros interface to PX4).

All tooling referenced here lives in **`~/src/ihunter_fixes/geo_tuner`**
(ROS 2 package + offline CLIs, see its README for command details).

---

## 1. Why gains can be pre-computed

The controller's position loop outputs *acceleration*:
`a_fb = kx·e_pos + kv·e_vel (+ feedforward + gravity)`. The ideal closed loop
per axis is therefore a double integrator under PD control, and the gains are
physical quantities:

| Gain | Meaning | Units |
|---|---|---|
| `kx` | ωn² (position-loop natural frequency squared) | 1/s² |
| `kv` | 2·ζ·ωn (damping) | 1/s |
| `attctrl_tau` | attitude loop time constant; bandwidth ≈ 2/τ | s |
| `max_thrust` | thrust map: throttle = force/max_thrust — **scales every gain implicitly** | N |

Design rule (cascaded loops, each 3–5× slower than the one below):

```
PX4 rate loop (PX4 Autotune)  ~30–60 rad/s
  └── attitude loop            = 2/attctrl_tau   (τ=0.3 → 6.7 rad/s)
        └── position loop      ωn ≤ (2/τ)/4  AND  ωn ≤ 0.35/latency
```

The only vehicle-specific unknowns are **mass** (measured), the **thrust map**
(one hover flight), and the **inner-loop quality** (PX4 Autotune). Everything
else follows from formulas — no guessing.

Known controller quirks (verified in source):
- `gains.ki.*` integrates per-callback **without dt** → keep 0 during tuning.
- `gains.kib.*` is dead code — never tune it.
- Gains are hot-reloadable via `ros2 param set` → enables in-flight tuning.
- The node commands **body rates + thrust** (IGNORE_ATTITUDE), so PX4 runs
  only its rate loop underneath.

---

## 2. The plan, phase by phase

### Phase 0 — PX4 inner loops (you, one flight)
1. Verify RC failsafe behavior: flipping the mode switch out of OFFBOARD
   must instantly return control to PX4 Position mode. Test at altitude.
2. Run **PX4 Autotune** (rate + attitude) in Position/Altitude mode.
3. Raise mavros odometry stream rate to ≥ 50 Hz for
   `geometric_controller/odom`.

### Phase 1 — Thrust map from a hover ulog (you fly, tool computes)
1. Fly 1–2 min of calm hover in **Position mode**, full flight battery.
2. `geo-tuner-hover flight.ulg --mass 2.5 --design`
   → prints hover throttle, computes `max_thrust = m·g/u_hover`, and writes
   ready `geometric_controller.yaml` + `geometric_mavros.yaml` with
   principled starting gains (defaults: ωn≈1.7 rad/s, ζ=0.95, τ=0.3).

### Phase 2 — Fast simulation validation (no Gazebo, seconds per run)
`ros2 launch geo_tuner sim_tune.launch.py [thrust_scale_error:=0.9]`
runs the **real compiled controller node** + lightweight quad simulator +
auto-tuner end-to-end. Purpose: verify the whole toolchain and your config
before any propellers spin. **Status: DONE — see §4.**

### Phase 3 — Full PX4 SITL validation (d2dtracker docker)
Same auto-tuner, unchanged, against PX4 SITL + Gazebo + mavros in your normal
docker workflow (image `mzahana/px4-simulation-cuda12.2.0-ubuntu22`):
1. Start your container as usual (`./docker_run_with_cuda.sh`, shared volume
   at `$HOME/<container>_shared_volume`).
2. Copy/link `geo_tuner` and `mav_controllers_ros` into
   `shared_volume/ros2_ws/src`, `colcon build`.
3. Bring up PX4 SITL + mavros + geometric controller (d2dtracker_sim bringup),
   take off, enter OFFBOARD hover.
4. `ros2 launch geo_tuner field_tune.launch.py` — identical to the field run.
5. Accept when: session completes, report shows `wn_effective` at target on
   all axes, no safety aborts.
   (If the image defaults to zenoh RMW: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.)
**Status: environment READY** — container `d2dtracker_cuda` built per the
docs (shared volume `~/d2dtracker_cuda_shared_volume`): PX4 v1.14 SITL +
ihunter models built, full ros2_ws (27 packages incl. the hardened
`mav_controllers_ros` and `geo_tuner`) compiled, PX4+Gazebo boot smoke
test passed ("Ready for takeoff"). The tuning-session flight itself (steps
1–5 above) is the remaining action.

### Phase 4 — Field auto-tune (one flight, mostly hands-off)
1. Load Phase-1 yaml files. Conservative envelope: `max_tilt_angle: 0.52`,
   `max_accel: 5`, `ki: 0`.
2. Take off, hover in OFFBOARD near the `hover_position` configured in
   `geo_tuner/config/tuner_field.yaml` (≥ 10 m AGL, clear ground, geofence on,
   pilot's thumb on the mode switch — RC override always wins; the tuner
   never arms/disarms or changes modes).
3. `ros2 launch geo_tuner field_tune.launch.py`. The conductor automatically:
   - reads current gains as the safe baseline;
   - injects small alternating steps (z first, then x, y; ±0.4–0.5 m);
   - fits a 2nd-order + delay model per response; rejects bad fits;
   - identifies plant-gain factor α (absorbs thrust-map error + lag) and
     re-places the poles at target (ωn, ζ) via live parameter update;
   - walks ωn up the ladder `[1.2, 1.6, 2.0]` rad/s, re-identifying per rung;
   - trims steady offsets via bounded accel feedforward and converts a
     persistent z-trim into a `max_thrust` correction suggestion;
   - on ANY safety violation (tilt, pos error, speed, altitude, stale odom,
     rate oscillation): restores last-safe gains, holds hover, writes a
     diagnosis, stops.
4. Land. Copy `final_gains` from `/tmp/geo_tuner_report.yaml` into
   `geometric_controller.yaml`; apply any suggested `max_thrust` correction.

### Phase 5 — Agility pass (optional, chase performance)
Once Phase 4 is stable: `attctrl_tau: 0.2` (separation cap rises to
ωn ≤ 2.5), ladder `[1.6, 2.0, 2.5]`, then relax `max_accel`/`max_tilt_angle`
toward mission values and re-run one field session. Validate tracking on a
lemniscate/target-chase trajectory before mission use.

---

## 3. Safety architecture (all phases)

| Layer | Mechanism |
|---|---|
| Pilot | RC mode switch out of OFFBOARD overrides everything, always |
| PX4 | geofence + return altitude + battery failsafe |
| Controller | `max_tilt_angle`, `max_accel` clamps, `se3_cmd_timeout` |
| Tuner | independent monitor: tilt / pos-error / speed / altitude / odom-stale / oscillation → abort + gain restore |
| Tuner | fit-quality gates, per-episode gain-change limit (×1.6), ladder refuses ωn·delay > 0.45, steps only between settles |

---

## 4. Validation status (what has actually been tested, honestly)

| Test | Environment | Result |
|---|---|---|
| 24 unit tests (gain math, step fitting, safety logic) | host venv | PASS |
| Closed-loop auto-tune: **real controller node** + physics sim + tuner | host, ROS 2 Jazzy | PASS — all axes converge to ωn_eff = 1.60, ζ = 0.95 exactly |
| Same, 10 % thrust-map error | host | PASS — converges, reports "multiply max_thrust by 0.92" (truth: 0.90) |
| Same, 30 % thrust-map error | host | PASS — safe abort, gains restored, actionable diagnosis |
| Same closed-loop test | **inside your docker image**, ROS 2 Humble | PASS — identical convergence; caught & fixed 2 Humble incompatibilities |
| PX4 SITL + Gazebo boot (headless, ihunter stack) | docker | PASS — "Ready for takeoff", gz_bridge connected |
| Full PX4 SITL tuning-session flight | docker | **NOT YET RUN** — Phase 3 flight, next step |
| ki removes steady offset (hardened controller, 10 % thrust error) | host | PASS — z error 0.36 m → 0.000 m, identical at 25/50 Hz setpoint rates |
| Closed-loop tuning vs hardened controller | host | PASS — identical convergence to upstream |

### Controller hardening (production-hardening branch)

`~/src/ihunter_fixes/mav_controllers_ros`, branch `production-hardening`
(commit c9f8b01, see its CHANGES.md): dt-correct integrator + anti-windup,
altitude-priority saturation, body-rate feedforward from jerk (agility),
odometry/setpoint watchdogs, mavros command-timeout failsafe fix (stale
commands no longer block PX4's offboard-loss failsafe), online IMU-based
thrust-scale estimator (gated, slow, clamped; published for logging).

`geo_tuner` is published at git@github.com:mzahana/geo_tuner.git.

Important clarifications:
- **No docker image was built.** Your existing local image
  `mzahana/px4-simulation-cuda12.2.0-ubuntu22` was used to start a temporary
  container (`geo_tuner_test`) with a session scratch folder mounted; it was
  deleted after the test. Your `$HOME/gpsdnav_shared_volume`, `$HOME/src/`,
  and the `gpsdnav` container were not touched, and no new shared volume was
  created — that is why you don't see one.
- The "simulated flights" so far exercise the real controller and the real
  tuner against a simplified plant (rigid body + rate-loop lag + thrust error
  + odom delay/noise). PX4 firmware, EKF2, and mavros transport are only
  covered by Phase 3.

---

## 5. Appendix A — Mathematical foundations

Complete derivations for every rule and formula the tooling implements.
Notation: position $p\in\mathbb{R}^3$, velocity $v=\dot p$, rotation
$R\in SO(3)$ with body axes $x_B,y_B,z_B$ (columns of $R$), mass $m$,
gravity $g$, world up $e_3$.

### A.1 Vehicle model

A multirotor with collective thrust $T\ge 0$ along $z_B$:

$$m\ddot p = -m g e_3 + T\,R e_3 - D v, \qquad \dot R = R\,\hat\omega,$$

with $D$ a small linear-drag matrix and $\hat\cdot$ the skew map. The
system is differentially flat in $(p,\psi)$: given any smooth position
trajectory and yaw, the required attitude and thrust are algebraic
functions of $(\ddot p, \psi)$, and the body rates of $(p^{(3)},\dot\psi)$.
This is why a position controller that outputs *acceleration* plus an
attitude loop recovers full trajectory tracking (Mellinger & Kumar 2011;
Faessler et al. 2017).

### A.2 The implemented control law

The controller (`GeometricAttitudeControl`) computes, with tracking errors
$e = p - p_{ref}$, $\dot e = v - v_{ref}$:

$$a_{fb} = -K_x e - K_v \dot e - \textstyle\int K_i\, e\,dt$$
$$a_{des} = a_{fb} + a_{ref} - a_{rd} + g e_3$$

(rotor-drag term $a_{rd}=R_{ref} K_d R_{ref}^\top v_{ref}$), then

- desired attitude: $z_B^{des} = a_{des}/\lVert a_{des}\rVert$, with
  $x_B^{des},y_B^{des}$ from the yaw reference (the `acc2quaternion` map);
- force $F = m\,a_{des}$; commanded normalized thrust
  $u = F\cdot z_B / T_{max}$ where $z_B$ is the *current* body z;
- body-rate command $\omega_{cmd} = \frac{2}{\tau}\,e_R$ with the
  geometric attitude error
  $e_R = \tfrac12\big(R_d^\top R - R^\top R_d\big)^{\vee}$.

### A.3 Outer-loop error dynamics and the gain map

Assume (for now) perfect inner loops: the vehicle realizes $a_{des}$
exactly. Substituting into the model, gravity and feedforward cancel and
each axis obeys the linear error equation

$$\ddot e + k_v \dot e + k_x e + k_i\!\int\! e\,dt = 0.$$

**Without integral** ($k_i=0$) this is the standard second-order system

$$s^2 + k_v s + k_x = 0 \;\iff\; k_x = \omega_n^2,\quad k_v = 2\zeta\omega_n .$$

This is the entire justification for designing $(\omega_n,\zeta)$ instead
of raw gains: the map is exact, invertible, and dimensionally meaningful
($k_x$ [1/s²], $k_v$ [1/s]).

**With integral**, the characteristic polynomial is
$s^3 + k_v s^2 + k_x s + k_i = 0$. By the **Routh–Hurwitz criterion** the
loop is stable iff all coefficients are positive and

$$\boxed{\,k_i < k_x\,k_v\,}$$

Practical rule: $k_i \le 0.1\,k_x k_v$ keeps the integral pole slow and
non-oscillatory. (This bound is only meaningful now that the hardened
controller integrates with real $dt$; the original per-sample integrator
made $k_i$'s effective value rate-dependent and the bound unusable.)

### A.4 Inner loops and time-scale separation

Small-attitude-error linearization of the rate command: with
$e_{att}\approx\tfrac12\theta$ (rotation angle $\theta$),
$\dot e_{att} = -\tfrac{2}{\tau} e_{att}$, i.e. the attitude loop is a
first-order system with bandwidth

$$\omega_{att} = 2/\tau_{attctrl}.$$

Underneath it, PX4's rate loop acts as another lag $\tau_r$ (autotuned,
typically $\omega_{rate}\gtrsim 30$ rad/s). The cascade argument
(singular-perturbation / two-time-scale): if the outer loop is slower than
the inner by a factor $\varepsilon^{-1}$, the inner dynamics perturb the
outer poles only at order $\varepsilon$. Requiring the perturbation to be
a small fraction of the design damping gives the classic engineering
ratio:

$$\omega_n \le \frac{\omega_{att}}{3\ldots5} = \frac{2/\tau}{3\ldots5},
\qquad \omega_{att} \le \frac{\omega_{rate}}{3}.$$

The toolkit uses separation 4 by default. With $\tau=0.3$:
$\omega_n \le 1.67$ rad/s; with $\tau=0.2$: $\omega_n \le 2.5$ rad/s.

### A.5 Latency bound

Total loop delay $T_d$ (EKF, mavros transport, offboard path,
zero-order-hold) multiplies the loop transfer by $e^{-sT_d}$, which
subtracts phase $\omega T_d$ [rad] without changing magnitude. For the PD
double-integrator loop with $\zeta\approx 1$, gain crossover sits near
$\omega_c \approx 1.2\,\omega_n$ and the nominal phase margin is
$\approx 65^\circ$ ($1.14$ rad). Spending at most a third of that margin
on delay:

$$\omega_c T_d \le 0.35\ldots0.45 \;\Rightarrow\;
\boxed{\;\omega_n \le \frac{0.35}{T_d}\;}$$

(design cap; the in-flight tuner enforces the run-time version
$\omega_{target} \cdot \hat T_d \le 0.45$ with the *measured* delay
$\hat T_d$ from the step fit). With $T_d\approx 80$ ms this caps
$\omega_n$ at $\sim$4.4 rad/s — above the separation cap, so separation
usually binds; on a congested link ($T_d>120$ ms) latency binds instead.

### A.6 Thrust map: why max_thrust scales every gain

The chain commands $u = m\,a_{des}\!\cdot\!z_B / T_{max}^{param}$ and the
vehicle produces $T = u\,T_{max}^{true}$. Define
$\alpha = T_{max}^{true}/T_{max}^{param}$. The realized specific force is
then $\alpha\,a_{des}$, so the error dynamics become

$$\ddot e = -\alpha\,(k_x e + k_v \dot e) + (\alpha-1)g e_3.$$

Consequences, all used by the tooling:

1. **Effective poles move:** $\omega_n^{eff} = \sqrt{\alpha k_x}$,
   $\zeta^{eff} = \sqrt{\alpha}\,\zeta$. A mis-measured thrust map
   re-tunes every axis silently.
2. **Hover offset (z):** at equilibrium ($\ddot e=\dot e=0$, no integral)
   $\alpha(k_x \Delta z + g) = g$, giving
   $$\Delta z = \frac{g(1-\alpha)}{\alpha k_x}, \qquad
   \alpha = \frac{g}{g + k_x\,\Delta z}.$$
   The conductor's hover diagnosis is this formula inverted; its
   acceleration trim $a_{trim} = k_x\Delta z$ is exactly the missing
   specific force.
3. **Hover anchor:** at hover $u_h = mg/T_{max}^{true}$, so
   $T_{max} = mg/u_h$ — the Phase-1 measurement.

### A.7 In-flight identification and the gain correction

During a step episode the conductor commands a reference step $r$ with
zero velocity/acceleration feedforward, so the closed loop from $r$ to
position is exactly

$$\frac{X(s)}{R(s)} = e^{-sT_d}\,
\frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

with the delay $e^{-sT_d}$ lumping transport delay and the residual
inner-loop lag (a first-order lag at $\omega_{att}\gg\omega_n$ is
well-approximated by dead time $1/\omega_{att}$ at outer-loop
frequencies). The fitter estimates
$(\hat\omega_n,\hat\zeta,\hat T_d, A)$ by nonlinear least squares on the
time-domain step response (closed forms for under-, critically- and
over-damped cases).

Given applied $k_x^{app}$ and measured $\hat\omega_n$, the lumped plant
gain is identified as

$$\hat\alpha = \hat\omega_n^2 / k_x^{app},$$

and the gains that place the *effective* poles at the target
$(\omega_n^\star, \zeta^\star)$ follow from A.6:

$$k_x^{new} = \frac{(\omega_n^\star)^2}{\hat\alpha}, \qquad
k_v^{new} = \frac{2\zeta^\star\omega_n^\star}{\hat\alpha}.$$

This is a fixed-point iteration on $\alpha$; because $\alpha$ enters
multiplicatively and is re-identified each rung, one to two iterations
suffice (observed in every sim run). Safeguards: fit-quality gate
(NRMSE < 0.15, $\hat\zeta$ in [0.05, 2.5]), per-episode gain change
clamped to a factor 1.6, ladder monotonic in $\omega_n^\star$, and each
new gain set starts from a configuration that just flew safely — so the
iteration is confined to a box around a known-stable point.

**Identifiability.** For near-critically-damped responses the triple
$(\hat\omega_n, \hat\zeta, \hat T_d)$ is weakly identifiable: because the
true response is higher-order (inner-loop lag stacks on the position
loop), "high $\omega_n$, overdamped, large delay" explains the data as
well as — sometimes better than — the physically correct description.
Left alone, this produces absurd $\hat\alpha$ (7× observed in SITL). The
fitter therefore (i) runs multi-start optimization, and (ii) constrains
$\hat\omega_n$ to the *physically possible* interval implied by the
applied gains, $\hat\omega_n \in \sqrt{k_x^{app}} \cdot
[\sqrt{\alpha_{min}}, \sqrt{\alpha_{max}}]$ with
$[\alpha_{min},\alpha_{max}] = [0.4, 2.5]$ (a real thrust map is not off
by more than 2.5×). Within that box, $\hat\zeta$ and $\hat T_d$ absorb
the unmodeled lag. Fits with $\hat\zeta$ pinned at its optimizer bounds
are rejected outright, and a resulting $\hat\alpha$ outside the box
discards the episode instead of updating gains.

**Mode supervision.** Episodes only run while PX4 reports OFFBOARD
(`mavros/state`): outside OFFBOARD the vehicle ignores the controller's
setpoints and any "response" is noise. Before OFFBOARD the conductor
streams the current position as setpoint (bumpless engage; the stream is
also what makes PX4 accept the mode switch); leaving OFFBOARD
mid-session pauses tuning — episode discarded, gains kept — and it
resumes from a fresh hover when OFFBOARD returns.

### A.8 Body-rate feedforward (differential flatness)

Write the specific-force vector $f = a + g e_3 = \tfrac{T}{m} z_B$ with
$\lVert f\rVert = T/m$. Differentiating $T z_B$ and projecting out the
thrust-magnitude change:

$$h_\omega = \frac{j - (z_B\!\cdot\! j)\,z_B}{\lVert f \rVert}, \qquad
\omega_x^{ff} = -\,h_\omega\!\cdot\! y_B,\quad
\omega_y^{ff} = h_\omega\!\cdot\! x_B,\quad
\omega_z^{ff} = \dot\psi\,(e_3\!\cdot\! z_B),$$

where $j = \dot a$ is the reference jerk. These are the exact body rates
a flat trajectory demands; adding them to the feedback term converts the
attitude loop from *chasing* the moving reference (lag $\propto$
$\tau\,\lVert\dot R_d\rVert$) to *following* it, which is where most agile
tracking error comes from. At hover or on jerk-free references the term
is identically zero — it cannot affect the tuning episodes. Implemented
with a defensive $\pm 3$ rad/s clamp and disabled near free-fall
($\lVert a_{des}\rVert < 1$ m/s²).

### A.9 Online thrust-scale estimator

The accelerometer measures specific force; its body-z component in flight
is $a_z^{IMU} = T/m$ (drag along $z_B$ neglected — enforced by gating on
low body rates and mid-envelope thrust). With commanded thrust
$T_{cmd} = u\,T_{max}^{param}$:

$$\alpha_{sample} = \frac{m\,a_z^{IMU}}{T_{cmd}}, \qquad
\dot{\hat\alpha} = \frac{1}{\tau_{est}}(\alpha_{sample} - \hat\alpha),$$

discretized per sample and hard-clamped to $[0.8, 1.25]$. The corrected
throttle $u = T_{cmd}/(T_{max}^{param}\hat\alpha)$ makes the effective
loop gain $\alpha/\hat\alpha \to 1$: battery sag ($T_{max}\propto$ pack
voltage roughly quadratically through the ESC) and prop wear are trimmed
continuously with a 15 s time constant — two orders slower than the
position loop, so the estimator cannot interact with it dynamically
(same time-scale-separation argument as A.4).

### A.10 Safety monitor quantities

- **Tilt** from the quaternion: $\theta = \arccos(R_{33})$,
  $R_{33} = 1-2(q_x^2+q_y^2)$.
- **Oscillation:** RMS of mean-removed roll/pitch rates over a 2 s
  window; threshold 1.2 rad/s. A marginally stable pair at $\omega_{osc}$
  with tilt amplitude $\theta_0$ produces rate RMS
  $\theta_0\omega_{osc}/\sqrt2$ — e.g. 5° at 6 Hz ≈ 2.3 rad/s, well
  above threshold, while normal maneuvering stays below.
- **Stability envelope for episodes** follows from A.3–A.5: the wn ladder
  never exceeds the separation/latency caps, ζ target near 1 keeps
  $k_ik_xk_v$ margins trivially satisfied (ki = 0 during tuning).

### A.11 References

- T. Lee, M. Leok, N. H. McClamroch, *Geometric tracking control of a
  quadrotor UAV on SE(3)*, CDC 2010 — exponential stability of the
  geometric attitude/position cascade; the formal basis for A.2/A.4.
- D. Mellinger, V. Kumar, *Minimum snap trajectory generation and control
  for quadrotors*, ICRA 2011 — differential flatness, A.1/A.8.
- M. Faessler, A. Franchi, D. Scaramuzza, *Differential flatness of
  quadrotor dynamics subject to rotor drag*, RA-L 2017 — the drag-aware
  law this controller implements.
- F. Berkenkamp, A. P. Schoellig, A. Krause, *Safe controller
  optimization for quadrotors with Gaussian processes*, ICRA 2016 — the
  safe-BO tier (Phase 5+ option).
- K. J. Åström, T. Hägglund, *Advanced PID Control* — relay/step
  identification, anti-windup by conditional integration.

## 6. Deliverables map

```
~/src/ihunter_fixes/
├── TUNING_PLAN.md                  <- this document
└── geo_tuner/                      <- ROS 2 package (ament_python)
    ├── README.md                   <- command-level usage
    ├── geo_tuner/core/             <- pure logic: gain_design, step_fit,
    │                                  safety, thrust_model (unit-tested)
    ├── geo_tuner/cli/              <- geo-tuner-hover, geo-tuner-design
    ├── geo_tuner/tuning_conductor.py   <- in-flight auto-tuner node
    ├── geo_tuner/quad_sim.py       <- lightweight plant for fast sim tests
    ├── launch/sim_tune.launch.py   <- Phase 2 (controller+sim+tuner)
    ├── launch/field_tune.launch.py <- Phases 3 & 4 (tuner only)
    ├── config/tuner_field.yaml     <- field session configuration
    └── test/test_core.py           <- 24 unit tests
```
