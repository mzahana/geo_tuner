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
2. Take off and hover where the session should run — clear ground, geofence
   on, at or above `min_tuning_altitude` in
   `geo_tuner/config/tuner_field.yaml` (6 m AGL by default) — then hand over
   in OFFBOARD. The session tunes about the point it is handed: it never
   commands a climb, and handed over too low it holds position in state
   `TOO_LOW` until you climb or lower the gate from the panel. Pilot's thumb
   on the mode switch — RC override always wins; the tuner never
   arms/disarms or changes modes.
3. `ros2 launch geo_tuner field_tune.launch.py output_dir:=~/tuning_logs`
   (`output_dir`: timestamped report + raw episode CSVs per session, nothing
   overwritten — bring that folder home for analysis). The conductor
   automatically:
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
4. Land. Copy `final_gains` from the session's report (under `output_dir`,
   or `/tmp/geo_tuner_report.yaml` when unset) into
   `geometric_controller.yaml`; apply any suggested `max_thrust` correction.

### Phase 5 — Agility pass (optional, chase performance)
Once Phase 4 is stable: `attctrl_tau: 0.2` (separation cap rises to
ωn ≤ 2.5), ladder `[1.6, 2.0, 2.5]`, then relax `max_accel`/`max_tilt_angle`
toward mission values and re-run one field session. Validate tracking on a
lemniscate/target-chase trajectory before mission use.

**The aggressive-tracking recipe, in full.** Feedback bandwidth is only
one third of aggressive tracking — and the smallest third. In order of
payoff:

1. **Feedforward carries the maneuver.** The controller consumes the
   setpoint's velocity, acceleration *and* jerk (rate feedforward from
   differential flatness, `enable_rate_feedforward: true`). A trajectory
   generator that fills those fields moves the feedback loop's job from
   "chase the reference" to "reject disturbance" — tracking error during
   a 15 m/s² maneuver is then set by model error, not by ωn. Never
   command aggressive motion through bare position setpoints.
2. **An honest thrust map + integral action.** `ki.z` ≈ 1.0 with the
   online thrust-scale estimator keeps the feedforward calibrated as the
   battery sags; a 5% thrust error during a vertical maneuver is
   0.5 m/s² of acceleration the feedback must supply.
3. **Feedback bandwidth last**, raised only against measurement:
   inner loops first (PX4 autotune, then `attctrl_tau` 0.3 → 0.2),
   which raises the separation cap, then the ladder — with
   `episodes_per_rung: 3` so the identified α (and hence the final
   gains) are reproducible, and the report's median delay telling you
   where the latency ceiling `ωn ≤ 0.35/T_d` actually is. Stop at the
   first rung where the measured overshoot exceeds ~10% or the
   consistency gate starts refusing updates: past that point you are
   tuning to noise, and disturbance rejection no longer improves.

---

## 3. Safety architecture (all phases)

| Layer | Mechanism |
|---|---|
| Pilot | RC mode switch out of OFFBOARD overrides everything, always |
| PX4 | geofence + return altitude + battery failsafe |
| Controller | `max_tilt_angle`, `max_accel` clamps, `se3_cmd_timeout` |
| Tuner | independent monitor: tilt / pos-error / speed / altitude / odom-stale / oscillation → abort + gain restore |
| Tuner | fit-quality gates (incl. ambiguity + bound-rest rejection), per-episode gain-change limit (×1.6), ladder refuses ωn > 2ζ/(m·τ̂) from the identified in-loop lag, steps only between settles |

---

## 4. Validation status (what has actually been tested, honestly)

| Test | Environment | Result |
|---|---|---|
| 65 unit tests (gain math, step fitting, safety logic, episode schedule) | gtest, docker Humble | PASS |
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

(design cap, used offline by `geo-tuner-design`. The in-flight tuner no
longer uses a delay proxy at all — it enforces the Routh-Hurwitz margin
of A.5a on the lag it actually identifies.) With $T_d\approx 80$ ms this caps
$\omega_n$ at $\sim$4.4 rad/s — above the separation cap, so separation
usually binds; on a congested link ($T_d>120$ ms) latency binds instead.

### A.5a Run-time stability cap from the identified lag

The design cap above is a phase-margin rule of thumb applied to a delay
you have to guess before flying. In the air the conductor has something
better: $\hat\tau$, identified per bucket from $(\ast)$. Its
characteristic polynomial is

$$\tau s^3 + s^2 + \alpha k_v s + \alpha k_x,$$

and Routh-Hurwitz for a cubic $a_3s^3+a_2s^2+a_1s+a_0$ requires
$a_2 a_1 > a_3 a_0$, i.e.

$$\alpha k_v > \tau\,\alpha k_x \;\Longleftrightarrow\;
\boxed{\;k_v > \tau k_x\;}$$

— note $\alpha$ cancels, so the boundary depends only on the gains and
the lag. Substituting the design rule $k_x=\omega_n^2,\;
k_v=2\zeta\omega_n$ and keeping the Routh product a factor $m$ clear of
the boundary gives the ladder cap the tuner enforces:

$$\omega_{target} \;\le\; \frac{2\zeta^\star}{m\,\hat\tau},
\qquad m = \texttt{stability\_margin} \;(\text{default } 4).$$

At $m=4,\ \zeta^\star=0.95$ this is
$\omega_{target}\hat\tau \le 0.475$ — deliberately about as
conservative as the $\omega_{target}\hat T_d \le 0.45$ heuristic it
replaces, so field behaviour is unchanged in magnitude. What changes is
that the number is *derived* from a measured, physically meaningful
quantity instead of assumed, it scales correctly when $\zeta^\star$ is
changed (the old rule did not), and $m=1$ is the true instability
boundary rather than an arbitrary reference.

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
zero velocity/acceleration feedforward, so the controller is running

$$a_{des} = k_x (r - x) - k_v \dot x$$

with $k_x, k_v$ **known** — the conductor applied them. The vehicle
delivers a fraction $\alpha$ of that command through an inner loop of
effective lag $\tau$ (attitude tracking on $x,y$; thrust response on
$z$), and integrates it twice, so the closed loop is exactly

$$\frac{X(s)}{R(s)} =
\frac{\alpha k_x}{\tau s^3 + s^2 + \alpha k_v s + \alpha k_x}
\qquad (\ast)$$

with unit DC gain. **Only two dynamic quantities are unknown**: the
plant-gain factor $\alpha$ (thrust-map error, inner-loop droop — the
number the gain update needs) and the in-loop lag $\tau$ (the number the
bandwidth ladder needs). The fitter estimates
$(\hat\alpha, \hat\tau, A)$ by nonlinear least squares on
the step response of $(\ast)$, evaluated in closed form from the
residues of its three poles; $A$ absorbs steady-state offset.

**Where the transport delay goes.** Into $\tau$. The sensing delay is not
a harmless shift of the recorded curve — the controller feeds back that
same delayed odometry, so it sits *inside* the loop and costs phase
exactly where stability is decided. Below the outer-loop bandwidth a
delay and a lag are interchangeable to first order
($e^{-T_ds}\approx 1/(1+T_ds)$ for $T_d\omega\ll1$), so one effective
in-loop lag carries both, and the A.5a margin then sees the delay it
ought to see. Fitting a separate output shift as well was measured to be
*worse*: on responses from an exact delay-buffer simulation a free output
shift roughly doubled the error in $\hat\alpha$ (5.9% against 2.9%),
because it gives the optimizer somewhere to park phase that belongs
inside the loop. Since $\alpha$ is estimated *directly*,
$\hat\omega_n = \sqrt{\hat\alpha k_x}$ and
$\hat\zeta = \hat\alpha k_v / (2\sqrt{\hat\alpha k_x})$ are reported
as derived diagnostics rather than fitted quantities. Note the
consequence, which is the opposite of the usual intuition:
$\hat\zeta = \sqrt{\hat\alpha}\,\zeta_{design}$, so a plant *stronger*
than modelled is more damped, and it is a weak plant that rings.

Given applied $k_x^{app}$ and identified $\hat\alpha$, the gains that place the *effective* poles at the target
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

**Why not fit a free second order?** Until 2026-09-06 the fitter
estimated $(\hat\omega_n, \hat\zeta, \hat T_d, A)$ — a free
second-order system — and *derived*
$\hat\alpha = \hat\omega_n^2 / k_x^{app}$. That model is wrong whenever
$\tau$ is not small against the position loop, and it is wrong in a way
that goes straight into the gains: three free dynamic parameters can
trade against each other, so the optimizer buys fit quality by raising
$\hat\omega_n$, over-damping, and pushing dead time. Measured on step
responses synthesised from $(\ast)$ with known $\alpha$, the mean error
in $\hat\alpha$ was **49.7%**, growing systematically with $\tau$ — and
every one of those fits passed the quality gate. The same experiment on
the present model gives **1.0%**.

That bias had to be contained somehow, and it was: $\hat\omega_n$ was
confined to $\sqrt{k_x^{app}}\cdot[\sqrt{\alpha_{min}},
\sqrt{\alpha_{max}}]$ so the estimate stayed physical. The containment
had a sharp edge. Pinning at that prior was a routine outcome, and a
pinned $\hat\omega_n$ yields $\hat\alpha$ *exactly* $\alpha_{max}$;
the plausibility test is inclusive, so it passes, and two pinned episodes
agree to the digit, so the consistency gate — the defence that exists
precisely to catch failed identification — reads spread $1.00\times$ and
waves them through. A SITL session on 2026-09-05 updated the $y$ gains
from two such episodes.

$(\ast)$ removes the failure at its source rather than bounding it.
$\alpha$ and $\tau$ are separately identifiable because they enter
differently: $\alpha$ scales the loop gain, moving the poles along the
locus set by $k_x, k_v$, while $\tau$ contributes the third pole,
changing overshoot and ringing without touching the DC gain. Three
denominator coefficients are determined by two unknowns, so the fit is
over-determined. Consequently the solver bounds are now **numerical
sanity only** ($\hat\alpha\in[0.05,10]$), wide enough that
$[\alpha_{min},\alpha_{max}]=[0.4,2.5]$ is applied afterwards as a
judgement on an *unconstrained* estimate — which restores the meaning of
both gates: a parameter resting on a bound once again means the fit
failed, and an implausible $\hat\alpha$ is evidence about the episode
rather than an artefact of the prior. Multi-start is retained, but to
*detect* degeneracy rather than resolve it by preference: if two starts
reach near-equal residuals while disagreeing about $\hat\alpha$ by more
than 20%, the episode is refused as ambiguous.

**Repetition and robust aggregation (median-of-N).** A single 6-second
step fit is a *noisy estimator* of $\alpha$: process noise and the
finite excitation of a small step give it episode-to-episode variance, and because the applied
gain is $k_x = (\omega_n^\star)^2/\hat\alpha$, that variance maps 1:1
(inverted) into the gains — which is why two sessions can end with
visibly different gain sets that describe the same closed loop (the
reproducible quantity is $\omega_n^{eff} = \sqrt{\hat\alpha\,k_x}$, not
$k_x$ itself). The conductor therefore flies `episodes_per_rung` steps
(default 3, alternating ±) per (axis, rung) and updates from the
**median** of the accepted $\hat\alpha$ — robust to one corrupted
episode. A consistency gate refuses *any* update when the accepted
estimates disagree by more than `estimate_consistency` (default 1.35×,
i.e. worst pair within 35%): inconsistent estimates mean the
identification, not the plant, is the problem, and applying any of them
would bake noise into the gains. The stability gate below likewise uses
the median identified $\hat\tau$. Yaw episodes aggregate the identified time
constant $\hat T$ the same way. With the median of 3, the standard error
of the gain update falls by $\approx\sqrt{3}$ *and* single-outlier
sensitivity drops to zero.

### A.7a Session time — where it goes, and what is safe to cut

Repetition costs flight time, and a battery is the hard budget. Four
scheduling changes cut a session by ~3× **without touching a single
quality gate** (measured closed-loop against `quad_sim`: 259 s → 91 s
for `wn_ladder: [1.2, 1.6, 2.0]`, `episodes_per_rung: 3`, same
identified gains):

1. **Bidirectional episodes** (`bidirectional_episodes`, default on).
   The old schedule stepped out to $+d$, recorded, then flew *back* to
   the hover point and threw that leg away. The return leg is itself a
   clean step of size $d$ from a settled state, so it is now recorded:
   the setpoint walks $0 \to +d \to 0 \to -d \to 0 \dots$ and the
   episode yield per unit time doubles. The excursion envelope, the step
   size and the ± alternation are all unchanged.
2. **Quiet-based settling** (`settle_quiet_time`, `settle_tol_pos/vel`).
   A step must start from rest, but *rest* is a condition, not a
   duration: SETTLE now ends once the vehicle holds the setpoint within
   `settle_tol_pos` / `settle_tol_vel` for `settle_quiet_time`. This
   gate is *tighter* than the old fixed wait implied (it is checked, not
   assumed) and `settle_time` remains the hard cap — a noisy or windy
   plant simply degrades to the old fixed-time behaviour.
3. **Adaptive episode length** (`adaptive_episode`). Recording stops
   once the response has held its steady state for `episode_quiet_time`
   within `episode_settle_band`·|step| — flat tail samples carry no
   information about $(\omega_n, \zeta, T_d)$. It can never fire before
   `max(min_episode_time, episode_settle_periods/(\zeta\omega_n))`,
   i.e. before the transient of the *currently applied* loop could have
   finished, and `episode_time` is still the cap. The amplitude check
   ($|\bar y| \ge 0.6|step|$, correct sign) prevents the flat piece
   during the transport delay from being mistaken for settling.
4. **Sequential stopping** (`min_episodes_per_rung`, `early_stop_spread`).
   A bucket ends after `min_episodes_per_rung` reps *only if* every
   flown episode was accepted (no fit rejections, no implausible α) and
   they agree within `early_stop_spread` (1.15×) — deliberately stricter
   than `estimate_consistency` (1.35×), which the full bucket would only
   have to pass. Stopping early therefore requires *better* evidence
   than continuing would guarantee; any discard or disagreement flies
   the full `episodes_per_rung`.

Set `bidirectional_episodes: false` and `adaptive_episode: false` to
reproduce the original fixed-schedule timing exactly.

**Mode supervision.** Episodes only run while PX4 reports OFFBOARD
(`mavros/state`): outside OFFBOARD the vehicle ignores the controller's
setpoints and any "response" is noise. Before OFFBOARD the conductor
streams the current position as setpoint (bumpless engage; the stream is
also what makes PX4 accept the mode switch); leaving OFFBOARD
mid-session pauses tuning — episode discarded, gains kept — and it
resumes from a fresh hover when OFFBOARD returns.

### A.7b The manoeuvre envelope (tuning in confined space)

Identification needs excitation, and excitation needs room. The conductor's
whole demand on space is one step amplitude either side of the hover point:

| Parameter | Default | Axis |
|---|---|---|
| `step_size` | 0.5 m | x, y |
| `step_size_z` | 0.4 m | z |
| `yaw_step` | 0.5 rad (29°) | yaw |

The setpoint walks $0 \to +d \to 0 \to -d \to 0 \dots$ **about the hover
point**, always as `hover + offset` and never as an increment on the current
position, so the commanded envelope is exactly $\pm d$ per axis for a session
of any length. Allow a little more for overshoot (a few % at $\zeta$ 0.95) and
for wind. All three are live-settable — `ros2 param set` or the RViz Tuner
panel — and take effect at the next episode.

**Upper bound.** A step commands its full amplitude as *instantaneous position
error*, so a step at `safety.max_pos_error` trips the abort the moment it is
issued. The accepted ceiling is therefore
`min(max_step_size, 0.8 × safety.max_pos_error)`, and larger values are
refused with that arithmetic in the message.

**Lower bound is set by noise, not by the code.** The identification fits
$(\omega_n, \zeta, T_d)$ to the response, and the quality gate is a
*relative* one (`nrmse < 0.15`), so what matters is response size against
odometry noise. Halving the step halves the signal and doubles the relative
noise. Below `small_step_warn` (0.25 m) the conductor warns; it never silently
tunes to noise — bad fits are rejected and the session ends "keeping gains"
instead. Measured in `quad_sim` (3 mm position noise), 0.2 m steps gave 16/16
accepted episodes at a worst-case nrmse of 0.083 against the 0.15 gate. A real
EKF is noisier: treat 0.3 m as the practical floor outdoors, check `nrmse` in
the report, and raise the amplitude if episodes are being discarded.

Two related knobs scale themselves with the amplitude, so a small step is not
declared settled while the residual is still a large fraction of it:
`settle_tol_frac` (the quiet gate uses the tighter of the absolute and
fractional tolerance) and `episode_settle_floor` (an absolute floor under the
flatness band, so a small step does not demand a flatness finer than the
odometry noise).

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

### A.8b Yaw loop: model, identification, tuning

Yaw authority on a multirotor comes only from rotor drag torque, so the
heading loop is deliberately slower than tilt. The hardened controller
scales the body-z attitude-error component by $2/\tau_{yaw}$
(`yawctrl_tau`) instead of $2/\tau_{att}$, decoupling the two.

Small-angle closed loop for a heading step $\psi_{ref}$:

$$\dot\psi = \frac{2\beta}{\tau_{yaw}}(\psi_{ref}-\psi)
\;\Rightarrow\;
\psi(t) = \psi_{ref}\big(1 - e^{-(t-T_d)/T}\big),
\qquad T = \frac{\tau_{yaw}}{2\beta},$$

where $\beta$ lumps the PX4 yaw-rate-loop efficiency and drag-torque
authority. The conductor identifies $(T, T_d)$ from a yaw step by
first-order least squares, and because $T \propto \tau_{yaw}$ through the
*same* unknown $\beta$, the update

$$\tau_{yaw}^{new} = \tau_{yaw}^{app}\,\frac{T^\star}{\hat T}$$

converges to the target time constant $T^\star$ (default 0.35 s,
i.e. ~3 rad/s heading bandwidth) with $\beta$ cancelling — the exact
analogue of the $\alpha$-correction in A.7. Updates are rate-limited
(factor 1.6) and clamped to $\tau_{yaw} \in [0.15, 1.2]$ s; the safety
monitor watches yaw-rate oscillation energy separately (threshold
1.5 rad/s RMS). Yaw steps hold the hover position constant, and position
steps hold yaw constant, so the identifications don't cross-contaminate.

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
└── geo_tuner/                      <- ROS 2 package (ament_cmake, C++)
    ├── README.md                   <- quick start (build, sim check, the
    │                                  four steps of a campaign)
    ├── docs/ARCHITECTURE.md        <- package layout, nodes, launch args,
    │                                  parameters, ROS interface, outputs
    ├── docs/CONTROLLER_NOTES.md    <- mav_controllers_ros quirks
    ├── include/geo_tuner/core/, src/core/
    │                               <- pure logic: gain_design, step_fit,
    │                                  first_order_fit, least_squares, safety,
    │                                  aggregate (unit-tested)
    ├── src/tuning_conductor.cpp    <- in-flight auto-tuner node
    ├── src/quad_sim.cpp            <- lightweight plant for fast sim tests
    ├── src/nodes/design_gains_main.cpp  <- geo-tuner-design CLI
    ├── scripts/geo-tuner-hover     <- offline ulog analysis (Python/pyulog)
    ├── include/geo_tuner/rviz/, src/rviz/
    │                               <- the five RViz field panels
    ├── launch/sim_tune.launch.py   <- Phase 2 (controller+sim+tuner)
    ├── launch/field_tune.launch.py <- Phases 3 & 4 (tuner only)
    ├── launch/field_monitor.launch.py   <- RViz + panels for a field session
    ├── config/tuner_field.yaml     <- field session configuration
    └── test/test_core.cpp, test/test_schedule.cpp   <- 65 unit tests
```
