# TAORYX controller stack and fidelity mapping

Status: current implementation contract and promotion boundary

This document answers a practical question: when a TAORYX vehicle follows a
mission, which parts are generic and which parts are vehicle-specific?

The short answer is that TAORYX does not use one universal controller and it
does not use one bespoke controller per vehicle either. It uses a shared
control lifecycle with replaceable guidance laws, regulators, allocators, and
plant adapters.

The current architecture is:

```text
trim / operating point
        ↓
mission and segment supervisor
        ↓
guidance / path and waypoint reference
        ↓
achievability and command projection
        ↓
vehicle regulator
        ↓
allocator / control-surface or rotor realization
        ↓
actuator limits and dynamics
        ↓
vehicle plant and equations of motion
        ↓
truth telemetry, events, and independent evaluation
```

The generic interfaces are implemented in `taoryx.control`. The guidance
geometry and vehicle adapters remain the places where family-specific physics
enter. LQR is a reusable regulator option, not a requirement that every
vehicle or every fidelity use LQR.

## 1. What is generic and what is bespoke?

| Layer | Shared TAORYX contract | Vehicle- or mission-specific realization |
| --- | --- | --- |
| Operating point | `TrimSpec`, `TrimResult`, residual and bound contracts | The plant evaluator, source data, frames, trim variables, and residual meanings |
| Mission lifecycle | `SegmentPlan`, segment schedule, entry/exit events, objective types | The segment sequence, route shape, terminal contract, and family objective |
| Guidance output | Named reference or `ControlDemand` | Racetrack, waypoint, ProNav, glide, orbit, hover, transition, and vehicle-specific guidance law |
| Regulator | Controller protocol; LQR, gain-scheduled LQR, and explicit fallback slots | Selected state channels, gains, response law, operating-point schedule, and fallback policy |
| Allocation | `ControlAllocator` and bounded `ControlCommand` | Elevon mixing, surfaces, rotor mixing, thrust-vectoring, wheels, RCS, or direct moments |
| Plant | `VehiclePlant` / `PlantEvaluation` | Aerodynamic tables, propulsion, mass properties, loads, actuator dynamics, and EOM |
| Qualification | Common telemetry, truth evaluator, convergence, parity, and failure contracts | Family tolerances, evidence class, and nonclaims |

The distinction matters. A generic LQR can regulate a local attitude model,
but it does not know what a waypoint means. A racetrack guidance law can
produce a desired velocity and bank reference, but it does not know whether
that bank is produced by elevons, rotor moments, a reaction wheel, or a
prescribed pseudo-6DOF response.

## 2. The six controller levels

### Level 0 — Trim and operating-point selection

Trim is not guidance. It finds a physically consistent state and control
combination around which guidance and regulation can operate.

`TrimSpec` declares:

```text
state names       → alpha, beta, q, speed, altitude, ...
control names     → elevator, throttle, rudder, ...
residual names    → force, acceleration, moment, or rate residuals
bounds            → permitted state and control ranges
scales            → numerical normalization for unlike units
initial guess     → source or engineering starting point
```

The vehicle adapter supplies the residual evaluator. The generic solver uses
a bounded nonlinear least-squares solve; it does not invent aerodynamic,
propulsion, mass, or inertia equations.

Conceptually, a rigid-body trim solves the equilibrium conditions:

```text
translational residual = applied force − required steady acceleration
rotational residual    = applied moment − required steady angular acceleration
resource residual      = any declared steady-state resource condition
```

For straight-and-level flight this commonly means near-zero body
accelerations and moments at the selected speed, altitude, mass, attitude, and
control setting. For a hover, orbit, release, or powered climb the residuals
are different and must be named by the family adapter.

An accepted trim artifact contains the solved state, controls, unscaled
residuals, scaled residual norm, bounds, source/model identity, and operating
conditions. A source-provided trim may be imported instead, but it must carry
the same information and be distinguished from a newly solved trim.

### Level 1 — Mission and segment supervisor

The supervisor decides which segment owns the current time and which target
contract is active. It resets the controller at declared segment boundaries
and records transitions.

Examples:

```text
trim hold → climb → route leg → altitude capture → terminal gate
spool → hover → translation → landing → post-contact settle
release → burn → cutoff → coast → reentry corridor
```

This level does not directly move a surface. It provides the active objective,
target, limits, timeout, and handoff policy to guidance.

The independent mission evaluator is separate from this level. A supervisor
transition such as `CAPTURED` is diagnostic evidence; it is not by itself proof
that truth telemetry satisfied the objective.

### Level 2 — Guidance and reference generation

Guidance turns mission intent into a kinematic or attitude reference. It is
where much of the current vehicle/mission-specific behavior lives.

For a waypoint or path, the basic error is a truth-state difference:

```text
position error       = target position − current position
velocity error       = target velocity − current velocity
cross-track error    = distance from the declared path or gate
altitude error       = target altitude − current altitude
speed error          = target speed − current speed
heading error        = wrapped target course − current course
```

Those errors are transformed according to the declared objective. A
`fly_over` produces a capture target. A `fly_by_gate` produces an oriented
crossing reference. A `path_corridor` produces a path-tangent and bounded
cross-track correction. A terminal or intercept segment may produce a
ProNav-style acceleration demand rather than a waypoint velocity.

The guidance output can be:

```text
3DOF:          desired acceleration, lift vector, bank, flight path, throttle
pseudo-6DOF:   desired velocity plus attitude, bank, pitch, or body-rate intent
rigid 6DOF:    desired velocity/attitude/rates or a direct effector demand
```

For the X8 racetrack, the current path is:

```text
racetrack geometry
    → desired local tangent velocity
    → bounded position and altitude capture correction
    → route direction and scheduled bank reference
    → attitude error
```

The altitude capture term changes the requested velocity vector. It does not
add vertical force directly. The rigid-body plant must still create the
resulting flight-path response through its integrated loads and attitude.

The route geometry is reusable, but its numerical gains, turn radius, climb
rate, descent rate, bank schedule, and terminal gate are scenario parameters.
They are not universal aircraft constants.

### Level 3 — Vehicle regulator

The regulator converts a reference error into a generalized demand appropriate
to the fidelity.

The current regulator choices are:

1. **LQR / gain-scheduled LQR** — the primary reusable regulator path when a
   named trim, local `A/B` linearization, `Q/R` design, and channel contract
   exist.
2. **Bounded attitude-moment fallback** — an explicit runtime fallback for
   rigid-body bring-up when no LQR is declared.
3. **Kinematic response law** — a declared first-order or rate-limited
   attitude bridge for pseudo-6DOF.
4. **Direct or family-specific law** — appropriate for 3DOF commands,
   rotorcraft wrench control, ProNav, staged propulsion, or other cases where
   a local attitude LQR is not the right abstraction.
5. **Legacy PID-like baseline** — retained for regression comparison only; it
   is not controller-qualification eligible.

LQR is therefore generic in implementation, but local in meaning. Its gain is
computed for the declared operating point and exact named state/control order.
It is not a universal gain that can be copied between a Cessna, X8, helicopter,
rocket, and spacecraft.

For a rigid-body attitude LQR, the current demand is conceptually:

```text
state error = declared attitude error and body-rate error
moment demand = moment trim − K · state error
```

The runtime validates dimensions, controllability, finite `Q/R`, and closed
loop poles. A gain-scheduled design can rebuild the local controller when the
declared mass or inertia operating point changes. The controller never infers
inertia from mass without an explicit vehicle-provided schedule.

LQR design requires a local linearization. The preferred source is a
finite-difference linearization of the same plant evaluator used in the run:

```text
accepted trim + plant evaluator
    → perturb each named state/control
    → finite-difference state derivatives
    → local A/B artifact
    → LQR solve and robustness screen
```

The older residual Jacobian used for trim diagnostics is not interchangeable
with a dynamics `A/B` matrix.

### Level 4 — Allocation and actuator realization

The regulator's generalized demand is not automatically a physical actuator
command. Allocation applies the vehicle topology, limits, rates, and
couplings.

Examples:

```text
moment demand       → elevator / aileron / rudder or elevon commands
collective + moment → quadrotor rotor speeds
body wrench         → RCS thruster pulses
torque demand       → reaction-wheel torque
magnetic torque     → bounded dipole command
thrust-vector demand→ gimbal angles and engine commands
```

The allocator reports requested values, achieved values, saturation, rate
limiting, residual, and lost authority. These are separate from the truth
state and must be visible in qualification artifacts.

Important current boundary: the X8 direct-moment racetrack is retained as an
explicit baseline, while the active racetrack fixture now opts into a bounded
local elevon-table inversion. The active fixture is an allocator candidate,
not yet a qualified full flight-control system: it exposes travel and a
bounded deflection offset from the stored base, but not a time-normalized
actuator-rate model. If it
leaves the declared beta table envelope, it must fail closed rather than gain
authority through a hidden direct yaw moment.

### Level 5 — Plant and equations of motion

The plant evaluates the applied command and returns forces, moments, mass
flow, and diagnostics. The EOM then commits the next state according to the
standard truth-time contract.

Controllers do not mutate position, velocity, attitude, mass, or fuel
directly. They request bounded commands. The plant determines what actually
happens, and the telemetry records both requested and achieved values.

## 3. How the three fidelities use the stack

### Point-mass 3DOF

```text
truth position/velocity/resources
    → mission guidance and waypoint/path error
    → achievable acceleration / lift-vector / throttle projection
    → direct 3DOF command allocator
    → force and resource model
    → translational EOM
```

3DOF can prove center-of-mass path, energy, speed, altitude, range, mass, and
resource behavior represented by the model. It cannot prove physical
attitude, body rates, moments, or individual surfaces/rotors/wheels.

Trim at this level is usually a force/energy operating point or an imported
vehicle performance anchor. It does not create a hidden attitude state.

### Kinematic pseudo-6DOF / 3T+3K

```text
truth position/velocity + prescribed attitude state
    → guidance reference
    → bounded attitude/rate response law
    → quaternion and body-rate sidecar
    → force transformation and translational plant
    → translational EOM
```

The pseudo-6DOF bridge can show attitude lag, rate limits, bank/pitch/yaw
response, and terminal orientation feasibility. Its attitude is generated by
the declared response law, not by integrated physical moments. It must not
claim surface, rotor, wheel, or thruster activity unless those are explicitly
modeled as part of the bridge.

### Rigid-body 6DOF

```text
truth state at accepted time
    → mission/path guidance
    → attitude, rate, or wrench reference
    → LQR / bounded regulator / family controller
    → generalized moment or wrench demand
    → physical allocator and actuator dynamics
    → aerodynamic, propulsion, contact, or environmental loads
    → Newton–Euler 6DOF EOM
```

Rigid-body 6DOF can prove only the physical mechanisms actually present in the
selected plant and allocator. A direct-moment controller is a valid
engineering control path, but it is not the same evidence as a surface-driven
or rotor-resolved controller.

## 4. A concrete X8 racetrack walk-through

The X8 showcase now has two deliberately comparable realizations:

```text
initial source-bounded state
    → declared racetrack timing and phase geometry
    → local tangent velocity reference
    → bounded position and altitude correction
    → desired route direction + scheduled bank
    → attitude error and body-rate error
    → generic runtime attitude LQR
    → either direct canonical moment demand (baseline)
      or bounded local elevon inversion (candidate)
    → rigid-body aerodynamic/propulsive plant
    → quaternion/body-rate and translational EOM
    → truth-based altitude, bank, turn, and terminal-gate evaluation
```

The route phase is not itself a controller transition. It is a changing
guidance reference. The objective evaluator independently checks the truth
trajectory at the high-altitude gate, left-turn bank, left-turn exit, low-
altitude gate, right-turn bank, and start/finish gate.

The direct baseline proves a nominal source-bounded rigid-body trajectory under
a reusable LQR direct-moment control seam. The surface candidate tests whether
the same demand can be realized through the declared X8 elevon tables. Neither
run by itself proves:

- a source-validated LQR gain;
- time-normalized physical elevon actuator dynamics;
- wind robustness;
- family-wide controller qualification;
- 3DOF/pseudo-6DOF reduction parity for the same exact scenario.

## 5. What “generic controller” should mean going forward

The reusable object should be the controller contract and realization schema,
not one universal algorithm:

```text
GuidanceReference
    → GeneralizedControlRequest
    → ControllerRealization
    → AllocationResult
    → ActuatorState
```

Each realization should declare:

- fidelity and controller role;
- trim artifact and operating point;
- input state names, units, frames, and order;
- output demand names, units, frames, and order;
- design method and matrix hashes;
- gain schedule or response-law identity;
- allocator identity;
- limits, rates, saturation policy, and fallback;
- provenance, evidence level, and nonclaims.

This gives us one language for comparing an LQR fixed-wing regulator, a
helicopter cyclic/collective controller, a quadrotor wrench allocator, an RCS
pulse controller, and a magnetorquer detumble law without pretending they are
the same algorithm.

## 6. Qualification gates for each layer

| Layer | Minimum evidence |
| --- | --- |
| Trim | Residuals, bounds, source/model identity, repeatability, and operating point |
| Guidance | Independent truth objective result, reference continuity, error units, and no hidden target reset |
| Regulator | Named channels, `A/B` or response-law provenance, stability/response evidence, limits, and saturation telemetry |
| Allocator | Requested versus achieved command, allocation residual, rate limits, topology, and failure behavior |
| Plant/EOM | Force/moment closure, resource continuity, frame checks, state continuity, and timestep convergence |
| Mission | Ordered segments, valid transitions, terminal gate, batch/step parity, and robustness classification |

An attractive trajectory is not enough to promote a controller. Conversely, a
stable LQR pole set is not enough to promote a vehicle mission. The controller,
allocator, plant, and independent mission evaluator must agree about what was
actually commanded and achieved.

## 7. Current implementation and next promotion steps

Current shared foundation:

- generic trim and local-linearization contracts;
- generic controller and allocator protocols;
- implemented fixed-point LQR and gain-scheduled LQR support;
- explicit fallback and legacy-baseline classifications;
- reusable waypoint/path/objective and truth-evaluation contracts;
- rigid-body direct-moment and selected family allocator paths;
- kinematic pseudo-6DOF attitude bridge;
- point-mass direct-control adapters.

Next controller-stack work should be promoted in this order:

1. Make every showcase manifest expose the full chain above, including the
   active guidance law, regulator, allocator, and plant IDs.
2. Add a common achievability projector and achieved-command telemetry for all
   three fidelities.
3. Add a source-trim `A/B` and controller qualification artifact for each
   vehicle before using `controller_qualified`.
4. Complete the X8 elevon-driven path as a separate surface-authority case;
   retain the direct-moment racetrack as its honest baseline.
5. Bind pseudo-6DOF response laws and 3DOF command adapters to the same
   semantic guidance references and compare them through explicit
   cross-fidelity reports.

The result is a controller architecture that is generic at the seams and
specific where physics require specificity.

## 8. Two worked objective-to-actuator examples

The two current showcase vehicles make the distinction concrete:

| Vehicle | Physical type | Current nested control shape |
| --- | --- | --- |
| Skywalker X8 | Fixed-wing rigid-body 6DOF | Racetrack/path guidance → attitude reference → attitude LQR → direct canonical moment → aerodynamic/propulsive plant |
| Hummingbird | Quadrotor rigid-body 6DOF | Position/velocity guidance → desired force/thrust direction → bounded attitude moment law + yaw law → collective and quad-X rotor allocation → rotor plant |

Both are rigid-body 6DOF cases, but they do not use the same guidance or
actuator realization. The shared pieces are the contracts and telemetry
boundaries.

### 8.1 X8: racetrack objectives resolving to an attitude regulator

The current X8 racetrack has six independently evaluated mission objectives:

```text
high-altitude level gate
left-turn bank event
left-turn exit gate
low-altitude level gate
right-turn bank event
start/finish fly-by gate
```

The objective declarations define truth acceptance: gate plane, crossing
direction, altitude, speed, corridor, and time window. They do not directly
provide a control command. The route reference and the evaluator are separate:

```text
objective contract / route geometry
              │
              ├── guidance reference used by the controller
              │
              └── independent truth evaluator used for pass/fail
```

For the guidance branch, define local position and velocity errors:

\[
e_p = p_{ref}(t)-p(t), \qquad
e_v = v_{ref}(t)-v(t).
\]

The racetrack reference supplies a tangent velocity \(v_t\), and the current
implementation adds bounded capture terms:

\[
v_{cmd} = v_t
       + \operatorname{sat}_{v_{max}}(K_p e_p)
       + \hat r\,\operatorname{sat}_{\dot h_{max}}(K_h e_h),
\]

where \(e_h=h_{ref}-h\), \(\hat r\) is the local outward radial direction,
and the scheduled vertical component is retained when the phase is climbing
or descending. The phase geometry supplies the left/right bank reference and
the route tangent; the vehicle does not receive a force injection from this
calculation.

The desired body attitude is built from the route tangent and bank reference.
The attitude error is represented in the local body frame as a small-angle
three-vector, schematically:

\[
e_R \approx \frac{1}{2}
\sum_{i=1}^{3} b_i \times b_{i,ref}.
\]

The current X8 LQR then receives attitude and body-rate error channels:

\[
x_X =
\begin{bmatrix}
e_{R,x} & e_{R,y} & e_{R,z} & p & q & r
\end{bmatrix}^{T}.
\]

The generalized demand is a canonical body moment:

\[
M_{cmd} = M_{trim} - K_X x_X,
\]

followed by moment and body-rate limits. The rigid-body plant evaluates the
actual aerodynamic and propulsion loads:

\[
\begin{aligned}
F_b &= F_{aero}(\alpha,\beta,p,q,r,\delta)
      +F_{prop}(T,\text{throttle})+F_{other},\\
M_b &= M_{aero}(\alpha,\beta,p,q,r,\delta)
      +M_{prop}+M_{other},\\
\dot v_b &= F_b/m + g_b - \omega\times v_b,\\
\dot\omega &= I^{-1}(M_b-\omega\times I\omega).
\end{aligned}
\]

The direct baseline applies \(M_{cmd}\) through the canonical generalized
moment path. The current default LQR plant bridge uses rigid-body inertia for
its moment input matrix; it does not yet include the X8 aerodynamic
stiffness/damping derivatives in the attitude design. Consequently, a
stable LQR pole set does not guarantee that the direct moment will overcome
the source aerodynamic load at a commanded bank. The direct case must remain
an integration baseline until a plant-aware linearization or bounded
closed-loop tuning study closes that gap.

The surface candidate instead uses a bounded local solve of the
form

\[
\delta_{elevon}^{*}
=\arg\min_{\delta\in\mathcal U}
\left\|M_{aero}(\delta)-M_{cmd}\right\|_{W}^{2}
\]

with the source elevon tables. Its allocation residual and saturation state are
logged, but the current maximum-delta setting bounds deflection relative to the
stored base rather than implementing a time-normalized actuator rate. The
direct and surface cases are separate evidence classes and must not be
collapsed into one claim.

The objective evaluator independently checks the resulting truth history:

```text
truth trajectory → gate-plane crossing, altitude, speed, and timing
truth attitude   → bank and pitch event checks
truth telemetry  → envelope, saturation, and terminal evidence
```

Thus the X8 stack is currently a reusable guidance plus generic LQR regulator
experiment, with direct-moment realization and source-bounded aerodynamic
loads.

### 8.2 Hummingbird: position, attitude, yaw, collective, and rotor allocation

The Hummingbird strict landing objective set is:

```text
initial hover dwell
motor-shutdown event
touchdown state and post-contact dwell
terminal landing contract
```

The Hummingbird guidance is a cascade because a position error is not directly
an actuator command. The outer position/velocity loop first forms a desired
acceleration:

\[
a_{cmd} = K_p e_p - K_v v_{rel}
       + \hat r\left(K_h e_h-K_{\dot h}\dot h\right).
\]

The desired thrust force includes gravity compensation:

\[
F_{cmd}=m\left(a_{cmd}-g\right),
\qquad
\hat t_{cmd}=\frac{F_{cmd}}{\|F_{cmd}\|}.
\]

The thrust-axis error becomes an attitude demand:

\[
e_{tilt}=\hat t_{body}\times\hat t_{cmd}.
\]

The current Hummingbird path uses a bounded attitude-moment law rather than
the X8 attitude LQR:

\[
M_{tilt}=\operatorname{sat}_{M_{max}}
\left(K_R e_{tilt}-K_\omega\omega\right).
\]

If a yaw target is declared, it is an additional nested heading loop:

\[
e_\psi=\operatorname{wrap}(\psi_{ref}-\psi),
\qquad
M_z=\operatorname{sat}_{M_{z,max}}
\left(K_\psi e_\psi-K_r r\right).
\]

Collective is adjusted from the altitude demand, radial rate, and tilt
compensation:

\[
\Omega_c = \operatorname{clip}\left(
  \Omega_0 + K_\Omega
  \left(K_h e_h-K_{\dot h}\dot h+\Delta_{tilt}\right),
  \Omega_{min},\Omega_{max}\right).
\]

The quad-X allocator maps collective and body moments into rotor-speed
commands. With \(s_i=\Omega_i^2\), the idealized relationship is:

\[
\begin{bmatrix}T\\L\\M\\N\end{bmatrix}
= A_{quadX}
\begin{bmatrix}s_1\\s_2\\s_3\\s_4\end{bmatrix},
\qquad
s^{*}=\operatorname{clip}_{s_{min},s_{max}}
\left(A_{quadX}^{-1}w_{cmd}\right).
\]

The implemented allocator retains the bounded solve and reports individual
rotor speed commands and saturation. The plant then produces thrust and
reaction torque from those actual rotor commands; the EOM, not the guidance
law, determines the realized translation and attitude.

The important contrast is:

```text
X8:         route error → attitude LQR → moment demand
Hummingbird: position error → force/thrust direction → bounded moment
             + collective/yaw → rotor allocation
```

This is why “generic LQR controller” is not an accurate description of the
whole system. LQR is one reusable inner regulator. The outer guidance and the
actuator realization are family-specific.

## 9. Objective resolution as a typed compilation step

The objective-to-controller relationship should be presented as a compilation
pipeline rather than as a hidden controller trick:

```text
ObjectiveSpec
  + vehicle family
  + fidelity
  + environment
  + trim / operating point
  + controller realization
        ↓ resolve
GuidanceReferenceSpec
        ↓ execute at accepted truth time
GuidanceReference(t)
        ↓ project to capability and limits
GeneralizedControlRequest(t)
        ↓ regulate and allocate
ControlCommand(t)
        ↓ integrate
TruthState(t + Δt)
        ↓ independently evaluate
ObjectiveResult
```

For every objective, the resolved artifact should identify:

- objective type and target geometry;
- reference generator and its gains;
- regulator role and design ID;
- allocator and actuator topology;
- limits, rates, and saturation policy;
- truth channels used for independent evaluation;
- terminal or transition semantics;
- claim and nonclaim.

This makes it possible to answer, for example, “the X8 missed altitude at
the gate because the guidance reference was infeasible,” separately from “the
LQR tracked the attitude reference but the elevon allocator saturated.”

## 10. Where automatic tuning is weak today

The current implementation has useful automation at the local LQR-matrix
level, but much of the nested mission stack is still manually parameterized.

| Area | Current state | Weakness | Recommended automation |
| --- | --- | --- | --- |
| Trim | Generic bounded solver and source-trim adapters | Not every showcase starts from a newly solved, frozen trim artifact | Multi-start trim, continuation over speed/altitude/mass, automatic residual and margin gates |
| Local linearization | Finite-difference `A/B` utility exists | Source-consistent `A/B` artifacts are not yet universal across families | Automatically generate and hash `A/B` at every accepted operating point |
| LQR `Q/R` | Scaled profiles and pole/uncertainty checks exist | Mission-level tracking and saturation are not part of the design objective | Constrained `Q/R` search using settling time, overshoot, rate, moment, and table-margin metrics |
| X8 outer guidance | Racetrack gains, bank, altitude capture, and timing are hand-selected | Coupling between route geometry, bank response, and terminal closure is still tuned by reruns | Constrained trajectory optimization over capture gains, bank schedule, phase timing, and horizon |
| X8 actuator realization | Direct moment path works; elevon tables exist | No physical moment-to-elevon allocation qualification in the racetrack | Bounded nonlinear allocation with rate/travel limits and allocation residual objective |
| Hummingbird position loop | Hand-set position/velocity/altitude gains | No formal cascade bandwidth separation or source-trim LQR equivalent | Cascade autotuning with inner attitude/rate bandwidth fixed before outer position tuning |
| Hummingbird yaw loop | Explicit proportional/rate-damping law | Yaw exercise and torque authority are not yet tuned from a common response target | Step-response identification and bounded yaw-gain search with rotor saturation penalty |
| Rotor collective | Hand-set collective gain and tilt compensation | Thrust margin, battery/resource, and attitude coupling are not jointly optimized | Hover trim plus constrained energy/altitude response tuning |
| Objective geometry | Timing estimator and declared truth gates exist | The optimizer does not yet choose physically compatible gate spacing and controller gains together | Preflight feasibility oracle plus route-geometry synthesis |
| Cross-fidelity tuning | Same concepts are documented | No automatic parameter mapping from 3DOF to pseudo-6DOF to 6DOF | Fit reduced-order response parameters to higher-fidelity step and mission fragments |
| Robustness tuning | Fixed perturbation and evidence contracts exist | Nominal tuning can still be separated from robustness optimization | Tune against worst-case or percentile objective margin, not nominal score alone |

The most important weak seam is the boundary between guidance and physical
actuation. A controller can be mathematically stable while the requested
reference is physically unattainable, or a mission can pass a broad gate while
the actuator layer is saturated. The automatic tuner must therefore score the
whole nested chain.

## 11. Recommended automatic-tuning order

Do not tune all gains at once. Preserve the nested-loop structure:

```text
1. plant units, signs, trim, and resource closure
2. actuator limits and allocation feasibility
3. body-rate response
4. attitude response
5. speed / alpha / altitude inner holds
6. position, waypoint, and route guidance
7. mission timing and gate geometry
8. fixed perturbation and cross-fidelity robustness
```

For each stage, freeze the lower-level result and expose only the parameters
owned by the next level. A candidate should be rejected before the outer loop
is scored when it has invalid trim, unstable inner poles, table-domain
violations, actuator saturation beyond policy, or resource inconsistency.

The resulting objective for automated tuning should be a structured vector,
not only one scalar:

\[
J = \left[
\begin{array}{c}
\text{required-objective failures}\\
\text{terminal miss and worst margin}\\
\text{settling / overshoot / tracking error}\\
\text{control and actuator saturation}\\
\text{table and envelope margin}\\
\text{energy or resource use}\\
\text{robustness failure count}
\end{array}
\right].
\]

Use feasibility and hard constraints first, then optimize soft performance.
This prevents a controller from earning a better score by trading away a
required altitude gate, actuator authority, or terminal condition.

## 12. Presentation summary

The coworker-facing summary is:

```text
Objectives say what must happen.
Guidance turns objectives into references.
Regulators turn reference errors into generalized demands.
Allocators turn generalized demands into physical commands.
The plant turns physical commands into forces and moments.
The EOM turns forces and moments into truth state.
The independent evaluator decides whether the objective happened.
```

For the current examples:

```text
X8          objective → racetrack reference → attitude LQR → direct moment → plant
Hummingbird objective → acceleration/thrust reference → bounded attitude/yaw
            moments + collective → quad-X allocator → rotor plant
```

That is the architectural story to present: shared seams, nested mathematics,
family-specific physics, and an explicit list of the seams that still need
automatic tuning or stronger qualification evidence.
