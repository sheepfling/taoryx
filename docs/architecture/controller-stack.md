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

Important current boundary: the X8 racetrack fixture declares and runs an
attitude LQR with a direct canonical moment demand. Its elevon tables and
control values remain in the aerodynamic plant, but that fixture does not yet
prove that an LQR moment is allocated through a physically identified elevon
allocator. That is a future surface-authority qualification, not something
the current racetrack result should imply.

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

The current X8 showcase is best understood as this chain:

```text
initial source-bounded state
    → declared racetrack timing and phase geometry
    → local tangent velocity reference
    → bounded position and altitude correction
    → desired route direction + scheduled bank
    → attitude error and body-rate error
    → generic runtime attitude LQR
    → direct canonical moment demand
    → rigid-body aerodynamic/propulsive plant
    → quaternion/body-rate and translational EOM
    → truth-based altitude, bank, turn, and terminal-gate evaluation
```

The route phase is not itself a controller transition. It is a changing
guidance reference. The objective evaluator independently checks the truth
trajectory at the high-altitude gate, left-turn bank, left-turn exit, low-
altitude gate, right-turn bank, and start/finish gate.

The run therefore currently proves a nominal source-bounded rigid-body
trajectory under a reusable LQR direct-moment control seam. It does not yet
prove:

- a source-validated LQR gain;
- physical elevon allocation and actuator dynamics;
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
