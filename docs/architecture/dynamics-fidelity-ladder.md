# Dynamics fidelity ladder

TAORYX exposes three equations-of-motion modes and four evidence tiers.  The
first three tiers are dynamics reductions; the fourth is a control-realization
split inside the rigid-body mode.  All tiers share environment, table,
propulsion, mass, guidance, and telemetry contracts so a vehicle can be
developed progressively without duplicating its mission contract.

For the operator-facing setup, trim declaration, commands, and failure
diagnostics, follow [Trim And Fidelity Walkthrough](../plan/trim-and-fidelity-walkthrough.md).
For the promotion order, evidence packet, and DaveML/IMU alignment, follow
[Fidelity-first vehicle integration program](../plan/fidelity-first-integration-program.md).

## Modes

| Mode | Translation | Attitude | Moments/inertia |
| --- | --- | --- | --- |
| `point-mass` | Force-integrated 3-DOF | Implicit/prescribed by the point-mass equations | Not integrated |
| `kinematic-6dof` | Force-integrated translation | Controller-prescribed quaternion/body-rate sidecar | Not integrated |
| `rigid-body-6dof` — direct/induced wrench | Force-integrated translation | Quaternion and body rates from Newton–Euler equations | Fully integrated; controller may add a declared generalized force/moment |
| `rigid-body-6dof` — surface allocated | Force-integrated translation | Quaternion and body rates from Newton–Euler equations | Fully integrated; declared surfaces/rotors/thrusters are solved and applied |

The runtime has three mode keywords, but a rigid-body run is not automatically
a physical-actuator run.  The direct/induced-wrench tier integrates the rigid
body while a controller supplies a generalized wrench or uses source-induced
loads.  It is now a first-class catalog bridge named
`rigid_body_6dof_direct_wrench`, between the pseudo-6DOF response law and the
surface-allocated tier.  It requires checked source-load, requested-wrench,
achieved-wrench, and residual-wrench evidence, but it does not require or imply
elevon, rotor, gimbal, motor, wheel, or thruster allocation.  The
surface-allocated tier adds an explicit bounded map from control demand through
declared effectors and actuator limits.  A direct moment is not allowed to
carry a surface-allocation claim.

The kinematic mode is a bridge for model development: it exercises the same
translational plant and aerodynamic queries while attitude is supplied by a
simple declared response law instead of being produced by moments.

The four stable showcase names are:

```text
point_mass_3dof
pseudo_6dof_kinematic_bridge
rigid_body_6dof_direct_wrench
rigid_body_6dof_surface_allocated
```

The last two share the `rigid-body-6dof` parser/runtime mode but have different
claim boundaries, telemetry, and qualification requirements.

## Kinematic bridge syntax

The bridge is a TAORYX successor extension and is not claimed as historical
TAOS syntax:

```text
*mode kinematic-6dof
*runtime status attitude mode=lag roll-deg=0 pitch-deg=4 yaw-deg=30 lag-s=0.5 max-rate-deg-s=180
```

The attitude status declaration supports:

- `mode=lag`: track fixed roll, pitch, and yaw targets through a first-order
  body-rate command with `lag-s` and `max-rate-deg-s`;
- `mode=route-lag`: derive the roll target from the active racetrack turn,
  pitch from flight path plus the declared alpha reference, and yaw from the
  route tangent; then track those targets through the same bounded response
  law. This is a kinematic mission-response proof, not a moment or effector
  model;
- `mode=prescribed`: the same fixed-angle target contract using the default
  lag and rate limit;
- `mode=rate`: prescribe `roll-rate-deg-s`, `pitch-rate-deg-s`, and
  `yaw-rate-deg-s` directly.

The bridge maintains a normalized quaternion and publishes `qw`, `qx`, `qy`,
`qz`, response-law Euler command/achieved channels, and commanded body-rate
channels in the run history. It does not generate aerodynamic moments,
estimate inertia response, or claim rigid-body stability.

## Recommended migration sequence

1. Run the vehicle in `point-mass` mode to verify trajectory geometry, energy,
   atmosphere, propulsion, and table queries.
2. Use `kinematic-6dof` with the same inputs to verify attitude-dependent force
   transforms, prescribed bank/pitch/yaw profiles, and controller timing.
3. Move to `rigid-body-6dof` with a direct/induced-wrench controller to verify
   coupled translation/rotation and source-load closure without claiming
   physical effector allocation.
4. Replace the direct-wrench path with declared control surfaces, rotors,
   thrusters, or wheels and a bounded allocator before claiming the
   surface-allocated tier.
5. Compare histories over the same initial condition and declared maneuver.
   Differences should be attributed to the named tier change.

Promotion is ordered, even though diagnostic execution is not. A higher-tier
case may be run to expose a missing model input, but it remains
`diagnostic_only` until the preceding tier has promoted evidence for the same
vehicle identity and mission boundary. Data readiness, runtime execution,
qualification, and promotion are separate statuses.

The bridge is therefore a diagnostic and development mode, not evidence that a
vehicle has passed full 6-DOF verification.

## Two separate evidence experiments

The ladder produces two different kinds of evidence and they must not be
overlaid as though they answer the same question.

### Reduction parity

This asks whether a lower-fidelity model is the reduction of the same plant.
The initial physical state, canonical units, environment, wind, source tables,
propulsion, mass model, command history, event schedule, duration, and
termination policy must all match. Only the equations-of-motion tier may differ.

The contract auditor compares these fields before a parity plot is accepted.
The current long showcase cases intentionally fail this gate because they use
different problem files, table sets, or durations. They remain useful mission
diagnostics, but are not reduction evidence.

### Fidelity separation / mission evidence

This asks what happens when attitude, rates, actuators, moments, and controller
coupling are released. A free rigid-body run is not expected to overlay a
prescribed-attitude reduction. It must instead provide source-model parity,
trim, independent equation closure, convergence, envelope compliance, and
declared controller metrics.

The evidence packet therefore has separate sections:

1. reduction parity, with comparable contracts only;
2. long mission behavior, with termination reasons and expected divergence;
3. source and numerical evidence supporting both claims.

## Continuity and event policy

The sampled state contract forbids position, velocity, and attitude jumps. A
mass jump is allowed only for a declared separation or jettison event. A
velocity or attitude jump is allowed only when the problem declares an impulse
or reset and records the event ID, time, pre-event state, post-event state,
impulse, mass change, and reason. Changes in force, throttle, or control
commands may be discontinuous; their integrated state response may not be.

`taoryx.validation.continuity_audit` applies scenario-specific one-sample
tolerances and reports unexplained jumps separately from declared event jumps.
It is deliberately an audit utility rather than a universal physics
tolerance: the allowed state increment depends on the output sampling rate
and vehicle scale.

Every plot and packet must distinguish commanded/reference channels from
actual integrated state channels. A trajectory that stops must include its
termination time, reason, last valid table coordinates, and envelope margins.

The common source-anchored plant harness now records two independent closure
checks during the rigid-body firewall: finite-difference inertial acceleration
against the emitted ECIC force sum, and finite-difference body-rate dynamics
against the emitted body moment sum. These checks are separate from the
integrator's direct RHS residual and are reported as evidence before any
controller or mission claim is made.

## Control-surface table composition

When a vehicle supplies several coefficient families with the same output
names, the table binder keeps the static family available through the ordinary
`cx`, `cy`, `cz`, `cmx`, `cmy`, and `cmz` references and exposes the control
families through qualified aliases. The rigid-body table model can compose a
static coefficient with each qualified control family as:

```text
C(adopted) = C(static) + sum(C(control, command) - C(control, zero))
```

This is an additive control-increment contract. It is appropriate only when
the source deck documents independently transcribed control families; a
combined-control table or a source-specific interaction model remains a
separate evidence requirement. Queries outside any participating table’s
declared envelope fail closed.

## Generic fixed-wing control holds

The existing guidance hold contracts select the actuator declared by the
problem file. `alpha-hold-gain-deg-per-deg` and `alpha-hold-target-deg` can
therefore drive an `elevator-deg`, `collective-elevon-deg`, or
`symmetric-stabilator-deg` control. Likewise, the sideslip hold can use a
`differential-elevon-deg` or `differential-stabilator-deg` control, with the
corresponding `*-hold-target-deg` attribute. This keeps the controller
semantics reusable across the B747, X8, and X-15 source decks.

Actuator bounds are still authoritative. The table evaluator tolerates only
machine-scale roundoff at an exact declared boundary and clamps that value to
the boundary before interpolation; a real out-of-envelope query remains a
diagnostic and stops the run.
