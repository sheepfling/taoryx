# Trim And Fidelity Walkthrough

This is the shortest supported path from a vehicle data package to a
reproducible trim and fidelity comparison. The trim solver is generic; the
vehicle adapter owns the equations, frames, units, table queries, and
operating-point interpretation.

## 1. Choose The Equation Tier

Do not start by copying a 6-DOF problem and deleting state names. Choose the
claim first:

| Tier | Integrated equations | What must be trimmed | What it does not prove |
| --- | --- | --- | --- |
| `point_mass_3dof` | `m * v_dot = sum(F)` | Translational force or acceleration equilibrium | Attitude dynamics, moments, inertia, actuator stability |
| `pseudo_6dof` | 3-DOF translation plus prescribed/response attitude | Translation plus the declared attitude-response operating point | Physical moment balance or inertia-driven rates |
| `rigid_body_6dof` | Translation plus `I * omega_dot + omega x (I * omega) = sum(M)` | Force and moment equilibrium, mass properties, rates, actuator state | Source fidelity beyond the declared data and envelope |

The repository names the runtime modes `point-mass`, `kinematic-6dof`, and
`rigid-body-6dof`. The catalog and evidence names use
`point_mass_3dof`, `pseudo_6dof`, and `rigid_body_6dof`.

## 2. Prepare The Shared Operating Point

Write down the operating point before writing the residual function:

1. Position and altitude, including geodetic versus geocentric convention.
2. Earth model and rotation rate; use the common Earth transport context when
   the runtime integrates in ECIC.
3. Air-relative speed, Mach, angle of attack, sideslip, wind, and atmosphere.
4. Mass, propellant state, center of gravity, and inertia when consumed by the
   selected tier.
5. Active propulsion, throttle/cutoff state, control defaults, and table
   coordinates.
6. Table domains, interpolation policy, units, body axes, signs, and source
   provenance.

Trim variables should remain local and air-relative. The rotating-Earth
context supplies transport into ECIC; it is not a reason to create one gain or
one hand-authored trim for every latitude.

## 3. Declare The Trim Contract

Create one `TrimSpec` in `verification/trim_specs.yaml` or a vehicle adapter.
The contract must explicitly name:

- ordered `state_names` and `control_names`;
- one `residual_names` entry for each equation being solved;
- finite initial values in the adapter's units and frames;
- physical bounds and positive `residual_scales` where residual magnitudes
  differ substantially; and
- operating-point and source metadata.

Use force/acceleration residuals for a point-mass trim. Use the same
translational residuals for pseudo-6DOF plus only the declared attitude-response
variables. Use moment residuals for rigid-body 6-DOF only when the plant
actually supplies physical inertia, moment coefficients, application points,
and actuator authority.

The generic solver entry points are:

```python
result = solve_trim(spec, evaluator)
procedure_result = solve_trim_procedure(procedure, evaluator, metrics=metrics)
continuation_result = solve_trim_continuation(procedure, evaluator)
```

The evaluator receives `(state, controls)` for `solve_trim`, or
`(state, controls, operating_point)` for a procedure. It must return every
declared residual as a finite numeric mapping. It must not silently change
frames, invent missing moments, or return a force residual under a derivative
name.

## 4. Walk The Fidelity Ladder

Use the same source tables, initial physical state, environment, mass model,
command history, event schedule, duration, and termination policy when testing
reduction parity. Change only the equation tier.

```text
point-mass 3-DOF
    -> kinematic-6DOF bridge with declared attitude response
    -> rigid-body 6-DOF with physical moments, inertia, and actuators
```

The existing ladder and parity commands are:

```bash
python tools/run_fidelity_parity.py
python tools/build_fidelity_ladder_packet.py
python tools/validate_vehicle_onboarding.py --vehicle skywalker_x8
```

A pseudo-6DOF bridge is a diagnostic step. It should expose attitude-dependent
force queries and timing without being reported as moment-complete evidence.
The rigid-body run is a separate experiment when it is intentionally free to
diverge from the prescribed-attitude reduction.

## 5. Vehicle Trim Examples

The source-backed examples use one solver per vehicle and preserve the source
plant adapter in Python:

```bash
python tools/solve_b747_trim.py
python tools/solve_x8_trim.py
python tools/solve_hummingbird_trim.py
python tools/solve_x15_trim.py
```

For the four representative rotating-Earth artifacts, use the single
regeneration command instead of editing solver scripts:

```bash
python tools/regenerate_rotating_earth_trims.py
```

The output reports retain solver status, residuals, gate results, Earth rate,
location, source hashes, and a latitude sensitivity screen. Direct legacy
solver invocation defaults to the explicitly labeled zero-rate source-parity
baseline; the regeneration command explicitly supplies nominal Earth rate.

## 6. Diagnose Failures In Order

Read the structured `diagnostics` list before changing tolerances.

| Code | Meaning | First repair |
| --- | --- | --- |
| `duplicate-variable-name` | A state/control channel is ambiguous | Make ordered state and control names unique |
| `invalid-initial-value` | A guess is missing or non-finite | Supply a finite value in the adapter's unit and frame |
| `unknown-bound-variable` | A bound key is not a declared channel | Correct the key or add the channel to the contract |
| `inverted-state-bounds` / `inverted-control-bounds` | Lower bound exceeds upper bound | Correct the bound order or unit conversion |
| `initial-value-clipped` | The solver silently had to clip a guess | Move the initial guess into the physical envelope |
| `missing-residual` | The adapter omitted a declared equation | Return every residual named by `TrimSpec` |
| `non-numeric-initial-value` / `non-numeric-bound` | A contract value cannot be converted to a scalar | Replace it with a numeric value in the declared units |
| `non-numeric-residual` / `nonfinite-residual` | The adapter returned invalid math | Inspect table lookup, atmosphere, propulsion, and frame conversion |
| `evaluator-failed` | The plant adapter raised during a candidate query | Read the wrapped exception, then check table coverage and required inputs |
| `residual-above-tolerance` | The best candidate is not equilibrium | Check equation balance, bounds, scales, and operating point before loosening tolerance |
| `solution-at-bound` | A variable is using its authority limit | Confirm the limit is physical; otherwise widen the envelope or correct the model |
| `solver-max-evaluations` | The numerical budget ended first | Improve scaling/initialization, then increase `max_nfev` |
| `gate-metric-unavailable` | A declared envelope metric was not published | Add the metric to the adapter or remove the gate from this profile |
| `gate-failed` | The trim is outside a declared operating envelope | Treat it as out of envelope or correct the plant/control authority |
| `missing-state-derivative` | Dynamics linearization lacks a derivative | Return one derivative per state; do not use force/moment residuals as `A/B` dynamics |
| `rigid-frame-mismatch` / `kinematic-frame-mismatch` | A state sidecar uses the wrong reference frame | Convert the translation to the frame required by that equation tier |
| `missing-kinematic-sidecar` / `unexpected-kinematic-sidecar` | The pseudo-6DOF attitude contract does not match the selected mode | Attach a sidecar only to `kinematic-6dof`, or select point-mass/rigid-body explicitly |
| `invalid-inertia` / `runtime-inertia-invalid` | The 6-DOF mass-property contract is absent or invalid | Provide positive finite principal inertia and validate every runtime endpoint |
| `nonfinite-linearization` | Finite differences produced invalid `A/B` | Check perturbation size and table validity around the trim |

The exception string is intentionally usable at a terminal, for example:

```text
[trim:missing-residual] field='residual_names' ... Fix: return every residual named by TrimSpec ...
```

The same information is available as JSON through `TrimResult.as_dict()` and
`TrimProcedureResult.as_dict()`, so notebooks and CI should preserve the
diagnostic records rather than scraping prose.

## 7. Promote Only With Evidence

A successful numerical least-squares status is not sufficient. Before calling
a trim ready, check:

1. residual norm and each unscaled residual;
2. active bounds and table margins;
3. mass/propellant and Earth/location provenance;
4. equation closure and timestep convergence;
5. control-direction and actuator authority probes;
6. reduction parity or the declared fidelity-separation claim; and
7. the exact source/table/catalog hashes used to produce the artifact.

The trim is ready for controller linearization only after the state and
control names, frames, units, and operating point are stable. Use
`finite_difference_dynamics_linearization` only with an evaluator that returns
named state derivatives; force/moment residual Jacobians are useful plant
diagnostics but are not automatically a controller `A/B` model.
