# Adding a vehicle to the parity harness

The parity harness is intentionally data-driven. A new vehicle family should
not require a new test runner or a new comparison implementation.

## Required inputs

1. Add the source-backed vehicle model and table bindings to the vehicle
   catalog.
2. Add one point-mass problem and one rigid-body problem using the standard
   problem-file generation workflow.
3. Add a bridge source problem; the harness derives the kinematic bridge from
   it, so the source translational inputs are shared.
4. Add one entry to `verification/fidelity_parity.yaml`.

The entry must identify:

- vehicle family and stable scenario ID;
- all three problem paths;
- source table paths;
- canonical SI initial state;
- atmosphere, wind, Earth model, and rotation;
- propulsion and mass-flow model;
- command and event schedules;
- duration and termination policy;
- point and rigid output profiles;
- reduction kind: `point-mass-3dof` or `kinematic-3-plus-3-dof`;
- explicit parity tolerances for each canonical state channel;
- the `state_mapping` for canonical altitude, speed, and mass.

Example mapping:

```yaml
state_mapping:
  altitude_m: {point: alt, rigid: altitude_m, point_scale: 0.3048, rigid_scale: 1.0}
  speed_m_s: {point: vel, rigid: speed_m_s, point_scale: 0.3048, rigid_scale: 1.0}
  mass_kg: {point: mass, rigid: mass_kg, point_scale: 0.45359237, rigid_scale: 1.0}
```

The mapping and reduction kind are the only lower-tier details used by the
parity runner. A point-mass reduction supplies translation with prescribed
flight conditions. A pseudo-6DOF reduction supplies translation plus a
prescribed or lagged attitude sidecar. Both are valid development inputs; the
rigid-body tier must not be promoted merely because one reduction executes.
If a new runtime publishes different names or native units, change this
metadata rather than adding a branch to `run_fidelity_parity.py`.

Tolerances belong in the vehicle contract because a high-speed release case
and a hover case do not have identical conditioning. The report records the
selected tolerances beside the measured differences; no global relaxation is
permitted.

## Workflow

```bash
python tools/dev.py check-parity
python tools/dev.py run-parity
python -m pytest tests/unit/test_fidelity_parity_contract.py -q
```

The report first checks execution, then transcodes both tiers to SI, compares
the initial state and paired translational history, audits continuity, and
performs a half-step convergence run. A family remains `blocked` or
`candidate` until all required gates pass.

## Family-specific adapters

Vehicle-specific physics belong in the standard problem-file and table
contracts. The harness does not infer fixed-wing versus rotorcraft semantics.
For example:

- a fixed-wing reduction must use the same aerodynamic and propulsion plant;
- a rotorcraft reduction must use a translational thrust-vector model;
- a rocket-plane reduction must share a release state and propulsion phase.

If those semantics cannot be made equivalent, record a blocker in the catalog
and run the case as fidelity-separation evidence instead of manufacturing a
parity result.
