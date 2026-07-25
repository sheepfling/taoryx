# TAORYX Alpha 2 public schema reference

This page is the human-readable boundary for the Alpha 2 configuration and
control contracts. The machine-readable source of truth is
`verification/alpha2_family_catalog.yaml`; the release packet contains a
catalog-derived JSON snapshot and PDF generated from that file.

## Frozen selection model

```text
family + fidelity + variant + loadout + mission + segment plan
       + controller + explicit overrides
       = immutable ResolvedCase
```

Resolution performs unit conversion, range/type checks, preset precedence,
compatibility checks, and per-value provenance. A native `.prb` file is a
provider output, not the configuration authority for new family combinations.

## Contract layers

| Layer | Alpha 2 contract | Release behavior |
| --- | --- | --- |
| Family | `FamilyPackage` | Versioned family identity, fidelity set, parameters, controls, observations, capabilities, component slots, resources, allocations, transitions, presets, and segment graphs. |
| Case input | `CaseIntent` | Human-authored family/fidelity/preset selection plus typed overrides and extensions. |
| Resolved case | `ResolvedCase` | Frozen canonical values, fixed schemas, segment graph, provenance, and identity SHA-256. |
| Provider | `CompiledCase` | Explicit translation from the neutral case into a provider/runtime binding. |
| Session | `reset`, `step`, `run_to_completion` | Batch execution is repeated canonical stepping; applied controls and diagnostics are retained. |
| Control | `ControlSchema` and `ControlFrame` | Autopilot, commanded, overlay, direct, and mixed authority are explicit and bounded. |
| Observation | `ObservationSchema` | Stable semantic IDs, units, frames, availability, evidence grade, sources, and descriptions are fixed after resolution. |
| Capability | `CapabilitySchema` | Supported fidelities, control intents, modes, terminal conditions, resources, allocation, transitions, and checkpoint support. |
| Components | `ComponentSlot`, `AllocationSchema`, `ModeTransitionSchema` | Typed composition points and hybrid-mode handoffs remain provider-neutral. |

## Fidelity profiles

- `point_mass_3dof`: translational position/velocity and point-mass forces.
- `pseudo_6dof`: translational motion plus an explicit kinematic attitude or
  response bridge; it is not rigid-body evidence.
- `rigid_body_6dof`: quaternion attitude, body rates, inertia, forces, moments,
  actuators, and rigid-body closure where the family implements them.

Fidelity is independent from control authority. A high-level commanded or
autopilot interface may be used at any advertised fidelity; direct effectors
are only valid when the family declares them.

## Evaluation envelope

Provider results may carry a `TrajectoryEvaluation`. It is the neutral
evidence boundary for a run, not another physics implementation. The envelope
keeps four claims separate:

| Field | Meaning |
| --- | --- |
| `validity` | Whether the resolved model and run satisfy internal integrity rules. |
| `qualification` | Whether the run remains inside the source-supported or explicitly extended range. |
| `feasibility` | Whether the requested mission is expected to be achievable before execution. |
| `outcome` | What actually happened: completion, degradation, resource or envelope limitation, abort, or numerical failure. |

Objective arithmetic remains owned by `taoryx.objectives.score_objectives`.
`objective_report_to_evaluation` promotes that result into the typed envelope
without recomputing it. Requested controls, achieved actuator states,
resources, events, closure, and convergence are separate channels. A required
closure or convergence metric therefore cannot be hidden by a passing weighted
objective score.

Every numeric evidence channel declares a unit; unavailable channels are
represented explicitly rather than as zero. This is the contract used by
family-specific plots and evidence packets.

## Claim boundary

Alpha 2 proves configuration, provider lifecycle, control authority, native
problem generation, fidelity composition, dual-launch handoff, evidence
packaging, and reproducibility for the checked-in synthetic proof families.
It does not claim historical TAOS 96.0 runtime compatibility, global vehicle
validity, flight qualification, or a universal autopilot.

## Reproduce

```text
python tools/run_alpha2_tranches.py
python tools/build_alpha2_release.py
python tools/audit_alpha2_release.py
```

The release packet freezes the catalog hash, source hashes, generated artifact
hashes, schema snapshot, claim matrix, and clean-source replay report.
