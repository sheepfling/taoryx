# Fidelity-first vehicle integration program

**Status:** active canonical workflow

**Purpose:** integrate, tune, compare, and promote a vehicle from reduced
order dynamics to physical 6-DOF without allowing a higher-fidelity run to
hide a lower-fidelity intake or validation gap.

This plan is the execution companion to [Dynamics fidelity ladder](../architecture/dynamics-fidelity-ladder.md), [Fidelity data requirements](../architecture/fidelity-data-requirements.md), [Trim And Fidelity Walkthrough](trim-and-fidelity-walkthrough.md), the [DAVE-ML round-trip plan](daveml-roundtrip.md), and the [Sensorized Scenario Integration Plan](sensor-scenario-integration.md).

## Promotion model

The four tiers are ordered. A vehicle may run a higher tier as a diagnostic,
but it cannot be promoted at that tier until all earlier tiers are promoted for
the same vehicle identity, source set, and declared mission boundary.

| Order | Tier | What is being proved | Explicit nonclaims |
| --- | --- | --- | --- |
| T0 | `point_mass_3dof` | Translation, loads, resources, environment, terminal behavior | Attitude dynamics, body rates, moments, effectors |
| T1 | `pseudo_6dof_kinematic_bridge` | A named attitude/response law coupled to the translational plant | Physical moment balance and physical actuators |
| T2 | `rigid_body_6dof_direct_wrench` | Newton-Euler translation/rotation and direct or source-induced wrench closure | Physical surface, rotor, gimbal, or motor allocation |
| T3 | `rigid_body_6dof_surface_allocated` | Bounded physical effector allocation and requested/achieved wrench closure | Unmodeled actuator, blade, inflow, plume, or drivetrain physics |

Every tier reports four independent statuses:

1. **Intake:** `blocked`, `partial`, or `ready_for_runtime_probes` from the
   fidelity readiness catalog.
2. **Execution:** whether the declared case ran with finite, in-domain,
   continuous output and an explicit termination reason.
3. **Qualification:** whether the tier-specific objectives and closure gates
   passed. A successful solver exit is not qualification.
4. **Promotion:** `blocked`, `diagnostic_only`, `candidate`, or `promoted`.
   Promotion is the only status inherited by the next tier.

`runtime_proof_status: not_evaluated` must remain visible until execution and
qualification artifacts exist. This prevents a complete-looking manifest from
being mistaken for a qualified model.

## Common run identity

Every tier case must record the same identity fields before comparison:

- vehicle/family ID, tier, parent case, and qualification class;
- source, table, DAVE-ML package, and external-profile hashes;
- units, body/navigation/inertial frames, epoch, Earth model, and Earth rate;
- initial state or trim ID, mass/resource policy, command schedule, event
  schedule, duration, step policy, and termination policy;
- software commit, dependency lock, random seed, and runner command;
- trajectory samples, event records, timeout records, failure diagnostics, and
  the plots generated from those exact records.

The four-tier comparison is valid only when these fields match except for the
declared equations-of-motion and effector realization changes. Otherwise the
result is mission evidence, not reduction parity.

## Ordered work package

### F0 - Intake and contract freeze

Resolve units, frames, sign conventions, geometry, mass/resource policy,
force/moment sources, valid domains, controls, and provenance. Run:

```text
python tools/validate_fidelity_readiness.py --vehicle <id> --through-tier point_mass_3dof
```

Exit: T0 intake is ready, all required source files are reachable, and every
missing item has an actionable disposition. Do not start tuning before this
gate is clear.

### F1 - Point-mass 3DOF baseline

Use the smallest representative mission first. Establish atmosphere/gravity,
propulsion and resource depletion, terminal/objective scoring, event and
timeout semantics, and the standard trajectory artifact. Plot position,
altitude, speed, energy/resources, control commands, table margins, and
termination reason.

Exit: the baseline is reproducible, finite and in-domain, its required
objectives pass, and its artifact is suitable as the parent for T1.

### F2 - Pseudo-6DOF bridge

Reuse the T0 plant and mission identity. Add only the named response policy:
velocity alignment, bank/pitch/yaw commands, low-speed hold, pole-crossing
behavior, rate limits, and attitude provenance. Compare translation against T0
in a declared parity window and plot attitude/command/response residuals.

Exit: translation parity passes for the shared window, attitude behavior is
continuous and deterministic, and synthesized channels are labeled as
synthesized rather than physical truth.

### F3 - Rigid-body direct-wrench 6DOF

Add positive inertia, body-rate and quaternion propagation, source force/moment
decomposition, rotational trim or initial-condition evidence, and independent
finite-difference force/moment closure. Keep the controller generalized: direct
wrench authority is allowed, but it must not be labeled as a surface or rotor
model.

Exit: translational and rotational equation closure, trim/initial-condition
validity, finite/in-domain history, timestep and batch/step parity, and the
declared mission objectives all pass.

### F4 - Physical-effector 6DOF

Replace the generalized wrench with a declared surface, rotor, thruster,
gimbal, wheel, or other allocator. Record requested and achieved wrench,
effector commands, saturation, rate/lag, allocation residual, and resource
coupling. A changing control value alone is not evidence of allocation.

Exit: allocator direction, bounds, rate/lag, achieved wrench, residual,
resource, and mission gates pass in the declared table domain.

## DaveML alignment

DAVE-ML work has two separate promotion axes:

- **Language/package axis:** intake, lossless package retention, semantic IR,
  canonical export, re-import comparison, official/qualified check cases, and
  fresh-process replay.
- **Vehicle fidelity axis:** attach the imported channels to T0, then promote
  through T1, T2, and T3 using the ordered gates above.

A DAVE-ML round trip can be complete while a vehicle's T2 or T3 dynamics claim
remains blocked. Conversely, a reduced surrogate can be executable without
being source-equivalent. Every reduction records its immutable parent,
omitted physics, comparison window, and nonclaims.

The current DaveML examples should therefore be worked in this order:

1. F-16 and HL-20 source intake and T0/T1 operating-point evidence;
2. NESC staged trajectory at T0, then separate deployment lineage;
3. A320 derived-exact T0 and surrogate-composite T1, without implying an
   authoritative Airbus T2/T3 package;
4. source-backed T2/T3 overlays only where moments, controls, and allocator
   data are actually present.

## IMU alignment

Sensorization follows the same tier boundary and never manufactures channels:

| Tier | Initial sensor contract |
| --- | --- |
| T0 | ECI acceleration/specific-force translation only; orientation and gyro are unavailable |
| T1 | Synthesized attitude/body rate from the declared response policy; label it synthesized |
| T2 | Physical quaternion/body-rate truth and full IMU path; MEKF is allowed when channels are valid |
| T3 | Same as T2 plus actuator/resource telemetry and SWIL/HWIL channel substitution where declared |

The translation and rotational authorities remain independently selectable for
HWIL. A three-axis table can supply rotation while simulated ECI translation is
substituted, and the artifact must say so. MEKF attachment is blocked when the
required truth channels are absent; dead reckoning or a reduced navigator may
still be used where its claim boundary is explicit.

## Example progression

| Example | First job | Next promotion blockers to resolve |
| --- | --- | --- |
| Generic/simple aero | Exercise all four gates and tooling quickly | Keep as the deterministic contract fixture |
| Hummingbird | T0/T1 sensor and policy diagnostics | Rotor geometry and complete T2/T3 allocation evidence |
| Skywalker X8 | Fix common aero convention at T0 | Reuse the same T1/T2/T3 runner and source tables |
| X-15 | Repair T0 intake, then release/glide T1 | Declare attitude limits/direct authority before T2; stabilator allocation before T3 |
| B747 | Maintain T0/T1 and direct-wrench diagnostics | Physical allocator declaration before T3 |
| DAVE-ML families | Preserve source/package provenance at the lowest useful tier | Promote only the tiers supported by imported data and controls |

## Completion conditions

The fidelity-first program is complete when:

- the validator can audit a vehicle `--through-tier` in order and stops at the
  first blocked prerequisite;
- at least one deterministic example has promoted evidence at all four tiers;
- Hummingbird and X-8 demonstrate the sensor contracts at T0/T1/T2, including
  MEKF only where its channels are valid;
- X-15 demonstrates the complete lower-to-higher integration workflow with
  explicit release/glide events, timeout/failure subsets, and tier-separated
  plots;
- DaveML language/package completion and vehicle-tier qualification are
  reported independently;
- every tier packet contains trajectory, exploration/capability, failure and
  provenance artifacts, and the full applicable CI gate is reproducible.

Higher-tier diagnostic runs may remain useful, but they do not satisfy these
exit conditions while a lower tier is blocked or only `diagnostic_only`.
