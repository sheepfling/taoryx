# X-15 Reachability Integration Notebook

This notebook records the work required to connect an existing vehicle family
to the Alpha 3 reachability envelope. It is intentionally part of the product:
the integration path, failed assumptions, and evidence boundary are as
important as the final plots.

## Objective

Exercise the reachability process with a vehicle whose capability is bounded by
both delivered energy and post-release glide energy:

```text
booster-powered -> booster-coast -> X-15 release -> unpowered glide
```

The first deliverable is a deterministic reduced-order batch study. It is not
a replacement for the existing source-backed X-15 rigid-body cases.

## Source Anchors

| Concern | Repository anchor | Integration use |
| --- | --- | --- |
| Registry geometry and mass | `verification/vehicle_models.yaml`, vehicle `x15` | Reference area, reference length, mass and inertia provenance |
| Staged mission shape | `examples/showcases/x15_rocket_to_hawaii/mission.prb` | 30,000 kg launch state, 40,000 N booster, 20 s powered segment, coast, release |
| Reduced-order mission | `examples/showcases/x15_rocket_to_hawaii/mission_3dof.prb` | Point-mass powered-ascent and coast precedent |
| Release glide parity | `examples/showcases/x15_rocket_to_hawaii/release_glide_reduction_3dof.prb` | Unpowered reduced-order release state |
| Rigid-body comparison | `examples/showcases/x15_rocket_to_hawaii/release_glide_parity_6dof.prb` | Native 6-DOF release comparison boundary |
| Aero and propulsion tables | `tests/fixtures/x15_coherent_6dof_public_research_v1/tables` | Source evidence and future provider inputs |

## Integration Log

### 1. Initial interface mismatch

The first reachability adapter used a `RocketGlideVehicle` with a 40 s burn,
but initialized it at 21.5 km and 1,283 m/s. That is an already-released X-15
state, so it did not exercise the booster or prove staged integration. The
result was useful as a plotting smoke test but was not a valid X-15 integration
example.

**Resolution:** add an explicit optional booster contract to the reduced-order
vehicle: powered burn, coast, release mass drop, and glide phases.

The first staged smoke run then exposed a second mismatch: retaining the
reduced mission's 500 m/s initial speed with the staged mission's 40 kN
booster caused every command to contact the ground before release. The native
staged case starts with an approximately 1,555.6 m/s inertial velocity.

**Resolution:** use that source initial-speed magnitude for the reachability
surrogate and retain the low-speed result as a negative integration test case.

### 2. Staged mass closure

The source mission declares a 30,000 kg launch mass, 9,000 kg propellant, and a
100 kg/s flow for a 20 s powered segment. That consumes 2,000 kg, while the
release reset is 14,641.0545 kg. Those values do not form a complete directly
usable mass-flow model without deciding what residual booster mass is discarded
at release.

**Resolution:** close the reduced-order study at the source release mass,
model 2,000 kg as consumed booster propellant, and model the remaining
13,358.9455 kg as discarded booster mass. Record the source discrepancy and
the closure assumption in provenance rather than silently changing the source
numbers.

The original workflow could still have regressed because those decisions were
only recorded as prose and literals in the adapter. The X-15 run now performs
an integration preflight before generating an envelope. It reads the staged
source mission anchors, checks launch mass, initial speed, thrust, consumed
propellant, burnout time, release time, and release mass against the surrogate,
and probes the burnout/release transitions with several integration steps.
The source's 9,000 kg declared propellant versus 2,000 kg consumed remains an
explicit 7,000 kg discrepancy in the report; it is not silently treated as a
valid mass-flow model.

### 3. Native provider boundary

The repository has native X-15 rigid-body cases and aerodynamic tables, but no
batch provider that can yet accept the reachability command grid and return the
common envelope result contract.

**Resolution:** keep the current tiers explicitly named as an X-15-scaled
surrogate. The native cases remain comparison evidence, not an unearned native
batch envelope claim.

### 4. Energy-limited evidence

Terminal speed alone does not explain why a candidate failed. A boost/glide
study must retain phase durations, burnout/release mass, and specific-energy
history so that the plots can distinguish poor pointing from insufficient
delivered energy or an unrecoverable glide state.

**Resolution:** retain `boost`, `coast`, and `glide` phase rows and add initial,
maximum, and terminal specific-energy path metrics to every candidate.

## Current Contract

The staged X-15 surrogate currently provides:

- one deterministic command grid shared by both reduced-order fidelities;
- point-mass 3-DOF and pseudo-6-DOF trajectory tiers;
- explicit booster burn and release events;
- complete successful and unsuccessful candidate records;
- source, assumption, and claim-boundary provenance;
- Matplotlib trajectory, coverage, capability, and cross-fidelity plots.

The preflight is also a deliberate onboarding pattern for future vehicles:
source anchors are compared before the batch run, discontinuous events are
tested independently of the production search grid, and the known source
closure decision is emitted as data rather than hidden in a constructor.

The shared vehicle model in `src/taoryx/vehicle.py` now expresses that same
boundary directly through `StageMassDefinition`, `StagedVehicleDefinition`,
and `StageSeparationEvent`; the reachability names are compatibility aliases.
A vehicle is
constructed from a retained core stage, attached stages, and explicit ejection
events. Modeled propellant flow must close over burn time, while an optional
source-declared propellant value is retained for discrepancy reporting.
Propulsion capabilities also distinguish liquid, solid, hybrid, and unknown
systems and reject throttle or cutoff commands that the declared hardware
cannot realize. The reduced solver currently accepts one attached stage, but
the configuration boundary no longer requires callers to coordinate unrelated
booster mass, propellant, thrust, and release-time fields by hand.

Separation events can additionally declare a passive, spring, pneumatic,
pyrotechnic, or explosive mechanism, a body-frame impulse applied to the
retained stack, and optional separation energy. The impulse produces a
retained-stack delta-v when a retained mass is supplied; energy is retained as
evidence until an explicit impulse-partition model exists. This prevents a
source document's separation energy from being silently treated as vehicle
translation.

The default 45-command demonstration uses a 720--950 m/s terminal-speed
window. At the current reduced-order settings it produces 27 feasible
point-mass candidates and 33 feasible pseudo-6-DOF candidates. This is a
demonstration criterion chosen to expose tier behavior, not a source-validated
X-15 terminal requirement.

## Selective Native Replay

After generating the pseudo-6-DOF artifact, selected checkpoints can be sent
through the existing native rigid-body runner:

```bash
taoryx reachability x15-native-replay \
  artifacts/x15-reachability/pseudo_6dof.json \
  --output-dir artifacts/x15-reachability/native-replay
```

The replay adapter currently selects representative feasible and infeasible
samples, searches for a glide checkpoint below a 20 km safety threshold, and
executes a short native replay using the source X-15 static aerodynamic table.
The native table extends to approximately 24.4 km, but the safety threshold
leaves room for the native integration step.

Two additional limitations are recorded in each replay record:

- The reduced local tangent state is embedded into ECIC using a declared bridge;
  it is not a geodetic state reconstruction.
- The native replay uses the source-aligned release attitude and tangential
  velocity policy to stay inside the narrow source aero table domain. The
  original reduced velocity and attitude remain recorded for comparison.

This makes the result useful as integration evidence without presenting it as a
native X-15 batch envelope or an exact same-state 6-DOF validation.

## Timeout Follow-Up

Horizon termination is retained separately from feasibility. A longer follow-
up study can focus only on those candidates:

```bash
taoryx reachability rerun-timeouts \
  artifacts/x15-reachability/pseudo_6dof.json \
  --horizon-s 600 \
  --output artifacts/x15-reachability/pseudo_6dof-timeout-rerun.json
```

The rerun carries the parent study ID and timeout-subset provenance. This keeps
the original bounded-time result intact while allowing unresolved long-flight
cases to be investigated without paying to repeat the completed candidates.

## Remaining Boundary

The next integration step is a native-provider adapter that consumes the X-15
source tables and can replay selected reachability boundary points at rigid-body
6-DOF fidelity. Until that exists, the reduced-order X-15 artifacts demonstrate
the onboarding process and envelope mechanics, not X-15 performance validation.
