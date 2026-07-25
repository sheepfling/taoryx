# Taoryx model-integration workflow

**Purpose:** Give contributors a repeatable, evidence-preserving process for
adding a new aircraft, spacecraft, rotorcraft, rocket, surrogate, or other
source-grounded model to the Taoryx library.

This workflow is source-agnostic: DAVE-ML is the current F-16/HL-20 path, but
the same process applies to OpenAP, NASA/NESC, JSBSim, CCSDS orbit products,
analytical models, and explicitly labeled synthetic or public-data surrogates.

## The integration boundary

Never edit an accepted source plant in place to add convenient controls,
actuators, controllers, missions, sensors, or RL behavior.

```text
source acquisition and lock -> source evaluator -> immutable plant
    -> canonical units/frames/force-moment adapter -> mass/configuration binding
    -> actuators/effectors -> allocation/controllers/guidance
    -> sensors/segments/objectives -> reductions and provider/Lab facades
```

Each layer gets its own version, provenance, evidence class, tests, and claim
boundary. A Taoryx-designed overlay must not inherit the source plant’s
evidence strength merely because it is packaged beside the plant.

## Integration record: create this first

Every new model starts with a versioned record such as
`families/<family-id>/qualification/integration-record.yaml`:

```yaml
schema: taoryx.model-integration/v1alpha1
family: reference_f16_s119
source:
  kind: daveml
  uri: <immutable-source-or-archive>
  revision: <commit-or-release>
  sha256: <source-bytes-hash>
  license_and_notices: <path-or-review-id>
plant:
  package: <immutable-package>
  package_sha256: <package-hash>
  fidelity: rigid_body_6dof
  body_frame: FRD
  navigation_frame: NED
  validity_envelope: <path>
  qualification: source_checked
layers:
  mass_properties: {version: v1, evidence: source_bound_fixed}
  actuators: {version: v1, evidence: taoryx_engineering_assumption}
  controller: {version: v1, evidence: taoryx_reference_design}
  sensors: {version: v1, evidence: synthetic_testbed_profile}
  mission: {version: v1, evidence: taoryx_qualification_scenario}
claims: []
nonclaims: []
open_questions: []
```

Unknown values remain unknown, ranges, or distributions. They are never filled
with zeros or precise-looking guesses.

## The ten integration stages

1. **Register the source.** Record source type, repository/owner, revision,
   exact path, license/notices, retrieval date, source hash, and separate
   mass/inertia or geometry sources. For an archive, record archive and member
   hashes. Do not rename files inside a pinned baseline revision.
   For DAVE-ML reference inputs, run
   `tools/verify_daveml_reference_inputs.py` against the external input root
   before claiming that the package is available. A recorded digest without a
   present, matching file remains an intake blocker.
2. **Inspect before compiling.** Inventory variables, equations, tables,
   dimensions, frames, units, controls, check cases, limits, clamps, and
   unavailable fields. Classify exact, reconstructed, digitized, analytical,
   synthetic, or mixed data.
3. **Build the source evaluator.** Verify source check cases and static vectors
   before connecting the common runtime. Raw nonlinear tables remain
   authoritative; resampled grids and derivatives carry their own validity.
4. **Pass package and interface gates.** Run integrity, canonical loading,
   source/oracle vectors, trim/steady-state, and dynamic/event regression in
   that order. Do not add controllers or flagship missions while a lower tier
   is failing.
5. **Define the canonical adapter.** Convert units and frames only at an
   explicit boundary. Preserve separated aero, propulsion, contact, and other
   force/moment channels, resource flow, and validity/clamp/extrapolation
   status. Unknown quantities remain unavailable.
6. **Bind mass and lifecycle.** Add mass, CG, inertia, payloads, consumables,
   staging, deployment, and transforms as separately sourced records. Validate
   positive mass, positive-definite inertia, identifiable frames, and complete
   post-event states.
7. **Add actuators and controls as overlays.** Document identity, frame, sign,
   neutral, limits, rates, latency, dynamics, health/failure behavior, and
   effectiveness. Test direct, commanded, residual/overlay, mixed, and failsafe
   paths separately.
8. **Add reductions from the qualified parent.** Generate 3DOF and pseudo-6DOF
   from trim/force/response sweeps over the immutable plant. Record retained
   physics, mappings, envelope, and expected disagreement.
9. **Add missions last.** Define typed starts, segment contracts, objectives,
   waypoints, terminal finality, evaluator profiles, and replay commands. A
   hero mission sits above plant, trim, closure, convergence, actuator,
   controller, and envelope evidence; it cannot replace them.
10. **Publish the handoff packet.** Include source/package hashes, schemas,
    provenance, all regression reports, configurations, telemetry, controls,
    actuators, events, envelope/closure/convergence/evaluation reports,
    terminal state, reproduction command, limitations, and open questions.

## Qualification tiers

| Tier | Gate | Evidence |
| --- | --- | --- |
| 0 | Distribution integrity | Hash ledger, archive CRCs, expected paths, IDs, schemas, notices. |
| 1 | Canonical interface | Package loads and exposes fidelity, geometry, frames, envelope, and SI/FRD results. |
| 2 | Source/oracle vectors | Pointwise coefficients, forces, moments, propulsion, and units agree. |
| 3 | Trim/equilibrium | Source-grounded trim, hover, or steady state reproduces with residuals. |
| 4 | Dynamic regression | Hold, maneuver, event, or trajectory checks replay deterministically. |

The family’s maturity is bounded by its lowest applicable passing layer.

## Evidence classes

Record evidence per subsystem, not only per family:

```text
source_checked
source_checked_steady
source_bound_fixed
analytical
reference_correlated
test_correlated
taoryx_engineering_assumption
taoryx_reference_design
synthetic_testbed_profile
taoryx_qualification_scenario
unknown
```

## F-16 and HL-20 application

The active intake notebook is
[DAVE-ML reference intake notebook](daveml-reference-intake-notebook.md). It
records the current external-input boundary, the exact verification command,
and the friction ledger for new contributors.

| Family | Immutable plant role | Immediate overlay work |
| --- | --- | --- |
| F-16 S-119 | Nonlinear powered conventional fixed-wing 6DOF; subsonic, steady thrust, fixed mass/inertia | Actuators, SAS, rate/attitude/route control, arrival mission, reductions. |
| HL-20 Mod K | Byte-pinned nonlinear unpowered lifting-body 6DOF; fixed mass/inertia, seven direct surfaces | Actuators, allocation, energy-management guidance, release/arrival mission, reductions. |

The F-16 semantic reconstruction and the HL-20 exact pinned DAVE-ML source
retain different evidence classifications. Neither gets runway or touchdown
claims until low-speed and contact dynamics are independently qualified.

## Baseline archive note

The reviewed `taoryx-testing-baseline-v1.0.zip` is an external baseline archive
with SHA-256:

```text
69200c30ef7f45b7fdf56a14f7b6bd25bf545ecda9ff79451c92b7b1c37ea721
```

Its F-16 and HL-20 packages are reference inputs, not silently absorbed into
the historical manual or treated as Taoryx runtime compatibility. Future
integration records pin the exact package and source hashes they consume.

## Contributor checklist

Before opening a model-integration PR, answer:

- What exact source bytes and license/notices are being used?
- Which fields are source facts, derived values, assumptions, or unknown?
- What evaluator and oracle/check-case evidence exists?
- What are the units, frames, signs, interpolation, clamp, and extrapolation rules?
- Which forces, moments, mass properties, resources, and events are modeled?
- Which overlays are Taoryx engineering assumptions?
- What is the lowest passing qualification tier?
- What are the valid fidelities, envelope, starts, terminal states, and nonclaims?
- Can a new contributor reproduce the packet from one documented command?

If any answer is unavailable, leave the integration at the appropriate maturity
level and record the blocker rather than promoting the model by association.

## Reachability-analysis handoff

Reachability is an application over an integrated provider, not a new vehicle
physics layer. A model becomes eligible when its provider can resolve the case,
initialize and restore deterministic checkpoints, step controls, report path
validity, and return replayable telemetry. The workbench owns adaptive
sampling, candidate search, boundary extraction, uncertainty classification,
and witness storage. It must preserve `verified_feasible`,
`unresolved_search`, `invalid_query`, `invalid_model_region`,
`numerical_failure`, and `certified_infeasible` as distinct outcomes.

Sensorized model integrations must also follow the [sensor and measurement
orchestration backlog](sensor-measurement-orchestration.md). Source truth,
ideal projection, sensor/electronics error, estimator state, and controller
command are separate evidence-bearing layers.

For spacecraft, also apply the [spacecraft 6DOF data and qualification
track](spacecraft-6dof-data-and-examples.md). Register wheel assemblies and
thrusters as physical actuator families with their own geometry, limits,
resources, failure states, allocation, and analytical tests.

Parametric spacecraft additionally follow the
[spacecraft family expansion](spacecraft-parametric-family-expansion.md):
sample requirements and geometry, derive coherent wheel/thruster sizing, then
validate the actual allocation and resource envelopes. Do not independently
sample inertia, desired slew time, wheel torque, momentum, thrust, and impulse
bit as unrelated fields.

The Anduril-inspired aircraft intake uses the same process. Start with the
[surrogate integration notebook](anduril-surrogate-integration-notebook.md) and
the versioned parameter pack before adding an executable runner. Record rejected
assumptions such as architecture misclassification, simultaneous brochure
maxima, hard speed clamps, and discontinuous hover-to-cruise transitions in the
notebook’s friction ledger.

## Parametric and ML-corpus extension

When the integration target is a parametric archetype or corpus generator,
extend the record with:

- a compact latent design vector and derivation graph;
- coupled geometry, mass, inertia, aero, propulsion, actuator, and envelope
  rules;
- separate deterministic seeds for vehicle design, mass properties,
  aerodynamics, propulsion, actuators, controller, initial conditions,
  environment, observations, and failures;
- rejection reasons for incoherent or envelope-invalid samples;
- static latent truth, aerodynamic-function truth, per-step truth, and derived
  aggregate truth;
- fidelity-matched identification targets and required excitation fragments;
- realized-vehicle corpus split assignment before window generation; and
- coverage, nearest-training-vehicle distance, and rejection statistics.

Never split trajectory windows from the same realized vehicle across train and
test unless that is the explicitly named maneuver-generalization benchmark.
Three-DOF targets must remain force-level; pseudo-6DOF targets may include
response-law parameters; rigid-body targets may include physical rotational
properties only when the experiment excites and observes them.

####
