# Vehicle-integration automation plan

Status: Alpha 3 active long-running workstream

## Purpose

New vehicle integration currently has reusable runtime machinery, but still
requires too much family-specific discovery and hand-authored configuration.
This plan turns the observed F-16 onboarding pain points into a repeatable
intake, diagnosis, fidelity, controller, and showcase workflow.

The target is not a zero-configuration vehicle compiler. Physical models still
require family knowledge and source interpretation. The target is that Taoryx
asks for the missing declarations, generates the worklist, runs all generic
checks, and refuses unsupported promotion without manual bookkeeping.

The F-16 and HL-20 work also exposed a planning-metadata discipline: a
family's public `next_gate` must identify that family, not a copied neighbor.
The source-integration pipeline publishes this value directly to authors, so a
named-family mismatch is treated as a manifest defect and regression-tested.
Stage-local integration-record gates may remain more granular; they are not
silently substituted for the public family next gate.

## Current baseline

The F-16 integration demonstrates the complete vertical slice at one local
operating condition: source replay, trim, runtime linearization, local LQR,
bounded surface allocation, nonlinear route evidence, reductions, and a
self-contained packet. It remains local development evidence rather than
family or release qualification.

The principal manual seams are:

- source state/control/unit/frame mapping;
- family-specific trim unknowns and residual definitions;
- effector topology, signs, limits, and effectiveness provenance;
- controller scaling, gains, and schedule transitions;
- mission geometry and achievable timing;
- resource and depletion laws;
- evidence interpretation and maturity promotion;
- regeneration of dependent manifests and evidence artifacts.

## Target workflow

```text
vehicle intake
    → readiness matrix
    → generated family scaffold
    → source/convention diagnostics
    → trim recipe and operating-point catalog
    → plant/effector identification
    → controller and fidelity tests
    → mission preflight and time estimate
    → independent qualification packet
    → explicit promotion or blocker report
```

Every stage produces machine-readable evidence and a human-readable summary.
Unknown data remain unknown; the tool never fills missing physics with zeros or
unlabeled plausible constants.

## Milestone A — Vehicle intake and data readiness

Add a provider-neutral intake and source-readiness surface:

```text
taoryx vehicle intake existing-family ...
taoryx vehicle intake new-topology ...
taoryx vehicle integration readiness <family-id>
```

The author-facing implementation is now available as:

```text
taoryx vehicle integration readiness reference_f16_s119
taoryx vehicle integration pipeline reference_f16_s119
taoryx vehicle integration pipeline reference_hl20_mod_k --allow-blocked
```

It currently consumes the supported source-family registry and family
manifests. The F-16 report is `ready_for_runtime_probes`; the HL-20 report
exposes its semantic glide mission and the separately planned runtime gate.
Neither result promotes a runtime tier or silently lowers fidelity. The
lower-level validation scripts remain CI/developer entry points, while the
Product 3 CLI is the author-facing projection of the same evidence.

The intake record should generate or validate:

- identity, provenance, source hashes, and evidence class;
- state, control, effector, resource, and observation schemas;
- units, frames, quaternion convention, and sign conventions;
- geometry, mass, CG, inertia, atmosphere, and envelope declarations;
- required data by each of the four fidelity tiers;
- missing, assumed, derived, and source-backed values;
- the first diagnostic and qualification worklist.

The next transitional command is now implemented as a staged composition
report:

```bash
.venv/bin/python tools/validate_vehicle_integration_pipeline.py \
  --family reference_f16_s119
.venv/bin/python tools/validate_vehicle_integration_pipeline.py \
  --family reference_hl20_mod_k --allow-blocked
.venv/bin/python tools/dev.py integration-pipeline
.venv/bin/python tools/validate_vehicle_integration_pipeline.py \
  --family all --allow-blocked \
  --json verification/vehicle_integration_pipeline.json \
  --packet-dir /tmp/taoryx-vehicle-integration-packets
```

It consumes the readiness report, typed reference-family manifest, DAVE-ML
import record, operational contract, and integration record.  Its fixed stage
order is:

```text
intake → conventions → plant → effectivity → trim → operating_points
       → fidelity → controller → mission
```

The report distinguishes `passed`, `development`, `planned`,
`not_applicable`, and `blocked`.  A source replay can therefore pass while a
surface allocator or mission overlay remains development evidence.  For the
current pilots, F-16 reaches `T2_source_plant_ready` with development gates;
HL-20 reaches the same plant tier with a translation-ready semantic mission
but an explicitly planned runtime. That distinction is intentional: the
pipeline does not invent a reduction, native mission plant, or silently
inherit a missing profile.

The packet writer copies the family manifest, import record, integration
record, operational contract, declared bindings, referenced evidence, and the
pinned source package when it is locally available.  `manifest.json` records
the SHA-256 of every copied input; common external corpora remain hash
references unless explicitly included by a future release policy.

The next staged command is the first implemented trim-orchestration slice:

```bash
.venv/bin/python tools/validate_vehicle_trim_orchestration.py \
  --family all \
  --json verification/vehicle_trim_orchestration.json
.venv/bin/python tools/dev.py trim-orchestration
```

This validates `taoryx.trim-recipe/v1alpha1`, maps catalog values into typed
`TrimSpec` work items, and records the adapter, residuals, bounds, fixed-input
policy, continuation strategy, and source evidence.  The generic layer still
does not invent a plant evaluator, but the two conformance adapters now bind
and execute those worklists.  The generated solve artifact is:

```bash
.venv/bin/python tools/solve_vehicle_trim_evidence.py
```

The F-16 currently solves all seven points and the HL-20 solves its one
source-bounded pitch point.  The F-16 catalog still warns where six points do
not yet carry independently materialized residual values in the catalog
itself; the adapter solve report is the stronger current evidence.

Controller and mission preflight is now also available:

```bash
.venv/bin/python tools/validate_vehicle_controller_mission_preflight.py \
  --family all --allow-blocked \
  --json verification/vehicle_controller_mission_preflight.json
.venv/bin/python tools/dev.py controller-mission-preflight
```

This checks controller profile identity, linearization/effectiveness
references, canonical control bindings, unresolved direct-wrench paths, mission
realization IDs, phase order, route geometry, and a capability-based duration
estimate where that family owns one. It also accepts a generic
`semantic_composition` mission binding: checked-in witnesses must compile to
the declared public composition family/mission/fidelity and reproduce their
expected semantic-preflight status. It does not solve gains or execute a
plant. The F-16 currently reports development preflight with its racetrack
estimate; the HL-20 reports two translation-ready reduced intent witnesses but
remains development because no high-altitude runtime, truth objectives, or
terminal evaluator is bound. Its absent controller profiles remain explicitly
not applicable.

Numeric effectivity and overlay preflight is now a separate reusable stage:

```bash
.venv/bin/python tools/validate_vehicle_effectivity_preflight.py \
  --family all --allow-blocked \
  --json verification/vehicle_effectivity_preflight.json
.venv/bin/python tools/dev.py effectivity-preflight
```

The effectivity check understands both a declared numeric local matrix and a
per-sample matrix emitted by a physical-wrench run. It reports matrix shape,
rank, singular values, condition number, source location, and a bounded
reference-demand replay when compatible actuator limits are declared. It keeps
that result at development-screen status until actuator dynamics and nonlinear
response evidence exist. It also
recognizes a family-specific downstream overlay and reports its logical to
physical mapping without treating it as source-declared authority. The F-16
therefore has a numeric 4-by-4 rank-4 development screen with a feasible
reference replay and condition number about 20.18; the HL-20 has a seven-output
synthetic allocation overlay with no numeric effectivity claim.

Exit criteria:

- a new family produces a deterministic scaffold and readiness report;
- every missing required input has a named blocker;
- source, derived, estimated, and unavailable values are distinguishable;
- schema and convention errors are detected before runtime integration.

## Milestone B — Generic trim recipe and operating-point generation

Define a family-supplied trim recipe rather than a family-supplied bespoke
solver. The recipe declares unknowns, fixed variables, residual groups,
constraints, seed policy, and validity limits. The generic orchestration layer
validates the recipe and performs bounded catalog-to-`TrimSpec` worklist
generation. The family adapter binds each work item to the existing generic
trim solver and serializes residual/convergence evidence.

Required outputs:

- translational, rotational, and kinematic residuals;
- solved state and physical effectors;
- solver settings and convergence history;
- table-domain and resource checks;
- neighboring-point continuity;
- qualified, extended, or failed validity classification.

Exit criteria:

- F-16 and HL-20 use the same declarative recipe/worklist layer with different
  state/control/residual contracts;
- F-16 and B747 use the same trim engine with different family recipes;
- X8 and Hummingbird can register family-specific residual maps without
  duplicating the solver;
- arbitrary requested points fail with structured diagnostics when unsupported;
- midpoint points are explicitly retrimmed or explicitly marked interpolated.

## Milestone C — Plant and effector contract generation

Before actuator information can participate in trim, allocation, or a runtime
plant, intake must classify its destination.  Scalar values that the native
grammar explicitly supports may be lowered to a native runtime directive.
Nested values such as actuator dynamics, allocation matrices, provenance, and
sensor settings remain structured manifest data until a named adapter consumes
them.  The generator must not stringify nested YAML/Python values into a
native scalar attribute: every generated problem is successor-parsed as an
onboarding regression check.  This preserves the source/model record without
mistaking metadata serialization for an implemented actuator.

Automate the distinction between direct-wrench and physical-effector paths.
The intake should require an explicit control path and generate tests for:

- source-load evaluation;
- state and effector perturbation derivatives;
- effectiveness rank and conditioning;
- bounded weighted allocation;
- actuator position, rate, lag, health, and saturation;
- requested versus achieved wrench;
- no-direct-injection enforcement for physical-effector runs.

Exit criteria:

- a physical-effector tier cannot run without actual effector state reaching
  the nonlinear plant;
- direct-wrench results are automatically labeled as the integrated bridge
  tier, separate from physical-effector allocation;
- effectivity and actuator assumptions appear in the evidence manifest;
- infeasible demands become structured results, never silent clipping.

The first automation slice is now present as effectivity preflight. It is
diagnostic rather than promotional: it does not synthesize a matrix, infer
actuator dynamics, or turn direct-wrench stability into physical control
evidence. The bounded replay is a numerical consistency check only. The
remaining work is to add generic signed perturbation probes against a live
plant adapter, actuator-limit fixtures, and a no-direct-injection runtime
assertion behind the same provider-neutral adapter.

## Milestone D — Automatic controller and schedule workbench

Generalize the existing scaled-LQR utilities into a controller workbench that
can evaluate, compare, and tune local controllers without embedding vehicle
names in the tuning code.

The workbench should support:

- declared state/reference scaling;
- deterministic weight sweeps and bounded search;
- pole, damping, control-demand, saturation, and nonlinear-response gates;
- neighboring operating-point synthesis;
- schedule interpolation and transition tests;
- automatic selection of the highest earned controller tier.

The first implementation should tune local trim-hold and isolated-axis
responses. Mission tuning comes only after local authority passes.

Exit criteria:

- F-16, X8, B747, and Hummingbird run through one tuning interface;
- all selected gains retain provenance and operating-point fingerprints;
- a schedule is not eligible until its nodes and transitions pass;
- failed tuning candidates produce diagnostics rather than merely a score.

## Milestone E — Mission scaling and preflight estimation

Make shared missions reusable without copying vehicle-specific constants.
The preflight estimator should use the selected vehicle/reduction to estimate:

- straight-leg duration;
- turn radius from speed, bank, and authority limits;
- climb/descent time and distance;
- acceleration and braking distance;
- resource consumption when modeled;
- terminal capture horizon;
- whether the requested route is geometrically and energetically plausible.

The result must be an estimate, not a guarantee. Runtime truth evaluation remains
authoritative.

Exit criteria:

- the powered-fixed-wing racetrack is parameterized by vehicle capability;
- X8, B747, F-16, and later aircraft use the same mission template;
- infeasible route geometry is identified before a long run;
- estimated and realized event times are reported together.

## Milestone F — One-command evidence and promotion

Create a family pipeline that runs the applicable checks in dependency order:

```text
taoryx vehicle qualify <family-id> --profile development
```

It should build:

- readiness and provenance reports;
- trim and operating-point evidence;
- derivative and authority evidence;
- controller and reduction evidence;
- nominal mission packet;
- fixed perturbation results;
- fidelity-selection eligibility;
- promotion decision with exact nonclaims.

Generated artifacts must carry dependency hashes so stale evidence is detected
and regenerated rather than silently reused.

Exit criteria:

- one command rebuilds a self-contained family packet;
- every displayed claim traces to current evidence;
- promotion is monotonic across T0–T6 and R0–R4;
- failed or incomplete layers identify the first blocking stage;
- no family receives a qualification badge from a controller transition or
  scalar mission score alone.

## Implementation order

1. Consolidate the stale F-16 onboarding/status records into one maturity view.
2. Build the readiness schema and generated intake report.
3. Extract the generic trim engine around the F-16 and B747 recipes.
4. Add generated plant/effector and resource checklists.
5. Generalize the tuning workbench across X8, B747, F-16, and Hummingbird.
6. Add capability-based racetrack scaling and preflight estimates.
7. Connect all outputs to the existing showcase packet and promotion tools.
8. Use HL-20 as the next onboarding test, because it exercises unpowered
   energy management and surface-allocation differences.

## F-16 / HL-20 conformance pilot

The two pilots deliberately exercise opposite outcomes of the same workflow.

### F-16 S-119 — complete-path pilot

The F-16 must demonstrate that a source-grounded powered fixed-wing family can
move through every layer:

```text
source manifest
→ convention firewall
→ seven operating points
→ runtime 6DOF plant
→ local A/B derivatives
→ bounded elevator/aileron/rudder/throttle allocation
→ local LQR and maneuver witnesses
→ 3DOF/pseudo-6DOF reductions
→ shared racetrack
→ packet and local T5 promotion
```

The current F-16 packet is the reference output for this path. It remains
explicitly local-development evidence: continuous schedule transitions,
source-validated actuator dynamics, release robustness, fuel depletion, and
full-envelope qualification remain separate gates.

### HL-20 Mod K — fail-closed pilot

The HL-20 must use the same readiness and promotion machinery while preserving
its actual gaps:

- source rigid-body replay may be available;
- surface allocation may be contract-ready but not qualified;
- 3DOF and pseudo-6DOF reductions may be planned or pending;
- glide guidance, energy corridors, and terminal handoff remain future gates;
- landing/contact behavior remains a nonclaim.

The workflow passes only if it produces a useful blocker report rather than
silently selecting a parent profile, inventing an actuator model, or lowering
to an unvalidated reduction.

## Operational command sequence

The long-running workflow should converge on this sequence:

```bash
# 1. Intake and readiness
taoryx vehicle intake existing-family ...
taoryx vehicle integration readiness <family-id>
taoryx vehicle integration pipeline <family-id>

# 2. Convention and source diagnostics
taoryx vehicle diagnose <family-id> --stage conventions
taoryx vehicle diagnose <family-id> --stage plant

# 3. Trim and operating points
taoryx vehicle trim <family-id> --recipe <trim-recipe>
taoryx vehicle operating-points <family-id> --catalog <catalog>
# Transitional provider-neutral recipe/worklist validation:
.venv/bin/python tools/validate_vehicle_trim_orchestration.py \
  --family all --json verification/vehicle_trim_orchestration.json

# 4. Fidelity and control evidence
taoryx vehicle fidelity <family-id>
taoryx vehicle controller <family-id> --profile development
.venv/bin/python tools/validate_vehicle_controller_mission_preflight.py \
  --family all --allow-blocked

# 5. Mission preflight and packet
taoryx vehicle preflight <family-id> --mission <mission-id>
taoryx vehicle qualify <family-id> --profile development
taoryx vehicle report <family-id>
```

The lower-level CI/developer counterpart remains available when a repository
artifact is required rather than a Product 3 authoring report:

```bash
.venv/bin/python tools/validate_vehicle_integration_readiness.py \
  --family reference_f16_s119
```

Unimplemented commands are contractual targets, not pretend aliases. Until a
command exists, the readiness report must identify that layer as pending.

## Automation boundary

Taoryx should automate:

- schema and artifact discovery;
- source hash and convention checks;
- fidelity data checklists;
- trim solving once a family recipe declares unknowns and residuals;
- deterministic trim worklist generation from a family recipe and operating-
  point catalog;
- operating-point continuation;
- finite-difference/automatic linearization evidence;
- effectivity, allocation, actuator, and saturation diagnostics;
- bounded controller sweeps;
- mission time and authority preflight;
- controller-profile and mission-binding preflight with capability-based time
  estimates;
- packet generation, hashes, and promotion decisions.

The family adapter must still declare:

- what the source states and controls mean;
- the trim equations and physical constraints;
- force, moment, resource, and frame conventions;
- physical effector topology and effectiveness;
- family-specific validity limits;
- which missing physics are acceptable nonclaims.

Automation may report uncertainty or block a tier. It may not fill missing
physics with plausible constants or infer physical actuator authority from a
stable direct-wrench trajectory.

## Long-running exit matrix

| Gate | F-16 pilot | HL-20 pilot | Completion evidence |
|---|---|---|---|
| Intake/readiness | ready for runtime probes | explicit blockers allowed | machine-readable readiness report |
| Convention firewall | pass | pass or named blocker | units/frames/sign report |
| Trim recipe | seven local work items; residuals pending at six points | one source-bounded pitch-channel work item | recipe/worklist artifact, then adapter residual and continuation artifact |
| Parent plant | runtime 6DOF | source replay | plant diagnostic |
| Physical controls | local bounded surface path | contract or qualified allocator | effectivity/allocation report |
| Reductions | semantic mission parity | pending until derived | reduction comparison report |
| Mission | shared powered-fixed-wing racetrack | glide/energy handoff | independent objective report |
| Promotion | local T5, no family badge | blocked until gates pass | exact promotion/blocker report |

The automation workstream is complete only when both pilots use the same
commands, schemas, diagnostic stages, and promotion logic, while producing
different evidence outcomes where their physical data justify different
claims.

## Definition of done

A new vehicle is generically integrated when a source package can be registered
through the intake workflow, its missing data are reported, its trim and
operating points are generated by a reusable engine, its fidelity eligibility
is explicit, its controller path is physically honest, its mission horizon is
preflight-estimated, and one command produces the evidence packet and exact
promotion decision.

The family adapter may still contain domain-specific physics. What must no
longer be manual is discovering the required work, duplicating validation
plumbing, regenerating dependent artifacts, or deciding maturity from an
informal reading of plots.

## A320 and NESC automatic-integration pilots

The next test slice deliberately exercises two manifest boundaries rather than
forcing every model through the F-16/HL-20 DAVE-ML reference-family pipeline:

```bash
.venv/bin/python tools/validate_vehicle_integration_pilots.py \
  --family all --allow-blocked \
  --json verification/vehicle_integration_pilots.json \
  --packet-dir verification/integration-packets
```

The pilot uses the same staged vocabulary as the main pipeline, but discovers
either a `collection-manifest.json` or a source-grounded `family.yaml` and
reads the existing operational contract and operating-point catalog.

The pilot command first regenerates the A320 reduced mission evidence through
the shared `powered_fixed_wing_racetrack_v1` seam. This is the executable
mission hook, not a generic controller shortcut:

```bash
.venv/bin/python tools/validate_a320_racetrack.py --mode all \
  --dt-s 0.2 --output-dir verification/a320_racetrack
```

The runner uses OpenAP performance channels for speed, altitude, mass, fuel
flow, thrust, and drag; it uses a bounded kinematic navigation law for the
point-mass path; and it adds the named pseudo-6DOF attitude-response bridge
only in the surrogate lane. Truth gates are evaluated independently from the
telemetry. The pseudo lane's typed
`families/a320_openap_jsbsim_pseudo6dof/qualification/effector-contract.yaml`
records policy bounds and source/overlay boundaries without implying physical
actuator qualification.

The expected first result is intentionally mixed:

| Pilot | Passing automatic lanes | Explicit boundary or blocker |
| --- | --- | --- |
| NESC Scenario 17 | Intake, conventions, source replay plant, trajectory checkpoint, and mission segment graph | Trim, effectivity, and controller are not applicable; deferred attitude reduction keeps fidelity at development. |
| A320 OpenAP 3DOF | Collection, conventions, plant, derived-exact trim, operating point, performance fidelity, shared racetrack, and four independent truth gates | No physical attitude/effectivity claim; broader operating-point witnesses remain. |
| A320 OpenAP + JSBSim pseudo-6DOF | Collection, conventions, plant, matched trim, typed overlay effectivity, operating point, shared racetrack, and four independent truth gates | Composite effectivity, Taoryx controller, and pseudo-6DOF fidelity remain development. |

This pilot is diagnostic evidence, not a new qualification badge. The
normalized snapshot now records whether evidence is a local file, an external
dependency, or inline status, along with declared and observed hashes. Local
hash drift is a hard intake blocker; external source packages remain explicit
handoff dependencies. NESC additionally reports the eligible lower profiles
(`nesc_rocket.performance_3dof.v1` and
`nesc_rocket.variable_mass_6dof.v1`) while retaining its deferred
`attitude_response_p6dof` profile. `automatic_lowering_eligible` is still false
for the surrogate-composite A320 pseudo-6DOF collection because its rotational
authority and control overlay are composed from a different source boundary.

####
