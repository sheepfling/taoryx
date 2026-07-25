# TAORYX Alpha 1 release plan

The running implementation log is [`taoryx-alpha-1-progress.md`](taoryx-alpha-1-progress.md).

## Purpose

This is the release-level plan for the first usable TAORYX Alpha. It turns the
Alpha definition of done into a bounded sequence of implementation, verification,
and evidence gates. It is deliberately narrower than “implement the whole
manual” and stronger than “the examples run.”

Alpha 1 is complete only when each supported claim is traceable to a source
requirement, an executable implementation, a test, and a reproducible artifact.
Unsupported or ambiguous behavior must remain visible as a diagnostic or an
explicitly provisional extension.

## Historical reference and terminology

The historical reference is SAND95-1652, printed in December 1995. Its title
identifies the software as **TAOS Version 96.0**. In this plan:

- **TAOS 96.0 scope** means the documented table/problem language and the
  manual-bounded point-mass 3-DOF behavior. It is not a claim of runtime
  equivalence with the historical executable.
- **TAORYX extension** means successor-side behavior explicitly identified in
  the `taoryx` grammar profile or runtime contracts.
- **3-DOF** means point-mass translational dynamics.
- **pseudo-6-DOF** means the kinematic 3+3 bridge: 3-DOF translation plus a
  declared prescribed or lagged attitude/control sidecar. It does not integrate
  free rigid-body moments.
- **rigid-body 6-DOF** means free translational and rotational dynamics with
  body rates, attitude, inertia, moments, actuators, and frame adapters as
  applicable.

Modern TAOS descriptions may mention both point-mass and rigid-body modes. That
does not change the historical manual boundary. Alpha 1 keeps the three levels
explicit so a successor extension cannot be mistaken for a historical claim.

## Release definition of done

TAORYX Alpha 1 is done when all of the following are true:

1. The repository contains an executable, test-linked specification of the
   in-scope TAOS 96.0 table and problem language features from Chapters 3 and 4.
2. The typed parser and validator recover through a file, report source-located
   diagnostics, and reject unsupported or ambiguous runtime behavior before
   initialization.
3. The deterministic fixed-step runtime has one canonical public transition.
   `run()` is implemented as repeated calls to `step()`; both paths share state,
   derivatives, controls, events, and artifact contracts.
4. The four manual problem families execute through the supported 3-DOF path:
   ballistic reentry, ballistic rocket, air-launched intercept, and
   ground-launched intercept.
5. TAORYX has documented, validated composition contracts for reusable
   parameters, immutable tables, runtime-owned state, named controls, segments,
   events, and outputs.
6. The example suite contains explicit point-mass 3-DOF, kinematic pseudo-6-DOF,
   and rigid-body 6-DOF cases. Every case declares its claim boundary and mode.
7. The four current research-surrogate vehicle families—B747, Skywalker X8,
   Hummingbird, and X-15—have catalog entries, source/provenance records,
   convention-firewall reports, and at least one reproducible artifact-backed
   scenario at their claimed fidelity. A bounded scenario is not promoted to an
   engineering-validity claim by passing this gate.
8. Run artifacts include resolved inputs, source/table hashes, model graph,
   events, controls, Parquet telemetry, summaries, diagnostics, and declarative
   plots. A reader can audit a result without relying on a stale image.
9. A clean build, the required tests, and the release handoff complete in a
   pinned Python environment with no unresolved Alpha stop-ship condition.

This definition does **not** claim historical TAOS 96.0 numerical compatibility,
global vehicle validity, flight qualification, or certification.

## Claim matrix

Alpha 1 uses separate claims instead of one global “compatible” label.

| Claim | Alpha 1 meaning | Required evidence | Release status |
| --- | --- | --- | --- |
| D — Documentary fidelity | The reconstructed manual and source-linked metadata represent the printed reference. | Page comparison, transcription review, equation/table provenance, editorial ledger. | Required for the manual scope. |
| S — Semantic coherence | Definitions, units, defaults, restrictions, and derived rules are internally coherent. | Requirement registry, dimensional checks, derivations, contradiction and ambiguity review. | Required for executable features. |
| P — Parser conformance | The parser accepts documented forms and diagnoses malformed, unsupported, or ambiguous forms. | Positive/negative fixtures, recovery tests, source locations, diagnostic matrix. | Required for the in-scope language. |
| N — Numerical correctness | The implemented kernel satisfies independent mathematical checks for its declared mode. | Invariants, independent oracles, property/mutation tests, convergence, closure. | Required per promoted runtime feature. |
| C — Composition/execution | Models, controls, segments, events, and outputs compose through the public runtime contracts. | Contract tests, step/run parity, event traces, replayable artifacts. | Required for Alpha examples. |
| H — Historical compatibility | Behavior matches TAOS 96.0 itself. | Historical executable/source or a trusted historical output corpus. | Explicitly excluded until an oracle exists. |

No D, S, P, N, or C result may silently imply H.

## Scope boundaries

### Included in Alpha 1

- Chapter 3 table declarations and Chapter 4 problem constructs represented in
  the feature registry, including the four manual problem families.
- Lossless parsing, validation, recovery, and diagnostics for the supported
  syntax profiles.
- Fixed-step fourth-order Runge-Kutta execution for the TAOS-bounded 3-DOF
  runtime path.
- A public `step()` transition, deterministic `run()`, named control schemas,
  event/segment transitions, and artifact sinks.
- Reusable rail, boost, coast, staging, guided-flight, intercept, and reentry
  segment contracts where each contract has typed preconditions, exits,
  postconditions, and safety bounds.
- Kinematic pseudo-6-DOF and rigid-body 6-DOF successor extensions with their
  own state, data, frame, actuator, and verification contracts.
- Source-bounded demonstration data for B747, Skywalker X8, Hummingbird, and
  X-15, including units, mass/inertia, controls, table margins, and provenance.
- Reproducible numerical telemetry, SQLite/JSON/Parquet views, summaries, and
  plots generated from the same normalized run artifact.

### Explicitly not an Alpha 1 claim

- Full historical TAOS runtime compatibility or an implementation of every
  undocumented/defaulted historical behavior.
- A complete modern flight-dynamics product, certified aerodynamics, or a
  globally valid envelope for any research-surrogate vehicle.
- Historical 6-DOF behavior merely because modern TAOS descriptions mention
  rigid-body capabilities.
- Certification, flight qualification, or operational mission performance.
- Silent resolution of Class-D ambiguities, unverified source corrections, or
  unsupported optimizer/search semantics.

## Required example coverage

The release suite has two related but separate responsibilities: prove the
manual-facing 3-DOF execution surface and prove the successor composition/mode
extensions.

### Manual-facing problem families

| Family | Minimum Alpha 1 route | Required evidence |
| --- | --- | --- |
| Ballistic reentry | TAOS-profile `.prb` + referenced `.tbl`, parsed, validated, and executed as 3-DOF. | Source fixture, resolved input, trajectory artifact, invariant/tolerance report, plot. |
| Ballistic rocket | 3-DOF boost/coast problem with declared propulsion and segment/event boundaries. | Mass/propellant history, event trace, continuity and convergence report. |
| Air-launched intercept | Multiple trajectories/vehicles and intercept guidance through the supported problem route. | Target/vehicle identity, guidance diagnostics, endpoint metric, event trace. |
| Ground-launched intercept | Ground launch, guidance, and endpoint/termination semantics through the supported problem route. | Initial-condition audit, route/endpoint metric, failure-safe termination, plot. |

These are minimum execution surfaces, not historical output-equivalence claims.

### Fidelity ladder and vehicle extensions

| Tier | Alpha 1 requirement | What it must demonstrate |
| --- | --- | --- |
| 3-DOF | Source-compatible point-mass state and fixed-step transition. | Translation, atmosphere/forces where declared, controls/guidance, events, and deterministic artifacts. |
| Pseudo-6-DOF | Kinematic 3+3 bridge with explicit prescribed/lagged attitude and control semantics. | Same translational source history plus an auditable attitude sidecar; no false moment-integration claim. |
| Rigid-body 6-DOF | Typed translational/rotational state and free attitude/rate integration. | Force/moment closure, inertia/CG contract, convention firewall, actuator limits, and convergence. |

Each tier gets an independent fixture and report. A bridge result cannot promote
the corresponding rigid-body result.

The four current vehicle families must each have:

- a source/provenance-backed catalog record;
- canonical units, frames, reference geometry, mass, CG, and inertia;
- table-axis and control-direction verification;
- a trim or bounded-equilibrium report appropriate to the family;
- at least one longer, family-appropriate scenario artifact;
- explicit unsupported/envelope termination behavior; and
- a claim matrix that says whether the result is documentary, semantic,
  numerical, bounded-execution, or controller evidence.

## Alpha 1 workstreams and exit gates

### A0 — Claim freeze and release registry

Create and maintain:

- `verification/claims.md` for claim boundaries and exclusions;
- `verification/requirements.yaml` for stable requirement IDs;
- `verification/alpha1_release_plan.yaml` for this release gate inventory;
- `verification/alpha1_feature_matrix.yaml` for the generated per-feature
  grammar, semantics, implementation, fixture, test, and requirement links;
- an Alpha feature matrix linking each Chapter 3/4 feature to grammar,
  semantics, validation, implementation status, fixtures, and tests;
- an ambiguity ledger with source/derived/adopted forms and reviewers.

Exit gate: every Alpha claim has an owner, evidence type, and explicit
non-claim. A release report can distinguish “documented,” “parsed,” “validated,”
“executable,” and “historically compared.”

Regenerate the matrix and current gate status with:

```bash
python tools/dev.py alpha1-feature-matrix
python tools/dev.py alpha1-composition-case
python tools/dev.py audit-alpha1
```

The status report is written below the ignored `artifacts/` tree and does not
widen the bounded parser or historical-compatibility claims.

### A1 — Manual, language, and diagnostic coverage

Complete the source-linked language inventory before expanding runtime behavior.
The parser must:

- accept every supported documented construct in the selected profile;
- recover after an error and report more than the first issue where possible;
- retain source spans and recovery text;
- reject unsupported runtime constructs before initialization;
- distinguish `taos96` syntax from `taoryx` extensions; and
- generate a compatibility matrix from metadata and tests.

Exit gate: every in-scope feature has a positive or negative fixture, a stable
diagnostic/semantic rule, and a linked test. No unsupported form is silently
lowered to a different behavior.

### A2 — Canonical runtime transition

Make the runtime boundary explicit:

```text
parse -> validate -> resolve -> RuntimeProblem
                              |
                 +------------+------------+
                 |                         |
                 v                         v
               step()                    run()
                 |                         |
                 +------------+------------+
                              v
                     normalized artifact
```

Implement or finish:

- typed state snapshots and `StepResult`;
- fixed-step deterministic stepping;
- named, unit-aware, bounded controls;
- event detection and segment transitions;
- pause/resume/interrupt semantics;
- batch/step replay parity; and
- resolved scenario identity and provenance.

Exit gate: a deterministic batch run is exactly a documented sequence of the
public transition; the same command stream and seed reproduce the same state,
events, and hashes.

### A3 — TAOS-bounded 3-DOF execution

Prioritize the historical manual-facing mode. Establish the canonical state,
frames, units, force providers, fixed-step RK4, and source-linked example
problems without implying historical runtime equivalence.

Exit gate: all four manual problem families parse, validate, execute, terminate
according to their declared event policy, and produce independent invariant,
continuity, and convergence evidence.

### A4 — Parameters, tables, controls, and composition

Stabilize the reusable data path so a new vehicle or segment is a catalog entry,
not a bespoke runner:

- immutable table registration and axis/margin reports;
- parameter and reference-geometry declarations;
- mass, CG, inertia, and actuator contracts;
- control profiles with normalization and saturation accounting;
- reusable segment preconditions, transitions, exits, and postconditions;
- typed model ports with frames, units, state ownership, and dependencies; and
- generated `.prb` products from metadata rather than duplicated hard-coded
  test setup.

Exit gate: a new catalog record can generate a problem fixture, run the common
convention firewall, and produce a diagnosable failure without editing a
vehicle-specific test harness.

### A5 — Pseudo-6-DOF bridge

Define the bridge as a real contract, not a second hidden solver. It must state
which attitude is prescribed, how lag/rates are generated, what controls are
available, and which translational quantities remain inherited from 3-DOF.

Exit gate: bridge outputs preserve the source 3-DOF translational history within
the declared tolerance, publish attitude/angle sidecar channels, and are visibly
labelled as kinematic. Bridge tests must fail if they accidentally claim free
moment dynamics.

### A6 — Rigid-body 6-DOF extension

Promote only the successor-side rigid-body contract:

- typed body/world frames and quaternion identity/handedness;
- force and moment dimensionalization;
- positive mass/inertia and CG checks;
- aerodynamic angle and table-margin checks;
- actuator command/achieved/rate/saturation channels;
- direct and independent equation-closure metrics;
- fixed/adaptive integration convergence; and
- family-appropriate trim and bounded maneuvers.

The current four vehicle families are evidence subjects, not one shared
controller model:

- B747: local source-anchored powered trim and bounded attitude/altitude
  maneuver;
- Skywalker X8: powered local trim, control doublets, and in-envelope recovery;
- Hummingbird: rotor allocation, hover, takeoff/waypoint/landing behavior;
- X-15: declared release/boost/coast/descent or valid ground termination with
  source-bounded hypersonic tables.

Exit gate: each family has a passing convention firewall and a report that
separates plant, controller, envelope, closure, convergence, and mission claims.

### A7 — Four manual examples and extension showcases

Build the release examples from standard problem files and generated metadata.
Each showcase must include:

- source and resolved problem files;
- model/segment graph;
- controls and event schedule;
- raw telemetry and normalized channels;
- table query/margin audit;
- equation closure and convergence report;
- objective/waypoint metrics with units;
- plots that mark mode changes and objective completion; and
- a claim-boundary README.

Exit gate: every image is reproducible from the packaged telemetry and every
reported score can be recomputed from a machine-readable metric definition.

### A8 — Release audit and handoff

Build a clean UUID-scoped handoff bundle. It must include source hashes,
resolved inputs, software commit, dependency lock, numerical settings, seeds,
all output hashes, reports, tests, and reproduction commands.

Exit gate: a clean environment can rebuild the manual, audits, generated
problem files, parser/runtime tests, and Alpha evidence packet; all references
resolve; no stale artifact is allowed into the bundle.

## Evidence and scoring contract

Alpha 1 uses mandatory validity gates before quality scoring.

### Mandatory gates

For every executable case, record pass/fail for:

- source and resolved-input provenance;
- canonical-unit and initial-state parity;
- table ingestion, axis order, and envelope margins;
- frame, quaternion, alpha/beta, and control-direction conventions;
- mass/CG/inertia positivity;
- continuity and declared event semantics;
- equation closure with metric name, frame, normalization, and sampling policy;
- step-size convergence on smooth intervals;
- actuator feasibility and saturation policy; and
- declared objectives, waypoints, or termination conditions.

Any mandatory failure blocks promotion of the affected claim.

### Quality score

Only after mandatory gates pass, compute a family-appropriate quality score from
declared, unit-bearing metrics such as:

- terminal and return-to-start error;
- cross-track and along-track error;
- altitude, speed, attitude, and rate tracking;
- control effort, average/maximum saturation, and control derivative;
- minimum normalized table margin;
- closure and convergence residuals; and
- robustness pass rate with retained failure cases.

The score formula must be monotonic in each metric, expose good/target/hard
limits separately, and never award a near-perfect quality score merely because a
trajectory stays below a loose hard-failure boundary.

## Alpha 1 release gates

| Gate | Name | Exit condition |
| --- | --- | --- |
| R0 | Claim freeze | Claims, exclusions, terminology, and operating modes are published. |
| R1 | Requirement inventory | All in-scope Chapter 3/4 features and Alpha extensions have stable IDs and owners. |
| R2 | Language safety | Positive/negative fixtures, recovery diagnostics, and profile boundaries pass. |
| R3 | Runtime transition | `step()` is canonical; `run()` replay, controls, events, and artifacts are deterministic. |
| R4 | Manual examples | All four manual problem families execute through the declared 3-DOF path. |
| R5 | Composition | Parameters, tables, controls, segments, and model ports compose without bespoke runner logic. |
| R6 | Fidelity ladder | 3-DOF, pseudo-6-DOF, and rigid-body 6-DOF have explicit state/claim contracts and tests. |
| R7 | Vehicle evidence | B747, X8, Hummingbird, and X-15 reports pass their declared convention/plant gates. |
| R8 | Artifact reproducibility | Raw inputs, telemetry, reports, plots, hashes, and reproduction command agree. |
| R9 | Handoff | Required repository commands and release-profile tests pass in a clean environment. |

R9 is the Alpha 1 release gate. R0–R8 are not replaceable by a single aggregate
score.

## Stop-ship conditions

Alpha 1 must not be declared complete if any of the following is true:

- unsupported syntax is silently accepted or assigned invented semantics;
- a parser diagnostic loses the source location needed to fix the file;
- `run()` and `step()` use separate numerical paths;
- a state, control, table, or segment contract has undeclared units or frames;
- a bridge result is presented as free rigid-body dynamics;
- a source correction or ambiguity is adopted without a permanent record;
- a vehicle passes a short smoke test while its declared table envelope or
  termination policy is violated;
- a mass, inertia, force, or moment scale is hard-coded outside the vehicle
  contract;
- a score cannot be recomputed from packaged telemetry and metric definitions;
- a packet references stale, absolute-path, missing, or unhashed inputs;
- a Class-D ambiguity silently affects normal execution; or
- any result uses historical TAOS 96.0 compatibility language without a
  historical oracle.

## Required release commands

The development loop remains fast by default. The Alpha 1 release profile must
run the normal gates plus the explicitly selected long/artifact/Simple Aero views:

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py generate-problems
python tools/dev.py check
python -m pytest
python tools/dev.py test-simple_aero
python -m pytest -m "slow or artifact or simple_aero"
python tools/dev.py alpha1-packet
python tools/dev.py handoff
```

The default pytest configuration continues to exclude `slow`, `artifact`, and
`simple_aero` tests for day-to-day development. Alpha release evidence must record
which optional views were run, their seeds/configuration, and their output
hashes.

## First execution tranche

The first Alpha 1 tranche should not start with more mission tuning. It should
close the release infrastructure in this order:

1. Add the feature/claim/status inventory and wire it to the generated
   compatibility report.
2. Define the serialized `StepResult`, control, event, and run-artifact
   contracts and add contract tests.
3. Make the four manual problem fixtures run through one 3-DOF transition.
4. Make generated vehicle problem files consume catalog metadata and emit the
   convention/scale/table audits.
5. Freeze the pseudo-6-DOF bridge semantics and add one parity fixture.
6. Promote one rigid-body 6-DOF golden plant per vehicle family, retaining
   failures at the lowest failing tier.
7. Build the four manual and four vehicle-family evidence packets.
8. Run the release profile, review the claim matrix, and either publish Alpha 1
   or record the exact blocked gate.

The release is complete when the last step has a reproducible green report—not
when every future TAORYX feature has been implemented.

####
