# Simulation Runtime maturity plan

Simulation Runtime is the TAORYX simulation, external-stepping, control, telemetry,
and artifact surface. Mission Composition can compose a vehicle and mission, but
Simulation Runtime owns the question “what happened at each accepted truth
boundary?”

This plan closes the consumer-facing maturity gap without widening model
claims. The operational entry point is
[Simulation Runtime onboarding](../SIMULATION_RUNTIME_ONBOARDING.md).

## Current baseline

The following capabilities are already present:

- source `.prb`/`.tbl` batch execution through one runtime runner;
- `LoadedProgram` inspection, case copying, branching, and checkpoints;
- `InteractiveSession.step(duration, commands)` with bounded named commands;
- deterministic event and command histories;
- committed-truth, transition, sensor-clock, and load-evaluation timing
  records;
- normalized `RunArtifact` telemetry, CSV/SQLite sinks, HTML, and plots; and
- four explicitly named realization tiers: point-mass 3DOF, pseudo-6DOF,
  rigid-body direct wrench, and rigid-body physical-effector allocation.

The baseline is evidence-bounded. A parser pass is not a run, a response-law
pseudo-6DOF result is not a participating rigid-body plant, and a successful
trajectory is not vehicle qualification.

## Consumer friction to remove

The current source and composition examples are correct but distributed across
`examples/mission_families/`, `examples/showcases/`,
`examples/vehicle_families/`, `families/`, and
`examples/vehicle_composition/`. A new engineer can find a filename without
knowing whether it is:

- a source scenario, a model package, or a semantic composition request;
- point-mass, pseudo-6DOF, native rigid-body, or physical-effector evidence;
- batch-only or episode-capable;
- expected to complete, expected to stop at an envelope, or intentionally
  classified as a bounded failure; or
- a synthetic California–Hawaii route, an HL-20 comparison, or an X-15
  staged-booster witness.

Simulation Runtime maturity therefore includes discoverability and honest result
classification, not only more integrators or more vehicle names.

## Work packages

| ID | Goal | Exit evidence |
| --- | --- | --- |
| `P2-DISC-01` | Give consumers one scenario map and a clear source/model/composition vocabulary. | The onboarding page links every primary staged-rocket, HL-20, X-15, and CA–HI entry with fidelity, operation, command, and nonclaim. |
| `P2-STEP-01` | Make fixed-step, adaptive-step, print cadence, and external accepted boundaries understandable. | A runnable `InteractiveSession` example records requested duration, accepted interval, commands, events, and a normalized artifact. |
| `P2-ART-01` | Make normalized source artifacts first-class CLI outputs. | `taoryx run --artifact ...`, `taoryx artifact inspect ...`, and `taoryx artifact plot ...` work from the same named JSON artifact. |
| `P2-PSEUDO-01` | Keep pseudo-6DOF response laws distinct from physical rigid-body realizations. | Every pseudo-6DOF recipe names its response law, omitted physics, operation type, and batch/episode availability. |
| `P2-CAHI-01` | Reconcile the nearby California–Hawaii names. | Synthetic CA–HI, HL-20 CA–HI, X-15 source, and X-15-scaled pseudo-6DOF paths are separately documented and regression-tested. |
| `P2-DIAG-01` | Give a junior engineer a narrowest-first diagnostic ladder. | Parser, table, resolution, preflight/lowering, execution, artifact, and model-specific gate commands are documented with exit semantics. |
| `P2-PARITY-01` | Expose batch/episode parity only when a committed-boundary witness exists. | Batch-only families stay batch-only; exact registered action traces are the only parity claims. |

## Maturity ladder

```text
D0 discover
  -> D1 validate and resolve
  -> D2 bind the exact model/fidelity
  -> D3 run or step at accepted truth boundaries
  -> D4 inspect diagnostics and artifacts
  -> D5 reproduce and compare only declared evidence
```

### D0 — Discover

A consumer can answer all of these before editing a model:

- What is the source scenario or composition request?
- Which model family and fidelity does it select?
- Is the operation batch, interactive, or both?
- What is the expected terminal condition?
- What is explicitly not claimed?

The [Simulation Runtime onboarding map](../SIMULATION_RUNTIME_ONBOARDING.md#find-the-right-scenario)
is the current D0 artifact.

### D1 — Validate and resolve

The source and table set must pass grammar validation and form a resolved
scenario or compiled composition. Diagnostics must include the input path and
the failing field/table/segment. A successful D1 result does not run dynamics.

### D2 — Bind

The runtime must select the exact declared family adapter or source-owned
factory. A missing episode operation, participating plant, or physical
effector remains a visible blocker. Generic family fallback is not a maturity
shortcut.

### D3 — Run or step

Every accepted interval must retain enough information to answer:

- requested versus applied command;
- start and end truth time;
- active segment and event transitions;
- selected integrator/cadence;
- source or composition identity; and
- the selected fidelity and realization.

For adaptive integration, internal solver steps may be shorter than the
external `InteractiveSession.step` duration. The returned snapshot, not the
requested duration alone, is the accepted truth record.

### D4 — Inspect

The first inspection should be structural, not visual. Review sample count,
time span, channel names, event count, termination, status/resource channels,
and source hashes before reading a plot. Then review native objective/gate
reports and normalized evaluation separately.

### D5 — Reproduce and compare

A result is reproducible only when the source/composition fingerprint, model
inputs, integrator/cadence, seed, command stream, and output contract are
retained. Batch/episode equivalence is a separate witness and is never inferred
from two nominal runs that happen to look similar.

## Milestone roadmap

The maturity objective is to make Simulation Runtime predictable and
self-describing. The next level is not simply more vehicle names; it is a
smaller gap between “I found an example” and “I can reproduce and interpret a
bounded result.” Work should proceed in dependency order.

### M0 — Baseline and contract reset

Purpose: establish a trustworthy starting point before adding more surface area.

Deliverables:

- resolve the existing HL-20 preflight status mismatch between the committed
  mission binding and the stale `blocked` test expectation;
- freeze the Simulation Runtime status vocabulary: `passed`, `development`, `blocked`,
  `incomplete`, `out_of_envelope`, and `failed`;
- define the minimum run-manifest fields and schema compatibility policy in
  [the Simulation Runtime run-manifest contract](../api/simulation-runtime-run-manifest.md); and
- record baseline timings, artifact sizes, and deterministic repeatability for
  the canonical regression set in
  [`verification/simulation_runtime_baseline.json`](../../verification/simulation_runtime_baseline.json),
  generated by [`tools/build_simulation_runtime_baseline.py`](../../tools/build_simulation_runtime_baseline.py).

Exit gate: the required repository checks are green, every canonical scenario
has an expected disposition, and no consumer-facing command relies on an
unstated status interpretation.

### M1 — Discoverability and `doctor`

Purpose: make the right scenario and first diagnostic discoverable without
reading the repository tree manually.

Deliverables:

- add a machine-readable scenario catalog with IDs, aliases, source inputs,
  fidelity, operation type, expected disposition, command, and claim boundary;
- add `taoryx scenario list`, `search`, and `show`; and
- add `taoryx doctor <scenario-id>` to run validation, table inspection,
  compilation, resolution, preflight, lowering, and capability checks in order.

Implementation: [`verification/simulation_runtime_scenario_catalog.yaml`](../../verification/simulation_runtime_scenario_catalog.yaml),
the `taoryx scenario` discovery commands, and the
[`simulation_runtime_doctor.py`](../../src/taoryx/simulation_runtime_doctor.py) diagnostic
ladder.

Exit gate: a new engineer can identify and validate each canonical scenario
from one catalog entry, and every blocking diagnostic includes a stable code,
location, remediation, and blocking status.

Dependencies: M0 status vocabulary and scenario disposition contract.

### M2 — Universal artifacts and reproducible bundles

Purpose: make source runs, composition runs, and interactive sessions produce
the same inspectable evidence shape.

Deliverables:

- extend the normalized run manifest with source/composition hashes, git
  revision, runtime version, platform, seed, integrator, cadence, fidelity,
  realization, and termination reason;
- make `RunArtifact` emission uniform across source, composition, and external
  stepping paths;
- add artifact schema validation and compatibility checks; and
- add `taoryx scenario bundle <scenario-id>` to collect inputs, manifest,
  resolved request, provenance, reproduction command, telemetry, and plots.

Implementation: [`simulation_runtime_manifest.py`](../../src/taoryx/simulation_runtime_manifest.py),
[`simulation_runtime_bundle.py`](../../src/taoryx/simulation_runtime_bundle.py), and the
`scenario bundle`, `scenario manifest validate`, and `artifact validate`
commands.

Exit gate: a bundle can be copied to another workspace and its reproduction
command, artifact inspection, and claim boundary remain self-contained.

Dependencies: M1 catalog IDs and M0 manifest schema.

### M3 — Stepping, replay, and boundary semantics

Purpose: make external time stepping reliable enough for consumers, controls,
and later AI/RL integrations.

Deliverables:

- distinguish integration cadence, print cadence, requested external duration,
  accepted truth interval, and event-boundary truncation in the API;
- return structured requested/applied/accepted command records from every
  `InteractiveSession.step()` call;
- make checkpoint and replay identities include the command stream and model
  fingerprint; and
- register explicit batch/episode parity witnesses without implying parity for
  batch-only families.

Exit gate: a recorded action stream can be replayed deterministically, accepted
  intervals and event truncation are visible, and parity reports are emitted
  only for registered scenario pairs.

Dependencies: M2 universal manifest and artifact identity.

#### M3 completed implementation

The M3 implementation is attached to the shared interactive runtime rather
than a separate stepping kernel. `InteractiveSnapshot` records the requested
duration, accepted truth interval, integration/output cadence metadata,
accepted internal-boundary reasons, and event truncation. `AppliedCommand`
records requested, bounded/applied, and accepted start/end values. Explicit
`EventSpec.residual` functions are refined to an accepted boundary before a
stop or signal is applied. Interactive artifacts and checkpoints carry a
stable model fingerprint, command-stream hash, and replay identity.

The deterministic composition replay report now also carries the registered
batch/episode parity disposition. It invokes the centralized fail-closed
adapter only when the exact family/mission/fidelity tuple is registered and
nests the resulting parity report in `batch_episode_parity.report`. For
unregistered or batch-only tuples it emits only the explicit disposition and
never manufactures a parity report. The checked-in witness catalog is replayed
through this same report path by
`tools/validate_vehicle_execution_witnesses.py --execute-parity`, and
`tools/dev.py check` exercises that gate.

M3 exit evidence is therefore: deterministic action-stream replay, visible
accepted intervals and event truncation, identity-bound checkpoints, and
passing parity reports for all and only the 11 registered pairs.

### M4 — Numerical quality and fidelity gates

Purpose: distinguish a runnable trajectory from a numerically trustworthy
Simulation Runtime result.

Deliverables:

- add repeatability checks for fixed seeds and identical inputs;
- add `dt`/integrator refinement checks for selected canonical scenarios;
- validate finite telemetry, event ordering, state continuity, and resource or
  mass conservation where the model declares those invariants;
- require every pseudo-6DOF profile to declare its response law, omitted
  physics, controls, envelope, and operation availability; and
- keep HL-20, NESC, X-15, and synthetic CA–HI evidence in separate fidelity
  and claim-boundary lanes.

Exit gate: each canonical scenario has a machine-readable quality report with
pass, development, blocked, or bounded-failure disposition and no implicit
promotion from a visual or nominally successful trajectory.

Dependencies: M2 artifact fields and M3 accepted-boundary/replay records.

#### M4 implementation slice

The M4 contract is implemented in `taoryx.simulation_runtime_quality` and carried by
the Simulation Runtime scenario catalog. It emits
`taoryx.simulation-runtime-numerical-quality/v1alpha1` reports with explicit fidelity
lanes and claim boundaries, finite selected telemetry, event ordering, state
continuity, declared mass/energy invariants, fixed-input repeatability, and
coarse/fine step-and-integrator refinement. The catalog requires isolated
NESC, X-15, HL-20, and synthetic CA–HI lanes and has two executable source
refinement witnesses plus the interactive accepted-boundary witness.

`tools/dev.py check` writes the suite report under the ignored
`artifacts/verification/simulation_runtime_quality/` directory. The two source
witnesses currently pass the numerical gates, including a declared accepted
event-time bound for the spawned child in the changing-mass example. The
remaining family-owned lanes are reported as `blocked` or `development` until
their common Simulation Runtime artifact adapters are declared; this is intentional and
does not promote their nominal trajectories.

M4 evidence is therefore: a typed catalog declaration, a machine-readable
report for all eight canonical scenarios, passing selected repeatability and
refinement checks, explicit fidelity-lane separation, and bounded dispositions
for evidence that is not yet in the common artifact contract.

### M5 — Release and consumer operations

Purpose: make Simulation Runtime maintainable as a capability rather than a collection of
working examples.

Deliverables:

- execute documentation examples in CI and detect command/documentation drift;
- publish a versioned scenario catalog and artifact schema;
- add performance budgets for representative runs, stepping latency, and
  bundle size;
- provide a release smoke command for the canonical scenario set; and
- produce a release packet containing catalog, schemas, bundles, quality
  reports, known limitations, and reproduction instructions.

Exit gate: a clean checkout can discover, validate, run, inspect, and reproduce
the canonical scenarios using documented commands, with expected bounded
failures treated as explicit results rather than CI ambiguity.

Dependencies: M1 through M4.

### Milestone decision gates

Use these decisions to prevent scope from expanding faster than maturity:

| Gate | Decision | Required evidence |
| --- | --- | --- |
| G0 | Is the baseline trustworthy? | Green checks, resolved status mismatch, scenario dispositions. |
| G1 | Is the capability discoverable? | Catalog plus `scenario show`/`doctor` for every canonical case. |
| G2 | Is a result portable? | Self-contained bundle and schema-valid manifest. |
| G3 | Is stepping reproducible? | Replayable command trace with accepted-boundary records. |
| G4 | Is a result numerically bounded? | Repeatability/refinement/invariant reports. |
| G5 | Is the release operable? | Documentation CI, performance budgets, and release packet. |

Do not start M4-wide numerical promotion work while M2 artifact identity is
still unstable. Do not claim M3 parity for a family that has no registered
batch/episode witness. Do not use M5 packaging to conceal unresolved M0 status
or fidelity-boundary failures.

## Target regression set

These are the minimum Simulation Runtime onboarding regressions, not a promise that all
are equally qualified:

| Regression | Primary evidence |
| --- | --- |
| Two-stage ballistic smoke | Ordinary source runner, changing phase, finite output artifact. |
| Synthetic staged-rocket family | Source tables, stage separation, changing mass, and inherited child state. |
| NESC pseudo-6DOF | Source-history replay, ordered staged events, mass/resource telemetry, response-law label. |
| X-15-scaled pseudo-6DOF | Boost, coast/release, glide handoff, passive impact, and local-frame nonclaim. |
| HL-20 four-fidelity release ladder | Shared search space, source envelope checks, response-law/native/surface tier labels, deployment lineage, and timeout policy. |
| Synthetic CA–HI native 6DOF | Route artifact and synthetic claim boundary. |
| Interactive CA–HI stepping | Accepted external intervals, bounded commands, replayable artifact, and checkpoint boundary. |

## Definition of done for a new Simulation Runtime scenario

A new scenario is ready for consumer handoff when its packet has:

- one discoverable source or composition entry point;
- model family, fidelity, realization, and operation type;
- explicit initialization and table/source inputs;
- one validate/resolve command and one run/step command;
- expected completion, timeout, envelope, or failure disposition;
- normalized telemetry and event artifacts;
- source and input hashes or a stable composition identity;
- a diagnostic command for the most likely setup failure; and
- a nonclaim statement that is as specific as the positive result.

The required repository gates remain the authority after Simulation Runtime changes:

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py check
python -m pytest
```

For a release packet, use `python tools/dev.py handoff` after the normal gates.
