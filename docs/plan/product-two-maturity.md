# Product 2 maturity plan

Product 2 is the TAORYX simulation, external-stepping, control, telemetry,
and artifact surface. Product 3 can compose a vehicle and mission, but Product
2 owns the question “what happened at each accepted truth boundary?”

This plan closes the consumer-facing maturity gap without widening model
claims. The operational entry point is
[Product 2 onboarding](../PRODUCT_TWO_ONBOARDING.md).

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

Product 2 maturity therefore includes discoverability and honest result
classification, not only more integrators or more vehicle names.

## Work packages

| ID | Goal | Exit evidence |
| --- | --- | --- |
| `P2-DISC-01` | Give consumers one scenario map and a clear source/model/composition vocabulary. | The onboarding page links every primary staged-rocket, HL-20, X-15, and CA–HI entry with fidelity, operation, command, and nonclaim. |
| `P2-STEP-01` | Make fixed-step, adaptive-step, print cadence, and external accepted boundaries understandable. | A runnable `InteractiveSession` example records requested duration, accepted interval, commands, events, and a normalized artifact. |
| `P2-ART-01` | Make normalized source artifacts first-class CLI products. | `taoryx run --artifact ...`, `taoryx artifact inspect ...`, and `taoryx artifact plot ...` work from the same named JSON artifact. |
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

The [Product 2 onboarding map](../PRODUCT_TWO_ONBOARDING.md#find-the-right-scenario)
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

## Target regression set

These are the minimum Product 2 onboarding regressions, not a promise that all
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

## Definition of done for a new Product 2 scenario

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

The required repository gates remain the authority after Product 2 changes:

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py check
python -m pytest
```

For a release packet, use `python tools/dev.py handoff` after the normal gates.
