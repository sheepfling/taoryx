# taoryx simulation architecture

These notes describe the implementation boundaries for the TAOS successor.
The manual describes a trajectory-analysis system with several distinct
responsibilities; the replacement should preserve those boundaries instead of
turning the `.prb` parser into the simulator.

For a user-facing tour of the three connected layers—Model Authoring,
Simulation Runtime, and Mission Composition—start with the
[Authoring → Runtime → Composition showcase guide](../AUTHORING_RUNTIME_COMPOSITION_SHOWCASE.md).
This page is the deeper reference for the runtime and its boundaries.

Use this section when you want the runtime shape, the table explorer / plotter
boundary, the telemetry contract, or the analysis workspace in one place.

The broader implementation sequence is tracked in
[`../plan/taoryx-successor-roadmap.md`](../plan/taoryx-successor-roadmap.md).
The native guided rigid-body work is staged separately in
[`../plan/native-guided-6dof-implementation.md`](../plan/native-guided-6dof-implementation.md).
The composition, resolved-scenario, artifact, and visualization sequence is
tracked in
[`../plan/composable-scenario-runtime.md`](../plan/composable-scenario-runtime.md).
The common vehicle metadata, parameter, semantic control, resource, status,
and AI/RL observation boundary is defined in
[vehicle-interface-contract.md](vehicle-interface-contract.md).
The portable Mission Composition vehicle/trajectory plug-in publication,
request, and standard result envelope is defined in
[Mission Composition Provider API](mission-composition-provider-api.md).
Installable Python package discovery, typed registry aggregation, collision
rules, and the current extraction state are defined in
[Installable model and provider plug-ins](plugins.md).
Package selection, source and wheel installation, contributor bootstrap, and
installed-entry-point verification are covered by the
[installation guide](../INSTALLATION.md).
The shortest consumer-facing route through that contract is the
[Mission Composition front door](../MISSION_COMPOSITION.md).
The generated
[Mission Composition family/realization coverage matrix](mission-composition-coverage-matrix.md)
is the current inventory, exact batch/session availability, telemetry,
spawned-child, and blocker summary.
The progressive point-mass, kinematic bridge, and two rigid-body control-
realization tiers are described in
[`dynamics-fidelity-ladder.md`](dynamics-fidelity-ladder.md).
The controller, trim, guidance, regulator, allocator, and fidelity mapping
contract is described in
[`controller-stack.md`](controller-stack.md).
The explicit force, moment, actuator, propulsion, resource, and claim-boundary
inventory is described in
[`vehicle-realizations.md`](vehicle-realizations.md).
The tiered data intake contract and machine-readable readiness checklist is
described in
[`fidelity-data-requirements.md`](fidelity-data-requirements.md).
The reusable trim, true-derivative linearization, and dimension-matched LQR
tuning pipeline is described in
[`generic-controller-tuning.md`](generic-controller-tuning.md).
The common installed-model inventory, plain Python/YAML mission authoring,
segment/waypoint scaffolding, and plug-in-owned campaign registration path is
described in
[Model-to-mission authoring and automation](model-authoring-automation.md).
The promotion path from a direct-wrench LQR screen to bounded physical
effectors and nonlinear validation is described in
[`physically-realizable-lqr-control.md`](physically-realizable-lqr-control.md).
The accepted-state, force-evaluation, and sensor timing boundary is defined in
[`eom-timing-contract.md`](eom-timing-contract.md).
The typed sensor-family extension boundary, committed scene context, payload
codecs, and bundled inertial/IR/GNSS implementations are defined in
[`sensor-plugin-api.md`](sensor-plugin-api.md).
The language/implementation contract for pre/post truth around events is
defined in [`../extensions/transition-truth.md`](../extensions/transition-truth.md).

## System layers

| Layer | Manual source | Planned package boundary |
| --- | --- | --- |
| Source language | Chapters 3–4; `grammars/` | `taoryx.language` |
| Tables and constants | Chapter 3; `metadata/table_*.yaml` | `taoryx.tables` |
| Problem resolution | Chapter 4 data blocks | `taoryx.scenario` |
| State and reference frames | Chapter 2 §§1–2 | `taoryx.coordinates`, `taoryx.state` |
| Forces and rates | Chapter 2 §3; `metadata/acceleration_*.yaml` | `taoryx.models` |
| Integration and events | Chapter 2 §§2.1, 6; Chapter 4 `*Integ`, `*When` | `taoryx.integration`, `taoryx.events` |
| Guidance/search/optimization | Chapter 2 §§5–6; Chapter 4 `*Search`, `*Optimize`, `*Survey` | `taoryx.guidance`, `taoryx.search` |
| Outputs and reports | Chapter 2 §4; Chapter 4 `*Print`, `*Summarize` | `taoryx.outputs` |

The runtime now provides an executable, evidence-bounded subset of this
architecture: parsed `.prb`/`.tbl` files lower into runtime problems and
tables, the engine integrates trajectories, applies supported events and
controls, and renders outputs and summaries. It does not claim numerical
equivalence with historical TAOS 96.0, and unsupported or ambiguous language
shapes are diagnosed rather than silently executed.

## Batch plus live execution

The core TAORYX differentiator is that batch analysis and live control use the
same executable model. A `.prb` sequence can be parsed, validated, lowered,
and run deterministically like TAOS. The resolved runtime graph can also be
stepped from an external player, notebook, autopilot, reinforcement-learning
agent, or hardware adapter through bounded control channels. The controller
changes commands and effectors; it does not bypass the physical state model.

The execution modes are deliberately composable:

```text
source .prb/.tbl
       |
       v
parse -> validate -> lower -> RuntimeProblem
                              |
              +---------------+----------------+
              |                                |
              v                                v
       batch integration                 InteractiveSession
       fixed command plan                pause/step/commands
              |                                |
              +---------------+----------------+
                              v
                    telemetry / replay artifact
                              |
                              v
                    clone_at(time) -> branch
```

This supports controlled experiments such as comparing an open-loop batch
trajectory with a player or AI policy, interrupting at a flight event, changing
control laws, and continuing from the exact or interpolated runtime state. The
interactive and branching contracts are TAORYX extensions and are not claims
of historical TAOS 96.0 behavior.

## Program and emulator boundary

`taoryx.runtime.program.LoadedProgram` is the explicit bridge between the
generic problem-file program and the runtime emulator. It retains the
validated `ProblemDocument`, source paths, table documents, lowered tables,
executable cases, controls, searches, and segment structure.

```python
from taoryx.runtime.program import LoadedProgram

program = LoadedProgram.load("mission.prb", ("aero.tbl",), profile="taoryx")
summary = program.inspect()
runtime = program.case()
branch = program.clone_case_at(12.5)
program.set_control("throttle", 0.8)
live = program.inspect_case()
```

This makes the loaded program inspectable before execution and makes copying a
running case a normal operation rather than an ad hoc serialization trick.
The same loaded source can feed batch execution, interactive control, replay,
or a cloned experiment branch. Runtime controls and parameters can be modified
through explicit APIs; source text is not silently rewritten, so a modified
emulator state can always be distinguished from a modified source program.

Runtime consumers should use the tiered observation API for live data instead
of digging through internal named state: standard flight vectors first,
declared model/status values second, and deep diagnostics only on request.

## Related catalog pages

- `metadata/algorithm_catalog/` holds the reviewed planning catalog and the
  generated implementation ledger.
- [`docs/architecture/algorithm-catalog.md`](algorithm-catalog.md) explains the
  catalog as an architecture layer, not as a second equation registry.
- [`docs/architecture/state-model.md`](state-model.md) defines the canonical
  ECFC point-mass state and its runtime/integrator boundaries.
- [`docs/architecture/vehicle-data-model.md`](vehicle-data-model.md) groups
  the successor-side geometry, mass, propulsion, and effector data families
  that sit above the current TAOS `.tbl` abstraction.
- [`docs/architecture/vehicle-interface-contract.md`](vehicle-interface-contract.md)
  defines the resolved vehicle-facing parameter, action, status, resource,
  truth, and observation contract used by composition and interactive
  execution.
- [`docs/architecture/telemetry.md`](telemetry.md) defines the structured
  runtime artifact consumed by reports and visualization backends.
- [`docs/architecture/table-explorer.md`](table-explorer.md) explains the
  renderer-independent table inspection and plotting boundary.
- [`docs/architecture/interactive-engine.md`](interactive-engine.md) defines
  deterministic external stepping, command routing, replay, and artifacts.
- [`docs/architecture/deployment-and-spawning.md`](deployment-and-spawning.md)
  defines generic vehicle spawning and its aero-ballistic specializations.
- [`docs/extensions/README.md`](../extensions/README.md) defines the claim
  boundary and documentation contract for TAORYX-only capabilities.
- [`examples/showcases/README.md`](../../examples/showcases/README.md) lists
  reproducible demonstrations and their plot contracts.
- [`../../analysis/tumbling/README.md`](../../analysis/tumbling/README.md)
  collects the current aerodynamic-analysis workspace and generated studies.

## Execution shape

```text
.prb/.tbl files
      │
      ▼
parse → validate → resolve scenario
                        │
                        ▼
              state + model inputs
                        │
                        ▼
             derivatives + events
                        │
                        ▼
             integration / segments
                        │
                        ▼
              outputs / search results
```

## Implementation order

1. Extend typed parser defaults and cross-block validation.
2. Complete remaining unit-dimension projections and parser/runtime semantic
   coverage while preserving the lossless source boundary.
3. Expand trajectory-level regression baselines and resolve large synthetic
   optimization performance.
4. Establish historical comparison evidence when a trusted TAOS 96.0
   executable or output corpus is available.

Every implemented equation should link back to its canonical ID in
`metadata/equations.csv`; implementation coverage is separate from manual
transcription coverage.
- [Environment providers](environment-providers.md)
