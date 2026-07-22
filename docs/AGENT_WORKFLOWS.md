# Agent workflows

This is the shortest route from a new task to a traceable TAORYX change. Read
this page first, then follow the detailed architecture page for the workflow.

## First orientation

```text
.tbl/.prb source -> parse/validate -> lower -> runtime model -> artifact/plots
                                      ^                  ^
                              composition patches   controls/controllers
```

- The language layer preserves source text and reports diagnostics. Parsing
  successfully does not imply executable runtime coverage.
- The composition layer resolves typed overrides and scenario metadata. It is
  a TAORYX extension, not historical TAOS syntax.
- The runtime integrates the lowered model in batch or through an external
  timestep loop.
- Artifacts are the common output boundary for telemetry, events, replay,
  reports, and plots.

## Choose the workflow

| Task | Start here | Primary command/API |
| --- | --- | --- |
| Validate grammar | [Grammar guide](grammar/README.md) | `taoryx-validate file.prb file.tbl` |
| Add reusable segments | [Segmentation](architecture/declarative-segmentation.md) | `python tools/dev.py segment-lint` |
| Apply typed scenario changes | [Scenario runtime](architecture/README.md) | `ScenarioCompiler`, `ScenarioRequest` |
| Run a trajectory | [Runtime architecture](architecture/README.md) | `run_files(...)` or `LoadedProgram` |
| Drive timesteps | [Interactive engine](architecture/interactive-engine.md) | `InteractiveSession.step(...)` |
| Build plots | [Telemetry](architecture/telemetry.md) | `RunArtifact`, `render_run_artifact_plots(...)` |
| Add control above trim | [Control contracts](architecture/control-contracts.md) and [LQR](extensions/lqr.md) | `TrimSpec`, `solve_trim`, controller/allocator |

## Grammar validation

Validate source before attempting to run it:

```bash
taoryx-validate --profile taos96 path/to/file.prb path/to/file.tbl
taoryx-validate --profile taoryx path/to/extension.prb
python tools/dev.py grammar
python tools/dev.py test-grammar
```

Use `taos96` for historical-language fixtures and `taoryx` for successor
extensions. For malformed input, inspect located diagnostics and recovery
records rather than discarding source evidence. Add independent positive and
negative fixtures under `tests/fixtures/grammar_baseline/` when changing
grammar behavior.

## Segments and composition

Use native `.prb` `*segment`, `*when`, `goto`, and `stop` constructs when the
change belongs to the documented source language. Use the external
segmentation catalog when the task needs reusable orchestration metadata,
controller bindings, goals, events, or transition policies.

```bash
python tools/dev.py segment-lint
python tools/dev.py segment-build
python tools/dev.py segment-run
```

The compiler produces a generated `.prb`, resolved manifest, and transition
audit. Review the YAML catalog and source problem, not generated outputs.
Compilation proves composition and syntax, not plant or trajectory validity;
add closure, convergence, envelope, and controller tests.

For typed initialization/configuration changes, use `ScenarioCompiler` instead
of editing state tuples or source text in place:

```python
from taoryx.scenario import ParameterOverride, ScenarioCompiler

scenario = ScenarioCompiler().compile(
    "mission.prb",
    patches=(ParameterOverride("launch_altitude", 30_000.0, unit="m"),),
)
artifacts = scenario.run(output_dir="artifacts/mission")
```

Composition patches are ordered, unit-aware, recorded in resolution metadata,
and must not bypass declared control or actuator routes.

## Batch runs, timesteps, and plots

For a deterministic batch run, use the shared runner and choose the integrator
explicitly when numerical method matters:

```python
from taoryx.runtime.runner import run_files

report = run_files("mission.prb", ("vehicle.tbl",), max_steps=10_000,
                   integrator="rk4", output_dir="artifacts/mission")
```

For an external controller, player, notebook, or learning agent, use
`InteractiveSession`. Each call accepts a duration and named bounded commands;
it advances numerical time and records requested/applied commands:

```python
session = scenario.interactive_session()
snapshot = session.step(0.02, {"fin_pitch": 0.1, "throttle": 0.7})
artifact = session.to_run_artifact()
```

Render from the artifact rather than reparsing source:

```python
from taoryx.visualization import render_run_artifact_plots
render_run_artifact_plots(artifact, "artifacts/mission/plots")
```

Use standard observations for live consumers, declared status channels for
model-specific telemetry, and deep named state only for diagnostics.

The equivalent CLI routes are:

```bash
taoryx run mission.prb vehicle.tbl --profile taoryx --integrator rk4 --output-dir artifacts/mission
taoryx scenario compile mission.prb vehicle.tbl --profile taoryx --output artifacts/mission/scenario.json
taoryx artifact plot path/to/artifact.json --output-dir artifacts/mission/plots
taoryx integrators list
```

Use `python3` instead of `python` on systems where the `python` alias is not
installed. The repository's required gates are still the commands named in
`AGENTS.md`.

## Common traps

- Do not edit generated files under `build/`, `qa/`, or `artifacts/` as source.
  Change the YAML, TeX, fixture, or Python input that generates them.
- Do not use `docs/plan/` as an API reference without checking the current
  implementation. Plans may describe target APIs; current public runtime
  entry points are listed in this page and the architecture docs.
- Do not treat `parse`, `compile`, or `run` as equivalent evidence. Grammar
  validation, composition validation, numerical execution, and historical
  parity are separate claims.
- Do not mutate physical state through a controller command. Commands must
  pass through declared controls, bounds, slew limits, and allocators.
- Do not add a new `.prb` dialect for orchestration metadata. Use the
  segmentation catalog or scenario composition layer for TAORYX extensions.
- Check the existing worktree before editing. Preserve unrelated user changes
  and avoid modifying established fixtures silently.

## Controls above trim

The safe path is:

```text
plant -> TrimResult -> local A/B -> controller -> demand -> allocator -> plant
```

`plant_residual` in `solve_trim` must use the same frames, tables, mass
properties, actuators, and propulsion as propagation. After trim, use
`finite_difference_linearization` to obtain true state-derivative Jacobians;
force/moment derivatives are not automatically an `A,B` pair. Bind named
states and controls exactly to the trim artifact, then apply bounds, slew
limits, and family-specific allocation through the control contracts.

```bash
python tools/dev.py trim-vehicles
python tools/dev.py control-directions
python -m pytest -m algorithms
```

## Definition of done

1. Add or update the nearest README and machine-readable manifest.
2. State whether the result is manual-bounded, source-backed, or a TAORYX
   extension.
3. Add focused tests; keep generated artifacts under ignored `artifacts/` or
   `build/` paths.
4. Run the required repository gates from `AGENTS.md`.
5. Report unresolved grammar, numerical, provenance, or historical-runtime
   limitations explicitly.

## Further reading

- [Build and validation](BUILDING.md)
- [Test views](BUILDING_TESTS.md)
- [Extensions documentation contract](extensions/problem-file-guide.md)
- [Codex handoff](manual/CODEX_HANDOFF.md)
