# Agent workflows

This is the shortest route from a new task to a traceable TAORYX change. Read
this page first, then follow the detailed architecture page for the workflow.

Before adding a new source-grounded plant, DAVE-ML model, OpenAP model,
NASA/NESC scenario, JSBSim aircraft, orbital source, or public-data surrogate,
follow the [model integration workflow](plan/model-integration-workflow.md).
Start with the integration record and source hashes; keep the immutable plant
separate from Taoryx actuator, controller, mission, and RL overlays. This is
the contributor-facing Tier 0–4 process for new model work.

For sensors, estimators, seekers, or RL observations, also follow the
[sensor and measurement orchestration backlog](plan/sensor-measurement-orchestration.md).
It defines the committed-truth boundary, measurement timing, multi-rate event
ordering, and truth-isolated decision ports.

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
| Add control above trim | [Controller stack](architecture/controller-stack.md), [Control contracts](architecture/control-contracts.md), and [LQR](extensions/lqr.md) | `TrimSpec`, `solve_trim`, controller/allocator |
| Add an airbreathing vehicle mission | [Mission-composition automation](plan/mission-composition-automation.md) | `compile_powered_fixed_wing_racetrack(...)` |

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

For junior-friendly corpus regeneration, use the batch command from the
repository root:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family all --execute --output artifacts/examples/all
```

This writes AST/parser reports for both profiles, attempts the TAOS96-compatible
local runtime where bundled tables permit execution, executes Taoryx full
examples, and regenerates the indexed runtime showcases. Each successful case
gets a run report, normalized artifact, telemetry CSV, and plots. A short run
may be incomplete at the step budget; that is distinct from a runtime failure.

## Segments and composition

For a normal waypoint or segment course, use the high-level composition
builder. It keeps the source problem and native compiler boundary intact while
removing most of the graph bookkeeping:

~~~python
from pathlib import Path

from taoryx.composition import TrajectoryBuilder, WaypointSpec

builder = TrajectoryBuilder(
    "demo-course",
    vehicle="skywalker_x8",
    family="fixed-wing-uav",
    source_problem="examples/mission.prb",
)
builder.use(
    "trim_hold",
    "trim",
    duration_s=20.0,
    target={"speed_m_s": 18.0},
    tolerance={"speed_m_s": 1.0},
)
builder.waypoint_course(
    (
        WaypointSpec(
            id="north",
            target={"north_m": 100.0},
            tolerance={"north_m": 15.0},
            duration_s=30.0,
        ),
        WaypointSpec(
            id="east",
            target={"east_m": 100.0},
            tolerance={"east_m": 15.0},
            duration_s=30.0,
        ),
    )
)
scenario = builder.build()
problem, manifest, audit = builder.compile(Path("repo-root"))
~~~

Use `SegmentCompositionRegistry.standard().names()` to discover the reviewed
templates: `trim_hold`, `hover`, `waypoint`, `altitude_capture`,
`heading_capture`, and `moving_target_intercept`. The last one is intentionally
more demanding than a waypoint: it requires a target reference plus explicit
LOS/closure targets and tolerances.
builder.evaluate() returns a structural report; warnings are visible and
invalid capture goals, graph edges, or termination policies fail before
compilation. Use the lower-level catalog only when a component needs custom
events, state transitions, or non-time entry/exit expressions.

Use native `.prb` `*segment`, `*when`, `goto`, and `stop` constructs when the
change belongs to the documented source language. Use the external
segmentation catalog when the task needs reusable orchestration metadata,
controller bindings, goals, events, or transition policies.

### Simple Aero-style specialized segments

For the synthetic Simple Aero corpus, start with
[`docs/architecture/simple_aero-segments.md`](architecture/simple_aero-segments.md) and
[`verification/simple_aero_segment_catalog.yaml`](../verification/simple_aero_segment_catalog.yaml).
The reusable templates are `powered_ascent`, `ballistic_coast`,
`bank_maneuver`, `alpha_profile`, `skip_maneuver`, `terminal_pronav`, and
`moving_target_intercept`. They apply to point-mass 3-DOF, kinematic
pseudo-6-DOF, and—after additional plant gates—rigid-body 6-DOF.

The Simple Aero fixture status is deliberately separate from vehicle promotion:
`fixture-ready` means the synthetic source translation and provenance are
available. It does not prove a vehicle's thrust, aero tables, bank sign, alpha
response, target closure, or terminal behavior. Reuse the phase contract and
retune the vehicle-specific controls, tables, limits, and time-to-go values;
never copy those values blindly from the Simple Aero surrogate.

Before composing a Simple Aero phase into a vehicle route, run the isolated fixture
ladder:

```bash
python tools/dev.py test-simple_aero-segments
```

This Simple Aero-specific runtime view proves grammar, completion, telemetry,
finite samples, time ordering, and isolated segment span. It does not promote
the phase: vehicle-quality gates remain deferred until a vehicle adapter
supplies bounded aero/plant, control, convergence, and terminal evidence.

For the shortest path to a runnable reduced-order case, use the parameter
builder instead of hand-writing four native segments:

```python
from taoryx.simple_aero_builder import build_fixed_ld_3dof

build = build_fixed_ld_3dof(
    vehicle_id="generic-3dof",
    vbo_m_s=900.0,
    apogee_altitude_m=20_000.0,
    pitch_over_angle_deg=75.0,
    target_range_m=100_000.0,
    target_bearing_deg=90.0,
    initial_heading_offset_deg=8.0,
    lift_to_drag=4.0,
)
build.write("build/simple_aero-demo.prb", "build/simple_aero-demo.manifest.json")
```

Validate the generated extension with `taoryx`, then run it through the same
profile explicitly:

```bash
taoryx-validate --profile taoryx build/simple_aero-demo.prb
```

The generated manifest shows the computed phase durations, initial heading,
target location, and fixed-L/D assumptions. `vbo_m_s`, apogee, pitch-over,
heading offset, and `lift_to_drag` are convenience parameters—not claims that
the resulting trajectory exactly reaches those values. For a promoted vehicle,
override phase durations as needed, bind vehicle tables/controllers, and run
the segment evidence ladder before route verification.

For the X-15, the focused follow-on fixtures are the bounded 3-DOF
`x15_phugoid_3dof.prb` alpha/energy profile and the 6-DOF
`x15_weave_6dof.prb` two-cycle bank reversal. Their machine-checked gates live
in `tests/e2e/test_glider_family_validation.py`; the current claim boundary is
segment response, not natural-mode identification, crossrange optimization, or
route promotion.

For a low-code X-15 menu, use
[`verification/x15_maneuver_catalog.yaml`](../verification/x15_maneuver_catalog.yaml):

```python
from taoryx.x15_maneuvers import load_x15_maneuver_catalog

catalog = load_x15_maneuver_catalog("verification/x15_maneuver_catalog.yaml")
settled = catalog.select("settled")
```

Use only `settled` rows as inputs to new composition work. Inspect each row's
`focused_test`, `tables`, and `quality_gates` before changing parameters. The
catalog currently settles five X-15 maneuvers; its trim-to-terminal ProNav row
is still a candidate pending terminal miss-distance evidence. This menu is a
segment-quality checkpoint, not a route-promotion or flight-certification
shortcut.

Use the smallest test slice for the change. The X-15 module marks isolated
segment tests with `segment` and their dynamics tier with `dof3` or `dof6`:

```bash
python tools/dev.py test-x15-catalog
python -m pytest tests/e2e/test_glider_family_validation.py \
  -m 'x15 and segment and dof3' -k phugoid -o addopts=''
python -m pytest tests/e2e/test_glider_family_validation.py \
  -m 'x15 and segment and dof6' -k weave -o addopts=''
```

Use `python tools/dev.py test-plots` for visualization-only checks and
`python tools/dev.py test-grammar` for parser work. Reserve
`python tools/dev.py test-x15` and the full `check`/`pytest` gates for a
promotion checkpoint or a handoff; they intentionally include much more
vehicle and artifact coverage. The complete slice map is in
[`docs/BUILDING_TESTS.md`](BUILDING_TESTS.md).

After execution, score the composed run instead of inspecting plots by eye:

```python
from taoryx.composition import RuntimeEvaluationOptions, evaluate_run

evidence = evaluate_run(
    scenario,
    artifact,
    options=RuntimeEvaluationOptions(
        channel_aliases={"altitude_m": "position.altitude.geodetic"},
        transition_tolerances={"mass.total": 1.0e-6},
        max_saturation_fraction=0.05,
    ),
)
evidence.raise_for_failure()
```

This is a segment-level validation ladder: entry state, goal capture and
dwell, transition event/continuity, exit or terminal coverage, finite required
telemetry, and optional saturation. Missing channels produce `blocked`, a
contract violation produces `fail`, and omitted optional channels produce a
visible `warning`. Use canonical semantic output names or provide explicit
aliases; the evaluator never guesses that `altitude_m` or `north_m` means a
particular vehicle channel. For non-time segments, require runtime spans so
the evaluator does not infer boundaries from source text.

### Segment promotion before route verification

Do not promote a controller directly into a route. The segment promotion
matrix at `verification/segment_promotion.yaml` is the intermediate gate. Each
row names one vehicle/scenario/segment pair, its goal kind, the evidence
categories that must pass, and the focused test that produced the evidence.
Validate its coverage against the segmentation catalog with:

```python
from taoryx.segment_promotion import (
    load_segment_promotion_catalog,
    validate_promotion_coverage,
)
from taoryx.segmentation import load_catalog

segmentation = load_catalog("verification/segmentation_catalog.yaml")
promotions = load_segment_promotion_catalog("verification/segment_promotion.yaml")
errors = validate_promotion_coverage(promotions, segmentation)
assert not errors, errors
```

After running a focused segment scenario, pass its runtime report through
`evaluate_promotion_catalog(...)`. A segment is promoted only when every
required category is present and every required check is `pass`; a missing
channel, missing event, incomplete termination, or untested quality category
remains `blocked`. The resulting evidence hash is the stamp consumed by route
review. The matrix is coverage metadata, not a manual “green” override: the
existing family tests are evidence inputs, while the promotion report is the
decision boundary.

```bash
python tools/dev.py segment-lint
python tools/dev.py segment-build
python tools/dev.py segment-run
```

The compiler produces a generated `.prb`, resolved manifest, and transition
audit. Review the YAML catalog and source problem, not generated outputs.
Compilation proves composition and syntax, not plant or trajectory validity;
add closure, convergence, envelope, and controller tests.

For moving-target guidance, compose the segment with an explicit reference and
score the native guidance channels rather than treating a route waypoint as an
intercept:

```python
builder.moving_target_intercept(
    "terminal-intercept",
    duration_s=10.0,
    reference="target-2",
    target={
        "pro_nav_los_range_m": 25.0,
        "pro_nav_closing_velocity_m_s": 0.0,
    },
    tolerance={
        "pro_nav_los_range_m": 25.0,
        "pro_nav_closing_velocity_m_s": 5.0,
    },
)

evidence = evaluate_run(
    scenario,
    artifact,
    options=RuntimeEvaluationOptions(
        required_channels=(
            "pro_nav_active",
            "pro_nav_los_range_m",
            "pro_nav_closing_velocity_m_s",
            "pro_nav_acceleration_response_residual_m_s2",
        ),
    ),
)
```

The required-channel list is a hard evidence contract. Missing or non-finite
channels produce `blocked`; aliases must be declared explicitly. Consult
`verification/segment_capability_matrix.yaml` for the current vehicle
boundary: X15 has partial source-trim-to-ProNav evidence, while Hummingbird
has a bounded focused fixture but remains a candidate until the remaining
vehicle-quality gates and promotion hash exist.

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

## Vehicle addition and the validation ladder

Treat a new vehicle as a staged evidence problem. Each stage consumes the
artifacts from the previous stage; a later green plot does not promote an
earlier blocked convention or data check.

| Stage | Question | Required evidence |
| --- | --- | --- |
| Registry/data | Can the model be discovered and loaded? | SI metadata, family contract, table bindings, source provenance/hash, generated problem profile, onboarding report |
| Grammar/lowering | Does the declared source and composition mean what the author intended? | `taoryx-validate`, segment lint/build, resolved manifest, transition audit |
| Convention firewall | Are frames, axes, table orientation, coefficient signs, and controls correct? | table inspection, in-range/boundary queries, frame/quaternion tests, signed control-direction probes |
| Plant validity | Does the model satisfy its own equations? | initial-condition audit, trim residuals, force/moment dimensionalization, independent closure, bounded propagation |
| Numerical quality | Is the result reproducible and time-step credible? | `dt`, `dt/2`, `dt/4`, adaptive comparison, event-aware exclusions, output hashes |
| Controller validity | Does the controller stabilize the local plant without violating the interface? | named `A/B` provenance, controllability, LQR poles, uncertainty screen, actuator/slew/allocation telemetry |
| Segment validity | Do entry, handoff, and exit contracts hold? | segment manifest, inherited/reset state audit, controller reset events, goal/termination evidence |
| Checkpoint mission | Does the complete scenario meet bounded objectives? | generated fidelity-ladder packet, objective gates, quality metrics, plots, claim status |

Use the following commands as the normal progression:

```bash
python tools/validate_vehicle_onboarding.py --vehicle new_vehicle --strict
python tools/dev.py generate-problems
python tools/dev.py vehicles
taoryx table inspect path/to/vehicle.tbl --html build/table-explorer.html
python tools/dev.py control-directions
python tools/dev.py trim-vehicles
python tools/dev.py segment-lint
python tools/dev.py segment-build
python tools/dev.py segment-run
python tools/dev.py fidelity-packet
python tools/dev.py maneuver-matrix
```

The onboarding validator answers whether the metadata path is complete; it
does not prove trim or mission behavior. `vehicles` checks registry and
provenance coverage, the control harness checks signed responses, and the
fidelity packet owns the multi-tier trajectory evidence. Keep the first
failing stage visible in the report instead of replacing it with a score.

### Data and convention firewall

Before tuning, make a small model packet and inspect it by hand. It should
state the state/control order, body/wind/world bases, SI conversions, reference
area/span/chord, mass/CG/inertia behavior, table axis order and bounds,
interpolation policy, coefficient source status, and actuator bounds. Query a
nominal point, every relevant boundary, and one deliberately out-of-range
point. Out-of-range behavior must be an explicit block or diagnostic; never
reverse an axis or negate a coefficient merely to make a trajectory look right.

The control-direction harness is a firewall, not a tuning test. It perturbs one
declared control around an identical baseline and records the signed body
force/moment response, antisymmetry error, expected source sign, and achieved
command. For example, the B747 elevator should produce the declared negative
body-`y` moment, the X8 collective and differential elevon probes should test
body-`y` and body-`x` independently, and the Hummingbird rotor-speed probe
should test the declared body-`z` force sign. Frame transforms and
force/moment reference transfers must be checked separately from table lookup.

### Trim, LQR, and mass-property gates

The accepted trim must identify the exact state/control ordering and carry
unscaled force and moment residuals. Linearize the state-rate evaluator at that
trim with the same tables, atmosphere, propulsion, mass, CG, and inertia used
by propagation. Record perturbation sizes, `A/B` names, `Q/R`, controllability,
closed-loop eigenvalues, table margins, and actuator/slew/allocation behavior.
Require the nominal poles to be Hurwitz and fail closed if an allowed mass,
CG, inertia, or aerodynamic uncertainty corner produces an unstable pole or a
missing table query. A mass-dependent inertia provider can update attitude
gains, but it does not replace a fresh plant-bound `A/B` design when mass also
changes translation, propulsion, CG, or aerodynamic derivatives.

### Generated trajectories and segments

The checkpoint trajectories are catalog-driven, not hand-selected after the
fact. The fidelity ladder and long-validation catalogs parameterize vehicle,
source problem/table set, duration, step factors, bounds, objectives, and
termination rules. The generated packet must include a manifest, initial
condition audit, event timeline, closure metrics, convergence report, plots,
and hashes. The usual order is source/static trim hold, 3-DOF anchor, derived
kinematic bridge, short rigid-body 6-DOF run, time-step convergence, source
differential, bounded recovery/control maneuver, then long checkpoint mission.

For each segment, validate the entry state and inherited/reset fields before
integrating; validate continuity or an explicit impulse/mass change at the
handoff; and validate the exit condition, target/action, final state, and
termination reason. The compiler rejects duplicate IDs, missing targets,
cycles, invalid `stop`/`goto` forms, missing source segments, and missing
integration blocks. The controller must reset at the first segment and each
transition. Compilation is only composition evidence; plant closure,
convergence, envelope, and controller gates still apply.

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
`finite_difference_dynamics_linearization` with a state-rate evaluator to
obtain true state-derivative Jacobians; `finite_difference_linearization` is
for residual diagnostics, and force/moment derivatives are not automatically
an `A,B` pair. Bind named states and controls exactly to the trim artifact,
then apply bounds, slew limits, and family-specific allocation through the
control contracts.

For changing mass properties, provide the rigid-body model's
`inertia_provider(state)` from the vehicle adapter. The runtime has an explicit
`linear-dry-mass` interpolation for declared reference and dry-mass inertias,
but it never infers inertia from mass. Pair that provider with
`*runtime lqr attitude update=mass` when the direct-moment attitude bridge is
appropriate. A full source-trim `A/B` design is still required when changing
mass also changes translational, propulsion, CG, or aerodynamic derivatives.

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
