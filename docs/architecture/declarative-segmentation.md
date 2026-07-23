# Declarative segmentation

TAORYX uses an external segmentation catalog to describe reusable mission
orchestration. The catalog is deliberately not a new TAOS directive. It is
compiled into ordinary `.prb` files using existing `*segment`, `*when`, and
`goto`/`stop` syntax, while the resolved manifest and transition audit retain
the richer engineering metadata.

## What belongs in the catalog

Each scenario declares its vehicle family, source problem, dynamics mode, and
ordered segments. A segment records:

- entry and exit conditions;
- the controller and actuator binding;
- table and environment dependencies;
- the transition policy for position, velocity, attitude, rates, and mass;
- the source segment body from which native problem syntax is copied.

Segments may also carry a typed `goal`. A goal is an acceptance contract for
the evidence runner, not a controller command. The common goal kinds are
`trim_hold`, `hover`, `altitude_capture`, `heading_capture`, `waypoint`,
`racetrack`, `separation`, and `ground_contact`. Targets and tolerances use
canonical channel names such as `altitude_m`, `speed_m_s`, and `heading_deg`.
This makes the expected behavior explicit while allowing each vehicle adapter
to provide its own telemetry mapping.

Scenarios may carry typed `events` for orchestration metadata. Events describe
conditions such as `stage_burnout`, `separation`, `waypoint_reached`, or
`envelope_exit`, plus an optional `goto`/`stop` action and transition policy.
Declared impulses and mass changes must be explicit and are audited; ordinary
segment handoffs remain continuous. Event metadata never becomes a new
historical TAOS directive.

`point_mass_3dof`, `kinematic_3_plus_3`, and `rigid_body_6dof` are orchestration
labels. The current compiler does not silently claim that a historical TAOS
runtime supports a new mode directive. The source `.prb` remains the syntax
authority; the manifest says which TAORYX execution tier is intended.

State declarations are also external: `initial_state` and `inherited_from` are
recorded in the resolved manifest. Declared transition values lower through
existing native `*reset` and `*increment` blocks. Continuous and inherited
transitions emit no synthetic state jump; reset, impulse, and mass-change
transitions require explicit values and are recorded in the transition audit.
Source `*when` clauses are preserved additively.

## Build segments without hand-writing the catalog

Most authors should start with the composition layer in
taoryx.composition, not with SegmentSpec or YAML. It provides a small
reviewed registry and a sequential builder that assigns time windows, links
segment exits, validates typed goals, and terminates the final segment with
stop.

~~~python
from pathlib import Path

from taoryx.composition import TrajectoryBuilder, WaypointSpec

builder = TrajectoryBuilder(
    "x8-demo-course",
    vehicle="skywalker_x8",
    family="fixed-wing-uav",
    source_problem="examples/mission.prb",
    runtime_tables=("tables/x8.tbl",),
    mode="rigid_body_6dof",
)
builder.use(
    "trim_hold",
    "trim",
    duration_s=20.0,
    target={"speed_m_s": 18.0, "altitude_m": 100.0},
    tolerance={"speed_m_s": 1.0, "altitude_m": 5.0},
    controller="x8-trim-hold",
    actuator_binding="x8-elevons",
)
builder.waypoint_course(
    (
        WaypointSpec(
            id="north",
            target={"north_m": 100.0, "altitude_m": 100.0},
            tolerance={"north_m": 15.0, "altitude_m": 5.0},
            duration_s=30.0,
            dwell_time_s=5.0,
            controller="x8-recovery",
            actuator_binding="x8-elevons",
        ),
        WaypointSpec(
            id="east",
            target={"east_m": 100.0, "altitude_m": 100.0},
            tolerance={"east_m": 15.0, "altitude_m": 5.0},
            duration_s=30.0,
            controller="x8-recovery",
            actuator_binding="x8-elevons",
        ),
    )
)

scenario = builder.build()
problem, manifest, audit = builder.compile(Path("repo-root"))
~~~

The builder produces an ordinary SegmentationScenario, so the existing
compiler, manifest, transition audit, runtime, plots, and validation ladder
remain the execution path. Use builder.evaluate() or
evaluate_composition(scenario) to inspect warnings before compilation.
Warnings such as an omitted controller or actuator binding are visible; failed
capture goals, invalid transitions, duplicate IDs, and non-terminating graphs
are rejected.

Agents can discover the built-in vocabulary without reading implementation
details:

~~~python
from taoryx.composition import SegmentCompositionRegistry

registry = SegmentCompositionRegistry.standard()
print(registry.names())
print(registry.describe("waypoint").description)
~~~

The built-ins are trim_hold, hover, waypoint, altitude_capture, and
heading_capture. A project-specific reusable component can be added with
registry.register(...); custom factories still return the validated
SegmentSpec, and therefore cannot bypass the graph and goal checks.

### Evaluate a recorded run

`evaluate_composition(scenario)` is a pre-run structural check. After a run,
use `evaluate_composition_runtime(scenario, artifact)` to evaluate the
evidence in the normalized `RunArtifact`:

~~~python
from taoryx.composition import RuntimeEvaluationOptions, evaluate_composition_runtime

runtime_report = evaluate_composition_runtime(
    scenario,
    artifact,
    options=RuntimeEvaluationOptions(
        # Keep author-friendly goal names separate from emitted channels.
        channel_aliases={"altitude_m": "position.altitude.geodetic"},
        # Optional numeric handoff gates. Keys may be semantic channels or
        # source names; they are checked against the segment transition policy.
        transition_tolerances={"mass.total": 1.0e-6},
        max_saturation_fraction=0.05,
    ),
)
runtime_report.raise_for_failure()
~~~

The report evaluates each segment in order:

- entry checks compare declared `initial_state` values with the first sample;
- goal checks require every target channel to be within tolerance at exit and
  require the declared trailing dwell time;
- transition checks look for the recorded segment event and, when tolerances
  are supplied, audit the declared continuity policy at the handoff;
- exit checks verify the next segment or terminal coverage; and
- quality checks require finite goal/entry telemetry and optionally gate
  actuator saturation.

`pass` means the checks ran and passed. `warning` means the required evidence
passed but optional evidence was not emitted, such as a typed stop reason or a
saturation channel. `fail` means a recorded value violated a declared
contract. `blocked` means the evidence needed to decide is absent, most often
because a goal channel was not emitted or a non-time segment has no runtime
span. A blocked result must not be converted into a pass by supplying a
vehicle-specific guess. If the runtime did not emit `SegmentSpan` records,
standard time-window segments can be reconstructed with a warning; use
`require_explicit_segment_spans=True` when that inference is not acceptable.

Goal keys should normally be canonical output names such as
`position.altitude.geodetic`. If an authoring layer uses `altitude_m`, `north_m`,
or another local name, provide an explicit `channel_aliases` mapping for that
scenario. This is the firewall between composable authoring vocabulary and
vehicle-specific telemetry.

### Promotion boundary

The lifecycle catalog is not itself a promotion certificate. Before a segment
is reused in a route, validate the corresponding row in
`verification/segment_promotion.yaml`. The row must match the segmentation
scenario and goal kind and must name the focused evidence test. Feed the
runtime report into `taoryx.segment_promotion.evaluate_promotion_catalog` and
require `report.composition_ready` before route-level verification. This
separates the questions cleanly:

1. Does the controller run?
2. Does this vehicle/segment pair enter correctly, achieve its own goal, hand
   off correctly, and terminate with usable evidence?
3. Only then, does the composed route work?

The current matrix deliberately exposes coverage gaps rather than declaring
every goal kind available for every vehicle. `waypoint`, `racetrack`, and
other segment types still need concrete vehicle rows and focused runtime
evidence before they can be promoted.

## Workflow

```bash
./.venv/bin/python tools/compile_segments.py lint
./.venv/bin/python tools/compile_segments.py build
./.venv/bin/python tools/compile_segments.py \
  --catalog verification/segmentation_catalog.yaml
```

Compiled scenarios can be exercised through the standard runtime with explicit
table inputs:

```bash
./.venv/bin/python tools/compile_segments.py run \
  --table path/to/vehicle.tbl --max-steps 10000
```

`run` is a thin dispatcher over `taoryx.runtime.runner.run_files`; it does not
create a family-specific runner. Missing tables and runtime diagnostics remain
per-scenario failures.

For every scenario the compiler writes:

- a generated native `.prb` input;
- a resolved manifest with source hash, mode, bindings, and segment records;
- a transition audit showing every edge and continuity policy.
- the resolved goals and typed events in both the manifest and audit.

The compiler rejects duplicate IDs, missing targets, cycles, invalid stop/goto
forms, missing source segments, and source segments without an integration
block. A later extension may add explicitly declared loops, but accidental
cycles remain errors because they make bounded evidence and controller resets
ambiguous.

## Controller lifecycle

`taoryx.control.SegmentController` owns activation. It calls `reset` once at
the first segment and once at each transition. Controllers may additionally
implement `on_transition(SegmentTransition)` to clear integrators, update
targets, or record an event. Allocation remains a separate family-specific
adapter. This keeps lifecycle behavior reusable for the B747, X8,
Hummingbird, X-15, point-mass, and future vehicles.

## Claim boundary

Compilation proves metadata and syntax composition, not trajectory validity.
Each compiled scenario still needs the normal plant firewall, source
differential checks, closure, convergence, envelope, and controller evidence.
The generated artifacts are ignored evidence products; the YAML catalog and
source problem files are the reviewable inputs.

The recommended workflow for a new vehicle is therefore: add a catalog entry,
declare the family-specific source problem, tables, controller and actuator
bindings, then add goals and events before writing a new runner. A new vehicle
should not require a new problem-file dialect merely to express a waypoint,
stage boundary, air-launch release, or ground-contact criterion.
