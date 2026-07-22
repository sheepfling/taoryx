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
