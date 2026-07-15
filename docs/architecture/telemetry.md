# Simulation telemetry contract

TAORYX treats visualization as a consumer of simulation data, not as a parser
of human-readable reports. `*print` and `*file` remain report requests; the
structured visualization input is built directly from `ExecutionResult.states`.

The current path is:

```text
.prb/.tbl -> RuntimeProblem -> ExecutionResult -> RunArtifact -> renderers
```

`RunReport.artifacts` contains one `RunArtifact` for each lowered execution
case. A `RunArtifact` can also be written independently with
`RunArtifact.write_json(...)`.

## Human-readable and database sinks

For a quick inspection, use the deterministic text view:

```python
print(artifact.format_text(max_rows=20))
# or: artifact.print_text(max_rows=20)
```

The text view is an inspection aid, not a machine-readable interface. Plotters
and analysis tools should consume the artifact or its database representation.

The dependency-free SQLite sink preserves the complete plotter contract:

```python
artifact.write_sqlite("artifacts/run.sqlite", run_id="case-01")
```

Scenario-linked artifacts also retain the resolved scenario identity,
requested composition patches, applied resolution records, command/event metadata,
and visualization metadata. SQLite
stores these under `taoryx_run_metadata` so JSON and database views can be
reconciled without reopening source files.

For a renderer-independent HTML view:

```bash
taoryx artifact html artifacts/case-01.json --output artifacts/case-01.html \
  --channel position.altitude.geodetic
```

The HTML renderer consumes only `RunArtifact`. Requested channels that are not
present are recorded as omitted panels in its metadata rather than inferred
from source expressions.

Static PNG plots use the same artifact-only boundary:

```bash
taoryx artifact plot artifacts/case-01.json --output-dir artifacts/case-01-plots \
  --channel position.altitude.geodetic
```

The command writes one PNG per available channel and a deterministic
`plot-manifest.json` containing the scenario identity, renderer metadata, and
omitted-channel reasons.

It creates these normalized tables:

| Table | Contents |
| --- | --- |
| `taoryx_runs` | problem identity and artifact schema version |
| `taoryx_parameters` | scalar run parameters |
| `taoryx_vehicles` | vehicle family, dynamics mode, and attitude source |
| `taoryx_channels` | source names, semantic names, units, and interpolation policies |
| `taoryx_samples` | long-form time/channel/value samples for plotting and analysis |
| `taoryx_segments` | segment intervals and optional stage metadata |
| `taoryx_events` | discrete events and JSON-encoded value changes |

`run_id` allows several execution cases to share one database. Passing an
existing `sqlite3.Connection` to `artifact.dump_sqlite(...)` supports a caller-
owned database or a SQLAlchemy-managed SQLite connection without making
SQLAlchemy a runtime dependency. The `replace` option controls whether an
existing run ID is replaced or rejected by the database constraints.

The long-form sample table is intentional: new channels do not require a
schema migration, while channel metadata retains the interpolation policy that
plotters need for angles, stepwise segment values, and future quaternion or
event channels.

CSV remains backward-compatible for unlinked artifacts. Scenario-linked CSV
exports add `schema_version` and `scenario_identity` columns so every selected
sample can be reconciled with JSON, SQLite, text, and static plot outputs.

## Orthogonal vehicle and dynamics metadata

Vehicle family and dynamics are separate fields:

- `VehicleKind`: `generic`, `rocket`, `glider`, `airbreather`;
- `DynamicsKind`: `point_mass_3dof`, `kinematic_3_plus_3_dof`, `rigid_body_6dof`.

This avoids creating types such as `Rocket6DOF` or `Glider3Plus3DOF`. Runtime
vehicles default to `generic`; callers that know the vehicle family can provide
`RuntimeVehicle.vehicle_kind` when building an artifact.

## Semantic channels

Each telemetry channel retains its source variable while exposing a semantic
name and interpolation policy. For example:

| Source | Semantic channel | Policy |
| --- | --- | --- |
| `alt` | `position.altitude.geodetic` | linear |
| `x` / `xecfc` | `position.ecfc.x` | linear |
| `thrust` | `propulsion.thrust` | linear |
| `segment` | `phase.segment` | step |
| `alpha` | `aerodynamics.angle_of_attack` | angle |

Unknown variables are retained as `taos.<source-name>` instead of being
dropped. This keeps the artifact loss-aware while the semantic catalog grows.
The initial mappings are implemented in `taoryx.outputs`; the verified output
variable registry remains the provenance authority for future catalog
expansion.

## Segments and events

Contiguous segment values in runtime histories become `SegmentSpan` records.
Observed segment changes become `EventRecord` entries. This gives every chart
and animation a shared timeline without reparsing `.print` output. Richer
titles, stage grouping, geometry, and spawned-vehicle roles can be supplied by
a future optional visualization sidecar; they are not inferred from mass
alone.

## Visualization boundary

The artifact contract is intentionally backend-neutral. Static SVG/PNG plots,
interactive dashboards, animations, notebooks, and future columnar exports
should consume `RunArtifact`, not `RuntimeState` internals or report text.
Missing optional channels should be handled by profiles as unavailable panels,
not as failures of the simulation artifact itself.
