# Composable scenario runtime plan

## Purpose

Make TAORYX a composable aerodynamic and trajectory kernel while preserving
`.prb` and `.tbl` as lossless, source-faithful inputs. A user must be able to
resolve a mission, make explicit unit-aware initialization/configuration
changes, run it in batch or incrementally, and inspect the same traceable result
through artifact and visualization routes.

This is successor-side functionality. Extensions use the `taoryx` grammar
profile and are not TAOS 96.0 compatibility claims without an independent
historical oracle.

## Completion target

```text
.prb + .tbl + composition request
             |
             v
parse -> validate -> resolve -> ResolvedScenario
                                  |
          +-----------------------+-----------------------+
          |                                               |
          v                                               v
   batch execution                              InteractiveSession
          |                                               |
          +-----------------------+-----------------------+
                                  v
                       RunArtifact + event trace
                                  |
                  JSON / SQLite / CSV / text / plots
```

The first vertical slice supports a point-mass mission, with the same contract
shape intended for kinematic 3+3 and rigid-body 6-DOF modes. A resolved
scenario is immutable, versioned, content-addressed, inspectable, and
deterministic for identical source inputs, overrides, seed, and numerical
configuration.

## Design decisions

1. `.prb` stays declarative. Do not turn it into a general-purpose scripting
   language to solve composition needs.
2. `ResolvedScenario` is the public boundary between source resolution and
   execution. `RuntimeProblem` remains an internal executable graph.
3. Initialization changes are typed semantic operations, never positional tuple
   edits. Frames, units, and ownership are explicit.
4. Commands affect a model only through declared control/actuator routes; they
   cannot silently overwrite physical state.
5. Events are data. Trigger, action, source location, and effect appear in the
   artifact even when an adapter delivers a callback.
6. `RunArtifact` is the common observation boundary. Visualization, reports,
   notebooks, and streaming adapters consume artifacts or a defined live
   equivalent; they do not parse `.prb` files.
7. The compact cache is an optimization, not a source of truth. It can always
   be rebuilt from source and has explicit schema/invalidation rules.

## Public contracts

### Scenario identity and compilation

Create `taoryx.scenario` with:

- `ScenarioSource`: source paths, hashes, grammar profile, parser diagnostics.
- `ScenarioRequest`: overrides, random seed, numerical options, composition
  patches, and requested output subscriptions.
- `ResolvedScenario`: frozen resolved configuration, table provenance, vehicle
  specifications, initial-state recipes, event/control/output schemas,
  diagnostics, and identity hash.
- `ScenarioCompiler`: `compile(source, request)` and
  `load_or_compile(source, request, cache)`.
- `ScenarioValidationReport`: structured diagnostics and performed checks.

Serialized scenarios include a `schema_version`, package compatibility version,
grammar profile, normalized source hashes, request payload, resolved defaults,
identity hash, and provenance. They do not serialize Python callbacks, live
table objects, or mutable runtime state.

The current cache additionally stores the validated semantic `ProblemDocument`
and `TableDocument` payloads in Pydantic JSON form. Cache hits verify the source
manifest and request identity before reusing those pre-parsed documents; source
mutation forces a rebuild. This keeps the cache compact relative to source
manual artifacts while preserving a source-rebuild path.

### Semantic composition

Implement explicit patch types:

- `ParameterOverride(name, value, unit=None)`.
- `InitialValueOverride(vehicle, field, value, unit=None)`.
- `InitialFrameState(vehicle, frame, position, velocity, attitude=None)`.
- `VelocityImpulse(vehicle, frame, delta_velocity)`.
- `MassAdjustment(vehicle, delta_mass=None, mass=None)`.
- `InheritanceOverride(vehicle, source_vehicle, source_segment)`.
- `IntegratorOverride(name, absolute_tolerance, relative_tolerance, max_step=None)`.
- `RandomSeed(seed)`.

Patches apply in declared order and emit a resolution record with prior/new
value, unit, frame, source, and reason. Conflicts are errors unless the caller
selects a policy. Body-frame impulses require valid attitude and inertial
impulses require an unambiguous inertial frame.

```python
scenario = ScenarioCompiler().compile(
    "examples/data_driven/generic_cruise.prb",
    ScenarioRequest(
        patches=(
            ParameterOverride("launch_altitude", 30_000.0, unit="m"),
            InitialFrameState.geodetic(
                vehicle="1", latitude_deg=28.5, longitude_deg=-80.6,
                altitude_m="launch_altitude", speed_mps=0.0,
            ),
            VelocityImpulse.body(vehicle="1", forward_mps=12.0),
        ),
        seed=1729,
    ),
)
```

This is the target API, not a claim that every named field is already available
in the current point-mass problem language.

### Controls, status, events, and outputs

Promote existing interactive contracts into scenario-declared schemas:

- `ControlSpec`: name, unit, default, bounds, slew, semantics (`hold`,
  `impulse`, `per_step`), owner, valid modes, actuator route.
- `StatusSpec`: semantic channel, unit, interpolation, producer, mode availability.
- `EventSpec`: stable ID, residual, direction, scope, priority, location, action.
- `EventAction`: `stop`, `transition`, `signal`, or eventually `spawn`.
- `RuntimeEvent`: accepted time, vehicles, residual, action, state change,
  and diagnostic context.
- `OutputSubscription`: channels, sampling policy, event inclusion, sink.

`ScenarioRuntimeContract` is the serializable request-side form of the control,
status, and output declarations. `ResolvedScenario.interactive_session()` maps
it to the live dataclass contracts; batch artifacts retain the same normalized
declaration under visualization metadata. Predicates and model callbacks stay
out of the cache envelope.

Implement `signal` before external callback delivery. It is recorded at an
accepted boundary, then adapters may consume it. Do not implement `spawn` until
child state, model, inheritance, and output identity are typed.

Extend `RunArtifact`, rather than creating competing results, with source
manifest, resolved-scenario identity, numerical configuration, applied patches,
command history, and structured events. Preserve telemetry/SQLite compatibility
with a schema-versioned migration.

## Implementation phases

### Phase 0: Contract audit and boundaries

Document the mapping from `ProblemDocument`, lowering, `RuntimeProblem`,
`InteractiveSession`, and `RunArtifact` to scenario-layer counterparts.

Exit criteria:

- no scenario object duplicates a parser AST or mutable runtime vehicle;
- ownership/import boundaries are documented;
- `taos96` and `taoryx` profile rules are enforced at compile entry points.

### Phase 1: Resolved scenario and deterministic serialization

Implement `taoryx.scenario`, frozen models, canonical JSON, identity hashing,
and a cache envelope. Initially lower the existing point-mass subset without
changing numerical behavior.

Exit criteria:

- identical inputs yield byte-identical normalized JSON and identity hash;
- source/table bytes, profile, seed, overrides, or numerical settings invalidate
  the cache;
- deserialization validates schema and rejects incompatible versions;
- diagnostics retain parser locations;
- a CLI route writes and inspects a scenario without executing it.

### Phase 2: Initialization composition

Implement parameter/initial-state patches, coordinate checks, velocity impulses,
mass changes, inheritance overrides, and resolution records. Start with ECFC
and geodetic point-mass states. Add body/inertial operations only with an
attitude/reference-frame provider.

Exit criteria:

- each patch is unit-aware and frame-aware;
- a failed patch leaves the source scenario unchanged and emits diagnostics;
- launch variation needs no source-text edit;
- a body-frame impulse fails clearly without attitude;
- resolution-record replay recreates the same initial state.

### Phase 3: Unified batch and interactive execution

Construct batch and interactive runtime instances from `ResolvedScenario`.
Move control schemas into resolved form and include accepted commands/snapshots
in the standard artifact.

Exit criteria:

- batch and equal-duration interactive replay match within documented tolerance;
- commands reject unknown, out-of-mode, invalid-unit, and unrouted values;
- pause/resume/interrupt do not alter numerical results;
- generic derivative models still work without `.prb` input.

### Phase 4: Event dispatcher and trajectory composition

Normalize `*when` handling into `EventSpec` and `RuntimeEvent`. Add ordered
`stop`, `transition`, and `signal`; feature-gate `spawn` until complete.

Exit criteria:

- simultaneous events have deterministic recorded ordering;
- localization covers initial, crossing, and segment-boundary cases;
- transitions retain source/target provenance;
- signals appear in JSON, SQLite, text, and live-observer routes;
- unsupported spawn requests diagnose instead of partially executing.

### Phase 5: Artifacts and visualization

Define artifact migration and stable render specifications based on semantic
channels, not source spelling. Render static plots from `RunArtifact`, provide
HTML inspection and selected-channel CSV export. Adjacent plot metadata records
scenario identity, schema version, vehicle, channels, units, sampling, and
renderer version.

Initial plots:

- trajectory ground track or frame-appropriate position;
- altitude, speed, mass, and dynamic pressure versus time;
- commanded versus applied controls;
- event and segment-transition timeline;
- model-specific panels only when required semantic channels exist.

Exit criteria:

- plots never parse source or re-evaluate expressions;
- missing data emits an omitted-panel record, not invented values;
- repeated rendering yields equivalent data and metadata;
- JSON, SQLite, CSV, text, and plots identify the same scenario and run.

### Phase 6: Examples and release gate

Add source-controlled examples. Generated output goes under ignored `artifacts/`
or `build/`; source inputs, manifests, invariants, tolerance policy, and plot
specifications remain tracked.

Required examples:

- `ballistic-launch-variants`: altitude, mass, ECFC/geodetic changes;
- `initial-kick`: frame-aware impulse plus no-attitude negative case;
- `interactive-throttle`: bounds, slew, pause/resume, replay, control plot;
- `staged-transition-signal`: transition, signal artifact, event timeline;
- `artifact-round-trip`: cache, JSON, SQLite, CSV, plot metadata;
- `mode-availability`: invalid control/status requests across all dynamics modes.

Each example requires a README, manifest, claim class, command, invariants,
tolerances, and expected plot panels. It must run cleanly in the documented venv
and must distinguish synthetic TAORYX demonstrations from historical claims.

## Validation matrix

| Layer | Validation | Evidence |
| --- | --- | --- |
| Source | Parser, semantic, grammar-profile validation | Diagnostics and fixtures |
| Resolution | JSON, hash, cache invalidation, migration | Golden payloads and mutations |
| Composition | Units, frames, ordering, conflicts, immutability | Anchors, round trips, negatives |
| Runtime | Batch/interactive equivalence, events, command routing | Replay and physical invariants |
| Models | Control/status availability by mode | 3-DOF, 3+3, 6-DOF contract tests |
| Artifacts | JSON/SQLite/CSV/text reconciliation | Cross-sink tests and provenance |
| Visualization | Channels, omitted panels, metadata | Plot-data snapshots and smoke renders |
| Examples | Clean execution and stated invariants | Pytest examples and manifests |

Use independent oracles where possible. A scenario cannot pass solely because
its expected result repeats lowering/integration logic. Use analytical motion,
conserved quantities, coordinate round trips, known event times, and mutation
tests to expose shared mistakes.

Every implementation increment runs:

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py check
python -m pytest
```

Render and inspect changed PDF or plot outputs. Add an example test to
`tools/dev.py check` only after it is deterministic and has a documented
artifact root.

## Completion definition

The plan is complete when a user can compile validated source into a reusable
`ResolvedScenario`, apply semantic launch/initialization changes, execute via
batch or interactive APIs, receive deterministic commands/events, and inspect
one identity-linked artifact through JSON, SQLite, CSV, text, and plots.
Ambiguous frames, units, profiles, controls, events, and unsupported mode
combinations fail with structured diagnostics. Each supported claim has an
example and validation evidence; historical equivalence remains out of scope.
