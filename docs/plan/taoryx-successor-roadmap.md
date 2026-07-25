# TAORYX successor implementation roadmap

## Goal

Build a verified, evidence-bounded Python successor to TAOS that can take a
`.prb` mission and its `.tbl` data, resolve them into an executable scenario,
run a documented simulation mode, and emit reproducible state and diagnostic
artifacts.

The project must distinguish manual fidelity, semantic coherence, parser
conformance, numerical correctness, and historical compatibility. A feature
may be useful and executable without being historically compatible.

The implementation-level sequence for resolving, composing, stepping,
validating, and visualizing successor-side scenarios is maintained in
[`composable-scenario-runtime.md`](composable-scenario-runtime.md).
Checkpoint/restart is a release-level product goal tracked in
[`checkpoint-restart.md`](checkpoint-restart.md).

The first release-level definition of done is maintained in
[`taoryx-alpha-1.md`](taoryx-alpha-1.md), with its machine-readable gate
inventory in [`../../verification/alpha1_release_plan.yaml`](../../verification/alpha1_release_plan.yaml).
That plan is the umbrella for the language, runtime, composition, fidelity
ladder, vehicle evidence, and reproducible handoff work below.

The next release-level plan is
[`taoryx-alpha-2.md`](taoryx-alpha-2.md), with its machine-readable gate
inventory in [`../../verification/alpha2_release_plan.yaml`](../../verification/alpha2_release_plan.yaml).
Alpha 2 builds the catalog, immutable `ResolvedCase`, fidelity adapters, and
explicit control-authority layer on top of Alpha 1; it is the path away from
bespoke vehicle/mission runners.

Alpha 2.1 closes the bounded vehicle-variant seam: semantic modifiers,
derived-parameter provenance, hard versus qualified bounds, deterministic
reject/project policy, resource consistency, and immutable resolved-variant
fingerprints. Alpha 3 consumes that seam for new family qualification and
hybrid transitions; Alpha 4 scales it into backend-neutral parameterization,
optimization, corpus generation, and multi-backend conformance. See
[`taoryx-alpha-3.md`](taoryx-alpha-3.md) and
[`taoryx-alpha-4.md`](taoryx-alpha-4.md).

## Completion target for the next major milestone

Deliver a first integrated vertical slice:

```text
validated .prb/.tbl
        ↓
resolved simulation model
        ↓
3+3 runtime execution
        ↓
traceable state/output artifact
        ↓
independent verification scenario
```

The slice must include one documented scenario, one independently checked
numerical result, and a clean-build test path.

## Workstreams and exit criteria

### 1. Unified source-to-runtime pipeline

Connect the existing parser, validation, table resolution, problem lowering,
and runtime entrypoints into one explicit compilation route.

Required behavior:

- `.prb` and `.tbl` remain lossless source documents;
- unsupported or ambiguous syntax produces diagnostics;
- resolved defaults and table references are inspectable;
- historical syntax is not changed by taoryx extensions;
- analytical aerodynamic declarations are enabled only by an explicit profile.

Exit criteria:

- one API and CLI route compile a mission into a resolved scenario;
- the resolved scenario can be serialized and compared between runs;
- diagnostics retain source locations and recovery text.

### 2. Interactive time-steppable engine

TAORYX should also operate as a generic differential-equation state machine,
not only as a file-oriented batch runner. An interactive session must be able
to advance one controlled time interval, accept a command vector, pause, resume,
interrupt, and expose the resulting state and events.

The public boundary should look conceptually like:

```python
session = InteractiveSession(model, initial_state, control_schema)
snapshot = session.step(
    duration=0.02,
    commands={"fin_pitch": 0.15, "throttle": 0.72},
)
session.pause()
session.resume()
```

The command vector must be named and typed, not an unexplained positional
array. Each channel needs:

- name and units;
- current value and default;
- lower and upper bounds;
- rate or slew limits when applicable;
- hold, impulse, or per-step semantics;
- source/controller ownership;
- whether it is valid in 3-DOF, 3+3, or 6-DOF mode.

Commands are inputs to control laws and actuator models. They must not directly
overwrite physical state. A fin command should flow through the aerodynamic
or control model; a throttle command should flow through propulsion and mass
flow. This keeps interactive execution physically meaningful and replayable.

The session state machine should include at least:

```text
created → running ↔ paused → running
                 ├→ interrupted
                 ├→ completed
                 └→ failed
```

Each `step()` must return a snapshot containing the accepted time interval,
state, applied/clamped commands, derivatives or rates, active events, and
diagnostics. A command is applied at a defined integration boundary so replay
with the same state, command stream, and numerical configuration is
deterministic.

Required modes:

- fixed-step deterministic stepping for tests and replay;
- adaptive stepping constrained to a requested observation boundary;
- externally driven pause/resume and interrupt;
- command-stream replay from JSON/SQLite artifacts;
- optional real-time pacing as an adapter, never as the numerical clock.

Exit criteria:

- a generic scalar/vector derivative model can be stepped without `.prb` files;
- a 3+3 vehicle accepts named fin/rate/throttle commands;
- commands are bounded, unit-aware, and visible in output artifacts;
- pause/resume/interrupt behavior is deterministic and tested;
- batch execution and interactive execution share state, derivative, event, and
  artifact contracts;
- an interactive session can be driven by a future player, notebook, GUI, or
  hardware/controller adapter without changing the numerical kernel.

Checkpoint/restart is a companion release gate for this workstream. A paused
batch or interactive program must be persistable and resumable without losing
source provenance, accepted state boundaries, controls, events, or numerical
configuration. See [`checkpoint-restart.md`](checkpoint-restart.md).

### 3. Verified numerical kernel

Complete the independent verification layers for the mathematical kernel:

- coordinate frames and transformations;
- sign conventions and unit conversions;
- Earth and gravity models;
- atmosphere models;
- aerodynamic and propulsive forces;
- integration tolerances and event boundaries.

Exit criteria:

- each P0 equation or requirement has linked tests;
- anchor, round-trip, boundary, and metamorphic tests exist;
- tolerances are justified per quantity;
- known sign, transpose, unit, and coefficient mutations are detected.

### 4. Runtime state architecture

Make the state and mode boundaries explicit:

- 3-DOF point-mass translational mode;
- 3+3 mode with prescribed attitude or rate/controller state;
- separate 6-DOF rigid-body mode;
- initial conditions and inherited states;
- segment transitions, active trajectories, and event scheduling.

The 6-DOF model must not be implied by the current 3-DOF runtime or by
orientation-dependent aerodynamic tables alone.

Exit criteria:

- each mode has a typed state contract;
- mode selection is explicit and validated;
- minimal force/moment integration tests pass independently;
- segment and inherited-state behavior is tested independently from parsing.

### 5. Canonical verification scenarios

Build a small, trusted scenario ladder:

1. ballistic, no atmosphere and no aerodynamic force;
2. constant-thrust launch;
3. tabulated drag;
4. staged rocket with segment transitions;
5. prescribed tumble using an orientation-dependent table;
6. coupled planar rigid-body tumble;
7. one 6-DOF attitude case.

Each scenario needs a source manifest, expected invariants, output tolerance
policy, and a clear claim classification. Synthetic scenarios must not be
described as historical TAOS results.

Exit criteria:

- scenarios run from clean source inputs;
- outputs are reproducible;
- every asserted value links to an independent oracle or invariant;
- plot and SQLite artifacts are optional views of the same normalized run data.

### 6. Traceable outputs and analysis artifacts

Standardize the run artifact containing:

- normalized source manifest;
- resolved constants, defaults, and table provenance;
- state/time history;
- diagnostics and event trace;
- numerical configuration;
- optional CSV, SQLite, JSON, and plot views.

Exit criteria:

- a run can be inspected without rerunning it;
- generated plots identify their source run and configuration;
- artifact output stays under ignored build/artifact locations;
- the table explorer and plotters consume the normalized artifact contract.

### 7. Guidance, search, and optimization

After the basic runtime is stable, implement and verify:

- guidance-rule state machine;
- defaults, limits, and precedence;
- searches and surveys;
- optimizer adapters, including SciPy-backed choices;
- convergence, infeasibility, restart, and failure diagnostics.

Modern optimizer success must not be reported as historical `vf02ad`
compatibility without a historical oracle.

Exit criteria:

- formulation correctness is tested separately from solver behavior;
- solver selection is explicit and inspectable;
- failure modes produce structured diagnostics;
- historical compatibility claims remain disabled without evidence.

### 8. Historical compatibility layer

Only begin this work when a trustworthy historical executable, source tree, or
output corpus is available.

Required evidence may include:

- intermediate states;
- segment transition times;
- final states;
- optimization/search traces;
- output formatting and rounding behavior.

This workstream is separate from the manual-compatible implementation and must
never silently override the adjudicated or scientifically modern modes.

## Backlog: datasets, wind, and weather environments

TAORYX should support a data-generation and environment-provider layer for
wind, weather, propulsion, aerodynamics, control increments, and vehicle
properties. This is a future expansion, not a claim about historical TAOS
96.0 syntax.

### Design boundary

`.tbl` is the compiled execution format, not necessarily the authoritative
source dataset.

```text
CSV / YAML / JSON / NetCDF / GRIB / analytical models
                         ↓
                  typed DatasetSpec
                         ↓
       normalize units, conventions, grids, provenance
                         ↓
          ┌──────────────┴──────────────┐
          ↓                             ↓
 small deterministic profiles       large weather cubes
          ↓                             ↓
 generated .tbl/.prb              EnvironmentProvider
          └──────────────┬──────────────┘
                         ↓
                  TAORYX runtime
```

Small mission-specific profiles should compile into ordinary `.tbl` files.
Large four-dimensional weather products should remain external through an
`EnvironmentProvider` interface rather than becoming enormous text tables.

### What fits the current 3-DOF runtime

- thrust and mass-flow decks;
- thrust-vector direction as a translational force effect;
- axial, normal, lift, drag, side-force, and body-force tables;
- flap, spoiler, gear, and other incremental force coefficients;
- mass, center-of-gravity, and staged-vehicle data;
- standard, user, and eventually site atmosphere profiles;
- deterministic altitude/time wind profiles.

Moment coefficients, damping derivatives, inertia tensors, actuator dynamics,
and actual fin/rudder rotational response require the separate 6-DOF rigid-body
workstream. In the current 3-DOF mode, control-effect data must be trimmed or
represented as force-coefficient increments.

### Dataset compiler

Proposed package boundary:

```text
src/taoryx/datasets/models.py
src/taoryx/datasets/loaders.py
src/taoryx/datasets/compiler.py
src/taoryx/datasets/provenance.py
```

A dataset manifest must preserve:

- source hashes and source format;
- units and target unit system;
- body axes and coefficient conventions;
- wind-heading “from” convention;
- mass-flow sign convention;
- reference geometry;
- axis order and flattening order;
- interpolation and extrapolation policy;
- valid operating envelope;
- synthetic/measured/computed status;
- uncertainty and quality flags.

The compiler must reject missing rectangular grid combinations. The final
declared axis varies fastest, matching the table runtime’s documented stride
convention. Every generated table pack must round-trip source nodes through
parse, lowering, and table evaluation within a declared tolerance.

### Wind and weather correctness prerequisites

Before forecast weather is used for serious trajectories, fix and verify:

1. Complete wind-vector evaluation, including east/north/down and geodetic
   forms.
2. Meteorological wind “from” heading semantics.
3. Air-relative velocity:

   ```text
   V_air = V_earth-relative - V_wind
   ```

4. Dynamic pressure, Mach, angle of attack, sideslip, and force direction based
   on the air-relative vector—not merely its scalar magnitude.
5. One authoritative atmosphere implementation shared by batch and interactive
   runtime paths.
6. Correct site-atmosphere interpolation or an explicit unsupported diagnostic.
7. Clear out-of-domain behavior for profile and weather providers.

### Environment provider boundary

The future provider should return a typed environment sample containing at
least pressure, temperature, density, speed of sound, viscosity, and an ECFC
wind vector. Optional fields include humidity, cloud fraction, cloud liquid
water, cloud ice, and rain rate.

Candidate providers:

- standard-atmosphere provider;
- vertical-profile provider;
- route-weather provider;
- NetCDF provider;
- GRIB provider;
- composite provider combining atmosphere, wind, and weather fields.

At each derivative evaluation, the runtime queries the provider using current
simulation time and position, then derives air-relative quantities. Large
weather fields should not be masqueraded as generic output tables when they
are intended to affect physics.

### Suggested data examples

Create synthetic, openly distributable packs for:

- single-stage and two-stage rockets;
- generic subsonic cruise vehicle;
- generic ramjet vehicle;
- 3-DOF control-effect increments;
- humid vertical atmosphere;
- altitude-dependent wind profile;
- route-weather corridor.

The best first pack is a `generic_cruise_demo` containing clean and incremental
aerodynamics, thrust and mass flow, center of gravity, humid atmosphere, wind,
and one throttle-scheduled `.prb` scenario. It exercises the current 3-DOF
data path while exposing unit, block-variable, and wind-vector gaps before
6-DOF makes them harder to repair.

### Backlog exit criteria

- DatasetSpec models load at least CSV/YAML/JSON source manifests.
- A compiler emits provenance-linked `.tbl` and optional `.prb` products.
- Source-grid round trips pass structural and numeric validation.
- Wind direction and air-relative force tests cover headwind, tailwind,
  crosswind, and vertical wind.
- Humidity affects derived atmospheric properties through a documented path.
- A provider adapter can serve a bounded route corridor without requiring a
  global weather cube in `.tbl` form.
- Generated data remain explicitly synthetic or qualified according to their
  provenance; no historical TAOS compatibility is implied.

## Immediate execution order

1. Integrate analytical-table profile parsing into the unified ingestion route.
2. Add a resolved-scenario object and deterministic serialization.
3. Define and implement the generic interactive session and command schema.
4. Select and implement the first 3+3 ballistic/thrust/table-driven vertical
   slice.
5. Connect normalized telemetry, diagnostics, and artifact storage.
6. Add independent scenario oracles and mutation checks.
7. Document the result as a verified capability with explicit exclusions.
8. Expand to staged vehicles, prescribed tumble, and then 6-DOF.

## Stop-ship conditions

- analytical syntax changes historical parsing when the profile is disabled;
- a table is generated without source parameters or provenance;
- a runtime mode is inferred from an aerodynamic table rather than declared;
- an interactive command directly overwrites physical state instead of passing
  through the declared control/actuator model;
- a real-time wall clock is used as the numerical integration clock;
- a frame/sign/unit ambiguity is silently resolved;
- a scenario passes only because it shares all logic with its oracle;
- a modern optimizer result is labeled historical compatibility without an
  external oracle;
- a 6-DOF claim is made for a 3-DOF or prescribed-attitude execution.

## Current status

Completed foundations:

- parser and validator foundation for historical `.prb`/`.tbl` syntax;
- analytical aerodynamic model library and gallery generation;
- opt-in analytical-table AST, diagnostics, lowering, provenance, and fixtures;
- 3-DOF runtime and initial 3+3/6-DOF state scaffolding;
- normalized output and visualization boundaries;
- equation, manual, grammar, and test verification routes.

Alpha 2 core is complete at `A2-RELEASE-PASS`, including the provider-neutral
catalog/resolution contracts, deterministic session path, control authority,
fidelity composition, dual-launch proof, and self-contained release packet.
The next active work is the ranked P0 post-release backlog in
[`taoryx-alpha-2-backlog.md`](taoryx-alpha-2-backlog.md): common scenario
evaluation, interactive Lab/session closure, generic vehicle onboarding, and
checkpoint/replay before more family or domain expansion.
