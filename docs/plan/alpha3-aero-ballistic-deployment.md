# Alpha 3 Aero-Ballistic Deployment Plan

**Status:** Alpha 3 qualification complete for the passive aero-ballistic tranche
**Scope:** the aero-ballistic specialization of generic deployment: staged
separation, spawned ballistic bodies, and passive/tumbling deployable-body
reachability

The runtime contract can eventually spawn any `RuntimeVehicle`, including a
propulsive, guided, sensing, drone, spacecraft, or other active child. Alpha 3
deliberately implements only the easy, high-value specialization: passive
aero-ballistic children such as spent stages and tanks. Active arbitrary-child
configuration and higher-level builders are Alpha 4 work.

The compiled documentation catalog is maintained in
`manual/chapters/05_extensions.tex`; the governing tumbling mathematics is in
`manual/chapters/chapter02/02_07_ballistic_tumbling.tex`.

## Objective

Make deployment a first-class, time-stepped vehicle operation rather than a
mass subtraction hidden inside a reachability adapter. A separation may:

- remove dry hardware and residual propellant from the parent;
- apply a body-frame or inertial separation impulse to the retained parent;
- create a child ballistic body with its own mass, geometry, inertia, attitude,
  and aerodynamic model; and
- continue the parent and child on separate trajectories with shared event
  provenance.

The first proof cases are:

1. X-15-style spent rocket stage, represented as a cylindrical passive tumbler.
2. Spent fighter tank, represented as an ellipsoid passive tumbler.
3. A no-kick separation, proving that deployment does not invent momentum.

Alpha 3 will close on four canonical detached-body profiles:

1. sphere, for the orientation-independent baseline;
2. cylinder, for a spent rocket stage or fighter store;
3. cone, for a pointed aero-ballistic body; and
4. triaxial ellipsoid, for a parameterized elliptic tank or rounded store.

The two-axis spheroid constructor remains a compatible convenience profile;
the release qualification uses the more general triaxial ellipsoid contract.

## Contract Layers

### Shared vehicle model

The canonical shared definitions live in `taoryx.vehicle`:

- `StageMassDefinition`: dry mass, modeled propellant, flow closure, source
  declared propellant, and propulsion authority.
- `StagedVehicleDefinition`: retained core, attached stages, and separation
  events.
- `StageSeparationEvent`: event time, mechanism, residual policy, impulse frame,
  impulse, and optional spawned body.
- `DetachedBodyDefinition`: mass, shape, dimensions, reference area, inertia,
  tumbling policy, center of mass, and center of pressure.
- `PropulsionCapabilities`: liquid/solid/hybrid/unknown behavior, throttle
  range, cutoff capability, and cutoff delay.

The reachability names remain compatibility aliases only. They must not grow a
second staging schema. Reachability itself is not a staging feature: a vehicle
may run an envelope with no separation event, no detached body, and no child
model. Staging and deployment are an optional composition layer.

### Source language and lowering

The source language remains lossless and source-located. It may express:

- stage and propellant declarations;
- separation or jettison timing;
- separation mechanism and impulse;
- child body geometry and mass properties; and
- requested cutoff/throttle actions.

Lowering resolves these declarations into the shared vehicle contract. Source
contradictions, such as declared propellant not matching flow over burn time,
are diagnostics or explicit closure records, never silent corrections.

### Runtime event contract

Deployment is an accepted-boundary event, not an RK substage. Every accepted
deployment records:

- event ID and accepted time;
- parent pre-event state;
- parent post-event state;
- mass removed and residual propellant policy;
- impulse, frame, mechanism, and optional energy;
- child ID and child initial state, if spawned; and
- source/resolution provenance.

Position, velocity, attitude, and rate discontinuities are legal only when
declared by this event contract. A body-frame impulse requires a valid parent
attitude. Separation energy alone does not determine delta-v until an impulse
partition model is declared.

## Detached Body Tiers

### Point-mass 3DOF

Required fields:

- mass;
- reference area or orientation-dependent projected-area model;
- `CD` or `CD(M)`;
- initial position and velocity;
- parent-to-child frame transform; and
- terminal/impact criteria.

This tier is sufficient for ballistic footprint and reachability screening.

### Pseudo-6DOF

Adds prescribed or filtered attitude and tumble policy:

- fixed attitude;
- prescribed spin;
- passive tumble surrogate; or
- bounded/stochastic tumble ensemble.

The result must retain orientation, angular-rate, projected-area, and
orientation-dependent drag samples for plotting and uncertainty analysis.

### Rigid-body 6DOF

Adds full inertia, aerodynamic moments, center-of-pressure behavior, rate
damping, and coupled parent/child impulse bookkeeping. Alpha 3 implements this
as a clearly labeled native rigid-body child tier. The X-15 parent remains a
reduced-order surrogate; this tier does not claim a historically validated
native X-15 aerodynamic database.

## Time-Stepping Semantics

The runtime owns an ordered active-model collection. A model may request zero
or more new models during an accepted event, so parent/child propagation is a
normal collection operation rather than a special two-body code path. Each
model has a stable model ID, parent model ID, lifecycle state, fidelity tier,
and termination record.

The runtime must split an integration step at every deployment boundary:

1. Propagate the parent to the accepted event time.
2. Record the parent pre-event state.
3. Apply mass/ejection and declared impulse discontinuities.
4. Initialize the child from the declared frame transform and relative impulse.
5. Record parent post-event and child initial states.
6. Resume all active bodies from the same accepted time.

Events at the same time use stable declared ordering. Rejected solver trials,
finite-difference probes, and interpolation points never create deployments.
Parent and child histories must share the event ID and accepted timestamp.

The spawn/ejection protocol is:

1. A model emits a typed deployment request during derivative evaluation or
   control processing; the request is deferred and does not mutate the active
   collection.
2. The integrator accepts the boundary and validates the request against the
   parent state, mass budget, frame, mechanism, impulse, child geometry, and
   fidelity requirements.
3. The runtime commits one event transaction: parent state/mass update,
   ejection accounting, child initialization, active-model insertion, and
   event-record publication either all succeed or the deployment is classified
   as failed without a partial state mutation.
4. Parent and child models resume from the same accepted time and are stepped
   through the same scheduler, with independent termination criteria.

Invalid requests must be classified explicitly as deployment failures. They
must not silently become mass-only changes, implicit impulses, or untracked
child models.

## Reachability Composition

Reachability and deployment have separate ownership:

- Reachability owns search-space declaration, propagation, terminal criteria,
  candidate classification, timeout refinement, and envelope visualization.
- Deployment owns accepted-boundary events, mass/impulse bookkeeping, child
  initialization, active-model insertion, and deployment diagnostics.
- An envelope may consume only the Reachability capability, or compose it with
  Deployment when parent/child outcomes are part of the study.

The envelope result format supports a primary parent result plus optional child
result sets:

- parent-only reachability remains the fast default;
- `spawn_children=true` enables the optional deployment composition for models
  that declare a detached body;
- `spawn_children=false` is a valid standalone reachability result and must not
  emit deployment assumptions or child artifacts;
- each child has its own fidelity, termination, impact criteria, and plots;
- parent mass accounting includes the child/ejected mass exactly once; and
- summary reports distinguish parent success, child success, deployment
  failure, timeout, and impact failure.

The first composed deployment envelope should compare:

- no-kick parent-only baseline;
- parent with separation kick and no child propagation;
- parent plus cylindrical spent-stage child; and
- parent plus spheroidal tank child.

## Implementation Phases

### A3-D0 — Shared contract closure

- Stabilize the shared stage, capability, separation, and detached-body types.
- Add canonical serialization and schema fingerprints.
- Add source/provenance fields and closure diagnostics.

Exit: invalid mass closure, solid throttle/cutoff, missing tumble inertia,
detached mass mismatch, and invalid impulse frame all fail before integration.

### A3-D1 — Single-parent event dispatcher

- Add accepted-boundary separation handling to the common runtime.
- Replace parent-specific branching with an ordered active-model collection and
  deferred spawn-request queue.
- Preserve pre/post states and event records in all artifacts.
- Implement parent-only impulse and mass discontinuity first.

Exit: fixed-step and refined-step runs agree at the event boundary and no
unannounced state jumps remain; invalid requests leave no partial mutations.

Implementation note: the generic runtime now provides `SpawnRequest`, stable
model and parent IDs, an ordered active-model collection, deferred spawn
providers, atomic multi-child registration, and explicit deployment-failure
records. These records reuse the existing transition truth and event artifact
path.

### A3-D2 — Child spawn and ballistic 3DOF

- Add typed child identity and initialization mapping.
- Propagate sphere, cylinder, spheroid, and cone children independently.
- Add parent/child output identity, termination, and impact accounting.

Exit: all four canonical profiles, including the X-15 spent-stage and
fighter-tank examples, reproduce deterministic parent and child artifacts from
one source configuration.

Implementation note: the reduced reachability workbench currently propagates a
typed detached child as a 3DOF or fixed-attitude pseudo-6DOF witness when
`spawn_children=true`. The X-15 example declares a cylindrical spent-booster
body and emits child trajectories and deployment records in its JSON artifact.

Implementation note: `rigid_body_6dof` now maps the accepted release state into
the native `RigidBody6DofState` and propagates it with `RigidBody6DofModel`.
The ballistic force/moment provider uses shape-dependent projected area,
center-of-pressure torque, and explicit unresolved-rate damping. Coarse outer
envelope steps are internally substepped for rotational stability, and every
deployment records a momentum residual for validation.

### A3-D3 — Tumbling and pseudo-6DOF

- Add fixed, prescribed-spin, and passive-tumble policies.
- Use projected-area models for sphere, cylinder, spheroid, and cone.
- Record angular state and projected area at every accepted sample.

Exit: timestep refinement bounds endpoint, mass, angular-rate, and integrated
drag-area differences under declared tolerances.

### A3-D4 — Language and visualization promotion

- Lower source deployment declarations into the shared model.
- Add event timeline, parent/child trajectories, projected-area, and ballistic
  footprint plots.
- Add deployment diagnostics to JSON, SQLite, text, and plot manifests.

Exit: source, resolved scenario, runtime events, telemetry, and plots identify
the same deployment and child identities.

### A3-D5 — Native rigid-body child promotion

- Map separation impulse and parent attitude into native rigid-body child state.
- Propagate quaternion attitude and body rates through the shared native model.
- Record force, moment, inertia, projected area, and conservation diagnostics.
- Add the rigid-body X-15 tier to the same search space and plot bundle.

Exit: rigid-body child trajectories pass quaternion, momentum-residual, outcome,
and timestep-refinement tests, while artifacts preserve the reduced-parent and
native-child fidelity boundary.

### Required visualization artifact set

Every deployment-envelope case must emit a manifest that points to these
artifacts, using stable case, body, and event identifiers:

- `trajectory_parent.png`: parent ground track and altitude/time view, with the
  deployment marker and terminal outcome;
- `trajectory_children.png`: one trace per spawned body, or an explicit
  `no-child` record for parent-only cases;
- `event_timeline.png`: accepted event times, parent mass before/after, impulse,
  mechanism, and child identity;
- `capability_footprint.png`: successful and unsuccessful terminal points,
  separated by outcome such as impact, timeout, constraint failure, or
  deployment failure;
- `exploration_coverage.png`: sampled search-space occupancy and uncovered
  bins/cells, so sparse envelopes are not mistaken for vehicle limits;
- `fidelity_comparison.png`: the same initial condition across 3DOF,
  pseudo-6DOF, and any promoted 6DOF tier, with endpoint and outcome deltas;
- `telemetry_parent.csv` and one `telemetry_<child-id>.csv` per spawned body,
  containing time, position, velocity, mass, attitude/rate where applicable,
  projected area, aerodynamic loads, event ID, and termination state.

The manifest must also record the source configuration hash, resolved model
hash, search-space definition, solver settings, fidelity tier, random seed,
plot-generation version, and artifact paths. A visualization run fails the
release gate if any required artifact is missing, if a child has no terminal
classification, or if event markers cannot be joined to the runtime event
records by ID and accepted time.

## Non-Goals

- Do not infer impulse from separation energy without a partition model.
- Do not claim arbitrary solid motors are throttleable or cutoff-capable.
- Do not make the reduced reachability solver pretend to support arbitrary
  multi-stage stacks before its event dispatcher does.
- Do not make a reachability provider declare staging, ejection, or child models
  merely to participate in an envelope study.
- Do not promote a child 3DOF tumble result to rigid-body aerodynamic validity.
- Do not bury child deployment in vehicle-specific X-15 code.

## Alpha 3 Release Gate

Alpha 3 aero-ballistic deployment is complete when the four canonical passive
profiles, including the X-15 cylindrical stage and triaxial ellipsoid tank examples,
pass shared contract validation, deterministic event-boundary tests,
active-model spawn/ejection processing, parent/child 3DOF propagation,
pseudo-6DOF tumble evidence, native rigid-body child promotion, complete
artifact accounting, seeded uncertainty sweeps, physical terminal-footprint
distributions, and Matplotlib visualization. Historical X-15
aerodynamic validation and arbitrary active-child deployment remain outside
this release gate.

The canonical qualification command is
`python -m tools.dev qualify-passive-deployment`. It writes
`artifacts/verification/alpha3-passive-deployment.json` and the associated
`artifacts/deployment/passive_ballistic` manifest, nominal tier artifacts,
CSV samples, footprint distributions, and deployment plot bundles. Its
footprints are physical child outcomes only; they do not promote the parent
vehicle to a reachability or controller capability claim.

The Alpha 3 user path should be straightforward: declare the detached-body
shape, mass, inertia, impulse/ejection policy, and passive aerodynamic model;
run the staged case; and receive parent/child trajectories, event diagnostics,
terminal outcomes, and plots without writing a vehicle-specific spawn loop.

## Alpha 4 Deployment Follow-On

Alpha 4 should revisit the generic side of deployment with explicit design
work for active children. It should settle how child propulsion, controls,
guidance, sensors, dependencies, resource ownership, and vehicle-specific
initialization are declared and validated before promoting them to a first-
class user-facing API. The Alpha 3 `RuntimeVehicle`/`SpawnRequest` contract is
the lower-level seam, not the final authoring format.
