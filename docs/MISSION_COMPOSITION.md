# Mission Composition

Mission Composition is the front door for choosing a vehicle model, exposing
its configurable inputs, assembling an ordered mission, and receiving a
standard trajectory. A caller should be able to build a UI or batch planner
from the provider publication without importing provider-specific dynamics
classes.

The repository uses three functional layers:

```text
Model Authoring       defines and preserves the model
Simulation Runtime    executes and records the model
Mission Composition   discovers, configures, and assembles the mission
```

The short explanation is: Authoring defines the model. Runtime executes the
model. Composition lets users choose and assemble a mission.

## For UI, AI, and library consumers: start here

Mission Composition is the public integration boundary for a generic client.
Use it when a caller needs to discover available models, render configuration
and control forms, run a batch mission, or keep one model alive for live
control. The client consumes the provider's typed publication; it must not
import a vehicle-specific dynamics class, guess a control from its spelling,
or infer availability from a fidelity label.

```text
discover models and capabilities (no plant execution)
    -> choose model / realization / fidelity / operation
    -> render and validate typed configuration
    -> batch: run once and receive a trajectory
       or
       session: open one persistent model, inspect, step, switch an allowed
                authority profile, reset, or close
```

| Client need | Contract surface | Treat this as authoritative |
| --- | --- | --- |
| Model picker, catalogue, or capability search | `list_models()` / `taoryx model list` | Model identity, presentation metadata, exact available operations, blockers, and claim boundary |
| Configuration form or generated mission input | `get_model_schema()` / `taoryx model plan` or `scaffold` | Typed inputs, defaults, units, bounds, choices, compatible fidelity, and validation rules |
| Control-mode picker | `control_scheme_support` and the realization's authority profiles | UI order, consumer roles, ownership, streaming preference, switching policy, and which action channels belong to one mode |
| Live action form or agent policy | Open-session descriptor | The selected action schema, units, bounds, topology, action ordering, and normalization projection |
| Live status display and command acknowledgement | Step result and committed observation | Current authority, runtime action mask, requested/applied/achieved feedback, lowered action, limiting, diagnostics, and lifecycle |

The normal consumer sequence is:

1. Discover with `list_models()` or `taoryx model list`. This is safe to use
   for a catalogue: discovery must not construct or run a plant.
2. Select an exact realization, fidelity, mission, and supported operation;
   then obtain the configuration schema. A discoverable model can still state
   that a combination is blocked or batch-only.
3. Submit canonical values and validate them into a prepared, fingerprinted
   configuration. Display units and editor hints help render a client but do
   not change what the provider accepts.
4. For batch work, submit a run request. For interactive work, open a session
   and build controls from the descriptor's *selected* action schema.
5. On every live step, use the returned sequence number, respect
   `control_authority.available_action_ids`, and display
   `control_feedback`, `lowered_action`, and `lowering_evidence` rather than
   assuming that a request was physically achieved.

An authority profile is one coherent command surface, not an additive menu of
all controls advertised by a model. A UI or policy must select one profile and
send only its action channels. It may inspect inactive profiles before a
handoff, but may transfer only when both profiles explicitly allow a bumpless
transition. This is what lets a waypoint mode, a pilot-like mode, and a
low-level expert seam coexist without presenting a misleading mixed control
vector.

The client-facing metadata is transport-neutral. Taoryx currently defines the
Python/domain session protocol, not a REST, WebSocket, gamepad, authentication,
or heartbeat/deadman protocol. A front end may map keyboard, controller, or
network input into the selected action schema, but it owns transport timing,
authentication, reconnect, and safety policy. In particular, a held action for
`duration_s` is simulation semantics, not a transport-level deadman policy.

For the complete schema reference, see the
[Mission Composition Provider API](architecture/mission-composition-provider-api.md).
For the authority, lowering, status, and claim-boundary rules that every
consumer must preserve, see the
[Vehicle Interface Contract](architecture/vehicle-interface-contract.md).

## Start with discovery and configuration

The canonical Python surface is
[`taoryx.trajectory.mission_composition`](../src/taoryx/trajectory/mission_composition.py).
The portable provider contract separates self-description from execution:

```text
provider metadata
      -> list_models()
      -> get_model_schema(model_id)
      -> get_model_output_schema(model_id)
      -> validate_configuration(instance)
      -> prepared, fingerprinted configuration
```

This is the **Trajectory Provider Configuration Contract**. Provider metadata
contains provider identity and contract versions. `list_models()` returns
common model metadata without loading or running a plant.
`get_model_schema()` returns a typed tree made from `Parameter`, `Group`,
`Choice`, `Sequence`, and `Optional` nodes. A configuration instance has the
same shape and identifies the exact schema fingerprint and requested fidelity.
`get_model_output_schema()` returns the independent, fingerprinted description
of required core state, optional telemetry, and multi-entity result behavior.
Each realization returned by `list_models()` also carries its required control
advertisement: semantic actions/effectors, exact native bindings, authority
profiles, mission intents, and explicit available/internal/uncontrolled/
blocked/unsupported status.

The repository-backed compatibility implementation is
`RegistryMissionCompositionProvider`. It projects, rather than copies, the
canonical vehicle-composition, fidelity, value-space, and execution-binding
authorities selected by the aggregate catalog. It advertises all nine resolved
vehicle families, the `simple_aero` trajectory workflow, and the dual-launch
glider family. New plug-ins should instead use a focused package-owned provider
and the catalog-scoped host documented in
[Vehicle plug-in authoring](architecture/vehicle-plugin-authoring.md). A model
or realization may remain discoverable with explicit blockers; discovery is
never a promise that every mission or fidelity is executable:

```python
from taoryx.trajectory.mission_composition import (
    RegistryMissionCompositionProvider,
    render_configuration_schema,
)

provider = RegistryMissionCompositionProvider()
models = provider.list_models()
schema = provider.get_model_schema("hummingbird")
output_schema = provider.get_model_output_schema("hummingbird")
print(render_configuration_schema(schema))
```

For the checked-in families, segment sequences are closed over the exact
advertised mission templates. The analytical ballistic and waypoint examples
use the same grammar but intentionally publish open, variable-length segment
sequences.

Users and agents do not need to instantiate the schema node classes merely to
start a mission. The common authoring CLI turns these same advertisements into
plain YAML and validates the result through the exact provider:

```bash
taoryx model list
taoryx model plan <provider-id> <model-id>
taoryx model scaffold <provider-id> <model-id> --output mission.yaml
taoryx model compile mission.yaml --output prepared.json
```

Controlled plug-ins may also register model-owned inputs for the common
automatic-tuning runner exposed by `taoryx model tune`. See
[Model-to-mission authoring and automation](architecture/model-authoring-automation.md)
for the host/plug-in boundary and the concise Python API.

## Common model metadata

Every model advertisement separates authoritative semantics from optional
presentation hints. A generic consumer receives the same categories from every
provider:

| Metadata | What the provider publishes |
| --- | --- |
| Identity | Stable model/version/family IDs, model kind, status, tags, provenance, and claim boundary |
| Presentation | Display and short names, summary, category, sort key, badges, and default selections |
| Read-only properties | Typed model facts with semantic role, value or range, quantity, canonical/display units, format, grouping, ordering, and evidence boundary |
| Configuration | Typed input tree with units, intervals, defaults, topology, fidelity compatibility, and non-authoritative editor hints |
| Realization and fidelity | Dynamics fidelity, input/control realization, actuator type, compatibility aliases, operation support, blockers, and explicit step-up/step-down transitions |
| Controls | Per-realization semantic actions and effectors, native type/shape/unit/bounds/topology bindings, authority profiles, mission-intent resolution, operation availability, and claim boundaries |
| Missions and deployments | Ordered templates, exact operation matrices, and parent/child lifecycle contracts |
| Coordinates and outputs | Reference-frame definitions, required core channels, selectable telemetry groups, entity-output behavior, units, and interpolation rules |

Canonical units are the transport and execution values. `display_unit` and
numeric formatting are presentation requests; a front end may convert and
format them without changing the submitted canonical value. Likewise, a
`slider` or `coordinate_picker` hint does not relax bounds or change a value's
topology. Output availability is explicit: `guaranteed`, `conditional`, or
`runtime_reported`.

## Ask separately what can be configured, controlled, and returned

Mission Composition publishes two schemas around the execution call:

```text
                         MODEL REGISTRY
                              │
            ┌───────────┼───────────┐
            ▼           ▼           ▼
 ConfigurationSchema   Controls     OutputSchema
    what can I set?   how is intent   what can I
                     realized?       request?
            └───────────┼───────────┘
                              ▼
                   generate(config, outputs)
                              │
                              ▼
                       MissionResult
                 ┌────────────┴────────────┐
                 ▼                         ▼
             root entity            spawned entities
          core + telemetry        core + telemetry + lineage
```

The output schema keeps canonical kinematics separate from model-specific
telemetry. Optional channels may describe control surfaces, actuator state,
throttle, fuel and burn rate, propulsion, aerodynamic quantities,
guidance/control outputs, stage state, or provider-specific internals. Each
channel retains identity, primitive data type, scalar/array shape, quantity,
canonical/display units, frame, sampling semantics, interpolation,
fidelity/realization/mission/operation compatibility, availability,
provenance, and presentation hints. Related telemetry can be requested through
named groups.

Control metadata is neither mutable configuration nor result telemetry. A
semantic control channel describes the provider-neutral meaning. Its native
binding separately describes the exact action coordinate accepted by a live
session, including data type, shape, quantity, canonical unit, interval, and
value-space topology. Internally generated batch commands and planned source
effectors use the same structure but cannot advertise an interactive
operation. This lets one generic composer render every model without treating
`guidance.bank.command`, an X8 elevon coordinate, and a direct body wrench as
the same control realization.

`MissionCompositionOutputSelection` supports:

- `mode="core"`: required core state only;
- `mode="selected"`: core plus explicit channels or telemetry groups; and
- `mode="all"`: core plus every applicable advertised telemetry channel.

Time remains a structural field on every sample. Core state cannot be removed
by a telemetry selection. Spawn relationships and the lifecycle events needed
to interpret them are also structural rather than optional telemetry. Entity
metadata states whether descendants may themselves spawn and advertises the
maximum descendant depth when it is bounded.

## The Simple Aero workflow model

`simple_aero` is advertised as a `trajectory_workflow`, not as a tenth
physical vehicle. It is a particularly useful composition example because its
configuration follows the shape of an engineering mission form:

- launch latitude/longitude, altitude, speed, pitch-over angle, and heading offset;
- either a range/bearing endpoint or an explicit geodetic aimpoint;
- endpoint altitude, speed, and heading;
- initial mass, thrust, mass flow, fixed drag coefficient, and fixed `L/D`;
- burnout-speed and apogee checkpoints;
- an ordered list of independently configured segment occurrences; and
- integration step, output interval, and environment preset.

Focused hosts select `taoryx.simple-aero.mission-composition` from the
`taoryx.simple-aero` plug-in. That provider has its own package version and
endpoint witness; it does not require the legacy aggregate provider to author,
run, or inspect Simple Aero. In a source checkout, a selected plug-in scope
sees only its endpoint record; an installed wheel reads that same record and
witness from Simple Aero package data.

The segment list is open for composition experiments and publishes reviewed
templates for `ballistic`, `cbcr`, `crossrange`, `marv`, `phugoid`,
`range_extension`, `skip`, `slalom`, and `weave`, plus the runnable
`fixed_ld_baseline`. Every reviewed template has a bounded fixed-L/D
point-mass batch lowering. Segment parameters stay with each occurrence, so two
weaves or ballistic coasts may carry different durations or checkpoints.

For normal application authoring, use the typed convenience layer. It maps
directly to the portable `Group` / `Choice` / `Optional` / `Sequence` tree, so
it is easier to read without hiding any advertised consumer-facing semantics:

```python
from taoryx.trajectory import (
    SimpleAeroGeodeticAimpoint,
    SimpleAeroLaunch,
    SimpleAeroMission,
    SimpleAeroSegment,
    prepare_simple_aero_mission,
)

mission = SimpleAeroMission(
    configuration_id="crossrange-demo",
    launch=SimpleAeroLaunch(altitude_m=1_000.0, speed_m_s=100.0, longitude_deg=190.0),
    endpoint=SimpleAeroGeodeticAimpoint(latitude_deg=36.0, longitude_deg=-5.6),
    segments=(
        SimpleAeroSegment.powered_ascent(duration_s=4.0),
        SimpleAeroSegment.ballistic_coast(duration_s=8.0, alpha_deg=1.0),
        SimpleAeroSegment.crossrange(initial_heading_error_deg=12.0, minimum_time_to_go_s=30.0),
        SimpleAeroSegment.terminal_pronav(duration_s=3.0, capture_range_m=50.0),
    ),
)
prepared = prepare_simple_aero_mission(mission)
```

`SimpleAeroLaunch`, aimpoint and endpoint-state headings normalize periodic
angles to the canonical advertised intervals. `SimpleAeroSegment` has named
constructors for every variant: `powered_ascent`, `ballistic_coast`,
`bank_maneuver`, `cbcr`, `crossrange`, `marv`, `phugoid`, `range_extension`,
`skip`, `slalom`, `weave`, and `terminal_pronav`. Optional source checkpoints
remain explicit optional nodes in the resulting tree, and generated occurrence
IDs make repeated segment variants traceable. `SimpleAeroMission.template("phugoid")`
materializes a reviewed starting sequence with schema-owned defaults.

Arbitrary sequences use the separately advertised `Custom Composition (Open
Sequence)` batch binding by default. That binding means only that a
schema-valid sequence has a bounded fixed-L/D lowering; it does not relabel the
sequence as a reviewed source template or make a vehicle-performance claim.

Only `point_mass_3dof` is declared. The metadata explicitly marks all higher
fidelity realizations and transitions unavailable; sharing a segment name with
a pseudo-6-DOF or rigid-body implementation does not authorize automatic
promotion. Source-shaped maneuver parameters lower to bounded generated
fixed-L/D alpha, bank, and duration profiles before batch execution. That makes
the templates executable workflow recipes, not validation of vehicle-specific
physics, historical Simple Aero semantics, optimization, or terminal accuracy.

## Execution, feedback, and results are separate contracts

The common backend lifecycle is:

```text
advertisement -> typed configuration -> validated/fingerprinted configuration
              -> MissionCompositionRunRequest
              -> MissionCompositionRunnerRegistry.run()
              -> trajectory response | failure response
```

Validation resolves defaults and checks model IDs, fidelity IDs, parameter
names, units, types, bounds, choices, and segment ordering without executing a
plant. The prepared configuration fingerprints both the caller-authored tree
and resolved values; deserialization rejects a handoff whose contents no
longer match its fingerprint.

The common runner dispatches by `(provider_id, model_id)`. It catches provider
exceptions at that boundary and always returns a discriminated response:

- `kind: trajectory` contains `taoryx.mission-composition-trajectory/v1`;
- `kind: failure` contains `taoryx.mission-composition-failure/v1`.

Failures carry a stable category, lifecycle phase, retryability, request and
configuration identity, and one or more structured diagnostics. Each
diagnostic has a stable kebab-case code, severity, recoverability, JSON-pointer
path, user-facing message and hint, and optional provider/model/object/segment
identity. Unknown plug-in exceptions become `provider-internal-error`; their
private message and traceback are not leaked through the public response.

`MissionCompositionRunRequest` is batch-only. Interactive execution uses the
stateful session contract described below; a caller cannot obtain persistent
state semantics by sending a stateless request with `operation="step"`.

For the repository provider, `build_registry_mission_composition_runner()`
registers one generic native adapter path. That adapter compiles the portable
configuration into the existing immutable vehicle-composition representation,
resolves the exact family/model/realization × mission × batch binding from the
factory registry, runs that existing implementation, and projects every result
through the same multi-object output contract. There is no vehicle-specific
Mission Composition runner per family.

For canonical vehicle families, model metadata contains an exact operation
matrix for every mission-template/fidelity pair. `available` applies only to
that exact tuple. `availability_scope` distinguishes a provider-interface
operation from an existing native runtime binding, while
`common_runner_status` distinguishes `registered`, `adapter_required`, and
`not_available`. A generic configuration provider does not gain permission to
dispatch a runtime merely because a model has some native runnable operation.

The complete contract is in the
[Mission Composition Provider API](architecture/mission-composition-provider-api.md).

## Audit every advertisement

`audit_provider_advertisement()` JSON-round-trips provider and model metadata,
loads every advertised configuration and output schema, verifies their
model/version/schema/fingerprint identities, checks configuration-frame references, fidelity declarations,
deployment trigger and operation references, common-runner registration, and
the presence of output metadata. It also requires every interactive
realization to publish active action channels and exact native binding schemas.
Each model report inventories node kinds, value types, model properties,
reference frames, control statuses/channels/authorities/intents, native-bound
controls, and output channels.
The report includes a pass/fail record for every model, not only a provider-wide
boolean.

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --audit
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --audit --json
```

This is publication conformance. The integration suite separately executes
every exact tuple marked available. The checked-in completion audit combines
the inventory, publication audit, exact common-runner registrations, generated
coverage matrix, and intentionally deferred physics backlog:

```bash
python tools/dev.py mission-composition-completion
```

The authoritative records are
[mission_composition_inventory.yaml](../verification/mission_composition_inventory.yaml),
[the generated family/realization matrix](architecture/mission-composition-coverage-matrix.md),
and
[mission_composition_physics_backlog.yaml](../verification/mission_composition_physics_backlog.yaml).
Their completion rule is strict: every exact combination advertised as
available must execute through the common interface; every other combination
must be blocked or unsupported, and every realization must explicitly publish
how its control intents are resolved. The session suite also opens every
registered episode and compares its live action schema with the advertised
native control binding. None of these audits promotes fidelity
evidence or vehicle qualification.

## The contract-probe vehicle

`ContractProbeMissionCompositionProvider` is the development proof for the
entire plug-in contract. Its single `contract_probe_vehicle` is explicitly
synthetic and non-physical. The advertisement includes every configuration
node and value type, presentation controls and numeric formats, scalar/vector/
range/unknown model properties, three reference frames, core and grouped
telemetry channels, all output interpolation kinds, a fidelity ladder with
available and blocked operations, all four deployment states, and a runnable
parent/child/grandchild lineage result with explicit spawn initial states. Its
control surface is the exhaustive consumer fixture: bounded and unbounded
continuous commands, periodic values, vectors, booleans, enum selectors,
detents, one-shot events, provider-defined composite effectors, every command
lifecycle/release policy/sampling state, and a matching native binding for
each. It is designed to produce the mixed Torch-friendly action-space and
event masks that a generic agent consumer must support.

The example also submits an invalid output channel. The same runner returns a
normal trajectory for the valid request and a structured `unsupported` failure
for the invalid one:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py \
  --output /tmp/mission-composition-contract-probe.json
```

This is the first integration target for a generic editor or remote plug-in
host. Passing it proves contract coverage and serialization behavior, not
physical trajectory correctness.

It is installed with the model profile and is discoverable as
`taoryx.debug.mission-composition-contract-probe`, alongside the two reference
models. After compiling a prepared configuration, consumers can use the same
batch boundary for all three fixtures:

```bash
taoryx model run <provider-id> <prepared.json> --output <response.json>
```

The independently installable `taoryx-debug-models` package also owns focused
batch witnesses for `reference-ballistic-3dof-batch`,
`reference-waypoint-3dof-batch`, and `debug-contract-probe-batch`. Discover
them with `taoryx model endpoint-specs` and run an exact proof with
`taoryx model verify <endpoint-id> --execute`. Those checks validate the
authored configuration, provider/model versions, declared runner, output
surface, and normalized result. They remain nonphysical API fixtures; the
waypoint and probe's selectable session modes expose their own runtime masks
and requested/applied/achieved readback separately.

The command revalidates the prepared configuration, dispatches only a
provider-owned common batch runner, and writes a discriminated trajectory or
failure response. The contract probe is particularly useful with
`--output-mode all` and `--maximum-objects 2` to exercise typed output and
lineage limits.

## The two reference models

The checked-in example provider publishes exactly two deliberately small
native `point_mass_3dof` models. They share the same discovery, parameter,
segment, preparation, and trajectory-result architecture.

| Model | Initialization | Segment | What it demonstrates |
| --- | --- | --- | --- |
| `reference_ballistic_3dof` | `launch_state`: position, speed, heading, and flight-path angle | `ballistic_coast` | A single repeatable segment with constant-gravity propagation |
| `reference_constant_velocity_waypoint_3dof` | `initial_state`: position, speed, and heading | `waypoint_leg`: duration, target position, and arrival tolerance | A variable-length sequence of simple waypoint segments |

Both models advertise native `point_mass_3dof` batch execution. The provider
executes that fidelity directly. The waypoint model travels at
constant speed toward its target, holds for the remainder of a captured leg,
and resumes its configured cruise speed at the next leg boundary.
There is no automatic lowering step between composition and these reference
executors. This is an intentionally small contract fixture, not source-grounded
vehicle evidence or a claim of historical TAOS runtime compatibility.

## The runnable reference binding

`ReferenceMissionCompositionProvider` publishes the two analytical models,
validates the portable configuration tree, and registers both batch executors
with the common runner. Its friendly builders are the recommended consumer
entry point for these fixtures: callers provide SI-valued launch or route
objects, while the provider still creates and validates the same portable
`Group` / `Choice` / `Sequence` configuration tree that generic consumers use.

```python
from taoryx.trajectory import (
    ReferenceBallisticLaunch,
    ReferenceMissionCompositionProvider,
    ReferenceWaypoint,
    ReferenceWaypointCourseStart,
)

provider = ReferenceMissionCompositionProvider()
ballistic = provider.prepare_ballistic(
    ReferenceBallisticLaunch(
        altitude_m=1_000.0,
        speed_m_s=150.0,
        heading_deg=360.0,  # normalized to the canonical 0-degree bearing
        flight_path_angle_deg=20.0,
    ),
    coast_durations_s=(2.0, 2.0),
)
course = provider.prepare_waypoint_course(
    ReferenceWaypointCourseStart(
        altitude_m=1_000.0,
        speed_m_s=100.0,
        heading_deg=90.0,
    ),
    waypoints=(
        ReferenceWaypoint(north_m=0.0, east_m=500.0, altitude_m=1_000.0, instance_id="outbound"),
        ReferenceWaypoint(north_m=0.0, east_m=0.0, altitude_m=1_000.0, instance_id="return"),
    ),
)
```

`ReferenceWaypoint.duration_s` is optional. When omitted, the builder derives
a direct Euclidean travel time at the configured cruise speed; an explicitly
timed short leg changes the planned origin used for the next automatic leg.
That convenience deliberately does not model turns, guidance performance, or
vehicle capability. Callers that need arbitrary configuration-tree assembly
can continue to use the provider-neutral authoring API.

The checked-in example writes the discriminated common response.

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_reference.py \
  --output /tmp/mission-composition-trajectory.json
```

The provider schema supplies labels, descriptions, physical quantities,
canonical and display units, hard/qualified/safe-extended intervals, defaults,
periodicity, value-space topology, transforms, projection policy, roles,
frames, fidelity compatibility, availability, coupling, invalidations, and
provenance. Each segment occurrence carries its own values, so the two
waypoints can be edited or generated independently.

## Standard multi-object trajectory

One response may contain a primary vehicle and any number of independently
propagated children: spent boosters, detached stages, deployed payloads, or
tumbling bodies. Every `TrajectoryObject` owns its model and realization IDs,
role, channel metadata, samples, segment spans, active interval, status,
fidelity, provenance, and claim boundary. A child additionally names its parent object, the advertised
`deployment_id`, and the accepted-boundary spawn event. Every child also has a
`TrajectoryEntityRelationship` containing the parent and child IDs,
relationship kind, deployment and event IDs, accepted creation time,
state-transfer policy, and initial-state snapshot. The snapshot must equal the
child's first returned sample. Death events are linked the same way. Validators
require unique IDs, monotonic finite samples, exact channel shapes, valid event
and relationship references, consistent spawn/death times, and an acyclic
parent graph.

Model advertisements expose deployment capability separately from returned
lineage. The X-15 and HL-20 publications advertise independently propagated
spent-booster children. The NESC two-stage publication advertises its staging
event lineage. By default, that source replay returns no detached stage; a
composition may instead select its separately installed passive-body child
runtime for the accepted stage-separation event. That child is explicitly a
synthetic local-atmosphere cylinder, not a source-exact NESC spent-stage
trajectory. An empty `deployments` list means the model publishes no child
emission capability; consumers must not infer one from a segment name.

### Selected cross-plug-in deployments

An independently propagated child is always composition-selected. The binding
names the parent deployment/event, child plug-in and runtime, model/family,
requested fidelity, transfer policy, and the claim boundary. This makes the
same selection visible to a UI, a training client, and an execution host;
the returned receipt repeats the realized fidelity, accepted state, telemetry,
status, termination, and provenance.

```yaml
deployment_bindings:
  - id: nesc-synthetic-passive-cylinder-pseudo6dof
    deployment_id: stage_separation_lineage
    release_event_id: stage_separation
    child_object_id: nesc-synthetic-cylinder-1
    child:
      plugin_id: taoryx.passive-bodies
      runtime_id: taoryx.passive-bodies.local-atmosphere-release.v1
      model_id: nesc-synthetic-cylinder
      family_id: tumbling_body
      fidelity: pseudo_6dof
    state_transfer: source_replay_kinematic_projection
    claim_boundary: Explicit synthetic child; no source-exact stage or impulse claim.
```

The host resolves that runtime only from the catalog selected for the parent
execution. It rejects an absent, differently owned, or incompatible runtime;
it never silently imports an installed aggregate or substitutes a model. The
NESC handoff projects the accepted ECI replay state into the child's declared
event-local tangent frame. Its provenance records both the original replay
state and the projection, while the parent source-replay truth remains
unchanged.

A direct vehicle-composition request carries this binding with its parent
composition. The portable common runner instead accepts the same typed values
on `MissionCompositionRunRequest.deployment_bindings`, deliberately outside the
prepared parent configuration. This lets callers reuse a parent configuration
without implicitly spawning a child, while still returning the selected child,
event, relationship, accepted state, and realized-fidelity readback through the
standard multi-object result. A binding that the selected model does not
advertise is rejected rather than ignored.

Simulation Runtime artifacts can be converted with
`trajectory_result_from_run_artifact()`. Runtime telemetry now preserves model
and parent-model identity, while committed spawn events provide accepted-time
lineage. The caller supplies a `RuntimeArtifactProjection` containing the
provider/request/configuration identity and claim boundary that cannot be
derived safely from generic telemetry.

## Stateful interactive sessions

Interactive availability in a mission operation record means that an exact
session adapter is registered. It does not mean that the batch runner accepts a
stateless step request. The common lifecycle is:

```text
open/create(prepared configuration, seed, integration step)
    -> inspect()
    -> step(action, held duration, expected sequence) ...
    -> reset(seed) or close()
```

`MissionCompositionSessionManager` adapts all existing native episode
factories to this contract. The session descriptor fixes provider/model/
mission/realization identity, configuration fingerprint, provider-owned state,
deterministic-reset policy, integration timestep semantics, action schema,
observation schema, and spawned-entity capability. Every action and observation
is checked against its advertised primitive type, shape, bounds, units, and
value-space metadata. Sequence numbers reject stale concurrent steps. Inspect
never advances state; reset reconstructs the prepared initial state; close is
idempotent and rejects subsequent state mutation. Unknown, duplicate, stale,
closed, and native execution failures use the same structured diagnostic
vocabulary as batch execution.

The session observation reserves `spawned_entity_ids` for models that emit
children while stepping. No current native episode advertises that capability,
so it remains false rather than synthesizing child histories.

### Selecting a live control mode

Callers normally choose an `authority_profile_id` in the open-session request,
or set `startup_authority_profile_id` in the reusable configuration before it
is validated. The returned descriptor then projects only that profile's
semantic action schema. This lets a generic UI render an exact input surface,
and lets an AI/RL adapter obtain a matching normalization-ready action-space
projection, without learning a provider's private plant coordinates.

Each completed step returns four complementary views of control:

| Field | Consumer meaning |
| --- | --- |
| `requested_action` | The new semantic values submitted by the client |
| `applied_action` / `control_feedback` | Which values were accepted, held, limited, withheld, or unavailable, plus any same-unit achieved readback |
| `lowered_action` | The request delivered to the registered model adapter or plant seam |
| `lowering_evidence` | The adapter's documented targets, limits, gains, mode, and resource reasoning |

`observation.control_authority` is the live source of truth for the active
profile, command owner, phase, availability reason codes, and action mask. A
static authority profile says what a model can support in principle; it does
not promise that a resource-limited, terminal, or otherwise unavailable model
will accept every channel now.

#### Hummingbird pseudo-6DOF example

The current multirotor example demonstrates the intended progression from
direct user input to higher-level guidance without exposing invented physical
controls:

| Profile | Suitable client input | Low-level boundary |
| --- | --- | --- |
| `body_motion_response` | Roll, pitch, yaw, aggregate thrust fraction, and propulsion enable | Five-coordinate aggregate response-law seam |
| `velocity_yaw_command` | North/east/positive-up velocity, yaw, and propulsion enable | Bounded velocity response lowers to the same aggregate seam |
| `live_waypoint_guidance` | Local waypoint, capture radius, speed limits, yaw, and propulsion enable | Waypoint/velocity cascade lowers to the same aggregate seam |

All three profiles are caller-owned and declare explicit bumpless transfer.
They publish requested, applied, and achieved values, waypoint progress, and
battery-aware availability. They do **not** expose individual rotor commands:
the Hummingbird individual-rotor LQI screen is a separate batch-only physical
realization. A generic front end should therefore render the profile selected
by the session descriptor, not promote rotor inputs because the family is a
quadcopter.

See [the Hummingbird profile reference](architecture/vehicle-interface-contract.md#hummingbird-quadcopter-streaming-profiles)
for exact channel units and bounds, and
[select a streaming control profile](architecture/model-authoring-automation.md#select-a-streaming-control-profile)
for a Python session example.

## Run the checked-in example

Inspect every canonical family or one complete schema:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py \
  --model reference_nesc_two_stage_rocket --json
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py \
  --model simple_aero
```

Run the analytical waypoint executor:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_reference.py \
  --output /tmp/mission-composition-trajectory.json
```

The example prints the advertisement-audit status and a compact run summary. The
standard result can be consumed by reports, plots, replay tools, or another
composition client without knowing how either analytical model propagates.

## Fidelity is a request graph, not a live model switch

Every canonical model advertises all four ladder positions, including explicit
`declared: false` records where a tier is unavailable. Adjacent transition
records distinguish:

- `step_up`: always an explicit target-fidelity request;
- `step_down` with `exact_only`: an explicit lower-fidelity request; and
- `step_down` with `validated_lower_only`: an opt-in fallback that still
  requires qualified profile evidence and required adapter operations.

The transition status, requirements, automatic-selection policy, and state
transfer disposition are separate fields. The current contract advertises no
live state transfer: changing fidelity starts a newly prepared run. It never
uses lower rank alone as evidence that a model is a valid substitute.

## Boundaries

Mission Composition owns discovery and mission assembly. Model Authoring owns
source meaning and diagnostics. Simulation Runtime owns model execution,
accepted truth, stepping, and artifacts. A composition provider must publish
what it can execute and must preserve the claim boundary of the underlying
model; a successful request is not qualification evidence.
