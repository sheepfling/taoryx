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

The repository-backed implementation is
`RegistryMissionCompositionProvider`. It projects, rather than copies, the
canonical vehicle-composition, fidelity, value-space, and execution-binding
authorities. It advertises all nine resolved vehicle families, the
`simple_aero` trajectory workflow, and the dual-launch glider family. A model
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
| Missions and deployments | Ordered templates, exact operation matrices, and parent/child lifecycle contracts |
| Coordinates and outputs | Reference-frame definitions, required core channels, selectable telemetry groups, entity-output behavior, units, and interpolation rules |

Canonical units are the transport and execution values. `display_unit` and
numeric formatting are presentation requests; a front end may convert and
format them without changing the submitted canonical value. Likewise, a
`slider` or `coordinate_picker` hint does not relax bounds or change a value's
topology. Output availability is explicit: `guaranteed`, `conditional`, or
`runtime_reported`.

## Ask separately what can be set and what can be returned

Mission Composition publishes two schemas around the execution call:

```text
                         MODEL REGISTRY
                              │
                 ┌────────────┴────────────┐
                 ▼                         ▼
       ConfigurationSchema          OutputSchema
          what can I set?         what can I request?
                 │                         │
                 └────────────┬────────────┘
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

The segment list is open for composition experiments and publishes reviewed
templates for `ballistic`, `cbcr`, `crossrange`, `marv`, `phugoid`,
`range_extension`, `skip`, `slalom`, and `weave`, plus the runnable
`fixed_ld_baseline`. Segment parameters stay with each occurrence, so two
weaves or ballistic coasts may carry different durations or checkpoints.

Only `point_mass_3dof` is declared. The metadata explicitly marks all higher
fidelity realizations and transitions unavailable; sharing a segment name with
a pseudo-6-DOF or rigid-body implementation does not authorize automatic
promotion. The fixed-L/D baseline advertises batch generation/execution. The
named Simple Aero maneuver templates advertise schema validation and
fixture-level semantics but keep batch execution blocked until a
configuration-to-runtime adapter and vehicle-specific evidence are registered.

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
the presence of output metadata. Each model report also inventories node kinds,
value types, model properties, reference frames, and output channels.
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
must be blocked or unsupported. None of these audits promotes fidelity
evidence or vehicle qualification.

## The contract-probe vehicle

`ContractProbeMissionCompositionProvider` is the development proof for the
entire plug-in contract. Its single `contract_probe_vehicle` is explicitly
synthetic and non-physical. The advertisement includes every configuration
node and value type, presentation controls and numeric formats, scalar/vector/
range/unknown model properties, three reference frames, core and grouped
telemetry channels, all output interpolation kinds, a fidelity ladder with
available and blocked operations, all four deployment states, and a runnable
parent/child/grandchild lineage result with explicit spawn initial states.

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
constant speed toward its target and holds once it captures the waypoint.
There is no automatic lowering step between composition and these reference
executors. This is an intentionally small contract fixture, not source-grounded
vehicle evidence or a claim of historical TAOS runtime compatibility.

## The runnable reference binding

`ReferenceMissionCompositionProvider` publishes the two analytical models,
validates the portable configuration tree, and registers both batch executors
with the common runner. The checked-in example contains the complete typed
waypoint request and writes the discriminated common response.

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
event lineage but explicitly does not advertise an independent spent-stage
trajectory. An empty `deployments` list means the model publishes no child
emission capability; consumers must not infer one from a segment name.

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
