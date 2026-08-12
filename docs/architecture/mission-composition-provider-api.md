# Mission Composition Provider API

The Mission Composition provider API is the plug-in boundary between a host
application and a vehicle or trajectory implementation. A consumer with no
provider-specific knowledge must be able to discover models, construct and
validate a configuration, select an exact operation and fidelity, request a
trajectory, and interpret either the result or failure.

The canonical Python surface is
[`taoryx.trajectory.mission_composition`](../../src/taoryx/trajectory/mission_composition.py).
The shortest consumer guide is the
[Mission Composition front door](../MISSION_COMPOSITION.md).

## Contract separation

The versioned artifacts prevent discovery, authoring, dispatch, and execution
from becoming one provider-specific dictionary:

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
                validated configuration + output selection
                              │
                 ┌────────────┴────────────┐
                 ▼                         ▼
       batch run request             stateful session
       -> trajectory | failure       open/inspect/step/reset/close
```

The public schemas are:

| Artifact | Schema ID |
| --- | --- |
| Provider metadata | `taoryx.trajectory-provider-metadata/v1` |
| Configuration schema | `taoryx.trajectory-provider-configuration-schema/v1` |
| Output schema | `taoryx.trajectory-provider-output-schema/v1` |
| Per-realization control advertisement | Nested `TrajectoryControlAdvertisement` in model metadata |
| Per-fidelity control-scheme matrix | `control_scheme_support` in model metadata |
| Configuration instance | `taoryx.trajectory-provider-configuration/v1` |
| Prepared configuration | `taoryx.trajectory-provider-prepared-configuration/v1` |
| Run request | `taoryx.mission-composition-run-request/v1` |
| Trajectory result | `taoryx.mission-composition-trajectory/v1` |
| Failure | `taoryx.mission-composition-failure/v1` |
| Open session request | `taoryx.mission-composition-open-session/v1` |
| Session descriptor | `taoryx.mission-composition-session/v1` |
| Session observation | `taoryx.mission-composition-session-observation/v1` |
| Session step request/result | `taoryx.mission-composition-session-step-request/v1`, `taoryx.mission-composition-session-step-result/v1` |
| Authority switch/transition | `taoryx.mission-composition-switch-authority/v1`, `taoryx.mission-composition-authority-transition/v1` |
| Reset/inspect/close session | `taoryx.mission-composition-reset-session/v1`, `taoryx.mission-composition-inspect-session/v1`, `taoryx.mission-composition-close-session/v1` |
| Advertisement audit | `taoryx.mission-composition-advertisement-audit/v1` |
| Asset inventory | `taoryx.mission-composition-inventory/v1` |
| Completion report | `taoryx.mission-composition-completion/v1` |

Provider metadata publishes these common contract IDs separately from its
native execution-binding contract. A provider may therefore wrap the TAORYX
runtime, an analytical model, a recorded trajectory service, or an external
implementation without changing the host contract.

## Discovery and configuration

```python
class ConfigurableTrajectoryProvider(Protocol):
    @property
    def metadata(self) -> TrajectoryProviderMetadata: ...

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]: ...
    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema: ...
    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema: ...
    def validate_configuration(
        self, configuration: TrajectoryConfigurationInstance
    ) -> PreparedTrajectoryConfiguration: ...
```

Discovery must not execute a trajectory. `list_models()` returns stable model
identity, family and model kind, presentation metadata, typed read-only model
properties, per-realization controls, a flattened control-scheme support
matrix, reference frames, a fingerprinted output schema, exact operations,
mission templates, deployments, source references, fidelity records, and
fidelity transitions.
Each model record identifies its configuration schema and SHA-256 fingerprint.

### Presentation and read-only model properties

Presentation metadata is non-authoritative. It lets a generic front end choose
labels, groups, ordering, default views, editor controls, precision, and display
units while canonical values and validation remain unchanged.

`TrajectoryModelPropertyMetadata` describes model facts rather than mutable
run inputs. Each property publishes:

- stable ID, label, description, and semantic role;
- value type and whether the value is exact, nominal, representative, a limit,
  a range, merely declared, or explicitly unknown;
- a typed value or interval;
- quantity, canonical unit, display unit, periodicity, frame, and value-space
  topology where applicable;
- group/order/visibility/control and numeric-format hints; and
- source references, provenance, and claim boundary.

`TrajectoryReferenceFrameMetadata` defines every frame named by configuration
or output metadata. `TrajectoryOutputChannelMetadata` publishes semantic
channel ID, primitive type (`float64`, `int64`, `boolean`, `string`, or JSON),
scalar/array shape, quantity, canonical/display units, frame, sampling
semantics, interpolation, periodicity, compatible fidelities, realizations,
missions and operations, and whether presence is `guaranteed`, `conditional`,
or `runtime_reported`. Returned objects repeat the actual channel metadata so
runtime output remains self-contained and values can be validated without a
provider-specific state-vector definition.

### Per-realization control advertisement

Every `TrajectoryRealizationMetadata` must contain a complete
`TrajectoryControlAdvertisement`; an omitted controls dictionary is not a
valid advertisement. It publishes:

- a control status: `available`, `internally_generated`, `uncontrolled`,
  `blocked`, or `unsupported`;
- semantic action and effector channels with primitive type, shape, quantity,
  units, interval, frame, sampling, topology, availability, operations, and
  presentation/provenance metadata;
- mutually exclusive authority profiles and their channel membership;
- mission-level intents, the segments and templates that require them, and
  whether each intent is externally resolved, provider-internal, open-loop,
  blocked, or unsupported; and
- an exact native binding schema when a semantic channel maps to a provider
  coordinate.

The semantic channel and native binding deliberately retain separate value
spaces. For example, a stable semantic angle may bind to a source-table angle
coordinate, while a bounded speed command may bind to a native positive-half-
line coordinate with additional runtime bounds. The native record repeats its
ID, type, shape, quantity, unit, interval, topology, and provider binding so a
composer can validate an interactive action before opening a plant. Discovery
tests then compare that record byte-for-byte at the field level with every live
session action schema.

The channel's command lifecycle is declared separately from its mathematical
value space. See [Control semantics and agent action spaces](control-agent-action-space.md)
for absolute/rate/increment/event commands, held/momentary/latched/pulse
behavior, numeric detents, and the Torch-friendly action-space projection.

An internally generated command is discoverable but is not presented as an
interactive action. Likewise, planned source effectors remain visible with no
execution operation, and an uncontrolled realization cannot silently inherit
controls from another fidelity.

`TrajectoryModelMetadata.control_scheme_support` is the UI-ready join over
those exact authority profiles. Each row names one cross-family scheme, the
realization and fidelity IDs that support it, the underlying authority and
action IDs, intended consumer roles, streaming preference, operations,
ownership, switching policy, and claim boundary. It is materialized in the
serialized model record so a client does not reconstruct a taxonomy from
channel spelling. Legacy undeclared profiles may be conservatively labeled
`authority_kind_fallback`; promoted profiles and the low-fidelity pilot must
use `declared` records. A model with no supported authority publishes an empty
matrix rather than an invented open-loop or native bridge.

### Output capability and selection

`TrajectoryOutputSchema` is the output-side peer of the configuration schema.
It has a stable JSON contract and fingerprint and separates:

- required core-state channels;
- optional, model-specific telemetry channels;
- named telemetry groups such as controls, actuators, resources, propulsion,
  aerodynamics, guidance, stage state, or diagnostics; and
- multi-entity output capability, relationship kinds, child-schema policy,
  lifecycle-event support, spawn initial-state support, recursive-spawn
  support, and maximum descendant depth.

Telemetry stays out of the canonical state. Control-surface positions,
actuator states, throttle, fuel quantity and flow, propulsion state,
aerodynamic quantities, guidance outputs, stage state, and provider-specific
measurements are advertised as typed telemetry channels with the same unit,
frame, interpolation, fidelity, and presentation metadata used elsewhere.

`MissionCompositionOutputSelection.mode` has three portable meanings:

| Mode | Result |
| --- | --- |
| `core` | Return the applicable core state only |
| `selected` | Return core plus named telemetry channels and telemetry groups |
| `all` | Return core plus every applicable advertised telemetry channel |

Core state is never removed by a telemetry selection. Unknown or
fidelity/operation-incompatible telemetry fails during preflight. Conditional
telemetry that a runtime cannot emit is reported with a structured warning.
Lifecycle relationships are structural: suppressing ordinary events does not
turn a spawned child into anonymous telemetry or erase the event required to
establish its lineage.

Accepted time is the structural `time_s` field on each sample rather than a
duplicated channel. Every returned object must contain at least one guaranteed
core-state channel, and every applicable core channel must be present at every
accepted sample. The core vocabulary uses canonical position, velocity,
attitude, and rate identities with declared frames where the selected native
model actually exposes them. A reduced or local controller-screen realization
does not fabricate unavailable attitude or navigation state merely to fill a
larger vector; its exact core scope and claim boundary remain discoverable.

The configuration schema is a portable AST:

| Node | Meaning |
| --- | --- |
| `Parameter` | One typed value with units, bounds, topology, defaults, role, and provenance |
| `Group` | Fields that apply together |
| `Choice` | Exactly one structurally distinct variant |
| `Sequence` | An ordered, bounded list with advertised templates or custom composition |
| `Optional` | An explicitly present or absent subtree |

Mutually exclusive modes are distinct `Choice` variants, not booleans plus
hidden cross-field rules. Repeated segments are `Sequence[Choice[Segment]]`,
so every occurrence carries its own parameter subtree. No public node contains
a Python callback.

Numeric metadata keeps representation, physical quantity, canonical unit,
display unit, validity interval, qualified interval, safe-extended interval,
and periodicity separate. A heading is a periodic value space, not merely a
number bounded between 0 and 360. Current validation accepts canonical units;
conversion belongs at the application boundary.

`validate_configuration()` checks the complete tree and returns resolved
values plus a fingerprint over both the caller-authored configuration and
those resolved values. `PreparedTrajectoryConfiguration` rechecks that digest
when parsed, so a stale or modified handoff fails before execution.

## Exact operation and fidelity publication

Model-level operations are an index, not blanket permission. Every mission
template publishes an operation matrix keyed by
`(mission, fidelity, realization, operation)` with
`available` or `blocked` status, blockers, execution mode, and claim boundary.
Availability also names its scope and common-runner disposition. A native
runtime binding may be `available` while `common_runner_status` remains
`adapter_required`. Only model `common_runner_operations` and operation records
marked `registered` authorize common dispatch. This keeps an existing native
factory visible without falsely claiming that the provider plug-in already
adapts its request and result envelopes.

Likewise, model status `common_runner_ready` means only that the model has at
least one exact registered tuple. It does not make every realization, mission,
or operation runnable. A realization is independently `available`, `blocked`,
or `unsupported`, and a generic consumer must resolve the exact operation
record before presenting a Run or Open Session action.

Dynamics and controls are independent axes. The normalized dynamics values are
`point_mass_3dof`, `pseudo_6dof`, and `rigid_body_6dof`. Input realization is
published separately as uncontrolled, guidance-command, direct-wrench,
actuator-allocated, source-replay, or provider-defined. Actuator type is a
third field: aerodynamic surfaces, rotors, thrust vectoring, gimbals, RCS,
mixed, or not applicable. Historical four-tier fidelity IDs remain selectable
compatibility aliases; they are not used to infer controls or effectors.
The normalized input-realization field is only a classification; the nested
control advertisement is the authoritative inventory of channels, native
bindings, authority profiles, intent resolution, and operation availability.

All four canonical fidelity positions remain visible. An absent realization is
`declared: false`; it is not omitted. Adjacent transition records distinguish
step-up from step-down, explicit selection from validated lower-fidelity
fallback, requirements, and state-transfer support. The current registry
advertises no live state transfer: selecting another fidelity creates another
prepared run.

## Deployment advertisement

`TrajectoryDeploymentMetadata` describes one parent-to-child capability:

- stable deployment ID, trigger segments, and trigger event kinds;
- status (`available`, `declared`, `blocked`, or `not_available`);
- child role, model identity/scope/kind, and child cardinality;
- compatible fidelities and exact batch/step operations;
- native/provider availability scope and common-runner registration status;
- accepted-boundary state-initialization and fidelity policy;
- lifecycle (`independently_propagated`, `attached_only`, `event_only`, or
  `external`); and
- blockers, provenance, sources, and claim boundary.

An `event_only` declaration does not authorize a synthetic child trajectory.
An empty deployment list means that the provider publishes no child-emission
capability for that model.

The registry currently exposes:

| Parent | Publication |
| --- | --- |
| X-15 | Available, independently propagated passive spent booster |
| HL-20 | Available, independently propagated synthetic passive spent booster |
| NESC two-stage rocket | Declared cutoff/separation/ignition lineage; no independent spent-stage trajectory |
| Dual-launch glider | Source-generated point-mass batch path for both launch forms; declared event-only separation with no independently propagated child |
| Other models and `simple_aero` | No advertised child emission |

## Common run request

`MissionCompositionRunRequest` carries only identity-bearing, provider-neutral
data:

- request, provider, and provider-version IDs;
- the batch operation (interactive state changes use the session contract);
- the validated `PreparedTrajectoryConfiguration`; and
- output mode, selected telemetry channels/groups, cadence,
  event/segment/child inclusion, and sample or object limits.

`MissionCompositionRunnerRegistry` registers executors by
`(provider_id, model_id)`. It refuses missing executors, validates result
identity against the request, and catches every provider exception at the
plug-in boundary. A caller receives a discriminated response and does not need
provider-specific exception classes.

The repository adapter uses one reusable path rather than a runner per vehicle:

```text
portable prepared configuration
    -> compile native vehicle composition
    -> resolve exact family/model/realization × mission × batch factory
    -> execute existing native binding
    -> select core/selected/all outputs
    -> project one or more trajectory objects
```

The callable factory registry is outside the CLI. Advertisement conformance
requires every exact available batch tuple to name the registered common
executor, and integration tests execute all such tuples.

## Stateful interactive session contract

An operation record with `operation="step"` advertises availability through the
session executor; it never authorizes a stateless `MissionCompositionRunRequest`.
The common session lifecycle consists of:

- `open`/`create`: validate the prepared configuration, exact mission,
  realization and session operation; create provider-owned state;
- `inspect`: return the current committed observation without advancing time;
- `step`: validate a typed action, hold it for an explicit caller duration,
  require an optional expected sequence, and return the committed observation;
- `switch_authority`: for a semantic-profile session, require an expected
  sequence and an explicitly bumpless pair of profiles, preserve plant state
  and time, clear held references, and return the replacement action schema;
- `reset`: deterministically reconstruct the prepared initial state using the
  declared seed semantics; and
- `close`: release native state and retain a terminal acknowledgement.

The immutable descriptor publishes session identity, state ownership,
configuration fingerprint, seed, integration timestep semantics, action and
observation schemas, deterministic reset policy, claim boundary, and whether
the session can emit spawned entities. It also advertises every authority
profile, the default and active profile, command source, ownership, selection
scope, switching policy, phase applicability, and lowering chain. Every
profile row includes its complete action schema and dependency-free agent
action space, including stable order, flat offsets, native bounds, agent
bounds, periodic wrapping, discrete choices, and whether external
normalization statistics are required. A mode picker therefore does not need
to switch authority merely to discover another mode's UI or policy-head shape.

An open request with `authority_profile_id` receives a
`selected_semantic_profile` action schema containing only that profile's
semantic channels. A prepared composition may instead set
`startup_authority_profile_id`; an explicit open request must agree with that
selection. Omitting both retains the legacy `native_union` schema for older
canonical episodes, while low-fidelity fixtures that explicitly opt into
default selection open their advertised default profile. Action
and observation channels carry type, shape, quantity, unit, frame, bounds,
sampling semantics, and explicit value space. Selected-profile steps separate
requested and applied semantic values from lowered adapter values and lowering
evidence. Observations include lifecycle, sequence, time, events, diagnostics,
spawned entity IDs, and inspectable control ownership. The current native
episodes advertise no interactive child generation; that absence remains
explicit.

Runtime support is a second, changing contract rather than an inference from
the model or fidelity label. `observation.control_authority` reports the active
scheme, current phase when the provider exposes one,
`runtime_availability`, stable reason codes, `available_action_ids`, and
per-action unavailability reasons. An empty `applicable_phase_ids` means that
no static phase restriction is declared; it does not promise that resources,
faults, or terminal state can never make the profile unavailable. A provider
with phase- or resource-dependent authority implements
`control_authority_availability(profile_id, observation)` and returns the
current status and, when only part of a profile is usable, its exact
`available_action_ids`. The common manager refuses only requested channels
outside that mask, includes their reason mapping in the structured error, and
refuses a transfer into an unavailable profile.

For every registered caller-controlled `step` tuple, advertisement conformance
requires active semantic action channels with exact native or declared adapter
binding schemas. The vertical session suite opens every episode witness and
checks profile selection, identity, type, shape, quantity, unit, frame, bounds,
value-space topology, lowering evidence, and state-continuous transfer. A
provider-controlled session may instead advertise a zero-action profile. CADAC
uses that form to identify its retained source program and must not manufacture
an external action seam.

## Control readback, feedback, and failures

Every selected-profile step returns `control_feedback` in active-schema order.
Each row repeats the semantic channel and unit, distinguishes a newly requested
value from a held value, records the applied value when one exists, and uses a
closed disposition vocabulary:

| Disposition | Meaning |
| --- | --- |
| `not_commanded` | No new or held semantic value was applied for this channel |
| `held` | A previously accepted value remained active |
| `applied_as_requested` | The accepted semantic value equals this request |
| `limited` | The applied semantic value differs from the request |
| `withheld` | The provider accepted the frame but suppressed this command or event |
| `unavailable` | Runtime phase, resources, fault state, or lifecycle blocked it |

An action channel may declare a same-unit, same-shape
`feedback_channel_id`. When that committed observation exists, the feedback
row reports `achievement_status="observed"` and its achieved value. Otherwise
it reports `not_observed`; it never copies the request and calls that
achievement. This makes throttle exhaustion, saturation, waypoint progress,
and similar behavior machine-readable as providers add truthful status and
resource bindings. The current analytical waypoint fixture binds kinematic
and waypoint coordinates to committed position/velocity/attitude truth, and
Simple Aero binds direct throttle to its committed realized throttle. A
capture radius has no achieved coordinate and therefore remains explicitly
`not_observed`.

`MissionCompositionDiagnostic` is the common feedback vocabulary. It includes:

- severity: `error`, `warning`, or `info`;
- lifecycle phase: `discovery`, `configuration`, `preflight`, `execution`, or
  `projection`;
- recoverability: `correctable`, `retryable`, `degraded`, or `fatal`;
- stable kebab-case code and user-facing message;
- optional JSON-pointer path and corrective hint; and
- provider, model, object, and segment identities plus JSON-safe details.

The failure category tells a backend how to route the response:

| Category | Meaning |
| --- | --- |
| `invalid_request` | The caller can correct the request or configuration |
| `unsupported` | The requested model/fidelity/operation is not advertised |
| `unavailable` | It is advertised but temporarily unavailable; retryability is explicit |
| `execution_failed` | Execution started but no standard trajectory is available |
| `provider_error` | The provider violated the common boundary or failed without a public diagnostic |

Known configuration and Pydantic validation errors are normalized into
structured diagnostics. Providers may raise `MissionCompositionExecutionError`
to supply an exact public diagnostic. Unknown exceptions become the safe
`provider-internal-error`; only the exception type is placed in public details.
The private message and traceback stay in backend logs correlated by request ID.

Use a failure response when no usable trajectory exists. Use a `partial` or
`failed` trajectory result only when useful accepted samples exist; that result
must include at least one error diagnostic.

## Standard multi-object trajectory

`MissionCompositionTrajectoryResult` is an object graph, not one flat state
matrix:

```text
MissionCompositionTrajectoryResult
├── primary_object_id
├── objects[]
│   ├── primary vehicle history
│   ├── spent booster history
│   └── detached/tumbling child history
├── events[]
├── relationships[]
│   └── parent + child + deployment + event + initial state
└── diagnostics[]
```

Every `TrajectoryObject` has its own model and realization IDs, role, fidelity,
status, active interval, channel metadata, finite samples, segment spans,
provenance, and claim boundary. A child additionally names `parent_object_id`,
`deployment_id`, and `spawn_event_id`. The spawn event identifies the child as
its subject, the parent, the deployment, and the accepted spawn time. A corresponding
`TrajectoryEntityRelationship` records the relationship kind, state-transfer
policy, and a typed initial-state snapshot that must exactly match the child's
first returned sample. A death event may close the active interval
independently; a child may outlive its parent.

The contract validates:

- unique object, event, channel, and segment-instance IDs;
- exact sample/channel primitive type and shape, finite numeric values, and
  JSON-safe provider telemetry;
- monotonic accepted time and exact active-interval endpoints;
- valid parent, spawn, death, event, and segment references;
- deployment-category spawn events and consistent deployment identity;
- exactly one spawn relationship for each child, with matching event, time,
  parent, deployment, and initial state;
- child activation within the parent's active lifetime;
- acyclic parent/child lineage; and
- consistency between overall status, active objects, and diagnostics.

`in_progress` supports a streaming or nonterminal provider result with active
objects; interactive sessions return their own committed observations.
`completed` and `terminated` are successful terminal outcomes. `partial` and
`failed` retain accepted samples while carrying error diagnostics.

Channels declare primitive type, shape, quantity, unit, frame, sampling and
interpolation rules, and description. This lets plotting, comparison, replay,
and storage code consume every object without knowing the provider's private
state vector.

`trajectory_result_from_run_artifact()` is the standard Simulation Runtime
bridge. `VehicleTelemetry` preserves each runtime `model_id` and
`parent_model_id`; committed spawn events provide the child object, parent
object, accepted time, and deployment event ID. `RuntimeArtifactProjection`
adds the provider/request/configuration identity, fidelity, roles, status, and
claim boundary that a generic runtime artifact cannot infer. Sparse channels
are omitted with structured warnings; lineage gaps fail closed.

## Advertisement conformance

`audit_provider_advertisement(provider, runner=None)` loads every schema and
produces a model-by-model report. It verifies:

1. provider metadata serialization, model count, and unique model IDs;
2. model metadata serialization and model/version/schema/fingerprint identity;
3. configuration- and output-schema serialization with fingerprint-preserving
   JSON round-trips;
4. advertised core/telemetry/entity metadata and configuration-node/value-type
   coverage;
5. parameter frames against the model's reference-frame publication;
6. schema fidelity references against declared model fidelities;
7. deployment trigger segments, child identities, and operations against the
   parent publication;
8. every available operation, realization, output channel, and deployment
   against exact common-runner registration; and
9. exact agreement between model-level operation indexes and the underlying
   mission tuple records.

The separate authoritative inventory reconciles every source catalog identity,
production model, non-compatibility realization, stage/component, generated
child, debug fixture, and explicit exclusion. It rejects debug leakage,
dangling realization or child IDs, and catalog/registry drift. The completion
report joins that audit to exact batch and session registrations and generates
the checked-in
[family/realization coverage matrix](mission-composition-coverage-matrix.md).

Run the registry audit with:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --audit
python tools/dev.py mission-composition-completion
```

The advertisement audit proves publication integrity. The exact-tuple
integration tests prove common adapter execution. Neither establishes model
qualification.

## Repository implementations

`RegistryMissionCompositionProvider` projects all nine canonical physical
vehicle families, the `simple_aero` workflow, and the dual-launch glider
family from existing authorities. It does not maintain a second model catalog.
The Simple Aero schema publishes
launch and endpoint choices, mass/boost/aero inputs, checkpoints, open segment
composition, named maneuver templates, and an explicit point-mass-only fidelity
boundary. Every fixed-L/D template has a common batch binding and a registered
persistent point-mass session. The session reuses the existing Simple Aero
reference kernel: its default generated schedule is provider-owned and accepts
no caller action, while the alternate direct-throttle profile lowers to the
native `command.throttle` coordinate. Generated bank remains visible telemetry,
not an invented steering input. Both
dual-launch forms have a source-generated point-mass batch binding; they remain
truthfully limited to one continuous primary trajectory and an event-only
separation boundary, without independent attached-stack or released-glider
histories.

`ReferenceMissionCompositionProvider` is the runnable interface witness. It
publishes the analytical ballistic and constant-velocity waypoint models,
validates the portable configuration, registers common batch and persistent
step executors, and converts their output to the standard trajectory response.
Ballistic stepping is explicitly zero-action open loop. Waypoint stepping
defaults to its configured provider guidance and can switch, without resetting
time or state, to caller-owned kinematic velocity or live-waypoint authority.
Run it with:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_reference.py \
  --output /tmp/mission-composition-trajectory.json
```

`ContractProbeMissionCompositionProvider` is the full-surface development
witness. It publishes every metadata/configuration/deployment shape, returns a
three-generation lineage batch result, demonstrates a structured rejected
request, and provides coarse/medium streaming profiles for typed continuous,
discrete, and event controls:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py \
  --output /tmp/mission-composition-contract-probe.json
```

## Provider implementation checklist

1. Publish stable provider/model IDs and version every schema independently.
2. Model configuration structurally with `Choice`, `Group`, `Sequence`, and
   `Optional`; do not publish callbacks or legal-combination prose.
3. Publish typed read-only model properties, reference frames, a complete
   core/telemetry output schema, canonical/display units, and
   non-authoritative presentation hints.
4. Publish canonical units, bounds, periodicity, topology, defaults, provenance,
   and fidelity compatibility for every parameter.
5. Publish exact mission/fidelity/operation availability and blockers.
6. Publish every child-emission capability, including event-only boundaries.
7. Validate into an immutable prepared configuration before execution.
8. Register the exact provider/model executor with the common runner.
9. Adapt interactive execution through open/inspect/step/reset/close sessions;
   never substitute a stateless step request.
10. Raise structured public errors and keep private traces in correlated logs.
11. Return one object history per independently propagated body with explicit
    lineage events, a first-class relationship record, and an initial-state
    snapshot at the accepted spawn boundary.
12. Add every source asset and non-compatibility realization to the inventory.
13. Run the advertisement, inventory, exact-tuple, serialization, output, and
    lineage audits while preserving each underlying claim boundary.

Mission Composition owns discovery and mission assembly. Model Authoring owns
source meaning and source diagnostics. Simulation Runtime owns propagation,
accepted truth, stepping, and artifacts. A successful common response does not
promote fidelity evidence or establish vehicle qualification.
