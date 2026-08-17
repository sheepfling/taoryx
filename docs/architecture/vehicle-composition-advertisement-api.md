# Vehicle Composition Advertisement API

The Vehicle Composition Advertisement API is a first-class Taoryx product.
It gives a catalogue, planner, UI, orchestration service, or agent a
provider-neutral answer to one question:

> What forms of mission and vehicle composition does this exact model support,
> for which operations, missions, fidelities, and realizations?

The API can be used independently of the Taoryx simulation runtime. A backend
may wrap a native simulator, a remote trajectory service, an analytical model,
recorded trajectory data, or another execution engine. It does not need to use
Taoryx dynamics internally. It must publish truthful authoritative model
metadata and obey the common advertisement semantics.

The canonical Python surface is
[`taoryx.trajectory.mission_composition`](../../src/taoryx/trajectory/mission_composition.py).
The standalone wire artifact has schema ID
`taoryx.trajectory-composition-advertisement/v1`.

## Product boundary

The advertisement is the capability-negotiation layer of the broader
[Mission Composition Provider API](mission-composition-provider-api.md):

```text
provider-owned facts
  mission operations + control authorities + deployments
  + output entity semantics + fidelity transitions
                              |
                              v
              canonical composition projection
                              |
                              v
       TrajectoryCompositionAdvertisement (JSON-safe)
             /                 |                 \
            v                  v                  v
      catalogue/UI       planner/agent      backend gateway
            |                  |                  |
            +---------- exact selection ----------+
                              |
                              v
            provider validation and execution API
```

The advertisement answers discovery questions. It does not execute a mission,
authorize an unchecked mutation, replace the configuration schema, or replace
the provider's runtime validation. A consumer uses it to hide impossible
choices and identify candidates, then submits an exact configuration or
session request through the provider protocol.

This separation is intentional:

- provider-neutral consumers do not inspect private dynamics classes;
- backends do not invent a different capability dictionary for each UI;
- unsupported behavior is explicit instead of omitted;
- discovery does not construct or run a plant; and
- execution remains owned by the backend that advertised the exact operation.

## Normative v1 artifact

`TrajectoryCompositionAdvertisement` contains:

| Field | Meaning |
| --- | --- |
| `schema` | Exact contract ID; v1 is `taoryx.trajectory-composition-advertisement/v1` |
| `features` | The complete, canonically ordered v1 feature partition |
| `claim_boundary` | Plain-language limit on what the model-level advertisement proves |

Every `features` entry is a `TrajectoryCompositionFeatureMetadata` record:

| Field | Meaning |
| --- | --- |
| `category`, `id` | One key from the closed v1 vocabulary |
| `status` | `available`, `conditional`, `declared`, `blocked`, or `not_available` |
| `operations` | Exact common operations: `validate`, `batch`, and/or `step` |
| `mutation_timing` | When the behavior may occur |
| `mission_template_ids` | Mission templates to which the row is narrowed |
| `fidelity_ids` | Fidelity selections to which the row is narrowed |
| `realization_ids` | Model realizations to which the row is narrowed |
| `requirements` | Conditions a consumer or provider must still resolve |
| `blockers` | Reasons a known feature cannot currently be used |
| `claim_boundary` | What the individual row does and does not claim |

The supported mutation timings are `configuration`, `preflight`,
`between_segments`, `accepted_boundary`, `session_boundary`, and `in_step`.
They distinguish authoring an input before execution from changing a live
runtime graph or transferring state at an accepted integration boundary.

### Status semantics

| Status | Consumer meaning |
| --- | --- |
| `available` | The feature is supported for at least one listed operation and scope. Exact operation validation and any stated requirement still apply. |
| `conditional` | The feature is usable only for the listed scope and after satisfying any `requirements`. |
| `declared` | The provider recognizes and describes the concept, but publishes no executable common operation for it. |
| `blocked` | The concept is known but unusable for the reasons in `blockers`. |
| `not_available` | The model makes no provider-neutral v1 claim for this feature. Applicability fields are empty. |

An empty mission, fidelity, or realization selector means that the feature row
does not add a restriction along that dimension. It never overrides the exact
mission-operation matrix, configuration schema, control authority, deployment,
or session descriptor. Consumers must intersect non-empty selectors and still
ask the provider to validate the resulting exact request.

## Closed feature vocabulary

Every v1 advertisement contains every key below exactly once and in this
canonical category order. This is a capability partition, not a snapshot of
only the positive rows.

| Category | v1 feature IDs |
| --- | --- |
| `authoring_mode` | `fixed_template`, `caller_ordered_sequence`, `caller_authored_graph`, `provider_generated_graph` |
| `execution_mode` | `batch`, `stateful_session`, `provider_controlled_program`, `caller_controlled_actions`, `multi_entity` |
| `graph_form` | `linear_sequence`, `directed_acyclic`, `conditional_branching`, `cyclic`, `parallel_fork_join`, `nested_subgraph`, `runtime_mutable` |
| `node_kind` | `segment`, `decision`, `fork`, `join`, `deployment`, `synchronization`, `terminal`, `subgraph`, `external` |
| `transition_kind` | `success`, `timeout`, `abort`, `resource_limit`, `envelope_limit`, `event`, `condition`, `manual` |
| `rearrangement` | `reorder`, `insert`, `remove`, `replace`, `duplicate`, `rewire`, `enable_disable`, `parameter_patch`, `bind_deployment` |
| `runtime_transition` | `authority_switch`, `fidelity_switch`, `mode_switch`, `graph_patch`, `model_migration` |
| `entity_topology` | `spawn`, `attach`, `detach`, `split`, `merge`, `replace_child`, `recursive_spawn` |
| `state_transfer` | `previous_terminal_truth_state`, `accepted_boundary_snapshot`, `state_continuous_reference_handoff`, `fidelity_projection`, `provider_defined` |

The vocabulary is deliberately broader than the capabilities of the current
model catalogue. For example, a linear fixed mission still publishes explicit
`not_available` rows for conditional branching, arbitrary rewiring, graph
patches, and model migration. A generic client therefore does not need a
provider-specific convention for missing fields.

## Consumer usage

The advertisement is embedded in every `TrajectoryModelMetadata` returned by
`provider.list_models()`:

```python
from taoryx.plugins import discover_plugins

providers = discover_plugins().build_mission_composition_provider_registry()
provider = providers.provider("taoryx.registry.mission-composition")
model = provider.model("simple_aero")

composition = model.composition_advertisement
open_sequence = composition.feature("authoring_mode", "caller_ordered_sequence")

if open_sequence.status in {"available", "conditional"}:
    print(open_sequence.operations)
    print(open_sequence.mission_template_ids)
    print(open_sequence.requirements)
```

The `.feature(category, id)` helper performs an exact lookup and raises
`KeyError` for an unknown feature. Code that receives serialized data can
perform the equivalent lookup over `features` after validating the artifact.

A safe generic decision procedure is:

1. Require the exact schema ID the consumer supports.
2. Locate the exact category/feature row.
3. Reject `blocked` and `not_available`; treat `declared` as descriptive only.
4. Require the requested operation to appear in `operations`.
5. Intersect every non-empty mission, fidelity, and realization selector.
6. Satisfy and present any `requirements` and retain the `claim_boundary`.
7. Resolve the exact authoritative operation record and ask the provider to
   validate the complete request.

Do not collapse the five statuses into a boolean. Doing so loses the
difference between a concept that is absent, known but blocked, merely
declared, and conditionally executable.

## Provider implementation

Backends should not manually maintain the normalized matrix. Publish the
authoritative records first, then derive the advertisement:

```python
from taoryx.trajectory.mission_composition import (
    TrajectoryModelMetadata,
    build_trajectory_composition_advertisement,
)

composition = build_trajectory_composition_advertisement(
    capabilities=capabilities,
    realizations=realizations,
    mission_templates=mission_templates,
    deployments=deployments,
    output_schema=output_schema,
    fidelity_transitions=fidelity_transitions,
)

model = TrajectoryModelMetadata(
    # identity, presentation, schemas, fidelities, and other model fields ...
    capabilities=capabilities,
    realizations=realizations,
    mission_templates=mission_templates,
    deployments=deployments,
    output_schema=output_schema,
    fidelity_transitions=fidelity_transitions,
    composition_advertisement=composition,
)
```

`TrajectoryModelMetadata` also derives the field when it is omitted during
Python construction. On validation it recomputes the canonical projection and
rejects a stale or more optimistic serialized matrix. Explicit construction,
as above, is useful when the provider wants to inspect or persist the artifact
before assembling the complete model record.

The derivation authorities are:

| Composition claim | Authoritative source |
| --- | --- |
| Batch and stateful execution | Exact mission/fidelity/realization operation rows |
| Caller- or provider-controlled behavior | Realization control authorities and command ownership |
| Fixed versus caller-ordered authoring | Mission template and open-sequence grammar |
| Deployment and child topology | Deployment records plus entity-output metadata |
| Multi-entity output | Entity-output capability, distinct from dynamic spawning |
| Authority handoff | Explicit bumpless authority switching records |
| Fidelity selection or transfer | Fidelity-transition status and state-transfer declaration |

This projection can make a claim narrower, but it cannot promote a provider
fact. In particular:

- a fixed linear sequence is not advertised as a caller-authored DAG;
- an open sequence permits only its declared pre-run edits;
- static provider-owned multiple roots do not imply dynamic spawning;
- an accepted-boundary child deployment does not imply arbitrary attachment,
  merging, or recursive generation;
- selecting another fidelity for a new run does not imply a live fidelity
  switch; and
- two control profiles do not imply a runtime handoff unless the transition is
  explicitly state-continuous and bumpless.

### Implementing a non-Taoryx backend

An independent Python backend can depend on the core `taoryx` package for the
Pydantic contracts while keeping its solver in another distribution or
process. It implements `ConfigurableTrajectoryProvider`, returns
`TrajectoryModelMetadata`, validates configurations, and optionally registers
batch and session executors. The provider package is discovered through the
normal plug-in entry point; its dynamics never need to be imported by a
generic consumer. The packaging and registration procedure is documented in
[Authoring a vehicle plug-in](vehicle-plugin-authoring.md).

A service written in another language can publish the same JSON artifact. The
generated JSON Schema validates field shape, enums, and additional-property
closure. The service must additionally enforce the semantic invariants that
JSON Schema cannot express conveniently:

- exactly one row for every v1 vocabulary key;
- canonical row order;
- at least one operation for `available` and `conditional` rows;
- at least one blocker for `blocked` rows;
- empty applicability fields for `not_available` rows; and
- agreement with the model's authoritative operation, control, deployment,
  entity-output, and fidelity-transition records.

Export the structural JSON Schema from the reference implementation with:

```python
import json

from taoryx.trajectory.mission_composition import TrajectoryCompositionAdvertisement

print(json.dumps(
    TrajectoryCompositionAdvertisement.model_json_schema(by_alias=True),
    indent=2,
    sort_keys=True,
))
```

HTTP routes, authentication, streaming transports, and service discovery are
not normative parts of v1. A gateway may place the artifact in a REST, gRPC,
message-bus, or file envelope without changing its field semantics or schema
ID.

## Serialization and compatibility

Use aliases at a JSON boundary:

```python
payload = composition.model_dump_json(by_alias=True)
validated = TrajectoryCompositionAdvertisement.model_validate_json(payload)
```

The Python models are frozen and reject extra fields. Consumers should fail
closed on an unknown schema ID. The following require a new advertisement
schema version rather than an untyped extension:

- adding, removing, or redefining a category or feature ID;
- adding a status, operation, or mutation timing;
- changing selector or claim-boundary semantics; or
- weakening an invariant that affects generic consumer behavior.

A provider may change which existing v1 rows are available as its backend
matures, but it should advance its provider/model version and preserve the
new advertisement with the run or audit artifact that relied on it. Free-form
requirements and claim boundaries may explain provider constraints; they must
not smuggle in a new graph primitive or mutation operation.

## Conformance and audit

`audit_provider_advertisement(provider, runner)` checks the model publication
and reports the composition schema ID, total feature count, and counts for all
five statuses. Model validation and the audit together test the publication;
they do not establish physical model validity or numerical fidelity.

A conforming provider should test that:

1. discovery does not instantiate or execute the plant;
2. every model round-trips through JSON with aliases;
3. every model has the exact exhaustive v1 partition;
4. the canonical projection equals the serialized advertisement;
5. every advertised executable tuple has the claimed runner/session binding;
6. blocked and unavailable features fail closed; and
7. installing another valid plug-in adds models without invalidating fixed
   provider-inventory assertions.

Run the executable inspection and schema-export example from
[`examples/trajectory_provider/composition_advertisement.py`](../../examples/trajectory_provider/composition_advertisement.py),
and use [`taoryx model overview`](vehicle-model-cards.md) for a reviewer-facing
composition table alongside fidelity, provenance, tuning, and parameters.

## What v1 does not claim

The advertisement is not itself a graph document, graph execution engine,
scenario scheduler, control protocol, or certification statement. It does not
claim that every vocabulary item is implemented by Taoryx models. It records
the exact current boundary so another backend can implement more of that
vocabulary without changing how a generic client asks the question.

That is the core interoperability promise: common semantics at discovery and
validation boundaries, independent provider implementations behind them.
