# Authoring a vehicle plug-in

This guide is for developers adding a vehicle family or a family-owned control
surface to Taoryx. It answers a narrower question than the general
[family-integration playbook](../plan/generic-family-integration-playbook.md):

> How does a package make one vehicle's data, configuration, controls, batch
> execution, and optional live episode visible to the Taoryx host without
> leaking vehicle-specific knowledge into a consumer?

The answer is deliberately vertical. A vehicle plug-in owns the facts and
runtime seams for its family; core owns the generic schemas, discovery,
registry aggregation, session lifecycle, and result envelopes. A client then
discovers the vehicle and renders only the controls that the plug-in has
truthfully advertised.

Start from the direct developer profile:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
taoryx plugins check --profile developer
```

If you have not yet decided whether your work belongs in this internal
TAORYX-host path or in the standalone external provider/consumer contract,
start with [Developer interface layers](interface-layers.md).

That profile intentionally excludes `taoryx-reference-models`. The aggregate
is a migration surface for older catalogue consumers, not a dependency or a
reference implementation for a new plug-in. Use an owning focused provider in
examples, tests, and documentation.

For package ownership and the current split rationale, see
[Vehicle plug-in isolation](../architecture/vehicle-plugin-isolation.md). For the consumer
contract that this guide produces, see the
[Mission Composition front door](../MISSION_COMPOSITION.md).

## What a vehicle plug-in owns

A standalone vehicle package should own every family-specific input needed to
make an advertised endpoint real:

- source assets, resource loading, hashes, provenance, and claim boundaries;
- family/fidelity/composition/endpoint catalog fragments and checked-in
  witnesses;
- model adapter, semantic preflight, configuration-to-composition lowering,
  and exact batch or episode factories;
- semantic control metadata, authority profiles, native bindings, lowering,
  committed status, resource limits, and readback; and
- any family-owned controller campaigns, batch/episode parity evidence, and
  focused vertical tests.

The package must not depend on a compatibility aggregate merely to advertise
or run itself. It loads packaged data through `importlib.resources`, not by
walking from the source-checkout root. An endpoint can be discoverable while
blocked or batch-native, but a runnable operation must name an exact registered
factory. Taoryx never substitutes a nearby vehicle or factory; a requested
fidelity can resolve lower only through its explicitly declared, validated
fallback policy.

## Keep documentation with its owner

Every independently installable package has a package-root `README.md`; that
is its required documentation landing page. Put longer model-specific material
under `packages/<distribution>/docs/`: source-deck limits, evidence/resolver
rules, family control semantics, model architecture, calibration notes, and
package-owned witnesses. Add the package to the
[plug-in documentation directory](../plugins/README.md).

Do not put those facts in `docs/architecture/` or duplicate shared contracts
inside a package. A reusable host/provider/control/result contract belongs in
the [API reference](../api/README.md); a shared TAORYX design belongs in
`docs/architecture/`. Before handoff, run:

```bash
python tools/dev.py docs-layout
python tools/dev.py plugin-focus <wheel-selector>
```

The first command checks placement, migration pointers, and local Markdown
links across repository and package documentation. The second prints only the
selected package's discovery and evidence loop.

## Start with one vertical slice

Do not begin by publishing a broad control catalogue. Start with one exact
combination:

```text
family + model + mission + realization + fidelity + operation
    -> typed configuration and public interface
    -> one registered batch factory and, when applicable, a native episode factory
    -> checked-in witness and focused verification
```

Every TAORYX-owned executable model has the common `batch` and `step`
surfaces. Batch is a bounded run that returns a standard artifact. A provider
may additionally own a live, persistent episode that can be opened, inspected,
stepped, reset, and closed with caller actions. Where a TAORYX provider has
only a registered batch factory, core exposes that result through an explicitly
read-only replay session: its `step` advances through committed truth
boundaries and has no action schema. The reusable external-provider interface
also admits an honestly batch-only provider; it must advertise that capability
instead of claiming a step seam. Publishing live actions without lowering or
runtime behavior is invalid; a replay session must not advertise actions at
all.

### Publish a provider-selected runnable default

An authoring schema must leave genuine launch, mission, and endpoint decisions
required; never hide those decisions behind invented schema defaults. Every
TAORYX-hosted model nevertheless needs one deterministic, preparable,
provider-selected runnable default so a generic consumer can prove discovery,
preparation, batch, and step without manufacturing values.

If ordinary schema defaults already compile, core derives the runnable default.
If the model has required choices, implement this small provider-owned method
and use a checked-in fixture/witness or an equally traceable package builder:

```python
def build_model_default_configuration(
    self,
    model_id: str,
    *,
    configuration_id: str,
) -> TrajectoryConfigurationInstance:
    # Select one exact advertised model/mission/fidelity/realization and return
    # a normal configuration that provider.validate_configuration accepts.
    ...
```

The common adapter exposes it as
`DefaultConfigurationProvider.build_default_configuration(model_id)`. Its
stable configuration ID is `<model-id>-default`; it is a provider-selected
runnable starting point, not a physical nominal condition, calibration, or
qualification claim. Verify it without widening to every package:

```bash
taoryx model default <provider-id> <model-id> --output default.json
python tools/validate_mission_composition_provider_contract.py \
  --plugin <plugin-id> --contract-profile taoryx-universal --execute-defaults
```

`--execute-defaults` verifies one public streaming step for every model in the
selected plug-in. Add `--execute-batch-defaults` for that plug-in's full public
batch routes when those routes are deliberately bounded; a longer physical
mission should instead keep its batch evidence in its own vertical test.

The [A320 plug-in](../../packages/taoryx-a320/src/taoryx_a320/plugin.py),
[F-16 plug-in](../../packages/taoryx-f16/src/taoryx_f16/plugin.py), and
[Hummingbird plug-in](../../packages/taoryx-hummingbird/src/taoryx_hummingbird/plugin.py)
are the current reference vertical slices. Each owns an isolated Mission
Composition provider, package data, exact batch and episode factories where
the endpoint supports an episode, preflight callbacks, control campaigns, and
batch/episode parity verifier where parity is claimed. A320 additionally
shows a native batch-screen boundary: its named-coordinate LQI screen owns an
executable definition for core preflight/batch dispatch and an identical
static local-screen advertisement for UI and agent plans; core still gives it
the read-only standard session when no native live episode is registered.
F-16 demonstrates a
source-model dependency on DAVE-ML while retaining a reduced 3DOF/pseudo-6DOF
control surface separate from its bounded local physical-controller screens.

Plug-in runtime code must depend only on public core seams. In particular,
`native_action_for_frame` and `status_frame_for_observation` bind a
package-owned episode or parity verifier to the resolved interface contract,
while `preflight_powered_fixed_wing_racetrack` provides the shared geometric
preflight for a family-owned standard racetrack translator. Core helpers whose
names begin with `_` are implementation details, not extension points.

## Select the fidelity contract before writing the model

Every realization must use one of four explicit tiers:

```text
point_mass_3dof
    -> pseudo_6dof
    -> rigid_body_6dof_direct_wrench
    -> rigid_body_6dof_surface_allocated
```

The ladder names both equations and control realization. `pseudo_6dof` is a
named response model, not automatically physical attitude dynamics. The two
rigid-body tiers both integrate Newton–Euler translation and rotation, but
direct wrench applies a generalized force/moment while surface allocated
requires actual bounded effectors and allocation evidence. Do not publish the
ambiguous legacy label `rigid_body_6dof`.

Use [Fidelity tiers and vehicle plug-in requirements](../architecture/fidelity-data-requirements.md)
as the per-tier definition of done. It distinguishes a contract-complete
advertisement, data readiness for runtime probes, and qualified/promoted
execution, and lists the required physics, controls, status, and evidence for
each tier. Fidelity selection never gives a plug-in permission to relabel a
lower-tier control as a physical actuator.

## Publish data as a typed coordinate contract

An ID and a scalar are not an API. For each configuration value, action,
effector, observation, status value, and output channel, publish the following
as a single contract:

| Field | Required declaration | Important distinction |
| --- | --- | --- |
| Identity and role | Stable ID; configuration, action, effector, status, output, resource, or diagnostic role | A configuration parameter is fixed before a run; an action is a runtime command; status/output is readback, not an input echo. |
| Storage type and shape | `float64`, `int64`, `boolean`, `string`, or `json`; `()` for a scalar, `(3,)` for a 3-vector, `(m, n)` for a tensor, and `"variable"` only for an explicitly variable axis | Shape is the serialized coordinate shape, not a claim about physical degrees of freedom. |
| Quantity and units | Quantity meaning plus a canonical unit; an optional display unit is presentation only | Numeric values use canonical units at the provider boundary. Boolean, string, and JSON values have no unit. |
| Coordinates | A declared reference frame, axes/order, origin, orientation, and handedness whenever the value is geometric | `body_force[0]` is not portable until its body-axis order and sign are declared. |
| Value space | Topology, representation, error rule, interpolation rule, normalization/equivalence rule, and bounds or finite choices | Degrees alone do not say whether a value is a heading on a circle or a signed pitch rate on a line. |
| Time and authority | Sampling/hold semantics, command mode, repeat/release policy, availability, operation, and authority owner | A continuous slider, latched mode, and one-shot event have different lifecycle rules. |
| Provenance and realization | Source/provenance, claim boundary, exact native binding, and lowering/readback relationship | A semantic action is only usable when it has a concrete model seam. |

The public metadata accepts the five storage types above. `shape=()` is a
scalar; a shape axis must be positive unless it is the explicit
`"variable"` marker. Non-numeric channels cannot advertise canonical units.
An interactive control also needs a `native_channel_id` and native binding with
the same type, shape, unit, interval/value space, and command semantics.

Do not infer independent dimensions from storage. A quaternion has shape
`(4,)` but represents a three-degree-of-freedom rotation with `q` equivalent
to `-q`; its value space is `rotation_group_so3`, not a four-dimensional
Euclidean vector. Likewise, an output can be a dependent/derived value even
when it is scalar. The plug-in must name the independent plant and command
coordinates in its adapter and describe a derived channel's source in its
description/provenance and claim boundary. A consumer uses the selected
authority profile to learn which independent action coordinates it owns now;
it must not treat all outputs, telemetry, or redundant state coordinates as
separately commandable degrees of freedom.

The authoritative value-space rules, including circular angles, quaternions,
mixed tuples, and the controller error/interpolation implications, are in
[Public value-space contract](../api/public-value-spaces.md). The exact API fields
are [`TrajectoryControlChannelMetadata` and
`TrajectoryOutputChannelMetadata`](../../src/taoryx/trajectory/configuration_contract.py).

## Package and discovery skeleton

Declare one standard Python entry point whose name exactly matches the plug-in
metadata ID:

```toml
[project.entry-points."taoryx.plugins"]
"acme.tiltrotor" = "taoryx_acme_tiltrotor.plugin:PLUGIN"
```

Keep registration light. Discovery is allowed to import the plug-in module and
collect stable identities, but it must not construct a plant, parse a large
asset, or run a trajectory. Use the factory forms below when a contribution
would otherwise trigger numerical imports or catalog parsing.

```python
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment


def _register(registrar: PluginRegistrar) -> None:
    registrar.register_vehicle_catalog_fragment(
        VehicleCatalogFragment(
            id="acme.tiltrotor.vehicle-catalog",
            resource_package="taoryx_acme_tiltrotor",
            family_ids=("acme_tiltrotor",),
        )
    )
    family = registrar.register_family_adapter_factory(
        "acme_tiltrotor", build_family_adapter_registration,
    )
    registrar.register_model("acme_tiltrotor", family)
    registrar.register_mission_composition_provider_factory(
        "acme.tiltrotor.mission-composition", build_provider,
    )

    # Only a source-backed family with a declared trim worklist needs this.
    registrar.register_trim_evidence_binding_factory(
        "acme_tiltrotor_source", build_trim_evidence_binding,
    )

    # Required only when the selected composition advertises the matching seam.
    registrar.register_semantic_preflight_handler_callback(
        "acme.tiltrotor.conversion.v1", preflight_conversion,
    )
    registrar.register_execution_factory_request_v1(
        "acme_tiltrotor_conversion_batch.v1", run_batch,
    )
    registrar.register_episode_factory(
        "acme_tiltrotor_conversion_episode.v1", open_episode,
    )
    registrar.register_batch_episode_parity_verifier(
        "acme.tiltrotor.conversion.parity.v1", verify_batch_episode_parity,
    )


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="acme.tiltrotor",
        package="taoryx-acme-tiltrotor",
        version="0.1.0",
        api_version="1",
        description="Acme tiltrotor vehicle models.",
    ),
    register_callback=_register,
)
```

This is a shape, not a mandate to register every contribution. A package with
only an analytical Mission Composition provider may register only that provider;
a batch-native local screen needs no native episode factory because core supplies
its read-only replay session; a family with no tuning
campaign should not invent one. The identifiers must match the corresponding
family metadata, preflight records, execution bindings, and witnesses exactly.
The typed registrar and its versioned contribution surface are defined in
[`taoryx.plugins.contracts`](../../src/taoryx/plugins/contracts.py).

Every physical vehicle package should also register one
`VehicleCatalogFragment` rooted at its packaged `data/` directory. Its
identity is static discovery metadata; it does not parse the catalog during
discovery. When a caller passes a selected `PluginCatalog` to the generic
composition loader or compiler, Taoryx resolves only those declared package
resources and fails closed if a required file is absent. This prevents a
focused vehicle from silently borrowing a sibling's or compatibility
aggregate's catalog row.

An optional workflow package that augments a family must instead register a
`VehicleCatalogOverlayFragment` and name the base `VehicleCatalogFragment` it
extends. Overlay catalog documents are append-only: they can add new
initialization, segment, mission, or variant IDs, along with the matching
execution bindings and witnesses, but cannot rewrite the base family. The
generic resolver requires both packages in the selected `PluginCatalog`; an
overlay selected by itself fails closed. Keep the overlay's composition
requests and evidence in the optional package, while shared source assets stay
with the family that owns them. If a host selects the optional package for a
different family, an overlay whose base is absent remains inactive rather than
blocking that unrelated focused catalog.

Keep execution proof in that same package boundary. If
`vehicle_execution_parity.yaml` registers a batch/episode pair, include the
matching `vehicle_execution_parity_witnesses.yaml` rows and the composition
inputs they name. A family with no registered pair needs no placeholder
replay file—the selected resolver exposes an empty parity-witness catalog—but
it must never recover an unrelated trace from the compatibility aggregate.

A trim-evidence binding is a package-owned factory that returns a
`VehicleTrimEvidenceBinding`: it names the source family, package-relative
recipe/resource root, source adapter, and evaluator for the generic declared
worklist. Register it only when the family has source-grounded trim evidence.
The core aggregates its report; it must not hard-code the plug-in module name,
data path, or evaluator. A caller with a focused `PluginCatalog` passes that
same catalog to `solve_vehicle_trim_evidence`, preserving the selected package
boundary while the binding is resolved lazily.

For a batch-only native-coordinate LQI screen, make the two projections
explicit and keep them identical:

```python
registrar.register_local_native_coordinate_lqi_screen_definition(screen_definition)
registrar.register_local_controller_screen_advertisement(screen_advertisement)
```

The first registration is the typed executable definition that resolves a
specific family/mission/fidelity/initialization endpoint. The second is the
static authoring record with named controls, units, bounds, cadence, status
boundary, and claim boundary. Discovery rejects a missing, differently owned,
or metadata-mismatched pair. Neither registration turns internal named
coordinates into caller actions or physical effectors.

## Decide what a new control is before implementing it

The most important design step is classifying a proposed control correctly.
Do not use an action merely because a UI has a button, and do not use an
authority profile merely to decorate a label.

| If the change is… | Model it as… | Example |
| --- | --- | --- |
| Fixed for one prepared mission and invalid once execution starts | Configuration parameter | Initial payload, requested fidelity, initial handling preset |
| Continuously or periodically commanded during a session | Numeric/vector semantic action | Tilt angle, velocity reference, aggregate thrust, body-rate reference |
| A discrete state a caller can change while running | Boolean or enum action with explicit command lifecycle | Motors enabled, `handling.mode = gentle | sport`, landing-gear command |
| A single accepted occurrence, not a value to hold | One-shot event action | `maneuver.flip.trigger`, release, camera snapshot |
| A different coherent action surface, ownership model, or agent policy | Authority profile | Hover velocity/yaw controls versus airplane airspeed/bank controls |
| A family-internal behavior with no external command seam | Provider-generated control/status | Gain scheduling, source autopilot state, internal allocator choice |
| A real physical actuator coordinate | Effector action only when the selected realization actually models it | Nacelle angle, rotor speed, surface position |

Every external action declares primitive type, shape, canonical unit or choices,
bounds/value-space topology, temporal semantics, repeat policy, availability,
operation, and presentation metadata. It also needs a declared native binding
or lowering chain. The session descriptor projects that metadata to a UI and
to an AI/RL action-space representation; neither consumer should infer a
control from a channel name.

For a runtime mode, also decide whether it is held, latched, momentary, or
one-shot. Implement duplicate-event behavior deliberately. A `flip` trigger,
for example, needs explicit acceptance, repeat, resource, terminal-state, and
failure behavior; it must not be modeled as a boolean that silently stays true
forever.

## Design modes, transitions, and high-level controls honestly

A tiltrotor conversion is not a fidelity switch. Fidelity is resolved for the
prepared configuration under its explicit fallback policy and has no implied
live state transfer. A conversion is instead family runtime behavior: the
plugin must define its nacelle or mode command, transition state machine,
changing force/actuator behavior, limits, resource use, and committed
telemetry.

Choose the narrowest truthful surface:

- If hover, conversion, and airplane use the same action coordinates but
  different bounded response laws, a latched enum such as
  `flight.mode.command` can be appropriate. Publish its current state and the
  active limits/readback.
- If the client should command physical nacelle motion, expose the bounded
  nacelle effector only for a realization that actually models it. Preserve
  requested, actual, rate-limited, saturated, and failed values separately.
- If hover uses position/velocity/yaw while airplane uses airspeed, bank, and
  flight path, expose distinct authority profiles. A profile transition may
  be allowed only when both profiles declare `explicit_bumpless` and the
  episode implements a state-continuous transfer hook. Otherwise reject it or
  require a fresh session.
- If `gentle` and `sport` merely select response gains/limits over the same
  inputs, use a discrete mode action and publish the selected schedule. If
  sport changes attitude commands into rate/acro commands, use separate
  authority profiles because the action schema has changed.
- If a maneuver is source-owned or unsupported at the selected fidelity,
  publish no caller action. A visible `blocked` or provider-generated status
  is safer than an optimistic button that cannot affect the plant.

This same rule prevents a high-level waypoint controller from being mistaken
for a physical rotor allocator. A high-level action may lower through guidance
and response logic to a real plant seam, but the plug-in must expose that
lowering evidence and must not claim unmodeled effectors.

## Carry the control through the whole contract

Adding a row to a model catalogue is only the first step. For a caller-owned
control, all of the following must agree:

```text
model/realization control advertisement
    -> authority-profile membership and selected session action schema
    -> validation, command lifecycle, and runtime availability mask
    -> episode action handling and declared lowering chain
    -> native plant/controller/allocator behavior
    -> committed observation, control feedback, and diagnostics
    -> checkpoint/reset and, where advertised, batch/episode parity
```

At the session boundary, a step result keeps four distinct records:

- `requested_action`: values sent by the caller at this boundary;
- `applied_action` and `control_feedback`: accepted, held, limited,
  withheld, or unavailable semantic values, with achieved readback only when
  a real compatible observation exists;
- `lowered_action`: the values sent to the registered adapter or plant seam;
  and
- `lowering_evidence`: targets, active mode, limits, resources, and other
  adapter reasoning needed to interpret the lowering.

`observation.control_authority.available_action_ids` is the dynamic action
mask. Use it for phase, fault, terminal, or resource-dependent unavailability;
a statically advertised profile does not promise that every action is usable
now. Reset and checkpoints must restore the mode, held values, controller
state, resource state, and active authority so the returned sequence is
reproducible.

The authoritative rules and field definitions are in the
[Vehicle Interface Contract](../api/vehicle-interface-contract.md) and
[Mission Composition Provider API](../api/mission-composition-provider-api.md).
The independently reusable capability-negotiation layer and its canonical
derivation rules are in the
[Vehicle Composition Advertisement API](../api/vehicle-composition-advertisement-api.md).

## Choose validated lowering explicitly

**Fidelity lowering is neither a controller nor a live mode switch.** It is a
request-time choice of an already-declared lower model. It has two independent
opt-ins:

1. The family author permits it by setting `automatic_lowering: true` in both
   the horizontal-fidelity family record and its pseudo-6DOF profile binding.
   The catalogs are cross-checked, so a disagreement fails validation.
2. The caller requests `fallback_policy: validated_lower_only` and supplies a
   `minimum_acceptable` tier. A caller may always choose `exact_only`, even
   for a family that permits lowering.

With both opt-ins, the selector walks only downward from the requested tier
and accepts the first candidate with qualified profile evidence and every
required adapter operation. It returns the selected tier plus every considered
candidate and first blocker. It never selects an unqualified profile just
because it is lower fidelity. With `automatic_lowering: false`, with
`exact_only`, or when the checks fail, the requested model fails closed and
the caller must make a new explicit lower-tier request.

For a normal registry-backed family, the paired declarations look like this:

```yaml
# horizontal_fidelity_registry.yaml
families:
  - family_id: acme_tiltrotor
    automatic_lowering: true

# pseudo6dof_profiles.yaml
bindings:
  - family_id: acme_tiltrotor
    automatic_lowering: true
```

Turn both values off for an uncontrolled/passive model, for a source boundary
where tiers are not substitutes, or whenever the author cannot support a
validated reduction. Automatic lowering does not authorize a fidelity
upgrade, transfer a running state, change a VTOL flight mode, or apply a
controller. A different fidelity starts a newly prepared run. The user-facing
transition records and exact fallback semantics are described in [Fidelity is
a request graph, not a live model switch](../MISSION_COMPOSITION.md#fidelity-is-a-request-graph-not-a-live-model-switch).

## Controller choice: caller, provider, or common tuning

These are separate supported arrangements. A family may offer more than one,
but each authority profile must say which one owns the command:

| Arrangement | Public contract | What the plug-in supplies | What Taoryx does not assume |
| --- | --- | --- | --- |
| Bring your own controller | `command_owner: caller`; a caller-owned semantic, wrench, or effector profile is available for `step` | Exact action/native-binding schema, validation, limits, lowering to the plant, achieved readback, and checkpoint/reset behavior | That the caller uses LQR, that an advertised semantic input is a physical effector, or that caller gains are qualified. |
| Provider/source controller | `command_owner: provider_controller`, `source_program`, or `open_loop`; controls are internally generated | The controller/source-program identity, active-mode/status/readback, limits, and its claim boundary | That the controller is tunable by the common LQR/LQI host. |
| Common automatic campaign | A registered controller-tuning campaign, optionally exposing a local-controller screen | A family adapter plus immutable trim, derivative, coordinate, scale, authority, and campaign inputs; a runtime that applies an exact returned candidate | Missing physics, a generic gain for another vehicle, a nonlinear/physical winner, or qualification. |

For caller ownership, a gamepad, UI, Python callback, or AI policy simply
streams the selected profile's named actions. The plug-in is responsible for
the conversion to bank/rate/thrust/wrench/effectors and for exposing the
requested, applied, lowered, and achieved values. This is the correct route
for a joystick throttle/stick or a waypoint/velocity API. It does **not**
require a tuning campaign.

For a common campaign, register a
`ControllerTuningCampaignRegistration` with
`register_controller_tuning_campaign(...)`. Its adapter factory supplies a
bounded trim problem, true named state derivatives, control coordinates,
engineering scales, authority/actuator semantics, and any allocator or
nonlinear-response evaluator. Its campaign factory supplies the exact family,
fidelity, realization, mission, operating points, and profile grid. A
`ControlAutomationDeclaration` is the concise way to construct the common
single-node campaign: the plug-in still registers the resulting campaign and
adapter.

The numerical sequence is deliberately visible:

```text
declared trim and exact state/control order
    -> true derivative f(x, u) at that trim
    -> named finite-difference A = df/dx, B = df/du
    -> scaled LQR or LQI candidate grid and authority checks
    -> family nonlinear/allocator/actuator screen
    -> exact runtime application receipt and retained evidence
```

For LQR, the host solves the declared local quadratic regulator around the
trim. For LQI, the plug-in explicitly names the outputs whose persistent error
matters; the host augments only those with `z_dot = y - reference` and returns
both state and integral gains. A plug-in must never substitute a residual
Jacobian for true state derivatives, reorder the named coordinates, or label
a hand-tuned/scheduled gain as a returned candidate. When a batch/episode
actually applies a common candidate, it compares all coordinate names and
fingerprints and emits the `tuning_binding` receipt. Until then, the result is
only `candidate_ready`, not a runtime-bound controller.

The complete data and mathematical contract is in [Generic controller tuning
pipeline](../architecture/generic-controller-tuning.md) and [Physically realizable LQR
control](../architecture/physically-realizable-lqr-control.md). Use `taoryx model plan` to
inspect the registered campaign and `taoryx model tune <provider> <model>` to
run it; plan/tune does not silently attach it to an externally controlled
session.

## Mass and robustness are declared test seams

Mass robustness is not implied by a gain schedule, an LQI integrator, or a
successful nominal run. Choose and declare one honest disposition for every
controller endpoint:

- **No perturbation seam:** declare robustness `not_applicable` with the
  specific reason (for example, a pseudo-6DOF source replay has no mass or
  persistent-disturbance injection). This is not a pass.
- **Fixed-controller mass screen:** provide actual mass/inertia inputs to the
  plant, independently re-trim every declared mass case, retain the nominal
  controller, run the real allocator/actuator path, and write the typed
  `mass_variation` robustness report with the declared metrics and thresholds.
- **Scheduled controller:** provide each mass/inertia/flight-condition point,
  its source, trim, and local `A/B`; create a declared schedule. Core never
  infers inertia from mass. Verify every named point and schedule transition
  through the same actuator and nonlinear plant path.

Matched constant wrench offsets and wind biases are separate seams with their
own injection location and case records. An offset added after the plant's
load evaluation is not wind; a few mass points are not a payload envelope or
an in-flight mass transition. The endpoint verifier treats an absent artifact
as `not_executed`/blocked, never as a robustness pass. The standard report and
required runtime binding are documented in [Model-to-mission authoring and
automation](model-authoring-automation.md#registered-automatic-tuning).

## Build the provider and its data without aggregate coupling

A family-owned Mission Composition provider must provide the normal discovery
and configuration surface: stable provider metadata, `list_models()`, a typed
configuration schema, output schema, configuration validation, model controls,
and operation matrix. It may use `CatalogMissionCompositionProvider` over its
own packaged catalog fragments or implement the configurable-provider protocol
directly. Either way, it must load only its package data and use stable
non-overlapping identities that the core can merge with other installed
families.

Import the host from
`taoryx.trajectory.catalog_mission_composition`; direct packages should not
use the historical registry-named module or the compatibility aggregate.

Keep these boundaries separate:

| Contract | What the vehicle author supplies |
| --- | --- |
| Family/model metadata | Identity, source/provenance, realization/fidelity, controls, missions, blockers, and claim boundary |
| Configuration and output schemas | What a caller may set before execution and what it may request/read after execution |
| Semantic preflight | Validation and lowering from the portable composition into the exact family runtime request |
| Execution binding | One exact family/mission/fidelity/operation to one factory ID; planned remains explicitly planned |
| Batch factory | A typed `VehicleBatchExecutionRequest` handler that returns the standard vehicle execution artifact |
| Episode factory | A persistent episode with the resolved interface contract and committed-boundary `observe`, `step`, reset, checkpoint, and close behavior |
| Endpoint/witness evidence | A checked-in composition and vertical proof for the advertised runnable slice |
| Revision identity | A package version plus model/provider versions; discovery derives scoped plug-in and metadata fingerprints for client refresh and cache invalidation |

The interface/endpoint catalog joins these records so a developer can inspect
one exact runnable claim rather than reconstructing a vehicle from separate
registries. Keep blocked, batch-native, and no-control cases explicit; absence
of a native factory, adapter, sensor, or physical-effector model is meaningful.

## Verification checklist

Before promoting a new package or control surface, work from narrow to broad:

```bash
# Entry-point metadata, staged registration, ownership, and discovery.
taoryx plugins entry-points --no-builtin --json
taoryx plugins list --no-builtin
taoryx plugins inspect acme.tiltrotor

# The consumer sees a model, schema, exact controls, and truthful blockers.
taoryx model list --provider acme.tiltrotor.mission-composition
taoryx model plan acme.tiltrotor.mission-composition acme_tiltrotor

# Run the family-focused catalog/interface/witness gate once it is registered.
python3 tools/dev.py check-vehicle acme_tiltrotor
```

Add focused tests that prove, as applicable:

- discovery does not construct the plant and a focused catalog does not import
  unrelated vehicle packages;
- package resources resolve from an installed wheel, not checkout paths;
- a package release version and every advertised model version are nonempty,
  while a controlled metadata change updates the derived scoped revision
  fingerprint without relying on unrelated plug-ins;
- configuration, advertised controls, selected action schema, native binding,
  units, bounds, topology, and output metadata agree;
- each new enum, boolean, continuous action, or event has defined validation,
  repeat behavior, lowering, availability, feedback, diagnostics, and
  checkpoint/reset behavior;
- an authority handoff preserves state only where it declares
  `explicit_bumpless`;
- an action affects the declared family runtime or is explicitly reported as
  withheld/unavailable—never silently ignored; and
- every advertised batch/episode pair has a matching exact execution binding
  and parity evidence when parity is claimed.

For package extraction or ownership changes, add an installed-wheel smoke
test analogous to the Hummingbird proof described in
[Vehicle plug-in isolation](../architecture/vehicle-plugin-isolation.md#focused-verification).
Run the repository-wide gates after shared contract changes; the focused gate
is the everyday vehicle-authoring loop, not a substitute for integration or
release validation.

## Definition of done for a new control

A new control is ready for a front end, Python caller, or AI consumer only
when the package can answer all of these questions from its public contract:

1. What does the control mean, and is it configuration, action, event,
   effector, provider-internal behavior, or an authority profile?
2. Where is it valid—family, realization, fidelity, operation, phase, and
   resource state—and what are its type, units/choices, bounds, frame, and
   temporal semantics?
3. Which exact component lowers it, and what plant/controller/allocator state
   does it affect?
4. What happens if it is repeated, limited, unavailable, depleted, terminal,
   or rejected?
5. What committed status proves its actual state or achieved effect, and which
   value intentionally remains unobserved?
6. Can a session reset/checkpoint reproduce it, and can batch/episode parity
   be demonstrated if both operations are advertised?

If those answers are not yet implemented, retain an explicit `planned`,
`blocked`, internally generated, or unavailable status. That gives future
vehicle packages room to grow without advertising a control that a caller
cannot use truthfully.
