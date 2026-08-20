# Model-to-mission authoring and automation

Taoryx has one common path from an installed model advertisement to an
editable mission, a provider-validated configuration, and—when the model has
the required control data—a reproducible controller-design campaign.

This layer is deliberately split between the host and model plug-ins. The host
owns generic orchestration. A plug-in owns every fact that depends on its
physics, source data, route semantics, or evidence. Supplying a data file is
therefore enough to begin discovery, but not enough to claim navigation or an
automatically tuned controller.

## The boundary

| Shared `taoryx` host | Model or mission plug-in |
| --- | --- |
| Plug-in discovery and typed registries | Model identity, version, family, provenance, and source references |
| Portable configuration and output schemas | Parameters, units, frames, value spaces, and declared defaults |
| Plain Python/YAML scaffold and exact-schema compiler | Initialization modes, segment variants, mission templates, and route geometry |
| Stable segment occurrence IDs and template ordering | Capability compiler and family-specific segment semantics |
| Common trim → derivative → authority → candidate sequence | Nonlinear plant, trim definition, state derivatives, effectors, and allocation |
| Normalized controller candidate search and report shape | Operating points, state/control scales, authority requirements, and mission gates |
| Honest readiness and blocker reporting | Qualification evidence and claim boundaries |

The host never invents a waypoint, envelope limit, trim target, control axis,
or actuator. Those omissions are integration blockers, not gain-tuning
problems.

### Native batch plug-in handoff

New batch executors should use the typed request contract rather than the
legacy positional `(composition, output_dir, max_steps)` signature:

```python
from taoryx.vehicle_batch_execution import (
    VehicleBatchExecutionRequest,
    batch_factory_request_v1,
)


@batch_factory_request_v1
def execute_my_vehicle(request: VehicleBatchExecutionRequest) -> MyExecution:
    composition = request.composition
    output_dir = request.output_dir
    # request.binding is the exact runnable factory declaration.
    return run_source_owned_model(composition, output_dir, max_steps=request.max_steps)
```

The provider returns its own result and sidecars as usual. The host then
validates that result against the requested immutable composition and replaces
`execution.json` with a canonical
`taoryx.vehicle-execution-packet/v1alpha1`. That packet preserves the full
provider payload under `provider_execution`, while its common fields make
cross-family diagnosis direct:

- `host_execution.request` identifies the exact composition, factory, output
  directory, and optional step cap;
- `host_execution` records the advertised interface ID/fingerprint plus hashes
  of the provider result and preflight;
- `outcome.scope` is `mission` or `local_screen`, and
  `outcome.disposition` is `passed` or `failed`; do not infer a common outcome
  from a provider-specific status string;
- the CLI run manifest inventories the canonical file and repeats its packet
  identity and file digest. Result catalogs validate this chain when present.

This means a plug-in can keep high-value family diagnostics such as allocation
residuals, source-load fields, or controller traces without making every agent
learn an incompatible top-level result shape.

The request already binds the composition ID/fingerprint, family, mission,
fidelity, exact batch factory ID, output directory, and optional step limit.
The host rejects a returned artifact whose composition identity or output
directory disagrees with this request. The returned `as_dict()` payload must
also carry a schema, status, nonempty claim boundary, and one unambiguous
mission or screen pass disposition. Plug-ins may retain arbitrary
family-specific `runtime`, telemetry, envelope, and diagnostic fields as
extensions.

If a runtime declares an LQR or LQI controller, its method, realization,
integral-output coordinates, and optional tuning receipt are validated as a
common `ControllerRuntimeDeclaration` at batch execution. A common campaign
receipt is produced by `context.runtime_binding_after_application(...)` and
serialized with `.as_dict()`. Existing three-argument factories are supported
while they migrate, but new families should use the request contract from the
start.

### Release evidence and run metadata

A release-sidecar JSON file is evidence, not an opaque status note. Keep its
family-owned schema and metrics, then bind it before writing it:

```python
from taoryx.claim_bound_evidence import bind_release_evidence

payload = bind_release_evidence(
    robustness_report,
    kind="robustness",
    composition=composition,
)
write_json(output_dir / "robustness_report.json", payload)
```

The helper adds `release_evidence_schema`, `release_evidence_kind`, a full
compiled-composition subject (including its SHA-256 identity), and a typed
outcome that must agree with the report's top-level status. The release catalog
checks all of them. Older `status` + `claim` sidecars remain readable, but are
reported as `legacy_unbound` rather than equally strong release evidence.

For runtime artifacts, construct `RunCommandRecord`, `RunLifecycleEvent`,
`RunTermination`, `RunSensorExecution`, and `RunVisualizationMetadata` rather
than adding positional metadata maps. These records retain mapping-style reads
for compatible consumers, while validating the fields integration code needs:
finite command/event timing, named actions, nonblank channel/source labels,
failure reasons, sensor counters, and output-sampling policy. Provider-native
details remain additive extensions of the typed record.

## First use

Install the model profile, or an owning family package when working only on
that family, and verify real entry points:

```bash
python -m pip install taoryx-f16
taoryx plugins list --no-builtin
taoryx model list --provider taoryx.f16.mission-composition

python -m pip install taoryx-hummingbird
taoryx plugins list --no-builtin
taoryx model list --provider taoryx.hummingbird.mission-composition

python -m pip install taoryx-parametric-interceptors
taoryx-interceptor init my-sam.yaml --interceptor-id my-sam
taoryx-interceptor inspect my-sam.yaml --output my-sam-report.json
taoryx-interceptor run my-sam.yaml --set runtime.duration_s=2
taoryx model list --provider taoryx.parametric-interceptors.mission-composition

# Shared source-table fixed wing has one focused provider per family:
python -m pip install taoryx-source-table-fixed-wing
taoryx plugins list --no-builtin
taoryx model list --provider taoryx.x8.mission-composition
taoryx model list --provider taoryx.b747.mission-composition

# Compatibility aggregate and its dependency closure:
python -m pip install taoryx-reference-models
taoryx plugins check --profile compatibility
```

From a source checkout, contributors and agents should use:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
python -m tools.dev install-check
```

Then inventory the exact installed catalog:

```bash
taoryx model list
taoryx model list --provider taoryx.hummingbird.mission-composition
taoryx model assess --output build/model-assessment.json
```

The direct developer profile intentionally does not install the historic
aggregate provider. Any later example that names
`taoryx.registry.mission-composition` is a compatibility-consumer example and
requires the `compatibility` or `full` profile. For new work, substitute the
focused provider owned by the model family, such as
`taoryx.a320.mission-composition`, `taoryx.f16.mission-composition`,
`taoryx.hummingbird.mission-composition`, `taoryx.x15.mission-composition`,
`taoryx.x8.mission-composition`, or `taoryx.b747.mission-composition`.

The inventory includes advertised fidelities, realizations, input/control
status, mission templates, and registered tuning campaign IDs. It does not
promote blocked realizations merely because their schemas are discoverable.
`taoryx model plan` keeps an explicitly requested fidelity. If no mission is
specified and the presentation-default mission is incompatible with that
tier, it selects the first compatible mission with a runnable batch or episode
endpoint. An explicit incompatible mission/fidelity pair still fails closed.

Parametric interceptor intake has one extra pre-registration step because the
model identity itself comes from a user evidence file. `taoryx-interceptor`
owns that file-specific validation and report. Its `run` command still uses the
standard Composition provider and runner. The common `taoryx model` workflow
then applies unchanged to the fixed examples advertised by the installed entry
point; no host special case or ambient profile directory is involved.

The interceptor authoring schema is also the machine-readable option catalogue
for profile-level choices. For mixed aerodynamic/TVC studies it advertises
`control_allocation_policy` as exactly `aerodynamic_first`,
`thrust_vector_first`, or `proportional`; the resolved Composition model repeats
that domain in `control_allocation_policy.supported` and reports the active
policy plus achieved component shares at runtime. This keeps editors, agents,
and the model runner on one enum instead of relying on comments or silent
fallbacks.

For pseudo-6DOF response studies, the response-analysis request similarly owns
an explicit `command_support_fraction` from zero through one. The analysis and
bounded step witness use that same frozen multiplier, while Composition mission
telemetry reports the changing runtime fraction. This lets a developer compare
response tuning at a declared authority condition without pretending that a
local pole calculation qualifies an autopilot or a complete flight envelope.

### Common capability advertisement

Every `taoryx vehicle preflight` result with a family-owned capability adapter
now carries `capability_estimate.capability_advertisement`. Its standard
envelope contains the exact selection and full resolved interface contract
(parameters, controls, status, resources, diagnostics, authority and
observation profiles), plus the plug-in's `family_owned` payload. A plug-in
uses that payload to advertise facts meaningful for its fidelity: for example,
data and operating-point identity, authority bounds, or source-runtime
admission. The host preserves these facts without requiring a fictional common
physical schema across aircraft, rotors, rockets, or passive bodies.

The advertisement has its own `fingerprint_sha256` and retains the resolved
interface fingerprint; witnesses and new retained result packets verify both.
It remains planning evidence: it does not convert declared controls into an
executable controller, extend a source model beyond its validity envelope, or
qualify a trajectory. For example, the HL-20 high-altitude glide intent
advertises its Mach/altitude source-domain mismatch and missing runtime gates
rather than falling back to its separate low-altitude source replay.

The common selection, interface identity, capability fingerprint, feasibility,
and derived-mission fingerprint are validated contracts. `family_owned` is the
intentional extension point: include physical data, authority limits, control
metadata, resource information, and runtime-admission details there rather
than creating a second top-level capability schema.

### Interface binding declarations

Every advertised interface channel now validates its binding rather than
passing an unchecked dictionary. The common source variants are native action,
native effector, batch telemetry/report, episode value, runtime state, derived
value, constant, and composition parameter. A channel may combine sources
when a batch and interactive path expose the same semantic quantity.

Bindings remain serialized as the established flat metadata for existing
tools, but validation now rejects empty source names, transforms with no
`derived_from`, ambiguous initialization/segment parameter context, non-finite
or zero scales, and source units without an explicit scale. Family-owned
binding metadata remains additive. This makes interface construction the place
to catch a telemetry name, transform, frame, or unit seam before a vehicle run.

### Focused vertical endpoint contracts

Use a typed endpoint contract when a model has reached the point where a
developer needs a single trustworthy answer to “what can I actually run and
tune?” The catalog at `verification/vehicle_endpoint_specs.yaml` is the join
point between the existing composition, execution-binding, witness, generic
capability-advertisement, normalized-output, controller-campaign, and maturity
registries. Package-owned fragments travel with the vehicle package that owns
each family: F-16 rows and witnesses live in `taoryx-f16`, Hummingbird rows and
witnesses live in `taoryx-hummingbird`, while the reference-model package
carries the complementary compatibility fragment.
The core host merges non-overlapping fragments by stable identity at discovery
time; a family package never needs to import the aggregate merely to advertise
or run its own endpoint.

```bash
taoryx vehicle endpoint-specs
taoryx vehicle verify x15-source-surface-attitude-rate-lqi
taoryx vehicle verify x15-source-surface-attitude-rate-lqi --execute
taoryx vehicle verify x15-source-surface-attitude-rate-lqi \
  --execute --tune --cache-dir build/controller-tuning-cache \
  --results-dir build/vehicle-endpoints/x15-lqi
```

The default verification is a focused, non-executing cross-contract check. It
compiles the one checked-in witness, exercises and fingerprints the generic
advertisement, validates the resolved interface's data/control/resource
metadata, checks the exact factory and normalized core/telemetry channels, and
requires every endpoint-declared tuning operation to be usable by the
campaign-owned adapter. `--execute` runs only that endpoint's batch factory
(or initializes its exact episode); `--tune` runs only its named campaign and
uses a content-addressed cache by default. With `--execute --results-dir`, the
retained packet is also checked for finite samples of every required advertised
output and receives `controller_tuning_provenance.json`. That sidecar records
the campaign ID, cache key, selected candidate fingerprints, and the controller
method/configuration actually reported by the batch packet.

The report makes the acceptance sequence explicit:
`contract_valid → batch_executed → tuner_bound → tracking_passed →
disturbance_or_mass_screened`. A method match alone is deliberately not a
`tuner_bound` pass. A runtime must declare the exact campaign node, candidate
profile, and configuration fingerprint it applied. Until a model does that,
the report says `candidate_ready_not_runtime_bound`; this is an actionable
integration gap, not an implied tuned-controller claim. Missing disturbance or
mass evidence is likewise a declared `not_executed` (or source-model
`blocked`) screen, never a robustness pass. Neither option turns a local
screen into envelope, robustness, navigation, or qualification evidence.

Each `--tune` report now includes a `tuning_application_contexts` collection
when the selected candidates expose resolved gains. A context carries the
exact campaign/cache/node/profile fingerprints plus ordered state/control/
integral-output names, scales, weights, and physical-coordinate gain matrices.
It is the common handoff from the host tuner to a plug-in runtime; it is not a
claim that a batch applied those gains. A source-owning factory must compare
its controller coordinates exactly, instantiate from `resolved_gains`, and
only then emit the receipt returned by
`context.runtime_binding_after_application(...)`. This deliberately rejects a
projected, reordered, scheduled, or hand-designed controller rather than
allowing an LQR/LQI method label to imply common-tuner application.

Endpoint verification also reports `records.performance`: focused wall-clock
phase timings (compile, preflight, campaign, batch, output validation, result
indexing, and provenance) plus the cache disposition (`hit`, `miss`, or
`not_persisted`). The numbers are local iteration diagnostics, not a real-time
or controller-performance claim. Reuse a stable `--cache-dir` while iterating;
the report makes a cache miss visible instead of leaving a slow trim/tune run
mysterious.

Every endpoint now declares at least one robustness-screen contract. The
contract records its intended mass, matched constant-offset, or wind-bias
cases, required numeric metrics and thresholds, artifact filename, execution
readiness, and claim boundary. Until a source-owning factory writes the
standard artifact, the endpoint stays incomplete but tells the developer
exactly what is missing. The emitted artifact format is:

```json
{
  "schema": "taoryx.endpoint-robustness-screen/v1alpha1",
  "id": "the-endpoint-declared-screen-id",
  "kind": "mass_variation",
  "pass": true,
  "cases": [
    {
      "id": "mass-0.85x",
      "parameters": {"mass_factor": 0.85},
      "status": "pass",
      "metrics": {
        "final_attitude_rate_error_fraction": 0.42,
        "saturation_fraction": 0.0
      }
    }
  ]
}
```

The verifier compares the case IDs and parameters, each case disposition, and
every declared metric threshold. It does not infer wind physics from a matched
wrench offset or extrapolate a handful of mass points into gain-schedule or
qualification evidence.

When a plug-in genuinely instantiates a common campaign candidate, put this
claim in the batch `execution.json` runtime record (only after the candidate's
gains/configuration are actually used):

```json
{
  "tuning_binding": {
    "campaign_id": "provider-campaign-id",
    "node_id": "declared-operating-point",
    "candidate_profile_id": "selected-profile",
    "candidate_configuration_fingerprint_sha256": "<64 lowercase hex characters>",
    "applied_gain_fingerprint_sha256": "<64 lowercase hex characters>",
    "controller_method": "lqi",
    "cache_key": "<campaign-cache-key>"
  }
}
```

The host validates this shape in the result catalog and compares both the
candidate configuration and the applied ordered-gain fingerprint to the
specific cached campaign artifact during endpoint verification. Do not add it
for a separately hand-designed controller merely because both use LQR or LQI.
The current physical endpoint batches still use their source-owned controller
builders, so they correctly report `candidate_ready_not_runtime_bound` until a
compatible context is actually applied.

For a new physical control endpoint, add one record with its stable endpoint
ID, family/model/mission/fidelity selection, exact operation/factory/witness
pairs, required public outputs, campaign and required tuning operations, a
maturity-record reference, and an explicit claim boundary. Add the same data
to the plug-in package resource, then start with:

```bash
python -m pytest tests/unit/test_vehicle_endpoint_spec.py -m slow
python -m pytest tests/unit/test_tuning_application.py tests/unit/test_vehicle_endpoint_spec.py -m 'not slow'
taoryx vehicle verify <endpoint-id> --execute --tune \
  --results-dir build/vehicle-endpoints/<endpoint-id>
```

This is intentionally a vertical migration seam, not a replacement for the
source-owning registries. It makes cross-registry drift visible one vehicle at
a time while the wider catalog is progressively promoted.

### A runnable physical-control screen

The F-16's direct-wrench and surface-allocated tiers also publish a deliberately
small, source-trim control screen. It is the right fast path for inspecting
the physical controller, allocated effectors, source-envelope checks, and
portable control/status traces before attempting a full scheduled mission:

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  f16_s119 \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission f16_local_physical_control_screen_v1
taoryx model tune \
  taoryx.registry.mission-composition \
  f16_s119 \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission f16_local_physical_control_screen_v1 \
  --campaign f16-source-surface-local-lqr-v1

# Select the dedicated offset-free body-velocity recovery screen when that
# local use case applies. It is a separate exact endpoint from the one-second
# route-entry LQR screen.
taoryx model tune \
  taoryx.registry.mission-composition \
  f16_s119 \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission f16_local_physical_surface_lqi_screen_v1 \
  --campaign f16-source-surface-local-lqi-v1

# Inspect the explicit four-node LQR schedule-interior campaign. Its plan
# reports held-node controller selection; it does not claim a continuous
# gain scheduler.
taoryx model plan \
  taoryx.registry.mission-composition \
  f16_s119 \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission f16_local_physical_surface_lqr_schedule_interior_screen_v1
```

```bash
taoryx vehicle compose \
  examples/vehicle_composition/f16_local_physical_surface_screen_compose.yaml \
  --output build/f16-surface-screen.composition.json
taoryx vehicle preflight build/f16-surface-screen.composition.json
taoryx vehicle run build/f16-surface-screen.composition.json \
  --output-dir build/f16-surface-screen
taoryx vehicle result build/f16-surface-screen \
  --composition build/f16-surface-screen.composition.json
```

Use `f16_local_physical_surface_lqi_screen_compose.yaml` for the five-second
fixed-altitude source-trim velocity LQI recovery. Its reports retain the
output-integrator state, requested-versus-achieved wrench, actual engineering-
overlay effectors, and the fixed altitude/trim-attitude context. The reported
local origin and attitude are derivative conditions, not integrated navigation
or a route claim.

Use `f16_local_physical_surface_lqr_schedule_interior_screen_compose.yaml` for
the four-node physical LQR schedule-interior campaign. It reruns the two
retained body-``w`` perturbations at each independently source-retrimmed
altitude and records the selected node in the public status trace. The
controller is deliberately held at each node: this is evidence for a discrete
local schedule interior, not continuous gain interpolation, node-transition
replay, wind or mass rejection, or a flight envelope.

Use `f16_local_physical_surface_lqr_schedule_transition_screen_compose.yaml`
when the four retained time-marching schedule transitions are the appropriate
evidence. It uses the same registered LQR tuner, interpolates source-node
controller demand by the declared altitude coordinate, linearly blends only
the validated endpoint derivatives and effectiveness, and allocates every
demand through bounded elevator, aileron, rudder, and throttle. Its report
contains the blend policy, per-case allocation disposition, actuator history,
and common status/resource/control traces. It also emits a typed,
composition-bound `robustness_report.json`: nominal plus constant external
pitch moments of ±5% of the shared declared pitch-wrench scale. The offset is
converted into angular acceleration only after source derivative evaluation,
using the blended full inertia; it is never added to controller demand or the
allocator result. This is bounded local transition evidence—not an integrated
navigation path, wind or mass robustness, a full envelope, or flight
qualification.

Use `f16_local_physical_direct_wrench_screen_compose.yaml` to exercise the
same source-trim controller through the direct-wrench comparison path. Both
screens are valid development endpoints; neither claims a completed racetrack,
gain schedule, envelope sweep, or flight qualification. `taoryx vehicle authoring
f16_s119` exposes this distinction as `operational_maturity` instead of hiding a
runnable screen behind the fidelity-promotion blockers.

Hummingbird provides the corresponding
`hummingbird_local_individual_rotor_lqi_screen_v1` endpoint. Its source-hover
plant linearizes the three requested body moments, synthesizes an LQI
attitude/rate controller, and passes every command through the declared
four-rotor allocation, bounds, and motor lag before integrating the source
state. It uses the common physical-LQI validator, so the standard result also
retains `nonlinear_validation.json` with the exact output-integrator state,
requested/achieved/residual wrench, actual rotor allocation, and declared
derivative environment. The concise result reports actual rotor positions,
requested/achieved/residual moment, allocation saturation, and LQI integral
error:

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  hummingbird \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission hummingbird_local_individual_rotor_lqi_screen_v1
taoryx model tune \
  taoryx.registry.mission-composition \
  hummingbird \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission hummingbird_local_individual_rotor_lqi_screen_v1 \
  --campaign hummingbird-source-rotor-local-lqi-v1
```

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_individual_rotor_lqi_screen_compose.yaml \
  --output build/hummingbird-lqi-screen.composition.json
taoryx vehicle preflight build/hummingbird-lqi-screen.composition.json
taoryx vehicle run build/hummingbird-lqi-screen.composition.json \
  --output-dir build/hummingbird-lqi-screen
taoryx vehicle result build/hummingbird-lqi-screen \
  --composition build/hummingbird-lqi-screen.composition.json
```

That source-hover result also emits `robustness_report.json` in the generic
endpoint format. It records independently re-trimmed 85%, 100%, and 115%
source-mass cases under the one fixed nominal LQI design, including final
attitude/rate error fraction and allocation/actuator saturation fraction for
each case. This is discrete local mass-mismatch evidence only; it is neither a
gain schedule nor an in-flight mass-transition or payload-envelope claim.

It proves a local source-backed individual-rotor control realization, not a
wind-disturbance campaign, gain schedule, mission trajectory, envelope sweep,
or flight qualification. The generic Hummingbird tuning campaign remains a
separate `candidate_ready` design artifact; the physical screen is the
allocation-and-actuator evidence that its candidate does not provide.

`hummingbird_local_vertical_translation_lqi_screen_v1` is the matching
source-local collective path. It adds body-`z` force to the requested
roll/pitch/yaw moment coordinates and synthesizes an LQI design that tracks
vertical speed as well as attitude error. A bounded outer down-position layer
only selects the vertical-speed reference; it does not inject a force. The
common screen routes each request through six-axis source effectiveness,
bounded four-rotor allocation, motor lag, and the nonlinear source plant.
Its fixed 16-second capture sequence is climb, hover, descent, and return
hover. The result advertises requested, achieved, and residual body force and
moment, every actual rotor speed, and the fourth vertical-speed integrator.

```bash
taoryx model plan \
  taoryx.registry.mission-composition hummingbird \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission hummingbird_local_vertical_translation_lqi_screen_v1
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_vertical_translation_lqi_screen_compose.yaml \
  --output build/hummingbird-vertical-lqi-screen.composition.json
taoryx vehicle run build/hummingbird-vertical-lqi-screen.composition.json \
  --output-dir build/hummingbird-vertical-lqi-screen
```

The same endpoint also retains a discrete 85/100/115% source-mass screen.
Each case independently re-trims the source plant, then runs the same fixed
nominal-mass LQI design through collective force/moment allocation and the
motor lag; `mass_variation_report.json` keeps every trim and phase capture.
This is local mass-mismatch evidence, not a gain schedule, an in-flight mass
transition, or payload-envelope coverage.

Its focused endpoint is `hummingbird-source-vertical-translation-rotor-lqi`:

```bash
taoryx vehicle verify hummingbird-source-vertical-translation-rotor-lqi --execute
```

It still does not establish wind rejection, battery/SOC behavior, contact,
landing, gain scheduling, or flight qualification. Those are the Hummingbird
family’s remaining next gates.

`hummingbird_local_direct_wrench_screen_v1` is the companion local bridge
screen. It uses the same pinned source-hover state and the shared automatic LQR
recovery runner with explicit six-axis bounds. Its batch trace advertises the
controller-generated total force/moment requests, velocity/rate feedback,
requested-versus-achieved wrench diagnostics, and fixed source-hover mass. It
does not expose an external episode-control path or claim rotor allocation,
motor lag, translation, wind, gain scheduling, or flight qualification:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_direct_wrench_screen_compose.yaml \
  --output build/hummingbird-direct-wrench-screen.composition.json
taoryx vehicle preflight build/hummingbird-direct-wrench-screen.composition.json
taoryx vehicle run build/hummingbird-direct-wrench-screen.composition.json \
  --output-dir build/hummingbird-direct-wrench-screen
```

`model assess` is the all-model readiness matrix. Use `model assess --summary`
for a concise author-facing inventory of each realization's controls, campaign
IDs, and campaign-owned adapter IDs; omit `--summary` for the full matrix. The
full matrix exercises every installed advertisement and reports, per
realization:

- configuration/output advertisement completeness, properties, frames, and
  output groups;
- generic plan/scaffold/compile availability and advertised segment vocabulary;
- dynamics fidelity, input realization, actuator type, and actuation class;
- semantic control channel count and IDs, authority IDs, intent IDs, and the
  default authority;
- applicable family adapter, campaign-owned tuning-adapter state/control and
  operation metadata, common controller-automation disposition, campaign IDs,
  and exact declared blockers; and
- fidelity-lowering and RSLQR status.

Its `controller_readiness_summary` gives the corresponding compact integration
gate: every available realization with externally supplied controls must have
a matching common tuning campaign, while provider-managed, uncontrolled, and
explicitly blocked tiers remain separately counted. A `complete` summary is
advertisement coverage only, not controller qualification.

The assessment deliberately reports automatic lowering as `not_advertised`
unless a separate executable lowering contract supplies evidence. It similarly
reports RSLQR as `deferred`: the generic LQR/LQI hooks do not make robust or
adaptive synthesis available by implication. Use `model plan` when the full
channel schema, units, bounds, provider bindings, and controller pipeline are
needed for one selected realization.

Each full-assessment `advertisement` record now includes a `plan_exercise`.
This runs the shared plan/scaffold join for that exact provider/model and
checks that data, controls, navigation, modes, and segment automation are all
present. A failed join reports the model as `incomplete`; it never defaults to
an optimistic `complete` label. The exercise is metadata-only and does not run
the vehicle, tune a controller, or change the model's qualification status.
`model assess --summary` carries the corresponding
`advertisement_readiness_summary`, so an agent can identify incomplete
provider/model pairs without parsing the full matrix.

A planned direct-wrench action schema is a design interface, not a runnable
mission endpoint. A batch-only local screen can instead advertise its own
internally generated action trace—with no caller override—once it has a
checked-in composition witness, a runnable source-owned binding, and a
completed source run with accepted telemetry and truth evidence. If a source
table reaches an envelope boundary before producing an accepted trajectory, the
result remains explicitly blocked;
the public result records that incomplete runtime evidence without inventing
control, status, or resource traces.

A focused source-table smoke may deliberately pass `--max-steps` to check only
the first committed intervals of a long route. When that explicit cap is what
stops the run, its evaluation is `valid` with outcome `time_limited`, and its
run manifest records termination reason `max_steps`. It is still unqualified
and does not pass mission objectives. This is distinct from
`numerical_failure`, which must remain reserved for an actual runtime or
numerical problem rather than a requested bounded prefix.

## Plan before authoring

Ask the host to join all of the portable metadata for one selection:

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  hummingbird \
  --fidelity pseudo_6dof \
  --output build/hummingbird-plan.json
```

The plan reports:

- exact provider, model version, fidelity, realization, and mission selection;
- exact selected mission operation records, runnable/blocked batch and step
  endpoints, and the bounded `endpoint_maturity` derived from those records;
- data provenance, properties, frames, configuration/output fingerprints, and
  telemetry groups;
- controls, authorities, control intents, and family-adapter status;
- waypoint, target, gate, heading, altitude, and other navigation parameters;
- initialization modes, segment types, termination modes, and ordered segment
  instances;
- generated-draft gaps and the next commands; and
- whether tuning is registered, provider-managed, not applicable, or awaiting
  a plug-in campaign.

Only caller selections and advertised defaults are used. If an advertised
default mission supports exactly one fidelity, that declared compatibility
may resolve the fidelity. Ambiguous selections remain explicit gaps.

## Minimal automatic-control declaration

A plug-in does not need to hand-build campaign nodes, authority checks,
candidate grids, and LQR/LQI method objects. It may supply one compact core
declaration:

```python
from taoryx.control_automation import ControlAutomationDeclaration

declaration = ControlAutomationDeclaration(
    id="my-model-attitude",
    campaign_id="my-model-attitude-v1",
    family_id="my-model",
    tier="pseudo_6dof",
    strategy_id="fixed-wing.v1",
    node_id="cruise",
    state_scales={"bank_rad": 0.5, "roll_rate_rad_s": 1.0},
    control_scales={"aileron_rad": 0.2},
    offset_free_outputs=("bank_rad",),
)
campaign = declaration.build_campaign()
```

Core selects LQI when `offset_free_outputs` is nonempty and LQR otherwise. It
generates the normalized candidate grid, authority preflight, integral
weights, and standard stop-at-first-blocker sequence. A declaration may also
name `design_state_names` and `design_control_names` when a larger observable
plant contains sidecar response states that are not independent control axes.
Core then requires the selected subsystem to be dynamically closed before it
tunes it. The plug-in still owns the physical coordinate names, engineering
scales, trim hints, authority intent, and outputs whose steady error is
meaningful. A data table without a dynamics/derivative seam or
control-effectiveness mapping remains an explicit integration blocker.

For LQI, `integral_weight_multiplier` declares the base persistent-error
priority and `integral_weight_multipliers` can add a bounded, named lattice of
relative priorities. Every candidate serializes both the actual
`integral_q_diagonal` and its multiplier, so an agent can distinguish a
stronger offset-rejection candidate from a generic Q/R change. The host still
does not silently choose a nonlinear winner: use the endpoint's declared
allocator-backed robustness screen to select and retain a physical profile.

Each `model tune` LQI candidate retains and serializes its output matrix,
state gain, and integral gain under `lqi_controller`; it is therefore usable
by a plug-in-owned nonlinear screen rather than being reduced to a pole-only
report. For a model with declared native controls but no physical allocator,
use `validate_nonlinear_native_coordinate_lqi(...)`. That runner sends only
the named model controls to the adapter derivative and reports native-control
saturation plus integral telemetry. It deliberately does not manufacture
wrench, effector, actuator-lag, or hardware-allocation evidence. Models that
do have an allocator should instead use the physical wrench LQI validator.

When that native validation should become a runnable Composition endpoint, a
plug-in supplies `LocalNativeCoordinateLqiScreenConfig`: its existing plant,
retained campaign candidate, pinned perturbation, named-coordinate bounds,
and an exact committed-status mapper. The host then owns the nonlinear LQI
run, acceptance gates, preflight, result artifacts, and registration shape.
The plug-in registers the executable definition with
`register_local_native_coordinate_lqi_screen_definition(...)` and registers
the identical static UI/agent record with
`register_local_controller_screen_advertisement(...)`. Discovery rejects a
missing, differently owned, endpoint-mismatched, or metadata-mismatched pair;
the runtime and authoring view are therefore two projections of one package
endpoint rather than separate control descriptions.

The A320 `a320_local_native_coordinate_lqi_screen_v1` is the first shipped
example. It is batch-only and intentionally emits no external action trace:
the aileron/elevator/rudder coordinates are internally generated native model
commands, not advertised physical effectors or route-guidance overrides. For
this kind of endpoint, `taoryx model plan ... --mission <screen-id>` publishes
`controller_automation.local_controller_screen`: the exact cadence, campaign
ID, named-coordinate units and bounds, integral outputs, operation scope, and
physical-allocation boundary. This static record is available without running
a tuning campaign or executing the screen.

The focused endpoint contract separately records whether that fidelity owns a
mass, wind, or persistent-offset seam. A physical screen must declare its
screen cases and pass its typed robustness sidecar. A native-coordinate screen
such as A320's may instead declare `robustness_requirement: not_applicable`
with a specific reason, but only when it exposes no such seam. The A320 packet
therefore emits a composition-bound `convergence_report.json` for its nominal
LQI recovery; that evidence is deliberately not mislabeled as robustness.

The same `local_controller_screen` record covers the installed Hummingbird,
X-15, and HL-20 local direct-wrench screens. It identifies all six body-force
and body-moment coordinates with their exact local bounds, controller method,
cadence, campaign (when LQI is selected), and whether the endpoint also has a
caller-driven direct-wrench step bridge. A screen is still not a physical
effector allocator merely because its controls are completely advertised.

A tuning-campaign registration may declare the same static screen record for
a model-owned allocated path. Hummingbird's individual-rotor LQI screen is
the first example: its plan records the four source rotor-speed bounds, the
5 ms motor lag, allocated-LQI realization, and batch-only policy without
constructing a plant, solving trim, or tuning a gain. Plug-ins should attach
this record only when its exact mission/template pairing is already a
registered Composition endpoint.

Some local screens instead retain a source-owned controller profile and are
not eligible for common retuning. In that case a plug-in registers a
`LocalControllerScreenAdvertisement` through
`register_local_controller_screen_advertisement(...)`. The record is selected
only by its exact provider, model, family, fidelity, realization, and mission
template; it cannot leak to a neighboring tier. Its owning provider is
canonical, while explicit compatibility-provider aliases may expose the same
record to an aggregate host; alias-expanded endpoint identities are checked for
collisions. The F-16 direct-wrench local
screen is the shipped example. Its plan advertises the retained LQR profile,
fixed 0.2 s cadence, four native wrench components, and their normalization
scales. It also explicitly says that hard wrench limits and caller override
are not declared—so a host does not confuse an internal comparator with a
physical allocator or a tunable campaign.

`model plan` and `model assess` separately report the general family adapter
and the campaign-owned tuning adapter. This matters for a lower-tier guidance
or response-law campaign: a physical family adapter may correctly omit that
tier while the matching campaign still supplies the exact adapter needed for
its local screen. The latter includes its advertised state/control coordinates
and available operations; it does not execute a trim or synthesize gains until
`model tune` is requested.

Every `model plan` also carries a `maturity_advertisement`. It reports the
family-level planning maturity, evidence strength, current composition status,
and next gate separately from the selected endpoint's operation matrix. Thus a
batch-ready local LQI screen can be discoverable without implying a
vehicle-qualified controller or physical effector allocation.

`focused_endpoint_verification` supplies the executable proof route for the
model: physical families use `taoryx vehicle verify <endpoint-id>`, while
nonphysical provider workflows use `taoryx model verify <endpoint-id>`. The
advertisement distinguishes an exact selected mission/fidelity (and, for
workflows, realization) match from a different published endpoint, so an
agent cannot use a nearby local screen as evidence for another plan.

The shipped A320 pseudo-cruise, F-16 point/pseudo source-trim, Hummingbird
pseudo-hover, and X-15 local direct-wrench campaigns use this declaration
instead of constructing tuner nodes directly.

## Generate and compile a mission draft

Generate ordinary YAML rather than constructing the verbose schema value
classes by hand:

```bash
taoryx model scaffold \
  taoryx.registry.mission-composition \
  tumbling_body \
  --output build/tumbling-body.yaml
```

The generated file is bound to the exact provider, model version, and schema
fingerprint. Required values appear as `<REQUIRED>` and ambiguous choices as
`<SELECT>:...`. Numeric values use the canonical units advertised beside the
corresponding schema parameter; the authoring layer performs no hidden unit
conversion, clipping, or default invention.

After filling the placeholders, compile through the owning provider:

```bash
taoryx model compile \
  build/tumbling-body.yaml \
  --output build/tumbling-body.prepared.json
```

Compilation rejects unresolved placeholders, stale versions or fingerprints,
unknown fields, incompatible choices/templates, invalid values, and any
provider-specific semantic violation. The result is a normal
`PreparedTrajectoryConfiguration` that can enter Mission Composition.

## Run a prepared composition

Providers that publish a common batch runner can execute the prepared JSON
without a provider-specific script. Keep the provider ID on the dispatch
command: prepared configurations intentionally preserve portable model input
and therefore do not embed a provider identity.

```bash
taoryx model compile mission.yaml --output build/mission.prepared.json

taoryx model run \
  taoryx.reference.mission-composition \
  build/mission.prepared.json \
  --request-id example-run \
  --output build/mission.response.json
```

`model run` revalidates the configuration against the installed provider
before building the `MissionCompositionRunRequest`, then writes the normal
discriminated trajectory or failure response. Use `--output-mode all` for all
applicable telemetry, or `--output-mode selected --channel <id>` and
`--telemetry-group <id>` for a bounded selection. `--maximum-objects` and
`--no-spawned-objects` let consumers exercise lineage limits without changing
the configuration.

### Select a streaming control profile

Stateful Vehicle Composition models advertise all authority profiles under
each realization's `controls`, including the default authority, command owner,
selection scope, switching policy, phase applicability, units, bounds, frames,
sampling semantics, and lowering chain. A consumer selects one profile when
opening the common Mission Composition session; it does not merge controls
from several profiles into one action vector.

For a live-switchable multi-profile realization, the common model-authoring
plan's `controller_automation.channels` field is the default authority's
immediately usable channel projection, while `available_channels` is the
complete realization catalog. Legacy locked realizations retain their complete
catalog in `channels` for compatibility. Join `available_channels` to
`authorities[].channel_ids` to inspect any profile. `default_authority_id` and
`channel_projection` make that distinction machine-readable; clients must not
treat `available_channels` as one simultaneous action vector.

The same plan returns `controller_automation.control_scheme_support`, already
filtered to the selected realization and fidelity. Use that collection to
build a mode picker; use the referenced authority and action records to build
the actual control form. The full assessment summary also lists scheme IDs per
realization. This avoids treating a common scheme label as permission to send
the same numeric vector to unrelated families.

```python
from taoryx.trajectory import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
    MissionCompositionSwitchAuthorityRequest,
)

sessions = MissionCompositionSessionManager(provider)
descriptor = sessions.open(
    MissionCompositionOpenSessionRequest(
        session_id="remote-f16",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        authority_profile_id="live_waypoint_guidance",
        command_source_id="remote-waypoint-client",
    )
)

result = sessions.step(
    MissionCompositionSessionStepRequest(
        session_id=descriptor.session_id,
        authority_profile_id="live_waypoint_guidance",
        expected_sequence=0,
        duration_s=0.2,
        action={
            "navigation.waypoint.north.command": 2_000.0,
            "navigation.waypoint.east.command": 500.0,
            "navigation.waypoint.altitude.command": 3_000.0,
            "navigation.waypoint.capture_radius.command": 25.0,
            "navigation.waypoint.speed.command": 250.0,
        },
    )
)

handoff = sessions.switch_authority(
    MissionCompositionSwitchAuthorityRequest(
        session_id=descriptor.session_id,
        authority_profile_id="reduced_pilot_command",
        expected_sequence=result.sequence,
        command_source_id="remote-gamepad",
    )
)
```

The open descriptor and handoff response each return the exact active action
schema. A step returns requested/applied semantic values separately from the
lowered adapter values and lowering evidence. `control_authority` in every
committed observation identifies the active owner and source. Omitting
`authority_profile_id` on open retains the legacy native-action schema.

To make the selection part of a reusable prepared composition, set
`startup_authority_profile_id` before validation:

```python
configured = configuration.model_copy(
    update={"startup_authority_profile_id": "live_waypoint_guidance"}
)
prepared = provider.validate_configuration(configured)
descriptor = sessions.open(
    MissionCompositionOpenSessionRequest(
        session_id="remote-waypoint-composition",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
    )
)
```

An explicit open-session authority may be used instead, but it must agree with
the prepared selection. This prevents a deployment configuration and a remote
client from silently training or operating against different action spaces.

Every `descriptor.authority_profiles` item is normalization-ready even while
inactive. `action_schema` contains units, frames, topology, and bounds;
`agent_action_space` contains channel order, flattening offsets, encodings,
native/agent ranges, periodic wrap periods, discrete choices, and external
statistics requirements. The same authority-specific projection is available
before startup from
`realization.controls.rl_action_space(authority_id=..., operation="step")`.
Use `observation.control_authority.available_action_ids` as the runtime mask,
and use `result.control_feedback` for requested/applied/achieved readback. Do
not treat the presence of a static profile as proof that it is available in
the current phase.

The first provider-neutral streaming acceptance ladder is deliberately small:

| Model | Default profile | Selectable live profiles | Boundary |
| --- | --- | --- | --- |
| `reference_ballistic_3dof` | `open_loop_coast` | none | Zero-action analytical propagation; no controller is invented |
| `reference_constant_velocity_waypoint_3dof` | `configured_waypoint_guidance` | `kinematic_velocity_command`, `live_waypoint_guidance` | Shared analytical transition with explicit speed/angle or waypoint units |
| `contract_probe_vehicle` | `debug_guidance_control` | `debug_discrete_control`, `debug_event_control` | Non-physical API stress surface; tuning is not applicable |
| `simple_aero` | `generated_mission_commands` | `direct_throttle_command` | Existing point-mass kernel; bank is non-steering schedule telemetry |
| `hummingbird` (`pseudo_6dof`) | `body_motion_response` | `velocity_yaw_command`, `live_waypoint_guidance` | Aggregate thrust-vector plant with battery-aware readback; no individual-rotor claim |

These fixtures test open, observe, step, authority handoff, typed schemas,
lowering evidence, and checkpoint restoration before the same pattern is
promoted into source-owned vehicle families. A zero-action provider/open-loop profile is a
first-class profile, not a missing schema: it accepts `{}` and reports its
owner while rejecting invented caller coordinates.

Only profiles declaring `explicit_bumpless` can use the live handoff. The
current reduced A320/F-16 adapters preserve plant state and clear old held
references during transfer. CADAC source-managed sessions instead advertise an
empty `source_program_control` action schema with `provider_managed` switching;
their retained source controller cannot be replaced through this API.

For Hummingbird, set `startup_authority_profile_id="velocity_yaw_command"`
for north/east/positive-up velocity plus yaw control, or
`"live_waypoint_guidance"` for in-stream waypoint retargeting. Inspect each
inactive profile's `action_schema` before opening to obtain exact multirotor
units, bounds, topology, and normalization; do not reuse the fixed-wing
waypoint speed envelope. The session's `control_feedback` and
`lowering_evidence` report achieved state, held values, waypoint progress,
aggregate-thrust limiting, and battery availability.

### Focused workflow endpoints

`Simple Aero`, `dual_launch_glider`, and the debug-model fixtures are
registered model workflows, but they are not physical entries in the Vehicle
Composition registry. Their checked-in endpoint catalog keeps that boundary
visible while still proving the useful batch vertical path: authored draft →
installed-provider validation → advertised common batch registration →
normalized result surface. Each record is contributed by its owning plug-in; a
selected host sees only those fragments, and an installed wheel resolves the
corresponding package data. Simple Aero additionally exposes the persistent
session described above. The debug plug-in owns ballistic, waypoint, and
contract-probe witnesses; the waypoint and probe retain their selectable
session controls and feedback, while the endpoint witnesses remain batch
proofs. None of these records silently broadens a workflow into a physical
vehicle, controller, robustness, or qualification claim.

```bash
taoryx model endpoint-specs
taoryx model verify simple-aero-fixed-ld-batch
taoryx model verify simple-aero-fixed-ld-batch --execute
taoryx model verify dual-launch-attached-booster-batch --execute
taoryx model verify reference-ballistic-3dof-batch --execute
taoryx model verify reference-waypoint-3dof-batch --execute
taoryx model verify debug-contract-probe-batch --execute
```

The non-executing check catches stale model/schema identity, unfilled values,
missing output advertisements, a missing executor registration, and a changed
batch-only boundary. `--execute` additionally requires all declared core and
telemetry channels, plus any endpoint-declared lifecycle event. It does not
turn either witness into a vehicle, controller, robustness, or qualification
claim; their explicit `not_applicable` robustness dispositions explain why.

The shipped consumer fixtures are deliberately small and deterministic:
`reference_ballistic_3dof`,
`reference_constant_velocity_waypoint_3dof`, and the non-physical
`contract_probe_vehicle` under
`taoryx.debug.mission-composition-contract-probe`. The probe is useful for
testing complete metadata, typed telemetry, failures, and multi-object
lineage; it remains a development fixture rather than a physics claim.

The two analytical fixtures advertise their exact repeatable native sequence
and operation matrix. Their plans select
`reference_ballistic_3dof_repeatable_sequence_v1` or
`reference_constant_velocity_waypoint_3dof_repeatable_sequence_v1`, expose
`validate`, common-runner `batch`, and the registered persistent `step` route.
Both modes call the same analytical transition helpers, so stepping does not
introduce a second dynamics implementation. This makes their runnable contract
visible to agents without misrepresenting either fixture as a physical vehicle
or a control/qualification result.

In Python, use `run_prepared_mission_composition(providers, provider_id,
prepared, output=...)` for the same revalidate-and-dispatch boundary.

### Plain Python authoring

The same path is available programmatically:

```python
from taoryx.model_authoring import (
    author_configuration,
    select_variant,
    sequence_template,
)

prepared = author_configuration(
    provider,
    configuration_id="programmatic-tumbling-body",
    model_id="tumbling_body",
    fidelity="point_mass_3dof",
    realization_id="point_mass_3dof",
    mission_template_id="tumbling_body_release_damping_impact_v1",
    values={
        "initialization": select_variant(
            "atmospheric_release",
            altitude_m=1000.0,
            speed_m_s=120.0,
            body_rates_rad_s=[0.1, 0.2, 0.3],
            area_policy="orientation_averaged_projected_area",
        ),
        "segments": sequence_template(
            "tumbling_body_release_damping_impact_v1",
            {"duration_s": 10.0},
            {"impact_plane_altitude_m": 0.0},
        ),
    },
)
```

For a templated sequence, the compiler supplies the advertised segment types
and deterministic occurrence identities such as `01-passive_coast`. Repeated
segment kinds therefore remain distinct without making users hand-code the
full choice-value graph.

Providers that advertise an open sequence can use explicit, named
occurrences. This is the concise path for programmatic waypoint courses:

```python
from taoryx.model_authoring import custom_sequence, segment_occurrence

segments = custom_sequence(
    segment_occurrence(
        "waypoint_leg",
        instance_id="north-leg",
        duration_s=30.0,
        waypoint_north_m=1000.0,
        waypoint_east_m=0.0,
        waypoint_altitude_m=1000.0,
    ),
    segment_occurrence(
        "waypoint_leg",
        instance_id="east-leg",
        duration_s=30.0,
        waypoint_north_m=1000.0,
        waypoint_east_m=1000.0,
        waypoint_altitude_m=1000.0,
    ),
)
```

The helper does not assume what a waypoint means. Segment names, parameter
names, units, frames, and bounds still come from the selected model schema and
are checked by its provider.

## Registered automatic tuning

The numerical runner remains in core, while a model plug-in registers the
family adapter and immutable campaign inputs:

```python
registrar.register_controller_tuning_campaign(MY_CAMPAIGN_REGISTRATION)
```

`ControllerTuningCampaignRegistration` binds the campaign to an exact
provider/model/family/fidelity plus compatible realizations and missions. The
catalog validates those identities against the model advertisement before a
model command runs.

Execute the only registered campaign for a model, or name it explicitly when
several exist:

```bash
taoryx model tune \
  taoryx.registry.mission-composition \
  a320_openap_3dof \
  --output build/a320-pseudo-tuning.json

taoryx model tune \
  taoryx.registry.mission-composition \
  hummingbird \
  --campaign hummingbird-pseudo-hover-attitude-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  f16_s119 \
  --fidelity pseudo_6dof \
  --campaign f16-pseudo-source-trim-attitude-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  x15 \
  --campaign x15-source-release-direct-wrench-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  x15 \
  --campaign x15-source-release-direct-wrench-lqi-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  hl20_mod_k \
  --campaign hl20-source-subsonic-direct-wrench-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  hl20_mod_k \
  --campaign hl20-source-subsonic-direct-wrench-lqi-v1

# X8's source-table surface campaign is deliberately local. It may tune even
# while its end-to-end surface racetrack realization remains blocked.
taoryx model tune \
  taoryx.registry.mission-composition \
  skywalker_x8 \
  --campaign x8-source-surface-local-lqi-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  b747 \
  --campaign b747-source-surface-local-lqi-v1

# Lower tiers expose exact kinematic guidance, not source-surface tuning.
taoryx model tune \
  taoryx.registry.mission-composition \
  skywalker_x8 \
  --fidelity point_mass_3dof \
  --realization point_mass_3dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --campaign x8-language-backed-guidance-local-lqi-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  b747 \
  --fidelity point_mass_3dof \
  --realization point_mass_3dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --campaign b747-language-backed-guidance-local-lqi-v1

# Pseudo-6DOF adds the declared profile-backed bank sidecar. Pitch and yaw
# remain measured responses of the flight-path and heading targets, not
# invented independent actuators.
taoryx model tune \
  taoryx.registry.mission-composition \
  skywalker_x8 \
  --fidelity pseudo_6dof \
  --realization pseudo_6dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --campaign x8-language-backed-pseudo-guidance-local-lqi-v1

taoryx model tune \
  taoryx.registry.mission-composition \
  b747 \
  --fidelity pseudo_6dof \
  --realization pseudo_6dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --campaign b747-language-backed-pseudo-guidance-local-lqi-v1
```

An explicit registered campaign may select a blocked realization only for this
local design operation. The tuning result marks that choice as
`blocked_local_design_allowed_for_registered_campaign`; it never makes the
same realization available to `model plan`, `model scaffold`, `model compile`,
or a Vehicle Composition mission runner. This lets a plug-in expose valid
source trim/linearization/authority evidence without disguising a missing
end-to-end controller or route as runnable.

`model tune` writes a content-addressed report cache under
`build/controller-cache` by default. The key includes the installed plug-in
catalog fingerprint, registration advertisement, campaign adapter descriptor,
and generated campaign. `model plan` exposes these same rules under
`controller_automation.tuner_selection` and `.tuning_cache`, including the
matching campaign IDs and `--campaign` requirement when selection is
ambiguous. Use `--no-cache` to force reevaluation or `--cache-dir PATH` to
choose another artifact root.
Cache hits avoid rerunning the numerical campaign; they do not currently skip
Python startup, plug-in discovery, or provider-catalog construction. For the
small reference LQI campaigns those fixed costs dominate, so a warm standalone
CLI takes roughly the same time as a cold call. The cache becomes material for
nonlinear screens and larger operating-point grids; a long-lived tuning
session is the separate route to subsecond repeated interactive calls.

The shared sequence is:

```text
declared operating point and scales
  → bounded trim
  → two-step derivative consistency
  → declared-axis authority preflight
  → normalized controller candidate grid
  → local nonlinear-response screen
  → allocation/actuator and mission gates when supplied
```

The shipped A320, both F-16 reductions plus its source-surface local design,
Hummingbird, X8/B747 lower-tier guidance and source-surface local designs, and
the X-15 and HL-20 local direct-wrench bridges currently reach `candidate_ready`.
That campaign status is a local design result, not physical-surface,
nonlinear-mission, envelope, or flight qualification. Hummingbird additionally
has a separate executable individual-rotor allocation screen described above;
X-15 and HL-20 likewise expose separate executable batch-only body-speed LQI
screens that use their registered campaign result through bounded generalized
wrenches. None of those endpoints promotes the campaign beyond its evidence.
The F-16 pseudo-6DOF,
A320/Hummingbird attitude, X8/B747 point and pseudo guidance, and source-surface LQI campaigns
use LQI; the F-16 point-mass campaign uses LQR. Its source-surface tier keeps
both LQR and body-velocity LQI choices, while X-15 and HL-20 each keep both
generalized direct-wrench LQR and local-body-speed LQI choices. Neither LQI campaign
changes a direct-wrench bridge into physical effector allocation.

To run either direct-wrench LQI screen through the public composer:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_local_direct_wrench_lqi_screen_compose.yaml \
  --output build/x15-local-direct-wrench-lqi.composition.json
taoryx vehicle run build/x15-local-direct-wrench-lqi.composition.json \
  --output-dir build/x15-local-direct-wrench-lqi

taoryx vehicle compose \
  examples/vehicle_composition/hl20_local_direct_wrench_lqi_screen_compose.yaml \
  --output build/hl20-local-direct-wrench-lqi.composition.json
taoryx vehicle run build/hl20-local-direct-wrench-lqi.composition.json \
  --output-dir build/hl20-local-direct-wrench-lqi
```

Each result includes the selected campaign ID, controlled output, integral
activity, requested-versus-achieved wrench trace, and the explicit no-wind,
no-mass-variation, no-physical-effector claim boundary. These are batch-only
automatic controller screens; their existing LQR missions remain the separate
interactive direct-wrench bridge endpoints.

### X-15 source-surface authority screen

The X-15 also has a batch-only physical source-surface endpoint,
`x15_source_surface_authority_screen_v1`. It begins at a frozen retained
release/glide source fixture and allocates a three-axis moment request through
the actual source-table symmetric stabilator, differential stabilator, and
rudder coordinates. The result advertises the three bounds, effectiveness
rank, actual surface positions, requested/achieved nonlinear moments, and
allocation status in the common interface, status, and semantic-action trace.

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  x15 \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission x15_source_surface_authority_screen_v1

taoryx vehicle compose \
  examples/vehicle_composition/x15_source_surface_authority_screen_compose.yaml \
  --output build/x15-source-surface-authority.composition.json
taoryx vehicle run build/x15-source-surface-authority.composition.json \
  --output-dir build/x15-source-surface-authority
```

This is intentionally an authority screen, not an LQR/LQI or flight endpoint:
it does not propagate state or establish a full X-15 equilibrium. Its packet
therefore reports `trim.full_state.status = not_available` and does not claim
propulsion/RCS allocation, feedback control, navigation, high-energy guidance,
or flight qualification.

### X-15 source-surface LQI offset screen

`x15_source_surface_attitude_rate_lqi_screen_v1` is the corresponding
two-second local attitude/rate LQI recovery through those same actual source
surfaces. Its physical-wrench profile is explicit in the capability and
runtime advertisements, including the integral-Q diagonal, and the batch
packet emits `robustness_report.json`. That artifact tests the nominal case
plus constant external pitch moments of ±5% of the declared pitch-wrench
scale. The bias enters the plant's declared external-dynamics seam, then the
controller must reject it through the normal wrench request and bounded
surface allocator; it is never injected into the controller or allocator.

This is useful bounded evidence of matched pitch-moment rejection at the
frozen source fixture. It remains neither a wind model, mass variation,
gain-scheduled/adaptive control law, propagated X-15 trajectory, nor flight
qualification.

### HL-20 source-surface authority screen

HL-20 also has a first-class source-surface Composition endpoint,
`hl20_source_surface_pitch_authority_screen_v1`. It exposes all seven named
DAVE-ML surface inputs, their asymmetric bounds, fixed source mass, source
effectiveness rank, allocator residual/status, actual nonlinear pitch moment,
and the source pitch coefficient in the common interface/status/action-trace
artifacts. The screen begins at the checked Mach-1 scalar pitch-coefficient
trim anchor and applies a bounded pitch-authority request through the generic
surface allocator and declared local lag model.

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  hl20_mod_k \
  --fidelity rigid_body_6dof_surface_allocated \
  --realization rigid_body_6dof_surface_allocated \
  --mission hl20_source_surface_pitch_authority_screen_v1

taoryx vehicle compose \
  examples/vehicle_composition/hl20_source_surface_pitch_authority_screen_compose.yaml \
  --output build/hl20-source-surface-authority.composition.json
taoryx vehicle run build/hl20-source-surface-authority.composition.json \
  --output-dir build/hl20-source-surface-authority
```

This is deliberately not an LQR/LQI or glide endpoint. The retained source
evidence establishes only scalar pitch-coefficient trim, not a six-DOF
equilibrium with attitude/gravity/position propagation. Consequently the
endpoint explicitly reports `trim.full_state.status = not_available`, no
controller campaign, and no navigation/guidance claim. The next promotion is
to supply that full-state trim binding before surface-feedback tuning can be
made honest.

### HL-20 source-surface attitude/rate LQI screen

The separate `hl20_source_surface_attitude_rate_lqi_screen_v1` endpoint fixes
the same checked Mach-1 translation fixture, then closes local roll, pitch,
yaw, and body-rate feedback with all seven bounded source surfaces. Its
eight-second batch retains requested/achieved/residual pitch moments, surface
allocation status, source-fixture velocity telemetry, and a typed,
composition-bound `robustness_report.json`.

That report replays the nominal perturbation plus constant external pitch
moments of ±5% of the declared 100,000 N m pitch-wrench scale. The disturbance
is applied after source-surface loads through the full-inertia pitch dynamics
seam; it is not added to the controller request or allocator output. This is
executed narrow offset evidence only—not a wind model, mass-variation result,
full-glide trim, route, gain schedule, or flight qualification.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hl20_source_surface_attitude_rate_lqi_screen_compose.yaml \
  --output build/hl20-source-surface-lqi.composition.json
taoryx vehicle run build/hl20-source-surface-lqi.composition.json \
  --output-dir build/hl20-source-surface-lqi
```

The A320 and F-16 pseudo-6DOF LQI candidates additionally have focused
nonlinear native-coordinate recovery proofs. A320 uses its declared surrogate
control bounds; F-16 uses its source-overlay coordinate travel bounds without
applying that overlay's actuator lag. Both records retain integrator and
control-saturation telemetry, but neither upgrades a response-law campaign to
physical allocation, route, schedule, or flight qualification.

Every installed Mission Composition model supports `model plan` and
`model scaffold`. Models without a registered campaign are still useful and
honest: controlled realizations report `campaign_registration_required`,
provider-internal controls report `provider_managed`, and a provider-owned
screen that also offers a source-local tuning campaign reports
`provider_managed_with_campaign`. Uncontrolled or source-replay models report
tuning as not applicable.

## Vehicle discovery cards and common taxonomy

The full Vehicle Composition catalogue is the single developer-facing join
for identity, source-owned physical characteristics, fidelity semantics,
control authority, configuration inputs, and segment planning:

```bash
taoryx vehicle catalog --detail full
taoryx vehicle describe x15
taoryx vehicle parameters x15
```

Every family declares a concise summary, vehicle class, operating domains,
propulsion kinds, normal roles, tags, model basis, and claim boundary. Numeric
geometry, mass, envelope, and effector records are projected only from
`vehicle_models.yaml` or a typed source-family manifest. An envelope value such
as `max_mach` is a model-validity bound, not a demonstrated maximum vehicle
speed. The card uses discriminated physical-characteristic alternatives rather
than nulls: `fixed_reference_geometry`, `variant_geometry`,
`fixed_mass_properties`, `scheduled_mass_properties`,
`declared_validity_envelope`, `declared_effectors`, `not_applicable`, or
`not_represented`. The serialized card retains `unavailable_fields` for quick
UI filtering, but callers should preserve the typed alternative until their
own final serialization boundary.

Each of the four fidelity records includes a common display name and model
meaning plus the family-specific profile. Pseudo-6DOF cards expose their
declared roll, pitch, and yaw response limits. Direct-wrench cards expose
force and moment axes. Allocated tiers expose the declared allocator and
effector channels. Exact action bounds and authority modes remain in the
fingerprinted interface and are joined back into each tier by the typed
`VehicleControlAuthorityAdvertisement`; `control_authority` is its JSON/API
projection.

Initialization, runtime-variant, and segment inputs are deliberately separate
configuration scopes. Their public records carry semantic roles such as
position, speed, mass, event time, energy, or control limit; a runtime variant
still requires its exact native binding and committed-status evidence.

Internally, the catalogue keeps a typed `VehicleFidelityAdvertisement` and
`VehicleAuthoringTierAssessment` for each tier/mission choice. Batch/episode
parity is likewise resolved as a typed advertisement before producing the
authoring worklist or kit. Plug-ins should use these contracts rather than
serializing a catalogue card and reading it back as an argument bag.

Every segment resolves to the common category, execution style, and lifecycle
taxonomy. `segment_planning` compares the declared segment set with normal
capability slots for its vehicle class and reports missing categories without
promoting a planned segment or inventing an implementation. A plug-in may use
a common segment ID or provide an inline typed taxonomy for a genuinely new
segment kind.

## What a new model must provide

Use this progression instead of beginning with manual segment code or gains:

1. Publish model identity, provenance, properties, fidelities, frames, and
   portable configuration/output schemas through a Mission Composition
   provider.
2. Publish realizations with exact input realization, control channels,
   authorities, intents, telemetry, and availability blockers.
3. Publish initialization choices, mission templates, ordered segment types,
   termination modes, and every route/waypoint parameter. Register a capability
   compiler when generic schema lowering is insufficient.
4. For controller automation, implement the common family-adapter operations
   needed by the tier: state derivatives, trim, linearization, effectiveness,
   allocation, resource rates, observations, or replay.
5. Supply a `ControlAutomationDeclaration` with bounded operating points,
   scales, authority states, and offset-free outputs; core generates the
   normalized LQR/LQI campaign.
6. Add independent nonlinear, actuator/allocation, mission-objective, and
   envelope evidence before promoting the result.

Data-only contributions can complete steps 1–3 and immediately gain generic
discovery, planning, scaffolding, and validation. Steps 4–6 are what turn that
data into an automatically tunable and executable controlled model.

For the tier-specific version of this checklist—3DOF force model, pseudo-6DOF
response law, rigid-body direct wrench, or actuator-resolved rigid body—see
[Fidelity tiers and vehicle plug-in requirements](../architecture/fidelity-data-requirements.md).

### Replay and release witnesses

`taoryx vehicle maturity-report` also publishes the family strategy worklist.
Each tier retains its `pending_operations`; a source history, source-scheduled,
open-loop, or passive batch witness does **not** become a derivative, trim, or
controller-adapter probe. When such a runnable batch exists, the row exposes
its `runnable_batch_execution_modes`, `runnable_batch_missions`, and bounded
claim text, and may direct the author to the first source/release audit rather
than incorrectly blocking that audit on a future generic adapter.

For example, X-15's staged open-loop witness starts `release_audit`, HL-20's
source-scheduled witness starts `release_audit`, and NESC's history replay
starts `source_audit`. Their worklist rows still list any missing
`state_derivative`, trim, allocation, or controller work. A replay row is
therefore executable evidence with its own truthful boundary—not automatic
promotion to a participating plant, closed-loop mission, or qualification.

## Verification

Changes to this layer should at minimum exercise:

```bash
python -m pytest tests/unit/test_model_authoring.py
python -m pytest tests/unit/test_plugins.py
python tools/dev.py typecheck
python tools/dev.py lint
```

Repository-wide completion still requires the normal gates in `AGENTS.md`.
### X8 source-table physical screen

Skywalker X8 has batch-only physical local LQR and LQI screens at
`x8_local_physical_surface_lqr_screen_v1` and
`x8_local_physical_surface_lqi_screen_v1`. Each derives its declared
roll/pitch controller from the pinned nonlinear source-table plant, allocates
each requested moment through bounded collective/differential elevon
coordinates, and writes the requested, achieved, residual, actual-coordinate,
resource, status, and control-trace artifacts. The LQI screen also retains the
output-integrator state in its nonlinear validation artifact and emits
`robustness_report.json`. That standard eight-second LQI packet retains its
nominal recovery plus constant external pitch moments of ±5% of the declared
pitch-wrench scale. The moment enters the explicit local plant-dynamics seam
after source-table loads; it is never added to the controller request or
allocator output. Both leave yaw
as an unallocated *independent* axis and do not claim individual left/right
servo wiring or a completed racetrack. Their advertisements also publish a
source-linearized coupled-lateral diagnostic:
the real differential-elevon coordinate can influence local roll, yaw,
sideslip velocity, roll rate, and yaw rate. That result is a candidate gate for
future controller work, not nonlinear yaw/sideslip-recovery evidence or a
surface-racetrack promotion.

`x8_local_physical_surface_lqi_long_recovery_screen_v1` is the composable
extended-recovery endpoint. It freshly solves the pinned source-table trim for
both its capability estimate and its execution, records the actual nonzero
throttle and elevon coordinates, and then runs the same bounded roll/pitch
LQI realization for twenty seconds at a fixed 0.02-second cadence. This is
stronger local recovery evidence than the eight-second screen, but it remains
neither a yaw/sideslip, wind/mass-variation, racetrack, gain-schedule,
servo-wiring, nor flight-qualification claim. The separate standard LQI
packet is the bounded fixed pitch-offset evidence; the long-recovery endpoint
does not extend that screen.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x8_local_physical_surface_lqr_screen_compose.yaml \
  --output build/x8-local-surface-screen.composition.json
taoryx vehicle run build/x8-local-surface-screen.composition.json \
  --output-dir build/x8-local-surface-screen
taoryx vehicle result build/x8-local-surface-screen \
  --composition build/x8-local-surface-screen.composition.json

# Substitute x8_local_physical_surface_lqi_screen_compose.yaml to execute
# the LQI controller and its matched pitch-offset artifact; it is a separate
# exact composition endpoint.

# Use x8_local_physical_surface_lqi_long_recovery_screen_compose.yaml for
# fresh source-powered trim plus the twenty-second extended LQI recovery.
```

The `x8-source-surface-local-lqi-v1` campaign is the common-host
candidate-design workflow for the same local operating point. It exposes an
integral-priority lattice around the physical profile's `(0.15, 0.15)` base
weight, with multipliers `0.1`, `1`, `10`, and `100`. Those source-coordinate
candidates are a design aid; they are not silently substituted for the
allocator-runtime gain. The capability advertisement separately names the
applied physical-wrench profile,
`skywalker-x8-source-trim-roll-pitch-wrench-lqi-v1`, and ties its selected
integral-Q diagonal to the emitted pitch-offset screen. Its integrators
deliberately exclude yaw. This is bounded matched pitch-moment rejection, not
wind or mass robustness. Neither local LQI screen nor LQR screen promotes the
X8 surface-allocated racetrack beyond its explicit development blocker.

### X8 and B747 lower-tier kinematic guidance

The runnable X8 and B747 point-mass racetracks advertise an explicit
`kinematic_guidance` authority: an enable gate plus bounded speed, flight-path
angle, and heading targets. The pseudo-6DOF counterparts add a bounded bank
target and publish `attitude.euler` and `body_rate` from their declared
kinematic response profile. With `guidance.override.enabled: true`, those
targets replace the native route and sidecar targets at the native 0.05-second
guidance cadence. `guidance.override.active` reports whether that state-law
override is currently in force.

The accompanying `native_control_bridge` remains available for source-table
throttle/surface load probing, but is explicitly marked `tuning_eligible:
false` and `state_authority: diagnostic_only` at these lower fidelities. It
does not claim that a throttle or surface command controls the reduced route
state. Use `x8-language-backed-guidance-local-lqi-v1` or
`b747-language-backed-guidance-local-lqi-v1` for the exact point-mass local
LQI candidate workflow. Their pseudo counterparts,
`x8-language-backed-pseudo-guidance-local-lqi-v1` and
`b747-language-backed-pseudo-guidance-local-lqi-v1`, additionally tune the
closed altitude/speed/flight-path/heading/bank guidance subsystem against the
declared profile. Pitch and yaw are advertised as coupled response telemetry,
not independent LQI authority. Neither campaign tunes physical effectors,
route tracking, wind rejection, mass scheduling, or a complete mission.

### X8 source direct-wrench route

`x8_racetrack_direct_wrench_compose.yaml` is a separate batch-only nominal
source-route witness. Its X8-owned translator preserves the validated source
packet: 700 m legs, 250 m turns, 8-degree same-sense turns, and the source
altitude and position capture settings. The direct force/moment interface is
therefore advertised as planned rather than external: the source packet owns
those commands and the public runner records an action-free committed interval
trace.

```bash
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_direct_wrench_compose.yaml \
  --output build/x8-direct-wrench.composition.json
taoryx vehicle run build/x8-direct-wrench.composition.json \
  --output-dir build/x8-direct-wrench
taoryx vehicle result build/x8-direct-wrench \
  --composition build/x8-direct-wrench.composition.json
```

This establishes an exact source-autonomous nominal mission path. It does not
establish external six-axis control, physical elevon allocation, gain
scheduling, mass or wind robustness, envelope qualification, or flight
qualification. Use the X8 physical local screen for the separately advertised
surface-allocation evidence.

### B747 condition-3 physical screen

The B747 has batch-only physical local LQR and LQI screens at
`b747_condition3_local_physical_surface_lqr_screen_v1` and
`b747_condition3_local_physical_surface_lqi_screen_v1`. They use the pinned
NASA CR-2144 condition-3 source trim, derive the shared three-axis LQR from
the nonlinear source-table plant, and allocate requested moments through the
bounded elevator, aileron, and rudder coordinates. They emit requested,
achieved, residual, actual-coordinate, status, resource, and control-trace
artifacts; the throttle coordinate is retained at trim. The source package has
no servo rate or lag data, so the screen explicitly reports ideal bounded
coordinates rather than inventing actuator dynamics. The standard LQI packet
also emits `robustness_report.json`: nominal plus constant external pitch
moments of ±5% of its declared condition-3 pitch-wrench scale. The load enters
the explicit body-dynamics seam after source-table loads and must be rejected
through the normal controller and bounded source-table allocation.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/b747_condition3_local_physical_surface_lqr_screen_compose.yaml \
  --output build/b747-condition3-local-surface-screen.composition.json
taoryx vehicle run build/b747-condition3-local-surface-screen.composition.json \
  --output build/b747-condition3-local-surface-screen
taoryx vehicle result build/b747-condition3-local-surface-screen \
  --composition build/b747-condition3-local-surface-screen.composition.json
```

`b747-source-surface-local-lqi-v1` is the common-host candidate-design
workflow for the same source operating point. It exposes an integral-priority
lattice around the physical profile's `(0.025, 0.025, 0.025)` base weight, with
multipliers `0.1`, `1`, `10`, and `100`. Those source-coordinate candidates are
a design aid, not an implicit physical-wrench runtime binding. The capability
advertisement separately names the applied physical profile,
`b747-condition3-source-surface-wrench-lqi-v1`, and connects that selected
profile to the emitted pitch-offset evidence. This is bounded matched
pitch-moment rejection only—not wind or mass robustness. Neither route
establishes a gain schedule, transport racetrack, servo or engine dynamics, or
B747 flight qualification.

### B747 source direct-wrench route

`b747_racetrack_direct_wrench_compose.yaml` is a separate batch-only route
lane. Its plug-in-owned translator preserves the source packet's autonomous
capture gains, settling margin, and non-coordinated turn geometry; the generic
capability planner is deliberately not allowed to rewrite those controls. The
composition still supplies its initialization and route/segment geometry, but
`wrench.force.command` and `wrench.moment.command` remain planned rather than
external actions. The committed batch trace therefore records zero requested
public action channels while retaining truth, status, resource, diagnostics,
and source-control provenance.

```bash
taoryx vehicle compose examples/vehicle_composition/b747_racetrack_direct_wrench_compose.yaml \
  --output build/b747-direct-wrench.composition.json
taoryx vehicle run build/b747-direct-wrench.composition.json \
  --output-dir build/b747-direct-wrench
taoryx vehicle result build/b747-direct-wrench \
  --composition build/b747-direct-wrench.composition.json
```

This is a source-autonomous nominal mission witness, not an externally driven
six-axis controller, a surface-allocation path, a gain schedule, mass/wind
robustness evidence, or B747 flight qualification. Use the B747 condition-3
physical screen for the separately advertised source-table surface-allocation
evidence.

### A320 OpenAP reduced routes

`a320_openap_3dof` exposes two executable reduced adapters: the OpenAP
point-mass performance product and the named OpenAP/JSBSim pseudo-6DOF
response product. Both support the common derivative, trim, and
linearization seams, and both run the reusable racetrack through Composition.
The pseudo-6DOF route additionally exposes its achieved attitude/body-rate
response. Each batch result advertises committed dynamic pressure, installed
thrust, realized throttle, total mass, and OpenAP fuel-flow estimate.

The horizontal readiness worklist binds these routes to their checked
data-evidence artifacts: the OpenAP integration proof for point mass and the
matched OpenAP/JSBSim common-channel comparison for pseudo-6DOF. Thus both
reduced tiers are `probe_ready` rather than falsely blocked for missing
metadata. That evidence does not promote either tier: the declared operating
point and response-model expansion blockers remain visible, and the two
rigid-body tiers remain planned pending a participating physical plant and
surface-effectivity evidence.

Its registered attitude LQI campaign also has a focused nonlinear
native-coordinate recovery proof: the retained LQI gain drives the declared
aileron/elevator/rudder response coordinates through the OpenAP/JSBSim
surrogate. The result records integrator use and native-control saturation;
it remains distinct from a physical Airbus surface allocator or actuator
model.

The point-mass product also has a nominal cruise-performance LQR campaign in
its native throttle/flight-path coordinate basis:

```bash
taoryx model tune \
  taoryx.registry.mission-composition \
  a320_openap_3dof \
  --fidelity point_mass_3dof \
  --realization point_mass_3dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --campaign a320-point-cruise-performance-lqr-v1
```

Select the named response-law realization explicitly when creating an
authoring draft or handing the route to an agent. The returned plan includes
the registered LQI campaign, bounded generic guidance controls, and the exact
common batch and interactive endpoints.

```bash
taoryx model plan \
  taoryx.registry.mission-composition \
  a320_openap_3dof \
  --fidelity pseudo_6dof \
  --realization jsbsim_surrogate_composite_pseudo6dof \
  --mission powered_fixed_wing_racetrack_v1 \
  --output build/a320-jsbsim-surrogate-plan.json
```

```bash
taoryx vehicle compose \
  examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml \
  --output build/a320-pseudo-racetrack.composition.json
taoryx vehicle run build/a320-pseudo-racetrack.composition.json \
  --output build/a320-pseudo-racetrack
taoryx vehicle result build/a320-pseudo-racetrack \
  --composition build/a320-pseudo-racetrack.composition.json
```

These are reduced performance and response-law products. They do not claim an
authoritative Airbus 6DOF plant, physical control-surface allocation,
actuator dynamics, or flight qualification.

### Passive tumbling-body releases

`tumbling_body` exposes both of its executable direct-release reductions
through the same public passive adapter: `point_mass_3dof` explicitly uses an
orientation-averaged projected area, while `pseudo_6dof` reuses the native
rigid-body passive rotation equations. Both Composition batch results publish
committed position, velocity, mass, drag force, projected area, angular-rate
magnitude, and—for the pseudo route—attitude and body rate. They deliberately
advertise no controller, derivative, trim, wrench, or effector allocation
operation.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml \
  --output build/tumbling-pseudo-release.composition.json
taoryx vehicle run build/tumbling-pseudo-release.composition.json \
  --output build/tumbling-pseudo-release
taoryx vehicle result build/tumbling-pseudo-release \
  --composition build/tumbling-pseudo-release.composition.json
taoryx vehicle verify tumbling-body-passive-release-pseudo6dof --execute
```

The focused endpoint explicitly reports its controller and robustness gates as
`not_applicable`: it has no control or declared uncertainty seam. This is a
declared geometry/release-to-impact witness, not an independent source-vehicle
model, shape-wide damping qualification, or a qualified rigid-body 6DOF
aerodynamic-moment contract.
