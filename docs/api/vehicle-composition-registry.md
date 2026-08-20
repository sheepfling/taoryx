# Vehicle Composition Registry

The vehicle composition registry is the user-facing index for composing a
trajectory. It does not replace a vehicle's source data, nonlinear plant, or
qualification record. Instead, it joins those authorities so a caller can ask
one question:

> For this vehicle family and fidelity, what may I initialize, chain, and
> control?

The source overlay is [vehicle_composition_registry.yaml](../../verification/vehicle_composition_registry.yaml).
It is validated against the horizontal fidelity registry and the unified
family-manifest join before it can be inspected.

Mission Composition projects this registry together with native execution
bindings, Simple Aero, and dual-launch assets. The authoritative cross-catalog
disposition is
[mission_composition_inventory.yaml](../../verification/mission_composition_inventory.yaml),
and the generated
[family/realization matrix](../architecture/mission-composition-coverage-matrix.md) records exact
batch/session availability, normalized dynamics and control metadata,
telemetry, spawned-child behavior, and blockers. Model-level readiness is only
an index that at least one exact tuple is registered; it is never blanket
permission to run every fidelity or mission.

## Discovering a vehicle

```bash
taoryx vehicle catalog --detail summary
taoryx vehicle catalog --detail full
taoryx vehicle list
taoryx vehicle topology-report
taoryx vehicle inspect skywalker_x8
taoryx vehicle schema skywalker_x8 initialization
taoryx vehicle schema skywalker_x8 segments
taoryx vehicle schema skywalker_x8 missions
taoryx vehicle endpoints skywalker_x8
```

`catalog --detail summary` and `list` are intentionally compact.
`catalog --detail full` embeds every full composition descriptor so a remote
client need not iterate local registry IDs. `inspect` returns the same full
composition contract for one selected vehicle, including the four canonical
fidelity tiers:

1. `point_mass_3dof` — force-model translation.
2. `pseudo_6dof` — named attitude/rate response law.
3. `rigid_body_6dof_direct_wrench` — rigid-body plant with explicitly
   labeled generalized-wrench control.
4. `rigid_body_6dof_surface_allocated` — rigid-body plant with requested
   wrench allocated through declared physical effectors.

Each entry reports whether the profile is declared, its promotion status,
required adapter operations, and remaining blockers. A declared profile is
not automatically a runnable or qualified vehicle claim.

A family may publish a **capability adapter** before it has a semantic
translator or runtime endpoint. This is a planning-only seam: the capability
estimate can expose necessary-condition evidence while preflight remains
`blocked` with no semantic lowering. It must never be rendered as an
executable trajectory, trim result, controller result, or qualification claim.
The HL-20 is the next promotion stage: its reduced-fidelity public glide
mission has an exact semantic release/trim/opposing-bank/energy-handoff
lowering, so preflight is `translation_ready`, but native execution remains
blocked until a source-owned release/trim/glide runtime exists. A semantic plan
is an auditable mission contract, not a substitute for a plant or controller.

Parameter identity carries mathematical meaning. Fixed-wing `bank_limit_deg`
is a nonnegative magnitude paired with discrete `turn_direction`; HL-20
`bank_command_deg` is a signed bounded command whose sign denotes turn sense.
Reusing one identifier for both would hide a topology change from clients and
make an optimizer's search space ambiguous.

`endpoints` reports a separate and stricter capability: the exact source-owned
batch runner or accepted-truth episode factory currently registered for a
family/mission/fidelity tuple. The declaration lives in
[vehicle_execution_bindings.yaml](../../verification/vehicle_execution_bindings.yaml).
An entry can be `runnable` or `planned` with concrete blockers. Omission means
there is no execution path; TAORYX never falls back to a nearby family or a
generic force/moment model. Every endpoint also declares its `execution_mode`:
`closed_loop_controller`, `source_history_replay`,
`source_scheduled_replay`, `open_loop_witness`,
`local_direct_wrench_screen`, `passive_uncontrolled`, or `planned`. These are
not interchangeable evidence tiers. For example, a direct-wrench screen is a
local bridge-mode diagnostic, while a source replay is batch-only provenance
evidence rather than an externally controllable episode.

Each endpoint also declares `batch_action_trace`. This says whether a batch
execution emits the standardized, identity-bound held-command artifact;
explicitly lacks accepted-interval command history; has not yet emitted the
artifact; has no batch-action obligation; or is planned. Controls that happen
to appear in a committed state/status row are not automatically a command
trace: the runtime must retain the action held over the preceding accepted
integration interval before Mission Composition exposes it as requested-control
evidence.

An uncontrolled source replay or open-loop witness may emit this artifact with
empty action and effector channel sets. That is positive evidence that the
selected interface declares no public command—not an invented zero throttle,
zero gimbal, or inactive surface command. A controlled/direct-wrench endpoint
must instead publish its actual held semantic request, and an endpoint with
declared action/effectors cannot use the empty-trace form.

Runnable episode entries are additionally cross-checked with the runtime's
explicit factory registry. Every advertised factory ID must have exactly one
constructor in `registered_episode_factory_ids()`; an implementation-only
factory is also rejected because no vehicle endpoint selected it. This is a
dispatch-integrity check, not a generic simulator fallback or evidence
promotion mechanism.

For a release-oriented cross-family audit, use:

```bash
taoryx vehicle maturity-report
taoryx vehicle maturity-report --check-execution-witnesses
taoryx vehicle semantic-preflight-handler-report
```

The first is a fast declaration/coverage audit; the second additionally
compiles every checked-in endpoint witness and opens every declared episode.
Both preserve separate topology, endpoint, parity, and qualification
boundaries rather than returning one misleading fleet score.

When the witness audit executes a public batch endpoint, the generated packet
must also contain verified `mission_graph_execution.json` evidence. An
unobserved record is valid when the native executor did not emit controller
dispatches, but it must be explicit and composition-bound; a missing or
invalid record fails the batch witness. This prevents a normalized terminal
result from silently standing in for segment sequencing or committed-state
handoff evidence.

An opened episode exposes the complementary **graph-binding** projection in
its episode-contract report: the immutable graph status, entry instance,
ordered segment instances, and family-declared graph execution contract. The
projection is deliberately `not_emitted_by_episode` for transition evidence;
interactive reset/step calls may expose semantic controls and committed truth,
but they do not silently certify that a mission translator dispatched the
full graph. Batch `mission_graph_execution.json` remains the evidence source
for observed transitions.

The X-15 local direct-wrench screen is a deliberately narrow registered
batch/episode pair. Its replay witness compares one held semantic total-wrench
stream through the source-local bounded projection and local derivative at
each committed boundary. This supplies Runtime / Composition stepping evidence for the
bridge tier only; it does not claim physical X-15 effectors, trim, release,
or end-to-end flight behavior.

The topology report also contains `canonical_channel_coverage` and the
resource-only `resource_channel_coverage` projection. Each canonical hook
lists the exact vehicle/fidelity bindings, availability, provenance, and
sampling semantics. This gives a UI, optimizer, or RL integration a direct
answer to “which current models publish total mass, battery reserve, heading,
or a semantic command?” without converting an absent resource into zero or
assuming a source-specific channel is universal.

Initialization and segment inputs have an equally explicit mathematical
contract in [parameter_value_space_catalog.yaml](../../verification/parameter_value_space_catalog.yaml).
It declares the representation, topology, and relevant frame for each public
semantic parameter. Units alone are not used to infer topology: for example,
NED north/east coordinates are signed Euclidean values, whereas altitude and
duration are nonnegative magnitudes; headings live on a periodic circle; and a
quaternion is an `SO(3)` representation. Registry loading fails if an exposed
input lacks this catalog entry, and `topology-report` reports catalog coverage
separately from interface-channel coverage.

Every numeric public status, resource, and diagnostic channel must likewise
advertise either a `canonical_unit` or explicit `quantity_semantics`. The
unitless forms are deliberately narrow: `count` for cardinality/rank,
`normalized_error` for a scale-normalized feedback metric, and
`mixed_wrench_norm` where combining forces and moments has no honest single
physical unit. A plug-in must not label a mixed norm as `N` or `N*m` merely to
fill a field; publish separately unit-typed residual components when a
consumer needs them. `taoryx vehicle topology-report` enforces this contract.

`semantic-preflight-handler-report` is the narrow structural audit beneath
the maturity report. For every declared family/mission/fidelity translator it
reports the matching tier-scoped capability adapter, whether that adapter ID
and an immutable source-owned handler are installed, and the next integration
action. A declared translator without a handler or a declared planner without
an installed adapter is a fail-closed catalog error; a declared capability
adapter without a translator remains an explicit `translator_pending`
development state. Neither result proves that a concrete composition is
feasible or that its native vehicle execution is qualified.

When the endpoint witness matrix is checked, each `translation_ready`
composition additionally emits a
`taoryx.concrete-capability-preflight/v1alpha1` record. It names the selected
family-owned capability adapter and feasibility disposition, binds both to the
immutable composition identity, and records a SHA-256 of the exact derived
mission used by semantic preflight. The witness validator recomputes the hash
before accepting endpoint readiness. Use `taoryx vehicle maturity-report
--check-execution-witnesses` for the cross-family summary. This remains only a
semantic feasibility/translation record: it does not prove adapter binding,
trim, control allocation, integration, truth-objective completion, or
qualification.

Each current batch executor retains that same record as `preflight.json` in
its result packet. `taoryx vehicle results <directory>` validates the record
against the colocated compiled composition and projects it as
`capability_preflight_evidence`; a changed adapter, identity, feasibility
class, or derived-mission hash invalidates the packet. A legacy or external
packet without this sidecar is reported as `missing`, never upgraded by
trajectory geometry.

## Compile a selected mission

The registry is also an executable semantic boundary. A user selects one
vehicle, canonical fidelity tier, initialization contract, mission template,
and the template's exact ordered segment instances in a compact YAML request:

```bash
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_compose.yaml \
  --output generated/x8-racetrack-semantic.json
```

The compiler rejects unknown inputs, missing required inputs, noncanonical
units, a fidelity not declared by the family, an initialization not allowed by
the chosen mission, or a reordered/skipped segment sequence. Each following
segment receives the preceding segment's terminal truth state unless the
segment explicitly declares a physical transition.

Every initialization and segment input is a shared `ParameterSpec`. In addition
to ID, unit, required flag, representation, and value-space topology, its
descriptor carries explicit default disposition, hard/qualified/safe-extended
bounds, optimization transform, conditional visibility, coupling group,
projection policy, derivation, invalidations, and provenance. These fields are
not guessed for imported data: a missing value stays `null`, a missing default
is `default_declared: false`, and the default policy is `reject_invalid`.

`taoryx vehicle topology-report` also includes a
`parameter_contract_maturity` inventory for each family and for the catalog as
a whole. It counts declared defaults, explicit and effective hard bounds,
qualified/safe-extended ranges, transforms, provenance, derivations, and
runnable runtime-backed variants. Counts describe documented coverage, not
quality or qualification; the point is to make the next integration data gap
visible before a controller or mission author starts hand-tuning around it.
The same inventory appears in `vehicle describe <vehicle>` and
`vehicle authoring <vehicle>`, so a contributor can see the data-contract gap
for one selected family without first interpreting the fleet-wide audit.

The executable runtime-variant path currently consumes scalar values only.
Each runnable binding must declare a `value_space_profile` as well as its
transform and hard bounds; an inferred positive range from a lower bound is not
accepted as an advertised runtime modifier. This is separate from the shared
parameter catalog because a variant binding has its own semantic identity and
runtime input path.
Accordingly it supports `identity`, `log`, and `logit`: `log` requires a
strictly positive hard lower bound, and `logit` requires explicit `[0, 1]`
hard bounds and exposes a unit-interval value space. A simplex or categorical
modifier must use a future vector/discrete binding with explicit membership or
options; the scalar path rejects those labels rather than misrepresenting them
as ordinary numeric inputs.

Compilation preserves that same value-space record, transform, and hard bounds
inside each immutable runtime binding. A later runtime adapter sees the native
input path *and* the semantic scalar contract that authorized it; neither a UI
nor an optimizer needs to reverse-engineer topology from a field name or unit.

An authoring kit also carries the common truth-objective topology schema. That
keeps periodic targets such as heading, linear rates, unit gate normals, dwell
time, and discrete objective events mathematically explicit alongside the
vehicle-specific segment inputs. A family still must declare which objectives
it actually uses; the common schema does not fabricate one.

### Resolve a bounded vehicle variant

`vehicle parameters <vehicle> --scope variant_configuration` lists only
declared, independently selectable variant inputs. A variant request is not a
recursive override of the model asset: every input must name a source-owned
runtime binding, have an explicit value space and hard bounds, and say whether
it invalidates trim or qualification. An input with `status: planned` cannot
compile. A runnable binding replaces its declared initialization input before
the native translator opens the plant; supplying both forms is an error.

The default resolution policy is `reject_invalid`. A future family may opt in
to `project_to_valid` for a finite scalar optimizer proposal, but only with
declared hard bounds. Such a projection is never silent: the compiled variant
records the requested and resolved values, normalized projection distance, and
the bound(s) that applied. Projection cannot repair an unknown unit, a
non-finite value, an undeclared input, or a value-space invariant without a
corresponding valid hard-bound result. Existing runnable A320 and Hummingbird
mass bindings deliberately retain strict rejection.

For example, the A320 clean-performance slice can select an operating mass
inside the pinned OpenAP table domain. The value is forwarded to
`A320OpenAPOperatingPoint.mass_kg`, then used by the trim solve and runtime:

```bash
taoryx vehicle parameters a320_openap_3dof --scope variant_configuration
taoryx vehicle compose \
  examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml \
  --output generated/a320-racetrack-mass-variant.json
taoryx vehicle run generated/a320-racetrack-mass-variant.json \
  --output-dir artifacts/composition/a320-racetrack-mass-variant
```

The binding's 42,600--78,000 kg hard range is a clean OpenAP source-table
domain, not a mission-qualified load envelope. The compiled artifact records
the runtime path, provenance, requested value, mandatory retrim and
requalification invalidations, and an explicit variant qualification state.
`baseline` means no variant was selected, `qualified` requires a declared
qualified range, and `extended` means the value is hard-valid but not covered
by qualified evidence. Current A320 and Hummingbird mass modifiers are
therefore `extended`, never silently qualified. Most other families remain
`planned` until they have the same source-owned execution binding.

The result is an immutable **semantic adapter handoff**, not a completed
simulation. It records the selected native adapter, fidelity profile, required
adapter operations, promotion blockers, backing templates, and exact
inputs/units. Runtime lowering remains family-adapter work: this layer does
not create hidden forces, moments, effectors, or qualification claims.

For a selected runnable variant, the executor must also emit
`taoryx.variant-runtime-evidence/v1alpha1`. This binds the resolved value to
the **exact native input that was consumed**, rather than merely repeating the
requested initialization. It checks the first and final committed-truth status
samples against the binding's declared status relation. `equal_to_target`
means that the initial status value equals the variant value; `not_asserted`
means only that the named status channel was emitted. The current A320 and
Hummingbird mass bindings use `equal_to_target` for
`resources.mass.total`. This is intentionally not a claim about unmodeled
fuel, inertia, payload distribution, or endurance. A consumption or relation
mismatch fails the runtime hard gate. `vehicle results` fingerprint-binds a
present artifact to the compiled composition and rejects a cross-composition
record rather than accepting a plausible mass trace.

`taoryx vehicle authoring <vehicle>` additionally exports a
`variant_worklist`. It is the admission view for optional vehicle variation:
each declared modifier identifies its target, exact native input path, coupled
resource policy, committed-status channels, invalidations, and the next
source-owned work. A family with no safe modifier reports `not_declared` with
an explicit prohibition on generic model-field mutation. The aggregate
`taoryx vehicle authoring-all` report counts `runnable`, `planned`, and
`not_declared` variant spaces separately. This lets an integration author see
that, for example, the NESC source replay has a meaningful mass trace but no
safe runtime mass knob: modifying that trace without a participating rocket
plant would be a counterfeit parameterization.

Promotion is intentionally sequential:

1. bind the semantic modifier to an exact input consumed by the native plant;
2. declare every coupled mass, resource, geometry, or trim consequence;
3. emit the named values at committed truth boundaries and add an exact
   variant execution witness; and
4. record retrim/requalification invalidations before exposing it as runnable.

Passing this admission sequence establishes runtime provenance only. It does
not qualify the modified vehicle or widen a source model's envelope.

### Author a typed mission graph without inventing execution support

The request may optionally add `mission_graph` over the selected named segment
instances. Its nodes declare a `success_transition`, `abort_transition`,
`resource_limit_transition`, `envelope_limit_transition`, or
`timeout_transition`; every edge carries an explicit state-transfer rule.
The compiler requires an exact node set, known targets, reachability from the
declared entry, and an acyclic graph. A terminal `null` transition is a named
outcome, not a timeout silently counted as success.

Today, checked-in family translators accept the template-owned
`linear_sequence_only` graph and an exact caller-authored projection of that
same chain (`authored_linear_sequence_lowered`). A caller-authored branch,
fallback, timeout, resource, envelope, or alternate state-transfer graph
normally compiles to `authored_graph_not_lowered`, then `preflight`, `lower`,
and `run` return a structured blocker. This remains the default: graph
authoring is useful for UI and tooling integration, but no vehicle may execute
a branch that its native translator has not declared.

That opt-in lives in the affected mission template's typed
`graph_execution_extension` record. It names exactly one compatible fidelity,
the native translator, supported and rejected transition kinds, allowed truth
state transfer, any bounded timeout rule, and its claim boundary. Preflight
and runtime lowering query this registry-owned record through
`mission_graph_execution_contract`; they do not retain family-name conditionals.
An omitted record means template-owned `success` transitions only.

Each mission template may additionally declare `semantic_translator_id`. This
is the exact source-owned lowering expected whenever preflight returns
`translation_ready`; it is distinct from a capability estimator and from a
batch or episode factory. The preflight wrapper rejects a ready result whose
reported translator does not match the selected template, and blocked results
retain that same selected translator rather than being mislabeled as a
fixed-wing failure. A missing ID leaves the mission discoverable but prevents
it from receiving a ready execution-lowering claim.

`mission_capability_adapter_id` declares the separate family-owned first-pass
feasibility planner. Both IDs carry their own explicit fidelity lists. For
example, the Hummingbird mission is discoverable at several tiers but its
aggregate-thrust capability planner and hover/yaw translator currently exist
only at pseudo-6DOF. An ID without a compatible-tier list is rejected, as is a
runtime adapter whose ID disagrees with the selected template declaration.

Semantic preflight is dispatched by this translator identity through the
immutable `SemanticPreflightHandlerRegistry`. A family integration may add one
handler record for its declared translator; duplicate IDs are rejected. The
composition-level dispatcher only
checks graph eligibility, tier-compatible planner availability, registry
identity, and handler presence; it contains no family-name routing. A handler
continues to own its source-specific chronology, hover, route-geometry, or
authority checks. An unknown declared translator, a missing handler, or a
handler that reports a different identity is a structured blocked result.

Every source-owned execution now writes the common
`taoryx.mission-graph-execution/v1alpha1` artifact. Its
`observation_status` is `observed` when an executor emitted committed node
outcomes and selected declared edges, or `unobserved` when the runner only
supplied truth telemetry and independent objective evaluation. The Hummingbird
translator currently emits observed dispatches. The language-backed fixed-wing
batch runner currently emits an unobserved record, so even a passing
route/terminal truth evaluation cannot be misread as evidence that a
controller sequenced the graph. `completed_nominal_success_path: null` means
that the execution claim is absent; it is not a failed mission objective.
`taoryx vehicle results <directory>` verifies a present graph artifact against
the colocated compiled composition and exposes this evidence separately from
the independent objective outcome. A malformed or cross-composition artifact
fails result indexing; a missing historical/external artifact remains visibly
missing rather than being reconstructed from trajectory geometry.

The Hummingbird pseudo-6DOF translator is the first deliberately narrow
exception. It accepts a caller-authored `timeout_transition` from any airborne
segment directly to the declared final `touchdown_settle_disarm` segment, using
only `previous_terminal_truth_state`. It rejects abort, resource, envelope,
alternate-handoff, and arbitrary-target edges. The runtime records the
controller outcome, selected graph edge, state-transfer rule, and executed
instances. A timeout continues through the safe landing path, but every
skipped required objective remains independently failed, so the mission cannot
be reported as a nominal pass. This is recovery semantics, not
timeout-as-success. A future translator must make an equally explicit opt-in
and record each outcome transition in committed truth/event artifacts.

The shared `composition_graph_runtime` does not broaden that exception. It
calls a family-owned segment executor at each committed truth boundary and
selects only compiled graph edges. Its sole automatic transfer is
`previous_terminal_truth_state`; an edge marked
`declared_physical_transition` fails closed unless the owning family supplies
the physical transfer function. This preserves reusable dispatch and artifact
behavior without allowing generic composition code to invent separation,
contact, staging, or resource handoffs.

Every observed dispatch also records `committed_time_s`. The common runtime and
result-catalog validator reject non-finite, negative, or time-reversing
records, so a retained packet cannot make a transition appear before the truth
state that justified it.

The checked-in example
`examples/vehicle_composition/x8_racetrack_authored_linear_graph_3dof_compose.yaml`
shows the only caller-authored graph form that current fixed-wing translators
execute: one ordered success transition per segment, ending at the terminal
node, with `previous_terminal_truth_state` handoff implied at every edge.
It is intentionally not an example of abort or fallback execution. The
separate
`examples/vehicle_composition/hummingbird_timeout_recovery_graph_pseudo6dof_compose.yaml`
shows the sole current non-success path: a translation timeout routes to the
declared final landing segment, not to a successful terminal state.
The execution witness deliberately makes that translation unreachable and
verifies the observed `timeout` dispatch, committed-state handoff, skipped
nodes, and failed independent mission result. It is evidence of recovery
semantics, not of a successful fallback mission.

`verification/vehicle_execution_witnesses.yaml` records this separately as a
`graph_extension_witness`, rather than duplicating the ordinary one-witness-
per-endpoint matrix. The static gate proves the authored graph, exact family
extension declaration, capability preflight, and runtime lowering agree. Its
optional public batch smoke additionally requires `observed` dispatch evidence
from `mission_graph_execution.json`. The nominal graph witness follows the
success chain; the separate forced-timeout regression confirms the alternate
landing edge and failed mission result. Neither artifact lets timeout recovery
be reported as objective completion.

## Validate authoring witnesses as a catalog

### Generate an authoring kit without hidden numerical defaults

For a declared vehicle, mission, and fidelity, export the exact composition
shape that an author must fill:

```bash
taoryx vehicle authoring-template skywalker_x8 \
  powered_fixed_wing_racetrack_v1 point_mass_3dof
```

The result is a versioned `taoryx.vehicle-composition-authoring-kit/v1alpha1`
artifact. It lists initialization choices, ordered segment occurrences,
required inputs, canonical units, value spaces, control intents, graph shape,
available variant bindings, the source-owned variant-admission state, selected
interface, and runtime/evidence worklist.
For a development or planned fidelity, that worklist also projects the exact
`fidelity_promotion_blockers` declared by the family manifest before generic
endpoint work. This keeps a contributor from seeing only “add a factory” when
the actual first gate is, for example, source trim acceptance or a nonlinear
direct-wrench mission.
It also emits the exact selected execution endpoints (factory ID, claim
boundary, and blockers) plus only the commands the tier can run. For example,
`episode-info` appears only for a runnable episode endpoint, while
`batch-episode-parity` appears only when the exact family/mission/fidelity has
a registered parity witness.
An absent default remains `authoring_value: null`. When a family has declared
a source-backed recommendation, the kit exposes it as
`authoring_value_source: declared_recommendation`, but the compiler still
never inserts it. A required recommendation has the rule
`confirm_declared_default_explicitly`: an author must deliberately repeat it
in the request, just as they would a newly supplied value. The accompanying
`input_completion` record lists the values that must be supplied or confirmed
for each selectable initialization and segment occurrence. The kit is still
not an executable request and does not invent route geometry, trim values, or
source data. Repeated segments publish a deterministic
`suggested_instance_id`, preventing a later translator from losing semantic
turn or event identity.

Use the fixture validator during family authoring to distinguish a valid
composition request from one whose selected native mission has actually
declared a translation-ready preflight:

```bash
taoryx vehicle validate-examples
taoryx vehicle validate-examples --require-preflight
```

The default report compiles every YAML/JSON request in
`examples/vehicle_composition`, validates its selected public interface, and
records the lowering/preflight disposition. `--require-preflight` returns a
nonzero result for any fixture that is valid composition syntax but lacks a
translation-ready native mission. Neither mode executes a vehicle or promotes
the fixture to a qualified result.

## Bind the native adapter

The second explicit boundary binds a compiled scenario to the adapter declared
by the selected family and fidelity:

```bash
taoryx vehicle lower generated/x8-racetrack-semantic.json
```

There is no physical-family fallback. If the selected adapter factory has not
been moved into the runtime registry, the command returns `blocked` with that
specific integration gap. If it can construct the adapter, it returns
`adapter_bound`, the adapter state/control schema, and its operation-level
capabilities. `adapter_bound` still is not a mission run: the remaining
adapter-specific step is a semantic-segment translator that maps the compiled
segments to trim, controller, and runtime requests.

Some established source witnesses run through a source-owned batch factory
before they implement the generic `StandardFamilyAdapter` protocol. For an
exact composition whose semantic translator has preflighted and whose exact
batch factory is declared runnable, `taoryx vehicle lower` returns
`factory_bound`. That record includes the factory declaration, but no adapter
descriptor: it must not be confused with generic adapter conformance or a
substitute family plant. Both `adapter_bound` and `factory_bound` remain below
execution, evaluation, and qualification.

`taoryx vehicle validate-examples --require-preflight` applies this same
horizontal rule: a strict example must have a translation-ready semantic
mission *and* lower to either `adapter_bound` or `factory_bound`. The report
includes the exact factory declaration when the latter applies. This is still
an authoring/preflight check, not a simulation run.

## Preflight a native mission translation

Before a composition can run, a family translator must prove that the chosen
semantic values have an unambiguous mapping to its native mission geometry:

```bash
taoryx vehicle preflight generated/x8-racetrack-semantic.json
```

The initial vertical slices are the capability-scaled X8, B747, A320, and
F-16 racetracks. Their preflight independently derives the selected speed,
bank, turn radius, straight length, altitude gates, NED gate centers, and
start/finish heading from `verification/powered_fixed_wing_mission_profiles.yaml`.
A composition is `translation_ready` only when each input matches that derived
route. A nearby or legacy hand-authored route returns `blocked`; the runtime
may not silently substitute a different geometry.

The legacy [x8_racetrack_compose.yaml](../../examples/vehicle_composition/x8_racetrack_compose.yaml)
remains a deliberately loose composition example and correctly fails this
preflight. Use [x8_racetrack_capability_compose.yaml](../../examples/vehicle_composition/x8_racetrack_capability_compose.yaml)
when a capability-derived native route handoff is required.

`translation_ready` is an independent semantic gate: it says only that the
composition has a route representation for the selected native translator.
It may combine with a generic `adapter_bound` result or with a source-owned
`factory_bound` result, but it does not itself construct either runtime.
Trim, controller execution, independent truth evaluation, and qualification
remain subsequent gates. Families without a registered translator return
`not_applicable`, not an optimistic success.

## Materialize exact native inputs

The same translation is reusable by the language-backed execution path:

```bash
# Point-mass 3DOF
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml \
  --output generated/x8-racetrack-3dof-semantic.json

# Named pseudo-6DOF attitude-response bridge
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_capability_compose.yaml \
  --output generated/x8-racetrack-pseudo6dof-semantic.json

taoryx vehicle materialize generated/x8-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/x8-racetrack-native-inputs
```

Use the analogous 3DOF compiled artifact with `materialize` to produce its
point-mass route inputs.

The exact interface is selected by the compiled composition rather than only
by a family/fidelity lookup. This matters when an interactive-only composition
declares a sampled observation profile:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml \
  --output generated/x8-racetrack-sensor-episode.json
taoryx vehicle interface-composition generated/x8-racetrack-sensor-episode.json
```

The command prints the fingerprinted parameter/action/status/resource and
observation contract before an episode opens, including sensor cadence and
latency. The X8 and Hummingbird sensor fixtures also have aligned batch
translators: their `sensor_observations.json` artifact replays the declared
sensor only at committed truth rows and fails if a required capture or release
boundary is absent. That artifact is observation-timing evidence, not a
sensor-noise, estimator, or batch-showcase qualification claim.

For an exact compiled selection, `episode-info` opens only its declared
source-owned episode factory, resets it, and prints the selected action and
observation schemas plus the initial committed-truth, normalized status, and
selected observation frames. It is a safe discovery/open check rather than a
simulation fallback or an implicit trajectory run:

```bash
taoryx vehicle episode-info generated/x8-racetrack-sensor-episode.json --seed 7
```

If the selected family/mission/fidelity has no runnable episode binding, the
command fails with that declared gap; it never substitutes another family or
lower-fidelity model.

## Read a normalized result

Every current mission-style batch factory writes `evaluation.json` beside its
family-owned telemetry and objective evidence. The normalized record keeps
validity, qualification, feasibility, outcome, objective metrics, and hard
gates distinct. Read it without knowing the family-specific runner with:

```bash
taoryx vehicle result generated/x8-racetrack-run \
  --composition generated/x8-racetrack-sensor-episode.json
```

Supplying `--composition` is the strong form: the reader verifies both the
scenario identifier and immutable composition fingerprint. A missing,
malformed, or mismatched evaluation is an error. `result` does not infer an
evaluation for direct-wrench screens or other artifacts that deliberately lack
a mission-style evaluation envelope.

To discover a directory of existing mission-style results without re-running
them, use:

```bash
taoryx vehicle results artifacts/composition
```

The index validates each `evaluation.json`, retains malformed files as visible
errors, and exposes the recorded scenario identity, outcome, qualification,
gate statuses, and available sidecar artifacts. When a run supplies its
colocated `composition.json`, the index also parses it and requires exact
scenario ID and immutable-fingerprint agreement; a mismatch is an invalid
result, not merely a warning. A missing sidecar remains explicitly marked
`composition_provenance: missing` for external/provider results. The index is
an artifact catalog, not a cross-family score or qualification promotion.

The materialization command is currently registered for the X8 and B747 powered-fixed-wing
racetrack slices. It refuses an unready composition and writes three
disposable inputs: the native problem with exact route attributes, a retimed
independent-truth mission catalog, and a route catalog containing the same
resolved binding. It never edits a checked-in baseline `.prb`, mission
catalog, or template. Execution and qualification are deliberately separate,
later commands.

`verification/vehicle_execution_witnesses.yaml` is the companion onboarding
matrix. Every runnable exact family/mission/fidelity/operation binding must
have one checked-in composition request. The validation gate compiles each
request, requires translation-ready preflight, lowers it to either its generic
adapter or declared source-owned factory, resolves the exact endpoint factory,
and opens interactive endpoints once. It also requires one checked-in composed
witness for every advertised runtime-bound variant and records its exact native
input trace plus retrim/requalification invalidations. It does not substitute
for a batch mission run or qualification evidence. Use
`taoryx vehicle witness-report --execute-batch` for the slower public
compose-to-run smoke of every batch witness. Add `--family <id>` or
`--witness <id>` to keep ordinary edits focused on one exact vehicle or
endpoint. The
source-table fixed-wing factories use a declared eight-row translation smoke;
their full transport-sized racetracks remain separate nominal-mission runs.
Every batch smoke also requires and validates `status_trace.json` beside
`execution.json` and `vehicle_interface.json`. The validator checks the exact
interface ID and fingerprint, declared batch-visible channel set, committed
time ordering, and per-row completeness. It proves that the selected
interface's status/resource channels project from committed truth rather than
being present only as static catalog metadata.

When an endpoint publishes `semantic_action_trace.json`, its held-command
intervals are validated *against that same* `status_trace.json`: both an
interval start and its committed end must name actual committed truth
boundaries. An action trace may be sparser than the status trace while a
command is held, but it cannot invent a timestamp between integration commits.
This is the portable timing rule used by sensor, AI/RL, and batch consumers;
they may derive values only from explicitly declared committed samples, never
by interpolating a control or truth value across a fixed step.

`verification/vehicle_execution_parity_witnesses.yaml` is the narrower
companion for pairs already marked `registered` in
`verification/vehicle_execution_parity.yaml`. It supplies one exact
composition and short semantic action sequence for each registered pair. The
parity gate replays that trace through the pair's named adapter, verifies its
returned adapter identity, and rejects missing, duplicate, or excess witness
rows. Run it alongside the endpoint audit with:

```bash
python tools/validate_vehicle_execution_witnesses.py --execute-parity
```

This is exact committed-boundary batch/episode parity only. It does not extend
parity to unregistered pairs or establish mission, allocation, robustness, or
qualification evidence.

For the fast runtime-variant-only audit, run:

```bash
python tools/validate_vehicle_execution_witnesses.py --variants-only
```

It validates every advertised runnable variant’s exact composed request,
preflight, and runtime handoff without opening an interactive endpoint or
running a vehicle.

## Execute an immutable nominal-run artifact

`vehicle run` is the public execution step for the current airbreather and
source-replay slices. X8 and B747 re-run translation preflight, materialize native inputs in
a temporary workspace, then execute the normal language-backed runtime. A320
and F-16 bind the same resolved racetrack directly to their declared OpenAP or
source-reduced execution adapter. Each path writes the compiled composition,
preflight, runtime report, truth telemetry, independent objective report,
envelope report, a normalized Mission Composition `evaluation.json`, `status_trace.json`,
and stable `execution.json`; the reduced paths additionally write their trim
and model-provenance artifacts. `evaluation.json` preserves an explicitly
`unqualified` nominal outcome unless the selected family owns stronger
evidence; it is a common result projection, not a new family evaluator:

```bash
taoryx vehicle run generated/x8-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/x8-racetrack-pseudo6dof-execution
```

The command refuses a non-`translation_ready` composition and never selects a
generic substitute vehicle or a different route. A zero exit status is only a
nominal truth-objective and declared-envelope pass. It does **not** imply
controller physical-effector realization, timestep convergence, accepted-step
replay parity, robustness, or family qualification; those additional gates
remain explicit evidence consumers of the same execution inputs.

For example, the B747 request uses the identical segment vocabulary but its
capability profile derives a transport-scaled geometry:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml \
  --output generated/b747-racetrack-pseudo6dof-semantic.json

taoryx vehicle preflight generated/b747-racetrack-pseudo6dof-semantic.json
taoryx vehicle materialize generated/b747-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/b747-racetrack-native-inputs
taoryx vehicle run generated/b747-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/b747-racetrack-pseudo6dof-execution
```

The B747 pseudo-6DOF input uses a named route-lag attitude response sidecar.
It exposes commanded/achieved bank, pitch, heading, and body-rate response;
it does not establish a physical B747 surface allocation or moment balance.

The same user flow runs the OpenAP A320 and source-local F-16 reduced
profiles. There is no fallback from one aircraft to another, and the selected
semantic speed is not silently replaced by a model default:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml \
  --output generated/a320-racetrack-pseudo6dof-semantic.json
taoryx vehicle run generated/a320-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/a320-racetrack-pseudo6dof-execution

taoryx vehicle compose \
  examples/vehicle_composition/f16_racetrack_capability_3dof_compose.yaml \
  --output generated/f16-racetrack-3dof-semantic.json
taoryx vehicle run generated/f16-racetrack-3dof-semantic.json \
  --output-dir generated/f16-racetrack-3dof-execution
```

At present these adapters support only their declared point-mass 3DOF and
named pseudo-6DOF profiles. The A320 pseudo profile includes a response-law
and policy-surface diagnostics; the F-16 pseudo profile is a source-local
attitude/rate bridge. Neither result is a direct-wrench, physical-surface,
actuator, or full-controller-equivalence claim.

## Open a composition episode

The composition layer also has a narrow interactive projection. It does not
introduce a second numerical kernel: the language-backed airbreathers reuse
`InteractiveSession`, and the Hummingbird pseudo witness calls its existing
bounded aggregate-thrust model at fixed accepted substeps.

```python
from pathlib import Path

from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        Path("examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
)
episode = open_vehicle_composition_episode(composition)
observation = episode.reset()
transition = episode.step({"throttle": 0.60}, duration_s=0.10)
episode.save_checkpoint("generated/x8-racetrack.checkpoint.json")
episode.close()
```

Every episode reports its action and observation schema, including an explicit
`value_space` for every native channel. This preserves the distinction between
values such as a wrapped yaw angle, its linear yaw rate, a Cartesian wrench,
and a discrete contact flag even while legacy adapters use short native names.
Every episode exposes only the selected fidelity's declared controls, and
returns truth committed at the end of the requested external duration. An adaptive integrator may take smaller
inner steps, but it may not silently shorten `step(..., duration_s)`; it
continues to the requested boundary or a declared terminal boundary. A
checkpoint is bound to the immutable composition fingerprint and restores
into a newly built executable callback graph.

The semantic interface is authoritative. Native episode action names are
only the declared binding targets for that interface; a caller that needs
portable controls should use `ActionFrame` and the selected authority profile.
At episode-open conformance, Taoryx verifies that every available semantic
action maps one-to-one to an exposed native action with compatible units and
bounds, and that canonical status and every available observation profile
carry the chosen interface fingerprint at the current committed boundary.
The raw `observation_schema` is either a complete `native_truth` diagnostic
schema (the source-table and local-screen adapters) or a direct
`semantic_contract` projection (the newer reduced steppers). This distinction
is explicit in execution-witness evidence; it never permits an undeclared
native control or an untyped raw observation to become a public API.

The `vehicle authoring` worklist also exposes a mission capability planner per
mission/fidelity. The shared powered-fixed-wing racetrack derives feasible
turn radius, straight length, climb/descent timing, and mission horizon from
that family’s declared profile. Hummingbird’s named
aggregate-thrust pseudo-6DOF mission has a separate hover/translation planner
that checks thrust reserve and reports conservative vertical/lateral authority.
A missing planner is reported as an explicit onboarding/preflight gap; Taoryx
does not reuse fixed-wing geometry for a hover, rocket, lifting-body, or
passive-body mission.

The Hummingbird `grounded_idle.mass_kg` reset input is a narrower example of
runtime traceability: the declared value is passed into the aggregate-thrust
pseudo-6DOF batch and episode plants, and appears in the translated mission
manifest and runtime provenance. It changes thrust demand and translation
response under the named surrogate; it is not a qualified payload envelope or
individual-rotor mass/inertia model.

For a deterministic policy or RL-style action stream, retain the public
`CompositionPolicyTrace` with the compiled composition rather than a native
control history. `write_composition_policy_trace` persists that trace, and the
public command reopens the exact composition and compares every newly produced
public committed-boundary frame against the saved artifact:

```bash
taoryx vehicle replay-policy \
  generated/x8-racetrack-semantic.json \
  generated/x8-policy-trace.json \
  --output generated/x8-policy-replay.json
```

The replay rejects a changed composition fingerprint, interface fingerprint,
semantic action mapping, accepted-boundary result, or final public frame. It
is a same-episode-kernel replay check, not evidence that a separately compiled
batch mission, a physical effector model, or a robustness suite agrees with
the trace.

The `batch_episode_parity` field in every vehicle descriptor is a separate
capability declaration, not a derivation from the endpoint matrix. It records
`registered`, `not_registered`, or `not_available` for every declared mission
and fidelity: two runnable operations remain `not_registered` until an exact
witness has been registered. Exact comparisons are currently available for the
Hummingbird named aggregate-thrust pseudo-6DOF witness, the X8/B747
language-backed point-mass and route-lag pseudo-6DOF witnesses, and the A320
OpenAP and F-16 source-reduced point-mass and pseudo-6DOF reduced-stepper witnesses. Each
takes the same persisted policy trace,
runs every public semantic action through its separately owned source-runtime
batch loop, and compares the projected committed status after every action
boundary:

```bash
taoryx vehicle batch-episode-parity \
  generated/hummingbird-hover-yaw-sensor.json \
  generated/hummingbird-policy-trace.json \
  --output generated/hummingbird-batch-episode-parity.json

taoryx vehicle batch-episode-parity \
  generated/x8-racetrack-3dof.json \
  generated/x8-policy-trace.json \
  --output generated/x8-batch-episode-parity.json

taoryx vehicle batch-episode-parity \
  generated/a320-racetrack-pseudo6dof.json \
  generated/a320-policy-trace.json \
  --output generated/a320-batch-episode-parity.json

taoryx vehicle batch-episode-parity \
  generated/f16-racetrack-pseudo6dof.json \
  generated/f16-policy-trace.json \
  --output generated/f16-batch-episode-parity.json
```

The report records its exact adapter, batch factory, authority profile,
integration substep, composition identity, and interface fingerprint. It
rejects all unregistered family/fidelity/mission/factory combinations; it is
not a generic batch fallback. This is a semantic action-trace kernel witness
only, not a completed mission, physical rotor allocation, robustness, or
qualification result.

Normalized `evaluation.json` consumes the same committed `status_trace.json`
when a batch run provides it. It carries final scalar/boolean resource values
with their declared units and committed time, while command and effector
evidence is explicitly unavailable unless a dedicated run-level trace exists.
This preserves a common consumer hook without treating native controller
internals or a clean path as evidence of realized control authority.

When a source-owned runner can provide every declared action or effector value,
it additionally writes `semantic_action_trace.json`. Each sample identifies
the held action interval and the committed truth boundary at its end; it never
interpolates command history. The trace is fingerprint-bound to the exact
composition and interface, and `taoryx vehicle results` rejects an unrelated
trace. Its presence exposes only the recorded semantic actions and declared
effectors. In particular, the Hummingbird aggregate-thrust pseudo-6DOF trace
shows requested roll/pitch/yaw/thrust/enable values, but does not invent
individual motor, rotor, or physical-allocation evidence.

The A320 and F-16 reduced racetrack paths use the same artifact for held
speed, flight-path, heading, and bank guidance. Heading is projected into the
interface's declared periodic (0\leq\psi<360^\circ) chart, so a native
wrapped value such as (-0.1^\circ) is reported as (359.9^\circ) without
altering the modeled trajectory. Bank, flight-path, and rates remain linear
coordinates and are never wrapped by this projection.

The initial episode witnesses are X8/B747 language-backed racetracks, A320 and
F-16 source-owned reduced fixed-wing paths, and the Hummingbird named
pseudo-6DOF hover/yaw composition:

```python
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        Path("examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )
)
episode = open_vehicle_composition_episode(composition, seed=7)
transition = episode.step(
    {"roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 1.57, "thrust_ratio": 0.10},
    duration_s=0.10,
)
```

The Hummingbird episode is explicitly an aggregate-thrust-vector response
law: it does not claim individual rotor allocation. The A320 and F-16 episodes
are explicitly reduced kinematic-guidance paths: their source controls are
diagnostic observables, not allocated physical surfaces. Other families still
return an explicit unavailable-adapter error rather than borrowing X8 or
Hummingbird runtime behavior.

The same Hummingbird pseudo-6DOF composition also has a source-owned nominal
batch path. It translates the declared hover, yaw, translation, contact, and
post-shutdown settle segments into the existing aggregate-thrust response
model, then writes the standard composition, preflight, plan, truth,
objective, envelope, and transition artifacts:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml \
  --output generated/hummingbird-pseudo-semantic.json
taoryx vehicle run generated/hummingbird-pseudo-semantic.json \
  --output-dir generated/hummingbird-pseudo-execution
```

This is an executable nominal pseudo-6DOF witness, not a rotor-resolved
showcase or a promotion to individual-motor allocation.

The NESC two-stage rocket has an equally explicit but different batch binding:
the retained source translation is replayed only when the composed launch,
staging delay, and terminal kind match the pinned witness.  The pseudo-6DOF
case adds its named scheduled attitude response; neither path turns source
history into an active guidance, thrust-vector, or stage-separation model.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml \
  --output generated/nesc-source-replay-semantic.json
taoryx vehicle preflight generated/nesc-source-replay-semantic.json
taoryx vehicle run generated/nesc-source-replay-semantic.json \
  --output-dir generated/nesc-source-replay-execution
```

The resulting artifact contains the source-replay provenance, stage event
times, independently evaluated stage/cutoff/terminal objectives, and data-
integrity envelope. It is not a substitute for a participating rocket runtime
or a future vehicle-control episode.

An optional composition-level binding can independently propagate a synthetic
passive cylinder from the accepted stage-separation state. The binding names
the `taoryx.passive-bodies` plug-in, its exact runtime, the requested child
fidelity, and the ECI-to-local-tangent state transfer; it is not enabled by the
ordinary NESC replay example and does not claim a source-exact separated stage.
Use the explicit witness when that separate child is intended:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/nesc_staged_source_replay_with_passive_child_pseudo6dof_compose.yaml \
  --output generated/nesc-passive-child-semantic.json
taoryx vehicle run generated/nesc-passive-child-semantic.json \
  --output-dir generated/nesc-passive-child-execution
```

The X-15 registry deliberately exposes two different mission contracts. The
existing `rocket_aircraft_high_energy_v1` remains the planned local
direct-wrench X-15 bridge. The runnable
`x15_staged_booster_reachability_v1` is a different, local-frame, source-pinned
reduced witness at 3DOF and pseudo-6DOF:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml \
  --output generated/x15-staged-reachability-semantic.json
taoryx vehicle preflight generated/x15-staged-reachability-semantic.json
taoryx vehicle run generated/x15-staged-reachability-semantic.json \
  --output-dir generated/x15-staged-reachability-execution
```

The latter pins the retained X-15 source staging mass, speed magnitude,
cutoff, and release timing; it then evaluates booster deployment, the
high-energy corridor, a declared open-loop atmospheric handoff witness, and
passive impact. Its `open_loop` or `response_law` control realization is
intentional: it exposes no external action or physical-effector authority.
It is not a native X-15 rigid-body mission, a controlled terminal handoff, or
the separate synthetic California-to-Hawaii showcase.

The X-15 also exposes a deliberately narrower rigid-body direct-wrench
endpoint, `x15_local_direct_wrench_screen_v1`. It executes the retained local
source-plant LQR recovery screen from one source release/glide point. The
artifact records the local state and requested-versus-achieved force/moment
truth, residual, and saturation disposition at committed rows:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml \
  --output generated/x15-local-direct-wrench-screen-semantic.json
taoryx vehicle preflight generated/x15-local-direct-wrench-screen-semantic.json
taoryx vehicle run generated/x15-local-direct-wrench-screen-semantic.json \
  --output-dir generated/x15-local-direct-wrench-screen-execution
```

Its passing result is `screen_pass`, not `mission_pass`. It establishes only
that the bounded local source-linearized screen recovers its declared local
perturbation. It has a narrowly registered `direct_wrench` batch/episode
parity witness that compares caller-owned total-wrench commands at committed
truth boundaries. The reusable local-screen definition is available only for
explicitly registered source points; it does not make direct-wrench control a
generic fallback for other vehicles or missions. Neither path establishes
trim, carrier release, ignition, propulsion, navigation, handoff, physical
effectors, or X-15 family qualification.

The authoring catalog exposes this screen through
`taoryx.x15_local_direct_wrench_screen.capability.v1`, which records its exact
source-local state, finite six-axis authority and slew limits, and cadence.
That capability record confirms a valid pinned screen contract; it does not
predict route feasibility or turn a local `screen_pass` into mission evidence.
When its immutable composition also passes semantic preflight, runtime lowering
reports both the constructed X-15 source adapter and the exact
`local_direct_wrench_screen.v1` batch factory. This is execution wiring, not a
promotion beyond the local screen boundary.

The distinct `x15_source_surface_authority_screen_v1` Composition endpoint
adds the retained source-table controls without widening the direct-wrench
claim. It freezes the release/glide source fixture, obtains a three-axis local
moment effectiveness matrix by centered source-table differences, allocates
the requested moment through the symmetric stabilator, differential
stabilator, and rudder, and then re-evaluates the nonlinear six-axis source
loads at the achieved positions:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_source_surface_authority_screen_compose.yaml \
  --output generated/x15-source-surface-authority-semantic.json
taoryx vehicle preflight generated/x15-source-surface-authority-semantic.json
taoryx vehicle run generated/x15-source-surface-authority-semantic.json \
  --output-dir generated/x15-source-surface-authority-execution
```

The standard status/action artifacts therefore contain actual named source
surface positions, requested/achieved/residual body moments, rank, and the
fixed mass/source condition. The screen is batch-only and explicitly leaves
full-state trim, feedback, state propagation, propulsion/RCS allocation,
guidance, the energy-managed mission, and flight qualification unavailable.

HL-20 uses the same reusable local-screen definition for the separately pinned
Mach-0.5 source point:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hl20_local_direct_wrench_screen_compose.yaml \
  --output generated/hl20-local-direct-wrench-screen-semantic.json
taoryx vehicle preflight generated/hl20-local-direct-wrench-screen-semantic.json
taoryx vehicle run generated/hl20-local-direct-wrench-screen-semantic.json \
  --output-dir generated/hl20-local-direct-wrench-screen-execution
```

Its six-axis force/moment requests remain an explicit direct-wrench bridge;
they are not HL-20 body-flap, elevon, or other physical-surface commands. The
subsonic local screen does not clear the separately blocked Mach-2 balancing
condition and does not qualify release, glide navigation, high-energy flight,
or the HL-20 family. Its separately registered parity witness compares a
caller-owned wrench trace across its batch and episode paths at committed truth
boundaries; that is interface consistency evidence only, not a new flight or
allocation claim.

`taoryx vehicle results <directory>` indexes the screen as an explicitly
separate `local_controller_screen` record only when the colocated composition
verifies it as the same direct-wrench screen. Its `local_screen_pass` result
must not be interpreted as mission completion, a terminal result, physical
effector allocation, or family qualification.
`taoryx vehicle result <run-directory>` returns the same record for one such
run; it deliberately does not fabricate the normal mission-evaluation fields.

## Intake before composition authoring

`taoryx vehicle authoring-template` is for a declared catalog vehicle. Before
that vehicle exists, start with the same discoverable Mission Composition surface:

```bash
taoryx vehicle intake existing-family \
  --family-id example_uav \
  --physical-family powered_fixed_wing \
  --strategy-id powered_fixed_wing.v1 \
  --mission-overlay fixed_wing_racetrack \
  --adapter-id taoryx.fixed_wing.example_uav.v1
```

It selects an explicit integration strategy and exports all four canonical
fidelity tiers, minimum data/operations, calibration stages, non-tunable
blockers, and the compatible semantic mission contract. For a topology that
does not fit an existing strategy, use `taoryx vehicle intake new-topology`.
That path intentionally emits only a strategy-definition scaffold, so a new
physical family cannot enter composition through a copied controller or
invented force model.

The shared powered-fixed-wing racetrack intake goes one step further than a
prose worklist. It exports unfilled, validated `ParameterSpec` descriptors for
the source-owned capability profile and reusable mission intent. An
operating-point identifier is an explicit finite set; requested bank is a
bounded interval; speed, altitude, length, and dwell are positive magnitudes.
Each record is marked `source_value_status: authoring_required`, with no
invented default, qualified range, provenance, or runtime binding. An
integration author can therefore carry the same typed records into a family
registry entry rather than rebuilding units and topology from identifier
spelling.

Both paths also emit `interface_contract_required`: the author must define
parameter scopes and mathematical value spaces, semantic controls and their
effector/bridge boundary, canonical status/resources/observations, committed
truth and sensor timing, and the selected batch/episode/replay disposition.
An intake is therefore incomplete until it can explain not only the plant
data it needs, but also how a caller can configure, command, and observe the
vehicle without touching native state-vector layout.

## Source-family integration gates

Intake describes the work required to add a family; it does not show which
source-backed gates are already complete. The same `taoryx vehicle` surface
therefore exposes the provider-neutral integration records for the reference
families:

```bash
# Source manifest and four-tier metadata only; no plant execution.
taoryx vehicle integration readiness reference_f16_s119

# Staged source, trim, effectivity, controller, and mission evidence.
taoryx vehicle integration pipeline reference_f16_s119

# Inspect a semantic mission that is translation-ready while its runtime stays planned.
taoryx vehicle integration pipeline reference_hl20_mod_k
```

`readiness` reports only whether source manifests and four-tier profile
metadata are sufficient to begin work. `pipeline` joins recorded source
evidence into explicit stage gates and preserves every blocker or planned
runtime gate. A family may use a legacy racetrack binding or a generic
`semantic_composition` binding: the latter pins each composition witness,
public composition-family identity, phase order, and expected semantic
preflight status. Neither command runs a mission, proves physical effector
allocation, or promotes a vehicle or fidelity. `pipeline` returns a nonzero
exit status when blockers exist unless `--allow-blocked` is selected; that flag
changes only shell disposition, never evidence or promotion status.
With `--packet-dir`, it can also write a hash-bound integration packet for
handoff without rerunning the source plant.

The generated intake supplies the same choice before a source family exists:
`mission_binding_contract_required` supports the established fixed-wing
racetrack shape and the generic `semantic_composition` shape. The latter must
name both source and public composition identities, one mission, its exact
phase order, and checked-in per-fidelity witnesses whose compiled semantic
preflight status is explicit. A translation-ready witness is still not a
runtime, objective result, or fidelity promotion.

For a genuinely new topology, the intake additionally emits three
non-promotable authoring records: `source_manifest_template`, a four-tier
`synthetic_conformance_witness_plan`, and `new_family_decision_record`. The
decision record must document why an existing strategy does not fit, the
topology/resource/start/terminal/control/timing/objective/evidence contracts,
and the exact nonclaim boundary before the family can be admitted to the
registry. These records intentionally contain no guessed source facts,
runtime adapter, controller, or qualification result.

The passive tumbling family has a separate direct-release path. It intentionally
has no controller or allocation profile: 3DOF uses a declared orientation-
averaged projected area, and pseudo-6DOF explicitly reuses the native
rigid-body passive-tumble equations. Mission Composition publishes named
`cylinder`, `sphere`, `cone`, and `triaxial_ellipsoid` realizations. The selected
realization must match the typed `body_shape` parameter, and every named shape
executes through the same direct-release factory for both available dynamics
fidelities. Other geometry or release changes remain explicit bounded variants,
not silent model morphs.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/tumbling_body_direct_release_3dof_compose.yaml \
  --output generated/tumbling-3dof-semantic.json
taoryx vehicle run generated/tumbling-3dof-semantic.json \
  --output-dir generated/tumbling-3dof-execution

taoryx vehicle compose \
  examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml \
  --output generated/tumbling-pseudo-semantic.json
taoryx vehicle run generated/tumbling-pseudo-semantic.json \
  --output-dir generated/tumbling-pseudo-execution
```

The 3DOF artifact must not be used as a tumble proof. The pseudo artifact can
prove only the declared native-rigid passive rotation of this fixture; it does
not provide a pseudo response law, a wrench command, an effector allocation,
or a source-specific spent-stage claim.

## Composition model

A trajectory has one initialization contract and an ordered segment graph:

```text
vehicle + fidelity
    ↓
initialization contract and parameters
    ↓
mission template
    ↓
ordered segment instances with target parameters
    ↓
compiled scenario and independent evaluation
```

An initialization contract describes the one-time state setup, such as
`airborne_trim`, `grounded_idle`, `air_launch_release`, `high_altitude_release`,
or `pad_launch`. It declares the required fields and their canonical units.

A segment contract describes semantic intent rather than an implementation
shortcut. It declares compatible fidelities, user parameters, requested control
intents, and permitted physical transition events. Examples include
`climb_level_gate`, `fly_by_turn`, `hover_dwell`, `stage_separation`, and
`atmospheric_handoff`.

State is continuous by default across segments: position, velocity, attitude,
rates, mass, and resources carry forward. Only a named physical transition
event—such as release, stage separation, contact, or motor shutdown—may alter
that rule, and it must be recorded in the run artifact.

## Authority boundaries

The composition registry is a projection of existing authorities:

| Concern | Authority |
| --- | --- |
| Family identity, physical family, adapter, four fidelity slots, blockers | `verification/horizontal_fidelity_registry.yaml` |
| Profile control realization and evidence boundary | `verification/pseudo6dof_profiles.yaml` |
| Native source data and local plant details | `verification/vehicle_models.yaml` or `families/*/family.yaml` |
| Existing qualification scenario details and truth objectives | `verification/family_qualification_missions.yaml` |
| Reusable powered-fixed-wing geometry | `verification/racetrack_templates.yaml` |
| User-visible initialization, segments, and mission recipes | `verification/vehicle_composition_registry.yaml` |

The registry may advertise a **planned** or **development** mission template;
that is discoverability, not a promotion. Runtime selection must still apply
the existing fail-closed fidelity-lowering and qualification checks.
