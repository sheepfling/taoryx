# Product 3 maturity plan: vehicle and mission composition

**Status:** Active product-maturity plan

**Scope:** The user-facing layer that lets a person, script, optimizer, or RL
environment discover a supported vehicle, understand its evidence boundary,
configure a bounded variant and mission, compile it into an exact execution
binding, run it, and inspect an independently evaluated result.

**Companion documents:** [Three-product execution roadmap](three-product-execution-roadmap.md),
[three-product showcase guide](../THREE_PRODUCT_SHOWCASE.md), [vehicle
composition registry](../architecture/vehicle-composition-registry.md), and
[vehicle interface contract](../architecture/vehicle-interface-contract.md),
and the [living Product 3 coverage audit](product-three-coverage-audit.md).

## Product decision

Product 3 is not a gallery of model names or a collection of hand-authored
`.prb` files. Its public contract is:

```text
discover
  -> inspect capabilities and evidence
  -> select bounded vehicle configuration
  -> author semantic mission / trajectory intent
  -> compile and preflight
  -> bind the exact runtime or episode
  -> execute
  -> independently evaluate and reproduce
```

The product is mature when a caller does not need to discover family-specific
source files, controller names, raw state-vector indices, or hand-tuned route
geometry to use a supported configuration. Those details remain visible in
provenance and diagnostics; they are not required as hidden inputs.

The governing rule remains:

> Product 3 may simplify selection and composition, but it must never create a
> generic fallback plant, control path, resource model, or qualification claim.

## Current baseline

The current baseline is meaningful and should be retained:

- nine family records, 36 resolved family/fidelity interface contracts, and
  30 checked-in runnable batch or episode endpoints;
- `taoryx vehicle catalog`, `describe`, `list`, `inspect`, `schema`, `endpoints`, `interface`, `topology-report`,
  `compose`, `preflight`, `lower`, `materialize`, `run`, `episode-info`, `replay-policy`, and
  `batch-episode-parity`;
- immutable composition requests that validate vehicle, fidelity,
  initialization, exact ordered segment sequence, parameter names, and
  canonical units;
- source-owned execution factories and explicit `runnable`, `development`,
  and `planned` status; and
- committed-truth status/interface artifacts, independent objective reports,
  topology-aware objective errors, and no generic adapter fallback.

The catalog now has a separate, fail-closed topology audit:

```bash
taoryx vehicle topology-report
```

It covers registry-owned initialization, segment, and variant parameters;
every interface action/effector/status/resource/diagnostic channel; observation
bindings; and the common truth-objective input schema. A pending topology or a
missing observation-channel topology is a report failure. This proves only the
mathematical contract—not source fidelity, executable support, or qualification.
The public-facing operation rules and addition checklist are collected in the
[public value-space contract](../architecture/public-value-spaces.md): circular
heading/yaw values use wrapped error while their rates remain linear, and
discrete events/modes remain non-interpolable.
Initialization and segment parameters are now additionally resolved through
the versioned `verification/parameter_value_space_catalog.yaml` rather than
from their identifier or unit suffix. The catalog makes signed coordinates,
nonnegative magnitudes, periodic headings, finite choices, and quaternion
attitudes distinguishable to every caller. Registry loading fails closed when
an advertised input has no catalog entry; the topology report exposes this
catalog-coverage result separately. It corrects a real semantic error that a
unit-only heuristic cannot avoid: a north/east coordinate measured in metres
belongs to a signed Euclidean line, not a positive half-line.
The same fail-closed rule now applies to the resolved action, effector, status,
observation, resource, and diagnostic surface through
`verification/interface_channel_value_space_catalog.yaml`. It assigns every
current public interface ID to an explicit topology profile, including unit
dependent angle representations. The interface constructor retains an
ad-hoc primitive fallback only for local fixtures; a resolved catalog channel
using that fallback fails the topology report. This removes the remaining
identifier-based topology switch from the public Product 2/3 boundary.
The generic independent truth-objective vocabulary is likewise owned by
`verification/truth_objective_channel_value_space_catalog.yaml`. Its declared
targets now expose an explicit source and unit/topology profile; a new
family-owned target remains an unpromoted fallback until that family supplies
and reviews its semantic contract.
The same report now projects exact canonical and resource-channel coverage by
vehicle/fidelity, availability, provenance, and committed sampling semantics.
It is a discovery aid for stable Product 2/3 hooks, not an assertion that every
family has fuel, battery, or physical-effector telemetry.

The registry-owned input bridge is now a shared `ParameterSpec` rather than
only an ID/unit/required tuple. Every initialization and segment field exports
an explicit default disposition, hard/qualified/safe-extended intervals,
transform, conditional visibility, coupling group, projection policy,
derivation, invalidations, and provenance fields. Existing imported fields
remain deliberately sparse: absent evidence serializes as absent metadata and
`reject_invalid`, never as an inferred default, qualified interval, or silent
projection. This provides one compatible contract for future variant, reset,
segment, schedule, and optimizer inputs while preserving the existing YAML
registry.

Runtime binding now distinguishes two honest preparation results. An
`adapter_bound` composition constructed and passed the generic
`StandardFamilyAdapter` conformance gate. A `factory_bound` composition instead
has an exact semantic translator plus a declared source-owned batch factory,
but has not claimed generic plant-adapter conformance. This closes a discovery
contradiction for current source witnesses such as the X8: the exact
capability-derived composition can preflight and run through the
language-backed factory without being misleadingly reported as `blocked`.
Neither status means the vehicle was executed, evaluated, or qualified.

The local source plants used to exercise the physical X8, B747, Hummingbird,
and F-16 control seams are runtime-owned rather than tool-owned. The
developer evidence scripts import the same pinned builders that the Product 3
adapter registry binds, and all four pass the same operation probes at their
exact source operating point. This removes construction drift without
promoting a local trim witness into a runnable racetrack, gain-scheduled
controller, or envelope-wide qualification claim.

The execution-witness gate now enforces that relationship for every advertised
runnable endpoint: its checked-in composition must compile, preflight, lower
to `adapter_bound` or `factory_bound`, and resolve the same declared factory.
This is a catalog/onboarding invariant, not a substitute for the batch mission
smoke or family qualification.

The same gate now includes a separate variant-witness matrix. Every catalog
variant marked `runnable` must have one compiled request that selects exactly
that modifier, records the named native runtime input, and passes the selected
mission's preflight/lowering gates. This is how A320 operating mass and
Hummingbird grounded mass remain auditable exceptions rather than a pattern of
unverified metadata-only knobs.

The remaining gaps are equally important:

1. The release catalog now provides a hash-bound inventory of valid retained
   packets and reports the per-packet presence of convergence, robustness,
   reproduction, and cross-fidelity sidecars. It does not yet validate the
   family-specific numerical contents of those optional artifacts or construct
   a full showcase archive/landing page; those remain release-pipeline work.
2. `ParameterSpec` now exports the full shared field set—default disposition,
   bounds, transforms, conditional visibility, coupling, projection policy,
   derivation, invalidations, and provenance—but most imported vehicle inputs
   correctly remain sparse. Only A320 operating mass and Hummingbird grounded
   mass are runtime-bound modifiers; there is no generic family mass/resource
   derivation graph yet.
3. The compiler accepts an acyclic typed graph with declared success, abort,
   resource, envelope, and timeout transitions. Most native translators still
   intentionally execute only the template-owned linear graph. Hummingbird
   pseudo-6DOF is the first explicit branch-capable exception: an airborne
   timeout may transfer committed truth state to its final touchdown segment,
   while independently failing skipped required objectives. Reusable
   resource/envelope/abort execution and equivalent family translators remain
   to be implemented.
4. Capability preflight is now declared for the runnable mission tiers that
   have a family-owned adapter, but many planned tiers still have no adapter.
   The next adapters must be reusable family implementations, not bespoke
   scripts or fallback geometry.
5. Normalized `evaluation.json`, status traces, interface provenance, and the
   discovery result catalog are in place for the current composition
   executors. Every currently runnable batch path now emits either its complete
   held semantic-action trace or an explicit empty trace when the chosen
   interface has no public actions/effectors. This establishes command-history
   provenance, not physical allocation. Resource, achieved-effector, and
   mission-graph dispatch evidence must remain explicitly unavailable until
   their exact factories produce it.
6. Truth-objective target channels, tolerance dimensions, and evaluator-emitted
   gate metrics now resolve through the versioned objective value-space catalog.
   Unknown objective dimensions fail construction rather than becoming an
   undocumented scalar in model units. A new family must add its unit/topology
   contract before publishing its objective.

`taoryx vehicle results <directory>` now supplies the first discovery-only
result catalog. It recursively validates existing `evaluation.json` artifacts,
retains malformed records as explicit failures, and reports scenario identity,
outcome, qualification, gates, and present sidecar artifacts without executing
or re-scoring any vehicle. A colocated `composition.json` is now parsed and
fingerprint-bound to the evaluation; mismatch fails the result rather than
silently detaching provenance. Missing sidecars remain explicitly visible for
external/provider outputs. `taoryx vehicle release-catalog <directory>
--output <release-catalog.json>` builds the companion hash-bound inventory
from valid packets. It reports optional release evidence as present or missing,
requires a small structural envelope for a present JSON sidecar (nonempty
status plus claim/nonclaim text), and rejects a changed checksum ledger,
packet summary, or coverage matrix. Each indexed release packet also retains
only its already verified composition, interface/execution-mode, and
capability-preflight identity so a catalog client can select the exact
vehicle/fidelity path without parsing a family result. Missing external
provenance remains explicitly missing. The catalog does not interpret
family-specific numerical contents or promote a result to
qualification. A complete archive and showcase landing-page pipeline remain
future work.

When both sidecars are present, the result catalog additionally binds the
composition's exact vehicle, family, mission, fidelity, control realization,
and immutable variant manifest to a passing `vehicle_interface.json` record.
The normalized interface provenance also retains each declared endpoint's
`execution_mode`, so a result browser can distinguish closed-loop control,
source replay, direct-wrench screening, and passive execution without
reconstructing that distinction from a factory name.
The interface's family, fidelity, control realization, and execution bindings
must agree with that composition; a contradictory interface is an invalid
result, while an absent legacy/external interface remains explicitly `missing`.

Each valid mission record now also contains an `evaluation_summary` projected
only from the typed normalized envelope: required-objective pass/fail/blocked
counts, worst supplied normalized error, gate counts, and availability counts
for requested-control, achieved-control, resource, and event evidence. This is
the safe comparison surface for a UI or optimizer: it does not parse a family
plot, recompute objectives, derive a terminal state, invent a resource amount,
or turn missing channels into zeros.

Every normal batch executor now passes its committed `status_trace.json` into
that normalized evaluation. The final scalar/boolean resource values therefore
appear as unit-bearing `resources` evidence at their actual committed truth
time. Conversely, declared action and physical-effector channels remain
explicitly `unavailable` unless that executor writes a separate semantic
command or achieved-effector trace. A clean trajectory must not be used as a
proxy for either a requested command or a physically realized effector.

The same executors now produce `resource_ledger.json`, a generic,
identity-bound projection of only interface-declared resource channels at
committed truth boundaries. It gives clients a common history for represented
total mass, battery reserve, propellant, or passive mass without treating any
of those fields as a universal fuel model. The ledger records trace-derived
trends only; it does not invent flow rates, energy coupling, or resource
depletion validation. The result catalog reconstructs the projection from the
colocated status trace and rejects a changed ledger sidecar.

The common `semantic_action_trace.json` artifact records commands as held
interval values paired with the committed truth boundary at the end of that
interval. It carries the exact composition and interface fingerprints, and the
result catalog rejects a cross-composition trace or a command interval whose
start/end are absent from its colocated `status_trace.json`. A trace may omit
intermediate status rows when the command was held, but it may never create an
interpolated command or truth timestamp. The Hummingbird
aggregate-thrust pseudo-6DOF runner and the promoted X8/B747 reduced-fidelity
paths produce the artifact for their declared semantic action channels. Their
achieved physical-effectors remain absent unless the selected fidelity actually
contains that allocation. Every currently runnable batch binding now emits the
artifact: controlled/direct-wrench paths supply their actual held action values,
while action-free replay and passive paths emit an explicitly empty channel
set. A future runner may be promoted only when it can supply every declared
action or effector channel; a clean state trace is not a substitute.

Every execution endpoint now declares its batch-action-trace disposition, so
the composition catalog and authoring worklist distinguish five states rather
than treating a runnable batch endpoint as proof of command evidence:

| Disposition | Meaning |
| --- | --- |
| `emits_committed_interval_trace` | The batch executor emits the identity-bound standard artifact with a complete action/effector channel set. |
| `committed_interval_history_missing` | The native runner exposes controls in state/status, but does not retain the command held over each accepted integration interval. It must not emit the standard artifact. |
| `not_emitted` | A batch path exists but has not yet emitted the standard artifact; actions and effectors remain unavailable in the normalized result. |
| `not_applicable` | The endpoint is not a batch execution (currently an episode), or otherwise has no batch-artifact obligation. |
| `planned` | The execution path is not runnable yet. |

Every endpoint also declares an independent `execution_mode`. This answers a
different question from fidelity and `control_realization`: *how is this
specific factory producing its result?* It is exported by `describe`,
`endpoints`, the authoring worklist, and every exact authoring kit so a caller
does not mistake a reproducible source witness for a closed-loop mission.

| Execution mode | Meaning | Promotion boundary |
| --- | --- | --- |
| `closed_loop_controller` | The factory closes the declared family/mission controller loop at its advertised fidelity. | Still does not imply physical allocation beyond the selected fidelity's control realization. |
| `source_history_replay` | The factory replays a retained, source-owned translation history. | No participating plant, guidance, or allocator is claimed. |
| `source_scheduled_replay` | The factory runs a declared source model with a fixed, source-owned command schedule. | It is not a general user-driven guidance result. |
| `open_loop_witness` | The factory executes a bounded, declared open-loop scenario. | It does not establish terminal feedback control. |
| `local_direct_wrench_screen` | The factory runs a local direct-wrench controller screen. | It does not establish physical effector allocation or a full vehicle mission. |
| `passive_uncontrolled` | The factory evolves a passive/uncontrolled release. | There is no hidden control path or allocation claim. |
| `planned` | No executable factory exists. | The endpoint may not be selected at runtime. |

The catalog validates this declaration fail-closed: a planned binding must use
`planned`, and a runnable binding may not. The field is an execution-provenance
label, not a quality score; `claim_boundary`, control realization, evidence
tier, and qualification still govern what may be asserted about the result.
Source-history replay, source-scheduled replay, and passive-uncontrolled
bindings are also batch-only by contract; exposing one as an interactive
episode would falsely imply a controllable participant. A
`local_direct_wrench_screen` is similarly restricted to the explicitly named
direct-wrench fidelity. These are catalog-integrity rules, not generic claims
about what a future family may physically model.

The X8 point-mass native bridge emits a held-action trace: each accepted interval records
the declared throttle and collective/differential elevon coordinates, and a
trace is rejected if a solver stage changes any of them. Its native racetrack
heading, climb, and speed references are now also resolved once at each
committed truth boundary and frozen throughout the following integration
interval. They remain mission guidance rather than invented external actions.
The X8 point-mass and pseudo-6DOF paths and the B747 point-mass and
pseudo-6DOF paths now all emit the same held-action artifact through the
binding-declared language-backed executor. The repair remains a runtime-owned
committed-control ledger—not reconstructing a command timeline from trajectory
samples. Current reduced A320/F-16 and Hummingbird batch runners also emit the
standard artifact; a valid trace remains a semantic-control claim, not
physical allocation evidence.

### Language-backed committed-control-ledger remediation

The language-backed runtime now provides a reusable
`RuntimeVehicle.committed_control_resolver` seam. It resolves an owned command
mapping at an accepted truth boundary, freezes those names through every
solver-stage environment evaluation, and records their activation timestamp.
Runtime branches retain resolver ownership but re-resolve from the branch's
own committed truth state; callback code is reattached by lowering rather than
serialized as mutable checkpoint state.
The native route translator uses this seam for the X8 and B747 racetrack
references. Their semantic traces remain separately constrained to their
declared public bridge actions. Binding promotion is fail-closed: the generic
trace builder refuses a promoted run when an interval omits a declared action
or reports a solver-stage mutation.

The governing contract remains sample-and-hold at accepted truth boundaries:

```text
committed truth x_k
  -> resolve controller command u_k once from x_k
  -> retain immutable command record (t_k, u_k)
  -> integrate with u_k frozen over [t_k, t_{k+1}]
  -> commit truth x_{k+1}
  -> emit the interval record (t_k, t_{k+1}, u_k)
```

The runtime implementation must separate a pure or explicitly stateful
`resolve_controls_at_committed_boundary` hook from environment/load evaluation.
Solver-stage RHS calls may evaluate forces with the frozen `u_k`, but they must
not mutate the retained command or create a public command sample. The ledger
must retain the resolved action mapping, interval start/end truth times,
controller/update identity, and any source timing/cadence declaration. It
must also survive clone, checkpoint, restore, stepwise replay, segment
transition, and event handling without an interpolated command.

This is a control-timing correction, not an instruction to silently alter a
source control law. The migration has four gates:

1. Add the generic committed-control record and serialization/restore tests
   without changing existing language-backed control behavior.
2. Move one source-owned X8 point-mass route controller to the explicit
   boundary resolver; prove that its solver-stage control mapping is frozen
   during each accepted interval.
3. Compare old and new trajectories/objectives under fixed step sizes, record
   any expected response change, and require batch/step replay parity.
4. Map the complete declared X8 interface action set into
   `semantic_action_trace.json`, then repeat for pseudo-6DOF and B747.

Required negative tests are equally important: a controller that changes a
command only in a solver stage, a missing ledger interval, a checkpoint with a
different active command, an event that changes the command without a
committed boundary, and an attempted trace reconstructed from state telemetry
must all fail closed. Physical elevon allocation remains a separate higher
fidelity/evidence path; this repair only makes requested semantic action
timing honest.

**Progress:** the shared runtime now retains private
`ControlEvaluationRecord` samples at solver-stage and committed-truth
evaluations, plus one accepted `ControlIntervalRecord` per completed interval.
The latter stores the interval-start control vector and explicitly flags any
different solver-stage vector. Clone and interactive checkpoint/restore retain
these records while discarding speculative intervals after event refinement.
This is diagnostic provenance only for every path that has not established an
action mapping. The X8 point-mass and pseudo-6DOF bridges and the B747
point-mass and pseudo-6DOF bridges are the current language-backed exceptions:
their complete declared action sets map directly to retained interval controls
and therefore emit `semantic_action_trace.json` after the
no-solver-mutation check. The diagnostic ledger by itself still does not
promote any other endpoint.

Language-backed composition runs now retain this diagnostic record as
`control_provenance.json` with schema
`taoryx.runtime-control-provenance/v1alpha1`.  The default Product 3 artifact
is intentionally a compact count-only summary: accepted intervals,
solver-stage mutations, eligible intervals, evaluation counts, and the native
control names.  A developer may request full records from the lower-level
runtime API for a timing investigation.  The result catalog lists this file as
an artifact but never treats it as `semantic_action_trace.json`, requested
semantic-control evidence, physical-effector evidence, or a reason to promote
an endpoint disposition.

The catalog now indexes a source-local direct-wrench `local_screen.json` as a
separate `local_controller_screen` record after it verifies the colocated
direct-wrench composition. Its `local_screen_pass` or `local_screen_failed`
outcome is intentionally not a mission result and cannot be compared as a
terminal, allocation, or qualification outcome. This keeps bounded controller
evidence visible without forging an `evaluation.json` for a non-mission.

`taoryx vehicle maturity-report --results-dir <directory>` now adds the same
read-only artifact-catalog projection to the catalog-wide maturity report.
The result-directory dimension is deliberately `not_supplied` unless an author
names a directory, `empty` when that directory has no artifacts, and `fail`
when it contains malformed or contradictory evidence. It neither runs a
vehicle nor allows retained nominal results to promote a fidelity or family.
For supplied artifacts it also counts `verified`, `missing`, or invalid
semantic-action-trace evidence, making control-trace coverage visible without
turning that count into an actuator or qualification score.

6. The existing generic Alpha 2 variant resolver is proven on family-contract
   fixtures, but physical vehicle modifiers remain unavailable until a named
   runtime adapter demonstrably consumes them. The A320 operating-mass and
   Hummingbird grounded-mass bindings are the two current runtime-bound
   exceptions; all other family variation deliberately remains `planned`.

### Source-runner integration note: HL-20

The HL-20 has an explicit Product 3 capability estimate for its public
`high_altitude_release -> lifting_body_glide_energy_management_v1` contract.
It derives release specific energy from the declared altitude and Mach, checks
that the two commanded bank values have opposing signs, and rejects a terminal
energy request greater than the declared unpowered release energy. That is
useful planning evidence, and its point-mass and pseudo-6DOF entries now have
the exact `taoryx.hl20_glide_energy_intent.v1` semantic translator. That
translator emits the immutable release/trim/opposing-bank/energy-handoff plan
used by preflight; it does not itself construct or run the vehicle.

The reason is concrete rather than administrative: the existing source-backed
HL-20 runner is a synthetic-booster, ground-launch/release experiment with a
fixed 25 s release and a pinned time-scheduled bank program. It does not
accept the registry's arbitrary high-altitude release state or independently
execute its trim, crossrange, and energy-handoff segments. Product 3 must not
silently substitute that booster trace for the public glide mission merely
because both use the same DAVE-ML aerodynamic source.

The safe next integration slice is therefore a *separately named*,
source-compatible `hl20_source_booster_release_replay_v1` mission with exact
source-pinned inputs, phase/event objectives, a source-schedule claim
boundary, and a distinct execution binding. It can provide reproducible
3DOF/pseudo-6DOF replay evidence without promoting the high-altitude glide
template. The public glide mission remains non-runnable until a family-owned
native release/trim/glide runtime can consume its declared initial condition
and semantic bank/energy objectives. The semantic translator already makes
that executable contract precise without pretending it supplied the missing
plant or control authority.

The first runtime-bound exception is the OpenAP A320 operating mass. Its
`operating_mass_kg` variant is limited to the pinned clean-configuration table
domain and writes the same `A320OpenAPOperatingPoint.mass_kg` that the
executor uses for performance, trim, and telemetry. It is a valid composed
configuration, not an envelope or mission qualification; every use records
the required retrim and requalification invalidations. Every compiled variant
now also records its declared resolution policy (the current A320/Hummingbird
bindings use `reject_invalid`) and a qualification state:
without a family-declared qualified range, a hard-valid modifier is marked
`extended` rather than being allowed to inherit the baseline's evidence.

### Runtime-bound variant evidence

A selected modifier is not execution evidence merely because its semantic
resolution succeeded. A runnable binding now records its exact native adapter
input path, its coupling policy, and its declared committed-status relation.
Each supporting executor writes `variant_runtime_evidence.json` after a run:

1. it compares the resolved value with the exact adapter input consumed by the
   runtime;
2. it reads the first and final **committed** status samples—never an
   interpolated sensor value; and
3. it validates the binding's declared status relation.

`equal_to_target` means that the initial status channel must equal the
resolved value. `not_asserted` proves only that the declared status channel is
available. The current A320 and Hummingbird mass bindings use
`equal_to_target` for `resources.mass.total`; this does not establish fuel,
inertia, payload-distribution, or endurance derivation. A mismatch fails the
runtime hard gate and is retained in the artifact rather than silently
repairing the request.

The public variant path now mirrors that contract: `taoryx vehicle variant
validate <request>` reports the resolved bounded inputs, runtime input paths,
invalidations, and qualification boundary without running a plant; `taoryx
vehicle variant resolve <request> --output <composition.json>` writes the
same immutable composition that later preflight and execution consume. A
binding defaults to `reject_invalid`; it may explicitly choose
`project_to_valid` only for finite scalar inputs bounded by its declared hard
range. In that case the immutable composition records the original and
resolved values, normalized projection distance, and active bound(s). The
commands never infer an unbound modifier or turn an extended hard-valid
request into qualified evidence.

Each runnable modifier now also declares a `coupling_group`, the status
channels it directly changes, a `resource_derivation` disposition, and a
claim boundary. This is deliberately narrower than a generic mass/energy
derivation graph: A320 mass is adapter-owned at its OpenAP operating point,
whereas the Hummingbird aggregate-thrust bridge explicitly declares battery,
inertia, payload distribution, and rotor resources as unrepresented. A UI or
optimizer can therefore see that these are bounded mass inputs without
mistaking them for a complete loadout or energy model. A runnable modifier is
also fidelity-scoped at compile time; e.g., the Hummingbird mass witness is
accepted only by its named pseudo-6DOF bridge and cannot leak into a lower or
higher fidelity merely because the initialization contract happens to list it.

These are product gaps, not excuses to weaken the existing compiler. The
current exact-template compiler remains the safe authoring path while the
general interfaces mature.

Runtime-consumption audit: a public reset/configuration input is not treated
as implemented until both the declared batch and episode binding consume it
and record the resulting native input. The Hummingbird grounded-reset mass is
now traced in this way; its qualified range remains intentionally unset.

The graph-authoring slice validates exact node coverage, target identity,
reachability, acyclicity, and state-transfer rules before any adapter opens.
An exact caller-authored projection of the template-owned success chain emits
`authored_linear_sequence_lowered` and may reuse that native sequence. Other
branch, fallback, timeout, resource, envelope, or alternate state-transfer
semantics emit `authored_graph_not_lowered` and are blocked until a selected
family explicitly declares support. The Hummingbird timeout-to-safe-touchdown
case is the first such opt-in: it carries that status because it does add a
real branch, but its selected translator records the graph dispatch and keeps
the resulting partial mission failed. This lets composition clients create and
inspect mission decisions without pretending unsupported vehicle runners know
how to execute them.

**Integration note:** a graph node's `instance_id` is semantic provenance, not
display-only numbering. The current powered-fixed-wing compiler uses the
declared `left-turn` and `right-turn` instances to bind source-owned turn
segments. Generic composition tooling must preserve declared instance IDs;
renaming or regenerating them is a semantic change that needs an adapter-level
mapping rather than an implicit convenience conversion.

Every mission authoring kit now includes a `graph_execution_contract` for its
exact family, mission, and fidelity. The ordinary contract permits only the
template success sequence and committed-truth handoff. Hummingbird pseudo-6DOF
is the sole current extension: a timeout may transfer to the declared final
touchdown segment, while abort, resource, and envelope branches remain
explicitly unsupported. A composition UI can therefore reject an unexecutable
graph before it asks a native translator to lower it.

## Maturity model

| Level | Caller can do | Minimum evidence |
| --- | --- | --- |
| P3-0 — declared | Discover a family and its advertised tiers/templates. | Registry parses and cross-references source/fidelity records. |
| P3-1 — inspectable | Query all public metadata, parameter/action/status schemas, execution endpoints, and blockers. | Every advertised record has a fingerprinted, validated descriptor. |
| P3-2 — configurable | Select only bounded independent parameters at correct scopes. | Deterministic resolution report, coupled derivation, and no silent projection. |
| P3-3 — composable | Build a typed mission graph from reusable segments/objectives. | Graph, continuity, capability, and transition validation before integration. |
| P3-4 — executable | Resolve a composition to one exact batch or episode binding. | Source-owned factory, committed truth/status artifact, no fallback. |
| P3-5 — evaluated | Compare requested mission behavior with independent truth results. | Objective table, terminal result, envelope/numerical/resource status, reproduction record. |
| P3-6 — authorable | Add a member of an existing family through a standard intake/template path. | Generated onboarding worklist, conformance fixtures, and one reproducible witness. |

`qualified` is deliberately outside this ladder. Product 3 can make a run
discoverable, executable, and evaluated; family qualification still requires
the fidelity, control, numerical, and robustness evidence declared by the
selected vehicle realization.

## Target public surface

The public surface is a CLI and Python API first. A network REST or UI service
can be layered on later without changing the semantic artifacts.

### Catalog and descriptor queries

Add one versioned aggregate payload and retain current focused commands as
convenient projections:

```bash
taoryx vehicle catalog --detail summary --format json
taoryx vehicle catalog --detail full --format json
taoryx vehicle describe skywalker_x8 --format json
taoryx vehicle missions skywalker_x8
taoryx vehicle segment skywalker_x8 fly_by_turn
taoryx vehicle parameters skywalker_x8 --scope variant_configuration
```

`vehicle list`, `inspect`, and `schema` remain supported aliases/projections.
The full descriptor must include:

```text
identity and source/evidence manifests
physical family and mission overlays
available fidelities and control realization
execution bindings and blockers
initialization, variant, segment, and action schemas
status, resource, diagnostic, and observation schemas
mission templates and segment/objective graph rules
capability/preflight adapter declaration
artifact/evaluation and qualification vocabulary
fingerprints and schema versions
```

The output must explicitly distinguish `declared`, `runnable`, `planned`,
and `not_applicable`. A missing physical rotor, surface, sensor, or resource
channel is not represented by a plausible default.

### Bounded parameter contract

Replace the current shallow composition-input descriptor with a shared
`ParameterSpec` used by variants, initialization, segments, controller
schedules, and optimizer/RL metadata. It should contain at least:

| Field | Purpose |
| --- | --- |
| semantic ID, title, description | Stable user-facing identity. |
| value type, unit, frame, shape | Unambiguous values and vectors. |
| scope | Family, variant, reset, segment, step, or derived/status. |
| default and required rule | A request is complete without hidden defaults. |
| hard/qualified/safe-extended bounds | Validity is distinct from evidence. |
| enum/options or transform | Safe categorical and normalized optimization input. |
| conditional visibility and coupling group | Only meaningful combinations are expressible. |
| derivation and invalidation effects | Recompute mass, resources, trim, capability, or schedules. |
| projection policy and provenance | No silent repair or fabricated confidence. |
| retrim/reschedule/requalification flags | A change visibly carries its required follow-up work. |

Users and optimizers select independent variables such as payload, fuel,
propellant loading, battery state, engine derate, declared controller schedule,
or waypoint target. They do not independently set wet mass, mass flow, total
impulse, inertia, raw aerodynamic-table cells, or internal states unless a
family exposes a controlled calibration interface.

### Value-space and topology contract

Every exposed parameter, action, status, observation, resource, objective,
and error must also declare the mathematical space on which its value lives.
Unit, frame, and vector shape alone are insufficient. In particular, a
heading angle and heading rate are not the same kind of quantity merely
because both have angular units:

| Quantity | Value space | Correct error/operation |
| --- | --- | --- |
| Heading, course, or absolute yaw | Periodic circle \(S^1\) | Wrapped shortest-angle difference, not ordinary subtraction across \(\pm\pi\). |
| Heading/yaw rate | Tangent scalar \(T S^1 \cong \mathbb{R}\) | Ordinary signed linear difference in rad/s. |
| Bank, pitch, angle of attack, sideslip, gimbal, nacelle, or surface deflection | Bounded interval | Ordinary difference and clipping inside declared physical limits; no wrapping. |
| Quaternion attitude | Rotation group \(SO(3)\), represented by unit quaternion with \(q\sim-q\) | Normalization, sign-aware comparison, log-map/geodesic attitude error, SLERP where interpolation is explicitly permitted. |
| Body angular rate | Tangent vector \(\mathbb{R}^3\) | Component-wise linear operations in the declared body frame. |
| NED/ECI position, velocity, force, moment, or wrench | Cartesian vector space \(\mathbb{R}^n\) | Component-wise operations in the declared frame. |
| Unit line-of-sight, thrust direction, or body axis | Unit sphere \(S^2\) | Renormalization and angular/geodesic error. |
| Mass, energy, duration, speed magnitude, or density | Positive/nonnegative half-line | Positivity-preserving bounds and transforms. |
| Fuel fraction, throttle fraction, SOC, or blend weight | Closed unit interval \([0,1]\) | Bounded scalar operations. |
| Mass-fraction/loadout distribution | Simplex | Sum-preserving bounded transform, not independent scalar clipping. |
| Mode, stage, contact state, boolean/event | Discrete finite set | Equality/transition semantics; no numeric interpolation. |

The descriptor should carry a compact, machine-readable `value_space` record:

```yaml
value_space:
  kind: periodic_angle             # e.g. euclidean_vector, bounded_interval, so3_quaternion
  topology: S1
  representation: scalar_radians
  period: 2*pi
  error_rule: wrapped_difference
  interpolation_rule: circular_geodesic
  normalization_rule: none
```

The current executable scalar variant contract is intentionally narrower than
the catalog vocabulary: `identity`, `log`, and `logit` are supported.
`log` requires a strictly positive hard lower bound; `logit` requires explicit
hard bounds `[0, 1]` and publishes a `unit_interval` value space. A simplex
requires a jointly resolved vector and membership contract, while a categorical
modifier requires a declared finite option set. Those are rejected at the
scalar runtime boundary until their richer schemas exist—never silently
flattened into numeric controls.

For quaternion attitude, `representation: quaternion_wxyz`,
`topology: SO3`, `equivalence: q_equals_negative_q`,
`error_rule: log_map`, and `normalization_rule: unit_norm` are required.
For a physical elevon, `kind: bounded_interval` and its mechanical range are
required; it must never be labeled periodic merely because it is measured in
degrees. For an Euler representation, the descriptor must identify the
Euler order and singularity boundary; it is a coordinate representation of
orientation, not a declaration that its three components are independent
linear variables.

This metadata governs more than display formatting:

- controller/reference errors and termination tolerances;
- action validation, clipping, projection, and rate limits;
- optimization/RL transforms and distance metrics;
- sensor/noise representation and residual calculation;
- replay, comparison, and any explicitly allowed interpolation;
- waypoint, gate, pointing, and attitude-objective semantics; and
- plot unwrapping versus physical error calculation.

The accepted-truth contract still prohibits synthesizing a sensor value by
interpolating from post-step truth. If a permitted diagnostic renderer or
comparison tool resamples declared values, it must use this value-space rule
and label the result as derived rather than committed truth.

### Mission and trajectory authoring contract

The semantic mission layer needs a reusable graph, not arbitrary controller
instructions:

```text
MissionTemplate
  -> SegmentGraph
      -> SegmentSpec
          -> objective(s), path constraints, resources, controller authority
          -> named success, abort, resource, envelope, and timeout transitions
```

Use typed objective primitives:

- `fly_over` and `dwell` for hover/precision capture;
- `fly_by_gate` and `path_corridor` for fixed-wing route geometry;
- `loiter`, `event`, `energy_corridor`, `terminal_state_gate`, and `touchdown`;
- family extensions such as staging, conversion, detumble, wheel unload, or
  impact, each with explicit truth-evaluation semantics.

Waypoints are therefore values within declared objective/segment types, not
untyped coordinate dictionaries. A fixed-wing fly-by route must state gate
orientation, crossing direction, corridor, speed/altitude/heading constraints,
and turn intent. A multirotor capture must state position, velocity, heading,
and dwell acceptance. Controller transitions remain diagnostic; an independent
truth evaluator determines objective success.

## Delivery plan

### M0 — freeze and publish the existing contract

**Goal:** Make the existing Product 3 surface easy to discover without
changing execution behavior.

**Progress:** `vehicle catalog --detail summary|full` now exposes both a
compact discovery index and one self-contained, versioned full descriptor
document. The value-space foundation is implemented for the resolved
vehicle interface, public episode schemas, and all current composition inputs: every exported channel
and registry parameter now carries topology, representation, error,
interpolation, and normalization metadata; compile-time input validation uses
that contract; known hard bounds and finite options are enforced; and all 36
catalog contracts pass the fail-closed check. `episode-info` now also carries
reviewed topology for legacy native channels rather than leaving an AI/RL
caller to guess that a yaw is circular or that a wrench is Cartesian. The
source-table episode mapping is deliberately source-owned: X8/B747 source
degree headings use a period of 360 rather than inheriting a radians default,
and known source speed/mass/resource/pressure/time magnitudes, bounded angles,
fractions, and numeric flags retain their distinct spaces. The independent truth-objective
layer now publishes the same contract per target channel and evaluates heading
as a periodic circle, so a crossing of the \(\pm180^\circ\) chart boundary
cannot manufacture a terminal miss. Gate normals are explicitly unit
directions on \(S^2\). `taoryx vehicle topology-report` is now the catalog
gate for this promise: it audits all registered initialization/segment/variant
inputs, 36 resolved interfaces, observation bindings, and the objective schema
for absent or pending topology. Qualified bounds remain unset where vehicle evidence
has not established them. The next migration slices are explicit YAML-owned
parameter/objective metadata and runtime-bound bounded variants. Every
existing ordered mission template now also serializes as a versioned
`linear_sequence_only` graph projection with explicit instance identity and
truth-state handoffs. The first M2 slice accepts a validated acyclic graph but
keeps it blocked from native execution until a family translator declares
branch semantics.

Deliver:

- a `vehicle catalog`/`describe` JSON document that joins all current records;
- machine-readable CLI help examples and a schema version policy;
- an explicit endpoint matrix: family × fidelity × mission × batch/episode;
- a separate batch/episode parity matrix whose `registered` state requires an
  exact committed-boundary witness; two runnable operations are explicitly
  `not_registered` until that evidence exists;
- a documentation table showing exactly what `list`, `inspect`, `schema`,
  `interface`, `endpoints`, `compose`, `preflight`, `lower`, and `run` answer;
- one conformance test ensuring every CLI projection agrees with its resolved
  full descriptor.
- define the `ValueSpaceSpec` vocabulary and publish a migration inventory for
  every currently exposed parameter/action/status/observation/objective
  channel.

**Progress:** `taoryx vehicle authoring <family>` now provides the companion
composition worklist: mission × fidelity rows list interface validation,
available batch/episode operations, graph support, variants, and the next
declared Product 3 action. It does not duplicate source-intake or promotion
evidence owned by the vehicle-integration workflow.

Exit:

- a client can enumerate every current vehicle and obtain the full metadata
  for any one with no repository-file knowledge;
- all descriptor references/fingerprints are valid; and
- no command describes a planned tier as executable.
- every exposed channel is either assigned an explicit value-space contract or
  marked `topology_pending`; no caller may infer periodicity from units or
  identifier spelling; and
- truth-objective comparisons use their declared error rule rather than a
  generic scalar subtraction.

The same topology report now emits `parameter_contract_maturity` per family
and across the catalog: it counts only declared defaults, explicit/effective
hard bounds, qualified/safe-extended ranges, transforms, provenance,
derivations, and runnable runtime-backed variants. This is an intake and
prioritization instrument, not a quality score: a missing field remains a
visible source or adapter decision rather than an excuse to invent a generic
controller constant.

`taoryx vehicle maturity-report` now joins that topology evidence with the
authoring catalog's graph/capability coverage, exact endpoint operations, and
parity disposition. It now also projects the family-integration strategy
worklist: strategy-conformance, per-tier development state, and the exact next
admissible integration action. This makes reusable family onboarding and its
unresolved adapter/data prerequisites visible beside composition maturity,
without treating `planned`, `blocked`, or `strategy_development` as a catalog
failure or a qualification result. Its optional `--check-execution-witnesses` mode invokes
the checked-in composition-witness gate. The stronger
`--execute-batch-witnesses` option also runs each batch endpoint and validates
its declared action-trace disposition: promoted batch endpoints must write a
valid identity-bound `semantic_action_trace.json`, and unpromoted endpoints
must not write one. The companion `--execute-parity-witnesses` option replays
one checked-in, exact semantic trace through every pair whose parity record is
already `registered`; a missing or excess witness fails the parity gate. It
does not turn a `not_registered` or unavailable pair into a parity claim. The
default deliberately records `not_checked` rather than presenting catalog
declaration as execution proof.

Its execution-mode inventory deliberately preserves two views: an exact
endpoint-binding count (`execution.endpoint_execution_mode_counts`) and a
deduplicated mission-tier availability count
(`discovery_and_authoring.execution_mode_counts`). A batch/episode pair can
therefore count as two runnable endpoints but one mission tier offering the
same execution semantics; neither number is silently substituted for the
other.

The batch action-trace contract also distinguishes an action-free replay from
missing command history. A source replay or explicitly open-loop trajectory
with no batch-visible action/effectors emits an identity-bound trace with empty
channel sets and accepted truth timestamps. This never means “all controls are
zero.” A path that declares an action or effector must emit the actual held
value; otherwise its execution binding remains below the action-evidence tier.

### M1 — bounded variant and configuration resolver

**Goal:** Let users select legitimate vehicle-specific variation without
arbitrary model mutation.

**Progress:** The resolver now emits an immutable `CompiledVariantResolution`
with applied values, runtime bindings, invalidations, qualification state, and
an explicit projection record when a family deliberately selects
`project_to_valid`. Projection is currently limited to finite scalar values
and declared hard bounds; wrong units, non-finite values, unsupported value
spaces, and undeclared modifiers always reject. Existing A320 and Hummingbird
runtime-backed modifiers retain the strict `reject_invalid` policy, so this is
an optimizer-facing capability rather than a silent change to operational
vehicle requests. Bound, derivation, and evidence completeness remains visible
through the catalog's `parameter_contract_maturity` report.

Runtime modifiers also enforce their declared coupling groups. By default a
group is `exclusive`: two modifiers in it are rejected before an adapter sees
them. A family may set every participating binding to `composable`, but that
choice is serialized in the resolved runtime bindings and must still name the
same derivation/retrim/requalification consequences. A coupling label is no
longer merely descriptive metadata.

**Current scalar-topology boundary:** executable runtime bindings accept only
`identity`, `log`, and `logit`. Log bindings require a strictly positive hard
lower bound; logit bindings require explicit `[0, 1]` hard bounds and expose a
unit-interval value space. Every runnable scalar binding now also serializes a
declared `value_space_profile`; it may not infer a topology merely from a
nonnegative lower bound. `simplex` and `categorical` are deliberately rejected
by the current scalar binding. The next schema increment is a
separate vector/discrete variant binding with an explicit simplex membership
group or finite option set, corresponding runtime lowering, and a
reproducible coupled-resolution artifact. It must not be implemented as a
dictionary override or a scalar encoding convention.

Deliver:

- `ParameterSpec`, `VariantSpace`, semantic modifiers, derivation graph, and
  immutable `ResolvedVehicle` manifest;
- a `VariantRuntimeBinding` for every advertised modifier, naming the exact
  source-owned adapter input, the affected resource/mass/aerodynamic state,
  and the required retrim, reschedule, or requalification probes. An absent
  binding keeps the modifier `planned`; it must never be treated as a harmless
  metadata-only override;
- `ValueSpaceSpec` validation for parameter domains, control commands,
  objective tolerances, and canonical error/projection operations;
- hard-valid, qualified, and safe-extended ranges with explicit reject or
  project policy;
- generic resource-ledger mapping and coupled mass/energy derivation;
- `taoryx vehicle parameters`, `variant validate`, and `variant resolve`; and
- source-backed initial parameter spaces for X8, B747, Hummingbird, X-15,
  F-16, HL-20, A320, NESC rocket, and tumbling body where applicable.

Exit:

- no accepted variant contains inconsistent mass, resource, or declared
  bounded parameter values;
- every projection, extrapolation, and derived value is recorded; and
- heading, quaternion, unit-vector, simplex, positive-resource, and
  bounded-effector fixtures reject mathematically invalid operations.
- a resolved variant has a reproducible fingerprint before mission compilation.
- a vehicle variant changes the same adapter input that appears in its runtime
  provenance and telemetry; a modifier that cannot meet this traceability rule
  remains undiscoverable as an executable vehicle option.

### M2 — semantic mission graph and objective library

**Goal:** Generalize from one rigid sequence per template to safely configurable
mission composition.

**Progress:** The public `taoryx vehicle mission` workflow now presents the
registry/compiler seam as one authoring path: `inspect` resolves the exact
mission, selectable starts, ordered segment contracts, graph, and compatible
fidelity contracts; `create` emits the existing no-invented-default authoring
kit; and `validate` compiles a request into its immutable fingerprinted
composition while reporting interface validation, graph status, preflight, and
runtime lowering separately. It does not create an alternative mission format
or treat a semantic pass as an executable/qualified run. The current graph
compiler validates typed acyclic transitions and can lower only an exact
template-owned success sequence; arbitrary fallback and bounded loop semantics
remain intentionally blocked until a family translator explicitly owns them.
The independent truth-objective evaluator now likewise has a closed generic
vocabulary: every current target, tolerance, and emitted gate metric has an
explicit unit/topology record. It rejects an undeclared objective dimension
instead of accepting a model-unit scalar by convention. That makes a mission
author add the parameter-space decision before a new family objective can
enter a public evaluator or result artifact.

Deliver:

- graph-form mission templates with named entry/exit and fallback transitions;
- common typed objective schemas and truth evaluators;
- reusable waypoint/gate/corridor constructors with explicit frames and units;
- segment constraints for resource reserves, envelope margins, timing, and
  controller authority; and
- `taoryx vehicle mission create`, `mission validate`, and `mission inspect`
  APIs that produce the existing immutable composition request format.

Exit:

- a user can create a valid racetrack with different waypoint geometry,
  altitude gates, turn direction, and terminal gate without editing a
  controller or native input file;
- an invalid sequence, incompatible objective, or discontinuous state handoff
  fails before simulation; and
- controller advance cannot make an objective pass without truth evidence.

### M3 — family capability, geometry, and feasibility adapters

**Goal:** Replace route trial-and-error with first-pass, explainable planning.

**Progress:** `taoryx.mission_capability` now defines the fail-closed
`MissionCapabilityAdapter` seam. The existing capability-scaled racetrack is
registered as `taoryx.powered_fixed_wing_racetrack.capability_scaled.v1` for
X8, B747, A320, and F-16 only. It remains the same source-provenanced turn,
climb/descent, dwell, and horizon derivation used by execution preflight, but
is now independently discoverable through each family authoring worklist. The
Hummingbird pseudo-6DOF route uses its separate
`taoryx.multirotor_hover_translation.capability.v1` planner, which derives
aggregate hover thrust reserve and conservative vertical/lateral authority
without implying individual rotor allocation. The HL-20 point-mass and
pseudo-6DOF tiers now own the deliberately narrow
`taoryx.hl20_glide_energy.capability.v1` estimate: it derives release specific
energy from the declared altitude and Mach, verifies finite opposing bank
intent, and rejects a terminal energy target above that unpowered release
energy. It reports `likely_feasible` only as a necessary-condition screen.
Its installed semantic lowering makes the release/trim/opposing-bank/
energy-handoff intent auditable at preflight, while native execution remains
blocked and makes no trim, crossrange, nonlinear plant, direct-wrench, or
physical-effector claim.
This intake also splits the fixed-wing `bank_limit_deg` magnitude from the
lifting-body `bank_command_deg` signed scalar. The latter has a bounded signed
interval rather than borrowing turn direction from a fixed-wing
route field; its value-space and `[-89, 89]` hard bounds are therefore visible
to authoring and optimization clients.
Other lifting bodies and planned family tiers still require their own
source-owned calculator rather than a fallback. The
canonical passive-body witness now owns
`taoryx.passive_tumbling_release.capability.v1`: it reports release energy,
ballistic coefficient, area policy, and whether the declared horizon contains
the vacuum-fall lower bound. It does not invent control, wrench, or actuator
authority; native passive propagation remains authoritative for impact and
rigid-body rotation. The pinned NESC source replay likewise owns
`taoryx.nesc_staged_source_replay.capability.v1`, which verifies ordered
ignition, separation, cutoff, and endpoint intervals and reports the retained
mass/heading provenance. It is explicitly a source-history capability record,
not evidence of a participating propulsion, gimbal, or guidance plant.
The separate X-15 staged reduced witness owns
`taoryx.x15_staged_reachability.capability.v1`, which exposes source-pinned
boost, release, and horizon chronology while retaining its open-loop or named
response-law boundary. Neither the NESC nor staged X-15 adapter promotes a
source replay or reduced witness to an effector-allocated vehicle controller.
The narrower X-15 local direct-wrench screen likewise owns
`taoryx.x15_local_direct_wrench_screen.capability.v1`. It publishes only the
pinned source-local state, six-axis bounded wrench authority, rate limits, and
screen cadence. It is not route feasibility, physical trim, or surface/RCS
allocation evidence. Where a native adapter exists, runtime lowering now joins
the adapter-bound result to an exact runnable batch factory only after that
same immutable composition is `translation_ready`; adapter construction alone
does not imply segment execution.

Every local direct-wrench screen now has an explicit equilibrium gate before
its LQR recovery score can pass. The requested balancing wrench must survive
the declared projection without saturation, and the resulting source-plant
reference derivative must be below a declared normalized residual limit. This
prevents a linear controller from being reported against a source point that
the bridge cannot hold. The current HL-20 Mach-2 source point fails this gate:
its required X/Z balancing forces exceed the declared `±200 kN` bridge limits.
That remains a source-owned authority/trim blocker; the limits must not be
widened merely to promote the screen.

The same source package also supplies a distinct Mach-0.5, alpha-5-degree
local point whose balancing wrench is inside those existing bounds. It is now
registered as `hl20_local_direct_wrench_screen_v1`, with the same explicit
batch and interactive bridge contract as the X-15 screen. This is a reusable
screen-definition path, not a reinterpretation of the blocked Mach-2 case:
it adds local source-load recovery evidence only. The HL-20 high-energy glide
mission remains development status until release/trim/gravity/guidance and its
authority requirements are independently satisfied.

The Hummingbird `grounded_operating_mass_kg` variant is the second runnable
variant binding after the A320 operating mass. Its hard upper bound is derived
from the aggregate pseudo model's declared maximum collective thrust and
requires strictly positive stationary-hover reserve. It is intentionally
unqualified: it does not imply an identified payload/inertia range, rotor
allocation, or battery-endurance validation.

The checked-in variant witness catalog now has the same optional batch-smoke
mode as endpoint witnesses. When enabled, it runs each exact variant
composition through its declared batch factory and requires both the
identity-bound `variant_runtime_evidence.json` sidecar and one valid normalized
mission result record. The A320 operating-mass and Hummingbird grounded-mass
examples currently pass this gate. The gate proves selected adapter-input
consumption and the declared committed-status relation only; it does not
promote either modifier to a retrim, energy-resource, or envelope
qualification claim. A modifier advertised as runtime-bound must declare that
public batch endpoint even when smoke is not requested; otherwise the witness
catalog fails before a user can discover an executable-looking variant with no
committed-truth evidence path.

The fixed-wing racetrack compiler now treats the paired turn-radius and
bank-limit inputs as actual semantic geometry requests. The template requires
an explicit left turn followed by an explicit right turn with matching radius
and bank magnitude; this preserves the reusable symmetric racetrack contract
rather than silently selecting one side's values. A requested radius larger
than the capability-derived minimum is preserved. A tighter request is raised
to the declared bank-margin radius and recorded as a capability clipping
diagnostic. The same resolved radius is then used for truth-gate geometry and
the disposable native problem materialization. This is a bounded template
feature, not a claim that arbitrary waypoint graphs or asymmetric turns are
currently executable.

An exact template-preserving mission graph is now also eligible for the
source-owned batch-factory handoff after semantic preflight. `taoryx vehicle
lower` records this as `factory_bound` and serializes the exact runnable
factory declaration; it does not manufacture a generic adapter to bridge the
gap. The Hummingbird translator additionally opts into a timeout-to-final-
touchdown graph edge and emits `mission_graph_execution.json`; skipped
objectives fail independently, so this recovery branch cannot promote a
partial mission. Other branching graphs and compositions without an exact
translator or batch binding remain blocked.

Graph-extension eligibility is read from the catalog's
`mission_graph_execution_contract` by both semantic preflight and runtime
lowering. The contract is a typed `graph_execution_extension` field on the
affected mission template, rather than a family-specific conditional in the
runtime. A family therefore declares its one supported extension in one
discoverable place; the family translator must still validate the concrete
nodes, outcomes, target, and state-transfer rule before execution. The current
Hummingbird timeout-to-touchdown path is the only extension, and it remains a
safe recovery branch rather than a generic branching permission.

Graph execution evidence is now a common artifact rather than an
adapter-specific inference. `mission_graph_execution.json` uses
`taoryx.mission-graph-execution/v1alpha1`: `observed` records exact committed
outcomes, selected declared transitions, and state-transfer rules; `unobserved`
records that a runner has no controller-transition evidence. Hummingbird
pseudo-6DOF is observed. The current language-backed fixed-wing factory is
unobserved even when its independent truth objectives pass. This prevents a
successful batch trajectory from promoting itself into a controller or
fallback-graph claim.

Observed graph dispatches additionally carry the exact `committed_time_s` at
their terminal truth boundary. The runtime and result-catalog validation reject
non-finite, negative, and time-reversing records. Therefore a transition packet
cannot reorder its committed truth evidence after execution.

The Hummingbird graph has both a nominal and a forced-timeout execution
witness. The latter makes a translation target unreachable, observes the
declared timeout-to-touchdown dispatch, and confirms that independent truth
evaluation rejects the mission because required nodes were skipped. This is a
bounded recovery proof only; it does not qualify timeout recovery as mission
completion.

The endpoint-witness catalog now also carries a dedicated graph-extension
witness class. It does not duplicate the mandatory endpoint witness for the
same family/mission/fidelity/batch tuple. Instead it proves that an authored
graph with a declared alternate transition compiles, preflights, and lowers
through the same family translator. Under the optional batch smoke it must
produce an `observed` `mission_graph_execution.json` record. Hummingbird's
normal graph witness follows the success chain; the forced-timeout regression
remains the evidence for the actual recovery edge and failed mission result.
This is graph-execution promotion, not a general branch interpreter or a
timeout-success policy.

The reusable `composition_graph_runtime` remains narrower than a generic
mission simulator. It invokes a family-owned segment callback, selects only
the compiled edge for that outcome, and transfers the committed terminal truth
state unchanged for `previous_terminal_truth_state`. A
`declared_physical_transition` fails closed unless the family supplies its own
transfer callback. Consequently the shared layer cannot invent a stage
separation, contact reset, resource handoff, or fallback controller merely
because a semantic graph names one. Hummingbird is its first client, but its
timeout-to-landing behavior remains a Hummingbird translator opt-in rather
than a generic graph capability.

The normalized result catalog now reopens the colocated immutable composition
when that artifact is present. It verifies the graph status, entry node,
expected node order, each observed dispatch's segment/outcome/transition/state
transfer, and the declared completion flag against the compiled graph. Thus a
retained packet cannot relabel a timeout as success, alter a target edge, or
claim a complete nominal path through a partial dispatch list. `unobserved`
records still remain valid only with an explicit reason and no implied
controller progress. The Product 3 maturity report exposes verified/missing
graph-artifact and observed/unobserved counts when it indexes retained result
packets.

Deliver:

- a common `MissionCapabilityAdapter` interface;
- family calculators for powered fixed wing, multirotor, rocket, lifting body,
  passive ballistic body, and later rotorcraft/spacecraft;
- derived turn radius, climb/descent, resource, release, staging, or energy
  estimates appropriate to each family; and
- structured classifications: `feasible`, `likely_feasible`, `unknown`,
  `likely_infeasible`, and `certainly_infeasible`.

Exit:

- the route compiler provides its geometry/horizon derivation and clips or
  rejects impossible requests visibly;
- the same X8/B747/A320/F-16 mission intent scales from each capability
  profile; and
- preflight never silently replaces requested geometry or dynamics.

### M4 — uniform execution and episode factory contract

**Goal:** Make one resolved composition open the same externally visible run
or episode interface across families.

**Progress:** Strict semantic action-trace parity is now available for the
Hummingbird named aggregate-thrust pseudo-6DOF tier, the X8/B747
language-backed point-mass and route-lag pseudo-6DOF tiers, and the A320
OpenAP and F-16 source-reduced point-mass and pseudo-6DOF tiers. Each witness requires its exact declared
batch and episode factories, replays the episode's canonical action frames
through a fresh source-owned runtime session, preserves the declared sensor
truth boundaries, and compares every committed canonical status frame. The
adapters are intentionally family and fidelity specific; neither substitutes a
nearby plant. This is action-trace kernel parity only: it does not replace the
closed-loop mission runner, establish physical allocation, or promote
mission/qualification evidence.

The X-15 source-local direct-wrench screen now has the same explicitly
registered batch/episode parity evidence. Its verifier replays caller-owned
total force/moment requests through a fresh bounded direct-wrench projection
and source-local velocity/rate derivative, then compares the resulting
committed status frame to the episode. This is intentionally limited to the
local bridge: it does not promote the X-15 screen to release, propulsion,
flight trim, physical effectors, or a high-energy mission.

The same verifier now serves the separately declared HL-20 subsonic local
screen. Its parity witness binds the HL-20 composition identity, its generic
local-screen episode factory, and the same canonical wrench channels. This
proves only batch/episode agreement for that bounded source-local bridge; it
does not substitute the screen for the blocked Mach-2 balance point, the
high-energy glide mission, or physical control-surface allocation.

For X8/B747, the parity witness is an explicit replay-policy trace supplied to
the batch/episode adapter. It is distinct from a run-owned
`semantic_action_trace.json`: the replay trace proves batch/episode status
parity but cannot substitute for run-owned action provenance. The
language-backed X8/B747 point-mass and pseudo-6DOF endpoints now emit the
separate held-action artifact through the committed-control ledger described
above.

The witness is available both as the Python API
`verify_serialized_composition_batch_episode_parity` and as
`taoryx vehicle batch-episode-parity <composition> <trace>`. The latter makes
the persisted public trace, immutable composition identity, selected
interface, authority profile, and declared substep part of a reproducible
external handoff. It still refuses all unregistered combinations instead of
claiming catalog-wide parity from one Hummingbird result.

The remaining parity work is intentionally an onboarding matrix, not a
generic comparison loop that would accidentally substitute a nearby plant:

| Family path | Current executable operations | Honest parity disposition | Next required adapter work |
| --- | --- | --- | --- |
| Hummingbird aggregate pseudo-6DOF | batch + episode | `registered` | Preserve the witness as rotor/motor fidelity increases; do not reuse it for a different multirotor plant. |
| X8/B747 language-backed point-mass and route-lag pseudo-6DOF fixed wing | batch + episode | `registered` | Preserve the source-materialized action-trace replay separately for each declared fidelity; the pseudo witness does not promote response-law evidence to rigid-body or effector evidence. |
| A320/OpenAP point-mass and route-lag pseudo-6DOF | batch + episode | `registered` | Preserve the exact stateful-stepper action-trace witness; it does not promote kinematic guidance into physical surface or moment evidence. |
| F-16 source-reduced point-mass and attitude-response pseudo-6DOF | batch + episode | `registered` | Preserve the family-specific source trim/load bindings and action-trace witness; do not interpret sampled source controls as allocated surfaces. |
| X-15 local direct-wrench screen | batch + episode | `registered` | Preserve the source-local total-wrench bridge witness; do not interpret it as physical effector allocation or flight-mission parity. |
| HL-20 local direct-wrench screen | batch + episode | `registered` | Preserve the bounded subsonic source-local bridge witness; it neither clears the blocked Mach-2 point nor promotes lifting-body glide or surface-allocation evidence. |
| NESC staged replay and passive tumble | batch only | `not_available` | Decide whether interactive stepping is a supported product need; passive/uncontrolled paths may legitimately remain batch-only. |

This matrix is also a tuning guardrail: a controller, trajectory, or scalar
terminal result cannot be used as a surrogate for batch/episode equivalence.
Where no paired operations exist, the correct client-visible answer remains
`not_available`, not an inferred green check.

#### Reduced fixed-wing episode onboarding note

The A320/OpenAP and F-16 source-reduced point-mass and pseudo-6DOF executors
now expose an accepted-truth episode built on a source-owned stateful stepper.
The F-16 implementation retains its family-specific source trim, loads, and
attitude response while reusing the same public kinematic-guidance action,
canonical status, checkpoint, and committed-boundary parity protocol. A
completed telemetry file would not be an episode: external commands must
change committed truth state through a stateful reduced-flight stepper with
these explicit operations:

1. initialize the family-specific trim, route, response profile, and runtime
   state from the immutable compiled composition;
2. accept only the composition interface’s declared semantic action profile;
3. translate that action to the runner’s native command space without
   replacing its family plant or response-law fidelity boundary;
4. advance to the minimum of the requested action endpoint and any declared
   sensor truth boundary; and
5. emit the same canonical committed status projection used by batch
   telemetry.

The A320 implementation established this protocol; the F-16 now proves it can
be reused with thin family-specific initialization/evaluation bindings rather
than a trace-playback episode or a separate public control vocabulary.

The implementation boundary is `ReducedFixedWingCompositionEpisode`: a new
member supplies a stateful source-owned stepper, its trim/response factory,
native conversion of the shared `kinematic_guidance` coordinates, and an exact
checkpoint-state codec. It inherits bounded action validation, semantic action
lowering, canonical status/observation projection, committed-boundary timing,
and the public checkpoint lifecycle. A conforming family must still register
its own execution factories and parity adapter; the shared class never grants
an endpoint or evidence tier merely because a similar aircraft already has
one.

All runnable episode entries are also checked against the runtime's explicit
factory registry. `registered_episode_factory_ids()` must equal the set of
`status: runnable`, `operation: episode` factory IDs in
`vehicle_execution_bindings.yaml`. This turns a stale YAML factory ID into a
focused conformance failure before a user reaches `open_vehicle_composition_episode`.
The factory registry remains deliberately shallow: it chooses the exact
family-owned constructor; it does not choose a controller, invent an adapter,
or imply parity/evidence for an endpoint that lacks a separately registered
witness.

#### Episode schema conformance and native-name boundary

`validate_vehicle_composition_episode_contract` now runs when every
advertised episode witness opens. It protects the important distinction
between a stable Product 3 API and source-owned runtime coordinates:

1. The resolved `VehicleInterfaceContract` is the cross-family public API.
   Semantic action IDs, canonical units, value spaces, authority profiles,
   canonical status, resources, diagnostics, and observation profiles are
   all fingerprinted there.
2. An episode may retain legacy native names such as `throttle`,
   `collective-elevon-deg`, or a source-table state path. Those names are an
   implementation sidecar, never an alternative semantic action API.
3. Every available semantic action must have exactly one declared
   `native_action` that appears in the episode's native action schema, with
   compatible unit and bounds. Conversely, an episode cannot expose an
   externally commandable native action without a semantic authority binding.
4. Every episode returns canonical status and every available observation
   profile at the same committed truth boundary, carrying the exact interface
   ID and fingerprint selected during composition.

There are two honest observation-schema projections:

| Projection | Current users | Meaning |
| --- | --- | --- |
| `native_truth` | X8/B747 source-table runtime, Hummingbird response witness, X-15 local direct-wrench screen | The episode publishes every committed raw diagnostic under explicit native schema entries. The conformance check fails if emitted raw truth and that schema disagree. |
| `semantic_contract` | A320 and F-16 reduced episodes | The episode publishes the canonical available status/resource/diagnostic channels directly. Raw stepper telemetry remains an implementation detail behind the status projection. |

The projection is recorded in the endpoint-witness report as
`episode_contract`; neither projection changes the public semantic interface.
This closes a practical onboarding gap discovered during the A320/F-16 work:
new adapters no longer have to pretend a family-specific source state is a
universal observable, but they also cannot leak an unregistered native
control bypass. Batch/episode parity remains a separate claim and still
requires its registered action-trace witness.

Deliver:

- one factory protocol for batch and episode capabilities;
- selected action profile, observation profile, reset scope, seed policy, and
  checkpoint/replay declaration in every resolved composition;
- an `episode-info` open check that emits the exact declared episode factory,
  schemas, and reset-time committed-truth/status/observation frames without a
  family or fidelity fallback;
- exact batch/episode status/resource projection from committed truth; and
- one semantic action-trace parity harness where both modes are advertised.

Exit:

- clients do not branch on native state-vector layout to create a supported
  batch run or episode;
- all advertised status/resource/observation fields have declared availability
  and timing; and
- unsupported episode/reset/sensor capabilities fail explicitly.

### M5 — evaluation, catalog results, and showcase handoff

**Goal:** Give external tools one result vocabulary that preserves family
detail without hiding evidence boundaries.

**Progress:** `taoryx vehicle results <directory>` now indexes normalized
evaluation artifacts without re-running a vehicle or recomputing objectives.
It separately verifies composition and interface provenance, and now verifies
the optional graph-execution artifact against the exact compiled composition.
The catalog projects `observed` versus `unobserved` graph evidence and nominal
success-path disposition rather than inferring controller progress from a
passing trajectory. A malformed or cross-composition graph artifact fails the
catalog, including a local direct-wrench screen; a missing artifact remains
explicitly `missing` for retained or external results. Each bound result also
publishes the execution catalog's `registered`, `not_registered`, or
`not_available` batch/episode-parity disposition. A result never promotes
itself to parity merely because the two operations happen to exist.

Every current public `taoryx vehicle run` batch endpoint also writes an
identity-bound `reproduction.txt`. The record names the exact composition ID
and fingerprint, execution factory, execution mode, and the public command
that produced the packet. The endpoint witness gate rejects an absent,
malformed, or cross-composition record. This makes reproduction provenance a
runtime obligation for newly generated packets, while the discovery catalog
continues to report a missing record explicitly for retained or externally
provided packets rather than fabricating one.

When a retained packet contains both `reproduction.txt` and a verified compiled
composition, the discovery catalog now validates the command’s required public
shape and its composition fingerprint, selected factory, and execution mode.
An external recipe without a Taoryx composition remains explicitly `unbound`;
it is preserved for discovery but cannot be mistaken for a bound public-run
recipe. The maturity report projects the resulting verified/missing/unbound
coverage alongside action and resource evidence.

The result catalog applies a deliberately narrower rule than the execution
witness gate. A present `semantic_action_trace.json` must match a verified
composition and a selected batch binding that declares
`emits_committed_interval_trace`; otherwise the result is invalid. A missing
trace remains explicitly `missing`, even for a promoted binding, because the
catalog also indexes retained and externally supplied partial packets rather
than re-executing them. The endpoint witness smoke is the authoritative
fail-closed proof that a runnable promoted endpoint actually emitted the trace.
That smoke now also indexes each generated packet through the result catalog:
every batch endpoint must produce exactly one valid normalized mission record,
except the explicit X-15 local direct-wrench controller screen, which must
produce exactly one valid `local_controller_screen` record. This proves packet
interoperability without treating a bounded translation smoke or local screen
as a qualification result.

The same endpoint gate now builds and validates a one-packet release catalog
for every generated batch output. It requires a verified bound reproduction
identity in that packet, so release packaging regressions are caught at the
endpoint seam rather than discovered after artifacts are collected. This is
release-packet conformance only; it does not supply robustness evidence or
promote any nominal mission or local screen to qualification.

For the currently runtime-bound A320 and Hummingbird mass variants, the same
catalog now validates the optional `variant_runtime_evidence.json` sidecar.
It binds the artifact identity to the compiled composition, requires a
consistent pass/fail/not-applicable aggregate over its explicit bindings, and
exposes the evidence status separately from the mission result. It does not
recompute the plant or infer unrepresented resource couplings.

Deliver:

- normalized mission outcome, segment outcome, failure taxonomy, objective
  result, terminal contract, resource, envelope, and numerical reports;
- catalog query for available and completed showcase/qualification packets;
- objective-to-telemetry provenance and reproduction records; and
- common summary cards for UI, batch search, and release catalogs.

Exit:

- a caller can rank or compare runs without parsing a family-specific plot;
- a scalar quality score cannot hide a missed required objective; and
- every result identifies the exact vehicle, variant, mission, fidelity,
  interface, execution binding, and evidence boundary.

### M6 — family-authoring kit

**Goal:** Make adding a vehicle to an existing family predictable and creating
a new family deliberate.

Deliver:

- authoring templates for source manifest, fidelity declaration,
  `ParameterSpec`, interface binding, mission/capability adapter, and witnesses;
- generated readiness/worklist diagnostics;
- synthetic conformance fixtures for four fidelity tiers, status/action
  projection, parameter resolution, and mission graph compilation; and
- a new-family decision record requiring topology, resource, start/terminal,
  control, objective, and evidence contracts.

**Progress:** `taoryx vehicle authoring-template <vehicle> <mission> <fidelity>`
now exports the first no-invented-default kit. It turns the resolved registry
into a schema-plus-checklist artifact containing every initialization and
segment input, topology/value-space contract, repeated-instance requirement,
control intent, declared graph, selected interface, variant binding, and
runtime/evidence worklist. An absent default is explicitly null. A
source-backed declared recommendation may be displayed, but it must be
explicitly confirmed by an author rather than inserted by the compiler; the
kit's `input_completion` worklist makes that decision visible per
initialization and segment occurrence. The selected kit now includes the exact
batch/episode execution endpoints, their execution modes, factory IDs, and
claim boundaries, and only commands whose endpoint/evidence prerequisites are
met.
This prevents a planned or batch-only tier from being presented as though it
were interactively runnable or parity-verified.

The existing fail-closed four-tier
intake compiler is now also a Product 3 command:

```bash
taoryx vehicle intake existing-family \
  --family-id example_uav \
  --physical-family powered_fixed_wing \
  --strategy-id powered_fixed_wing.v1 \
  --mission-overlay fixed_wing_racetrack \
  --adapter-id taoryx.fixed_wing.example_uav.v1

taoryx vehicle intake new-topology \
  --family-id example_tailsitter \
  --physical-family tailsitter_vtol \
  --strategy-id tailsitter_transition_vtol.v1 \
  --topology-summary "Vertical propeller takeoff followed by wing-borne transition."
```

The first form refuses an ambiguous topology choice; the second creates only
a strategy-definition scaffold. Neither command creates a registry entry or
claims source mappings, trim, controls, or qualification. Source-manifest
scaffolding, synthetic four-tier witness plans, and a new-family decision
record now accompany the new-topology scaffold. The source template contains
no assets or hashes until supplied; the witness plan describes only generic
interface/committed-truth/graph/binding checks; and the decision record keeps
topology, resource, start/terminal, authority, timing, objective, provenance,
and nonclaim decisions explicitly unresolved until reviewed.

Both intake outputs now include an `interface_contract_required` checklist and
the same four-tier synthetic conformance-witness plan. The generator validates
its own artifact before exporting it: canonical tier order and realization,
interface/timing checklist, witness coverage, and the appropriate
existing-family registry or new-topology decision scaffolding must all be
present. The emitted `intake_validation` remains deliberately narrow—it proves
the work package is structurally complete, not that any source data, plant
adapter, trim, controller, or qualification result is valid.

The checklist requires the prospective family to declare parameter mutability and
value-space topology; authority and effector boundaries; canonical
status/resource/observation channels; committed-truth and sensor timing; and
the exact execution mode, batch/episode/replay-or-parity disposition, and
batch action-trace disposition. This keeps a vehicle
from becoming merely a source/plant import that later needs an ad hoc
composition, AI, or telemetry wrapper. The checklist is declarative only: it
does not invent missing values, bind runtime code, or promote a fidelity.
The intake packet exposes the canonical execution-mode vocabulary and its
intrinsic endpoint constraints directly, so a contributor does not reverse
engineer a permissible mode from an existing vehicle's YAML binding.

Every intake now also carries `mission_binding_contract_required`. A fixed-wing
family may retain the legacy racetrack binding, but any family can instead use
`taoryx.mission-binding/v1alpha1` with `kind: semantic_composition`. That form
must distinguish the source-family identity from the public composition-family
identity, pin a public mission ID and exact phase order, and retain one or more
checked-in composition witnesses with their expected semantic-preflight
dispositions. It proves only that the semantic mission still compiles and
lowers as declared; source runtime, controller, truth-objective, terminal, and
qualification gates remain independent. The HL-20 glide intent is the first
non-racetrack witness of this reusable authoring path.

The source-family work is now discoverable through the same Product 3 CLI,
rather than requiring a contributor to know a developer-only validation script:

```bash
taoryx vehicle integration readiness reference_f16_s119
taoryx vehicle integration pipeline reference_f16_s119
taoryx vehicle integration pipeline reference_hl20_mod_k --allow-blocked
```

The first command checks only manifest/profile readiness. The second exposes
the ordered intake, conventions, plant, effectivity, trim, operating-point,
fidelity, controller, and mission gates already recorded for a source family.
It remains deliberately non-promotional: a report can identify a passed plant
or trim gate while still reporting a blocked mission path. The command can
write a hash-bound handoff packet, but it does not execute a mission or turn
recorded integration evidence into physical-effector or family qualification.

The authoring worklist also projects the blocker list from every explicitly
planned execution binding. A tier without a public runner therefore reports
both the generic requirement for a source-owned batch factory and the exact
family-owned deficiencies already recorded by its binding. For example, the
HL-20 direct-wrench glide path names release/trim/gravity binding,
energy-glide translation, source-bounded handoff evaluation, and its declared
local-authority deficit. These are planning diagnostics, not generic fallback
permissions or evidence promotion.

Mission templates now carry an optional typed `graph_execution_extension` for
the exceptional case in which a native translator can execute more than the
template-owned success chain. It fixes the fidelity, translator identity,
supported and rejected outcomes, allowed truth-state transfer, bounded timeout
rule, and claim boundary in the public registry. The Hummingbird
timeout-to-touchdown recovery path is the first instance; all other templates
remain success-sequence-only. This removes the last family-name conditional
from graph-extension eligibility while keeping a caller-authored branch
blocked until its own family declares and implements it.

The same template now declares `semantic_translator_id` whenever a source-owned
semantic lowering exists. `preflight` compares its returned translator against
that registry value before it may report `translation_ready`; a mismatch or an
undeclared ready translator is blocked with the selected composition identity
and derived mission retained for diagnosis. This makes source-specific
lowering provenance visible to authoring clients and prevents a generic or
misrouted translator from becoming an implicit fallback.

Mission capability and semantic-lowering declarations are tier-scoped. A
template can remain discoverable across its family fidelity ladder without
silently lending a pseudo-6DOF planner or translator to a point-mass or
direct-wrench tier. The resolver requires the installed capability adapter to
match the registry declaration at the selected tier, otherwise it reports a
structured configuration error instead of selecting the nearest compatible
implementation.

The composition preflight dispatcher now selects a source-specific handler only
by this declared semantic translator ID through a validated, immutable handler
registry. It no longer branches on vehicle
family or mission name. The family handler retains its concrete route,
chronology, hover, passive-release, or local-authority validation; the shared
layer verifies graph eligibility, planner availability, declaration/handler
agreement, and fail-closed blocking for missing or mismatched handlers. This
is the reusable integration seam for the next family rather than another
central conditional.

The catalog now produces `taoryx.semantic-preflight-handler-report/v1alpha1`
through `taoryx vehicle semantic-preflight-handler-report`. It audits every
family/mission/fidelity tuple to distinguish a translator that is installed,
a capability planner waiting for a translator, an undeclared translation, and
the fail-closed error cases where a declared translator lacks an installed
handler or a declared capability adapter lacks an installed planner.
`taoryx vehicle maturity-report` embeds the same report, so Product 3 cannot
appear mature while a registry declaration selects a dead planner or semantic
dispatch path. The report remains structural: it does not compile a concrete
case, prove a planner supports it, determine capability feasibility, bind an
adapter, or promote any model or run to qualification.

The endpoint-witness matrix supplies the complementary concrete check. Every
`translation_ready` witness now retains a
`taoryx.concrete-capability-preflight/v1alpha1` projection: the exact
family-owned capability adapter, feasibility disposition, immutable
composition identity, and SHA-256 of the derived mission consumed by
preflight. The witness gate recomputes that fingerprint and rejects a bare
ready result, adapter mismatch, changed derived mission, or a capability
record attached to a different composition. `maturity-report
--check-execution-witnesses` summarizes those witnessed adapter and
feasibility dispositions separately. It is still pre-execution evidence, not
a claim that a native plant, controller, objectives, or showcase has passed.
Every current batch packet also preserves this exact preflight as
`preflight.json`. The normalized result catalog verifies its composition
identity, translator, capability adapter, feasibility class, and
derived-mission fingerprint before exposing `capability_preflight_evidence`.
Older/external packets remain explicitly `missing`; result discovery does not
invent planning provenance from a successful trajectory.
The normalized evaluation also preserves the family estimate rather than
equating `translation_ready` with `feasible`: `likely_feasible`, `unknown`,
and `likely_infeasible` retain their conservative meanings, while only a
family `certainly_infeasible` estimate maps to the public `infeasible` state.

For the reusable `powered_fixed_wing.v1` / `fixed_wing_racetrack` path, the
existing-family intake now also emits real, unfilled `ParameterSpec`-shaped
contracts for both source-owned capability inputs and author-supplied mission
intent. The records already carry units, value spaces, mathematical hard
validity, required flags, scopes, and explicit absence of source defaults;
for example, an operating-point identity is a declared finite set while a
requested bank is a bounded interval. This lets the first family intake feed
the same parameter vocabulary used later by the registry, rather than asking
the next contributor to translate a prose checklist into an ad hoc schema.
Vehicle-specific values, qualified ranges, provenance, and runtime bindings
remain authoring requirements and are not fabricated by the scaffold.

Exit:

- a new fixed-wing member requires data/capability bindings, not a copied
  route runner or controller stack;
- omissions return one actionable checklist; and
- no new physical family enters the catalog without a declared Product 3
  composition contract.

## Priority and Alpha placement

| Horizon | Product 3 work | Why it comes first |
| --- | --- | --- |
| Current / Alpha 2 closeout | M0, plus preserve existing compose-to-run witnesses. | Makes the existing product discoverable and honest. |
| Alpha 3 core | M1, M2, and M3 for the current nine-family catalog. | Removes manual parameter/waypoint tuning as the dominant integration path. |
| Alpha 3 continuation | M4 and initial M5. | Makes the product useful to AI/RL, batch search, and showcase generation. |
| Alpha 4 | M6 and a network/UI adapter if needed. | Scales authoring after the semantic and evidence contracts are stable. |

The completed reusable vertical slices now include the X8/B747 fixed-wing
path, A320 and F-16 reduced-family paths, Hummingbird pseudo-6DOF, the X-15
local direct-wrench bridge, an HL-20 subsonic local direct-wrench bridge, and
an HL-20 source-aerodynamic booster/release replay at point-mass and named
attitude-response tiers. The HL-20 replay is a separate
`hl20_source_booster_release_replay_v1` mission with fixed launch, release,
and opposing bank-schedule inputs. The local bridge is likewise separate from
both replay and glide missions. It is intentionally *not* the public
`lifting_body_glide_energy_management_v1` contract: the latter is now
semantically lowered at point-mass and pseudo-6DOF through its exact public
release/trim/opposing-bank/energy-handoff plan, but still lacks a genuine
high-altitude native runtime, trim, energy/crossrange execution, and
terminal-handoff evidence. The immediate Product 3 frontier is no longer a
generic controller or another inferred adapter: it is (1) source review that
fills bounded, provenance-bearing parameter/variant contracts for the
advertised vehicle knobs; (2) family-owned semantic lowerings where a
capability adapter is intentionally `translator_pending`; and (3) wider
retained-result/robustness packets over the already
verified composition, parity, and evaluation seams. Each item must preserve
the current fail-closed status until the family supplies the required plant,
data, or evidence; Product 3 must not fill those gaps with a nearby vehicle,
direct wrench, or response-law fallback.

### Source-owned promotion queue

The next promotions are ordered by reusable evidence rather than by the number
of advertised family names.

1. **HL-20 glide-energy point-mass/pseudo path:** retain the existing semantic
   lowering as the contract, then add a high-altitude source-aerodynamic
   runtime that binds release, trim/gravity, opposing-bank crossrange, and the
   terminal energy handoff. Its current source-scheduled booster/release replay
   remains a separate, runnable witness.
2. **X-15 high-energy direct-wrench path:** bind release-to-powered-to-coast
   phase state, source-bounded terminal objectives, and a phase-scheduled
   direct-wrench controller. The existing local screen remains control-screen
   evidence only, not a flight executor.
3. **A320 and NESC wrench tiers:** do not synthesize them from the existing
   performance/replay paths. Promote only after a source-owned force/moment or
   gimbal plant, a physically meaningful trim condition, and requested versus
   achieved wrench telemetry exist.
4. **Surface-allocated tiers:** require each family’s actual effector map,
   bounded allocation, actuator dynamics, and nonlinear validation. They are
   never an automatic promotion from a direct-wrench bridge.

The registry worklist exposes this queue at the exact mission-tier level via
`planned_execution_blockers`; the ordered list above supplies planning context
but does not loosen any binding or evidence requirement.
`taoryx vehicle maturity-report` also aggregates the exact planned-binding
count and blocker vocabulary so progress can be measured by retired
family-owned prerequisites rather than by a misleading count of promoted
vehicle names.

### Variant admission and integration notes

Runtime variation is now treated as a separate integration gate, not as an
optional YAML override. `taoryx vehicle authoring <vehicle>` provides a
`variant_worklist` with three honest states:

| State | Meaning | Required next action |
| --- | --- | --- |
| `runnable` | An exact native runtime input consumes the modifier and the declared committed-status relation has an execution witness. | Preserve the witness and retrim/requalify after every declared invalidation. |
| `planned` | A semantic modifier has been declared but its plant/evidence path is incomplete. | Bind the runtime input, derive coupled state/resources, emit committed truth evidence, and add the witness. |
| `not_declared` | This family has no safe public modifier yet. | Keep the model immutable; do not repurpose an initialization input or mutate a source trace. |

The current catalog intentionally has only two runnable variants: A320
operating mass and Hummingbird grounded mass. Both prove exact native-input
consumption and `resources.mass.total` at committed boundaries. They do not
prove fuel loading, inertia, payload distribution, endurance, or a qualified
load envelope. X8, B747, F-16, X-15, HL-20, NESC, and passive tumbling entries
remain `not_declared`; their current plants either use pinned source data or
lack the coupled derivations needed for a truthful modifier. In particular,
the NESC replay's changing mass is source history, not a supported request to
change propellant loading or stage mass.

This is a deliberate integration lesson: a parameter becomes public only when
the same runtime that produces the trajectory consumes it and reports the
resulting coupled state. The Product 3 maturity target is broader *source-
owned* variation, not a larger list of knobs.

The same authoring worklist now exposes each declared
`fidelity_promotion_blockers` before its endpoint blockers. This produces a
usable ordering for integration: retire the family’s stated model/trim/control
gate first, then bind the source-owned batch or episode factory. A planned
factory alone cannot resolve a declared lack of source trim acceptance,
nonlinear direct-wrench mission behavior, or physical allocation evidence.

`taoryx vehicle maturity-report` independently cross-checks the authoring
variant-admission counts against topology's `runtime_backed_variant_count`.
The report fails if these projections drift. This is a catalog consistency
gate—not proof that a mass, fuel, or payload modifier is physically qualified.
It also aggregates declared `fidelity_promotion_blocker_counts` separately
from planned endpoint blockers, allowing progress to be measured by retiring
source/model gates rather than merely by adding runnable factory names.

## Product 3 next-phase work packages

The M0–M6 milestones above describe capabilities. The work packages below are
the execution units used by issues, commits, validation reports, and release
reviews. A package is not complete because a command exists: its deliverables,
verification evidence, and explicit nonclaims must all be present.

### Package register

| ID | Owner surface | Objective | Depends on | Primary deliverables | Exit evidence |
| --- | --- | --- | --- | --- | --- |
| `P3-RUN-01` | Runtime/family adapters | Make each advertised plant construction source-owned and reusable by runtime, tools, and witnesses. | M0 | Source-owned factory; registry binding; generic operation probe; developer-tool reuse; no-fallback test; claim/nonclaim record. | The declared adapter builds and passes the operations required by its tier; no runtime module imports a developer-only plant; the maturity report names the exact remaining blocker. |
| `P3-RUN-02` | Integration/verification | Turn missing vehicle data and fidelity prerequisites into a generated, fail-closed checklist. | `P3-RUN-01` | Per-family/fidelity data matrix; provenance/hash record; trim/effectivity/resource/frame/timing requirements; automatic-lowering decision; actionable diagnostics. | All nine families have explicit `passed`, `development`, `planned`, `not_applicable`, or `blocked` records with no silent unknowns; F-16, HL-20, A320, and NESC are retained as onboarding regressions. |
| `P3-COMP-01` | Composition/mission graph | Make mission templates reusable across vehicles while keeping objective truth independent from controller transitions. | M0, M2, M3 | Versioned mission templates; segment/objective/terminal schemas; graph transitions; capability and timing preflight; source-owned batch/episode binding. | Every advertised endpoint has an immutable composition identity, preflight result, and explicit branch/timeout behavior; no controller transition can certify a missed objective. |
| `P3-COMP-02` | Variants/resources | Add only coupled, source-owned vehicle variation that the runtime actually consumes. | `P3-RUN-01`, `P3-RUN-02` | Variant schema; derivation/invalidation graph; resource and mass coupling; fingerprints; re-trim/requalification declarations; rejection/projection evidence. | No accepted variant has inconsistent mass, resource, inertia, or units; every applied value appears in runtime provenance and committed status; unsupported families remain undiscoverable as executable variants. |
| `P3-EXEC-01` | Execution/evaluation | Make batch, episode, status, action, and evaluation contracts uniform and reproducible. | M4, `P3-RUN-01` | Action/status trace contract; parity registry; normalized result/evaluation; resource ledger; graph execution packet; release/reproduction manifest. | Current advertised endpoints retain their registered parity witnesses; committed timestamps agree; new endpoints cannot be promoted without the same witness and artifact set. |
| `P3-AUTH-01` | Product 3 authoring/catalog | Give users one discoverable path from vehicle metadata and bounded parameters to a compiled trajectory. | `P3-COMP-01`, `P3-COMP-02`, `P3-EXEC-01` | Catalog/describe/schema/interface/endpoints projections; authoring kit; parameter and value-space topology; compose/lower/materialize/run/preflight surfaces; blocker report. | A user can query a vehicle, select valid parameters and segments, compile a trajectory without editing code, and receive either an executable binding or a precise blocker. |

### Dependency and release sequence

```text
P3-RUN-02 ─┐
           ├──> P3-COMP-02 ─┐
P3-RUN-01 ─┼──> P3-EXEC-01 ─┼──> P3-AUTH-01
           └──> P3-COMP-01 ─┘
```

The sequence is deliberately incremental:

1. **R0 — Baseline contract:** preserve the current catalog, topology,
   committed-truth, graph, parity, and fail-closed evidence. This is the
   starting baseline, not a new qualification claim.
2. **R1 — Runtime ownership:** the X8, B747, Hummingbird, and F-16
   source-backed witnesses now use runtime-owned factories and generic
   operation probes. The next tranche is not another construction fork: it is
   extending the same data/trim/mission evidence to additional operating
   points. This does not promote any local witness to family or envelope
   qualification.
3. **R2 — Automatic integration:** execute `P3-RUN-02` through the F-16,
   HL-20, A320, and NESC pilots. The expected output is a complete worklist,
   including blocked and not-applicable tiers, rather than forced promotion.
4. **R3 — Composition breadth:** complete `P3-COMP-01` and `P3-COMP-02` for
   the reusable powered-fixed-wing racetrack, Hummingbird graph, and one
   source-owned bounded variant. Expand only after the runtime consumes the
   selected modifier or graph branch.
5. **R4 — Product 3 maturity gate:** complete `P3-EXEC-01` and `P3-AUTH-01`,
   then publish the catalog maturity matrix, endpoint/parity matrix, variant
   matrix, and exact family/tier blockers as one release packet.

### Per-family deliverable queue

The queue below is the minimum next artifact for each current family. It is
not a promise that every family will receive every fidelity tier.

| Family | Current useful evidence | Next concrete deliverable | Promotion boundary |
| --- | --- | --- | --- |
| X8 | Source-table local direct-wrench and surface-allocation witnesses; fixed-wing composition seam. | Runtime-owned racetrack plant/controller packet using actual elevons and throttle, with requested/achieved wrench and objective evidence. | Do not call the local witness family-qualified or schedule-qualified. |
| B747 | Source-table local direct-wrench and surface-allocation witnesses; transport composition path. | Scaled transport racetrack packet at declared operating points, with actual surface allocation, trim residuals, and schedule-transition evidence. | Cruise/local evidence does not imply takeoff, landing, or full-envelope qualification. |
| Hummingbird | Pseudo-6DOF route/graph, grounded-mass variant, and runtime-owned four-rotor local plant with operation probes. | Native mission/controller/resource packet with yaw authority, altitude gates, individual effector telemetry, contact, and post-touchdown settle. | The local hover plant proves construction, trim, linearization, and allocation seams only; it does not prove waypoint, battery, or envelope qualification. |
| F-16 | Source replay, local trim/LQR/surface path, reduction scaffolding, and runtime-owned first-operating-point factory with probes. | Remaining six-point residual/schedule continuation and fixed-wing racetrack reduction comparison using the same runtime-owned source construction. | Local T5 evidence is not family or envelope qualification. |
| X-15 | Source/replay and local direct-wrench bridge evidence. | Phase-scheduled powered/coast/atmospheric composition with explicit release, cutoff, energy, and handoff artifacts. | Direct-wrench evidence does not imply physical surface or full mission qualification. |
| HL-20 | Source replay and semantic point-mass/pseudo path. | Source-owned glide runtime with trim/gravity, opposing-bank crossrange, energy corridor, and terminal handoff. | No landing/contact or native high-altitude claim until those gates pass. |
| A320 | 3DOF and pseudo mission pilot; mass variant witness. | Calibrated named attitude-response model across additional operating points with retained source/overlay boundary. | Composite pseudo effectivity is development evidence, not physical actuator qualification. |
| NESC rocket | 3DOF/source replay and variable-mass lineage. | Staged pseudo attitude/gimbal/separation contract, or an explicit not-applicable record where no attitude reduction is supportable. | A source replay with no controls is not a controlled pseudo-6DOF result. |
| Passive/tumbling body | Passive 3DOF projected-area baseline and rigid-body rotational path. | Shared passive-family packet: average/steady projected-area policy for 3DOF, native rigid-body reuse for pseudo-6DOF, and rotational invariants. | No controllability or actuator tier is advertised; 3DOF cannot prove tumble. |

### Package completion checklist

Each package is complete only when its release review can point to all of the
following committed artifacts:

- a versioned manifest naming family, vehicle, fidelity, plant, interface,
  mission, and evidence class;
- a machine-readable input/data requirement and provenance report;
- a deterministic diagnostic or operation probe with a retained result;
- an independent objective/evaluation report, when a mission is involved;
- a reproducibility command and hashes for all local source inputs;
- a negative-control or fail-closed test for the principal failure mode; and
- an explicit list of what the package does **not** prove.

The package owner must update the maturity report and the integration worklist
in the same change. A green unit test without the corresponding registry,
manifest, and documentation projection is incomplete Product 3 work.

## Product 3 success measures

Track these measures instead of counting vehicle names or rendered plots:

- time and handwritten code required to add a vehicle to an existing family;
- percentage of vehicle/configuration/mission choices discoverable from the
  public descriptor alone;
- percentage of parameters with bounds, defaults, provenance, and derivation
  effects;
- number of family missions using shared objective and capability adapters;
- number of failures diagnosed at catalog/variant/mission preflight rather
  than after tuning a simulation; and
- percentage of public results containing complete identity, objective,
  evidence, and reproduction records.

## Non-goals

- A generic REST service before the CLI/Python semantic artifacts are stable.
- Arbitrary direct mutation of source tables, inertia, forces, moments, or
  model state through “parameters.”
- One universal controller, waypoint law, or resource model for all families.
- Promoting a direct-wrench bridge, pseudo-6DOF response, or nominal mission
  solely because Product 3 can construct and execute it.

Product 3 is successful when it makes the right path easy: select declared
meaningful parameters and objectives, obtain a truthful answer or a precise
blocker, and retain all of the source and evidence context needed to reproduce
the result.
