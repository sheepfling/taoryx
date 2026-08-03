# Horizontal vehicle-integration plan

Status: Active Alpha 3 execution plan  
Owner: Taoryx runtime and vehicle-family integration  
Scope: All supported families and all four advertised realization tiers

## Execution status

HVI-1 is complete for the current Alpha 3 surface. The repository now has a
compatibility-safe canonical contract module at
`src/taoryx/fidelity_contracts.py` defining the four-tier order, parent
relationships, control realizations, qualified evidence statuses, and explicit
normalization rules for the legacy `rigid_body_6dof` spelling. The
pseudo-profile catalog, controller provenance, trajectory contracts, and
readiness profile order consume that vocabulary where their existing artifact
compatibility permits it. Remaining legacy artifact labels are tracked for the
migration stage; they are not silently rewritten.

HVI-2 has its first executable slice. The nine-family
`verification/horizontal_fidelity_registry.yaml` declares the same four tier
slots, physical-family overlay, adapter identity, and lowering policy for every
current Alpha 3 family. `taoryx.horizontal_fidelity` validates those entries
against the canonical pseudo/direct/surface catalog, and
`tools/validate_horizontal_fidelity.py` emits the checked manifest at
`verification/alpha3_horizontal_fidelity/manifest.json`. Automatic lowering is
evidence-only: catalog status can describe a profile, but it cannot authorize
selection without a checked passing artifact. Passive tumbling explicitly has
no actuator tiers.

HVI-3 is now the first executable façade slice: adapt the existing X8,
Hummingbird, F-16, X-15, and tumbling implementations to one
capability/result façade, then use those as the conformance witnesses before
migrating the remaining families. The façade is executable in `src/taoryx/family_adapter.py`: it
wraps the existing numerical plant contract, publishes ordered state/control/
resource channels, and reports every operation as available, not applicable,
not available, or planned. Passive bodies can therefore use the same seam
without acquiring fabricated controls.

The registry-backed witness slice is now executable through
`tools/validate_family_adapter_registry.py`. It runs the same conformance
validator against the X8 table-coordinate plant, B747 condition-3 surface
plant, Hummingbird individual-rotor plant, F-16 source-backed local plant,
the OpenAP A320 point-mass and pseudo-6DOF products, the HL-20 source-load
surface-allocation witness, the X-15 source direct-wrench bridge, and passive
tumbling topology, plus NESC source replay. The report now covers all nine family bindings; planned
physical tiers remain visible in each registration and never fall back to
another vehicle's plant.
The report also emits a declared-tier matrix for available registrations. Each
executable witness is constructed and probed independently for every tier it
claims, so direct-wrench and surface-allocation evidence cannot be conflated by
a representative default-tier check; planned bindings remain visible in the
compact family report.
Each four-tier slot now also carries a typed promotion gate: declared maturity,
required adapter operations, and named blockers. The registry report evaluates
those requirements against the actual operation probes and keeps declared
development/planned status separate from a failed probe. This makes a direct
wrench bridge visibly different from a surface-allocation promotion without
making either one disappear from the common matrix.

The current HVI-5 execution slice is also complete at the contract boundary.
`showcase/adapter_binding.py` constructs a `FidelityShowcaseRealization` from
the adapter descriptor and capability report, copies the adapter state schema,
and exposes physical effectors only for the canonical surface-allocation tier.
Showcase-required operations are preflighted before a claim is constructed.
Legacy showcase records using `rigid_body_6dof` are normalized only when their
control realization explicitly identifies direct wrench or surface allocation;
ambiguous records fail with an actionable diagnostic. The DAVE-ML showcase
catalog was migrated accordingly: F-16 and HL-20 evidence boards are explicit
direct-wrench records, while NESC replay boards use the supported pseudo/replay
realization rather than implying a controlled rigid-body tier.

The next HVI-2/HVI-4 slice is now implemented for source-family selection.
`runtime_fidelity_for` is the single mapping from a canonical advertised tier
to the native runtime mode. `select_validated_canonical_fidelity` preserves the
four-tier distinction during evidence-based lowering: direct-wrench and
surface-allocation profiles are separate candidates even though both execute
the native rigid-body integrator. Legacy profiles without a declared control
realization are visible to compatibility callers but fail closed for canonical
selection. The F-16, HL-20, and NESC source manifests now declare their control
realization explicitly, and readiness resolves rigid tiers by that metadata
before using profile-name compatibility matching.

The manifest/lowering join is now executable for all nine families. The
`UnifiedFamilyManifestCatalog` joins the horizontal registry, pseudo-profile
catalog, legacy vehicle definitions, and source-backed family manifests when
present, and emits `verification/alpha3_horizontal_fidelity/unified_manifest.json`.
It reports missing authority or mismatched tier bindings before a runtime
adapter is selected. The canonical lowering algorithm in
`fidelity_lowering.py` is shared by pseudo-profile lowering and reference-
family selection, so automatic lowering cannot select a surface profile
without its direct-wrench parent or a pseudo profile without its 3-DOF parent.
The legacy selector remains only as an explicitly compatible boundary.

The shared lowerer now accepts an optional adapter-operation status map. A
candidate may declare required operations such as `state_derivative`, `trim`,
`effectiveness`, or `allocate`; those operations must be reported as
`pass`/`available` before that tier can be selected. Profile evidence and
adapter availability remain separate inputs, but a caller that supplies both
can no longer select an evidence-qualified tier whose executable façade is
missing the requested operation. Existing profile-only callers remain
backward-compatible.

The first source-trim consistency slice is now also in place for HL-20. Both
rigid tiers are built through the same tier-aware factory and expose the same
typed `trim_fragment` result for the verified Mach-1 scalar pitch-channel
source evidence. The operation is deliberately not `trim`: the fragment has
one `alpha` unknown, no physical controls, and an explicit nonclaim that it is
not a full six-degree-of-freedom equilibrium. This makes the partial evidence
portable across tiers without allowing the direct-wrench bridge or the
logical-surface overlay to promote themselves to controller-ready trim.

The next horizontal readiness slice is executable through
`src/taoryx/horizontal_readiness.py` and
`tools/validate_horizontal_readiness.py`. It joins the nine-family horizontal
registry, the source or legacy vehicle authority, and the latest adapter probe
artifact into one 36-slot matrix. Each slot reports its declared maturity,
data source/status, required adapter operations, operation results, and named
blockers. This is deliberately a pre-simulation worklist: `probe_ready` means
the declared data and adapter operations are available, not that the family
mission or full envelope is qualified. Passive tumbling remains explicitly
`not_applicable`, while planned A320 and NESC rigid tiers remain planned rather
than inheriting a neighboring aircraft's data.

The next horizontal execution slice closes the hand-tuning gap through
`verification/family_integration_strategies.yaml`,
`src/taoryx/family_strategy.py`, and
`tools/compile_family_strategy_worklists.py`.  A strategy is selected by
physical topology, rather than vehicle name, and declares the required data,
adapter operations, calibration mode, and non-negotiable diagnostic order for
each canonical tier.  The compiler joins that declaration with the readiness
matrix and produces `verification/family_strategy_worklists.json`:

```bash
.venv/bin/python tools/compile_family_strategy_worklists.py --check
.venv/bin/python tools/dev.py family-strategy-worklists
```

The resulting worklist makes the next *admissible* action explicit.  A fixed
wing vehicle follows source audit → trim grid → derivative consistency →
authority preflight → operating-point campaign → authority envelope, while a multirotor follows
hover trim → rotor authority → motor-limit → yaw/translation → landing-settle
checks.  Rocket-aircraft and staged-rocket strategies reset or schedule that
sequence at declared phase boundaries; passive tumbling bodies explicitly have
no controller or allocator work.  The worklist never relabels a missing
effector, rank-deficient allocation, source-table exit, absent gimbal data, or
planned plant as a gain-tuning issue.  It is a process/evidence planner, not a
qualification claim and not an automatic controller.

The generic `trim_linearize_and_tune` seam now also accepts a family-provided
`AuthorityPreflightReport` between finite-difference linearization and its LQR
profile sweep. The preflight can report controlled-axis rank, conditioning,
limits, and a named structural blocker. A blocked result returns the trimmed
plant and derivative evidence but **does not synthesize any gain candidates**.
This turns the X8 two-elevon/yaw issue into reusable behavior: preserve the
diagnostic, fix the topology or relax the unsupported mission requirement, and
do not spend time retuning a controller that cannot create missing authority.

`LinearAuthorityRequirement` now makes that gate executable.  It evaluates
the controllability matrix of the actual declared local input space, records
rank, condition number, and the unreachable fraction of every required state,
then returns either a passed `AuthorityPreflightReport` or stable blockers such
as `uncontrolled_required_state:yaw_rate`.  The generic trim-to-tune pipeline
uses it before creating any LQR candidate; the X8 and B747 source-table
physical-wrench validation scripts use the same gate before deriving a
controller; and the four retained direct-wrench screens load their required
state set from `verification/reference_tuning_bindings.yaml`.  A passed screen
still means only that its declared input abstraction is locally controllable;
it does not promote direct wrench or lower-tier semantic inputs to physical
effector evidence.

### Reusable reduced-tier plant bridge

The lower tiers now use the same disciplined integration seam instead of
being treated as informal mission-only scripts. `ReducedOrderControlPlant`
carries the stable state/control ordering, family trim provider, nonlinear
derivative, and finite-difference provenance contract. It deliberately
refuses `effectiveness` and `allocate`: a semantic force input or response-law
command must never be mistaken for a physical surface or rotor command.

The first two migrations reuse existing models rather than creating another
vehicle-specific controller path:

- Hummingbird point mass supplies an analytic aggregate-thrust hover trim;
  its named pseudo-6DOF plant supplies the continuous right-hand side of the
  bounded attitude-response law used by the runtime. Both remain
  aggregate-thrust models with no rotor-allocation claim.
- F-16 point mass and pseudo-6DOF project the verified source trim into their
  reduced state schemas, then derive matrices through existing source-force
  and source-calibrated response laws. They remain lower-tier force/response
  models; physical surface allocation is still the distinct rigid-body tier.

This reduced the generated 36-slot strategy worklist from fourteen to ten
missing adapter-operation slots. The remaining slots are real model work, not
pipeline work: X8, B747, X-15, and HL-20 still need declared point/pseudo
plants, while tumbling bodies still need participating passive derivatives.
Hummingbird's lower tiers remain `strategy_development` until their
data-readiness blockers are cleared; that status is intentionally different
from a missing adapter.

### Repeatable vehicle and family intake

The intended fast path for a new vehicle is now:

```text
source lock + frame/unit contract
  -> choose an existing physical-family strategy
  -> declare four canonical tier bindings and nonclaims
  -> bind the actual lower or rigid-body plant to the common adapter seam
  -> run trim/derivative/authority preflight before gain search
  -> compile the per-tier worklist
  -> execute the next admissible probe and publish its evidence boundary
```

An existing-family vehicle should normally add data, operating points,
family-specific mappings, and a thin plant binding—not a new controller
architecture or bespoke tuning loop. A new physical family is justified only
when its topology changes non-tunable constraints or the ordered calibration
process. In that case it adds one strategy record with four tier recipes, data
requirements, and topology blockers; the manifest, adapter, worklist,
readiness, lowering, and showcase contracts remain the same.

Automation can sweep trim nodes, compare derivative steps, estimate authority,
reject rank loss, scale candidate controllers, and run fixed probes. It cannot
manufacture missing force/moment data, actuator limits, source trim, or a
control axis. Those cases stop at intake with a named blocker rather than
consuming days of manual gain tuning.

### No-manual-gain-search operating-point campaigns

The generic strategy worklist now has a matching numerical control-design
seam in `taoryx.tuning_campaign`.  A `TuningCampaign` is a small, versioned
set of family-declared operating points.  At each node the common runner does
exactly this:

```text
family trim target + initial estimate
  -> actual adapter trim
  -> two-step finite-difference consistency check
  -> declared-state controllability / authority check
  -> normalized scaled-LQR candidate sweep
  -> candidate-ready result or named first blocker
```

The family still owns the meaningful inputs: trim target, source operating
point, physical state/control scales, and which axes it needs to control.
Taoryx owns the order of diagnostics and refuses to try gains when trim,
derivatives, or authority are inadequate.  This is the practical policy for
reducing tuning time: no human gain adjustment begins until a campaign node
has reached `candidate_ready`; then nonlinear, allocation, and mission tests
decide whether the local candidate survives beyond the design screen.

Nested loops are explicit.  A campaign node may identify a closed inner-loop
state/control subset, such as attitude and rates inside a multirotor position
controller.  The runner measures the parent model's omitted-state coupling
into that subset and rejects the projection when it exceeds the declared
tolerance.  That makes a legitimate inner-loop reduction reusable while
preventing an integration from hiding coupled dynamics merely to obtain an
easy LQR result.

The initial runtime witnesses are intentionally from two different existing
families: Hummingbird uses a pseudo-6DOF hover-attitude inner loop, and A320
uses a pseudo-6DOF cruise attitude-response inner loop under the shared
powered-fixed-wing strategy.  Their campaign bindings live next to the real
family adapters, not inside a one-off showcase script.  Both are
candidate-design screens only; neither manufactures a rotor or conventional
surface allocation claim.

Run or check their common artifact with:

```bash
.venv/bin/python tools/validate_reduced_tuning_campaigns.py
.venv/bin/python tools/validate_reduced_tuning_campaigns.py --check
```

The campaign is deliberately not a shortcut around physics.  It returns only
candidate-design evidence.  A lower-tier response law still cannot claim a
surface/rotor allocator, and a direct-wrench result still cannot claim that
physical effectors could realize the requested wrench.

The provider-neutral vehicle-integration pipeline now consumes this contract
for declared LQR profiles.  A controller profile names the required state set
next to its plant-derived linearization artifact; the pipeline runs the common
authority preflight and includes the resulting rank, state-reachability, and
blocker record in its controller-stage metrics.  The F-16 reference family is
the first intake witness: both its local trim-hold and physical-wrench
profiles prove six-state local controllability from the pinned source A/B
artifact, while their independent evidence tiers and actuator nonclaims remain
unchanged.

The strategy schema enforces the same discipline for future families: any
tier recipe that declares `linearize` must include `authority_preflight` in
its ordered stages. A new family cannot create a trim/linearize/LQR recipe
that silently jumps past the authority decision.

This is the practical new-family intake seam.  Adding a vehicle to an existing
family means selecting the compatible strategy, declaring its four registry
slots, providing the named data and adapter operations, then executing the
generated worklist.  Creating a new family means defining one new topology
strategy with all four tier recipes and its non-tunable blockers; the common
worklist compiler, readiness checks, and fidelity-lowering path stay unchanged.

The same distinction is now executable before any source files are edited:

```bash
# Existing topology: emits four tier slots, adapter/data requirements,
# non-tunable blockers, and operating-point campaign requirements.
.venv/bin/python tools/plan_vehicle_integration_intake.py \
  --mode existing-family \
  --family-id example_uav \
  --physical-family powered_fixed_wing \
  --strategy-id powered_fixed_wing.v1 \
  --mission-overlay fixed_wing_racetrack \
  --adapter-id taoryx.fixed_wing.example_uav.v1

# New topology: emits the strategy-authoring decisions without inventing a
# force model, actuator map, or qualification claim.
.venv/bin/python tools/plan_vehicle_integration_intake.py \
  --mode new-topology \
  --family-id example_tailsitter \
  --physical-family tailsitter_vtol \
  --strategy-id tailsitter_transition_vtol.v1 \
  --topology-summary 'Vertical propeller takeoff followed by wing-borne transition.'
```

When a physical-family label matches more than one strategy, the command
fails closed and lists the choices.  In particular, a rocket aircraft must
not silently inherit the air-breathing fixed-wing tuning sequence merely
because both have wings and propulsion.

## Objective

Make adding a vehicle primarily a data-and-adapter task. Family-specific physics,
source conversions, and effectivity models remain specialized, but the process
for declaring, trimming, controlling, evaluating, lowering, and showcasing a
vehicle must be the same across families.

The governing rule is:

```text
family plant adapter
    + one canonical fidelity contract
    + one canonical evidence manifest
    + one common validation pipeline
    -> reproducible family/tier result
```

This plan standardizes process and evidence. It does not make a pseudo-6DOF
model claim physical moments, or make a direct-wrench bridge claim physical
effectors.

## Canonical fidelity ladder

The public tier identifiers are ordered and explicit:

| Tier | Realization | Common claim ceiling |
|---|---|---|
| `point_mass_3dof` | Translational force/resource model | Center-of-mass translation, energy, range, and declared resources |
| `pseudo_6dof` | Translational model plus named attitude/rate response law | The parent 3DOF behavior plus the declared response law, lag, limits, and saturation |
| `rigid_body_6dof_direct_wrench` | Newton–Euler integration with bounded generalized force/moment | Coupled translation and rotation from the declared loads and generalized wrench |
| `rigid_body_6dof_surface_allocated` | Desired wrench mapped through physical effectors and actuator dynamics | Direct-wrench behavior plus declared effector allocation, limits, and achieved response |

`rigid_body_6dof` is a legacy compatibility spelling and must not be used in
new manifests without an explicit `control_realization` that disambiguates
direct wrench from physical allocation.

The passive tumbling-body family uses the same state, resource, event, and
artifact contracts. Its controller and allocator stages are explicitly
`not_applicable`; its rigid-body tier is the uncontrolled plant, not a direct-
wrench controller.

## Common integration façade

Each family supplies one plant adapter. Generic tier wrappers and validators
consume that adapter through the following capability surface:

```text
FamilyAdapter
├── describe()
├── state_schema()
├── control_schema()
├── resource_schema()
├── state_derivative(state, effectors, environment)
├── trim(trim_spec)
├── trim_fragment(request)
├── linearize(trim, options)
├── effectiveness(state, effectors)
├── allocate(state, desired_wrench, previous_effectors, dt)
├── resource_rates(state, commands)
├── observe(state, controls, resources, events)
├── replay(request)
└── capability_report()
```

Unsupported operations return a typed `not_applicable` or `not_available`
capability result with a reason. They must not be represented by omitted
fields, silent direct-moment injection, or a family-specific exception path.
Provider-backed partial plants may expose only the operations supported by
their source data—for example, an aerodynamic derivative plus local surface
effectivity and allocation before a defensible trim exists. When a source
contains a bounded scalar or channel-specific equilibrium, it may expose
`trim_fragment` as a typed evidence record. `trim_fragment` is deliberately
not `trim`: it cannot satisfy a full-state, physical-effector trim gate or
authorize controller synthesis. The common façade keeps both operations
independently callable; this is development evidence, not automatic promotion
to closed-loop qualification.

The existing generic components are the implementation basis:

- `vehicle_integration_readiness.py` supplies manifest-level readiness.
- `vehicle_trim_orchestration.py` supplies declarative trim worklists.
- `control_allocation.py` supplies plant linearization, effectivity, and
  constrained allocation contracts.
- `generic_tuning.py` supplies repeatable scaled-LQR tuning.
- `pseudo6dof_profiles.py` supplies response-law and direct-wrench evidence.
- `showcase/contracts.py` supplies common mission and artifact semantics.

The work is to make these the required seam rather than adding another
family-specific implementation beside them.

## Existing-family and new-topology intake

The family strategy catalog now has a direct intake compiler. It makes the
distinction that prevents most wasted tuning effort:

- An **existing-family** intake selects one explicit topology strategy and
  emits its four-tier data contract, required adapter operations, non-tunable
  blockers, promotion placeholders, and whether the tier requires the
  standard operating-point campaign.
- A **new-topology** intake emits a strategy-authoring scaffold for every
  canonical tier. It deliberately makes no model, data, or control claim.

For example:

```text
PYTHONPATH=src .venv/bin/python tools/plan_vehicle_integration_intake.py \
  --mode existing-family \
  --family-id example_uav \
  --physical-family powered_fixed_wing \
  --strategy-id powered_fixed_wing.v1 \
  --mission-overlay fixed_wing_racetrack \
  --adapter-id taoryx.fixed_wing.example_uav.v1

PYTHONPATH=src .venv/bin/python tools/plan_vehicle_integration_intake.py \
  --mode new-topology \
  --family-id example_tailsitter \
  --physical-family tailsitter_vtol \
  --strategy-id tailsitter_transition_vtol.v1 \
  --topology-summary "Vertical propeller takeoff followed by wing-borne transition."
```

The existing-family command refuses ambiguity. For instance,
`powered_fixed_wing` alone may describe either the air-breathing fixed-wing
strategy or a rocket-aircraft strategy; a caller must select one explicitly.
That is a source/model decision, not a tuning variable. The output is an
intake blueprint only; it does not synthesize source mappings, trim evidence,
or control authority.

## Canonical family manifest

Every family must expose one manifest with this shape:

```yaml
schema: taoryx.vehicle-family/v1alpha1
family_id: <stable-family-id>
physical_family: <powered_fixed_wing | multirotor | rocket_plane | ...>
variant_id: <source-or-surrogate-variant>
source: {}
frames: {}
mass_properties: {}
environment: {}
resources: {}
fidelity_profiles:
  point_mass_3dof: {}
  pseudo_6dof: {}
  rigid_body_6dof_direct_wrench: {}
  rigid_body_6dof_surface_allocated: {}
trim: {}
linearization: {}
controller: {}
allocation: {}
mission: {}
artifacts: {}
claim_boundary: {}
```

Each profile records:

- adapter identifier;
- state and control schemas;
- equations and active force/moment sources;
- input data and source hashes;
- omitted physics;
- status and evidence artifact;
- automatic-lowering eligibility;
- exact claim and nonclaim.

Each profile also records a promotion gate:

```yaml
promotion_status: development | planned | qualified | not_applicable
required_operations: [state_derivative, trim | trim_fragment, linearize, effectiveness, allocate]
blockers: [source_trim_acceptance]
```

`promotion_status` is a declared maturity boundary, not a substitute for
runtime evidence. The registry combines it with operation-level probe results
to produce `validation_status` values such as `pass`, `blocked`, `planned`, or
`not_checked`. A qualified declaration with a failed required probe is a hard
registry failure; a development declaration with named blockers remains
visible development evidence.

A tier that is not supported is still declared:

```yaml
status: not_applicable
reason: passive_uncontrolled_body
```

## Standard execution pipeline

The same stages run for every family:

```text
source intake
    -> frames, units, and table-domain checks
    -> data readiness
    -> family plant binding
    -> trim or explicit not-applicable decision
    -> source-derived linearization or declared response law
    -> controller synthesis and scaling
    -> control realization selection
    -> mission execution
    -> truth-based evaluation
    -> convergence and replay checks
    -> automatic fidelity lowering
    -> claim and showcase artifact
```

Automatic lowering is adapter-aware. The resolver joins the selected profile's
qualification evidence with the tier's manifest-declared `required_operations`
and the adapter's capability status. A profile with valid evidence but a
missing `trim`, `linearize`, `effectiveness`, or `allocate` operation is
blocked and the next permitted tier is selected only when the lowering policy
allows it. The lowering artifact preserves both the required and missing
operation lists.

Family-specific code may implement equations and source adapters, but it may
not bypass a stage without recording `not_applicable`, `not_available`, or a
qualified exception in the manifest.

## Standard control path

All controlled families use the same semantic path:

```text
mission objective
    -> guidance reference
    -> attitude/rate or kinematic command
    -> controller request
    -> desired generalized wrench
    -> direct-wrench bridge OR physical allocator
    -> actual plant inputs
    -> achieved state and wrench
```

The four tiers differ only in the final realization:

- 3DOF applies the declared translational force model.
- Pseudo-6DOF applies the declared response law and does not claim physical
  moments or individual effectors.
- Direct wrench applies the requested generalized force/moment to the
  integrated rigid-body plant and logs requested versus achieved wrench.
- Surface allocation solves a bounded, rate-limited allocation problem and
  logs requested wrench, allocated effectors, actual effectors, achieved
  wrench, residual, saturation, and authority margin.

## Evidence and artifact contract

Every tier produces the same top-level artifacts, even when some channels are
not applicable:

```text
manifest.json
claim.json
resolved_case.json
realized_fidelity.json
source_provenance.json
state_schema.json
control_schema.json
resource_schema.json
telemetry.parquet
controls.parquet
effectors.parquet
resources.parquet
events.json
trim_report.json
linearization_report.json
controller_report.json
allocation_report.json
envelope_report.json
convergence_report.json
batch_step_parity.json
evaluation.json
reproduction.txt
plots/
```

Unavailable channels are represented in the schema and marked
`not_applicable`; a renderer must never infer a physical effector from a
surrogate command.

## Work packages

### HVI-1 — Canonical vocabulary and compatibility layer

- Create one source of truth for tier identifiers, ordering, parent tiers,
  control realization, and evidence statuses.
- Add explicit `direct_wrench_bridge` terminology.
- Preserve legacy spellings through an explicit normalizer and diagnostics.
- Update controller provenance and runtime lowering to use the canonical path.

Exit criteria:

- No module defines a private four-tier vocabulary.
- Direct wrench is not reported as a generic or ambiguous rigid-body tier.
- Legacy `rigid_body_6dof` records are either normalized with an explicit
  realization or rejected with an actionable diagnostic.

Current status: the canonical runtime mapping and selector are implemented;
the remaining legacy runtime contracts are retained only at compatibility
boundaries and still need staged migration.

### HVI-2 — Manifest and readiness unification

- Make `physical_family` drive shared overlay requirements.
- Require one four-profile manifest for every supported family.
- Reuse the same data-readiness checker for all tiers.
- Emit a machine-readable missing-data worklist.

Exit criteria:

- The nine current families produce the same manifest-shaped readiness report.
- Missing source, trim, effectivity, or actuator data are reported at the
  same path and severity regardless of family.
- Unsupported tiers are explicit rather than absent.

Current status: source-family readiness now resolves the direct and
surface-allocated slots from explicit `control_realization` metadata, so a
legacy profile name is no longer required to discover the correct tier. The
nine-family manifest join is now checked by
`tools/validate_unified_family_manifest.py` and the repository check sequence.
A family with conflicting horizontal, pseudo, legacy, or source authorities is
rejected before lowering or showcase construction.
The joined readiness artifact now makes the same check visible for all nine
families and all four tier slots, including families that do not yet have a
legacy vehicle-definition entry. It is regenerated by
`python3 tools/validate_horizontal_readiness.py --check` and included in the
repository check sequence.

### HVI-3 — Adapter façade and common pipeline

- Wrap existing family-specific plants behind the common adapter surface.
- Make trim, linearization, tuning, allocation, and telemetry use common
  result types.
- Return typed capability results for passive or open-loop models.
- Remove duplicated family-specific orchestration where the generic utility
  already covers the operation.

Current execution slice:

- `StandardFamilyAdapter.from_control_plant` wraps the existing
  `ControlPlantAdapter` without changing its state or effector ordering.
- `descriptor_from_control_plant` derives the public channel schema from the
  plant ordering, leaving only identity and unit declarations to the family
  binding.
- `StandardFamilyAdapter.passive` records an uncontrolled body with explicit
  `not_applicable` operations.
- `FamilyCapabilityReport` is the common preflight result consumed before a
  numerical operation is invoked.
- `ADAPTER_OPERATIONS` is the single operation vocabulary used by both the
  façade capability report and the horizontal manifest promotion gates;
  adding a new operation therefore cannot silently create a second registry
  vocabulary.
- `trim_fragment` is the typed source-evidence seam for partial equilibria;
  the HL-20 source-surface witness now exposes its verified scalar
  pitch-channel trim through this operation while full `trim` remains
  unavailable.
- Unit tests cover a controlled plant, a passive body, missing capability
  declarations, and duplicate-channel rejection.
- The real X8 table-coordinate and Hummingbird individual-rotor plant
  builders now run through that same façade/conformance test; their plant
  state and effector orderings are not re-declared by the witness.
- The registry-backed harness also exercises the B747, F-16, X-15, and HL-20
  source direct-wrench bridges and emits
  `verification/alpha3_horizontal_fidelity/adapter_registry.json`.
- The passive tumbling witness uses the same descriptor and conformance path
  with an explicit `uncontrolled` realization and no fabricated controls.
- `family_adapter_probes.py` now exercises the declared operation path for
  executable controlled witnesses: finite state derivative, accepted trim,
  provenance-bearing linearization, local effectiveness, and bounded
  allocation. Provider-backed partial plants can pass only their declared
  operations; the HL-20 source-surface witness uses this path with full trim
  explicitly `not_applicable` and its scalar source trim fragment checked
  separately. Resource and observation
  operations remain explicit `not_applicable` until family providers are bound.

Exit criteria:

- X8, Hummingbird, F-16, X-15, and tumbling body pass the same adapter
  conformance suite despite different physical topologies.
- B747, A320, F-16, X-15, and HL-20 migrate without new pipeline code; the
  remaining work is family data and qualification, not another façade.

### HVI-4 — Common conformance harness

Run the same checks for each family and advertised tier:

- schema and source-hash validation;
- frames, units, and table-domain checks;
- trim or explicit non-applicability;
- derivative and response-law provenance;
- controller scaling and matrix dimensions;
- requested/achieved control channels;
- resource closure;
- automatic lowering;
- numerical convergence;
- batch/step parity;
- claim/nonclaim consistency.

The harness adds family probes only for genuinely family-specific behavior:
elevon signs, rotor mixing, gimbal authority, staging, spacecraft torque
authority, or projected-area behavior.

Initial execution slice:

- The registry-backed harness runs schema/control-realization conformance for
  all nine family topologies; NESC source replay is checked through the same
  façade while its physical-control tiers remain explicit promotion gates.
- For X8, B747, Hummingbird, F-16, and the A320 reduced products it now runs
  the declared operating point through the common derivative/trim/linearize
  probe. Effectivity and allocation are required only for the physical
  surface/rotor tier; lower force/response models report both operations as
  explicitly not applicable.
- For HL-20 it additionally checks that the source-backed scalar trim
  fragment is present, finite, provenance-bearing, and still bounded by its
  explicit nonclaim; its direct-wrench bridge is checked separately.
- The registry now emits a twenty-entry available-tier matrix: direct wrench
  and surface allocation are checked separately for X8, B747, Hummingbird,
  and F-16; Hummingbird and F-16 point-mass and pseudo tiers are also checked
  independently; A320 point-mass and pseudo tiers are checked independently; the
  HL-20 direct and source-surface tiers are checked with their respective
  partial-operation contracts; the
  X-15 direct-wrench bridge is checked as a local high-energy load witness with
  bounded local trim and source-derived linearization; the
  NESC point-mass and pseudo tiers are checked through replay; and the passive
  tumbling witness checks its pseudo tier.
- It is available as `python tools/dev.py family-adapter-registry` and is part
  of the repository `check` sequence.
- Resource and observation providers remain explicit `not_applicable` entries
  for these witnesses; a passing adapter report alone is not a claim of plant,
  controller, or mission qualification.
- The same registry emits a thirty-six-entry promotion matrix covering all
  nine families and four tier slots. It records declared status, required
  operations, operation results, and blockers, including explicit
  `not_applicable` direct/surface tiers for passive tumbling.
- The registry report now also emits `horizontal_lowering`: profile evidence
  is joined with the live operation-probe map before a canonical tier is
  selected. In the current artifact, X8, B747, F-16, X-15, Hummingbird, and
  HL-20 select their checked direct-wrench bridge; NESC remains at pseudo-6DOF
  because its participating direct-wrench operations are not available, and
  passive tumbling remains unselected.
- `horizontal_readiness` now emits a 36-slot pre-simulation worklist. It joins
  metadata readiness for X8, B747, Hummingbird, and X-15 with source-manifest
  evidence for F-16, HL-20, and NESC, and explicitly reports unavailable
  metadata for reduced A320 tiers rather than silently treating profile
  declarations as data completeness.

### HVI-5 — Standard showcase and release packet

- Make the common artifact contract the input to all showcase renderers.
- Generate the same card, evidence board, and machine-readable evaluation for
  every family/tier.
- Keep family-specific panels composable rather than custom report formats.
- Include the exact control realization and claim boundary on every page.
- Construct `FidelityShowcaseRealization` through the common adapter binding;
  copy the adapter state schema and surface effectors only when the canonical
  tier is surface-allocated.
- Preflight showcase-required adapter operations before rendering a claim.

Current execution status: adapter-to-showcase binding, legacy ambiguity
diagnostics, and generated DAVE-ML catalog migration are implemented and
covered by focused tests. Full renderer input unification and family-specific
panel completeness remain release work; this slice prevents a showcase from
claiming a control path that its adapter does not provide. The same operation
requirements are now consumed by the canonical automatic-lowering report, so
profile qualification cannot be mistaken for executable adapter capability.

The artifact boundary is now centralized as well. `ShowcaseRunArtifact` may
retain the resolved `FidelityShowcaseRealization`, and
`showcase/artifact_binding.py` constructs fidelity, control realization, claim,
and nonclaim fields from that object rather than accepting a second set of
renderer-owned values. The DAVE-ML evidence-board builder uses this factory;
its evidence-summary telemetry is explicitly labeled as evidence telemetry,
not a live plant state schema. Direct-wrench evidence therefore remains
free of physical-effector claims in the emitted run manifest.

### HVI-6 — Remaining-family promotion gates

The current horizontal slice deliberately stops short of pretending that every
family has the same source data. The next migrations are data-gated:

- **X-15:** the source-backed local release/glide load witness now occupies the
  direct-wrench tier with a bounded local source-load-cancellation trim and an
  A/B pair differentiated from the same nonlinear bridge plant. This is not
  source physical-effector trim. It still needs powered/coast/atmospheric
  scheduling and physical stabilator/rudder/RCS/propulsion allocation before
  surface-tier promotion.
- **HL-20:** source surface effectivity, bounded allocation, the explicit local
  direct-wrench bridge, and the scalar source trim fragment are executable
  through one tier-aware factory. Full-state source trim,
  attitude/position propagation, and closed-loop qualification remain the
  next vertical slice. The direct bridge remains a generalized wrench claim
  and does not imply physical surface control.
- **NESC:** its source translation and scheduled response products now use the
  common optional `replay` capability at the point-mass and pseudo tiers. Its
  source manifest still declares no controls, gimbal channel, or event-aligned
  participating attitude input. Those replay adapters do not supply a common
  nonlinear derivative or physical control path. Do not synthesize gimbal
  authority merely to make the registry matrix rectangular.

The NESC promotion gate is therefore explicit: bind an authoritative
time-aligned attitude/gimbal/effectivity data contract before promoting the
direct-wrench or surface tiers. Until then, the registry reports replay
conformance while the showcase claim remains source replay, not
participating-simulation control qualification.

- **B747:** the first canonical physical-surface witness is now checked in at
  `examples/mission_families/slower_b747/SV01_racetrack_surface_allocation_6dof.prb`.
  It uses the source elevator, aileron, and rudder tables through the shared
  bounded allocator, and `tests/e2e/test_b747_surface_allocation.py` runs the
  file directly rather than constructing a one-off problem in the test. This
  is a local three-surface allocation/telemetry witness, not yet a transport
  route qualification or scheduled controller promotion.

The joined readiness report is the first intake artifact to consult when a new
family is added. A family author should be able to answer, for each tier,
"what data is present, what adapter operations passed, and what remains
blocked?" without opening four independent reports.

The promotion matrix is the authoritative worklist for these gates. It may
report `pass` for a callable development operation probe while retaining a
non-empty promotion blocker: adapter structure is not the same evidence as a
completed family mission or source-authoritative effectivity.

The unified manifest validator is also a direct repository-root command:
`python3 tools/validate_unified_family_manifest.py --check`. The horizontal
registry and adapter-registry validators use the same standalone invocation
pattern, so a new-family author can run the evidence checks without relying on
an activated package environment.

## Current family migration matrix

| Family | Physical adapter work | Main horizontal task |
|---|---|---|
| X8 | Existing surface and direct-wrench paths | First fixed-wing conformance adapter |
| B747 | Existing table plant and candidate allocator | Replace transport-specific orchestration with shared façade |
| A320 | OpenAP and pseudo response bindings | Point-mass and pseudo tiers now use the common façade; direct/surface tiers remain planned |
| F-16 | Source plant, source-force 3DOF/pseudo adapters, local linearization, surface path | Use shared scheduled-controller contract |
| X-15 | Staged source/reduced plant | Direct-wrench local release/glide bridge has source-derived local trim and linearization; powered/coast/atmospheric scheduling and physical effectors remain planned |
| Hummingbird | Aggregate 3DOF/pseudo adapters plus native rotor allocator | Prove rotor topology through the same allocation result contract |
| HL-20 | Source plant and logical-surface overlay | Source direct-wrench and surface-allocation witnesses use one tier-aware façade and the same scalar trim-fragment seam; full trim and closed-loop flight remain next |
| NESC rocket | Source-replay point/pseudo adapters | Use replay for retained translation/response artifacts; gimbal and participating direct/surface tiers remain planned |
| Tumbling body | Passive rigid-body plant | Same artifacts and checks; controller/allocation explicitly not applicable |

## Validation snapshot

The current executable slice is green under the repository environment:

- `python3 tools/dev.py check` passes, including 1,907 selected tests, 72 E2E
  tests, type checking, lint, manual/PDF builds, and the adapter registry
  harness.
- The project interpreter's full `pytest` invocation passes with 1,907 tests
  passed, 60 skipped, and 245 repository-opt-in tests deselected by the
  project configuration.
- The dedicated adapter tests pass for the controlled, direct-wrench, surface,
  open-loop, and passive paths.
- `verification/alpha3_horizontal_fidelity/adapter_registry.json` is
  intentionally `development`: nine family witnesses pass and no family
  binding silently falls back. Its tier matrix contains twenty passing
  available-tier checks, including Hummingbird and F-16 point/pseudo
  reductions plus separate HL-20 direct-wrench and surface-allocation
  witnesses;
  the controlled witnesses also pass the common derivative, trim,
  linearization, effectivity, and allocation probes.
- Its `promotion_matrix` contains all 36 family/tier slots and separates 29
  development slots, 5 planned slots, and 2 passive not-applicable slots from
  operation validation. No family is promoted merely because a lower-tier or
  direct-wrench probe passed.
- `verification/alpha3_horizontal_fidelity/readiness.json` contains one
  joined record for each of the 36 family/tier slots. It is a development
  readiness artifact, not a family qualification badge; its `probe_ready`,
  `data_ready`, `planned`, and `not_applicable` statuses preserve that
  distinction.
- The repository's pinned interpreter is `.venv/bin/python`; a bare `python`
  executable is unavailable in the current shell, so the reproducible command
  is `./.venv/bin/python -m pytest`.

Current complete repository verification:

- `python3 tools/dev.py manual` passed.
- `python3 tools/dev.py equation-audit` passed for 326 equations.
- `python3 tools/dev.py check` passed, including repository type checking,
  lint, manual/PDF generation, the nine-family adapter registry, the
  configured fast-test and E2E passes, and the artifact/readiness checks.
- `./.venv/bin/python -m pytest` passed with 1,907 tests passed, 60 skipped,
  and 245 deselected. The only warning is the pre-existing Matplotlib legend
  warning in `tools/build_family_qualification_packet.py`.
- Focused showcase-contract, adapter-binding, and DAVE-ML evidence-builder
  tests passed; the DAVE-ML builder now retains the resolved fidelity/control
  realization and emits no physical-effector list for direct-wrench evidence.
- Direct validators for the horizontal registry, unified family manifest,
  family adapter registry, and joined readiness report all passed.
- The rebuilt Alpha 2 catalog-reference report passes with four resolved child
  packets and the strict child boundary passes for five retained run artifacts
  (the four family packets plus the supporting B747 template packet). The
  Alpha 3 composition catalog also regenerates as
  `development_catalog_verified` with nine boards, nine paired-fidelity
  records, and nine cross-fidelity passes; its supplemental evidence remains
  composition-level evidence rather than automatic family promotion.
- The HL-20 direct-wrench bridge and surface-allocation witness both pass the
  typed scalar trim-fragment probe; full `trim` remains `not_applicable` for
  both because the source evidence is only a pitch-channel equilibrium.
- The canonical lowerer has an adapter-aware mode covered by unit tests: a
  qualified profile with a missing required operation is blocked, and an
  explicitly permitted lower tier may be selected instead. The adapter
  registry now supplies those operation statuses to the horizontal lowering
  report, rather than leaving the join to separate artifacts.
- The repository project interpreter is `.venv/bin/python`; no `python`
  executable is available on `PATH`. The reproducible validation path uses the
  pinned project interpreter; a separate system `python3 -m pytest` run should
  not be used as the authoritative result because it is sensitive to globally
  installed packages.

Post-verifier focused verification:

- Ruff passes for `tools/dev.py`,
  `tools/validate_showcase_catalog_references.py`, the legacy migration tool,
  and their tests.
- The catalog-reference and legacy-migration focused tests pass (`3 passed` in
  the latest focused run).
- The tracked Alpha 2 catalog was rebuilt from canonical X8, B747, CA-HI, and
  migrated Hummingbird child packets. The catalog-reference validator resolves
  all four children with no boundary findings. Its status remains an honest
  evidence/integration catalog: X8 is `numerical_failure` at the source beta
  boundary, CA-HI is partial at its endpoint, Hummingbird is a nominal pass
  pending family qualification, and B747 is a nominal point-mass pass pending
  higher-fidelity qualification.

### HVI-7 — Showcase-boundary migration

The common builder migration is now implemented for the named run producers;
the remaining work is release regeneration and aggregate-reference checking:

1. `build_showcase_run_artifact` is the canonical constructor for migrated
   builders, retaining one resolved realization object per emitted run.
2. `validate_showcase_run_artifact_boundary` and
   `tools/validate_showcase_artifact_boundary.py` reject fidelity/control,
   claim, nonclaim, and physical-effector drift in every generated packet.
   The main `tools/dev.py check` path builds the current DAVE-ML packet in a
   temporary directory and validates all six emitted run artifacts.
3. `preflight_horizontal_showcase` and
   `tools/validate_showcase_preflight.py` make the joined readiness report the
   input to showcase selection. Required adapter operations are checked before
   a renderer or mission run is started, while mission qualification remains an
   advisory rather than being silently promoted.
4. The F-16 S-119 racetrack ladder now emits canonical `run_artifacts` for
   point-mass, pseudo-6DOF, direct-wrench, and surface-allocated modes. Its
   generated packet boundary was checked across eight runs in five manifests;
   the surface packet retains declared elevator/aileron/rudder/throttle
   effectors while the direct-wrench packet retains none.
5. The shared family-qualification builder now requires an explicit canonical
   `fidelity_tier` in its mission catalog and emits `claim.json`,
   `realized_fidelity.json`, and a canonical run artifact. X8 3DOF, X8
   surface-allocation, and B747 3DOF generated packets pass metadata-boundary
   validation. A numerical or objective failure remains an honest artifact
   outcome and is not promoted by the builder.
6. The full standalone suite now passes with 1,907 tests; the new artifact
   boundary, explicit mission-fidelity, and native CA-HI wrapper tests are
   included in that count.
7. The synthetic CA-HI X8-plus-boosters packet now emits the same canonical
   run-artifact boundary. Its rigid-body direct-wrench realization declares
   staged mass flow and table-driven loads, no physical effectors, and an
   honest partial outcome for the independently failed endpoint. A generated
   packet passed the strict boundary validator with one run artifact.
8. The legacy B747/X8 evidence builder now emits one canonical run artifact
   per declared case. Its bounded `--case` selector provides a deterministic
   smoke path for individual family/tier slices; a two-case B747/X8 point-mass
   packet produced two artifacts and passed the strict boundary validator.
9. The four-family fidelity-ladder builder now emits one canonical artifact
   for each of its twelve core family/tier runs. Legacy point-mass and
   kinematic labels lower to `force_model` and `response_law`; the rigid-body
   ladder is explicitly `surface_allocated` with family-specific effectors.
   A real B747-scoped ladder smoke packet produced three artifacts and passed
   the strict boundary validator.
10. The orbital, suborbital, and quadcopter synthetic example generators now
    emit canonical manifests alongside their existing telemetry/debug files.
    Their fixtures are explicitly `synthetic`, and the quadcopter remains a
    pseudo-6DOF response-law record with no fabricated motor-effector claim.
11. The native California-to-Hawaii example runner now emits the same
    canonical realization, claim, and run-artifact manifest as its packet
    builder. Its native rigid-body path is explicitly a synthetic
    `direct_wrench` bridge: the presence of aerodynamic coefficient tables and
    semantic `alpha`/`bank` commands does not promote it to physical
    control-surface allocation.

The producer audit classifies the remaining showcase-related tools as follows:

| Producer class | Current members | Horizontal contract |
|---|---|---|
| Native run producer | F-16, family-qualification, CA-HI, B747/X8 evidence, fidelity ladder, Hummingbird, DAVE-ML, extended synthetic examples | Must emit `realized_fidelity.json`, a resolved `run_artifacts` entry, and pass the strict artifact-boundary validator |
| Aggregate catalog/compositor | `build_x8_showcase_composite.py`, `build_alpha2_showcase_catalog.py`, `render_showcase_composites.py`, `render_evidence_showcase.py` | Must consume already-resolved packets and preserve their status/claim boundary; it must not create a second run artifact or infer fidelity |
| Evidence/probe packet | `build_maneuver_evidence_packet.py`, `validate_*_r1_matrix.py`, source-control/effectivity/trim probes | May emit a domain-specific evidence manifest, but is not a mission run and must identify itself as evidence/probe rather than a `ShowcaseRunArtifact` |
| Native example API | `examples/showcases/california_to_hawaii/run_showcase.py` and sibling `run_showcase.py` modules | May continue returning the native `RunArtifact`, but generated showcase directories must also carry the canonical wrapper; the wrapper is the claim boundary used by catalogs and renderers |

This classification is intentional. Forcing an aggregate or diagnostic packet
to masquerade as a live run would make the horizontal contract less honest.
The boundary validator therefore rejects an aggregate manifest when it is
passed as a run manifest, while catalog builders remain responsible for
referencing and preserving the canonical child packets.

The remaining implementation items are:

12. Rebuild the tracked Alpha 2/Alpha 3 catalog outputs from the migrated
    producers and run the boundary validator on every child packet. Alpha 2 is
    now rebuilt and its four child packets resolve canonically; its strict
    boundary report passes. The Alpha 3 composition catalog is also regenerated
    and verifies its nine board/pair/cross-fidelity records, while its
    supplemental evidence remains a separate composition-level reference
    surface. The retained Hummingbird packet uses the explicit
    `tools/migrate_legacy_showcase_packet.py` wrapper with
    `rerun_performed: false`; a current full-builder rerun was attempted but
    remains a performance/pending issue after reaching the climb gate. Alpha 3
    tracked catalog outputs still require the same release-boundary audit.
13. Add a catalog-level reference check that verifies aggregate records point
    to canonical child manifests and retain child status, fidelity, and
    claim-boundary hashes without reinterpreting them. Implemented in
    `tools/validate_showcase_catalog_references.py`, with explicit rejection
    of aggregate-only or diagnostic manifests passed as child runs.
    It is exposed as `python3 tools/dev.py showcase-catalog-references` and
    passes against the rebuilt Alpha 2 catalog.
14. Use the 36-slot promotion matrix to schedule the next source-backed
    vertical slices: X8 surface mission, B747 scheduled surface mission, and
    then the gated X-15/HL-20/NESC promotions described above. The X8 full
    route currently fails closed when the two-elevon witness reaches beta
    5.012 degrees against a +/-5 degree source-table boundary; this is a
    promotion blocker to solve through an independently supported yaw/sideslip
    control path, not an envelope relaxation. The short X8 surface witness now
    asserts the two-effector/rank-two boundary explicitly: roll and pitch are
    controlled axes, yaw is an uncontrolled residual, and no third actuator is
    inferred from coupled aerodynamic response. Enabling a yaw weight in an
    experiment did not change that rank or close the route, so it is not a
    promotion fix. The B747 work has advanced one step further with a checked
    canonical three-surface local witness; its full scheduled surface route is
    still the next B747 promotion slice.

This sequence keeps the family process identical while preserving the honest
family-specific boundary: a rectangular tier matrix is an integration
contract, not evidence that every family has the data needed for every tier.

## Definition of done

The horizontal integration work is complete when:

1. A new family can be registered with one manifest and one plant adapter.
2. The same commands validate readiness, trim, linearization, control
   realization, mission evidence, and showcase artifacts for every family.
3. The four fidelity tiers have one vocabulary and one lowering policy.
4. Direct-wrench, surrogate-response, and physical-effector claims cannot be
   confused by artifact names or legacy defaults.
5. The conformance harness identifies missing data before a family-specific
   simulation is run.
6. A passive or open-loop family can participate without fabricated controls.
7. Every published result identifies its tier, control path, source status,
   qualified envelope, and exact nonclaims.
