# Alpha 3 Fidelity-Ladder Execution Plan

**Status:** Active long-running goal  
**Scope:** solid `point_mass_3dof` and `pseudo_6dof` realizations for the
supported atmospheric, launch, and passive-body families  
**Parent plan:** [`taoryx-alpha-3.md`](taoryx-alpha-3.md) and the
[`fidelity-first-integration-program`](fidelity-first-integration-program.md)

## Working definition of a solid fidelity pair

For this plan, **solid** means executable and auditable at the declared scope;
it does not mean that every pair is source-exact, actuator-realizable, or
family-qualified. A solid pair must have:

1. a deterministic, runnable `point_mass_3dof` record;
2. a named `pseudo_6dof` record, or an explicit native rigid-body realization
   when the subject is passive rotation rather than controllability;
3. an independent truth-based mission or event evaluation;
4. a machine-readable claim/nonclaim boundary and fidelity provenance;
5. a paired comparison that reports terminal, event, resource, and applicable
   envelope differences; and
6. a fail-closed promotion status when required source data, effectors, or
   parent capability evidence are missing.

The Alpha 3 program therefore reports three different outcomes rather than
collapsing them into one badge:

| Pair outcome | Meaning |
| --- | --- |
| `nominal_pair_ready` | The declared 3DOF and pseudo mission records pass and may participate in validated automatic lowering. |
| `passive_pair_ready` | A passive body has a passing averaged-area 3DOF reduction and native rigid-body reuse; no controller is invented. |
| `development_pair` | Both tiers execute and pass their local structural/event gates, but source, envelope, terminal, or physical-effector promotion remains open. |

This is the acceptance language used by the long-running execution goal for
X8, B747, A320, F-16, X-15, Hummingbird, HL-20, NESC, and tumbling bodies.

## 1. Release objective

Alpha 3 is complete for this workstream when each target family has a
reproducible 3DOF case and a named pseudo-6DOF or rigid-body realization, with
the exact physics boundary visible in its manifest, telemetry, evaluation, and
showcase.

The target families are:

| Family | 3DOF claim | Pseudo-6DOF claim |
| --- | --- | --- |
| X8 | Fixed-wing translation, energy, route, altitude, and speed | Bounded bank/pitch/yaw response coupled to the same plant |
| B747 | Transport-scale translation, energy, route, and arrival | Slower scheduled attitude/rate response and transport limits |
| F-16 | Subsonic energy maneuvering and envelope behavior | High-rate response with alpha, beta, load-factor, and rate limits |
| X-15 | Release, powered flight, coast, descent, and energy corridor | Flight-condition-scheduled attitude/rate response across regimes |
| Hummingbird | Hover/translation/resource behavior and landing | Rotor/thrust-vector attitude response, yaw authority, and contact |
| A320 | OpenAP performance and route/energy mission | Calibrated response-law surrogate at declared operating points |
| HL-20 | Unpowered glide, crossrange, bank, and terminal energy | Bank/pitch/alpha response and glide attitude feasibility |
| NESC rocket | Staging, mass flow, coast, deployment, and target state | Gimbal/thrust-vector response, staging settling, and deployment attitude |
| Tumbling body | Orientation-averaged or scheduled-area translation | The actual rigid-body rotational model, exposed through the common interface |

This is not a claim that every family has physical effectors. A pseudo model
may use a response law or a source-derived reduction. Direct wrench injection,
prescribed attitude, and synthetic channels must remain visibly classified.

The direct-wrench bridge is an explicit fourth selectable realization for
families whose rigid-body plant can accept a bounded generalized wrench before
their physical effectors are ready.  The catalog binds this tier separately
from `pseudo_6dof`; automatic lowering may select
`rigid_body_6dof_direct_wrench` only when its own checked artifact and the
parent 3DOF evidence both pass.  Hummingbird, HL-20, and X-15 now have this
bridge represented explicitly.  Hummingbird retains native rotor allocation
as its preferred physical path; HL-20 and X-15 retain their source-trim and
surface/terminal blockers.  This is a bridge mode, not a hidden claim of
actuator realization.

## Execution board for the long-running goal

The program is deliberately staged.  The first milestone is breadth: every
target family must have a runnable 3DOF record and a named pseudo-6DOF record,
or an explicit rigid-body reuse record for a passive body.  The second
milestone is depth: only families with the required source data and physical
effectors may advance into surface/rotor/gimbal allocation and envelope
qualification.

### Breadth milestone — paired reduced fidelity

The nine-family paired index is now executable and reproducible:

- X8, B747, A320, F-16, Hummingbird, and X-15 have nominal paired mission
  evidence and are the current automatic-lowering-authorized reference set.
  Hummingbird authorization is explicitly limited to its bounded
  `hover_box_yaw_contact_v1` pseudo case; X-15 authorization is limited to
  `staged_event_energy_corridor_impact_v1`.
- HL-20 and NESC now have scoped nominal paired evidence for their synthetic
  release/glide/impact and source-translation/stage/orbit-coast missions;
  source-exact control and physical-effector promotion remain blocked.
- The tumbling body has an averaged-area 3DOF reduction and native rigid-body
  reuse for the requested pseudo tier; it receives no invented controller.

This milestone proves interface breadth, not family-wide qualification.  A
development pair may run and compare, but it may not be silently selected as a
qualified fallback.

### Depth milestone — physical or source-bound promotion

Execute these gates in order and retain boundary failures:

1. Use the resolved X8 source-paper left/right elevon mapping, then retain the
   four-pass local roll/pitch source-coordinate R1 matrix and the explicit
   beta-domain boundary.  The short mapped-surface witness and the long
   fail-closed boundary packet are now reproducible.  The full mapped-surface
   racetrack remains blocked until an independent yaw realization or a
   source-bounded route contract is available.
2. Close B747 source-condition trim/table-domain blockers before emitting any
   interpolated physical schedule; FC3 remains the regression oracle.
3. Compare Hummingbird aggregate pseudo translation/contact/resource behavior
   with a translated native rotor plant; preserve local hover rotor R1 as a
   separate physical witness. The individual-rotor source pad-to-pad packet
   now closes the end-to-end rotor command/contact slice; electrical battery,
   wind, ground-effect, and full-envelope promotion remain separate gates.
4. Expand the F-16 local physical schedule authority packet into a declared
   Mach/dynamic-pressure operating envelope, without converting partial
   authority into a pass.
5. Add source-backed powered/coast/atmospheric X-15 scheduling and a controlled
   terminal handoff.
6. Use the pinned HL-20 DAVE-ML surface-direction probe as the source-bound
   starting point, retain its Mach-3 rudder-authority boundary, and close
   trim, allocation, and bank/alpha/energy evidence; do not promote the
   synthetic aerodynamic branch as source-exact.  The source allocation
   probe now uses the shared `EffectorEffectiveness` and bounded
   `allocate_and_advance_wrench` path, including declared surface position,
   rate, and first-order lag limits.  This proves reusable source-load
   allocation infrastructure and retained authority boundaries; it is still
   open-loop evidence until source-backed trim and nonlinear closed-loop
   response are demonstrated.
7. Add independent parent attitude/gimbal evidence for NESC before claiming
   source-exact thrust-vector control.
8. Add the passive tumbling-body cross-fidelity loss report while keeping the
   pseudo tier equal to native rigid-body dynamics and the 3DOF tier
   orientation-averaged.  The shape/rate/area-policy and zero-moment
   diagnostic witnesses are now complete; retain the no-controller boundary.

Each gate must emit a manifest, reproduction command, exact claim boundary,
and structured failure/boundary cases.  The release index may count evidence,
but it must never infer a higher tier from a neighboring family or fidelity.

### Current implementation checkpoint

The current depth checkpoint is intentionally asymmetric and must remain so in
the release language:

- **X8:** the mapped left/right elevon allocator is a real source-coordinate
  physical path for local roll/pitch demand.  The full route is not promoted:
  it reaches the published beta boundary before an end-to-end surface-routed
  racetrack can be claimed.  No direct yaw moment or table widening is an
  acceptable substitute.
- **B747:** condition 3 is a passing local physical-surface node.  Conditions
  4--7 now close source trim through an explicitly bounded local re-trim, but
  remain controller-authority boundary witnesses because no generic local
  design passes every interior witness.  Conditions 8--10 remain blocked
  during source-trim/table-domain closure.  The schedule generator filters
  repeated source grids by condition, Mach, and altitude, and records the
  re-trimmed truth alpha, controller trials, machine-readable blocker code,
  worst residual axis, candidate state/control, and remediation rather than
  exposing only an optimizer exception.  Until all nodes close and their
  physical controllers pass, no interpolated runtime schedule is emitted.
- **F-16:** four source-retrimmed physical-effector nodes pass the declared
  local matrix and adjacent allocation probes.  Continuous nonlinear schedule
  replay and full-envelope qualification remain separate gates.

This is the intended automation boundary: a new vehicle can be lowered to a
solid reduced pair before physical effectors are available, but promotion to a
surface/rotor/gimbal tier requires source-trim closure, plant-derived
effectiveness, constrained allocation, and nonlinear evidence at every
declared operating point.  A blocked operating point is a typed integration
result, not a reason to silently reuse a neighboring node.

F0 is executable in the current repository:

- `verification/pseudo6dof_profiles.yaml` declares all nine family bindings;
- `pseudo6dof_profiles.py` validates references, response axes, area policies,
  and tumbling-body rigid-body reuse;
- `response_laws.py` provides the shared bounded axis-response primitive;
- `build_automatic_lowering_report()` produces a fail-closed, evidence-keyed
  lowering decision. It records every considered tier, the first missing
  prerequisite, and the selected tier; development profiles and disabled
  family policies cannot be silently bypassed; and
- the X8/F-16/A320 family path can select a catalog profile and records its
  profile ID in telemetry; the native X8 and B747 `.prb` bridge now consumes
  the same catalog through `response-profile-id`; and
- the A320 and F-16 pseudo racetrack validators pass their four independent
  truth gates with the shared response law selected.

The native `.prb` bridge uses the explicitly named
`first-order-rate-limited-v1` compatibility law. It consumes the catalog
time-constant and rate-limit fields per axis. Its acceleration field remains
reserved for the persistent-rate upgrade in F1 because the current native
kinematic sidecar does not yet store angular-rate state. This is therefore a
real catalog binding, but still a response surrogate: it does not claim
physical moments, elevons, or other effectors.

The reachability-backed X-15 and source-bound HL-20 vehicles now bind their
declared pseudo-6DOF profile IDs into the executable reduced envelope and
provenance telemetry. The reachability bridge deliberately retains its
established first-order angle propagation for checkpoint comparability; the
shared bounded second-order law remains available to stateful pseudo runners.
The catalog resolver is cached, so profile validation is not a file parse in
the inner integration loop.
The X-15 now also emits a common-family evidence packet through
`tools/validate_x15_fidelity_ladder.py`. Its independent truth evaluator
checks boost/cutoff, booster release, unpowered glide, a declared high-energy
speed/altitude corridor, and terminal impact ordering at both 3DOF and
pseudo-6DOF. The packet is a scoped `nominal_case_pass` for
`x15.staged_event_energy_corridor_impact_v1`; it still does not prove a
controlled terminal handoff or physical X-15 effector allocation.
The X-15 pseudo profile now carries an explicit `boost`/`coast`/`glide`
response schedule. The reduced runtime selects that schedule from the actual
phase state and records `pseudo6dof_response_phase` plus
`pseudo6dof_response_schedule_applied` in parent telemetry. These are
auditable engineering response assumptions; they are not source-exact X-15
control laws and do not promote the profile beyond its declared staged-event,
energy-corridor, and impact scope.
NESC now has an executable composite pseudo-6DOF development bridge: it
retains the qualified source translation/staging history and adds the declared
bounded attitude-response law. This remains deferred from source-exact pseudo
qualification because the retained DAVE-ML evidence has no independent
attitude-response comparison channel and no physical gimbal allocation is
claimed.
Its profile now schedules bounded response assumptions over the retained
`stage1_burn`, `stage2_burn`, `stack_coast`, and `orbit_coast` labels. Each
composite row records the active response phase and whether a scheduled entry
was applied; the schedule is evidence of explicit regime handling, not source
gimbal-effectivity evidence.
The source-data contract audit now makes that blocker machine-checkable. It
inspects the pinned aerodynamics, propulsion, and inertia variable inventories
and records that alpha/beta are state or environment inputs, the moment outputs
are static source outputs rather than control derivatives, and the propulsion
member exposes axial thrust without a gimbal or thrust-vector input. The audit
therefore fails closed and lists the exact gimbal/effectivity, actuator, and
event-aligned attitude data required before a parent comparison or physical
allocator can be attempted.

Hummingbird already has an executable physical rigid-body/rotor allocation
path and a separate 3DOF mission path. It now also has a stateful aggregate
thrust-vector pseudo runner. That runner exercises attitude, yaw, translation,
thrust, and battery response, but it is not yet an automatic lowering of the
full mission compiler and it does not claim motor-level allocation. The
`tools/validate_hummingbird_fidelity_ladder.py` packet now supplies an
independent truth-evaluated development mission: takeoff and altitude gate,
yaw gate, four ordered hover-box corners, a declared one-time velocity
disturbance and recovery dwell, touchdown/contact, and a post-contact
motor-shutdown settle. Its point-mass artifact is explicitly generated by an
independent translation-only force model; it does not inherit attitude or yaw
behavior from the pseudo run. The packet maps the
semantically common objectives to the existing native rigid-body pad-to-pad
packet; the new disturbance objective is explicitly recorded as unmapped
rather than treated as a disagreement. All mapped objectives agree, and the
native packet supplies independent contact and rotor-runtime evidence. This
is a semantic parent comparison only: the two runtimes use different
controllers, horizons, and sample rates, so no trajectory-time equivalence is
claimed. A separate Euler-bounded native hover parity report remains attached
as a shorter translational convergence witness. The native/pseudo comparison
now reconciles mapped objective status, source-derived hover thrust, contact,
shutdown, and bounded pseudo-battery evolution. It explicitly records that
the native packet has no electrical battery/SOC model, so the resource result
is partial rather than a fabricated energy comparison. Direct translated-
flight comparison against individual-rotor allocation remains the next
Hummingbird gate.

These are integration checkpoints, not yet family-wide promotion. The X8,
B747, A320, F-16, Hummingbird, and X-15 profiles now carry
`nominal_case_pass`
because their 3DOF and named pseudo-6DOF missions pass independent truth gates
at the declared operating points. Hummingbird's nominal scope is explicitly
the hover-box/yaw/contact response-law case; it does not imply rotor allocation
or translated-flight qualification. X-15's promotion is likewise limited to
the staged event/energy/impact witness and does not imply a controlled
approach or historical flight-control reconstruction. These are nominal-case
promotions only; no profile is family-qualified until its operating-point,
robustness, and cross-fidelity gates are complete.

The lowering report is deliberately separate from profile discovery. A
catalog binding says that a reduction is structurally available; only a
qualified evidence record can make it selectable. This distinction keeps
automatic lowering useful for family integration while preventing a
development-only pseudo model from being presented as a qualified fallback.

`tools/build_alpha3_readiness_report.py` now produces
`verification/alpha3_robustness_matrix/manifest.json`. This is an evidence
index, not a robustness simulator. It records, for each paired family, the
nominal 3DOF/pseudo status, cross-fidelity status, attached parent or R1
artifacts, required fixed-matrix cases, and the next executable gate. The
current matrix has nine paired families, eight nominal pairs authorized for
automatic lowering, eight controlled/reduced R1 matrices (A320, X8, B747,
F-16, HL-20, Hummingbird, X-15, and NESC), one passive shape/uncertainty
matrix for tumbling bodies, one physical-surface R1 boundary witness (F-16),
and no family is allowed to infer physical-effector qualification from these
reduced records. The index therefore prevents a nominal pass from silently
becoming a robustness badge or a physical-effector claim.
The F-16 reduced matrix contains six passing initial-condition witnesses and
two retained altitude-boundary failures. The HL-20 matrix contains eight
passing passive/open-loop release witnesses. The Hummingbird matrix contains
eight passing aggregate-thrust/force-model witnesses. The X-15 matrix retains
explicit deployment events but still stops short of a controlled terminal
handoff. The NESC matrix varies only the declared composite response-law lag
and initial attitude while replaying the retained source stage/mass/translation
history; it does not invent gimbal effectiveness. None of these results
promotes a physical-effector or source-exact controlled-flight claim.
The tumbling-body matrix is complete only as passive evidence: its pseudo tier
reuses the native rigid-body equations and its 3DOF tier uses the declared
area policy. It does not create a controller or a prescribed-tumble claim.
It also indexes the existing local physical-effector artifacts separately:
B747 and Hummingbird have local T5 nonlinear witnesses, while X8 now has a
passing T4 source-coordinate witness and a four-pass local physical R1 matrix;
its mapped left/right end-to-end racetrack promotion remains pending. The new X8 mapping witness exercises both algebraically
reversible left-plus/right-minus and left-minus/right-plus hypotheses, records
the source-paper equation and the alternate convention conversion, and selects
only the source-paper mapping. These local records do not
promote the paired 3DOF/pseudo mission or imply envelope-wide qualification.
The B747 now also has a fixed local physical-surface R1 matrix: three interior
condition-3 witnesses pass through elevator/aileron/rudder/throttle allocation,
while one coupled-rate authority boundary is retained as a classified failure.
This is local physical evidence, not transport-envelope or gain-schedule
qualification.
Hummingbird now has the analogous individual-rotor R1 matrix: three local
hover attitude/rate witnesses pass through four motor-speed commands and motor
lag, while one rotor-authority boundary is retained as an infeasible case.
Translated-flight, contact, and battery/resource claims remain separate.
The F-16 source-effector path now has a fixed local physical R1 matrix. It
exercises the source nonlinear plant, finite-difference control effectiveness,
bounded elevator/aileron/rudder/throttle allocation, and actuator overlays.
A reusable generic profile sweep selected the state-and-wrench-balanced
`q10/r0.01` profile; all five local witnesses recover with zero saturation.
This is still local physical-effector evidence, not scheduled control,
statistical reliability, or full-envelope flight-control qualification. A
four-node source-effector schedule packet now covers sea level, 3 km, 6 km,
and 9 km at the catalog airspeed: all twenty local witnesses pass with rank-4
effectiveness and zero saturation. Adjacent gain changes are recorded, and
the generic interpolated wrench-demand contract is exercised at adjacent
midpoints. Fifteen additional probes pass physical allocation at both
endpoints of every adjacent interval. The packet still does not claim
time-marching nonlinear continuous transition replay. The shared
``run_scheduled_physical_wrench_transition`` contract now owns that replay:
it evaluates the scheduled demand at committed truth samples, allocates
through bounded physical effectors, holds the accepted actuator state over
the RK4 interval, and records allocation status and realized residuals. The
F-16 transition witness uses this generic runner and passes four
bidirectional perturbation cases with zero saturation and only feasible
allocation statuses. Its family adapter explicitly blends validated endpoint
source derivatives/effectiveness, so it remains local schedule-transition
evidence; it does not yet establish wind, statistical, or full-envelope
qualification. Future family integrations should supply a plant factory and
environment policy to this runner rather than copy a new time marcher.
The companion ``validate_f16_physical_schedule_envelope.py`` packet now
probes alpha, beta, and body-rate authority at all four nodes. It records 20
passing local cases and 12 partial-authority boundary cases, concentrated at
the higher-altitude nodes. This is the declared local authority boundary, not
a full Mach/dynamic-pressure, wind, or statistical envelope.
The B747 source bundle is now exercised through the same schedule-node intake
path. Eight nominal CR-2144 condition nodes (FC3--FC10) are materialized from
the preserved CSV control grids in isolated temporary decks and passed through
the runtime trim/linearization/allocator seam. FC3 reproduces the existing
physical-surface witness. A bounded local re-trim now allows only the body-w
velocity/alpha coordinate to move inside each source deck's published +/-4
degree offset domain; it keeps the source speed, altitude, physical controls,
and table bounds explicit instead of silently changing the source row. FC4--FC7
close source force/moment trim but remain controller-authority boundary nodes
because the current generic local LQR candidates do not pass every interior
surface witness. FC8--FC10 remain blocked by source trim closure inside the
declared alpha domain. No B747 gain interpolation is emitted. This is useful
integration evidence: source metadata is not being mistaken for a runnable
scheduled plant, and trim, controller-authority, and table-domain blockers are
attached to the node rather than discovered during a mission run.
The node initializer now carries each source row's ``alpha0_deg`` into the
initial body attitude relative to the *derived* FC3 body-velocity alpha, not
the rounded FC3 label. This removes the prior condition-mismatch artifact
without changing the FC3 regression oracle. The remaining nodes fail closed
at the appropriate layer: FC4--FC7 are physical-surface authority boundaries
and FC8--FC10 are source-trim boundaries. This is an evidence improvement, not
a schedule promotion.
The resolver also requires the pseudo-6DOF evidence and the parent 3DOF
evidence together: an attitude response result cannot certify the underlying
translational/resource model by itself.
Each catalog profile now declares its control realization as well. Ordinary
pseudo profiles are `response_law`; the tumbling-body exception is explicitly
`rigid_body_6dof` because it reuses the native rigid-body equations. A
direct-wrench or physical-surface claim cannot be inferred from a pseudo
profile’s existence.

The passive-body deployment witness is also wrapped by
`tools/validate_tumbling_body_fidelity_ladder.py` into the common family
evidence schema. Its 3DOF record is the averaged-area translation reduction;
its pseudo record is explicitly native rigid-body reuse, because a passive
tumbler has no control law to lower. Both records preserve parent/child
lineage and impact classification, and now carry `nominal_case_pass` status
for the declared passive deployment contract. The family remains
qualification-pending for parent capability and source-exact footprint
claims; those are explicit claim boundaries, not missing controller
evidence. A passive body has no controller or actuator allocation to qualify.

`verification/alpha3_fidelity_evidence.yaml` is now the checked promotion
manifest. Its records reference the generated X8, B747, A320, F-16, X-15,
Hummingbird, HL-20, and NESC truth-evidence packets; the loader accepts a
record only when the artifact exists, reports the declared status, passes its
mission evaluation, and passes its hard runtime gates. The automatic resolver
consequently selects pseudo-6DOF for those eight scoped cases and leaves only
the passive tumbling-body controller path unavailable by design.

`tools/validate_alpha3_fidelity_ladder.py` now builds the paired-tier index at
`verification/alpha3_fidelity_ladder/manifest.json`. The index covers all
nine target families and verifies that each has a 3DOF record and a
pseudo-6DOF or rigid-body-reuse record. It reports development witnesses as
passable artifacts without granting them automatic-lowering authority.

`tools/build_alpha3_cross_fidelity_reports.py` consumes that index and emits
one semantic comparison for every family under
`verification/alpha3_cross_fidelity/`. Each report records objective-ID
preservation, truth-event-time and margin deltas when available, mission-pass
agreement, and the two control-realization boundaries. A `pass` comparison
means the paired artifacts agree on the declared semantic mission; it does
not promote a development packet or imply physical effectors that the model
does not contain.

The legacy reduction-parity harness now accepts `--families`, `--integrator`,
and `--max-steps`. This makes stiff family windows reproducible and bounded;
for example, the Hummingbird native hover parent parity is recorded with an
explicit Euler policy rather than allowing the default adaptive integrator to
stall in a bridge-half diagnostic. That result is a translational hover parity
witness, not a full mission or rotor-control qualification.

The overview renderer now carries the control-realization label into its
scenario contract and board header. A bounded diagnostic render is available
with `--max-steps 200`; it emits all four boards, the overview, and a manifest
while retaining each runtime-limited artifact. The manifest marks every tier
as `incomplete: runtime-incomplete` and declares that the pack is diagnostic,
not qualification evidence. The long B747 pseudo bridge still reaches the
runtime step limit in the bounded window, so the next rendering slice must
use a justified family-specific horizon and preserve completion status rather
than treating a truncated plot as a pass.

### Current family readiness matrix

| Family | 3DOF | Pseudo-6DOF | Current evidence boundary | Next executable gate |
|---|---|---|---|---|
| X8 | nominal racetrack pass | nominal racetrack pass | source-coordinate T4 plus four-pass local physical R1; full surface racetrack fail-closed at uncontrolled beta table boundary | add a source-supported yaw realization or redesign the route around a proven valid envelope, then run mapped-surface end-to-end evidence |
| B747 | nominal racetrack pass | nominal racetrack pass | transport-scaled response surrogate; FC3 physical surface node, FC4--FC7 authority boundaries, FC8--FC10 source-trim blockers | close FC8--FC10 source trim and improve FC4--FC7 authority, then schedule interpolation and arrival-scale comparison |
| A320 | pass | nominal pseudo pass | calibrated reduced response runner; operating-point R1 complete | add throttle/physical-control evidence and broader envelope comparison |
| F-16 | pass | nominal pseudo pass | source-derived reduced response runner; conservative schedule-wide physical interior now qualified | broaden beyond the validated alpha interior into beta/high-rate and higher-dynamic-pressure envelope witnesses |
| X-15 | scoped staged event-chain, high-energy-corridor, and impact witness | same event/energy/impact witness with phase-scheduled response law | source-bounded surrogate; phase-scheduled response telemetry; selective native rigid replay; no controlled handoff | controlled terminal handoff and source-backed physical-effector evidence |
| Hummingbird | mission path | aggregate pseudo runner | nominal only for the hover-box/yaw/contact scope; no motor allocation or translated-flight claim | sustained directional mission, common mission adapter, and physical rotor promotion |
| HL-20 | release/glide path | profile-bound source release | source force graph plus reduced response; no physical surface claim | source-bound trim and controlled bank/alpha/energy gate |
| NESC | verified bounded replay | executable composite surrogate | retained DAVE-ML source has no independent attitude channel; no gimbal claim | parent comparison and staged response qualification |
| Tumbling body | averaged-area reduction | rigid-body reuse | no prescribed-tumble pseudo claim | parent capability and source-exact footprint review |

## 2. Common fidelity contract

Every family must resolve through the same ladder:

```text
family variant + operating point
        ↓
trim / release contract
        ↓
3DOF translational plant
        ↓
named pseudo-6DOF or rigid-body realization
        ↓
achievability and resource accounting
        ↓
mission objectives and terminal contract
        ↓
paired evidence and cross-fidelity comparison
```

The shared pseudo profile contains:

- quaternion attitude and body rates;
- named roll, pitch, and yaw response laws;
- command, rate, acceleration, and envelope limits;
- response lag and damping parameters;
- control-intent mapping;
- requested versus achieved semantic commands;
- saturation and authority telemetry;
- parent 3DOF case identity;
- validity envelope and evidence grade; and
- explicit unsupported physics.

Showcase realizations now also carry an explicit `control_realization` value:
`force_model`, `response_law`, `direct_wrench`, or `surface_allocated`.
`surface_allocated` requires declared physical effectors, while
`direct_wrench` rejects an active-effector claim. This keeps the four practical
evidence tiers visible even when the underlying dynamics label remains
`rigid_body_6dof`.

The response law is not allowed to manufacture physical moment or effector
claims. A profile must declare one of:

```text
attitude_response_surrogate
source_derived_reduced_model
control_surface_surrogate
thrust_vector_surrogate
rigid_body_reuse
```

### Tumbling-body exception

Tumbling bodies do not receive a made-up attitude-response surrogate. Their
pseudo-6DOF realization is the actual rigid-body model because rotational
motion is the subject of the qualification.

Their 3DOF reduction uses:

```text
D = 1/2 * rho * V^2 * C_D * A_effective
```

where `A_effective` is explicitly selected as average projected area, a
steady-stage cross-section, or a geometry-dependent schedule. The reduction
may prove center-of-mass trajectory and average drag only; it cannot prove
tumble, moments, angular momentum, or orientation-dependent drag excursions.

## 3. Work packages

### A3-F0 — Contract and evidence foundation

Deliver:

- reusable 3DOF/pseudo-6DOF profile schema;
- profile resolver and parent-case fingerprint;
- response-law catalog;
- family applicability matrix;
- common comparison artifact;
- telemetry names for attitude, rates, limits, authority, resources, and
  unsupported channels;
- automatic lowering report that stops at the first missing prerequisite.

Exit criteria:

- a synthetic vehicle resolves and runs at both tiers;
- pseudo channels are labeled synthesized or physical by provenance;
- a mismatched parent case fails closed;
- 3DOF/pseudo comparison contains event, terminal, energy, and resource deltas.

### A3-F1 — Fixed-wing reference ladder

Use the X8 as the reusable reference, then apply the profile to B747, A320,
F-16, and X-15.

Deliver:

- shared racetrack/energy mission template;
- speed, altitude, flight-path, bank, pitch, and heading response laws;
- schedule interpolation and operating-point transition checks;
- cross-fidelity route and terminal evaluator;
- fixed-wing response showcase modules.

Order:

1. X8: close bank reversal, pitch reversal, altitude gates, and route proof.
2. B747: scale time constants, turn radius, climb/descent, and arrival gates.
3. A320: replace policy overlay with calibrated response parameters and broaden
   operating points.
4. F-16: add alpha/beta, load factor, high-rate, and energy limits.
5. X-15: schedule powered, coast, atmospheric, and terminal regimes.

Exit criteria:

- each vehicle passes its declared 3DOF mission;
- each pseudo case passes the same semantic mission or returns a classified
  capability failure;
- no pseudo case claims physical surface allocation without an allocator;
- response schedules are continuous at operating-point boundaries.

### A3-F2 — Rotorcraft and lifting-body ladder

Deliver Hummingbird and HL-20 profiles using family-specific response laws.

Hummingbird must exercise hover, vertical motion, forward/lateral/rearward
translation, yaw, disturbance recovery, and landing. Its pseudo model may use
aggregate thrust-vector response, but motor-level claims require rotor/motor
states. The current nominal packet is deliberately scoped to
hover-box/yaw/contact semantics with independent 3DOF translation and native
semantic comparison; the separate directional witness below now covers the
reduced-tier forward/rearward mission. Physical rotor promotion remains a
separate gate.

The directional pseudo command uses the explicit `body_euler` aggregate
thrust-vector frame. The pseudo plant maps the achieved roll/pitch/yaw through
the body-z thrust direction, while the legacy hover-box regression retains its
`world_euler` compatibility mode. This provides a useful debugging seam and
prevents the directional witness from silently treating body tilt as an
already-allocated world force; it still does not claim rotor-level allocation,
inflow, reaction torque, or physical moment closure.

The initial executable pseudo slice is `hummingbird_pseudo6dof.py`. It is
intentionally a response-and-resource witness rather than a motor allocator;
the common mission adapter and rotor-resolved comparison remain separate gates.

The separate `hummingbird_directional_translation_v1` witness now closes the
reduced-tier mission gap. It preserves the existing hover-box regression and
adds independent truth evaluation of altitude capture, hover settlement, a
body-forward leg, yaw scan, body-right leg, body-rearward leg, descent,
disturbance recovery, touchdown, and post-contact shutdown at both
`point_mass_3dof` and `pseudo_6dof`. Its board shows the route, altitude and
speed, yaw frame change, body-frame truth velocities, bounded thrust/resource
state, and the objective table. The point-mass record marks yaw as
`NOT_APPLICABLE`; the pseudo record uses the aggregate thrust-vector response
law. This is a directional mission witness, not physical motor promotion:
body-frame direction is resolved from truth velocity and the declared yaw,
while native rotor translation, body-frame thrust coupling, electrical
battery/SOC, and physical allocation remain explicit future gates.

The native `hummingbird_native_horizontal_translation_v1` witness now closes
the next physical-control slice without hiding that boundary. It reuses the
source-backed local rigid-body plant, plant-derived LQR, actual four-rotor
commands, motor lag, nonlinear source forces/moments, and bounded allocation.
Its independent truth objectives are forward, yaw, body-right, rearward, and
horizontal-return gates. The packet records no direct body-moment injection and
passes all five objectives. Four samples at the yaw transition are explicitly
classified `partially_achievable` because the demanded yaw moment reaches the
motor envelope; they are not silently converted to `feasible`. The native
mission pass therefore excludes only disallowed allocator states
(`infeasible`, `numerically_singular`, or `solver_failure`) and exposes the
authority boundary in its status and telemetry. This is a local horizontal
translation witness, not altitude/collective, battery/SOC, contact/landing,
wind, or full-envelope rotorcraft qualification.

The native `hummingbird_native_vertical_force_v1` witness now closes the next
physical rotorcraft slice. It uses the same source-backed local plant and
plant-derived attitude LQR, but extends the local effectiveness contract to
all six body-wrench axes. Collective authority is therefore differentiated
from the actual four rotor-speed effectors and allocated together with the
three attitude moments; no direct body-force or body-moment command enters the
plant. Independent truth evaluation requires a bounded position and vertical
speed capture for 0.8 s at climb, hover, descent, and return-hover gates.
The result is a native vertical-force witness, not yet a pad-to-pad landing
qualification: battery/SOC, contact, touchdown, wind, and full-envelope rotor
behavior remain open. The packet records the physical-control path and any
authority boundary rather than promoting a local collective result to a full
helicopter claim.

The separate `hummingbird-pad-to-pad-altitude-yaw-individual-rotor-v1` packet
now closes the end-to-end rotor-resolved mission slice. It composes the runtime
takeoff, hover, yaw, altitude-gated perimeter, descent, touchdown, and
post-contact phases with complete rigid-body handoff. The runtime is configured
with the individual-rotor source equations, bounded quad-X allocation, and
first-order motor lag; the aggregate/common-speed direct-wrench table bridge is
not used for this witness. Independent truth evaluation passes all thirteen
required objectives, including contact, post-contact settling, and motor
shutdown. The contact portion remains an explicit static-pad impulse/reaction
contract: landing gear, tire, ground-effect, electrical battery/SOC, wind, and
full-envelope rotorcraft behavior are still nonclaims.

HL-20 must exercise release, glide capture, bank reversal, crossrange, energy
dissipation, and terminal energy. Alpha, bank, and flight-path limits remain
explicit even when surfaces are not physically allocated.

The source-bound HL-20 control seam is now split into three explicit layers:

```text
pinned DAVE-ML loads
    → centered local source effectiveness matrix
    → shared bounded allocator
    → declared surface position/rate/lag realization
    → source-load replay
```

`tools/validate_hl20_source_allocation_probe.py` records the matrix source,
allocation status, commanded versus achieved surface positions, realized
wrench residual, and retained infeasible stress requests.  It does not turn
the allocation matrix into a controller, trim solver, or trajectory model.
This is the reusable integration pattern intended for later source-backed
lifting-body and spacecraft-effectors adapters.

The supplemental `source_surface_replay` witness now exercises the next
runtime seam: explicit seven-surface commands enter the pinned DAVE-ML load
graph and native rigid-body integration with `direct_body_moment_injection =
0`.  It is intentionally short and open loop, uses a declared source-valid
authority-boundary command, and does not claim trim, stabilization, bank/
alpha/energy guidance, or family-envelope qualification.

The current `tools/validate_hl20_fidelity_ladder.py` packet now records paired
3DOF/pseudo reduced witnesses, independent phase/impact evaluation, and a
terminal cross-fidelity comparison. It is intentionally `development`: the
reduced witness uses source-bound geometry and mass but a synthetic booster and
surrogate aerodynamic branch, while the separate source-DAVE-ML run still
exposes an invalid-state/trim blocker. Promotion requires the source-bound
bank/alpha/energy mission to pass without that substitution.

### A3-F3 — Staged rocket ladder

Deliver the NESC pseudo profile with:

- staged mass and resource continuity;
- powered attitude response;
- gimbal/thrust-vector surrogate boundaries;
- separation settling;
- deployment attitude/state transitions;
- coast and terminal event handling.

The current executable slice is `nesc_pseudo6dof.py`: it preserves the
verified source translation/staging history and applies the bounded attitude
response profile. Its artifact is a scoped `nominal_case_pass` for
`nesc.source_translation_staged_orbit_coast_v1`, not source-exact
qualification.

`tools/validate_nesc_fidelity_ladder.py` now packages that source history and
the pseudo bridge as paired 3DOF/pseudo evidence with a translation-row
comparison. Both scoped nominal records pass their structural truth checks,
while the manifest explicitly blocks source-exact promotion on independent
parent attitude evidence and physical gimbal allocation.

The 3DOF profile remains authoritative for mass flow, staging, range, and
target-state behavior. Pseudo-6DOF adds bounded attitude behavior without
claiming source-exact gimbal dynamics unless those data are available.

### A3-F4 — Passive and tumbling-body ladder

Deliver:

- geometry-aware effective-area policy for 3DOF;
- full rigid-body reuse through the common pseudo-6DOF interface;
- damping/no-damping comparison;
- projected-area, drag, moment, angular-energy, and angular-momentum evidence;
- cross-fidelity error report showing what the averaged-area reduction loses.

Exit criteria explicitly prohibit a pseudo profile that prescribes tumble while
claiming moment-driven behavior.

The current reachability implementation enforces this boundary for passive
detached bodies: point-mass propagation uses a deterministic
orientation-averaged projected area, while a requested pseudo-6DOF child
reuses the native rigid-body state and is reported as realized
`rigid_body_6dof` with the requested pseudo tier retained in the artifact.
This prevents a kinematic tumble surrogate from being mistaken for
moment-driven rotation.

The existing `tools/validate_passive_deployment.py` packet is the current
development evidence source for this exception. It covers sphere, cylinder,
cone, and triaxial-ellipsoid geometry, compares averaged-area 3DOF against
native rigid-body pseudo reuse, and records projected area, drag, moments,
angular rates, lineage, uncertainty samples, and an explicit passive
aerodynamic-moment versus zero-moment spin diagnostic. It remains a
passive-body qualification witness rather than a controller or parent-vehicle capability
claim.
The readiness report therefore counts this as `passive_pair_ready` when both
the averaged-area 3DOF and native rigid-body-reuse pseudo records pass their
declared passive matrix. That category is intentionally separate from
`nominal_pair_ready` and never authorizes automatic controller lowering.

The catalog records the native rigid-body reuse profile as
`nominal_case_pass`, not `development`: the requested pseudo tier is therefore
solid at its declared passive scope.  `automatic_lowering` remains `false`
because lowering is a controller-selection operation and a passive body has no
controller to synthesize.  A request for this pseudo tier must resolve to the
native rigid-body realization or fail closed; it must never fall back to an
invented attitude-response law.

The paired `comparison.json` now reports the measured terminal and
projected-area loss from the averaged-area reduction, and the common Alpha 3
cross-fidelity report links that loss record without turning it into a scalar
quality score. The passive family therefore has an explicit geometry/rate/
area-policy comparison; its remaining blocker is source-exact passive
aerodynamics or parent-capability evidence, not an invented controller.

### A3-F5 — Cross-fidelity qualification and release

For every family, run:

1. source/provenance and data-readiness checks;
2. trim, hover, release, or separation checks;
3. 3DOF nominal mission;
4. pseudo/rigid nominal mission;
5. timestep convergence;
6. batch/step parity;
7. fixed perturbation witnesses;
8. cross-fidelity comparison;
9. showcase/artifact regeneration; and
10. claim and nonclaim review.

The release matrix must contain separate statuses for intake, execution,
qualification, and promotion. A passing 3DOF case never promotes a failed
pseudo case.

### A3-F6 — Direct-wrench 6DOF depth

The direct-wrench tier is the declared intermediate between a pseudo-6DOF
response law and physical effector allocation:

```text
guidance error
    → attitude/rate controller
    → desired body-frame six-axis wrench
    → bounded direct-wrench projection
    → nonlinear rigid-body load composition
    → 6DOF integration
```

It is a valid rigid-body control-screen tier, but it is not an elevon, rotor,
gimbal, wheel, or thruster claim. Every direct-wrench run must expose:

- canonical force/moment ordering and SI units;
- lower, upper, and slew authority limits for all six axes;
- requested, achieved, and residual wrench;
- position-saturation and rate-limiting flags;
- source aerodynamic and propulsion loads separately from injected control
  force and moment;
- `control_realization: direct_wrench` and
  `physical_effector_allocation: false`; and
- trim, nonlinear perturbation, force/moment closure, resource, timestep, and
  cross-fidelity evidence.

The reusable implementation is `taoryx.direct_wrench`.  Its bounded
projection is intentionally distinct from `allocate_and_advance_wrench`: the
former limits an already generalized wrench, while the latter solves for
physical effector positions and actuator dynamics.  A direct-wrench result
must therefore never satisfy a physical-effector promotion gate by itself.

Execution order:

1. establish the common contract and synthetic bounded-projection tests;
2. complete the X8 six-axis direct-wrench vertical slice;
3. reuse it for B747, A320, and F-16 operating-point/schedule evidence;
4. add phase-aware X-15, HL-20, and NESC surrogate wrench paths;
5. expose a Hummingbird direct-wrench debug path beside the native rotor path;
6. retain tumbling bodies as native uncontrolled rigid-body evidence rather
   than adding a controller; and
7. regenerate paired showcases and the release matrix.

The first milestone is complete only when a synthetic plant and X8 both show
bounded six-axis demand, explicit residuals, and nonlinear replay without
physical-effector claims.

Current direct-wrench tranche:

- X8: nominal local direct-wrench bridge witness passes through the table-backed
  nonlinear plant; it remains a one-point Tier-3 realization.
- B747: the shared transport racetrack direct-wrench packet passes its four
  independent truth gates; it remains a nominal source-table/direct-moment
  realization with no physical surface, fuel, or landing claim.
- A320: the pinned OpenAP/JSBSim surrogate now has a bounded local
  six-axis response witness; it is not yet a full rigid-body mission.
- F-16: the racetrack direct-wrench route remains an honest failed-route
  boundary artifact, while the source-plant local direct-wrench witness
  passes. The local witness is the current Tier-3 evidence; the route is not
  promoted by it.
- X-15: a local release/glide source-load direct-wrench bridge now passes with
  an explicit load-balance bias and numerically bounded LQR demand. The
  source-bounded trim gate still fails, so this remains scoped T3 bridge evidence;
  reproduce it with `PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3
  tools/validate_x15_direct_wrench_local.py`.
- HL-20: source surface-direction, source-load allocation, open-loop surface
  replay, and a local T3 source-load direct-wrench bridge are available. The
  source-surface path is preferred; the direct bridge explicitly uses a
  source-load balance bias and is not promoted until source-bound trim and
  controlled bank/alpha/energy evidence exist. Reproduce it with
  `PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3
  tools/validate_hl20_direct_wrench_local.py`.
- NESC: the source translation/staging replay has no declared gimbal,
  thrust-vector, force, or attitude input. A translation-derived local
  direct-wrench bridge now exercises the common seam using declared engineering
  inertia and an explicit ECI/body alignment assumption. It is scoped T3
  engineering-bridge evidence; reproduce it with
  `PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3
  tools/validate_nesc_direct_wrench_local.py`. It does not invent source
  gimbal authority.
- Hummingbird: a bounded direct-wrench bridge comparator runs against the same local
  RotorPy-derived plant, but the native four-rotor allocation remains the
  promoted 6DOF path. The comparator is indexed at
  `verification/alpha3_hummingbird_direct_wrench_debug/manifest.json` and is
  explicitly Tier-3 bridge evidence; the native rotor path remains preferred
  for physical-effector claims.

The next direct-wrench tranche must preserve this distinction: every family
gets a local six-axis witness before a mission packet is promoted, and a
mission pass never erases a missing effector, resource, trim, or schedule
claim. The family matrix indexes local witnesses separately from R1
robustness so direct-wrench evidence cannot be mistaken for physical-control
or statistical robustness evidence.

The regime boundary is generated by
`tools/build_alpha3_regime_direct_wrench_readiness.py` and recorded in
`verification/alpha3_regime_direct_wrench_readiness/manifest.json`. Its
`blocked_or_debug_count` is intentionally nonzero: it is the evidence that
the common seam refuses to convert missing trim or missing effectivity into a
false 6DOF claim.

The cross-family contract audit is generated by
`tools/build_alpha3_direct_wrench_contract.py` and recorded in
`verification/alpha3_direct_wrench_contract/manifest.json`. It is the
authoritative normalized ledger for trim/release, force/moment composition,
resource behavior, envelope, and claim boundaries. It records the X-15 and
NESC local screens without promoting their missing source trim/effectivity,
and records tumbling bodies as native uncontrolled rigid-body evidence.

For the F-16 physical schedule, the conservative interior qualification is
computed as the intersection of cases passing at every source-retrimmed node.
The current interior is the pair of ±1 m/s alpha witnesses across the four
validated altitude nodes. Beta and high-rate cases remain boundary evidence in
`verification/alpha3_f16_physical_schedule_envelope/manifest.json`; the
derived interior record is written by
`tools/qualify_f16_physical_schedule_interior.py`. This is a local
schedule-wide promotion, not a full-envelope claim.

## 4. Standard evidence gates

### 3DOF gate

- finite, in-domain trajectory;
- valid trim/release/start contract;
- force, propulsion, mass/resource, and environment channels;
- objective and terminal closure;
- no attitude or physical-effector claim;
- numerical replay and timestep evidence.

### Pseudo-6DOF gate

- all 3DOF gates;
- quaternion norm and body-rate continuity;
- named response law and parameter provenance;
- command/response tracking and limit telemetry;
- bounded attitude/rate behavior;
- translation parity window against the parent 3DOF case;
- explicit synthesized-versus-physical channel classification.

### Tumbling rigid-body gate

- all 3DOF reduction gates where applicable;
- positive inertia and rigid-body propagation;
- force/moment closure;
- orientation-dependent projected area and drag;
- angular momentum and energy evidence;
- damping/no-damping witness comparison;
- no prescribed-tumble substitution for physical proof.

## 5. Execution artifacts

Each family produces:

```text
verification/fidelity-ladder/<family>/
├── 3dof-evidence.json
├── pseudo6dof-evidence.json
├── comparison.json
├── telemetry-3dof.csv
├── telemetry-pseudo6dof.csv
├── showcase-3dof.png
├── showcase-pseudo6dof.png
├── cross-fidelity-board.png
├── runtime_diagnostics.json
└── reproduction.txt
```

Every displayed metric must be recomputable from these artifacts and tied to
the same resolved variant, mission, environment, and source hashes. When a
runtime fails before producing truth telemetry, `runtime_diagnostics.json`
and the qualification board must preserve the exact diagnostic; an empty
telemetry file is a classified integration boundary, never an implicit
objective pass.

The shared showcase-archetype catalog now contains explicit recipes for all
nine target families. X8, Hummingbird, and the passive tumbling body are
family-local additions rather than being forced into a generic transport or
controller story: the X8 recipe names its beta-domain boundary, Hummingbird
names yaw/contact/shutdown evidence, and the tumbling recipe names passive
rotation and impact with no-controller semantics. Recipe presence is a
composition contract; it does not promote a development evidence packet to a
qualified physical-effector claim.

The current X-15 staged evidence is also rendered by
`tools/render_x15_fidelity_board.py` into
`verification/x15_fidelity_ladder/x15_fidelity_evidence_board.png` with a
hashed manifest. The board consumes the paired 3DOF and pseudo-6DOF telemetry
packets directly and makes the staged boost, cutoff, coast, glide, energy,
open-loop atmospheric handoff gate, and impact claim visible. It carries the
explicit boundary that controlled terminal handoff and native X-15 effectors
remain unqualified.

The passive tumbling-body evidence now has the same standardized board through
`tools/render_tumbling_body_fidelity_board.py`, consuming the nominal cylinder
3DOF/pseudo telemetry and the four-shape comparison record. Its manifest keeps
the no-controller boundary explicit: the 3DOF path uses averaged projected
area and the pseudo path reuses native rigid-body rotation.

`tools/validate_hummingbird_directional_mission.py` is included in the normal
`alpha3-readiness` regeneration. Its manifest is indexed in the readiness
matrix as a directional translation witness and does not change the
Hummingbird physical-effector tier.

The rotor-resolved Hummingbird pad-to-pad packet is generated with:

```bash
MPLCONFIGDIR=/tmp/taoryx-mpl PYTHONPATH=.:src \
  python3 tools/build_hummingbird_pad_to_pad_packet.py \
  --output verification/alpha3_hummingbird_native_pad_to_pad \
  --individual-rotor-source
```

It is indexed as supplemental evidence rather than replacing the legacy
aggregate runtime packet. This keeps the evidence ladder honest: the packet
proves individual rotor-source equations, bounded quad-X allocation, motor lag,
and static-pad contact, but does not silently promote battery/SOC, landing gear,
tire, ground-effect, wind, or full-envelope behavior.

`tools/validate_alpha3_showcase_catalog.py` now provides the catalog-level
artifact gate. It binds all nine target family IDs to their recipe, paired
fidelity records, passing cross-fidelity report, and canonical board, and emits
`verification/alpha3_showcase_catalog/manifest.json`. Its successful status is
`development_catalog_verified`; it is deliberately not a family-qualification
or physical-effector promotion badge.

## 6. Priority and promotion policy

The first vertical slice is X8 because it establishes the reusable fixed-wing
pattern. B747 and A320 test scaling; F-16 tests high-rate envelope behavior;
X-15 tests regime scheduling. Hummingbird, HL-20, and NESC then exercise
family-specific response laws. Tumbling bodies close the passive-body
exception.

The program may report a family as `development_ready_with_gates` while a
known response or effectivity source remains surrogate. It may not report
`promoted` until all declared gates for that tier pass.

## 7. Definition of done

The workstream is complete when the common resolver can produce paired 3DOF
and pseudo/rigid cases for all target families, every run has explicit physics
and nonclaims, every family has a characteristic mission and comparison pack,
and the repository validation gates pass.
