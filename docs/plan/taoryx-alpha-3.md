# TAORYX Alpha 3: vehicle breadth and qualification expansion

**Status:** Active; passive aero-ballistic deployment tranche qualified
**Predecessor:** Alpha 2 closeout contract  
**Boundary:** Add and qualify new vehicle archetypes without weakening the
common runtime, family-package, control, or evidence contracts.

**Reprioritization:** The breadth work below follows the P1/P2 common
compose-to-run airbreather slice in
[authoring-runtime-composition-execution-roadmap.md](authoring-runtime-composition-execution-roadmap.md).
New-family work may maintain evidence and conformance witnesses in the
meantime, but it must not create another bespoke scenario-to-run path.

The [Family Showcase Composite Master Plan](taoryx-family-showcase-composite-master-plan.md)
is the governing execution plan for Alpha 3 flagship packs, standard evidence
boards, family-local event markers, and cross-fidelity qualification.

The active implementation sequence for the 3DOF/pseudo-6DOF breadth goal is
the [Alpha 3 Fidelity-Ladder Execution Plan](alpha3-fidelity-ladder-execution.md).
It extends this roadmap with the X8-first fixed-wing ladder, the
Hummingbird/HL-20/NESC family profiles, and the averaged-area versus
rigid-body treatment for tumbling bodies.

Alpha 2 must complete the [future-family interface stress test](future-family-interface-stress-test.md)
before this breadth work begins. That is a contract-closure gate, not a
requirement to implement the future families early.

## Why Alpha 3 exists

Alpha 2 should finish the reusable platform around the four established proof
families: B747, Skywalker X8, Hummingbird, and X-15. It should not also try to
finish the F-16/HL-20 source anchors, passive-body geometry tranche, or every
new aircraft, rotorcraft, spacecraft, surrogate, corpus, and fleet idea now in
the backlog.

Alpha 3 is the breadth and qualification release. Its purpose is to prove that
the Alpha 2 contracts extend cleanly to new families while preserving explicit
claim boundaries. A new family is not complete merely because it has a JSON
deck or can take one numerical step.

Alpha 3 consumes the Alpha 2.1 bounded-variant contract. New families must be
constructed by resolving an immutable variant, not by mutating a family deck
or copying a vehicle-specific runner. The Alpha 3 question is:

```text
Can one bounded, provenance-linked semantic variant drive
3DOF, pseudo-6DOF, and rigid-body 6DOF bindings with
family-appropriate resources, controls, feasibility, and evidence?
```

## Alpha 2 finish line

Alpha 2 is complete when all of the following are true:

1. The common scenario/evaluation substrate is the only scoring and evidence
   route for batch, interactive, and showcase runs.
2. The session API supports reset, step, run, pause/resume, interrupt,
   deterministic replay, and checkpoint/restart through one transition.
3. The onboarding/convention firewall accepts a new family through metadata,
   source records, tables, adapters, trim, closure, and bounded propagation,
   with actionable diagnostics.
4. The four established families have reproducible, family-appropriate
   flagship missions with source/evidence boundaries, event markers, achieved
   controls, envelope margins, closure, convergence, and objective scores.
5. Plant-truth trim and normalized gentle/standard/aggressive controller
   presets work through the common control contract; LQR remains a selectable
   baseline rather than a permanent architecture.
6. Checkpointed and uninterrupted runs agree within declared tolerances.
7. The four families reach at least M4 Composable for their supported claims;
   no global M5/M6 or real-aircraft validity claim is implied.
8. Alpha 2 does not require the F-16/HL-20 reference anchors or passive-body
   geometry tranche. Those are Alpha 3 inputs and must not block the Alpha 2
   release signal.

Alpha 2 explicitly does not require implementing the new helicopter, tiltrotor,
small-aircraft, spacecraft, surrogate, or source-anchor families. Their intake
records and source hashes may be present, but their runtime qualification
belongs to Alpha 3.

## Alpha 3 workstreams

### A3-W0 — Variantized family bindings and achievable execution

Before adding broad vehicle data, bind the Alpha 2.1 resolver to the first
new families and make the result useful to segments and controllers. This
workstream adds the parts intentionally deferred from Alpha 2:

- coupled mass, propulsion, energy, and resource ledgers;
- capability envelopes and an achievable-command projector;
- approximate preflight feasibility classification;
- structured segment outcomes such as `COMPLETED`, `PARTIAL`,
  `RESOURCE_LIMITED`, `ENVELOPE_LIMITED`, and `INFEASIBLE_PREFLIGHT`;
- fidelity-specific parameter bindings for 3DOF, pseudo-6DOF, and 6DOF; and
- family-specific modifiers with explicit evidence grade and requalification.

The resolver remains backend-neutral. A segment consumes achieved response and
resource state rather than assuming that a requested acceleration, bank,
throttle, or rotor command occurred.

A3-W0 exit criteria:

- one powered fixed-wing and one rotorcraft/VTOL family resolve operational
  variants through the common path;
- the same semantic loadout produces paired 3DOF and pseudo-6DOF cases;
- resource depletion removes capability without creating energy or mass;
- infeasible missions return structured best-effort outcomes rather than
  numerical failures or false success;
- requested and achievable controls are separately recorded; and
- qualification status, projection distance, and requalification requirements
  survive into the run artifact.

### A3-W1 — Family package and maturity expansion

Implement the family package layout and onboard, in order:

1. C172P-class light propeller aircraft.
2. R44-class helicopter.
3. UH-1H scheduled helicopter.
4. UH-60A scheduled helicopter.
5. XV-15-class tiltrotor.
6. V-22-class scaled tiltrotor surrogate.
7. Learjet 24-class local business jet.
8. Spacecraft reference family.
9. Public-data flight-dynamics surrogate roster.
10. F-16 S-119 and HL-20 Mod K source-grounded reference anchors.
11. Passive/tumbling deployable-body geometry tranche.

Each package progresses independently through M0–M4. A weak package must not
inherit the maturity badge of a related family.

### A3-W2 — Hybrid mode transitions

Qualify a reusable transition contract for helicopter and tiltrotor modes:

- entry and exit guards;
- continuously scheduled force, lift-sharing, and mixer parameters;
- state, resource, and controller handoff;
- hysteresis and chatter prevention;
- actuator and nacelle/rate limits;
- abort/reversion;
- forward and reverse conversion;
- batch/step equivalence.

### A3-W3 — Source and model correlation

Add source differential checks and progressively stronger validation:

- R44 identified hover response;
- UH-1H source-point matrices, then nonlinear TM-73254 components;
- UH-60A source-point matrices, then GenHel/Airloads comparisons;
- XV-15 conversion and handling-quality references;
- C172 same-class performance and dynamic-response anchors;
- Learjet local modes and later Mach/configuration sources.

### A3-W4 — Breadth tooling

Add search/reachability, coherent parametric vehicles, trajectory corpora,
population/fleet execution, sensors, weather, and spacecraft qualification
only after A3-W0 and the family package/evidence contracts are stable. Search
may vary semantic parameters and segment/controller decisions, but it must not
edit raw table cells or bypass variant qualification.

The sensor work must use the [EOM timing and committed-truth
contract](../architecture/eom-timing-contract.md). IMU, estimator, and
multi-rate sensor development cannot begin by interpolating published vehicle
states or exposing RK solver stages as truth.

### A3-W4A — Bounded adaptive-control augmentation

Adaptive control is an Alpha 3 controller workstream, not an Alpha 2 exit
gate. It begins only after a vehicle has a validated scheduled or fixed
regulator, plant-derived trim and linearization, and a physically accountable
allocator. The first implementation should be a bounded augmentation around
the existing controller, not an unconstrained replacement:

```text
scheduled baseline
    -> bounded parameter/effectiveness estimator
    -> projected gain or feedforward update
    -> desired wrench
    -> physical allocator
    -> actuator limits and dynamics
    -> nonlinear plant
```

The common adaptive contract must declare:

- estimated parameters and their physical meaning;
- update law, update cadence, and required excitation;
- projection or normalization bounds;
- reset, freeze, and fallback behavior;
- interaction with gain scheduling and operating-point changes;
- anti-windup and actuator-saturation handling; and
- telemetry for estimates, updates, projection events, and achieved authority.

The first pilots should be one fixed-wing case (X8 or B747) and one
rotorcraft case (Hummingbird or R44). The adaptive layer must be tested against
the same nominal, perturbation, and near-limit cases as the scheduled
baseline. Required evidence includes:

- baseline-versus-adaptive tracking and authority comparison;
- boundedness when estimates are wrong or excitation is insufficient;
- actuator saturation and rate-limit behavior;
- update-law ablation showing that adaptation is responsible for the claimed
  improvement; and
- deterministic replay, batch/step parity, and explicit fallback events.

An adaptive run cannot promote a model past the evidence tier of its physical
plant or allocator. Unmodeled effectors, direct wrench injection, failed trim,
or unresolved sign conventions remain blockers regardless of adaptation.

### A3-W5 — Aero-ballistic deployment and spawned bodies

Promote stage separation from a parent mass adjustment into a first-class
accepted-boundary event. The canonical contract and release gate are defined
in [`alpha3-aero-ballistic-deployment.md`](alpha3-aero-ballistic-deployment.md).
The workstream covers:

- shared stage mass, propulsion-authority, separation, and detached-body
  definitions;
- parent pre/post event states and optional body-frame/inertial impulse;
- cylindrical spent rocket stages and ellipsoid spent tanks as ballistic
  child bodies;
- point-mass 3DOF, pseudo-6DOF tumbling, and separately gated rigid-body 6DOF;
- deterministic child initialization, mass accounting, event identity, and
  parent/child artifact visualization; and
- source-language lowering only after the shared semantic contract is stable.

A3-W5 is complete for the passive specialization: declared sphere, cylinder,
cone, and triaxial-ellipsoid children are initialized, propagated, terminated,
uncertainty-swept, and visualized with the same event and provenance identity
as their parent. Their physical terminal footprints remain separate from
parent capability claims. Active arbitrary-child deployment, including child
propulsion, guidance, and vehicle-specific configuration builders, is deferred
to Alpha 4.

### A3-W6 — Compatibility claim maintenance

Maintain the evidence-bounded TAOS 96.0 language/specification profile and its
machine-readable claim ledger. Historical runtime equivalence remains an
explicit nonclaim, not an Alpha 3 implementation target: it would require an
executable and complete historical table library, and reconstructed syntax or
modern numerical agreement is not a substitute.

## Alpha 3 exit gates

Alpha 3 can close only when:

- at least three new domain families reach M4 through the common onboarding
  route;
- at least two new families execute through an immutable resolved variant with
  coupled resources, achieved-control telemetry, and structured outcomes;
- one light fixed-wing, one conventional helicopter, and one tiltrotor have
  3DOF and pseudo-6DOF evidence with common scenario contracts;
- one source-grounded family reaches an independently reviewed source-dynamic
  comparison milestone;
- one hybrid transition family passes forward, reverse, abort, and replay
  tests;
- all promoted parameters retain source, derived, estimated, or unavailable
  labels;
- every promoted modifier declares hard and qualified bounds, transform,
  coupling, and retrim/requalification policy;
- breadth work has not introduced bespoke runners or bypassed the common
  evaluator;
- every release claim links to a machine-readable maturity and evidence record.
- if adaptive control is advertised, at least one fixed-wing and one
  rotorcraft pilot pass the bounded adaptive contract and publish a comparison
  against the validated non-adaptive baseline; adaptive control is not required
  for Alpha 3 completion of families that do not advertise it.

## Deferred beyond Alpha 3

Global flight validity, certification, proprietary aircraft reconstruction,
historical TAOS runtime equivalence without an oracle, and unrestricted claims for
post-stall, ground-effect, engine, rotor, or thermal behavior remain outside
the release definition. Large-scale multi-backend parameter search, ML corpus
generation, topology/geometry generation, advanced smooth aerodynamic morphing,
and fleet-scale optimization are Alpha 4 work rather than Alpha 3 exit gates.
The generalized deployment API for arbitrary active children, including
first-class child propulsion, guidance, and non-ballistic vehicle schemas, is
also Alpha 4 work rather than an Alpha 3 exit gate.

The Alpha 4 scale and backend plan is maintained in
[`taoryx-alpha-4.md`](taoryx-alpha-4.md).

## Immediate Alpha 3 execution tranche — B747 and X-15 showcase endpoints

The first showcase work should not be another four-family scoreboard. The
current composites are retained as integration baselines, while two
family-appropriate endpoint templates are rebuilt through the common
qualification path:

### A3-SHOWCASE-1 — B747 transport energy-management pack

Mission:

```text
airborne trim
→ climb to cruise
→ cruise stabilization
→ large-radius right turn
→ long cruise leg
→ second turn
→ managed descent
→ stabilized arrival gate
```

Required visual products:

- mission geometry: complete 3D/top-down/side/altitude-downrange path with
  family-local event markers;
- energy management: altitude, true airspeed, Mach, flight-path angle,
  specific energy, fuel remaining, throttle, lift, drag, and thrust;
- terminal arrival: along-track, cross-track, altitude, heading, vertical
  speed, airspeed, and bank in one simultaneous arrival contract; and
- envelope: alpha, lift, available thrust, fuel, Mach, table-domain, and
  minimum-margin evidence.

Exit criteria:

- both right and left large-radius turns are completed in the declared order;
- cruise altitude/speed dwell and managed descent are independently evaluated;
- the terminal gate passes all required dimensions for the required dwell;
- no runway takeoff/landing claim is made without low-speed/high-lift,
  gear/contact, brake, and steering evidence; and
- every displayed scalar is traceable to raw telemetry and the metric
  dictionary.

### A3-SHOWCASE-2 — X-15 boost-glide storyboard and lineage pack

Mission:

```text
carrier → release → ignition/boost → burnout/coast → apogee
→ atmospheric entry → glide capture → energy management → terminal handoff
```

Required visual products:

- mission storyboard with phase-colored trajectory;
- synchronized altitude, Mach, dynamic pressure, specific energy, mass,
  acceleration, vertical speed, and flight-path angle with event markers;
- vehicle lifecycle showing carrier, X-15, booster/glider roles and active
  intervals; and
- ordered event table for release, ignition, max-Q, burnout, separation,
  apogee, entry, glide capture, and terminal handoff.

When separation creates a detached body, the packet must include a typed
object-lineage graph with parent/child IDs, accepted event time, pre/post
states, active interval, and terminal disposition. A mass subtraction without
an auditable child is not lineage evidence. The aero-ballistic deployment
contract remains the source of truth for accepted-boundary spawning.

Exit criteria:

- event order is complete and deterministic;
- source-supported versus estimated phases are visibly separated;
- parent and detached-body mass/resource accounting closes exactly once;
- terminal handoff is multidimensional, not a timeout or loose position score;
- batch and stepwise runs agree at every event boundary; and
- the packet states explicitly whether the X-15 result is a source-bounded
  surrogate, a reduced model, or a rigid-body witness.

The common recipe and archetype IDs are frozen in
[`verification/showcase_archetype_catalog.yaml`](../../verification/showcase_archetype_catalog.yaml).
The typed runtime surface is `taoryx.showcase`, including
`ShowcaseArchetypeCatalog`, `ObjectLineage`, and
`ShowcaseRunArtifact.object_lineage`.

The first DaveML-family showcase tranche is now defined and executable in
[`daveml-family-showcase-tranche.md`](daveml-family-showcase-tranche.md). It
covers F-16, HL-20, NESC two-stage, the A320 derived-versus-surrogate split,
and a NESC-parent synthetic passive-child deployment witness. This tranche
does not promote the remaining NESC source-only atmospheric/orbital/tumbling
records into executable families.

## Next DaveML Tranche — Family Operational Qualification

The next long-running DaveML workstream is defined in
[`daveml-family-operational-qualification-tranche.md`](daveml-family-operational-qualification-tranche.md).
It closes the gap between a verified DAVE-ML round trip/showcase artifact and
an operational Taoryx family-library product:

1. typed family contracts and fail-closed provenance loading;
2. certified operating-point catalogs for F-16, HL-20, NESC, and both A320
   fidelity lanes;
3. applicable multi-point linearization and tuning, with explicit
   non-applicable dispositions for uncontrolled/open-loop sources;
4. clean-process objective and scenario runtime contracts; and
5. a deterministic cross-family release gate.

The tranche preserves source, derived, surrogate, synthetic, and nonclaim
boundaries. LaTeX, B747/X-15 parallel work, manufacturer-authoritative A320
source acquisition, and source-only NESC records remain out of scope.

The operational-contract and baseline operating-point artifacts are now part
of the DaveML release gate. The next increment expands the certified operating
point envelope rather than changing the source qualification classes.
