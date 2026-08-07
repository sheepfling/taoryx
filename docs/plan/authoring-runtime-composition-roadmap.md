# TAORYX Authoring → Runtime → Composition execution roadmap

**Status:** Active reprioritization decision  
**Scope:** successor language, executable simulation platform, and vehicle/mission composition  
**User-facing guide:** [Authoring → Runtime → Composition showcase guide](../AUTHORING_RUNTIME_COMPOSITION_SHOWCASE.md)
**Companion plans:** [TAORYX successor roadmap](taoryx-successor-roadmap.md),
[Composable scenario runtime](composable-scenario-runtime.md),
[Mission composition and vehicle-onboarding automation](mission-composition-automation.md),
[Vehicle-integration automation](vehicle-integration-automation-plan.md), and
[Horizontal vehicle integration](horizontal-vehicle-integration.md). The
Mission Composition maturity sequence is tracked separately in
[Mission Composition maturity plan](mission-composition-maturity.md).

## Decision

TAORYX is not one backlog.  It is three connected layers with different
definitions of done:

```text
1. Language successor
   typed .tbl/.prb source, validation, provenance, and explicit extensions

2. Simulation platform
   resolved plant -> accepted truth -> stepping/control -> artifacts/plots

3. Vehicle and mission composition
   discover -> configure -> compile -> bind -> run -> independently evaluate
```

All three must progress, but the immediate delivery priority is the seam
between Simulation Runtime and Mission Composition. A registry that can describe a vehicle but cannot
turn an accepted composition into a native run is useful infrastructure, not
yet the user-facing Mission Composition layer.

The next platform finish line is therefore:

> A caller can select a registered vehicle, declared fidelity,
> initialization contract, and mission template; TAORYX either produces a
> reproducible native run and truth-evaluated artifact, or fails before
> integration with a precise source, adapter, capability, or qualification
> blocker.  No generic force, moment, controller, or family fallback is
> permitted.

The first implementation scope is the common powered-fixed-wing racetrack:
Skywalker X8, B747, A320, and F-16. It is deliberately a capability vertical
slice, not a claim that those vehicles have equal source fidelity or physical
effector maturity.

## Current baseline

### Model Authoring — generic TAOS successor

In place:

- lossless source-oriented `.tbl` and `.prb` parsing, diagnostics, typed
  validation, source provenance, and successor grammar extensions;
- a documented lowering/runtime subset with executable fixtures; and
- a typed native-problem projection: only grammar-supported scalar metadata
  may enter a native directive, while nested actuator, sensor, evidence, and
  model metadata stays in its fingerprinted structured manifest; and
- manual, equation, parser, and corpus quality gates.

Boundary:

- TAORYX is **not** historically runtime-compatible with TAOS 96.0.  That
  claim remains unavailable without a historical executable or trusted output
  corpus.

Priority rule:

- preserve compatibility, parser, equation, and source-provenance gates as
  continuous release gates; do not let historical-runtime reconstruction block
  the composition vertical slice.
- Native lowering is a projection boundary, not a string-formatting shortcut.
  A generated `.prb` must parse under its declared grammar profile; a
  dictionary, list, or other structured metadata value must never be rendered
  as a scalar `key=value` attribute.  Omitted native fields remain available
  only from the resolved structured vehicle/evidence artifact, with an
  explicit native-binding nonclaim.

### Simulation Runtime — simulation, stepping, control, and artifacts

In place:

- batch and interactive execution, deterministic command/event histories,
  control and actuator contracts, trim/controller/allocator seams, and
  normalized run artifacts/plots;
- committed truth at accepted boundaries, pre/post transition truth pairs,
  segment-specific integration cadence, sensor-clock boundaries, load-evaluation
  state/control-time provenance, and no sensor interpolation from post-step
  states;
- registered declared sensor scenarios that reconstruct their provider,
  queued/delivered latency packets, interval baseline, and packet-only
  estimator state from a checkpoint; and
- four explicit fidelity tiers: point-mass 3DOF, named pseudo-6DOF,
  rigid-body direct wrench, and rigid-body physical-effector allocation.

Remaining layer work:

- extend checkpoint factories from the current registered declared-sensor
  scenario to additional named physical sensor and estimator families; and
- compare an identical declared semantic action stream through batch and
  interactive execution for each episode-capable composition, rather than
  inferring parity from their separate nominal artifacts;
- add declared reset/loadout perturbation recipes only where the selected
  family adapter can rebuild the affected plant, trim, resources, and
  provenance without arbitrary model mutation; and
- promote direct-wrench bridges only through explicit trim, authority, and
  nonlinear validation evidence; it remains a labeled screen or bridge until
  then.

The current consumer-facing maturity tranche is tracked in the
[Simulation Runtime maturity plan](simulation-runtime-maturity.md) and its
[onboarding guide](../SIMULATION_RUNTIME_ONBOARDING.md). The immediate gate is not
another unqualified model count: a new engineer must be able to discover a
source scenario or composition request, identify its exact fidelity and
operation type, run the narrowest setup/preflight diagnostics, step the same
resolved model at accepted truth boundaries, and inspect a named normalized
artifact. The staged-rocket, NESC pseudo-6DOF, HL-20 release, X-15-scaled
booster, synthetic California–Hawaii, and interactive stepping paths are the
regression set for this gate.

### Mission Composition — vehicle and mission composition

In place:

- a registry of current families, fidelity slots, source/evidence status,
  initialization contracts, segment schemas, mission templates, parameters,
  and blockers;
- versioned `taoryx vehicle catalog --detail summary|full`, plus focused
  `list`, `describe`, `inspect`, `schema`, `parameters`, `endpoints`,
  `authoring`, and interface discovery commands;
- `taoryx vehicle compose`, which creates an immutable semantic handoff and
  rejects incompatible vehicle/fidelity/init/segment combinations; and
- `taoryx vehicle lower`, which binds only the declared native adapter and
  returns a structured blocker instead of substituting a generic plant.
- `taoryx vehicle episode-info`, which opens only a declared interactive
  factory and exports its reset-time committed truth, and `taoryx vehicle
  result`, which validates the common `evaluation.json` envelope and can bind
  it to an exact compiled-composition fingerprint.

Remaining gap:

- the common airbreather translators are executable. The X-15 high-energy
  binding still needs its own source-bounded release/trim, phase, controller,
  and evaluation adapter. The HL-20 energy-glide binding now has an exact
  semantic intent translator, but still needs its native runtime, trim,
  controller, and evaluation adapter. Both remain non-runnable until those
  execution pieces exist; translator registration alone is never a run.

The HL-20 now has an earlier, deliberately narrower Mission Composition capability
step: its point-mass and pseudo-6DOF glide-energy composition exposes a
source-bound release-energy and signed bank-reversal estimate. It can reject a
handoff energy target above the unpowered release state. It now lowers the
composition into an exact release/trim/opposing-bank/energy-handoff plan, but
remains non-runnable without a native runtime, trim, control, or
truth-objective claim. This is the intended staging pattern for a new family:
make the composition and its feasibility assumptions inspectable first, then
promote only through a source-owned execution adapter.

Active progress:

- The X8 capability-derived 3DOF and named pseudo-6DOF racetracks now
  compile, preflight, materialize exact disposable `.prb`/catalog inputs,
  execute through the language-backed runtime, and produce independent
  objective, fixed-step-refinement, and batch/step-parity evidence.
- The B747 now reuses that same semantic translator and materialization path
  at the transport-scaled 3DOF/pseudo-6DOF geometry. Its pseudo-6DOF route
  response is a named route-lag bridge, not a surface or moment claim.
- `taoryx vehicle run` now provides the public, source-owned nominal execution
  seam for the four airbreathers. X8/B747 materialize disposable native inputs
  and execute through the language-backed runtime; A320/F-16 bind the same
  resolved semantic racetrack to their declared OpenAP or source-reduced
  adapter. Every path writes truth telemetry, independent objective/envelope
  reports, trim/provenance where applicable, and execution provenance. The
  candidate-packet tool remains a stricter downstream evidence consumer for
  fixed-step refinement, accepted-step replay parity, and promotion review.

## Priority order

### P0 — hold the common contracts steady

This is continuous work, not a feature tranche.

- Keep language/profile validation, source provenance, canonical units/frames,
  accepted-truth timing, value-space topology, control realization labels, and artifact schema as
  release gates.
- Maintain the four-tier fidelity vocabulary and fail-closed lowering.
- Fix regressions or contradictions in existing qualification evidence before
  broadening claims.

**Exit condition:** every subsequent milestone uses the same resolved-case,
truth, control, value-space, and artifact contracts. No layer-specific
alternate result format is introduced. Public fields must declare their
topology and operation rules as specified by the
[public value-space contract](../architecture/public-value-spaces.md); a
heading cannot silently be treated as a linear scalar merely because both are
stored in degrees.

### P1 — make one semantic airbreather mission executable end to end

Implement a `SemanticSegmentTranslator` contract at the family-adapter
boundary.  Given a compiled composition, it must:

1. resolve initialization into trim or declared state;
2. convert each semantic segment into controller references, event/gate
   geometry, duration/horizon, and numerical configuration;
3. validate required adapter operations and capability limits before run;
4. construct the native runtime without a generic model fallback;
5. run through the normal batch/interactive kernel; and
6. return the normal truth-evaluated artifact, including source/fidelity and
   control-realization claim boundaries.

First witness: **X8** at point-mass 3DOF and pseudo-6DOF for
`powered_fixed_wing_racetrack_v1`.  The existing direct-wrench and physical
surface work remain separate selectable evidence tiers; P1 must not replace
them with an attitude surrogate.

Current evidence: the X8 pseudo-6DOF witness has eight independently
evaluated route and route-lag attitude objectives. The same objective result
is preserved after halving the 0.05 s fixed step, and external accepted-step
replay agrees exactly at common samples. This is a narrow nominal-route
promotion witness, not a family qualification or physical-effector result.

Deliverables:

- `taoryx vehicle run <composition>` and `taoryx vehicle preflight
  <composition>`;
- one source-owned X8 runtime factory moved from qualification tooling into
  `src/taoryx`;
- capability-scaled racetrack geometry and horizon generation;
- controller-transition diagnostics plus independent truth objectives;
- a reproducible packet generated from the returned artifact, not a custom
  plotting path; and
- negative controls proving unavailable adapter operations, infeasible
  geometry, skipped objectives, and bad terminal states fail visibly.

**Exit condition:** one X8 composition request can be listed, compiled,
preflighted, executed through the public source-owned path, run in batch or
stepwise mode, replayed, and independently evaluated with no X8-specific CLI
or plotting path. The execution artifact and the stricter candidate evidence
must agree on the nominal truth outcome.

### P2 — prove the airbreather template is reusable

Reuse the P1 semantic translator and racetrack compiler for **B747, A320, and
F-16**, initially at each model's supported 3DOF and pseudo-6DOF tiers.

This phase is intentionally not a demand for identical tuning or equal
fidelity.  It must reuse:

- the same initialization/segment/objective vocabulary;
- the same feasibility and route sizing calculations;
- the same translation and artifact interfaces;
- the same controller/achievability telemetry; and
- the same failure taxonomy.

It may supply family-specific capability profiles, trim recipes, controller
schedules, control-intent mappings, source tables, and evidence boundaries.

Physical direct-wrench and physical-surface tiers are evaluated independently
after their adapter operations are available; they are never inferred from a
passing lower tier.

**Exit condition:** a user can change only `family_id`, initialization values,
and mission parameters to run the same semantic racetrack template across the
four airbreathers.  All differences are declared by capability/profile data,
not copied route code.

Current reuse witnesses: a B747 composition with the same ordered segment
vocabulary derives its 30 km straights and 8.081 km turns from the B747
capability profile, then materializes the corresponding source-table
pseudo-6DOF route-lag packet. A320 and F-16 now execute that same semantic
contract at their declared 3DOF and named pseudo-6DOF profiles through their
own OpenAP and source-reduced adapters. The A320 operating Mach is solved to
the semantic true-airspeed request; the F-16 remains at its source-trimmed
subsonic point. These are nominal objective passes only: neither lower-tier
result implies physical surface allocation, direct-wrench validity, schedule
coverage, or aircraft-family qualification.

### P3 — turn composition into an episode interface

Make the resolved executable case usable by a human controller, notebook, or
AI/RL environment without creating a second physics API.

```text
resolved composition
    -> executable episode
    -> reset(seed, initial/loadout overrides)
    -> observe()
    -> step(named action, duration or next required boundary)
    -> artifact / evaluation / checkpoint
```

Requirements:

- action and observation schemas come from the selected fidelity and adapter;
- semantic action, status, resource, and observation schemas are projected
  from a versioned VehicleInterfaceContract; native state/control names remain
  available as explicitly nonportable diagnostics rather than becoming the
  AI/RL API;
- an external policy can command only declared intent/effectors, never mutate
  arbitrary state or model tables;
- observation timestamps use committed truth or declared sensor models;
- episode reset, seeds, command history, event history, and checkpoints are
  reproducible; and
- batch, scripted stepwise, and RL-style stepping agree over the same action
  stream within declared tolerance.

**Exit condition:** X8 and Hummingbird each have one reproducible episode
adapter built from a composition request, not a bespoke simulator wrapper.

Current implementation: the initial exit witness is present. X8/B747 episode
requests materialize through the same language-backed runtime and project its
existing `InteractiveSession`; the Hummingbird pseudo-6DOF witness projects
its declared bounded aggregate-thrust model. Both expose action/observation
schemas plus `reset`, `observe`, `step`, checkpoint restore, and `close`.
`InteractiveSession.step` now continues across adaptive accepted inner steps
until the requested external truth boundary, rather than returning after the
first shortened RKF45 step. X8 and Hummingbird now also bind one declared
cadence/latency sensor profile through the composition: the scheduler visits
each capture and delayed-release truth boundary, policy observations hold the
last released sample with its source timestamp, and checkpoint restoration
reconstructs the sensor state. Their aligned batch paths emit the same held
observation trace only when their committed truth rows contain every required
capture and release boundary; they fail closed instead of interpolating a
post-run trace. Reset accepts a declared seed and rebuilds the exact immutable
composition plus selected declared-sensor profile; it never mutates source
parameters in place. Broader family-specific reset/loadout perturbations,
named physical sensor-noise/estimator models, RL reward, and additional
multi-family episode bindings remain later P3 work.

### P3a — stabilize the vehicle parameter, control, and status interface

The existing episode witnesses prove that a resolved composition can open an
accepted-truth control session. Their schemas are still adapter-native:
throttle in one witness, aggregate thrust ratio in another, and
vehicle-specific raw state names in the language-backed path. Simulation Runtime needs
a stable interface above those layouts so an AI/RL policy, user-control
surface, telemetry consumer, or showcase has reliable hooks without claiming
identical physics.

Implement the resolved
[vehicle interface contract](../architecture/vehicle-interface-contract.md):

1. Declare bounded parameters by model, configuration, reset, segment, and
   step-action scope.
2. Project canonical semantic action profiles through the control-authority
   ladder: mission, kinematic, body-motion, wrench, and physical effector.
3. Publish canonical status/resource hooks for execution state, kinematics,
   propulsion, requested/applied/achieved authority, saturation, envelopes,
   and numerical/truth timing.
4. Keep generic resources generic: fuel, propellant, battery, stored wheel
   momentum, and passive-family absence remain distinct ledger entries rather
   than an invented universal fuel field.
5. Separate committed plant truth, declared sensor observations, and
   policy-visible observations. A policy never obtains a future-interpolated
   sensor sample or arbitrary internal state by default.
6. Require availability, units, frames, bounds, provenance, evidence tier,
   and native binding for every advertised channel. Unavailable does not mean
   zero.

First migration witnesses are X8/B747 for air-breathing propulsion and
fixed-mass semantics, Hummingbird for aggregate-thrust and battery semantics, and NESC
for staged propellant/event semantics. Physical-surface X8 or F-16 and
rotor-allocated Hummingbird runs become the promotion witnesses for actual
effector status.

**Exit condition:** before opening an episode, a caller can discover the
exact parameter, action, status, resource, observation, and raw-diagnostic
schemas for one selected family/fidelity/authority profile. The same resolved
contract drives composition, episode control, artifact telemetry, and
showcase renderers. A generic policy harness can operate an X8 and
Hummingbird through declared semantic profiles without parsing native state
names or receiving invented controls/status.

**Current implementation:** the first interface slice is executable. The
vehicle interface CLI command resolves a fingerprinted contract and validates
its available channel bindings before a run. X8/B747 and Hummingbird episode
wrappers now retain their legacy native schemas for compatibility while
accepting profile-bound ActionFrame inputs and producing canonical StatusFrame
and truth-debug ObservationFrame outputs. The raw sidecar remains available
for source diagnostics, and every public vehicle-run packet writes the exact
vehicle interface artifact alongside its execution evidence. Frame traces
retain requested semantic actions, applied native actions, and applied
semantic actions after limiting. The current fixed-wing profile is explicitly a native
source-control bridge and the Hummingbird profile is explicitly aggregate
thrust-vector pseudo-6DOF; neither is presented as physical effector evidence.
The generic `run_composition_policy` harness now executes either witness only
through its declared authority/observation profile and emits a replayable
semantic request, native-application, applied-semantic, truth-observation, and
status trace. A persisted trace is bound to the exact composition identity and
interface fingerprint; `taoryx vehicle replay-policy` reopens a fresh episode
and compares every public committed-boundary frame to prove that the same
semantic action stream reproduces exactly. This is scripted/RL-style replay
evidence, not batch parity or physical-control validation. Composition-backed
showcase manifests can now retain the exact fingerprinted interface evidence and selected observation profile, preventing
a board from relabeling a sensorized bridge run as truth-debug or physical
allocation. The first declared-sensor profile is now
composition-bound and checkpointable for the X8 and Hummingbird witnesses;
named physical-sensor/estimator models, surface-allocated, staged-resource,
and broader showcase renderer adoption remain required before
the full P3a exit is claimed.

The shared declared-sensor scheduler now also supports an opt-in, explicitly
fingerprinted **scalar measurement transform** per selected channel: constant
bias, deterministic seed-controlled Gaussian sample noise, and an optional
quantizer. It operates at capture time only, is checkpointed with the episode,
and is replayed in batch only over existing committed rows. It is deliberately
not a generic IMU/GPS/estimator claim: vectors, booleans, enums, physical
sensor axes, correlation, drift, and estimator state remain named-model work.
This closes the ideal-only sensor bottleneck without creating an alternate
post-processed telemetry path.

`taoryx vehicle interface-report` is the catalog-wide static regression gate:
it resolves all advertised family/fidelity contracts and fails if an available
authority or observation profile lacks a runnable episode binding. It makes
interface drift visible before a vehicle is manually tuned or a new showcase
is rendered.

Execution factories are now a separate versioned composition authority:
`verification/vehicle_execution_bindings.yaml`. It maps one exact
family/mission/fidelity/operation tuple to a source-owned batch or episode
factory, or records its planned blockers. The public `taoryx vehicle endpoints
<family>` command exposes this distinction. `vehicle run` and episode creation
resolve this catalog rather than branching on a vehicle name; an unregistered
tuple fails before integration and cannot borrow a neighboring runtime.

`verification/vehicle_execution_witnesses.yaml` closes the discovery gap
between that authority and concrete user inputs: every runnable endpoint has a
checked-in compiled-composition witness. Its validation gate proves the
request compiles, preflights, resolves the exact factory, and opens when the
endpoint is interactive; it deliberately does not relabel this onboarding
check as a mission execution or qualification result. An opt-in batch-smoke
mode then runs each batch witness through the same public `vehicle run` seam
and requires its execution, exact interface, and canonical committed-status
trace artifacts. It also rejects a trace whose interface fingerprint, declared
channel set, committed-row ordering, or row completeness disagrees with the
resolved composition. The source-table
fixed-wing cases use an explicitly bounded translation smoke rather than
turning a long transport racetrack into an uncontrolled CI-duration test.

All current batch factories write `status_trace.json`: the selected resolved
interface's available and batch-only canonical status/resource/diagnostic
channels projected at committed truth boundaries. The trace fails if any
advertised batch channel is unbound, while the sensor trace remains a separate
selected-observation artifact. This proves artifact-interface consistency, not
batch/step parity, robustness, physical-effector behavior, or qualification.

The A320 and F-16 reduced bindings project the same local-navigation and
total-mass hooks as a portable status trace, with pseudo-6DOF attitude and
rate values explicitly labeled as response-law surrogates. Both also expose a
source-owned kinematic-guidance episode and registered action-trace parity
witness. Neither gains a physical control-surface claim or fuel-system ledger
from those reduced-fidelity hooks.

### P4 — onboard the next physical families through the same path

Only after P2/P3, promote the remaining current references through their
family-specific composition translators:

1. Hummingbird — grounded initialization, hover/dwell, yaw, disturbance,
   landing/contact, and rotor allocation boundary.
2. X-15 and HL-20 — release/trim, powered/coast or energy-glide segments,
   high-energy corridor, and atmospheric handoff.
3. NESC rocket — pad/release, staged resources, separation, coast, and
   terminal/deployment transitions.
4. Tumbling bodies — release/deployment and passive terminal contracts;
   direct-effector tiers remain not applicable.
5. Helicopter and tiltrotor — only after rotorcraft and hybrid-mode family
   strategies define their genuinely different capability, transition, and
   actuator contracts.

The first P4 implementation slice is complete: Hummingbird’s named pseudo-6DOF
hover/yaw/translation/contact composition has both an interactive aggregate-
thrust episode and a source-owned batch translator/evaluator. The batch path
emits the same compiled composition, translation preflight, lowered mission
plan, truth telemetry, controller-transition diagnostics, envelope report, and
independent objective report as other nominal composition runs. Its bound is
explicit: this is aggregate thrust-vector response evidence, not an
individual-rotor allocation or full-family qualification claim.

The second P4 slice is also executable: the NESC two-stage rocket composes
only when its launch origin, heading, mass, staging delay, and orbit terminal
match the retained source witness.  At 3DOF it replays that source translation;
at pseudo-6DOF it adds only the declared phase-scheduled response law.  Both
paths emit the common preflight, lowered-plan, telemetry, ordered stage-event,
terminal-state, envelope, and provenance artifacts.  They do **not** claim a
participating rocket plant, pitch program, active guidance, gimbal allocation,
or physical separation dynamics.

The third P4 slice is an explicitly bounded **X-15-scaled staged reachability
witness**, available at point-mass 3DOF and named pseudo-6DOF.  It pins the
retained X-15 rocket-to-Hawaii source deck's stacked mass, ECIC speed
magnitude, booster cutoff, and release timing, then runs the local reduced
boost/coast/release/glide model with one deterministic engineering launch
direction.  Independent truth evaluation requires the boost cutoff, passive
booster deployment, high-energy corridor, open-loop atmospheric handoff
witness, and impact event.  It records the retained source deck's 9,000 kg
declared-versus-2,000 kg consumed booster-propellant discrepancy rather than
silently repairing it.

This new executable witness does **not** turn the X-15 direct-wrench binding
into a high-energy mission.  The local direct-wrench adapter remains a
separate source-load bridge at a local X-15 state; it is available both as a
batch screen and a bounded total-wrench episode.  The synthetic
`examples/showcases/california_to_hawaii` rigid-body example is another
separate witness: its own realization identifies a synthetic CA-HI vehicle and
must never be displayed or selected as an X-15 family execution.  A future
native X-15 high-energy mission must bind its true release, propulsion,
guidance, aerodynamic, and terminal state consistently before it can replace
either bounded witness.

The local direct-wrench controller screens are now also a shared source-owned
execution primitive rather than duplicated vehicle-specific scripts.  The
generic runner derives its local LQR from the supplied nonlinear source-load
adapter, preserves a declared load-cancellation bridge bias, projects every
request through the supplied six-axis authority limits, and logs the
requested-versus-achieved residual.  The feasible retained X-15 release/glide
screen is now a distinct public composition endpoint:
`x15_local_direct_wrench_screen_v1` compiles, preflights, runs, opens as a
bounded episode, and emits the standard portable committed-status trace.  The
episode accepts only an explicit total six-axis body wrench, including the
declared local bridge bias, and logs the projected achieved wrench; it does
not expose a disguised actuator command.  Its status is deliberately
`development_local_screen_pass`, not a flight-mission pass.  At the retained
HL-20 Mach-2 point, the currently declared direct-force limits cannot cancel
the source load; that is a visible authority/trim blocker, **not** permission
to enlarge the limits or label the HL-20 bridge trimmed.  Neither local screen
is a release-to-handoff mission executor or a physical-effector claim.

The execution-binding catalog is the authoritative current P4 progress matrix:

| Family / mission | Current public batch binding | Evidence boundary |
| --- | --- | --- |
| Hummingbird hover/yaw/translation/contact | runnable, pseudo-6DOF aggregate-thrust response | No individual rotor allocation or full multirotor family claim. |
| NESC staged source replay | runnable, 3DOF and pseudo-6DOF | Pinned translation replay; pseudo attitude is scheduled response only. |
| X-15-scaled staged reachability | runnable, 3DOF and pseudo-6DOF | Local-frame reduced staging witness; neither native X-15 control nor CA-HI trajectory evidence. |
| X-15 local direct-wrench recovery screen | runnable, direct wrench | Pinned source-local LQR/authority screen; no flight mission, trim, scheduling, or physical effector claim. |
| X-15 high-energy mission | planned, direct-wrench | Local source bridge exists; release-to-handoff mission lowering and scheduling remain blockers. |
| HL-20 glide energy mission | planned, direct-wrench and surface-allocated | Local source loads/allocation exist; complete release/gravity/energy-glide mission loop remains blockers. |
| Tumbling-body direct passive release | runnable, 3DOF and pseudo-6DOF | Fixed engineering cylinder only; 3DOF is averaged-area, pseudo reuses native passive rigid-body tumble, and neither has control authority. |

Planned entries are intentional interface commitments, not runnable claims:
`taoryx vehicle endpoints <family>` exposes their exact blockers before any
integration attempt.

**Exit condition:** onboarding a new member of an existing family means
supplying source/package data, capability/trim/controller bindings, and
evidence—not writing a separate scenario runner or plot system.

### P5 — broader language and backend goals

Defer until P1–P4 have stabilized the public seams:

- further TAOS feature coverage and historical differential work;
- new model topologies and procedural geometry;
- external-backend adapters and cross-backend conformance;
- broad parameter search/optimization and corpus-scale RL; and
- release-scale robustness campaigns beyond the established family packets.

These remain important, but will move faster after a vehicle/mission request
has one reliable compile-to-run path.

## Core Authoring → Runtime → Composition handoff milestone

The initial cross-layer handoff is complete when all of the following are
true.  This is deliberately narrower than completing every planned physical
family or claiming a release-qualified controller envelope.

| Layer | Delivered core boundary | Evidence retained as a release gate |
| --- | --- | --- |
| Language successor | A typed, provenance-preserving `.tbl`/`.prb` source path and fail-closed native lowering projection. | Manual build, equation provenance audit, grammar/profile validation, and corpus tests. |
| Simulation platform | One accepted-truth runtime serves batch and interactive stepping; control activation and load evaluation are timestamped; declared sensor observations are captured only at committed boundaries and registered scenarios restore from checkpoints. | Timing, transition, sensor, checkpoint, deterministic replay, unit/type, and runtime tests. |
| Vehicle and mission composition | A caller can discover a registered family/fidelity interface, compile/preflight a bounded composition, resolve one exact batch or episode factory, execute it through the source-owned path, and inspect standard truth/status/provenance artifacts. | Catalog validation plus checked-in witnesses for every advertised runnable endpoint. |

At the current baseline, the catalog contains **nine families**, **36 resolved
vehicle-interface contracts**, and **26 checked-in runnable composition
endpoints**.  The endpoint gate exercises composition, preflight, factory
resolution, episode construction where advertised, and optional batch artifact
smoke without introducing a fallback plant or controller.

This milestone does **not** claim the following unfinished work:

- a generic batch-versus-interactive comparison for one identical external
  semantic action stream;
- arbitrary reset/loadout mutation; reset is seed-only and reconstructs the
  immutable resolved composition;
- automatic reconstruction of arbitrary sensor callbacks or all future
  physical sensor/estimator families;
- physical-effectors, trim, authority, nonlinear-envelope, robustness, or
  qualification evidence beyond each selected endpoint's stated fidelity
  boundary; or
- native X-15 high-energy or HL-20 energy-glide missions.  Their planned
  catalog entries remain explicit blockers until source-bounded adapters,
  schedules, and independent evaluators are supplied.

Accordingly, the next work must either close one of those named gaps or add a
new family through the same contracts.  It must not create another bespoke
route runner, observation path, or implicit fallback.

## Work explicitly deferred during P1/P2

To accelerate the shared path, do not start a new bespoke showcase, route
manager, controller stack, or vehicle-specific plotting layer for a new family
unless it either:

1. fixes a P0 contract regression; or
2. is a conformance witness needed by P1/P2.

Existing source correlation, direct-wrench/surface allocation, showcase, and
family research work continues as evidence maintenance.  It must consume the
shared contracts rather than create a competing execution path.

## Planning and status vocabulary

Use these terms consistently:

| Term | Meaning |
| --- | --- |
| `declared` | Registry/documentation says a configuration exists. |
| `composable` | A request compiles and validates semantically. |
| `adapter_bound` | The exact selected adapter constructed; no fallback occurred. |
| `factory_bound` | An exact semantic translator and source-owned batch factory are declared; generic adapter conformance has not been claimed. |
| `executable` | Semantic segments translated into and completed by the native runtime. |
| `evaluated` | Independent truth objectives and terminal conditions were computed. |
| `qualified` | Declared evidence, control, numerical, and robustness gates passed. |

The words are strictly ordered.  In particular, a declared profile or bound
adapter is not an executable, evaluated, or qualified mission.

## Success measures

This reprioritization is working when the following improve release over
release:

- number of vehicle/fidelity/mission combinations runnable through
  `vehicle compose` → `vehicle run` without bespoke glue;
- time and new code required to add a member of an existing physical family;
- percentage of a run's visible values recomputable from one standard artifact;
- number of failures diagnosed before integration rather than after manual
  tuning; and
- number of controller/route/plot implementations shared by multiple
  vehicles.

Do not measure success by the raw number of names in the vehicle registry or
the number of rendered composites.

## Plan ownership after reprioritization

| Existing plan | Role after this decision |
| --- | --- |
| `taoryx-successor-roadmap.md` | Umbrella vision and language/runtime contracts. |
| `composable-scenario-runtime.md` | Runtime and episode contract implementation. |
| `mission-composition-automation.md` | P1/P2 mission compiler, preflight, and translator work. |
| `horizontal-vehicle-integration.md` | Four-tier adapter/fidelity conformance. |
| `vehicle-integration-automation-plan.md` | Intake, readiness, trim, effectivity, and onboarding automation. |
| `taoryx-alpha-3.md` | Family breadth after the common compose-to-run airbreather slice. |
| `taoryx-family-showcase-composite-master-plan.md` | Evidence-pack rendering and qualification after executable scenarios exist. |
