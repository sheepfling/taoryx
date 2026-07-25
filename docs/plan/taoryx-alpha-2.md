# TAORYX Alpha 2: neutral trajectory contracts, vehicle families, and control tooling

**Target release:** `v0.2.0-alpha`  
**Depends on:** Alpha 1 language, validation, deterministic runtime,
composition, and run-artifact foundations  
**Status:** core contract complete — `A2-RELEASE-PASS`; bounded-variant
closeout tranche planned

## Purpose

Alpha 2 turns TAORYX from a simulator that can run individual problem files
into a platform for building and operating configurable vehicle families.

The user selects:

```text
family + fidelity + variant/loadout + mission + phase plan
       + controller preset + explicit overrides
       = resolved runnable case
```

TAORYX validates that selection, derives values such as mass and inertia,
composes the model graph, freezes the parameter/observation/control schemas,
and produces the same deterministic runtime whether it is run to completion or
stepped by an AI, script, or human-player adapter.

> A TAORYX run is a compiled selection of family, fidelity, loadout, mission,
> segments, and controller—not a hand-built monolithic model.

This is a successor-side release plan. It does not widen the historical TAOS
96.0 compatibility claim.

The vehicle qualification standard for this release is
[Vehicle Readiness Guide](vehicle-readiness-guide.md). Alpha 2 must use its
M0–M6 maturity ladder, dimension scorecard, graded composition statuses, typed
start contracts, classified finality, and readiness badges rather than treating
a single successful trajectory as vehicle qualification.

## Architectural boundary and dual role

Alpha 2 makes Taoryx one trajectory provider behind a backend-neutral contract.
The common client, CLI, AI environment, and host must not require Taoryx types,
TAOS names, native state indices, or Taoryx import behavior.

```text
client / CLI / UI / AI environment
              |
      trajectory-contracts
              |
       trajectory-host
        /             \
 Taoryx adapter    reference provider
       |                  |
    Taoryx       analytical/replay engine
```

The target package boundary is:

| Package | Responsibility |
| --- | --- |
| `trajectory-contracts` | Portable cases, schemas, capabilities, controls, observations, events, diagnostics, sessions, and results. |
| `trajectory-adapter-sdk` | Provider adapter interfaces, translation reports, and shared adapter utilities. |
| `trajectory-host` | Discovery, compatibility resolution, provider selection, job/session lifecycle, and common endpoints. |
| `trajectory-adapter-taoryx` | Translation between neutral contracts and Taoryx models, syntax, runtime, and artifacts. |
| `trajectory-conformance` | Capability-aware tests for every registered provider and binding. |
| `trajectory-env-*` | AI, Gymnasium, game-player, and task-specific wrappers above the common session. |

The neutral packages must not import Taoryx. The Taoryx adapter may depend on
the neutral SDK and Taoryx. Alpha 2 may implement these boundaries in the
existing repository before they become separately published distributions,
but the dependency direction is mandatory.

Taoryx is not merely an implementation hidden behind the provider adapter. It
has two first-class facades over one canonical simulation runtime:

```text
                         Portable trajectory ecosystem
                    clients / optimizers / other tools
                                      |
                                      v
                       Common trajectory-provider API
                                      |
                         Taoryx Provider facade
                                      |
              +-----------------------+-----------------------+
              |                                               |
              v                                               v
        batch / interactive clients                    Taoryx Lab API
                                                              |
                                      +-----------------------+
                                      v
                         Taoryx canonical simulation runtime
                                      |
                +-------------------+-------------------+
                |                   |                   |
                v                   v                   v
          vehicle families      controllers       environment models
                +-------------------+-------------------+
                                    v
                         3DOF / pseudo-6DOF / 6DOF
```

The provider facade exposes the portable portion of Taoryx: validation,
resolution, compilation, batch execution, interactive stepping when supported,
capabilities, normalized results, and native artifacts. Taoryx Lab is a richer
superset for controller and AI research: seeded reset, tasks, action and
observation profiles, rewards, termination, randomization, rollouts,
benchmarks, and experiment provenance.

Neither facade may contain a second simulation implementation. Both must
compile the same family, mission, loadout, controller, and fidelity
configuration into the same canonical simulation session. A batch run is
defined as repeated calls to the exact transition function used by interactive
control and learning environments:

```text
compile case
    |
    v
canonical session.reset(seed, initial_state)
    |
    +--> provider.run_to_completion()
    |
    +--> provider.step(control_frame)
    |
    +--> lab.reset(seed); lab.step(action)
                 |
                 v
       one runtime transition function
```

Physics models must not import a particular RL framework. RL wrappers must not
own stepping semantics. Vehicle packages define physical capabilities; task
packages define agent objectives.

### Trajectory cases and testbed environments

A trajectory case describes the physical simulation:

```text
family + fidelity + loadout + mission + phase plan + controller + overrides
                              = ResolvedCase
```

A testbed environment adds the agent-facing problem without changing that
physical case:

```text
ResolvedCase
  + task + action profile + observation profile + reward
  + success/failure + termination/truncation + randomization
  + agent period + seed
  = ResolvedEnvironment
```

The same Simple Aero configuration can therefore be a batch trajectory case,
autopilot verification case, scripted closed-loop run, human-player scenario,
terminal-accuracy task, energy-management task, or cross-fidelity transfer
benchmark. Rewards, curricula, and episode rules do not belong in a vehicle
physics package.

### Fidelity and control are independent axes

Physics fidelity and agent control abstraction are deliberately orthogonal:

```text
                         control abstraction
                 guidance   attitude/rate   effectors
              +------------------------------------------+
  3DOF        | native      unsupported      unsupported |
  pseudo-6DOF | native      response model   surrogate   |
  rigid 6DOF  | autopilot   native           physical    |
              +------------------------------------------+
                         physics fidelity
```

A rigid-body model may still expose bank, speed, heading, or normal-
acceleration commands through an autopilot. The resulting progression is:

```text
3DOF guidance policy
        -> pseudo-6DOF inner-loop response
        -> rigid-body 6DOF with the same high-level actions
        -> optional attitude/rate policy
        -> optional direct-effector policy
```

This makes control difficulty selectable without misrepresenting physics
fidelity.

### Taoryx capability tiers

The provider contract remains inclusive rather than falsely identical:

| Tier | Required behavior |
| --- | --- |
| Batch provider | Validate, compile, run to completion, and return normalized results. |
| Interactive provider | Also reset sessions, inject controls, step deterministically, return observations/events, and optionally checkpoint. |
| Learning/control testbed | Also provide seeded reset, tasks/rewards, termination/truncation, randomization, transforms, truth/sensor/privileged observations, evaluation seeds, vectorized rollouts, replay/branching, and experiment provenance. |

Taoryx targets all three tiers. Other providers may remain Tier 1 or Tier 2;
their unsupported learning capabilities must be reported rather than
emulated implicitly.

### Case lifecycle and representations

The plan distinguishes user intent from provider implementation:

```text
CaseIntent -> validate/resolve -> ResolvedCase -> compile -> CompiledCase
                                      |                    |
                             backend-neutral       provider-owned opaque
```

- `CaseIntent` is portable, human-authored input. It may contain presets,
  aliases, display units, backend preferences, and namespaced extensions.
- `ResolvedCase` is immutable and backend-neutral. It contains canonical units,
  explicit fidelity requirements, validated schemas, compatibility decisions,
  and provenance for every value. It contains no Taoryx class, TAOS variable,
  or native state-vector index.
- `CompiledCase` is provider-owned and opaque to the host. A Taoryx compiled
  case may reference its model graph, runtime settings, native mappings, and
  problem representation.

Vehicle-family identity, variants, loadouts, missions, fidelity requirements,
parameter metadata, controls, observations, presets, and result schemas belong
in the neutral contract. TAOS syntax/imports, Taoryx graph construction, native
state layout, solver implementation, and native plotting belong in the Taoryx
binding.

### Adapters, capabilities, and translation

Alpha 2 separates two adapter responsibilities:

1. A backend execution adapter starts and monitors runs, creates and steps
   sessions, checkpoints/restores, collects native files, normalizes results,
   and reports provider errors and artifacts.
2. A vehicle-family binding maps the neutral family into one provider: native
   parameters, realizable fidelity profiles, controller presets, controls,
   observations, events, and verification evidence.

The host exposes common operations for provider discovery, capability and
family-schema inspection, validation, resolution, compilation, batch runs,
run inspection, normalized results, and native artifacts. Interactive session
operations are negotiated capabilities, not assumed endpoints.

Every advertised capability is `native`, `emulated`, `approximated`, or
`unsupported`. Compilation produces a complete translation report and fails by
default when a required concept cannot be represented. A case may explicitly
allow selected approximations, but ignored parameters, controls, events, or
fidelity requirements are never silently dropped.

Fidelity is represented both by friendly profiles such as
`point_mass_3dof` and by an explicit vector covering translation, attitude,
angular rates, mass properties, aerodynamics, propulsion, actuators,
atmosphere/gravity, control-command level, and integration behavior. Results
record the requested vector, realized vector, and every approximation.

Backend-specific options use preserved namespaces such as
`extensions.org.taoryx`; provider matching marks mandatory extensions as a
Taoryx requirement without making the portable contract Taoryx-specific.

## Relationship to Alpha 1

Alpha 1 establishes the parser, validator, canonical runtime transition,
problem execution, evidence artifacts, and explicit 3-DOF/pseudo-6-DOF/
rigid-body 6-DOF boundaries. Alpha 2 builds the configuration and control layer
on top of those contracts.

Alpha 2 must not start by adding more bespoke `.prb` files. Existing B747,
Skywalker X8, Hummingbird, and X-15 cases should migrate toward family package
records, generated cases, and reusable segments. A legacy `.prb` remains a
valid source frontend; it is not the canonical configuration model for new
family combinations.

## Bounded vehicle variants: Alpha 2.1 closeout

The Alpha 2 composition contracts already resolve a family, fidelity, loadout,
mission, segments, controller, and overrides into an immutable `ResolvedCase`.
The remaining platform gap is an explicit, safe vehicle-variant compiler. This
is a focused closeout tranche, not a second physics runtime or a new problem-
file dialect.

```text
FamilyPackage + VariantSpace + semantic modifiers + candidate values
    -> VariantResolver
    -> ResolvedVariant + ResolutionReport
    -> ResolvedCase
    -> CompiledCase -> run() or step()
```

`ResolvedVariant` is an immutable vehicle binding inside the `ResolvedCase`.
It contains primitive and derived canonical values, selected components,
resource policies, validity/qualification status, capability envelope,
modifier history, source hashes, and a stable fingerprint.

The resolver must keep these decisions separate:

| Decision | Meaning |
| --- | --- |
| Internal validity | The model is physically and structurally consistent. |
| Qualification | The candidate remains within documented supported ranges. |
| Mission feasibility | The selected mission appears achievable for this candidate. |
| Execution outcome | What actually happened during the run. |

`best_effort` applies only to mission execution. It never permits negative
resources, invalid inertia, silent table extrapolation, or force after
propellant depletion.

Alpha 2.1 extends parameter metadata with independent, derived, and
developer-only roles; hard-valid, qualified, and extrapolated ranges; units,
transforms, coupling groups, provenance, evidence grade, and retrim/requalify
invalidation flags. Reject/project policies must record the original
candidate, projected candidate, constraints, and projection distance.

The initial scope is operational variation for the four established proof
families: payload, propellant/loadout, bounded thrust or power derating, CG
movement, actuator response, and controller/segment parameters. Arbitrary
table-cell mutation, unrestricted inertia entries, aerodynamic topology
changes, and geometry generation remain outside Alpha 2.

Variants are authored in catalog/configuration data and lowered through the
existing problem-file generator. A `.prb` file may remain an input or
generated artifact, but it is never the source of variant truth.

## Execution shape and milestone exits

Alpha 2 is intentionally not one all-or-nothing implementation effort. Work
ships through independently useful release points. A later tranche may not
silently compensate for a failed earlier tranche: each exit requires its own
machine-readable evidence, focused tests, and a clean claim boundary.

| Tranche | Release point | Primary result | Explicitly deferred |
| --- | --- | --- | --- |
| A2-T1 | `case-contracts` | **Complete:** neutral schemas, catalog records, immutable `ResolvedCase`, provenance, and resolver tooling. | Providers, Lab, controllers, and new 6-DOF physics. |
| A2-T2 | `provider-session` | **Complete:** one reference provider and the Taoryx adapter share validate/compile/reset/step/run/result semantics. | Vehicle-family breadth, learning tasks, and direct effectors. |
| A2-T3 | `control-authority` | **Complete:** versioned control schemas and deterministic autopilot/commanded/overlay/direct/mixed arbitration. | Policy transfer and complex missions. |
| A2-T4 | `simple_aero_3dof` | **Complete:** configurable booster/loadout/segment Simple Aero-style family with generated 3-DOF cases and evidence. | Pseudo-6DOF and rigid-body claims. |
| A2-T5 | `fidelity-ladder` | **Complete:** one resolved Simple Aero proof family runs a common mission at 3DOF, pseudo-6DOF, and rigid-body 6DOF with parity, divergence, closure, convergence, and native artifacts. | Dual-launch handoff and broad vehicle migration. |
| A2-T6 | `dual-launch-glider` | **Complete:** one resolved glider family generates air-release and attached-booster cases with a native continuous separation handoff, shared post-release guidance, authority metadata, trajectories, plot, and hashed evidence. | Global vehicle validity and historical TAOS claims. |
| A2-T7 | `alpha-2-release` | **Complete:** catalog-derived schema reference, explicit claim matrix, self-contained hashed evidence packet, isolated clean-source replay, and release audit gate. | Features outside the frozen Alpha 2 contract. |
| A2-T7.1 | `bounded-variants` | **Planned Alpha 2.1 closeout:** bounded semantic modifiers, coupled derivations, qualification tiers, reject/project reports, resource checks, and immutable variant fingerprints. | Raw table morphing, topology changes, geometry generation, and large-scale search. |
| A2-T8 | `source-reference-anchors` | **Moved to Alpha 3:** F-16 S-119 and HL-20 Mod K collections, native DAVE-ML replay, source check-case/trim evidence, and reproducible direct-control plant artifacts. | All source-anchor runtime qualification and overlays. |

Alpha 2 core is now closed at the core-contract boundary. The remaining
closeout is limited to the reusable platform seams and the four established
proof families. Source-grounded reference anchors, passive-body qualification,
Lab expansion, flagship mission qualification beyond the established family
slice, and deeper vehicle overlays are Alpha 3 work. Their dependencies are
maintained in the [post-release backlog](taoryx-alpha-2-backlog.md) and
`verification/alpha2_post_release_backlog.yaml`.

### Alpha 3 source-grounded reference-anchor objective

The Alpha 3 breadth plan includes a bounded F-16 S-119 and HL-20 Mod K
reference-anchor tranche. This is the bridge from verified external packages
to executable Taoryx library inputs; it is not a claim that either aircraft is
already an M5 pickup-ready mission family.

The Alpha 2 target is:

```text
immutable source package
        ↓
hash-pinned family manifest
        ↓
loss-aware .txcollection
        ↓
native Taoryx DAVE-ML replay
        ↓
source check cases + trim/hold evidence
        ↓
direct-control reference-plant artifact
```

F-16 and HL-20 are Alpha 3 reference anchors when a new contributor can:

- rebuild each collection from the pinned corpus without editing source data;
- load each family through the normal catalog and case-resolution path;
- replay the source DAVE-ML graph in a fresh Taoryx process;
- pass all embedded source check cases and the declared trim/hold regression;
- receive explicit unit, frame, envelope, source, and nonclaim diagnostics;
- reproduce the result and its hashes from one documented command.

The following remain separate post-anchor work: actuator dynamics, control
allocation, SAS/autopilot, flagship missions, sensor profiles, RL tasks, and
derived 3DOF/pseudo-6DOF reductions. Those layers may use the reference plants,
but must not be allowed to change or strengthen the source-plant claim.

### The first tranche: A2-T1 `case-contracts`

A2-T1 is the recommended first implementation target. It is deliberately
small enough to finish and valuable even if later provider or controller work
is delayed.

It delivers:

1. Neutral typed declarations for parameters, controls, observations,
   capabilities, events, and provenance.
2. `CaseIntent` parsing and validation with canonical units and frames.
3. A small versioned catalog containing one Simple Aero-style point-mass family,
   two loadouts, and two reusable segment plans.
4. Immutable `ResolvedCase` creation with precedence, compatibility checks,
   derived values, and stable identity hashing.
5. `case validate`, `case resolve`, `case explain`, `case diff`, and schema
   export commands.
6. JSON artifacts for the input, resolved case, provenance ledger, and fixed
   control/observation schemas.

T1 does not run a trajectory. That is a feature, not a gap: it proves that
configuration semantics are stable before runtime, provider, and controller
complexity is added.

T1 exit criteria:

- Two checked-in cases resolve in a clean process without source edits or a
  bespoke runner.
- Re-resolving identical input produces identical canonical JSON and case
  identity hash.
- `case explain` accounts for every resolved value with source layer, unit,
  default/override status, or derivation dependencies.
- Unknown fields, unused overrides, incompatible loadouts, unit mismatches,
  and precedence conflicts fail with source-located diagnostics.
- Generated schemas have stable IDs and remain fixed after resolution.
- The neutral contract modules pass an import-boundary test and do not import
  Taoryx runtime or native state-vector types.
- The T1 evidence directory contains resolved cases, schema exports,
  provenance, diagnostics, hashes, and a reproducibility manifest.

T1 stop conditions:

- a value is silently ignored or guessed;
- the resolver requires a vehicle-specific Python runner;
- a generated problem file is treated as the source of configuration truth;
- a neutral schema imports Taoryx-specific runtime types; or
- a failed resolution can still compile or run.

The T1 completion signal is `A2-T1-PASS`, recorded in the release inventory.
Only after that signal is present should provider/session work begin.

### Tranche exit contract

Every subsequent tranche must publish the following before it is considered
complete:

| Required item | Minimum evidence |
| --- | --- |
| Scope | A list of included and excluded capabilities. |
| Contract | Versioned schemas or interfaces with stable IDs. |
| Positive proof | At least one generated case and executable test path. |
| Negative proof | At least one failure, unsupported, or stop-ship fixture. |
| Artifact proof | Hashes for inputs, resolved data, outputs, and software version. |
| Replay proof | Deterministic rerun or an explicit reason replay is not applicable. |
| Exit record | Machine-readable status, owner, remaining blockers, and next tranche. |

Passing a tranche does not promote deferred capabilities. For example, a
passing Simple Aero 3DOF tranche cannot imply pseudo-6DOF or rigid-body validity;
those require their own ladder exit evidence.

## Definition of done

Alpha 2 is complete when a user can:

1. Select a versioned vehicle family, fidelity profile, compatible variant and
   loadout, mission and launch form, reusable phase plan, and controller preset.
2. Resolve that selection into an immutable, unit-checked model graph with
   complete parameter provenance, bounded-variant status, and fixed
   observation/control schemas.
3. Run the resolved case deterministically to completion or one step at a time.
4. Operate it through autopilot, high-level commands, bounded external overlays,
   or direct effectors under explicit per-channel authority.
5. Select either the Taoryx provider or a small non-Taoryx reference provider
   through the same host and client lifecycle.
6. Create a Taoryx Lab environment from the same resolved case, reset it with a
   seed, step it with actions, and produce reward/termination and provenance
   artifacts without a second simulation path.
7. Reproduce verified artifacts and plots for:
   - a Simple Aero-style configurable 3DOF family;
   - a family spanning point-mass 3DOF, pseudo-6DOF, and rigid-body 6DOF; and
   - a glider supporting both air release and booster launch.
8. Resolve operational variants through hard-valid and qualified bounds,
   coupled derived values, explicit reject/project policy, and stable
   fingerprints without mutating family definitions.

Each result must retain source hashes, selected versions, resolved values,
control authority, event history, numerical settings, and claim boundary.
It must additionally retain provider, adapter, binding, model, solver,
translation-report, approximation, and native-artifact provenance.

Taoryx itself must satisfy the complete provider/testbed proof: the same
Simple Aero case run through the provider facade and stepped through Taoryx Lab
must produce the same physical trajectory when controls, seed, cadence, and
initial state are held constant.

The reference provider may be an analytical ballistic propagator, a minimal
point-mass engine, or a replay provider. It is deliberately small: its purpose
is to prove that provider selection, common validation, batch execution,
inspection, normalized results, and negotiated stepping do not contain
Taoryx-specific branches. It need not reproduce Taoryx fidelity.

## Core contracts

### Vehicle Family Package

A Vehicle Family Package is the versioned unit of composition. It contains:

- family identity, schema version, display metadata, provenance, and envelope;
- common parameters, observations, controls, outputs, events, and invariants;
- supported point-mass 3DOF, pseudo-6DOF, and rigid-body 6DOF profiles;
- compatible component slots, variants, and loadout presets;
- fidelity-specific implementations and adapters;
- reusable segment templates and phase plans;
- guidance, autopilot, allocation, actuator, and controller presets; and
- examples, source data, numerical verification, and evidence requirements.

It is a composition boundary, not a subclass for every booster, payload,
mission, and fidelity combination.

Family packages also publish a `VariantSpace`. Independent parameters are
user- or optimizer-selectable; derived parameters are computed from declared
dependencies; deep parameters require a developer/calibration path. Semantic
modifiers such as payload, propellant load, thrust derating, drag increment,
CG shift, and actuator response replace arbitrary recursive overrides. Each
modifier declares hard/qualified bounds, affected fidelity bindings, resource
couplings, and retrim/requalification requirements.

### CaseIntent

`CaseIntent` is the portable input before preset expansion and backend
selection. It owns family, variant/loadout, mission, phase plan, controller,
overrides, requested controls/observations, fidelity requirements, backend
preferences, and namespaced extensions. It may use aliases and user-selected
units; those are resolved before execution.

### ResolvedCase

`ResolvedCase` is the immutable center of the Alpha 2 toolchain:

```text
case frontend -> parse/normalize -> catalog resolver -> ResolvedCase
                                                       -> runtime graph
                                                       -> run() or step()
                                                       -> RunArtifact
```

It contains:

- exact family, component, variant, loadout, fidelity, segment, controller,
  preset, and schema versions;
- all final parameters in canonical units;
- source, selection, derivation, and override history for every value;
- component/model graphs and capability matches;
- launch, target, waypoint, environment, and termination definitions;
- ordered segment graph and event priorities;
- state layout and fidelity adapters;
- fixed parameter, control, observation, output, and event schemas; and
- authority, scheduler, integrator, seed, and reproducibility settings.

The integrator must not reinterpret preset inheritance, configuration
precedence, or unit strings while integrating.

### CompiledCase and provider reports

`CompiledCase` is an opaque provider result. The host may inspect its identity
and lifecycle status, but only the provider adapter interprets its native model.
Every compilation also emits a translation report, for example:

| Requested concept | Provider realization | Status |
| --- | --- | --- |
| `vehicle.mass.initial` | Taoryx mass parameter | `native` |
| `command.bank` | Taoryx point-mass bank input | `native` |
| `surface.elevon_left` | No point-mass equivalent | `unsupported` |
| dynamic actuator | First-order lag | `approximated` |

Required unsupported concepts are errors. Approximation is valid only when the
case policy permits it and the result records the decision.

### Configuration versus runtime data

- Parameters configure a case before reset or at declared segment boundaries.
- Loadouts select structural components and derive assembly properties.
- Mission data describes launch, environment, targets, waypoints, and exits.
- Controls are time-varying inputs supplied during execution.
- State is runtime-owned and cannot be directly mutated by a policy or player.
- Observations are declared views of state and derived outputs.

No accepted override may be silently ignored. Unknown, inapplicable, or
misspelled overrides are resolution errors. After reset, schemas remain fixed;
stage-specific channels become inactive through activity masks rather than
changing the action or observation shape.

## Fidelity ladder

A family is one semantic vehicle with multiple model graphs, not three
unrelated simulators.

| Profile | Required semantics | Explicit non-claim |
| --- | --- | --- |
| `point_mass_3dof` | Translation, mass, lift/drag/thrust/gravity, atmosphere, and declared high-level guidance. | No rigid-body angular momentum or moment solution. |
| `pseudo_6dof` | Translation plus prescribed, filtered, or controlled attitude/rate/actuator-response state; force orientation through attitude. | Not a complete inertia-and-moment rigid-body solution. |
| `rigid_body_6dof` | Translation, quaternion attitude, body rates, mass/CG/inertia, forces, moments, effectors, and actuators as applicable. | No automatic claim of real-vehicle validity outside declared data. |

Each multi-fidelity family declares, where applicable:

1. a parameter adapter;
2. an initial-state lift/reduction;
3. a common-command to fidelity-command adapter; and
4. a fidelity-output to common-comparison projection.

Cross-fidelity verification uses one common mission, declared comparison
channels, expected divergence explanations, and higher-fidelity-only state and
control reporting.

## Components, variants, and loadouts

Initial typed component slots are:

```text
airframe | booster | engine | payload
control_surface_set | actuator_set | sensor_or_navigation_set
```

Slots declare interfaces, compatibility constraints, and optional/singular/
repeatable cardinality. Components may contribute mass, propellant, CG,
inertia, geometry, tables, forces, moments, runtime states, effectors,
actuator models, and lifecycle events.

The loadout resolver derives total mass, CG, inertia, propellant, geometry, and
available controls whenever the selected fidelity requires them. A simplified
3DOF family may expose a lumped `initial_mass`, but it must be marked as a
fidelity-specific simplification rather than conflicting silently with a
component-derived model.

Booster separation, payload deployment, fuel depletion, or control-surface
loss are declared events. They update the active model graph, mass properties,
control activity masks, and artifact event trace deterministically.

## Simple Aero concepts mapped into TAORYX

| Simple Aero concept | TAORYX concept |
| --- | --- |
| Fixed L/D by phase | Segment parameter or explicit `fixed_ld` aerodynamic mode. |
| Booster choice | Typed booster component slot in a loadout. |
| Payload/starting mass | Payload component or declared simplified loadout parameter. |
| Swappable phases | Reusable segment templates assembled into a phase plan. |
| Launch point | Mission launch specification. |
| Aim point | Mission target/aim-point specification. |
| Initial heading offset | Launch initialization parameter. |
| Burnout velocity | Powered-segment cutoff condition. |
| Autopilot behavior | Controller implementation plus versioned preset. |
| AI/player input | Validated `ControlFrame` through the authority layer. |

TAORYX distinguishes physical burnout from commanded cutoff. Physical burnout
comes from propellant depletion or a thrust-table endpoint. Commanded cutoff
comes from a declared time, speed, altitude, energy, or other mission
condition. Burnout velocity is segment-exit semantics, not hidden booster
behavior.

## Missions and reusable segments

Mission configuration stays outside the family so one family/loadout can be
reused across scenarios.

Supported launch schemas include surface/free launch, rail launch, air release,
and attached-booster launch. Each records the necessary position, velocity,
attitude/rate, rail, carrier, aim-point, heading-offset, ignition, separation,
and handoff data in named units and frames.

A waypoint declares position/frame/datum, optional altitude/speed/time/energy,
acceptance or crossing condition, arrival heading/path angle, capture/skip/fail
behavior, and guidance priority where supported.

Initial capability-based segment templates include:

```text
rail_launch | powered_boost | powered_waypoint_flight | ballistic_coast
fixed_ld_glide | controlled_glide | booster_separation
terminal_intercept | reentry | impact_or_termination
```

Each instance declares template/version, required capabilities, entry actions,
allowed overrides, active models/controls, exit conditions and priority,
handoff mapping, and safety timeout/max steps.

## Control architecture

Autopilot and external control must never both write directly to the same
physical variable:

```text
mission/waypoints -> guidance -> autopilot baseline
                   -> external-control arbiter
                   -> allocation/mixing -> limits/safety
                   -> actuator dynamics -> forces/moments/dynamics
```

Control levels remain distinct: mission intent, guidance command, vehicle
command, effector command, and runtime-owned actuator state.

Every channel supports an explicit subset of:

- `autopilot`: autopilot owns the channel;
- `commanded`: external source supplies a setpoint for autopilot tracking;
- `overlay`: external source supplies a bounded bias;
- `direct`: external source owns a physical effector; and
- `mixed`: authority is configured independently per channel.

For the Lab facade, `residual` is the public research name for a bounded
overlay: the agent supplies a correction while the autopilot remains active.
The runtime must retain both names in metadata where useful (`residual` at the
task boundary, `overlay` at the provider/control boundary). A mixed policy may
add a bank residual, directly own throttle, leave angle-of-attack limiting to
the autopilot, and keep stability augmentation enabled.

For overlay channels the ordered contract is recorded:

```text
bounded_external = clamp(external_bias, overlay_bounds)
combined = autopilot_command + bounded_external
limited = apply_hard_bounds_and_rate_limits(combined)
```

The runtime logs autopilot command, external input, post-arbitration command,
post-limit command, and actual actuator state.

For learning and player runs it additionally logs normalized agent action,
physical agent action, rejected/clipped components, action age, and the
observation profile used to produce the action. This action-to-applied-control
chain is part of the episode artifact, not an optional debug trace.

`ControlSpec` and `ObservationSpec` provide the shared metadata for AI, scripts,
players, optimization, and reports: semantic ID, type/shape, canonical unit,
frame, bounds, normalization, cadence, hold behavior, validity/activity masks,
authority, failsafe, and provenance. Normalized `[-1, 1]` values are boundary
representations only; canonical physical channels remain in artifacts.

Controls must distinguish the requested input from the achieved physical
actuator state. A channel may declare `absolute` input, `rate` input, or both;
rate input has an explicit rate unit and is integrated by the control arbiter
over the accepted transition interval. A hard actuator rate limit is separate
from a requested-rate command. The artifact records command mode, requested
rate, realized rate, post-limit value, and achieved value.

Observations are typed by role. `state` describes the dynamic state, while
`actuator_achieved` reports the current realized bank, throttle, surface,
rotor, or gimbal state. `resource` covers values such as fuel or propellant
remaining, and `event_prediction` covers available time or state-to-event
estimates such as time-to-burnout. Requested commands are never substituted
for these achieved/resource/event observations.

Family packages also declare truth-output schemas, sensor/observation profiles,
autopilot implementations, actuator presets, randomizable parameters and
valid distributions, supported task requirements, cross-fidelity comparison
channels, and verification/benchmark scenarios. A task package selects the
agent-visible subset; it does not redefine the underlying vehicle state.

The scheduler must define physics, actuator, autopilot, and external-policy
cadences and deterministic hold/ordering rules.

## Presets and resolution precedence

Presets are named, versioned, data-only partial configurations. Useful classes
are variant, loadout, mission, segment-plan, controller, autopilot-tuning, and
external-authority presets.

Recommended precedence is:

1. family defaults;
2. fidelity defaults;
3. selected variant;
4. components/loadout;
5. mission preset;
6. segment-plan preset and instance overrides;
7. controller/autopilot preset; and
8. explicit case overrides.

The resolver records the final value, unit, source layer/file, default or
override status, and derivation formula/dependencies. Parent-plus-ordered
overlays are preferred; cycles and same-priority conflicts are errors.

## Required tooling surface

The following is the target Alpha 2 CLI surface; each command becomes a release
obligation when its underlying contract is implemented:

```bash
taoryx catalog list families
taoryx family inspect simple_aero
taoryx family schema simple_aero --exposure common
taoryx preset list --family simple_aero --kind loadout
taoryx variant inspect simple_aero
taoryx variant resolve cases/simple_aero-heavy.yaml --candidate candidate.json
taoryx variant explain cases/simple_aero-heavy.yaml --parameter vehicle.mass.wet

taoryx case resolve cases/simple_aero-heavy.yaml --output resolved.json
taoryx case explain cases/simple_aero-heavy.yaml --parameter vehicle.mass.total
taoryx case diff cases/simple_aero-light.yaml cases/simple_aero-heavy.yaml
taoryx case validate cases/kestrel-air-release.yaml
taoryx graph show cases/kestrel-air-release.yaml

taoryx schema export cases/kestrel-air-release.yaml --kind controls
taoryx schema export cases/kestrel-air-release.yaml --kind observations
taoryx schema export cases/kestrel-air-release.yaml --normalized

taoryx run cases/simple_aero-heavy.yaml
taoryx compare fidelities cases/kestrel-course.yaml \
  --levels 3dof,pseudo_6dof,rigid_body_6dof
taoryx family verify simple_aero --all
taoryx family verify kestrel-glider --cross-fidelity
```

`case explain` is a priority feature: it must show exactly where a value came
from and how it was derived.

`variant explain` must additionally show parameter role, hard and qualified
ranges, modifier coupling, projection/rejection decisions, and whether the
resolved candidate invalidates trim or qualification evidence.

## Alpha 2 proof families

### Shared provider/Lab proof

Use one resolved Simple Aero case twice:

```text
Taoryx Provider: resolve -> compile -> run/step -> trajectory result
Taoryx Lab:     resolve -> compile -> reset -> action/observation loop
                                             -> reward/termination
```

Compare state transitions, applied controls, events, and termination. Any
difference must be attributable to an explicitly declared observation, action,
task, or randomization layer—not a second integrator or hidden controller.

### A — Simple Aero-style configurable 3DOF

Demonstrate multiple boosters, payload/loadout mass, fixed L/D by segment,
swappable phase plans, launch/aim points, heading offsets, physical and
commanded burnout, bounded overlays, and parameter sweeps without source edits.

### B — Complete fidelity-ladder family

Run one family and waypoint mission as `point_mass_3dof`, `pseudo_6dof`, and
`rigid_body_6dof`. Demonstrate common parameters and guidance, fidelity-specific
commands, autopilot presets, direct effectors, output projections, and expected
divergence.

### C — Dual-launch glider

Use one glider family for air release and attached-booster launch. Demonstrate
release/separation handoff, waypoint guidance, pseudo-6DOF easy controls,
rigid-body direct surfaces, autopilot-only operation, bounded overlay, and
direct-control operation.

The complete-fidelity family and glider may be the same family when the
composition remains physically coherent.

### D — Testbed progression

After the provider/Lab proof, extend the same family through:

1. pseudo-6DOF attitude and response dynamics;
2. rigid-body angular dynamics and moments;
3. physical actuators and control surfaces;
4. sensor noise, latency, and bias; and
5. cross-fidelity task and policy-transfer evaluations.

The action space may remain high-level while fidelity increases. Low-level
attitude/rate and direct-effector tasks are additional profiles, not mandatory
replacements for guidance-level control.

## Release gates

| Gate | Name | Exit condition |
| --- | --- | --- |
| A2-R0 | Neutral contract freeze | `CaseIntent`, `ResolvedCase`, `CompiledCase`, schemas, capabilities, translation reports, sessions, authority, and artifacts are published without Taoryx imports. |
| A2-R1 | Host and adapter SDK | Provider discovery, selection, lifecycle, and adapter/binding interfaces work through the common host. |
| A2-R2 | Catalog foundation | Versioned families/components/segments/controllers are discoverable with stable semantic IDs. |
| A2-R3 | Resolver/provenance | Units, compatibility, precedence, derived values, and provenance resolve immutably. |
| A2-R3V | Bounded variants | Independent/derived roles, hard/qualified bounds, semantic modifiers, deterministic reject/project, and variant fingerprints pass without silent repair. |
| A2-R4 | Fixed runtime schemas | Parameter, state, control, observation, output, event, and activity-mask schemas remain fixed after reset. |
| A2-R5 | Reference-provider parity | A non-Taoryx reference provider passes common validation, compilation, batch execution, inspection, normalized results, and any advertised session capabilities. |
| A2-R6 | Taoryx adapter | Taoryx is reachable only through its registered execution adapter and family bindings, with complete translation reports. |
| A2-R7 | Provider/Lab parity | The same resolved Simple Aero case produces equivalent physical transitions through provider batch/step and Taoryx Lab action loops. |
| A2-R8 | Simple Aero proof family | Configurable 3DOF boosters, payloads, fixed L/D, launch/aim, cutoffs, segments, and sweeps pass. |
| A2-R9 | Control authority | Autopilot, commanded, residual/overlay, direct, and mixed modes pass limits, cadence, logging, and replay tests. |
| A2-R10 | Fidelity ladder | One family runs the same mission at all three fidelities with declared adapters/tolerances. |
| A2-R11 | Dual-launch glider | Air-release and attached-booster paths pass deterministic handoff and mission evidence. |
| A2-R12 | Lab testbed | **Post-release:** seeded reset, action/observation profiles, reward, termination/truncation, randomization, and evaluation artifacts. |
| A2-R13 | Tooling/artifacts | Inspect, resolve, explain, diff, schema export, run, compare, verify, plot, reset, rollout, and replay paths emit audit artifacts. |
| A2-R14 | Release reproducibility | A clean checkout reproduces proof cases, reports, plots, and hashes without stop-ship findings. |

A2-R9 is the release gate. An aggregate score cannot substitute for an
individual gate.

## Evidence contract

Every Alpha 2 case artifact includes:

- case source and resolved-case hashes;
- family/component/variant/loadout/fidelity/preset versions;
- final parameters, derivations, defaults, and override records;
- model/segment/component graph and capability matches;
- fixed state/control/observation/event schemas;
- launch, waypoint, authority, and scheduler configuration;
- all command-arbitration stages;
- telemetry, events, termination reason, diagnostics, and plots;
- source/table/model hashes, numerical settings, seed, and software version; and
- comparison tolerances and claim classification.

Plots are views of artifacts, never the evidence source. Normalized controls and
display units must be accompanied by canonical physical channels.

## Implementation order

### A2-0 — Neutral contracts and package boundary

Define the neutral schemas, capability statuses, provider/family identities,
case lifecycle, translation reports, result provenance, and optional session
operations. Add import-boundary tests proving neutral packages do not import
Taoryx.

Exit: a provider-neutral client can validate and describe a case without
loading Taoryx.

### A2-1 — Host, adapter SDK, and reference provider

Implement provider discovery, compatibility selection, lifecycle management,
and the adapter/binding split. Register a tiny analytical or replay provider
and run the same host/client conformance suite against it and a stub Taoryx
adapter path.

Exit: provider-specific branches are absent from common host operations, and
capabilities determine whether interactive operations are available.

### A2-1b — Canonical session and Taoryx Lab facade

Expose canonical reset, transition, observation, and event semantics to both
the provider adapter and Taoryx Lab. Add task, reward, termination/truncation,
randomization, seed, rollout, replay, and action-to-applied-control contracts
without placing them in the physics package.

Exit: provider batch/step and Taoryx Lab stepping agree on physical transitions
for the shared Simple Aero proof case.

### A2-2 — Semantic schema and catalog

Implement stable IDs, typed metadata, family package discovery, capability
declarations, and schema export. Start with a small catalog, not a large vehicle
library.

Exit: a package can be inspected and its schemas generated without runtime
initialization.

### A2-3 — Resolver and provenance

Implement variants, loadouts, presets, override scopes, units, frames, derived
values, compatibility matching, immutable `ResolvedCase`, `case explain`, and
`case diff`.

Exit: derived mass, CG, inertia, launch heading, and other values are
inspectable; unknown/unused overrides are errors; identical inputs have stable
case identity hashes.

### A2-4 — Simple Aero 3DOF family and Taoryx binding

Build the configurable proof family: boosters, payloads, fixed L/D,
launch/aim, heading offset, target-speed cutoff, swappable segments, physical
burnout, commanded cutoff, and parameter sweeps.

Exit: configuration changes use case data and generated artifacts rather than
source edits or copied runners.

Add the first Taoryx Lab task profiles over this family: terminal accuracy,
energy management, and bounded residual guidance. Keep reward and episode
definitions in task metadata.

### A2-5 — Common control stack

Implement command levels, metadata, authority arbitration, controller presets,
cadences, normalized adapters, activity masks, limits, failsafes, and
intermediate-command logging.

Exit: the same `ControlFrame` contract accepts autopilot, script, AI, and player
adapters; replay is deterministic; effectors cannot bypass limits/dynamics.

### A2-6 — Pseudo-6DOF profile

Add explicit attitude/response dynamics, easy-to-drive commands, waypoint
autopilot integration, and force-orientation coupling. Publish what is
prescribed/filtered and what is integrated.

Exit: bridge parity preserves common translational channels while attitude,
rate, and authority channels remain independently auditable.

### A2-7 — Rigid-body 6DOF profile

Add package support for mass properties, moments, quaternions, physical
effectors, actuator models, direct-control interfaces, and fidelity adapters.
Use the existing convention firewall and trim/evidence harnesses.

Exit: one complete family runs a common mission at all three fidelities and
reports expected divergence.

### A2-8 — Dual-launch glider

Use the same family and mission tooling for air release and attached-booster
launch, including deterministic separation handoff, waypoint guidance,
autopilot-only, overlay, and direct-control cases.

Exit: both launch forms produce continuous, event-annotated, source-hashed
artifacts with shared comparison channels.

### A2-9 — Documentation and hardening

Generate family catalogs, preset tables, control/observation schemas, reference
pages, compatibility matrices, example indexes, plots, and release packets from
the same metadata used by validation.

Exit: clean-checkout reproduction passes and Alpha 2 public schemas are frozen.

## Migration and repository target

The eventual organization is:

```text
catalog/
  components/{boosters,payloads,engines,effectors}/
  segments/
  controllers/
  families/{simple_aero,kestrel-glider}/
    family.yaml
    fidelities/ variants/ loadouts/ segments/
    controllers/ presets/ examples/ verification/
cases/
spec/semantics/{parameters,controls,observations,events}/
src/taoryx/{catalog,schemas,resolution,composition,control,runtime,verification,artifacts,cli}/
tests/
```

Shared components, segments, and controllers belong in shared catalog folders.
Family-local resources are appropriate only when behavior is genuinely
family-specific. Existing vehicle metadata and generated problem files remain
migration inputs until the package catalog is authoritative.

## Stop-ship conditions and non-goals

Do not declare Alpha 2 complete if a supported combination needs copied runner
code, an override is silently ignored, a derived value lacks provenance, a
pseudo-6DOF result is presented as rigid-body 6DOF, external control bypasses
authority/limits, schemas change after reset, physical and commanded burnout
are conflated, a separation handoff is not event-annotated, normalized values
replace canonical artifact channels, plots cannot be regenerated, or missing
physics are hidden behind guessed defaults. Also stop if provider and Lab use
separate transition implementations, rewards or episode rules leak into
vehicle physics, action-to-applied-control provenance is missing, seeded reset
or evaluation replay is not reproducible, or fidelity is conflated with
control abstraction.

Alpha 2 does not include a remote model marketplace, untrusted hot-loading,
automatic 3DOF-to-valid-6DOF conversion, guaranteed policy transfer across
fidelities, a required RL/game/network framework, unbounded runtime schema
changes, generic undeclared control mixing, or one universal autopilot.

## Flagship showcase backlog

The [Four-Family Flight Showcase v1](four-family-flagship-flight-showcase-v1.md)
is the next qualification layer above the Alpha 2 family proofs. It is a
single versioned release containing one `Flagship Mission Qualification Pack`
for each archetype:

```text
Hummingbird: pad -> hover -> 3D waypoint box -> yaw -> disturbance -> land
Skywalker X8: launch -> bidirectional course -> speed/altitude -> recovery
B747: airborne trim -> climb/turn/accelerate -> descent -> arrival gate
X-15: air release -> powered climb -> burnout/coast -> glide -> energy corridor
```

The public badge is `Flagship Mission Qualified`. It is earned only when the
same resolved case can be launched or initialized through its documented start
contract, complete qualified segments, exercise the required controls and
effectors, capture ordered objectives, reach explicit family-appropriate
finality, and reproduce all artifacts from one command. The hero flight sits
on top of source/data, trim/hover, closure, convergence, actuator, controller,
segment, robustness, and replay evidence; it does not replace them.

The first implementation slice is common tooling: waypoint and terminal-
corridor evaluators, control-coverage reports, envelope reports, metric
dictionary, common plots, self-contained manifests, and batch/step equivalence.
Then implement Hummingbird, X8, B747, and X-15 in that order. Runway-to-runway
B747 and physical X8 touchdown remain separate follow-on badges until their
low-speed and ground-contact models qualify.

Supporting artifacts are recorded in `verification/showcase_validation_roadmap_v2.json`
and `verification/showcase_composites_v1_audit.md`.

## Fleet, space, and public-surrogate backlog

The [Fleet, Space, and Public-Surrogate Expansion Plan](fleet-space-public-surrogate-expansion.md)
extends Alpha 2 library maturity after the flagship layer:

1. Add a first-class `PopulationSession` with structure-of-arrays state,
   cohort grouping, sparse events, spatial indexing, scalar/batched
   equivalence, deterministic replay, and documented wall-clock/memory gates.
2. Qualify Hummingbird as one family with rotor-resolved rigid-body 6DOF,
   named attitude-response pseudo-6DOF, and `fleet_3dof` realizations.
3. Add a `reference_satellite` domain profile with explicit epochs, time
   scales, inertial frames, central bodies, force models, eclipse, visibility,
   CCSDS orbit data, and orbital/attitude/rigid-body realizations.
4. Define public-surrogate evidence before encoding product data. Tag every
   value as public fact, derived estimate, generic assumption, calibration
   choice, scenario convenience, or unknown.
5. Build the first three surrogates initially: tandem-rotor VTOL,
   tube-launched fixed wing, and reusable twin-jet VTOL. High-performance jet
   and modular cruise surrogates follow; Thunder remains catalog-only `E0`/`M0`.

These tracks reuse the same provider, Lab, composition, stepping, evaluation,
provenance, and readiness infrastructure. They do not create independent
fleet, spacecraft, or surrogate runtimes.

The ranked backlog treats the Cessna-class, rotorcraft/VTOL, spacecraft,
small-business-jet, and Anduril-inspired public-surrogate families as P2
domain pilots. They come after the generic P0 seams and current-family P1
proof, but before broad parametric qualification. This order tests whether the
common contracts truly generalize across low-speed propeller flight, hover and
transition, atmospheric jets, public surrogates, and orbital domains without
turning any domain into a bespoke runtime.

All future library additions must follow the
[model integration workflow](model-integration-workflow.md). The F-16 S-119
and HL-20 Mod K packages enter as source-grounded reference anchors with
immutable plant records; actuator, controller, mission, reduction, and RL
layers are separate integration work items.

The spacecraft family backlog expands these anchors into standard and resilient
`spacecraft.6u_observer_rw` variants, `spacecraft.agile_imager_rw.v1`,
`spacecraft.spheres_like_rcs.v1`, and `spacecraft.marco_like_hybrid.v1`.
CPOD-like proximity and one-thruster-out variants remain follow-on
qualification work. Corrected fixture values and lifecycle gates are recorded
in `spacecraft-parametric-fixtures-engineering-review.md`.

Spacecraft 6DOF is a dedicated qualification track documented in
[spacecraft-6dof-data-and-examples.md](spacecraft-6dof-data-and-examples.md).
Alpha 2’s space backlog begins with a reaction-wheel-primary Earth-observation
vehicle and a thruster-primary free flyer, followed by a hybrid. Required
evidence includes wheel momentum exchange/unloading, thruster attainable
wrench and rank checks, pulse/valve behavior, propellant use, failures, frame
and epoch correctness, and reproducible deployment/pointing or stationkeeping
missions.

The [Parametric Models and ML Trajectory Corpus Plan](parametric-models-ml-trajectory-corpus.md)
is a first-class backlog workstream, not an extension-field checklist. It
adds coherent latent vehicle sampling, identification fragments, multi-layer
truth, sensorized corpus editions, realized-vehicle leakage-safe splits, and
P0–P5 corpus maturity gates. Source-grounded F-16 and HL-20 families remain
immutable external anchors; synthetic archetypes provide controlled breadth.

The [Trajectory Reachability Workbench](trajectory-reachability-workbench.md)
is a separate application over the provider/session API. It owns launch and
target loci, candidate fan-out, search, adaptive boundary extraction,
uncertainty studies, classification, and visualization; Taoryx owns case
resolution, checkpoint/restore, deterministic stepping, controls, telemetry,
and validity reporting. Its first target is R3 along-track fan capability,
with R5 as the qualified workbench gate.

The [sensor and measurement orchestration backlog](sensor-measurement-orchestration.md)
adds committed truth staging, synthetic measurement production, causal
multi-rate scheduling, estimator truth isolation, and IMU/focal-plane adapter
contracts. It is shared by flight, RL, reachability, and ML-corpus studies.

####
