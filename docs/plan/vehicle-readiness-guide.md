# Taoryx Vehicle Readiness Guide

**Status:** Alpha 2 planning standard  
**Scope:** successor-side vehicle families, fidelity realizations, composition, trajectory evaluation, and Taoryx Lab readiness

This guide defines when a vehicle family is ready for another person to
configure, compose, control, run, and evaluate without editing model
implementation. A single successful example trajectory is evidence of
execution, not qualification.

## The readiness claim

The target release label is **M5 — Qualified / Pickup-Ready**:

> A new consumer can select an advertised fidelity, variant, and loadout; set
> documented parameters and initial conditions; compose a supported path from
> qualified mission nodes; pass graded preflight validation; run batch or
> stepwise; and receive reproducible results with finality, evaluation,
> provenance, control status, approximation disclosures, and known limits.

Engineering maturity and physical evidence strength are separate claims. A
synthetic research model may be M5 for controls or RL without claiming
predictive accuracy for a real vehicle.

## Maturity ladder

| Level | Name | Required meaning |
| --- | --- | --- |
| M0 | Cataloged | Identity, archetype, owner, intended fidelities, and data status are declared. |
| M1 | Executable | A canonical case initializes and reaches a declared terminal condition. |
| M2 | Coherent | Units, frames, states, equations, data coverage, and numerical behavior are internally verified over a limited envelope. |
| M3 | Operable | Start contracts, controls or explicit uncontrolled status, telemetry, stepping, events, finality, and batch/step equivalence work. |
| M4 | Composable | Components, loadouts, segments, objectives, and tested transitions assemble through public configuration with preflight validation. |
| M5 | Qualified / Pickup-Ready | Every applicable qualification dimension passes; a new consumer needs no model-code edits. |
| M6 | Reference | Broad envelope coverage, stronger correlation, uncertainty treatment, performance characterization, and long-term maintenance exist. |

Maturity is cumulative, but the number is not enough. Every family and every
advertised fidelity also carries a dimension scorecard.

## Qualified family structure

```text
vehicle archetype
    └── vehicle family
          ├── variants and component slots
          ├── loadouts
          ├── named fidelity realizations
          ├── start contracts
          ├── segment templates
          ├── objective fragments
          ├── controllers and presets
          ├── evaluators
          └── qualification evidence
```

An archetype identifies a broad physical pattern. A family is one coherent
semantic vehicle. A variant changes a supported configuration. Components
occupy declared slots, and loadouts are named compatible selections. A fidelity
realization is an independently qualified implementation of that family.

Pseudo-6DOF must have a specific profile name, such as
`kinematic_attitude_p6dof`, `attitude_response_p6dof`, or
`surrogate_moment_p6dof`. The profile must state retained/eliminated states,
surrogate behavior, mappings, supported segments, envelope, expected
disagreement, and known non-equivalences.

## Qualification dimensions

Every applicable dimension must score at least `3 — Qualified` for M5:

| Dimension | Qualification question |
| --- | --- |
| Data | Are geometry, mass properties, aerodynamics, propulsion, actuators, interfaces, units, frames, coverage, interpolation, uncertainty, sources, and hashes traceable? |
| Dynamics | Are states, equations, frames, force/moment composition, resources, topology changes, and event handling defined? |
| Numerics | Are stability, time-step sensitivity, determinism, seed behavior, and numerical failures characterized? |
| Controls | Are semantic commands, allocation, effectors, actuator limits/dynamics, authority modes, health, and presets defined and tested? |
| Lifecycle | Do advertised starts initialize correctly, and do runs end with classified finality? |
| Segments/objectives | Do reusable physical regimes and objective fragments have contracts and tests? |
| Composition | Are compatibility, attachments, mass properties, handoffs, resources, data overlap, solver needs, and event ordering checked? |
| Evaluation | Are numerical, physical, envelope, mission, control, resource, safety, robustness, and cross-fidelity measures available? |
| Verification/validation | Is there evidence for both implementation correctness and the strength of the model’s external evidence? |
| Packaging/usability | Can an independent consumer understand and use schemas, examples, APIs, plots, provenance, and limits? |
| Cross-fidelity | Are reductions, mappings, benchmarks, and expected disagreement documented? |
| AI/RL readiness | Are reset, seeding, action/observation profiles, stepping, termination, randomization, replay, and rollout behavior qualified? |

Scores are `0 Missing`, `1 Draft`, `2 Usable`, `3 Qualified`, `4 Reference`, or
`N/A` with an explanation. AI/RL readiness is required for the `RL-Ready`
badge; control may be N/A only for an explicitly uncontrolled family.

## Data, dynamics, controls, and lifecycle

The package must version and trace geometry, reference dimensions, mass/CG/
inertia evolution, aerodynamics, propulsion, resource consumption, effectors,
actuators, attachment and separation interfaces, and dataset semantics.
Synthetic data is permitted when labeled as synthetic.

The control path is explicit:

```text
semantic command -> autopilot/controller -> allocation
                 -> physical effector -> actuator state
                 -> forces and moments
```

Telemetry records user or agent command, autopilot output, residual/overlay,
allocated command, limits, actual actuator state, saturation, and health.
Each effector declares axes, limits, rates, latency, neutral/trim, resources,
force/moment contribution, active segments, failures, jams, and effectiveness.

Starts are typed rather than universally requiring takeoff: arbitrary state,
trimmed flight, runway, pad, rail/canister, air release, parent attachment,
booster handoff, post-separation, and randomized in-air reset are examples.

Every run ends with classified finality:

```text
success | failure | truncation
```

Reason codes include touchdown, impact, aim-region entry, burnout, commanded
cutoff, deployment, fuel exhaustion, loss of control, envelope violation,
timeout, and numerical failure. Reaching maximum simulation time is not,
by itself, success.

## Segments, objectives, and composition

A segment describes the physical regime; an objective fragment describes what
the vehicle is trying to accomplish. A mission node combines both:

```yaml
segment: energy_management_glide
objective: follow_waypoint_corridor
controller: {preset: nominal_glide}
transition:
  on_success: terminal_approach
  on_energy_floor: abort_descent
```

Composition status is graded:

| Status | Meaning |
| --- | --- |
| Schema-valid | Names, types, units, and graph structure are valid. |
| Initializable | The graph and initial state can be constructed. |
| Smoke-valid | A bounded deterministic preflight has no immediate failure. |
| Transition-qualified | Every selected segment edge has a qualifying test. |
| End-to-end qualified | The exact composition, or declared equivalence class, has a regression baseline. |

A qualified segment library does not imply every arbitrary segment ordering is
qualified.

## Trajectory evaluation

The evaluator must distinguish:

1. The simulator executed.
2. The trajectory remained physically feasible.
3. The mission succeeded.
4. The result remained inside its qualified envelope.

Reports should cover conservation and time-step sensitivity, mass/inertia/
quaternion/resource validity, Mach/altitude/angle-of-attack/sideslip/dynamic
pressure, endpoint and waypoint success, tracking/overshoot/saturation,
fuel/battery/range/reserve, loads and thermal proxies, clearance and collision,
seeded robustness, and cross-fidelity event/endpoint/energy comparisons.

## Evidence and badges

Evidence classes are `synthetic`, `analytical`, `reference-correlated`,
`test-correlated`, and `flight-correlated`. Reports must state engineering
maturity separately:

```text
Engineering maturity: M5 Pickup-Ready
Qualified fidelities: point_mass_3dof, attitude_response_p6dof, rigid_body_6dof
Badges: Multi-Fidelity Qualified; RL-Ready for guidance and residual control
Evidence: synthetic + analytical verification
Claimed use: trajectory, control, and RL research
Not claimed: predictive accuracy for a specific real vehicle
```

Recommended independent badges are `Runnable`, `Operable`, `Composable`,
`Pickup-Ready`, `Multi-Fidelity Qualified`, `RL-Ready`, and `Reference`.

## Breadth roadmap

Initial archetypes are free rigid bodies, ballistic/reentry bodies, rockets,
powered fixed-wing aircraft, cruise vehicles, hypersonic gliders, multirotors/
VTOL, autonomous fixed-wing/VTOL, payload/deployable vehicles, and composite
carrier–booster–payload systems. These must share reusable kernels for frames,
units, gravity/atmosphere, point-mass and rigid-body dynamics, mass properties,
aero, propulsion, actuators, topology, events, finality, and evaluation.

The vertical-slice order is:

1. Qualification schemas, lifecycle, composition, and evaluator infrastructure.
2. Free rigid body and basic ballistic body.
3. Spectre-style configurable 3DOF.
4. One complete powered fixed-wing multi-fidelity family.
5. Rocket staging and composite payload support.
6. Hypersonic glider.
7. Multirotor/VTOL.
8. Cruise and autonomous fixed-wing families.
9. Broader trajectory-provider adapters.

## Alpha 2 acceptance

Alpha 2 is readiness-ready when the Spectre proof family demonstrates M5 for
its advertised 3DOF realization, with the qualification schema, graded
composition checks, typed starts, classified finality, evaluator artifacts,
provider/Lab parity, and complete provenance. Additional fidelities receive
their own maturity and badge records; they are not promoted by association.

The same rule applies to every future family: “good to go” is a qualification
claim bounded by fidelity, configuration, envelope, evidence class, and
supported task—not merely a runnable example.

See [Alpha 2](taoryx-alpha-2.md) for release sequencing and
[`verification/alpha2_release_plan.yaml`](../../verification/alpha2_release_plan.yaml)
for machine-readable gates.

####
