# Taoryx Vehicle Readiness Guide

**Status:** Alpha 2 planning standard  
**Scope:** successor-side vehicle families, fidelity realizations, composition, trajectory evaluation, and Taoryx Lab readiness

This guide defines when a vehicle family is ready for another person to
configure, compose, control, run, and evaluate without editing model
implementation. A single successful example trajectory is evidence of
execution, not qualification.

For the concrete equation-tier and trim workflow, use
[Trim And Fidelity Walkthrough](trim-and-fidelity-walkthrough.md) before
adding controller or mission evidence.

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

## Equation-driven fidelity data contract

Fidelity is defined by the equations that consume the data—not by state-vector
length, control names, or how detailed the vehicle appears.

### Point-mass 3DOF: force-complete

The canonical point-mass model integrates translation:

```text
m v_dot = sum(F)
```

It does not integrate a physical angular-momentum equation. Heading, bank,
angle of attack, or displayed attitude may exist, but they are commanded,
algebraically derived, or imposed by guidance. Applied moments do not determine
attitude. The realization is **force-complete**, not moment-complete.

Required active data includes total mass and resource evolution, reference
area, atmospheric force laws, thrust magnitude/direction and resource use,
control-to-force or trajectory-command mappings, and declared units, frames,
envelopes, interpolation, and provenance. A fixed L/D schedule still needs a
drag-force scale such as `C_D` plus reference area, ballistic coefficient, or a
validated equivalent. L/D alone is insufficient.

### Pseudo-6DOF: attitude-response-complete

The model integrates translation and usually explicit attitude/rate state:

```text
m v_dot = sum(F)
q_dot = 1/2 Omega(omega) q
omega_dot = f_response(omega, omega_cmd, x, u)
```

The angular response comes from a named empirical or commanded response law,
not physical moments and inertia. The realization is **force-complete and
attitude-response-complete**, but not necessarily moment-complete.

Every pseudo profile must identify its law, for example:

- `p6dof_kinematic_attitude`: directly commanded or rate-limited attitude;
- `p6dof_attitude_response`: first/second-order response with gains,
  time constants, natural frequencies, damping, and scheduling; or
- `p6dof_effector_response`: surface-like inputs mapped to empirical
  attitude/rate response and force increments.

Response schedules must cover the claimed flight conditions, loadouts, and
authority limits. A model that begins using physical inertia and moments to
determine angular acceleration is no longer pseudo-6DOF; classify it as a
simplified or reduced-data rigid-body 6DOF realization.

### Rigid-body 6DOF: moment-, mass-property-, and actuator-complete

The model integrates Newton–Euler translation and rotation:

```text
m v_dot = sum(F)                      # exact frame convention is declared
I omega_dot + omega x (I omega) = sum(M)
```

The implementation must declare its exact translational frame convention; the
rotational equation must use a physical inertia tensor about the current center
of gravity. For variable mass, it must also disclose center-of-gravity,
inertia, and variable-mass momentum treatment.

The decisive classification rule is:

> If applied moments and a physical inertia tensor determine angular
> acceleration, the model is 6DOF—even when aero, inertia, or derivatives are
> simplified. If attitude follows a kinematic, commanded, first-order, or
> second-order response instead, it is pseudo-6DOF.

### Fidelity data matrix

| Data product | 3DOF point mass | Pseudo-6DOF response | Rigid-body 6DOF |
| --- | --- | --- | --- |
| Total mass and active mass evolution | Required | Required | Required |
| Center of gravity | Composition metadata/conditional | Conditional for schedules and handoffs | Required |
| Inertia tensor/evolution | Not consumed | Not consumed by canonical response law | Required about CG |
| Reference area and force data | Required | Required | Required |
| Moment data and angular-rate effects | Not consumed | Not consumed by canonical response law | Required or explicitly justified |
| Thrust magnitude/direction/resource use | Required when powered | Required when powered | Required when powered |
| Thrust application point/torque | Metadata or ignored | Optional metadata/surrogate | Required when moment-relevant |
| Effector limits and dynamics | Optional for surrogate controls | Required for exposed effectors | Required |
| Control-to-force mapping | Required when controlled | Required | Required |
| Control-to-moment effectiveness | Not consumed | Not consumed by canonical response | Required for each physical effector |
| Units, frames, signs, envelope, provenance | Required | Required | Required |

“Required” is feature-gated: an unpowered body does not need propulsion data,
and a vacuum body does not need atmospheric coefficients. A control-surface
name and deflection limit alone never establish 6DOF control authority.

### Mass-property rules

3DOF needs enough data to determine translational acceleration and resource
evolution: initial/dry mass, consumables, payload/loadout contributions,
flow or schedule, staging/deployment changes, and minimum permitted mass. It
may use a scalar schedule or component aggregation. CG, inertia, and lever arms
are retained as family metadata when other fidelities, handoffs, or composition
need them, but are not claimed as consumed by point-mass equations.

Rigid-body 6DOF requires, for each supported configuration, mass, CG vector,
full symmetric inertia tensor about CG, body datum/frame, component properties,
consumable behavior, deployment/staging changes, and attachment/release state.
The validator must reject or flag nonpositive mass, nonphysical inertia,
unidentified reference points/frames, undeclared discontinuities, unresolved
component transforms, and payload release without a post-release state.

Composite loadouts should aggregate component properties from declared
transforms rather than requiring a manually entered tensor for every possible
combination.

### Propulsion and effector rules

Force-only propulsion requires magnitude, direction, resource flow, and
ignition/cutoff behavior. Rigid-body propulsion additionally requires
application point, engine frame, gimbal limits/dynamics, torque where relevant,
and health/failure state. The application point is stored in a fixed body datum;
the runtime derives the current lever arm:

```text
M_thrust = (r_engine - r_cg) x T + M_engine
```

Each physical effector separates three contracts: identity and frame, actuator
behavior, and force/moment effectiveness. A 6DOF aerodynamic effector needs
force and moment coefficient increments or an equivalent declared model over
the applicable Mach, alpha, beta, configuration, and rate envelope. Geometry
alone is not effectiveness data.

### Machine-readable declaration

Every fidelity package should expose an inspectable declaration like:

```yaml
fidelity_profiles:
  point_mass_3dof:
    equations: {translation: newton_point_mass, rotation: none}
    completeness: [force]
    required_data:
      mass: scalar_schedule
      aerodynamics: forces_when_atmospheric
      propulsion: force_when_powered
      effectors: semantic_or_force_surrogate
  attitude_response_p6dof:
    equations:
      translation: newton_point_mass
      orientation: quaternion_kinematics
      rotation: scheduled_attitude_response
    completeness: [force, attitude_response]
    required_data:
      mass: scalar_schedule
      response: named_profile_and_schedule
      effectors: actuator_plus_response_map
  rigid_body_6dof:
    equations:
      translation: newton_center_of_mass
      orientation: quaternion_kinematics
      rotation: newton_euler
    completeness: [force, moment, mass_properties, actuators]
    required_data:
      mass: component_or_state_dependent
      center_of_gravity: body_frame_schedule
      inertia: full_tensor_about_cg
      aerodynamics: forces_moments_rates_and_controls_when_atmospheric
      effectors: actuator_plus_force_moment_model
```

The declaration also records claimed envelope, applicable segments, supported
loadouts, initialization requirements, extrapolation policy, omitted physics,
and evidence links.

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
3. Simple Aero-style configurable 3DOF.
4. One complete powered fixed-wing multi-fidelity family.
5. Rocket staging and composite payload support.
6. Hypersonic glider.
7. Multirotor/VTOL.
8. Cruise and autonomous fixed-wing families.
9. Broader trajectory-provider adapters.

## Alpha 2 acceptance

Alpha 2 is readiness-ready when the Simple Aero proof family demonstrates M5 for
its advertised 3DOF realization, with the qualification schema, graded
composition checks, typed starts, classified finality, evaluator artifacts,
provider/Lab parity, and complete provenance. Additional fidelities receive
their own maturity and badge records; they are not promoted by association.

The same rule applies to every future family: “good to go” is a qualification
claim bounded by fidelity, configuration, envelope, evidence class, and
supported task—not merely a runnable example.

The library-breadth backlog is extended by the
[Fleet, Space, and Public-Surrogate Expansion Plan](fleet-space-public-surrogate-expansion.md).
It adds Hummingbird `rigid_body_6dof`, named attitude-response pseudo-6DOF,
and `fleet_3dof` realizations; a `reference_satellite` orbital/attitude family;
and generic public-data autonomous-aircraft surrogates.

Public-surrogate evidence is separate from engineering maturity: `E0`
conceptual archetype, `E1` public descriptor, `E2` public envelope, `E3` public
behavioral correlation, and `E4` authorized correlation. Unknown values remain
ranges or distributions. They never become precise-looking specifications;
the initial surrogates are not proprietary replicas, and Thunder remains
catalog-only `E0`/`M0`.

Reachability studies are a separate analysis product described in the
[Trajectory Reachability Workbench plan](trajectory-reachability-workbench.md).
An Effective Kinematic Reachability Envelope is bounded by vehicle, fidelity,
loadout, environment, controller class, launch/target state families, path
constraints, and terminal success criteria. It is not an intrinsic vehicle
property. A witness proves declared-model feasibility; an unsuccessful search
is `unresolved_search`, not physical impossibility.

Spacecraft readiness follows the [Spacecraft 6DOF data and qualification
track](spacecraft-6dof-data-and-examples.md). Reaction wheels and thrusters are
distinct actuator families: wheels exchange and store angular momentum and
require external unloading, while thrusters generate external force/moment,
consume propellant, and have pulse/valve/attainable-wrench constraints. A
spacecraft family is not 6DOF-qualified from an inertia tensor and generic
torque input alone.

The spacecraft breadth backlog now includes the
[parametric spacecraft family expansion](spacecraft-parametric-family-expansion.md):
6U observer, agile imager, SPHERES-like free flyer, and MarCO-like hybrid. Each
uses evidence labels such as `public_vehicle_fact`, `public_component_range`,
`formula_derived`, `uniform_geometry_estimate`, and
`taoryx_engineering_nominal` so recognizable architecture does not become a
false claim of exact vehicle replication.

The next showcase layer is the [Four-Family Flight Showcase v1](four-family-flagship-flight-showcase-v1.md).
It adds a `Flagship Mission Qualified` badge above family and fidelity
qualification. The badge requires one complete, family-appropriate mission for
each advertised pack, with public start and terminal contracts, meaningful
control/effector coverage, ordered waypoint or objective capture, explicit
envelope and convergence evidence, and one-command reproducibility. It does
not promote a B747 to runway lifecycle readiness, an X8 to physical landing
readiness, or any family to global realism without the corresponding lower-
level evidence.

See [Alpha 2](taoryx-alpha-2.md) for release sequencing and
[`verification/alpha2_release_plan.yaml`](../../verification/alpha2_release_plan.yaml)
for machine-readable gates.

####
