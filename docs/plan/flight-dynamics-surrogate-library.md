# Taoryx flight-dynamics surrogate library

This workstream defines flight-dynamics surrogates for publicly described
autonomous-aircraft families. These are engineering proxies for trajectory,
control, reachability, and corpus work; they are not mechanical display models,
classified reconstructions, manufacturer-grade aerodynamic models, or claims
about undisclosed production specifications.

The latest numerical corrections and acceptance gates are recorded in the
[Anduril-inspired surrogate engineering review](anduril-surrogate-engineering-review.md).
That review supersedes earlier loose values where they conflict.

## Library boundary

The first roster contains thirteen named configurations, implemented by four
shared equation-of-motion families:

| Configuration group | Initial identities | Dynamics family |
| --- | --- | --- |
| Small VTOL and rotorcraft | `bolt`, `bolt_m`, `ghost_x`, `anvil` | multirotor / rotorcraft kinematic and attitude-response |
| Tube-launched fixed wing | `altius_600`, `altius_700_isr`, `altius_700m` | conventional fixed wing |
| Cruise and high-performance fixed wing | `barracuda_100`, `barracuda_250`, `barracuda_500`, `fury_fq44` | conventional fixed wing |
| Hover-to-cruise aircraft | `roadrunner`, `omen` | tailsitter / vectored-thrust hybrid |
| Large hybrid rotorcraft | `thunder` | tiltrotor |

Variants share equations and plant structure where justified, while mass,
payload, fuel or battery, drag, endurance, authority, and mission contracts
remain variant-specific. The model catalog must not imply that shared equations
mean shared real-world performance.

## Evidence contract

Every parameter and derived result carries one of four grades:

- **P — Published:** explicitly stated by the manufacturer, partner, or a government source.
- **D — Derived:** calculated from public geometry or performance using standard physics.
- **E — Engineering estimate:** constrained by vehicle class, imagery, and comparable systems.
- **S — Scenario placeholder:** selected to make a proxy operable and requiring full-band sensitivity testing.

Public facts, estimates, calibration choices, and unknowns remain separate in
the manifest. A proxy may be operationally useful at Grade C/D without making a
strong physical-realism claim.

## Shared fidelity ladder

### 3DOF proxy families

The common models are:

1. **Multirotor / rotorcraft:** Cartesian position and velocity plus energy or
   resource state. Commands are bounded accelerations or velocity/rate targets;
   thrust reserve, tilt, drag, climb/descent, and energy limits are explicit.
2. **Conventional fixed wing:** position, altitude, speed, flight-path angle,
   course, and fuel/resource state, with thrust, bank, and load-factor controls.
   The model supports turning, climb, loiter, energy management, and terminal
   corridors without claiming physical moment authority.
3. **Tailsitter / vectored thrust:** Cartesian motion plus a hover-to-cruise
   blend. Forces transition continuously between vectored thrust and aerodynamic
   lift/drag; modes are `vertical`, `transition`, `wingborne`, and `recovery`.
4. **Tiltrotor:** adds nacelle or rotor tilt as an explicit state or scheduled
   control, with rotor thrust, wing lift, induced-power, and transition limits.

These models are force-complete for their declared features. They do not claim
physical stability, actuator moment authority, or production flight envelopes.

### Named pseudo-6DOF profiles

Add attitude and body-rate response around the shared force model. Response
parameters include time constants or natural frequencies, damping, cross-axis
coupling, rate and acceleration limits, attitude limits, latency, saturation,
mode scheduling, and low-authority behavior. Attitude must not change
instantaneously when the pseudo-6DOF profile is active.

Recommended profile identities are:

```text
p6dof.multirotor_rate_response.v1
p6dof.fixed_wing_attitude_response.v1
p6dof.vectored_thrust_transition_response.v1
p6dof.tiltrotor_transition_response.v1
```

These profiles are not rigid-body 6DOF. If physical inertia and applied moments
determine angular acceleration, the realization must be classified as rigid-body
6DOF under the fidelity contract.

## Vehicle-specific modeling notes

- **Bolt / Bolt-M / Ghost-X / Anvil:** use the rotorcraft model. Vary payload,
  energy, thrust reserve, acceleration, and response; do not turn them into
  fixed-wing point masses merely to simplify route studies.
- **ALTIUS:** share a folding-wing fixed-wing plant while separating ISR and
  heavier 700M resource and payload configurations.
- **Barracuda:** keep the 100, 250, and 500 classes distinct because launch
  class, payload, range, and mass properties materially change the trajectory.
- **Roadrunner:** use explicit vertical, transition, wingborne, and recovery
  modes with vectored-thrust blending and high-subsonic proxy limits.
- **Fury / FQ-44:** begin as a generic subsonic high-performance fixed-wing
  surrogate. Do not claim production-vehicle 6DOF fidelity from public
  descriptions alone.
- **Omen:** share the transition framework with Roadrunner but use slower,
  lower-rate hybrid-electric hover-to-cruise behavior and a pronounced hover
  energy penalty.
- **Thunder:** begin as `thunder_concept_proxy_v0`, evidence-bounded and
  cataloged separately from mature families until public configuration and
  flight evidence improve.

## Coherent parameter generation

The generator samples a latent design and derives dependent values. It must not
independently sample mass, area, inertia, thrust, desired performance, drag,
endurance, and control response. It derives, checks, and records:

- geometry, reference area, and characteristic dimensions;
- empty, fuel/battery, payload, and gross mass;
- propulsion, power, resource flow, and depletion;
- speed, climb/descent, bank, load-factor, and range limits;
- aerodynamic polar or rotorcraft force surrogate;
- attitude-response and control-authority schedules;
- launch, transition, recovery, and terminal contracts.

Rejected samples are retained with reason codes. Validity requires positive
mass, feasible resource accounting, nonnegative drag or power, smooth force
behavior, coherent performance limits, and adequate authority for the claimed
mission.

## Minimum manifest

```yaml
identity:
  family:
  variant:
  proxy_version:
  evidence_grade:
equations:
  family:
  fidelity_profile:
geometry: {}
mass: {}
propulsion: {}
performance: {}
aerodynamics: {}
control: {}
envelope: {}
provenance:
  sources: []
  published_fields: []
  estimated_fields: []
  sensitivity_fields: []
```

Every generated run also records controller, initial-condition, environment,
failure, observation, and mission seeds independently.

## Qualification milestones

1. **Operable kinematic proxies:** straight-and-level, maximum-speed, turn,
   range/endurance, hover-hold, and continuous-transition checks pass.
2. **Family variants:** payload and resource changes alter performance
   coherently; ground, tube, air, and vertical starts resolve through public
   contracts.
3. **Pseudo-6DOF:** attitude lag, body rates, alpha/beta proxies, saturation,
   coordinated turns, and transition behavior are observable and reproducible.
4. **Corpus and reachability ready:** truth, observations, provenance, failure
   reasons, effective reachability envelopes, and leakage-safe splits are
   generated from the same resolved surrogate.

The first eight implementation targets are Bolt, Ghost-X, ALTIUS-600,
Roadrunner, Barracuda-250, Fury/FQ-44, Omen, and the explicitly low-confidence
Thunder concept proxy. Remaining variants are parameterized extensions, not
new equation-of-motion implementations.

## Energy-coupled and mode-aware implementation boundary

The v0.2 library expands the first-wave roster to include Bolt-M, Anvil,
ALTIUS-700 ISR, ALTIUS-700M, Barracuda-100, and Barracuda-500 while retaining
one shared family implementation per dynamics class. The promoted model is
not a point mass with a timer attached. It resolves:

```text
vehicle configuration + loading state + energy backend + mode policy
                              ↓
                 resource/mass-property scheduler
                              ↓
                 3DOF reduction and pseudo-6DOF realization
```

### Resource backends

Use `battery_electric`, `fuel_burning`, `series_hybrid`, or an explicit
`selectable` pair. Battery energy changes available power and reserve state but
does not reduce mass. Fuel flow changes mass and schedules CG and inertia.
Thunder's series-hybrid backend separately schedules generator power, battery
buffer power, fuel flow, and landing/go-around reserve. Omen and ALTIUS retain
both battery and fuel alternatives because public architecture descriptions do
not establish one production arrangement.

### Mode records

Every launch, deployment, transition, task, abort, recovery, and terminal mode
has entry conditions, a blend state, exit conditions, abort conditions, and a
resource policy. Roadrunner, Omen, and Thunder require continuous force,
moment, thrust-axis, and attitude blending through transition. Mode changes are
events in the result artifact, not hidden branches in a vehicle-specific
runner.

### Data and promotion rule

Each promoted numeric parameter has a value, unit, `P/D/E/S` grade, source
reference, uncertainty policy, and validity envelope in the parameter ledger.
The compact `geometry`, `mass`, `propulsion`, `performance`, and `control`
fields remain compatibility views for the current smoke adapter. They do not
replace the ledger or permit untagged values into a mission claim.

The validation ladder is:

1. data and provenance firewall;
2. resource balance, reserves, and payload monotonicity;
3. hover/cruise equilibrium and performance anchors;
4. transition continuity and mode-event correctness;
5. allocator, actuator lag, rate, and saturation checks;
6. paired 3DOF/pseudo-6DOF comparison from the same parent instance;
7. E-grade ±25% and S-grade full-band sensitivity/reachability studies.

The resulting status is one of `cataloged`, `data_ready`, `3dof_ready`,
`pseudo_6dof_ready`, `mission_ready`, or `blocked`. A configuration may be
useful for data or architecture work while remaining blocked from mission
promotion.

## Claim boundary

The qualification statement must identify the exact proxy identity, fidelity
profile, evidence grade, controller, mission, environment, and envelope. A
successful proxy trajectory establishes that the declared Taoryx model is
internally executable and useful for research. It does not establish exact
manufacturer performance, production configuration, classified capability, or
real-vehicle validation.
