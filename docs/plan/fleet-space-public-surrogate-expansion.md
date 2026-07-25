# Taoryx Expansion Plan: Fleet Quadrotors, Spacecraft, and Public-Data Autonomous-Aircraft Surrogates

**Status:** Planning draft  
**Scope:** Extension to the Taoryx vehicle-family, flagship-scenario, provider, and AI/RL-testbed plans  
**Primary release objective:** Add scale, a second physical domain, and a disciplined public-data surrogate program without creating separate runtimes.

---

## 1. Executive decision

Taoryx should add three coordinated workstreams:

1. **A fast Hummingbird reduction for fleet simulation**
   - A vectorized 3DOF model for very large populations.
   - A vectorized attitude-response pseudo-6DOF model for fleets that need believable orientation and control behavior.
   - The existing rotor-resolved Hummingbird rigid-body 6DOF model remains the truth/reference implementation.

2. **A satellite reference family**
   - Orbital 3DOF propagation.
   - Attitude-response pseudo-6DOF.
   - Rigid-body spacecraft 6DOF with attitude actuators.
   - Standard orbit-data import/export and one complete deployment-to-operations flagship scenario.

The spacecraft branch has its own [6DOF data and qualification track](spacecraft-6dof-data-and-examples.md).
Reaction-wheel-primary and thruster-primary examples are foundational control
archetypes; a hybrid wheel/thruster spacecraft follows them. Spacecraft 6DOF
qualification is not inferred from orbital 3DOF or a generic body-torque input.

The [parametric spacecraft family expansion](spacecraft-parametric-family-expansion.md)
adds four source-anchored notional families: a 6U reaction-wheel observer, an
agile reaction-wheel imager, a SPHERES-like thruster-only free flyer, and a
MarCO-like hybrid wheel/cold-gas spacecraft. CPOD-like proximity, BioSentinel-
like degraded-thruster, and NEA Scout-like low-thrust variants are follow-on
anchors.

3. **A public-data autonomous-air-vehicle surrogate catalog**
   - Generic Taoryx archetypes informed by publicly released descriptions of Anduril aircraft.
   - No claims of proprietary accuracy.
   - Explicit uncertainty, provenance, and evidence levels.
   - Initial surrogates for expeditionary rotorcraft, tube-launched fixed-wing aircraft, reusable jet VTOL aircraft, high-performance autonomous jets, and modular cruise vehicles.

These are not three separate simulation systems. They share:

- The same family, fidelity, parameter, control, observation, segment, objective, and result contracts.
- The same canonical stepping semantics.
- The same evaluator and qualification infrastructure.
- The same provider and Taoryx Lab facades.
- A new batched/population execution interface for fleets and constellations.

The [Parametric Models and ML Trajectory Corpus Plan](parametric-models-ml-trajectory-corpus.md)
defines the corpus and sampling layer for these families. Fleet, satellite,
and public-surrogate work must use coherent realized vehicles, separate
variation seeds, explicit truth/observation layers, and leakage-safe splits.

---

# Part I — Common architecture additions

## 2. Add a batched population runtime

Single-vehicle stepping is not enough for fleets. Taoryx needs a first-class population session:

```text
ResolvedPopulationCase
        ↓
PopulationSession
        ↓
reset(seed_set)
        ↓
step(actions[N, A])
        ↓
observations[N, O]
events[sparse]
status[N]
```

The population runtime must support:

- Homogeneous cohorts using one compiled family/fidelity profile.
- Multiple cohorts when a scenario contains several vehicle types.
- Structure-of-arrays state storage.
- Fixed-size control and observation schemas per cohort.
- Batched environment sampling.
- Sparse event queues.
- Per-agent active, terminal, failed, and removed masks.
- Deterministic partitioning and replay.
- Checkpoint and branch support.
- Aggregate telemetry plus selectable full-fidelity traces for sampled agents.
- Spatial indexing for neighbor, collision, and communication queries.
- No all-pairs \(O(N^2)\) interaction path in normal fleet operation.

The same single-vehicle transition logic should be usable through a batch kernel. The batch implementation may be separately optimized, but it must pass equivalence tests against independent scalar stepping for sampled agents.

### 2.1 Performance reporting

Fleet performance should be reported with:

```text
agent steps per wall-clock second
simulated agent-seconds per wall-clock second
bytes of dynamic state per agent
bytes of telemetry per agent-second
event-processing cost
neighbor-query cost
dynamics cost
controller cost
```

Every performance claim must identify:

- Hardware.
- Software version.
- Compiler/runtime settings.
- Fleet size.
- Update rate.
- Observation and logging profile.
- Interaction density.
- Environment complexity.

### 2.2 Initial performance gates

These are release targets, not current claims:

| Tier | Scenario | Acceptance target |
|---|---|---|
| Fleet-S | 256 pseudo-6DOF quadrotors at 50 Hz | At least real time on the documented reference workstation |
| Fleet-M | 1,024 pseudo-6DOF quadrotors at 20 Hz | At least real time |
| Fleet-L | 10,000 3DOF quadrotors at 10 Hz | Stretch goal; faster than real time with reduced telemetry |
| Constellation-S | 10,000 orbital 3DOF satellites | Bounded multi-orbit propagation with sparse events and reproducible output |

Performance qualification is separate from physical qualification.

---

## 3. Add a domain profile to the common contracts

Atmospheric aircraft and satellites share many abstractions, but they do not share every assumption.

Every case should declare a domain profile:

```yaml
domain:
  id: atmospheric.flight
```

or:

```yaml
domain:
  id: space.orbit
```

The shared contract owns:

- Time.
- State.
- Controls.
- Observations.
- Events.
- Segments.
- Objectives.
- Finality.
- Provenance.
- Results.

Domain profiles add required semantics.

### 3.1 Atmospheric-flight profile

Includes:

- Geodetic and local navigation frames.
- Atmosphere and wind.
- Terrain and ground.
- Aerodynamic state.
- Contact and launch/recovery modes.
- Atmospheric envelope metrics.

### 3.2 Space-orbit profile

Includes:

- Epoch and time scale.
- Central body.
- Inertial and body-fixed frames.
- Ephemerides.
- Gravity model.
- Eclipse and line-of-sight events.
- Orbital elements and ground track.
- Spacecraft attitude and pointing.
- Orbit-data exchange formats.

Do not force satellite state into an aircraft-specific NED-only API.

---

## 4. Add a formal surrogate evidence contract

A public-data surrogate is not the real vehicle. The package must say exactly what is known, inferred, assumed, and tunable.

Each surrogate includes:

```yaml
surrogate:
  archetype: reusable_twin_jet_vtol
  public_inspiration:
    organization: Anduril
    product: Roadrunner

  evidence_level: E1_public_descriptor

  source_claims:
    propulsion_class:
      value: twin_air_breathing_jet
      confidence: high
      provenance: official_public_description

    launch_recovery:
      value: vertical_takeoff_and_recovery
      confidence: high
      provenance: official_public_description

  assumed_parameters:
    mass:
      distribution: bounded_uniform
      min: ...
      max: ...
      confidence: low
      basis: generic_archetype

  not_claimed:
    - exact_mass
    - exact_geometry
    - exact_aerodynamic_database
    - exact_control_laws
    - exact_performance
    - signature
    - proprietary_payloads
```

### 4.1 Evidence levels

| Level | Meaning |
|---|---|
| **E0 — Conceptual archetype** | Generic class only; no named-product claim |
| **E1 — Public descriptor** | Reproduces publicly stated architecture and mission lifecycle |
| **E2 — Public envelope** | Also matches publicly stated quantitative ranges within declared tolerances |
| **E3 — Public behavioral correlation** | Correlated against public flight traces, demonstrations, or sufficiently detailed test data |
| **E4 — Authorized data correlation** | Correlated against licensed, partner, or owner-provided data |

Most initial Anduril-inspired surrogates should be E1. Do not force guessed values into E2.

### 4.2 Safe and credible scope

Base surrogates should focus on:

- Vehicle dynamics.
- Launch and recovery.
- Guidance and waypoint behavior.
- Formation and fleet interaction.
- Energy and resource use.
- Control abstraction.
- Payload mass, power, and drag effects.

The base models should not claim proprietary weapon, seeker, signature, targeting, or engagement behavior. Mission payloads should default to inert generic modules.

---

# Part II — Fast quadrotor fleet family

## 5. Keep one Hummingbird family with three fidelity realizations

Do not create an unrelated “swarm drone” model. Make the fleet models reductions of the same Hummingbird family:

```text
hummingbird.rigid_body_6dof
hummingbird.attitude_response_p6dof
hummingbird.fleet_3dof
```

That gives Taoryx a calibration and transfer ladder:

```text
full rotor and rigid-body truth
           ↓
attitude-response fleet model
           ↓
point-mass fleet model
```

## 6. Hummingbird fleet 3DOF

### 6.1 Purpose

Use for:

- Formation movement.
- Route planning.
- Collision avoidance.
- Communications topology.
- Resource allocation.
- Large-scale RL.
- Fleet command and control.
- Scenario generation.

Do not use it to claim:

- Rotor-level control allocation.
- Attitude stability.
- Body-rate response.
- Motor failure response.
- Aerodynamic moment behavior.

### 6.2 State

Minimum state:

```text
position
velocity
heading
remaining_energy
operating_status
```

Optional state:

```text
command lag
vertical and horizontal acceleration state
payload state
communication state
health scalar
```

### 6.3 Controls

Preferred high-level controls:

```text
velocity_north
velocity_east
vertical_speed
yaw_rate
```

Alternative:

```text
acceleration_north
acceleration_east
vertical_acceleration
yaw_rate
```

The control schema declares:

- Speed and acceleration limits.
- Climb and descent limits.
- Turn-rate limits.
- Command latency.
- First-order response.
- Energy cost.
- Failsafe behavior.

### 6.4 Required data

```text
total mass
maximum useful thrust-to-weight surrogate
horizontal drag or speed-power model
vertical drag or climb-power model
maximum speeds
maximum accelerations
maximum yaw rate
command response time constants
battery usable energy
idle, hover, climb, cruise, and maneuver power
payload mass/power increments
wind sensitivity
qualified operating envelope
```

No physical inertia or individual rotor maps are required.

---

## 7. Hummingbird attitude-response pseudo-6DOF

### 7.1 Purpose

This should be the default fleet model when orientation matters.

It supports:

- Camera or sensor pointing.
- Roll and pitch during turns.
- Yaw control.
- More credible wind response.
- Saturation and command lag.
- Easier policy transfer to full 6DOF.

### 7.2 State

```text
position
velocity
quaternion
body_rate
aggregate_thrust_state
remaining_energy
operating_status
```

### 7.3 Attitude response

Use a named response profile, for example:

```text
p6dof.multirotor_rate_response.v1
```

The model may use scheduled first- or second-order response:

```text
commanded roll/pitch/yaw rate
        ↓
rate limits and lag
        ↓
attitude kinematics
        ↓
aggregate thrust orientation
        ↓
translational force
```

It does not integrate rotor moments through a physical inertia tensor.

### 7.4 Required data

All 3DOF fleet data, plus:

```text
roll, pitch, and yaw response constants
maximum attitude
maximum body rates
maximum angular accelerations
cross-axis coupling approximation
aggregate thrust lag
control saturation behavior
low-battery authority reduction
wind-disturbance response
sensor/observation profile
```

Optional rotor-like controls may be exposed, but they must be explicitly labeled as surrogate effectors.

---

## 8. Full Hummingbird 6DOF remains the reference

The existing full model should contain:

```text
mass and full inertia
rotor locations and axes
rotor thrust and torque maps
motor/rotor dynamics
control allocation
body drag
ground contact
battery/resource behavior
sensor models
failure modes
```

The reduced models are calibrated against standard full-model maneuvers:

- Hover.
- Vertical step.
- Forward acceleration.
- Lateral acceleration.
- Yaw step.
- Coordinated turn.
- Gust recovery.
- Payload-mass change.
- Battery-state change.

The goal is bounded behavior agreement, not state-for-state identity.

---

## 9. Fleet flagship scenario

**Scenario ID**

```text
hummingbird_fleet_split_merge_return_p6dof
```

### 9.1 Mission

```text
distributed pad initialization
→ synchronized takeoff
→ formation acquisition
→ corridor transit
→ split into task groups
→ multi-altitude waypoint service
→ moving waypoint tracking
→ deconfliction event
→ communications degradation
→ group rejoin
→ return-to-home
→ distributed landing or terminal landing gates
```

### 9.2 Initial scale

Use 256 agents for the qualification scenario. Add 1,024-agent performance and robustness variants.

### 9.3 Required evidence

```text
single-agent equivalence against scalar stepping
sample-agent correlation to full Hummingbird 6DOF
formation error
waypoint completion
collision and near-miss counts
minimum separation distribution
communication graph health
energy distribution
agent losses and failure reasons
control saturation distribution
deterministic replay hash
wall-clock performance
memory use
```

### 9.4 Pickup-ready gate

A user can configure:

- Fleet size.
- Spawn geometry.
- Payload classes.
- Battery preset.
- Wind.
- Waypoints.
- Formation.
- Controller or policy.
- Failure/randomization rates.

They can run the fleet without editing model code and receive both aggregate and sampled-agent outputs.

---

# Part III — Satellite reference family

## 10. Satellite fidelity ladder

Use one reference family, for example:

```text
taoryx.reference_satellite
```

with:

```text
orbit_3dof
attitude_response_p6dof
rigid_body_spacecraft_6dof
```

## 11. Orbital 3DOF

### 11.1 Equations

Integrate center-of-mass translation in an inertial frame:

```text
position
velocity
mass/resource state
```

Possible force-model levels:

```text
two_body
j2
spherical_harmonic_gravity
atmospheric_drag
solar_radiation_pressure
third_body
finite_burn
impulsive_maneuver
```

The active model only requires data for selected effects.

### 11.2 Required data

Minimum:

```text
epoch
time scale
central body
initial state and frame
mass
propagator and integration tolerances
final time or terminal event
```

For drag:

```text
reference area
drag coefficient or macro model
atmosphere model
attitude-area assumption
```

For solar radiation pressure:

```text
illuminated area
reflectivity coefficient or macro model
eclipse model
attitude-area assumption
```

For propulsion:

```text
thrust
direction
mass flow or specific impulse
burn timing or command interface
```

### 11.3 Interchange

Support, at minimum:

- Cartesian state and classical orbital elements.
- TLE/OMM import through SGP4 as a distinct propagation source.
- CCSDS OPM, OMM, and OEM exchange.
- Later OCM support for richer maneuver and covariance content.

SGP4/TLE propagation must not be silently mixed with numerical force-model propagation. The result manifest records which model produced the state.

---

## 12. Satellite attitude-response pseudo-6DOF

### 12.1 Purpose

Use for:

- Constellation pointing studies.
- Observation scheduling.
- Communications pointing.
- Power and eclipse studies.
- Large-scale space RL.
- Simple deployment and acquisition timelines.

### 12.2 State

```text
orbital position and velocity
quaternion
body rate
pointing mode
resource state
```

### 12.3 Required data

```text
pointing-mode definitions
slew rate and acceleration limits
attitude response constants
settling criteria
keep-out and pointing constraints
surface area by pointing mode
power generation by sun angle
battery capacity and loads
sensor field of view
communications field of view
```

Physical inertia and torque balance are not required.

---

## 13. Rigid-body spacecraft 6DOF

### 13.1 Required mass properties

```text
mass
center of gravity
full inertia tensor
configuration-dependent mass properties
deployed appendage states
propellant state
```

### 13.2 Disturbance and environmental torques

As applicable:

```text
gravity-gradient torque
aerodynamic torque
solar-radiation-pressure torque
magnetic torque
thruster torque
internal momentum-exchange torque
```

### 13.3 Attitude-control effectors

As applicable:

```text
reaction wheels
control-moment gyros
magnetorquers
reaction-control thrusters
main-engine gimbals
```

Each effector needs:

- Axis and location.
- Torque/thrust capacity.
- Rate or momentum limits.
- Dynamics and latency.
- Resource use.
- Saturation and desaturation behavior.
- Failure states.

### 13.4 Sensors

For closed-loop qualification:

```text
rate gyro
star tracker
sun sensor
magnetometer
horizon or nadir sensor
navigation state source
```

Truth-only operation may omit sensor errors, but the claim must say so.

---

## 14. Satellite flagship scenario

**Scenario ID**

```text
reference_satellite_deploy_detumble_nadir_pass_6dof
```

### 14.1 Mission

```text
parent deployment handoff
→ separation clearance
→ free drift
→ detumble
→ sun-safe acquisition
→ battery-positive stabilization
→ nadir-pointing acquisition
→ optional orbit trim
→ target visibility window
→ observation dwell
→ ground-station visibility
→ downlink-pointing window
→ return to nominal safe mode
```

### 14.2 Finality

Success requires:

- Valid orbit state.
- Stable attitude.
- Energy reserve above threshold.
- Completed observation dwell.
- Completed or attempted downlink window.
- No keep-out violation.
- No actuator momentum or resource violation.
- All operations inside the declared model envelope.

### 14.3 Validation chain

```text
two-body analytical comparison
energy and angular-momentum conservation
J2 secular-rate comparison
bounded SGP4 comparison over declared intervals
frame-transform tests
eclipse event timing
torque-free attitude motion
reaction-wheel momentum exchange
detumble response
batch-versus-step equivalence
checkpoint replay
```

### 14.4 Later constellation scenario

```text
reference_constellation_visibility_and_retask_3dof
```

This reuses the population runtime for thousands of orbiting agents and sparse visibility events.

---

# Part IV — Public-data Anduril-inspired surrogate program

The [flight-dynamics surrogate library](flight-dynamics-surrogate-library.md)
defines the next implementation boundary for this program. It turns the public
surrogate roster into shared force-model families, named pseudo-6DOF profiles,
coherent parameter generation, and explicit P/D/E/S evidence grades. The
surrogates remain engineering proxies rather than replicas or claims of
manufacturer-grade performance.

The stricter numerical review is maintained in
[anduril-surrogate-engineering-review.md](anduril-surrogate-engineering-review.md)
and supersedes loose nominal values when they conflict. It adds loading-state
coupling, hover-power and stall checks, compressibility handling, and explicit
concept-level treatment for Thunder.

## 15. Program principle

Use public product descriptions to define **generic archetypes**, not replicas.

Recommended naming:

```text
surrogate.expeditionary_tandem_rotor_vtol
surrogate.tube_launched_fixed_wing_aav
surrogate.reusable_twin_jet_vtol
surrogate.high_performance_autonomous_jet
surrogate.modular_air_breathing_cruise_aav
surrogate.group5_hybrid_vtol_rotorcraft
```

Each package may include metadata such as:

```yaml
public_inspiration:
  organization: Anduril
  products:
    - Ghost-X
```

The stable simulation contract uses the generic archetype name.

---

## 16. Publicly visible classes and proposed Taoryx mappings

### 16.1 Ghost / Ghost-X class

**Publicly described architecture**

- Expeditionary autonomous VTOL aircraft.
- Quiet, modular reconnaissance-oriented platform.
- Current public descriptions identify a tandem-rotor configuration.

**Taoryx surrogate**

```text
surrogate.expeditionary_tandem_rotor_vtol
```

**Initial fidelity**

```text
attitude_response_p6dof
```

**Later fidelity**

```text
rigid_body_6dof with aggregate fore/aft rotor systems
```

**Flagship mission**

```text
pad takeoff
→ hover check
→ low-speed waypoint route
→ stationary observation dwells
→ communications-relay orbit
→ wind recovery
→ return and land
```

**Why first**

It reuses most Hummingbird fleet, VTOL, waypoint, and landing infrastructure while adding tandem-rotor control behavior.

---

### 16.2 ALTIUS class

**Publicly described architecture**

- Tube- or canister-compatible autonomous fixed-wing family.
- Publicly described for launch from air, ground, or maritime platforms.
- Multiple size/payload/endurance variants.

**Taoryx surrogate**

```text
surrogate.tube_launched_fixed_wing_aav
```

**Initial fidelity**

```text
point_mass_3dof
```

**Later fidelity**

```text
attitude_response_p6dof
```

**Flagship mission**

```text
carrier or tube-launch handoff
→ wing-deployment surrogate
→ launch stabilization
→ climb
→ waypoint route
→ loiter pattern
→ retask
→ terminal mission-complete corridor
```

**Why early**

It reuses the X8 point-mass, fixed-wing, air-release, and waypoint stack.

---

### 16.3 Roadrunner class

**Publicly described architecture**

- Reusable autonomous VTOL aircraft.
- Twin air-breathing jet propulsion.
- High-subsonic performance class.
- Modular payload concept.

**Taoryx surrogate**

```text
surrogate.reusable_twin_jet_vtol
```

**Initial fidelity**

```text
attitude_response_p6dof
```

**Later fidelity**

```text
rigid_body_6dof
```

**Flagship mission**

```text
vertical launch
→ transition to forward flight
→ high-speed route
→ moving waypoint capture
→ abort/retask branch
→ return
→ vertical recovery
```

**Key new kernels**

- Jet VTOL.
- Vertical-to-forward transition.
- Large thrust-vectoring authority.
- High-speed recovery transition.

---

### 16.4 Fury / YFQ-44A class

**Publicly described architecture**

- High-performance autonomous jet aircraft.
- Near-fighter-speed public positioning.
- Modular mission-payload concept.
- Collaborative-combat-aircraft program context.

**Taoryx surrogate**

```text
surrogate.high_performance_autonomous_jet
```

**Initial fidelity**

```text
point_mass_3dof high-performance guidance model
```

**Next fidelity**

```text
attitude_response_p6dof
```

**Flagship mission**

```text
runway or airborne initialization
→ climb
→ high-speed route
→ formation join
→ split and rejoin
→ altitude and energy changes
→ return-to-base arrival gate
```

**Why not first**

Credible transonic/high-performance 6DOF aerodynamics and control-effectiveness data are substantially harder than the public descriptions support. Begin with an honest E1 guidance-and-energy surrogate.

---

### 16.5 Barracuda class

**Publicly described architecture**

- Family of air-breathing autonomous air vehicles.
- Publicly organized into increasing-size variants.
- Designed around modularity and scalable production.

**Taoryx surrogate**

```text
surrogate.modular_air_breathing_cruise_aav
```

**Initial fidelity**

```text
point_mass_3dof
```

**Later fidelity**

```text
attitude_response_p6dof
```

**Flagship mission**

```text
air or ground launch
→ stabilization
→ climb or terrain-clearance phase
→ long-route cruise
→ waypoint replanning
→ energy/resource management
→ terminal region or recovery corridor
```

Use inert payload modules in the base package.

---

### 16.6 Thunder class

**Publicly described architecture**

- Newly announced Group 5 autonomous rotorcraft.
- Hybrid-electric/VTOL aircraft class in public reporting.
- Public flight evidence is not yet mature.

**Taoryx status**

```text
E0 conceptual archetype
M0 cataloged
```

**Proposed surrogate**

```text
surrogate.group5_hybrid_vtol_rotorcraft
```

Do not build a named public-envelope model until more public flight and configuration data exists. In the meantime, the generic architecture can be researched as a tiltrotor or compound-rotorcraft family without claiming product correlation.

---

## 17. Recommended first surrogate release

Do not attempt every product at once.

### Surrogate Release S1

Build these three:

1. **Expeditionary tandem-rotor VTOL**
   - Reuses Hummingbird and VTOL foundations.
   - Demonstrates takeoff, hover, route, dwell, and landing.

2. **Tube-launched fixed-wing AAV**
   - Reuses X8, launch, and point-mass guidance.
   - Demonstrates parent/tube handoff, deployment, route, loiter, and retask.

3. **Reusable twin-jet VTOL**
   - Adds the most architecturally valuable new behavior.
   - Demonstrates vertical launch, transition, fast flight, abort, return, and vertical recovery.

### Surrogate Release S2

Add:

4. **High-performance autonomous jet**
5. **Modular air-breathing cruise AAV**

### Research-only queue

6. **Group 5 hybrid VTOL rotorcraft**
7. **Compact modular VTOL scout**
8. **Small reusable interceptor archetype**

---

## 18. Surrogate data workflow

For each archetype:

### Step 1 — Public source register

Record:

```text
official product pages
official announcements
government program descriptions
public flight-test announcements
credible photographs and videos
public dimensions or performance statements
publication date
source hash or archive identifier
```

### Step 2 — Claim extraction

Classify every statement as:

```text
direct public fact
derived geometric estimate
generic archetype assumption
calibration choice
scenario convenience
unknown
```

### Step 3 — Parameter prior

Unknown values become ranges or distributions, not fabricated facts:

```yaml
mass:
  min: ...
  max: ...
  distribution: triangular
  mode: ...
  evidence: generic_archetype
```

### Step 4 — Realization

Create the lowest fidelity that can demonstrate the public lifecycle honestly.

### Step 5 — Sensitivity

Run the flagship scenario over the uncertain parameter ranges.

### Step 6 — Freeze a benchmark preset

The benchmark preset is a Taoryx test article, not a claim about the original product.

### Step 7 — Publish limitations

Every plot and report identifies:

- Surrogate evidence level.
- Source date.
- Assumed parameter fraction.
- Qualified envelope.
- Behaviors not represented.
- Whether the run is 3DOF, pseudo-6DOF, or 6DOF.

---

# Part V — Qualification and maturity

## 19. Additional badges

Use the existing maturity ladder, plus these badges:

```text
Fleet-Ready
Constellation-Ready
Space-Domain Qualified
Public-Descriptor Surrogate
Public-Envelope Surrogate
Cross-Fidelity Correlated
```

### 19.1 Fleet-Ready

Requires:

- Batched deterministic stepping.
- Scalar/batch equivalence.
- Performance report.
- Sparse event behavior.
- Collision/neighbor validation.
- Aggregate and sampled telemetry.
- Population checkpoint/replay.
- Stable memory use.
- Configurable population initialization.

### 19.2 Space-Domain Qualified

Requires:

- Declared frames and time scale.
- Validated orbit propagation.
- Standard orbit-data exchange.
- Event timing for eclipse/visibility.
- Domain-specific finality.
- Force-model provenance.
- Step/batch/replay equivalence.

### 19.3 Public-Descriptor Surrogate

Requires:

- Public source register.
- Claim extraction.
- Unknown/assumed parameter report.
- Generic archetype name.
- One qualified scenario.
- Explicit non-claims.
- No hidden guessed “exact” data.

---

## 20. New showcase scenarios

The expanded showcase becomes:

| ID | Vehicle/family | Main proof |
|---|---|---|
| `hummingbird_pad_box_yaw_land_6dof` | Full Hummingbird | Rotor-resolved pad-to-pad flight |
| `hummingbird_fleet_split_merge_return_p6dof` | Hummingbird fleet | Population scale, formation, split/rejoin, return |
| `x8_launch_figure8_recovery_6dof` | X8 | Flying-wing launch, elevon control, route, recovery |
| `b747_trim_route_arrival_6dof` | B747 | Transport-class controlled route and arrival |
| `x15_airlaunch_energy_target_6dof` | X-15 | Air launch, rocket burn, coast, high-energy glide |
| `reference_satellite_deploy_detumble_nadir_pass_6dof` | Satellite | Orbit, detumble, pointing, observation, downlink |
| `surrogate_tandem_rotor_route_land_p6dof` | Public-data VTOL surrogate | Expeditionary VTOL lifecycle |
| `surrogate_tube_launch_loiter_3dof` | Public-data fixed-wing surrogate | Launch, deploy, route, loiter, retask |
| `surrogate_twin_jet_vtol_dash_return_p6dof` | Public-data jet-VTOL surrogate | Vertical launch, transition, fast route, return |

This gives Taoryx:

```text
single vehicle + fleets
atmospheric flight + space
3DOF + pseudo-6DOF + 6DOF
fixed wing + multirotor + tandem rotor + jet VTOL + rocket aircraft
takeoff + air release + tube launch + parent deployment
waypoints + objectives + formations + observation windows
landing + arrival gates + orbit finality
```

---

# Part VI — Delivery order

## 21. Milestone A — Shared population and surrogate infrastructure

Deliver:

- `ResolvedPopulationCase`.
- Batched `PopulationSession`.
- SoA state layout.
- Cohort support.
- Deterministic seed sets.
- Aggregate telemetry.
- Surrogate evidence schema.
- Source and assumption reports.
- New badges.

## 22. Milestone B — Hummingbird fleet reductions

Deliver:

- Hummingbird 3DOF.
- Hummingbird attitude-response pseudo-6DOF.
- Cross-fidelity calibration suite.
- 256-agent flagship.
- 1,024-agent performance case.
- Fleet RL environment.

## 23. Milestone C — Satellite reference family

Deliver:

- Orbit 3DOF.
- TLE/OMM import with explicit SGP4 mode.
- CCSDS orbit-message exchange.
- Pseudo-6DOF pointing.
- Full 6DOF attitude.
- Deployment-to-observation flagship.
- Initial constellation batch example.

## 24. Milestone D — Public surrogate research catalog

Deliver:

- Source register.
- Claim extractor workflow.
- Evidence levels.
- Generic archetype catalog.
- Uncertainty schema.
- Initial parameter priors.
- Research notebooks/reports.
- No runnable claim yet.

## 25. Milestone E — Surrogate Release S1

Deliver:

- Tandem-rotor VTOL p6DOF.
- Tube-launched fixed-wing 3DOF.
- Reusable twin-jet VTOL p6DOF.
- One flagship scenario each.
- Sensitivity packs.
- Public-descriptor qualification.

## 26. Milestone F — Higher-performance surrogates

Deliver:

- High-performance autonomous jet 3DOF/p6DOF.
- Modular cruise AAV 3DOF/p6DOF.
- New data only when evidence supports stronger claims.
- Keep newly announced architectures catalog-only until public evidence matures.

---

# Part VII — Definitions of done

## 27. Fleet quadrotor definition of done

> The Hummingbird fleet realization is done when the same family can be selected as rotor-resolved 6DOF, attitude-response pseudo-6DOF, or fleet 3DOF; the reduced models are calibrated and bounded against the full model; a user can configure hundreds or thousands of agents, formations, routes, payloads, wind, failures, observations, and controllers without code changes; scalar and batched stepping agree; deterministic replay succeeds; fleet-level safety and mission metrics are generated; and the documented Fleet-S and Fleet-M performance gates pass on a named reference machine.

## 28. Satellite family definition of done

> The satellite family is done when a user can initialize from a state vector, orbital elements, standardized orbit data, or parent deployment; select orbital 3DOF, attitude-response pseudo-6DOF, or rigid-body spacecraft 6DOF; configure the applicable gravity, drag, radiation, propulsion, attitude-control, power, sensor, and event models; compose deployment, detumble, safe, pointing, maneuver, observation, and communications segments; run batch or stepwise; and reproduce a qualified deployment-to-observation mission with explicit frame, epoch, force-model, actuator, resource, event, and finality provenance.

## 29. Public-data surrogate definition of done

> A public-data surrogate is done when it has a generic archetype identity, a versioned public source register, a machine-readable separation between public facts and assumptions, uncertainty ranges for unsupported parameters, a declared evidence level, fidelity-specific data contracts, one pickup-ready flagship mission, sensitivity results over the uncertain parameters, explicit non-claims, and outputs that never label the result as the original manufacturer’s validated vehicle model.

## 30. Expanded showcase definition of done

> The expanded Taoryx showcase is done when it demonstrates full-fidelity single-vehicle flight, fleet-scale reduced-order flight, orbital spacecraft operation, and public-data surrogate research through one common runtime and qualification system; every example is configurable, composable, deterministic, stepwise, evaluable, and reproducible; and every physical or public-correlation claim is bounded by a declared fidelity, envelope, evidence level, and source/assumption report.

---

# 31. Immediate next actions

1. Freeze the Hummingbird full-model truth maneuvers used to calibrate reductions.
2. Define `PopulationSession` before optimizing any one fleet model.
3. Implement the pseudo-6DOF quad before the pure 3DOF fleet model so the common batched state layout is not too narrow.
4. Define the space-domain frame/time contract before writing the satellite propagator.
5. Add the surrogate evidence schema before collecting product data.
6. Build the public source register for Ghost/Ghost-X, ALTIUS, Roadrunner, Fury, Barracuda, and Thunder.
7. Select the first three surrogate archetypes: tandem-rotor VTOL, tube-launched fixed wing, and reusable twin-jet VTOL.
8. Add a combined release command:

```bash
taoryx showcase build breadth-v2
taoryx showcase verify breadth-v2
taoryx showcase report breadth-v2
```

The resulting claim should be:

> **Taoryx supports high-fidelity individual vehicles, reduced-order fleets, orbital spacecraft, and evidence-bounded public-data vehicle surrogates through one configurable trajectory and AI/RL testbed.**
