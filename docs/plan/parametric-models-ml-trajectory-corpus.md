# Taoryx Parametric Models and ML Trajectory Corpus Plan

**Status:** Planning draft  
**Scope:** Parametric 3DOF, named pseudo-6DOF, and rigid-body 6DOF vehicle generation; paired multi-fidelity trajectories; aerodynamic-property identification corpora; out-of-distribution evaluation  
**Depends on:** Vehicle families, fidelity contracts, composition, deterministic stepping, qualification, flagship scenarios, population execution, and source/evidence metadata

---

## 1. Executive decision

Taoryx should add a first-class **Parametric Vehicle and Corpus Program** with three complementary model sources:

1. **Source-grounded reference families**
   - Examples: F-16 S-119 and HL-20 DAVE-ML plants.
   - Their accepted source models remain immutable.
   - Permitted variation is limited to explicitly qualified operating parameters, loadouts, uncertainty envelopes, and separately named derivative models.

2. **Coherent parametric archetype families**
   - Examples: conventional fixed wing, flying wing, lifting body, ballistic body, rocket, multirotor, tandem rotor, and jet VTOL.
   - A compact latent design vector generates mutually consistent geometry, mass properties, propulsion, aerodynamics, controls, and valid operating bounds.
   - These families provide the largest breadth for machine-learning corpora.

3. **Derived multi-fidelity reductions**
   - A rigid-body 6DOF parent can generate calibrated pseudo-6DOF and 3DOF children.
   - The children share a semantic vehicle identity and preserve declared quantities, but their omissions and expected disagreements are explicit.
   - They enable paired trajectories for cross-fidelity learning and validation.

The key rule is:

> **Do not create diversity by independently randomizing every table cell and every physical quantity. Create diversity by sampling a coherent vehicle design, deriving coupled data, and then applying bounded uncertainty or model-form variation.**

A second key distinction is:

> **3DOF and pseudo-6DOF should expose broader notional design abstractions. Rigid-body 6DOF should expose more physical fields, but those fields must obey tighter coupling and validity constraints.**

The program should produce both compelling mission trajectories and deliberately informative identification trajectories. A large trajectory count alone does not make a useful aerodynamic-estimation corpus.

---

## 2. Product promise

A user should be able to select an archetype or qualified family, choose a fidelity, set or sample a parameter envelope, define initial conditions and a mission or identification experiment, and generate deterministic trajectories with complete truth metadata.

```text
model source
+ archetype or family
+ fidelity profile
+ design parameters
+ operating configuration
+ initial conditions
+ environment
+ controller and excitation
+ observation profile
+ seed
= resolved parametric experiment
```

The resulting run should contain:

```text
coherent realized vehicle instance
complete parameter provenance
realized force and moment models
initial and boundary conditions
commanded and achieved controls
truth states, forces, moments, and coefficients
sensorized observations when requested
terminal and validity classifications
corpus split assignment
reproduction information
```

Sensorized corpus editions use the [sensor and measurement orchestration
backlog](sensor-measurement-orchestration.md): truth is committed before
sampling, measurement packets carry sample and availability times, sensor
random state is reproducible, and trainable consumers cannot silently read
truth ports.

For paired multi-fidelity experiments:

```text
one semantic vehicle instance
+ one mission or identification plan
+ one environment realization
                 ↓
       3DOF / pseudo-6DOF / 6DOF
                 ↓
paired trajectories + mapping report
```

---

## 3. Do not conflate these kinds of variation

Every generated experiment should distinguish at least the following parameter layers.

| Layer | Meaning | Example |
|---|---|---|
| **Morphology/design** | What kind of vehicle was instantiated | wing area, aspect ratio, body fineness, rotor spacing |
| **Mass properties** | Translational and rotational mass behavior | total mass, CG, inertia, fuel schedule |
| **Aerodynamics** | Force and moment behavior | drag polar, lift slope, damping derivatives, control increments |
| **Propulsion** | Force, resource, and response behavior | thrust map, burn time, spool lag, rotor coefficients |
| **Actuators/effectors** | How commands become physical effector states | deflection limits, rate limits, motor lag |
| **Controller** | How references become actuator commands | gains, mode schedule, authority profile |
| **Initial state** | How the experiment begins | altitude, speed, attitude, body rates |
| **Mission/experiment** | What motion or excitation is requested | waypoint route, elevator doublet, coast-down |
| **Environment** | External physical conditions | density, gravity, wind, turbulence, atmosphere |
| **Observation system** | What the learner receives | truth, IMU, GNSS, air data, latency, noise |
| **Failure/damage** | Optional off-nominal configuration | stuck surface, reduced motor output, mass asymmetry |
| **Numerics** | Solver behavior when intentionally varied | step size, integration profile, event tolerance |

These layers need independent seeds and provenance. A corpus consumer should be able to hold any layer fixed while varying another.

Recommended seed structure:

```text
vehicle_seed
mass_seed
aero_seed
propulsion_seed
actuator_seed
controller_seed
initial_state_seed
environment_seed
sensor_seed
failure_seed
```

The final run seed may deterministically derive these, but they should remain inspectable.

---

## 4. Model-source classes

### 4.1 Reference family

A reference family is tied to a specific source model or accepted reconstruction.

Examples:

```text
reference_f16_s119
reference_hl20_mod_k
```

Rules:

- The pinned reference plant is immutable.
- A change to accepted aerodynamic tables or equations creates a new source version.
- Corpus variation may include initial conditions, missions, controls, environments, and explicitly qualified parameter schedules.
- Uncertainty perturbations must be separately named and must not be presented as source truth.
- Broad synthetic mutations should create a different parametric archetype identity.

### 4.2 Parametric archetype family

A parametric archetype defines a design grammar and generator rather than one vehicle.

Examples:

```text
synthetic.conventional_fixed_wing
synthetic.flying_wing_uav
synthetic.lifting_body_glider
synthetic.ballistic_body
synthetic.rocket_payload_stack
synthetic.multirotor
synthetic.tandem_rotor_vtol
synthetic.jet_vtol
```

An archetype package contains:

- Latent design variables.
- Derived geometry and reference dimensions.
- Mass-property generator.
- Aerodynamic-model generator.
- Propulsion and actuator generators.
- Supported fidelity realizations.
- Validity and rejection constraints.
- Experiment and mission fragment compatibility.
- Standard observation and label profiles.
- Corpus sampling profiles.

### 4.3 Reduced family realization

A reduction is calibrated from a specific higher-fidelity parent or parent distribution.

Examples:

```text
f16.performance_3dof.v1
f16.attitude_response_p6dof.v1
hl20.energy_glide_3dof.v1
hummingbird.fleet_p6dof.v1
```

A reduction package must identify:

- Parent family and version.
- Parent configurations used for calibration.
- Quantities preserved.
- States and physics removed.
- Fitted parameter maps.
- Residual error surfaces.
- Valid envelope.
- Unsupported maneuvers.
- Cross-fidelity benchmark results.

### 4.4 Public-evidence surrogate family

Public-data surrogates sit between reference and synthetic models.

They should separate:

```text
public fact
derived estimate
generic archetype assumption
calibration choice
unknown range
```

Their parameter distributions must not imply access to proprietary data.

---

## 5. The parameter schema needs corpus semantics

The existing unit, scaling, bounds, and provenance metadata should be extended for parametric generation.

Each parameter should declare:

```text
semantic identifier
type and shape
physical dimension
canonical and display units
runtime scope
default
hard validity bounds
qualified bounds
corpus sampling bounds
scaling transform
distribution family
correlation group
conditional dependencies
derivation rule
fidelity applicability
source/evidence class
whether it is known to the learner
whether it is a prediction target
whether it is a nuisance variable
split-group key
resampling policy
```

Illustrative declaration:

```yaml
parameter:
  id: aero.drag.cd0_subsonic
  unit: "1"
  type: float

  bounds:
    hard: [0.002, 0.30]
    qualified: [0.008, 0.12]
    corpus: [0.01, 0.09]

  sampling:
    distribution: log_uniform
    correlation_group: fixed_wing_drag_polar
    condition: "morphology.class in ['conventional', 'flying_wing']"

  metadata:
    fidelity: [point_mass_3dof, attitude_response_p6dof, rigid_body_6dof]
    evidence: synthetic_derived
    learner_visibility: hidden_target
    target_group: static_aero_parameters
    split_group: vehicle_instance

  constraints:
    - "aero.drag.cd0_subsonic < aero.drag.cd_at_stall"
    - "aero.drag.k_induced > 0"
```

### 5.1 Bounds have different meanings

Do not use one `min` and `max` for everything.

- **Hard bounds:** values outside these are structurally invalid.
- **Qualified bounds:** the model has evidence or validation inside these limits.
- **Corpus bounds:** the current dataset edition samples this subset.
- **Recommended bounds:** friendly defaults for ordinary users.
- **Stress bounds:** deliberate edge cases, stored in a separate corpus lane.

### 5.2 Parameters should declare mutability

Suggested categories:

```text
family_constant
vehicle_instance
loadout_dependent
segment_configurable
reset_randomizable
runtime_controlled
failure_injected
internal_derived
```

A learner-facing dataset should never mistake a runtime control for an immutable vehicle property.

---

## 6. Generate physical coherence from a latent design vector

A wide corpus should sample compact, interpretable design variables and derive the rest.

### 6.1 Fixed-wing latent variables

Useful design variables include:

```text
reference area
aspect ratio
taper ratio
sweep
thickness ratio
body fineness ratio
wing loading
thrust-to-weight ratio
CG as fraction of reference chord
pitch/roll/yaw radii of gyration
control-surface area fractions
control moment arms
maximum control deflections
```

Derived quantities can include:

```text
span and mean chord
reference lengths
component positions
mass and inertia
lift-curve slope prior
induced-drag factor prior
control effectiveness priors
qualified speed and load-factor envelope
```

### 6.2 Ballistic and lifting-body latent variables

Useful variables include:

```text
mass
reference area
fineness ratio
nose radius
center-of-pressure trend
CG location
radii of gyration
base-drag scale
wave-drag onset
normal-force slope
static-stability margin
control flap area and moment arm
```

### 6.3 Multirotor latent variables

Useful variables include:

```text
vehicle mass
arm length
rotor radius
rotor count and layout
motor/propulsor thrust coefficient
motor torque coefficient
maximum rotor speed
battery energy
payload mass and offset
body drag areas
attitude-response bandwidth
```

### 6.4 Rocket and composite latent variables

Useful variables include:

```text
stage dry and propellant masses
body diameter and length
thrust-to-weight ratio
burn time
nozzle or gimbal authority
payload mass and attachment transform
stage separation state
fin area and location
aerodynamic payload geometry
mass-property evolution model
```

### 6.5 Coherence constraints

Every generator should enforce at least:

- Positive mass and resource quantities.
- Positive-definite inertia.
- Resolvable component transforms.
- CG inside declared structural or qualified limits.
- Smooth mass-property evolution except at declared topology events.
- Nonnegative drag in the qualified domain.
- Smooth coefficient surfaces to the declared continuity order.
- Valid symmetry rules where the archetype assumes symmetry.
- Actuator authority compatible with the claimed maneuver envelope.
- Propulsion and resource schedules that close consistently.
- No unsupported combination of loadout, fidelity, and segment.

Generators should report rejection reasons and acceptance rate. A sampler that silently rejects 99% of candidates is not mature.

---

## 7. Aerodynamic model generation should use structured functions

Do not randomize each aerodynamic table cell independently. That produces rough, internally inconsistent surfaces and makes it easy for a learner to exploit artifacts that would never occur in a real model.

Use one of these structured representations:

1. **Analytical coefficient families**
   - Drag polar.
   - Lift curve with a smooth stall transition.
   - Ballistic drag law.
   - Low-order force/moment derivative model.

2. **Spline or basis-function coefficient surfaces**
   - Baseline surface plus a bounded set of smooth basis perturbations.
   - Explicit continuity and monotonicity constraints where needed.

3. **Reference-model deformation**
   - Begin with a source or validated synthetic baseline.
   - Perturb physically meaningful modes such as drag level, lift slope, stall onset, damping, or control authority.

4. **Model-form mixtures**
   - Select among several legitimate formulations so the learner does not memorize one equation family.

### 7.1 Example fixed-wing force parameterization

A synthetic subsonic model could use:

```text
CL = CL0(M) + CL_alpha(M) * alpha + CL_delta_e(M) * delta_e
CD = CD0(M) + k(M) * CL^2 + CD_beta(M) * beta^2 + increments
CY = CY_beta(M) * beta + CY_delta_r(M) * delta_r
```

with a smooth stall or saturation model outside the linear core.

### 7.2 Example 6DOF moment parameterization

```text
Cl = Cl_beta * beta + Cl_p * p_hat + Cl_r * r_hat + control increments
Cm = Cm0 + Cm_alpha * alpha + Cm_q * q_hat + control increments
Cn = Cn_beta * beta + Cn_p * p_hat + Cn_r * r_hat + control increments
```

The actual implementation may use nonlinear tables, but the generator should perturb meaningful modes rather than uncorrelated cells.

### 7.3 Symmetry and asymmetry

A corpus needs both:

- Symmetric nominal vehicles.
- Explicitly labeled asymmetric vehicles caused by payload offset, damage, manufacturing variation, or asymmetric actuation.

Symmetry should be a generator constraint, not an accidental property of sampled data.

---

# Part I — Fidelity-specific parameter surfaces

## 8. Parametric 3DOF

A 3DOF model is force-complete but does not solve physical angular momentum. It can therefore expose broad, interpretable force and guidance abstractions.

### 8.1 Mass and loadout parameters

```text
initial mass
dry mass
consumable mass
payload mass
mass schedule or mass flow
stage or release mass changes
reference area
ballistic coefficient override
```

### 8.2 Aerodynamic parameters

At least one complete force scale is required. Possible parameterizations include:

```text
CD versus Mach
ballistic coefficient versus Mach
CD0 and induced-drag factor
CL versus alpha
CL command limits
CLmax and stall surrogate
L/D by segment
maximum load factor
bank-angle limits
side-force or coordinated-turn approximation
configuration drag increments
```

Important rule:

> `L/D` is a ratio, not a complete force model. A 3DOF instance still needs a drag scale such as `CD × area`, ballistic coefficient, or an equivalent acceleration law.

### 8.3 Propulsion parameters

```text
thrust-to-weight ratio
thrust versus time
thrust versus Mach and altitude
minimum and maximum throttle
throttle lag
burn time
mass flow
cutoff rule
thrust-direction rule
```

### 8.4 Abstract-control parameters

```text
bank response time
normal-acceleration response time
maximum bank rate
maximum load-factor rate
heading response limit
speed-control authority
command latency
control saturation behavior
```

These are not physical moment parameters. They describe the 3DOF control abstraction.

### 8.5 Initial-condition breadth

```text
position and altitude
speed or velocity vector
flight-path angle
heading and heading offset
bank command or initial lift-vector orientation
mass and resource state
launch rail or release direction
waypoint geometry
aim point or terminal corridor
```

### 8.6 Appropriate aerodynamic-identification targets

A 3DOF corpus can support estimation of:

```text
ballistic coefficient
CD or CD-area combinations
lift-to-drag behavior
lift or normal-acceleration response
thrust acceleration
mass-flow behavior when mass is otherwise known
wind or density nuisance terms
```

It cannot support identification of physical moment derivatives, inertia, or true control-surface moments because those do not determine its state evolution.

### 8.7 Broad synthetic 3DOF options

3DOF should support the widest notional families, including:

```text
ballistic object
lifting ballistic object
fixed-L/D glider
lift/drag-polar aircraft
powered cruise vehicle
rocket boost/coast vehicle
aggregate multirotor translation
orbital point mass
```

The exact force-model profile must be included in the resolved fidelity metadata.

---

## 9. Parametric pseudo-6DOF

Pseudo-6DOF adds explicit attitude and usually body rates, but attitude response is supplied by a declared response model rather than physical moment balance.

### 9.1 Everything available to 3DOF remains available

Pseudo-6DOF should reuse the same:

```text
mass and resource model
static force model
propulsion force model
mission and segment parameters
initial translational state
```

### 9.2 Required response-profile choice

Every instance must choose a named profile such as:

```text
p6dof.kinematic_attitude.v1
p6dof.second_order_attitude_response.v1
p6dof.body_rate_response.v1
p6dof.effector_response.v1
p6dof.multirotor_rate_response.v1
```

### 9.3 Attitude-response parameters

Depending on profile:

```text
roll/pitch/yaw time constants
natural frequencies
damping ratios
steady-state gains
maximum attitudes
maximum body rates
maximum angular accelerations
cross-axis coupling
command latency
attitude or rate deadband
low-authority behavior
Mach, dynamic-pressure, mass, or battery scheduling
```

### 9.4 Surface-like or effector-like parameters

Pseudo-6DOF can expose low-level-looking controls while remaining a response model:

```text
surface command limits
surface rate limits
actuator lag
neutral position
mixing matrix
surface-to-body-rate gains
surface-to-attitude gains
surface-to-force increments
saturation and failure behavior
```

These mappings must be labeled surrogate. They do not establish physical moment authority.

### 9.5 Aerodynamic parameters

Because attitude is explicit, the force model should normally respond to:

```text
Mach
angle of attack
sideslip
configuration
surrogate effector state
```

A pseudo-6DOF corpus can therefore vary:

```text
CL-alpha behavior
CD-alpha behavior
side-force behavior
stall/saturation shape
surface force increments
force asymmetry
attitude-response schedule
```

### 9.6 Appropriate identification targets

Pseudo-6DOF can support estimation of:

```text
all force-level 3DOF targets
attitude response time constants
rate-response gains
cross-axis response
surrogate control effectiveness
actuator lag and limits
force changes caused by explicit attitude
```

It should not label physical inertia or aerodynamic moment derivatives as identifiable truth unless those quantities actually enter the response equations.

### 9.7 Why pseudo-6DOF is especially useful for corpus breadth

It offers:

- Explicit orientation for sensor fields of view and thrust direction.
- Believable control lag, rate limits, and saturation.
- Easier stable simulation across broad notional designs.
- Lower cost than physical rigid-body dynamics.
- A natural bridge between guidance-level and direct-effector learning.

This should be the default fidelity for many large synthetic corpora and fleet experiments.

---

## 10. Parametric rigid-body 6DOF

Rigid-body 6DOF has more physical fields than either lower fidelity, but broad variation must be generated coherently.

### 10.1 Mass-property parameters

```text
component masses
component locations
initial total mass
CG vector
full symmetric inertia tensor about CG
fuel or propellant distribution
mass-flow schedule
payload attachment and release
stage transitions
configuration-dependent mass properties
```

Preferred notional parameterization uses:

```text
mass
CG fractions relative to reference geometry
nondimensional radii of gyration
principal-axis orientation
bounded products of inertia
```

then derives a positive-definite tensor.

### 10.2 Aerodynamic force and moment parameters

A controlled atmospheric 6DOF generator may expose:

```text
base CX/CY/CZ or CL/CD/CY surfaces
base Cl/Cm/Cn surfaces
Mach dependence
alpha and beta dependence
rate derivatives
control-surface increments
configuration increments
stall and nonlinear saturation
moment reference point
reference area, span, and chord
propulsion-aerodynamic interaction
```

### 10.3 Stability and damping parameters

Useful synthetic modes include:

```text
static pitch stability margin
weathercock stability
roll stability
pitch damping
yaw damping
roll-yaw coupling
Dutch-roll tendency
spiral tendency
short-period frequency and damping
```

The generator may use these as latent targets and solve for compatible derivative sets. It must reject combinations that contradict the selected archetype or produce an unqualified plant.

### 10.4 Control-surface and actuator parameters

For every surface or effector:

```text
location and axis
positive convention
minimum and maximum position
rate and acceleration limits
latency and transfer function
neutral and trim state
control-force increments
control-moment increments
hinge moment when modeled
power dependency
health and failure states
```

Useful corpus variation includes:

```text
control area ratio
control moment arm
effectiveness versus Mach/alpha
actuator bandwidth
rate saturation
deadband
asymmetric authority
control reversal or degradation cases, explicitly labeled
```

### 10.5 Propulsion parameters

```text
engine/rotor location
orientation and line of action
thrust map
gimbal axes and limits
spool or motor dynamics
reaction torque
mass/resource flow
ignition and shutdown transients
failure modes
```

### 10.6 Appropriate identification targets

A sufficiently excited and observed 6DOF corpus can support estimation of:

```text
mass and CG
inertia or radii of gyration
static force coefficients
static moment coefficients
rate derivatives
control derivatives
actuator dynamics
thrust-line and gimbal effects
propulsor reaction torque
configuration changes
```

However, identifiability depends on known inputs, measurement quality, and maneuver excitation. The dataset task declaration must state what is known and what is hidden.

### 10.7 Reference-family perturbations versus synthetic 6DOF

For F-16 and HL-20:

- Preserve the exact accepted plant as the reference identity.
- Permit separate uncertainty ensembles with explicit bounds and names.
- Create new synthetic archetypes for broad morphology and coefficient variation.
- Do not call a heavily deformed model an F-16 or HL-20.

Recommended derivative identities:

```text
synthetic.conventional_fixed_wing.performance
synthetic.lifting_body.energy_glider
```

They may use the reference families to calibrate ranges or validate methods without inheriting their identity.

---

# Part II — Archetype-specific parametric catalogs

## 11. Conventional and autonomous fixed wing

Applicable anchors:

```text
X8
F-16
B747
high-performance autonomous-jet surrogate
cruise surrogate
```

Recommended common parameters:

```text
mass and payload
wing area and aspect ratio
wing loading
thrust-to-weight ratio
CG fraction
radii of gyration
CD0, induced-drag factor, wave-drag mode
lift slope and CLmax
stall onset and shape
static pitch stability
lateral-directional derivatives
control area fractions and moment arms
actuator bandwidth and limits
fuel or battery model
```

Corpus morphology modes:

```text
conventional tail
tailless/flying wing
high-aspect-ratio endurance
transport-like
high-control-authority performance
low-observable notional planform, without proprietary claims
```

## 12. Lifting bodies, hypersonic gliders, and rocket aircraft

Applicable anchors:

```text
HL-20
X-15
synthetic hypersonic glider
boosted aerodynamic payload
```

Recommended parameters:

```text
mass and reference area
fineness and planform descriptors
CG and inertia
Mach-dependent lift and drag modes
normal-force and pitching-moment behavior
wave-drag onset
high-alpha behavior
bank authority
control-flap effectiveness
heating or dynamic-pressure proxy limits
booster handoff state
propellant and burnout behavior when powered
```

The corpus should include:

```text
air release
booster release
powered climb
burnout/coast
glide capture
bank reversals
energy-management descent
terminal energy corridor
```

## 13. Ballistic and tumbling objects

Recommended parameter modes:

```text
sphere-like
cylinder-like
plate-like
slender body
irregular aggregate surrogate
stabilized ballistic body
unstable or tumbling body
```

3DOF parameters:

```text
mass
reference area
CD(M)
ballistic coefficient
lift bias when present
ablation or mass-loss surrogate when enabled
```

6DOF parameters:

```text
full inertia
orientation-dependent force coefficients
center-of-pressure behavior
aerodynamic moments
rate damping
off-axis CG
initial tumble rate
```

These are especially useful for inverse aerodynamic estimation because they create large variation in drag area, orientation, and rotational coupling.

## 14. Multirotor and VTOL

Applicable anchors:

```text
Hummingbird
fleet Hummingbird reductions
tandem-rotor surrogate
jet-VTOL surrogate
```

Recommended parameters:

```text
mass and payload offset
rotor count and geometry
rotor radius and arm length
thrust/torque coefficients
motor bandwidth and saturation
body drag area
battery energy and voltage sag surrogate
attitude-response bandwidth
control-allocation matrix
wind sensitivity
transition behavior for compound or jet VTOL
```

For fleet corpora, prefer pseudo-6DOF unless rotor-level effects are an explicit target.

## 15. Rockets and carrier-payload compositions

Recommended parameters:

```text
stage count
stage mass fractions
thrust-to-weight
burn time and thrust curve
nozzle/gimbal authority
fin geometry and aero effectiveness
payload mass and geometry
attachment transform
separation impulse and uncertainty
child initialization mapping
```

A single sampled composition should be runnable as:

```text
aggregate 3DOF stack
pseudo-6DOF controlled stack
rigid-body 6DOF stack with physical separation
```

when the selected package supports all three.

## 16. Spacecraft

Spacecraft broaden the corpus but should remain a separate task domain from aerodynamic identification.

Useful parametric dimensions include:

```text
mass and inertia
area-to-mass ratio
drag and reflectivity coefficients
reaction-wheel capacity
thruster layout
sensor and antenna fields of view
orbit and attitude initial conditions
```

They are valuable for testing the common parameter/corpus infrastructure and for learning force-model or attitude-property estimation in a non-atmospheric domain.

---

# Part III — Identification experiments and corpus design

## 17. Mission trajectories are not enough for aerodynamic identification

Closed-loop waypoint flights are useful but can hide plant behavior:

- The autopilot may compensate for large aerodynamic differences.
- Several parameter combinations can produce nearly identical trajectories.
- A controller may avoid the angles, rates, or speeds needed to expose a coefficient.
- Position-only measurements may collapse several physical parameters into one identifiable combination.

Taoryx therefore needs a reusable library of **identification fragments** in addition to mission segments and objective fragments.

An identification fragment declares:

```text
required fidelity
required initial condition or trim
excitation signal
controlled and uncontrolled channels
safety/envelope guards
required observations
intended target parameters
minimum information or excitation checks
terminal conditions
```

## 18. 3DOF identification fragments

Recommended fragments:

### 18.1 Ballistic coast-down

Purpose:

```text
drag-area or ballistic-coefficient estimation
```

Variation:

```text
initial speed
altitude/density
mass
wind
Mach range
```

### 18.2 Powered acceleration pulse

Purpose:

```text
thrust and drag separation
throttle-response estimation
```

### 18.3 Climb and descent energy exchange

Purpose:

```text
lift/drag and thrust-energy behavior
```

### 18.4 Constant-bank turns at several speeds

Purpose:

```text
lift capability
load-factor behavior
induced-drag behavior
```

### 18.5 Fixed-alpha or fixed-CL sweep

Purpose:

```text
CL/CD curve recovery when the model exposes these controls
```

### 18.6 Launch, boost, coast, and cutoff sweep

Purpose:

```text
rocket thrust, mass flow, drag, and cutoff-event estimation
```

## 19. Pseudo-6DOF identification fragments

Recommended fragments:

```text
roll/pitch/yaw command steps
positive and negative doublets
frequency sweeps or chirps
bounded multisine excitation
combined-axis commands
alpha and beta sweeps
surface-like command sweeps
low- and high-dynamic-pressure response comparisons
actuator saturation and recovery tests
```

Targets:

```text
response time constants
natural frequencies and damping
rate limits
cross-axis coupling
surrogate control effectiveness
actuator lag
force changes with attitude
```

## 20. Rigid-body 6DOF identification fragments

Recommended fragments:

```text
elevator/aileron/rudder doublets
3-2-1-1 or similar bounded input sequences
frequency sweeps
orthogonal multisine inputs
trim perturbations
throttle and thrust-vector pulses
roll/yaw coupling maneuvers
angle-of-attack and sideslip sweeps
spin or tumble decay
reaction-control or rotor pulses
mass/configuration step events
```

Every excitation must be bounded by:

```text
state envelope
aero-table envelope
load limits
actuator limits
resource limits
terminal safety conditions
```

The corpus should store both requested and achieved excitation. A clipped input is not equivalent to the requested experiment.

---

## 21. Identifiability contracts

Each ML task must declare four sets:

```text
known inputs
hidden targets
nuisance variables
observed channels
```

### 21.1 3DOF limitations

From position and velocity alone, these may be confounded:

```text
mass
reference area
CD
air-density bias
wind
```

The identifiable target may be ballistic coefficient or `CD × area / mass`, not the individual quantities.

A task that asks for separate mass, area, and drag coefficient must provide additional known information or measurements.

### 21.2 Pseudo-6DOF limitations

A response-model trajectory can identify its response parameters, but it cannot establish a unique physical decomposition into:

```text
inertia
aerodynamic moment derivative
physical control moment
```

unless those quantities actually drive the equations.

### 21.3 6DOF requirements

Estimating physical moment derivatives or inertia generally requires:

```text
known or measured control inputs
attitude and body-rate observations
sufficient axis excitation
known frame/sign conventions
adequate dynamic-pressure variation
sensor bias treatment
```

Taoryx should reject a benchmark definition whose requested targets are structurally absent or intentionally hidden by the selected fidelity.

---

## 22. Corpus record hierarchy

Recommended hierarchy:

```text
CorpusEdition
  └── Split
       └── VehicleInstance
            └── Experiment
                 └── Trajectory
                      └── Window or episode sample
```

### 22.1 Vehicle instance

Contains immutable realized properties:

```text
family/archetype and version
fidelity realization
latent design vector
derived geometry
mass-property truth
aerodynamic truth
propulsion truth
actuator truth
valid envelope
source/evidence metadata
```

### 22.2 Experiment

Contains:

```text
initial state
environment
mission or identification fragment
controller/excitation
sensor profile
seed bundle
known/hidden/nuisance declaration
```

### 22.3 Trajectory

Contains:

```text
time series
controls
actuators
states
forces and moments
coefficient evaluations
envelope margins
events
terminal classification
validity masks
```

### 22.4 Window

A dataset builder may create fixed-length or event-centered windows while preserving the parent vehicle and trajectory identifiers for leakage-safe splitting.

---

## 23. Truth labels should be available at several levels

### 23.1 Static latent labels

```text
mass
CG
inertia
reference dimensions
drag-polar parameters
lift-slope parameters
rate derivatives
control derivatives
actuator parameters
thrust-map parameters
```

### 23.2 Function labels

Store coefficient evaluations on standard query grids:

```text
CD(M, alpha, configuration)
CL(M, alpha, configuration)
Cm(M, alpha, q_hat, delta_e)
control increments
```

This allows models to predict an aerodynamic function rather than only a generator-specific latent vector.

### 23.3 Per-step realized labels

```text
dynamic pressure
alpha and beta
force coefficients
moment coefficients
aerodynamic force and moment
propulsive force and moment
actual actuator state
```

### 23.4 Derived aggregate labels

```text
ballistic coefficient
wing loading
thrust-to-weight ratio
static margin
short-period or response-mode summaries
control-authority margin
```

### 23.5 Masks

A common target vector must include masks for inapplicable quantities. For example, a ballistic 3DOF object has no elevator derivative.

---

## 24. Observation editions

Produce related editions from the same truth run.

### 24.1 Clean truth

```text
full state
full controls
full forces/moments
no sensor noise
```

Purpose:

```text
algorithm development
simulator verification
upper-bound identifiability
```

### 24.2 Idealized instrumentation

```text
position
velocity
attitude
body rates
specific force
air data
known controls
```

### 24.3 Sensorized

```text
IMU noise and bias
GNSS rate and noise
barometric altitude
airspeed/angle sensors
latency and packet loss
quantization
```

### 24.4 Sparse or weakly observed

Examples:

```text
position-only
position and attitude
radar-like track
intermittent observations
```

### 24.5 Privileged-training edition

The policy or estimator receives limited observations while training infrastructure retains truth labels for loss and evaluation.

---

## 25. Sampling strategy

Do not rely on independent uniform random sampling.

### Stage A — Canonical baselines

One or more well-understood nominal vehicles per archetype.

### Stage B — One-at-a-time sensitivity sweeps

Useful for validating signs, monotonicity, and learner response.

### Stage C — Space-filling coherent designs

Use a low-discrepancy or stratified design over the latent parameter space, followed by derivation and constraint validation.

### Stage D — Boundary and corner campaigns

Sample near:

```text
stall or control-authority limits
low/high ballistic coefficient
minimum/maximum thrust-to-weight
CG boundaries
actuator bandwidth boundaries
data-table boundaries
```

### Stage E — Correlated uncertainty ensembles

Perturb related parameters together according to declared covariance or correlation rules.

### Stage F — Adaptive campaigns

Use model error, uncertainty, or coverage gaps to request additional simulations.

### Stage G — Deliberate stress lane

Contains physically unusual or model-edge cases. It must be labeled separately from the physically plausible corpus.

---

## 26. Coverage is more important than raw trajectory count

Each corpus release should publish coverage reports for:

```text
marginal parameter distributions
pairwise and selected higher-order coverage
archetype and morphology balance
fidelity balance
initial-condition coverage
Mach/altitude/alpha/beta coverage
control-excitation coverage
environment coverage
terminal-outcome balance
failure/rejection reasons
```

Recommended summary measures include:

- Distance to nearest training vehicle in normalized latent space.
- Occupancy of declared parameter bins.
- Coverage of coefficient-query grids.
- Time spent in each flight-condition region.
- Excitation energy per control axis and frequency band.
- Fraction of valid, failed, truncated, and out-of-envelope runs.

The system should report when a million trajectories are merely repeated variations around a tiny portion of parameter space.

---

## 27. Leakage-safe splits

The split must occur at the **vehicle-instance or higher** level before trajectory windows are produced.

Never place different maneuvers from the same realized aerodynamic model into both training and test unless the benchmark explicitly measures within-vehicle maneuver generalization.

Recommended benchmark splits:

### 27.1 In-family interpolation

Held-out vehicle instances inside the training archetype and parameter range.

### 27.2 Parameter-corner extrapolation

Entire regions of the latent design space are held out.

### 27.3 Morphology holdout

A planform or body-shape mode is absent from training.

### 27.4 Family holdout

An entire family or archetype is held out.

### 27.5 Controller holdout

The same plant distribution is flown by unseen controllers or excitation laws.

### 27.6 Environment holdout

Wind, atmosphere, or density regimes are held out.

### 27.7 Sensor holdout

Noise, latency, or observation modality differs from training.

### 27.8 Fidelity transfer

Train on lower fidelity, evaluate on paired or held-out higher fidelity, and vice versa.

### 27.9 Source-model holdout

Reference models such as F-16 or HL-20 are reserved as external validation anchors rather than being used in synthetic training generation.

Every split should be reproducible from a split manifest and independent split seed.

---

## 28. Paired multi-fidelity corpus design

There should be two paired-data modes.

### 28.1 Parent-derived pair

A 6DOF vehicle instance is the parent. Taoryx derives and calibrates:

```text
rigid_body_6dof
attitude_response_p6dof
point_mass_3dof
```

The three runs share:

```text
semantic design identity
mass/loadout where applicable
environment
mission reference
initial center-of-mass state
high-level command history
```

They differ in state, control realization, and omitted physics.

### 28.2 Archetype-correspondence pair

The fidelities are generated from a shared latent design vector but are not fitted to one exact 6DOF parent. This provides greater breadth at lower cost.

Every paired sample should publish:

```text
parameter mapping
control mapping
observation mapping
preserved quantities
expected disagreement
actual disagreement metrics
invalid comparison channels
```

Cross-fidelity labels should include:

```text
position/velocity error
energy error
event-time error
control-effort difference
attitude-response difference
terminal-outcome agreement
```

---

## 29. ML task catalog

The corpus should support distinct, versioned task definitions rather than one vague “estimate aerodynamics” goal.

### 29.1 Static parameter identification

Input:

```text
one or more trajectory windows
known controls
known initial/environment data
```

Targets:

```text
ballistic coefficient
drag-polar parameters
lift slope
response constants
mass/CG/inertia where identifiable
```

### 29.2 Aerodynamic function reconstruction

Target:

```text
coefficient values on a standard condition grid
```

This avoids tying the learner to one generator’s latent basis.

### 29.3 Instantaneous force/moment estimation

Target:

```text
aerodynamic force and moment or coefficients at each time step
```

### 29.4 Online system identification

The model updates its parameter belief as a trajectory unfolds.

### 29.5 Cross-fidelity correction

Input:

```text
lower-fidelity trajectory and parameters
```

Target:

```text
higher-fidelity trajectory residual or corrected coefficients
```

### 29.6 Vehicle or morphology inference

Predict archetype, shape mode, or latent design class from motion.

### 29.7 Active experiment selection

Choose the next maneuver or control excitation that most reduces uncertainty while respecting the envelope.

### 29.8 Trajectory forecasting under unknown plant properties

Forecast future state and uncertainty from partial observations and controls.

---

## 30. Baseline evaluation metrics

### Parameter metrics

```text
normalized RMSE
relative error
signed bias
coverage of predicted intervals
```

### Function metrics

```text
coefficient error on held-out condition grids
weighted integrated surface error
error near envelope boundaries
```

### Physical replay metrics

Apply estimated parameters in a simulator and compare:

```text
trajectory position/velocity
attitude and body rates
event timing
terminal state
force and moment residuals
```

### Generalization metrics

Report separately for:

```text
in-distribution vehicles
parameter-corner holdouts
morphology holdouts
family holdouts
fidelity transfer
sensor shifts
environment shifts
```

A low latent-parameter error is not sufficient if the reconstructed model produces poor forces or trajectories.

---

# Part IV — Tooling

## 31. Proposed CLI

```bash
# Inspect parameter spaces
taoryx parametric list
taoryx parametric describe synthetic.conventional_fixed_wing
taoryx parametric schema synthetic.conventional_fixed_wing \
  --fidelity rigid_body_6dof

# Realize one coherent vehicle
taoryx parametric realize synthetic.conventional_fixed_wing \
  --fidelity attitude_response_p6dof \
  --seed 18421 \
  --output realized-vehicle.json

# Explain derivation and constraints
taoryx parametric explain realized-vehicle.json \
  --parameter mass_properties.inertia

taoryx parametric validate realized-vehicle.json

# Plan and generate a corpus
taoryx corpus plan corpora/aero-id-v1.yaml
taoryx corpus generate corpora/aero-id-v1.yaml \
  --shard 0/128
taoryx corpus resume runs/aero-id-v1
taoryx corpus audit runs/aero-id-v1

# Build leakage-safe splits
taoryx corpus split runs/aero-id-v1 \
  --profile morphology-holdout-v1

# Generate paired fidelities
taoryx corpus pair runs/aero-id-v1 \
  --fidelities 3dof,p6dof,6dof

# Run baseline estimators and benchmarks
taoryx corpus benchmark runs/aero-id-v1 \
  --task aerodynamic-function-reconstruction-v1
```

## 32. Proposed Python concepts

```python
vehicle_space = catalog.parametric_space(
    "synthetic.conventional_fixed_wing",
    fidelity="rigid_body_6dof",
)

vehicle = vehicle_space.realize(seed=18421)

experiment = IdentificationExperiment.resolve(
    vehicle=vehicle,
    fragment="fixed_wing.orthogonal_multisine.v1",
    initial_condition="trim.medium_speed",
    environment="standard_atmosphere.calm",
    observation_profile="sensorized.airdata_imu_gnss.v1",
)

trajectory = runtime.run(experiment)
```

The exact API may differ, but realization, experiment definition, execution, and corpus materialization should remain separate objects.

---

## 33. Corpus manifest

Illustrative structure:

```yaml
schema: taoryx.corpus-plan/v1alpha1

corpus:
  id: org.taoryx.corpus.aero-identification-wide-v1
  version: 0.1.0
  purpose: aerodynamic_property_identification

vehicle_sources:
  - archetype: synthetic.ballistic_body
    fidelities: [point_mass_3dof, rigid_body_6dof]
    instances: 20000

  - archetype: synthetic.conventional_fixed_wing
    fidelities: [point_mass_3dof, attitude_response_p6dof, rigid_body_6dof]
    instances: 30000

  - archetype: synthetic.lifting_body_glider
    fidelities: [point_mass_3dof, attitude_response_p6dof, rigid_body_6dof]
    instances: 15000

experiments:
  profiles:
    - coast_down
    - trim_perturbation
    - control_doublets
    - frequency_sweep
    - waypoint_mission

observations:
  editions:
    - clean_truth
    - idealized_instrumentation
    - sensorized_v1

splits:
  profile: family_and_parameter_holdout_v1
  seed: 920314

artifacts:
  time_series: parquet
  coefficient_grids: zarr
  metadata: json
  compression: declared_by_profile
```

Counts above are illustrative planning values, not release requirements.

---

## 34. Runtime and storage considerations

A large corpus should support:

```text
sharded deterministic generation
restartable jobs
content-addressed vehicle and experiment manifests
columnar time-series output
optional array storage for coefficient grids
selectable telemetry profiles
streaming summary generation
sampled full traces
aggregate fleet traces
```

Do not log every internal state at the highest rate for every large corpus edition. Use declared output profiles:

```text
minimal learner observations
standard identification telemetry
full truth/debug telemetry
```

The complete truth must remain reproducible even when not materialized in every shard.

---

# Part V — Maturity and delivery

## 35. Parametric-family maturity ladder

### P0 — Declared

- Archetype identity exists.
- Latent parameter list exists.
- Intended fidelities and tasks are named.

### P1 — Constrained

- Units, bounds, dependencies, and derivations are machine-readable.
- Generated instances pass structural validity checks.
- Rejection reasons are classified.

### P2 — Sampler-Qualified

- Space-filling and boundary samplers are reproducible.
- Sampling coverage is measured.
- Acceptance rate is bounded and reported.
- Nominal and stress lanes are separate.

### P3 — Trajectory-Corpus Ready

- Instances compose with qualified experiments and missions.
- Batch and stepwise execution agree.
- Truth and observation schemas are stable.
- Leakage-safe split tooling exists.
- Coverage and validity reports are generated.

### P4 — Multi-Fidelity Paired

- Advertised 3DOF, pseudo-6DOF, and 6DOF mappings are implemented.
- Paired experiments have fixed mapping and disagreement reports.
- Unsupported cross-fidelity comparisons are masked.

### P5 — Benchmark Release

- Corpus edition is immutable and content-addressed.
- Baseline models and evaluation scripts are published.
- In-distribution and out-of-distribution tracks exist.
- Reference families provide external validation anchors.
- Reproduction, storage, and performance requirements are documented.

---

## 36. Recommended first vertical slices

### Slice A — Ballistic aerodynamic identification

Fidelities:

```text
3DOF
orientation-aware rigid-body 6DOF
```

Variation:

```text
mass
area
CD(M)
shape mode
initial speed/altitude
orientation and tumble for 6DOF
```

Purpose:

```text
prove identifiability contracts and ballistic-coefficient tasks
```

### Slice B — Parametric fixed wing

Anchors:

```text
X8 methods
F-16 reference plant
B747 mission tooling
```

Fidelities:

```text
3DOF
attitude-response pseudo-6DOF
rigid-body 6DOF derivative model
```

Purpose:

```text
prove lift/drag, stability, control-effectiveness, and paired-fidelity tasks
```

### Slice C — Parametric lifting body

Anchors:

```text
HL-20
X-15 mission and energy-management tooling
```

Purpose:

```text
prove broad-Mach glide and energy-management variation
```

### Slice D — Parametric multirotor

Anchor:

```text
Hummingbird
```

Fidelities:

```text
fleet 3DOF
fleet pseudo-6DOF
rotor-resolved 6DOF
```

Purpose:

```text
prove population generation, response identification, and fidelity transfer
```

### Slice E — Rocket plus aerodynamic payload

Purpose:

```text
prove composition, changing mass properties, launch handoff, and multi-regime trajectories
```

---

## 37. Minimum Alpha corpus release

A credible first release should demonstrate the infrastructure across several different identification structures rather than maximize volume.

Recommended minimum:

1. One ballistic-body parametric space.
2. One conventional/flying-wing fixed-wing parametric space.
3. One lifting-body/glider parametric space.
4. One multirotor parametric space.
5. At least one paired 3DOF/pseudo-6DOF/6DOF family.
6. Clean-truth and sensorized observation editions.
7. Identification fragments and complete-mission fragments.
8. Vehicle-instance, parameter-corner, morphology, and controller holdouts.
9. A simple baseline estimator for at least:
   - ballistic coefficient;
   - drag-polar parameters;
   - pseudo-6DOF response constants;
   - one 6DOF force/moment parameter subset.
10. Coverage, leakage, validity, and replay audits generated from one command.

The release should be small enough to reproduce locally and structured so a larger corpus can be generated by increasing the instance and experiment counts without changing semantics.

---

## 38. Definition of done

> **Taoryx’s Parametric Vehicle and Corpus Program is ready when a user can select a reference family, public surrogate, or coherent synthetic archetype; choose 3DOF, a named pseudo-6DOF profile, or rigid-body 6DOF; set or sample documented design, mass, aerodynamic, propulsion, actuator, initial-condition, environment, controller, and observation parameters; resolve those choices into a physically coherent and validity-bounded vehicle instance; compose a qualified mission or identification experiment; and generate deterministic trajectories with complete truth, sensorized observations, parameter provenance, validity masks, and reproducible corpus splits.**

> **For aerodynamic-property learning, every task must state which parameters are known, hidden, nuisance, and observable; use maneuvers that excite the requested properties; avoid splitting trajectories from the same realized vehicle across train and test; and evaluate both coefficient recovery and physical trajectory replay. A multi-fidelity corpus must additionally publish the mappings, preserved quantities, approximation disclosures, and measured disagreements among its 3DOF, pseudo-6DOF, and 6DOF realizations.**

---

## 39. The most important design rules

1. **Sample coherent vehicles, not unrelated scalar values.**
2. **Keep source-grounded reference plants immutable.**
3. **Use separate synthetic identities for broad deformations.**
4. **Treat pseudo-6DOF as a named response law, not a vague label.**
5. **Match ML targets to what the selected fidelity can physically identify.**
6. **Include identification maneuvers, not only autopilot waypoint flights.**
7. **Split by realized vehicle before creating trajectory windows.**
8. **Store both latent parameters and coefficient-function truth.**
9. **Measure parameter-space and flight-condition coverage, not only trajectory count.**
10. **Use paired fidelities to study transfer without pretending the models are equivalent.**
