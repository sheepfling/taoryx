# Fidelity data requirements and readiness checks

Status: implementation contract for model intake

This document defines what Taoryx must know before a vehicle can honestly be
run or advertised at each of the four fidelity tiers:

1. `point_mass_3dof`
2. `pseudo_6dof_kinematic_bridge`
3. `rigid_body_6dof_direct_wrench`
4. `rigid_body_6dof_surface_allocated`

The fourth tier is not a fifth equation-of-motion mode. It is the physical
effector realization split inside rigid-body 6DOF. A rigid-body integrator can
run with a direct moment, a source table, or an actual allocator; those are
different claims and must remain different readiness results.

The machine-readable source of truth is
[`verification/fidelity_data_requirements.yaml`](../../verification/fidelity_data_requirements.yaml).
Run a check with:

```bash
python3 tools/validate_fidelity_readiness.py --vehicle b747 --tier all
python3 tools/validate_fidelity_readiness.py --vehicle hummingbird --tier rigid_body_6dof_surface_allocated
```

The validator reports metadata readiness only. `ready_for_runtime_probes` does
not mean that trim, control, mission, convergence, robustness, or source
correlation has passed. Every report carries `runtime_proof_status:
not_evaluated` until those separate probes are run.

## 1. The data boundary

Every vehicle intake has six separate data questions:

| Question | Examples | Why it matters |
| --- | --- | --- |
| What is the body and environment? | Frames, epoch, gravity, atmosphere, geometry | Prevents unit, frame, and reference-point ambiguity |
| What produces translational load? | Analytic force, aero tables, thrust map, rotor wrench | Defines the center-of-mass claim |
| What produces rotational load? | No moments, response law, source moments, induced moments | Defines whether rotation is physical or prescribed |
| What commands the plant? | Semantic acceleration, bank, direct wrench, surface/rotor/thruster commands | Separates guidance from effector realization |
| What changes with time? | Fuel, propellant, battery, rotor RPM, wheel momentum, actuator state | Prevents hidden energy or resource creation |
| What proves the model? | Provenance, domains, trim/hover/orbit, equation closure, probes | Separates data presence from qualification |

The checklist is intentionally conservative. A missing source table does not
become acceptable because a controller can still make a trajectory move. A
direct force or moment can be perfectly valid at a reduced tier, but the
result must carry the direct-wrench claim rather than an actuator claim.

## 2. Tier requirements

### 2.1 Point-mass 3DOF

The minimum executable contract is a translational state and a declared load
and resource policy:

```text
position, velocity, navigation frame
mass policy
gravity and environment
force / lift-vector / acceleration realization
propulsion realization, if powered
resource policy, including explicit fixed-mass/no-depletion cases
semantic controls and achievable limits
initial condition or trim source
terminal/objective contract
provenance and valid domain
```

Required physical data usually include mass, reference area or an equivalent
force scale, gravity, atmosphere when applicable, thrust or power authority,
drag/lift or a direct acceleration model, and fuel/propellant/battery data when
resource consumption is claimed. Inertia, moments, body rates, and individual
effectors are not required and must not appear as implied claims.

Typical realization:

```text
mission error → achievable acceleration / lift vector / throttle
             → force and resource model
             → translational EOM
```

### 2.2 Pseudo-6DOF / 3T+3K

This tier inherits the 3DOF requirements and adds a named attitude response
contract:

```text
attitude representation and frame
quaternion/Euler convention and propagation
response law or identified transfer model
time constants, damping, rate and acceleration limits
attitude command and achieved telemetry
initial attitude/rate and terminal orientation contract
response-law provenance and calibration cases
```

It may integrate forces for translation and a response law for attitude. It
does not require a physical inertia tensor if the response law does not use
one, but it must not claim moment balance or physical effector activity.

Examples include a bank-to-turn response for a fixed-wing vehicle, a
multirotor wrench-response surrogate, a helicopter scheduled response model,
or a spacecraft pointing response. The name of the response law is part of
the resolved fidelity identity.

### 2.3 Rigid-body 6DOF with direct or induced wrench

This tier inherits the translational and attitude contracts and requires a
real rotational plant:

```text
mass and positive inertia tensor
body frame, moment reference point, and sign conventions
force and moment source classification
source aerodynamic/propulsive load tables or analytic load model
direct/induced wrench authority and axis mapping, when used
rotational control law and limits
force/moment decomposition and closure telemetry
resource coupling for propulsion or energy claims
```

The phrase “direct wrench” is deliberately explicit. It means a generalized
force or moment is applied to the rigid-body equations. It does not mean an
elevon, rotor, thruster, wheel, or gimbal was solved. Source control-conditioned
tables are also not automatically an actuator model; the manifest must say
whether they are source-direct controls or part of an allocator.

### 2.4 Rigid-body 6DOF with physical effector allocation

This tier inherits direct-wrench data and adds the information needed to map a
request through declared physical effectors:

```text
effector identity, geometry, axes, and moment arms
control-to-load tables or a calibrated local Jacobian
sign conventions and trim/control offsets
absolute travel, rate, lag, and failure limits
allocation objective, weighting, regularization, and saturation policy
requested versus achieved effector telemetry
requested versus achieved wrench and allocation residual
resource/energy coupling for engines, rotors, thrusters, wheels, or dipoles
```

The minimum proof is not merely “the surface values changed.” It is:

```text
requested wrench
  → bounded allocation
  → achieved effector commands
  → force/moment evaluation
  → achieved wrench
  → residual and saturation telemetry
```

A table-conditioned surface inversion can qualify as a bounded local
allocation, but only inside its table domain and stated travel/rate limits. It
does not automatically provide a complete flight-control law, hinge moments,
servo dynamics, blade dynamics, inflow, plume interaction, or certification
fidelity.

## 3. What changes by family?

The four tier contracts are stable. Family overlays change the data needed to
instantiate each contract:

| Family | 3DOF emphasis | Pseudo-6DOF addition | Direct-wrench addition | Physical-effector addition |
| --- | --- | --- | --- | --- |
| Powered fixed wing | Aero convention, lift/drag/thrust, mass/fuel policy | Bank/pitch/yaw response and limits | Inertia, body loads, direct moments or induced forces | Elevator/elevon/aileron/rudder geometry, tables, rates, allocator residual |
| Rocket/booster | Thrust curve, mass flow, staging, gravity/drag | Attitude/gimbal response and separation settling | Changing mass/inertia and thrust-vector moments | Gimbal or thrust-vector geometry, actuator limits, stage events |
| Glider/lifting body | Polar, L/D, energy, release and terminal corridor | Bank/pitch response and speed-brake lag | Orientation-dependent aero moments | Elevon/body-flap/rudder tables and bounded allocation |
| Multirotor | Net thrust/force, mass, battery policy | Named attitude/wrench response | Rigid body plus source wrench and reaction-torque terms | Rotor positions, axes, spin signs, thrust/torque maps, motor limits and mixer |
| Conventional helicopter | Power/collective authority, hover/forward-flight schedule | Scheduled derivative or response model | Rotor/fuselage loads and rotational state | Main/tail rotor geometry, cyclic/collective/pedal mapping, inflow/flap/drivetrain only if claimed |
| Tiltrotor / VTOL transition | Blended force model and mode schedule | Mode-dependent response and transition lag | Nacelle/wing/rotor force and moment sources | Nacelle actuator, mixer schedule, conversion guards, asymmetric limits |
| Spacecraft | Orbit state, frames, gravity, maneuver resources | Pointing response | Inertia and external/internal torque model | Wheel axes, thruster layout, dipole/field model, momentum/propellant/power limits |
| Ballistic/tumbling body | Ballistic coefficient, atmosphere, impact state | Prescribed orientation response only | Inertia and orientation-dependent aero moments | Usually not applicable; only declare an effector if one exists |

Examples of family-specific consequences:

- A fixed-wing aircraft can use direct forces at 3DOF, a bank-response law at
  pseudo-6DOF, direct moments at rigid-body 6DOF, and elevon allocation at the
  physical tier.
- A Hummingbird can legitimately use a source-wrench table plus a quad-X
  mixer. That is physical rotor command allocation, but not blade-resolved
  aerodynamics or battery depletion unless those are separately modeled.
- A magnetorquer spacecraft has no meaningful 3DOF actuator claim. The orbit
  host may be 3DOF, while detumble authority requires the 6DOF magnetic field,
  dipole, and torque model.
- A tumbling body cannot prove moment-driven tumble at 3DOF or pseudo-6DOF;
  those tiers can only provide an orientation-averaged or prescribed-motion
  baseline.

## 4. Validation stages

The readiness tool is the first gate in a larger sequence:

```text
data intake
  → metadata and artifact readiness
  → units, frames, and table-domain validation
  → mass/inertia/resource invariants
  → trim, hover, release, orbit, or equilibrium probe
  → control-direction and allocation probe
  → force/moment/resource closure
  → timestep and batch/step parity
  → family mission objectives
  → fixed perturbations and robustness
  → claim review
```

The report deliberately has two statuses:

- `status`: whether the declared data contract is ready for runtime probes;
- `runtime_proof_status`: whether the later qualification probes have run.

This prevents a “complete-looking” registry entry from being promoted merely
because it has geometry, a table, and a controller profile.

## 5. Checklist output and promotion rules

Each result contains every requirement, its severity, the paths inspected, and
an actionable message. The statuses are:

| Status | Meaning |
| --- | --- |
| `ready_for_runtime_probes` | Required intake data are declared and reachable; qualification has not yet been proved |
| `partial` | Required intake data are present, but recommended disclosure or runtime-probe declarations remain |
| `blocked` | A required physical, provenance, or realization input is absent |
| `not_evaluated` | A runtime probe or equation check is intentionally outside metadata validation |

Promotion is tier-specific. A B747 can be ready for direct-wrench probes and
blocked for surface allocation at the same time. A Hummingbird can be ready for
rotor allocation while still blocked for a battery-energy claim. Family names
never promote a vehicle automatically.

The next implementation extension should add per-vehicle `readiness_probes`
and `claim_profiles` to the registry, allowing the same command to launch the
appropriate trim, hover, orbit, allocation, resource, and mission tests. The
current checklist intentionally stops before that execution layer so missing
data is not hidden by a passing controller.
