# Taoryx parametric spacecraft family expansion

Spacecraft should have their own **6DOF data and qualification track**, with
reaction-wheel and thruster systems treated as distinct actuator families.

The two best foundational examples are:

1. **A reaction-wheel-primary Earth-observation satellite**, with magnetic torquers or thrusters used only for momentum unloading.
2. **A thruster-primary free flyer**, with an RCS capable of producing commanded translation and rotation.
3. **A hybrid wheel/thruster spacecraft** as the follow-on example.

NASA’s current small-spacecraft guidance describes reaction wheels, magnetic torquers, and thrusters as common spacecraft actuators. Reaction wheels exchange and store angular momentum internally, eventually saturate, and therefore require an external torque source such as magnetorquers or thrusters for desaturation. Thrusters instead provide external forces and torques, consume propellant, and can control both translation and attitude. 

This plan captures the family contracts, source anchors, parametric ranges,
qualification tests, and corpus fixtures.

The numerical baselines and lifecycle corrections from the engineering review
are recorded in [spacecraft-parametric-fixtures-engineering-review.md](spacecraft-parametric-fixtures-engineering-review.md).
That review supersedes earlier loose nominal values where they conflict and
adds standard/resilient 6U variants, corrected SPHERES pulse behavior, and the
low-thrust MarCO-like mission interpretation.

The current fixture identities are:

```text
spacecraft.6u_observer_rw.standard.v1
spacecraft.6u_observer_rw.resilient.v1
spacecraft.agile_imager_rw.v1
spacecraft.spheres_like_rcs.v1
spacecraft.marco_like_hybrid.v1
```

# The basic spacecraft 6DOF contract

A space vehicle needs more than an inertia tensor and a generic torque input.

## Common data required by both vehicle types

| Area | Required data |
|---|---|
| Time and frames | Epoch, time scale, inertial frame, body-fixed frame, local orbital frame, quaternion convention |
| Initial translation | Position and velocity |
| Initial rotation | Attitude quaternion and body angular rates |
| Mass properties | Mass, center of gravity, full inertia tensor about the CG |
| Mass evolution | Propellant use, deployment, separation, moving appendages |
| Geometry | Actuator, sensor, payload, panel, and attachment transforms |
| Orbital environment | Central body, gravity model, atmosphere when applicable, solar radiation pressure, eclipse |
| Disturbance torques | Gravity gradient, drag, solar pressure, residual magnetic dipole, thrust misalignment |
| Sensors | Gyros, star tracker, Sun sensor, magnetometer, navigation or relative-navigation sensors |
| Resources | Wheel momentum, propellant, power, thermal or duty-cycle state |
| Control modes | Rate damping, inertial pointing, nadir pointing, Sun pointing, target tracking, translation hold |
| Finality | Stable hold, safe mode, maneuver completion, propellant floor, saturation, collision or numerical failure |

The six degrees of freedom describe three translations and three rotations. The actual simulation state will contain more than six scalar values because quaternions, actuator states, propellant, wheel speeds, sensor states, and controller states are also integrated or updated.

# Reaction-wheel-specific parameters

A wheel assembly should be a set of individual physical wheels, not a generic `body_torque` input.

Each wheel needs:

```text
spin axis in the body frame
rotor inertia
maximum positive and negative torque
maximum angular momentum
maximum speed
torque-speed relationship
motor response time
friction and drag
initial wheel speed
power use
temperature or duty limit
health and failure state
```

The ideal torque relationship is approximately:

\[
\boldsymbol{\tau}_{body}
=
-\mathbf A\dot{\mathbf h}_w
\]

where the columns of \(\mathbf A\) are the wheel axes and \(\mathbf h_w\) is the wheel-momentum vector.

A reaction-wheel command should not produce center-of-mass acceleration in the ideal model. It exchanges angular momentum between the wheel and the spacecraft.

NASA’s current small-spacecraft survey reports a very broad wheel-product range of roughly \(2.3\times10^{-4}\) to \(0.3\ \mathrm{N\,m}\) peak torque and \(5\times10^{-4}\) to \(8\ \mathrm{N\,m\,s}\) momentum storage. Those numbers are useful for defining notional wheel size classes, but not for independently randomizing torque and momentum; the same survey warns that catalog performance may come from manufacturer-provided information that NASA has not independently verified. 

## Wheel layouts

A minimal educational example can use three orthogonal wheels.

The pickup-ready family should use a **four-wheel skewed pyramid** and verify:

- Full three-axis torque authority.
- Allocation residuals.
- Momentum capacity in every direction.
- Continued three-axis control after any single wheel failure, within a reduced envelope.
- No hidden singularity in the advertised operating region.

NASA notes that three orthogonal wheels provide three-axis control, while four-wheel configurations are commonly used where mass and power do not permit a larger redundant set. 

## Reaction-wheel maturity levels

```text
RW0:
  ideal torque
  no wheel speed or saturation

RW1:
  wheel speed and momentum
  finite torque
  motor lag
  saturation
  allocation

RW2:
  torque-speed curve
  friction
  zero crossings
  jitter
  power and thermal effects
  detailed failures
```

The flagship example should require at least **RW1**.

# Momentum unloading is mandatory for the orbital example

The primary control system can still be described as reaction-wheel-controlled, but it needs an external actuator for unloading accumulated momentum.

For the first LEO example, use magnetic torquers:

\[
\boldsymbol{\tau}_{mtq}
=
\mathbf m \times \mathbf B
\]

where \(\mathbf m\) is the commanded magnetic dipole and \(\mathbf B\) is the local magnetic field.

For a deep-space or generic spacecraft, use balanced thruster pulses instead.

Reaction wheels can initially absorb deployment tipoff momentum if they have sufficient storage, but persistent environmental disturbances will eventually saturate them. NASA explicitly identifies magnetic torquers and thrusters as external desaturation actuators. 

# Thruster-specific parameters

Each RCS thruster needs:

```text
mount position in the body datum
unit thrust direction in the body frame
nominal thrust
on/off, pulsed, or proportional behavior
minimum impulse bit
minimum on time
minimum off time
valve latency
thrust rise and fall times
specific impulse
mass flow
tank-pressure or blowdown behavior
thrust dispersion
directional misalignment
intrinsic torque, when applicable
power and thermal limits
plume keep-out metadata
stuck-open, stuck-closed, and degraded states
```

The force and moment from each thruster are derived rather than separately handwritten:

\[
\mathbf F_i=T_i\hat{\mathbf d}_i
\]

\[
\boldsymbol{\tau}_i
=
(\mathbf r_i-\mathbf r_{cg})\times\mathbf F_i
+
\boldsymbol{\tau}_{intrinsic,i}
\]

This automatically captures the effect of changing center of gravity.

## RCS authority validation

Taoryx should derive an allocation matrix mapping individual thruster commands to the body wrench:

\[
\mathbf w =
\begin{bmatrix}
F_x & F_y & F_z & M_x & M_y & M_z
\end{bmatrix}^{T}
\]

A configuration advertised as fully controllable must have a rank-six authority matrix throughout its qualified CG range. NASA thruster-placement research similarly uses rank six as the condition for authority along all six degrees of freedom. 

Rank alone is not sufficient because ordinary thrusters cannot pull or command negative thrust. Taoryx should additionally calculate the **attainable wrench set** under:

```text
nonnegative thrust
maximum duty cycle
minimum pulse size
simultaneous firing restrictions
failed thrusters
propellant and power limits
```

## Thruster maturity levels

```text
RCS0:
  continuous thrust
  physical mount points
  simple mass flow

RCS1:
  minimum impulse bit
  minimum on/off times
  valve latency
  pulse modulation
  propellant and tank state

RCS2:
  blowdown
  thermal duty cycle
  plume interaction
  detailed pressure dependence
  detailed failure behavior
```

The flagship example should require at least **RCS1**.

Minimum impulse bit is especially important: NASA’s current guidance notes that thruster pointing and translation resolution depend on minimum impulse bit, while control authority depends on available force. NASA also identifies cold gas as a suitable first small-spacecraft example because it can provide small impulse bits, although its total impulse is comparatively limited. 

# Flagship example 1: reaction-wheel observer

```text
satellite_rw_deploy_sun_nadir_desat_6dof
```

Mission:

```text
launcher separation with tipoff
→ coarse rate damping
→ Sun-safe acquisition
→ reaction-wheel handoff
→ nadir acquisition
→ ground-target tracking
→ three-axis slew
→ disturbance-driven momentum accumulation
→ commanded momentum unloading
→ return to nadir or Sun-safe mode
→ terminal pointing hold
```

It must exercise:

- Positive and negative torque from every wheel.
- All three body axes.
- Changing orbital-reference pointing.
- Wheel momentum accumulation.
- Wheel saturation prevention.
- One complete unloading operation.
- Sensor-based rather than truth-only control.
- Safe-mode behavior.

Success should require:

```text
all pointing objectives completed
pointing and rate errors within limits
wheel momentum below final threshold
positive power/resource reserve
qualified final attitude mode
no model-envelope or frame errors
```

The key outputs are individual wheel torque, speed, momentum, saturation, total momentum vector, disturbance torque decomposition, pointing error, sensor-estimation error, and unloading performance.

# Flagship example 2: thruster-controlled free flyer

```text
freeflyer_rcs_waypoint_stationkeep_6dof
```

Mission:

```text
separation with position, velocity, attitude, and rate errors
→ attitude acquisition
→ relative translation hold
→ forward waypoint
→ lateral waypoint
→ vertical waypoint
→ commanded body slew
→ approach corridor
→ station keeping
→ retreat to safe separation
→ terminal hold
```

It must exercise:

- Positive and negative translation in all three axes.
- Positive and negative rotation about all three axes.
- Near-pure force commands.
- Near-pure moment commands.
- Combined force-and-moment commands.
- Pulse quantization and valve timing.
- Propellant use.
- Allocation saturation and failed-thruster reallocation in separate checkout fragments.

The initial scenario can use a virtual target and a declared LVLH or relative frame. A later version can propagate a second spacecraft and add relative-navigation sensors.

# Follow-on hybrid example

```text
satellite_hybrid_burn_pointing_desat_6dof
```

Mission:

```text
wheel-controlled nadir pointing
→ target slew
→ finite trajectory-correction burn
→ wheel rejection of thrust misalignment
→ post-burn reacquisition
→ thruster-based wheel unloading
→ science pointing dwell
```

This proves the realistic interaction between orbit control and attitude control:

- Thrusters change translation.
- Thruster offsets or misalignment disturb attitude.
- Wheels maintain or reacquire pointing.
- Thrusters unload accumulated wheel momentum.

# Required low-level tests

Before the full missions count, the library should pass:

### Common rigid-body tests

- Torque-free asymmetric body conserves rotational energy and angular momentum.
- Constant-torque response matches a reference.
- Two-body orbit conserves orbital energy and angular momentum.
- Quaternion norm remains bounded.
- Batch and stepwise execution agree.

### Wheel tests

- Equal-and-opposite wheel/body momentum exchange.
- No ideal translational acceleration from wheel commands.
- Three-axis allocation.
- Saturation at the declared momentum limit.
- Momentum unloading.
- Single-wheel-out allocation.

### Thruster tests

- Pure-force pair.
- Pure-moment pair.
- Correct translational and angular impulse.
- Correct propellant use.
- Minimum-impulse quantization.
- CG migration updates lever arms.
- Full-rank and attainable-wrench checks.
- Failed-thruster reallocation.

Failure and recovery should be treated as qualification behavior rather than optional decoration. NASA’s Goddard flight-system rules specifically emphasize limiting momentum from anomalous thruster firings, shutting down failed thrusters quickly enough for recovery, and testing polarity and performance before closed-loop use. In Taoryx, that maps naturally to stuck-open, stuck-closed, wrong-polarity, and degraded-thrust simulation tests. 

# Recommended pickup-ready claim

> **The Taoryx spacecraft 6DOF foundation is good to go when the library contains one reaction-wheel-primary spacecraft and one thruster-primary free flyer that share the same orbit, rigid-body, frame, control, event, telemetry, checkpoint, and evaluation infrastructure. The wheel vehicle exposes physical wheel axes, torque, momentum, speed, saturation, allocation, and external momentum unloading. The thruster vehicle exposes physical mount locations and directions, thrust, pulse resolution, valve dynamics, force/moment allocation, propellant use, center-of-gravity effects, and failed-thruster authority. Each vehicle completes a reproducible flagship mission and passes analytical, numerical, actuator, controller, and finality gates.**

That gives the space library three clear control archetypes:

```text
internal angular-momentum control
external impulse control
hybrid fine-pointing and maneuver control
```
