# Vehicle family validation v1

This plan defines the long-duration evidence required before TAORYX can call a
vehicle family validated. It deliberately does not use one short trajectory as
proof. Each family must pass plant, phase-mechanics, numerical-convergence, and
disturbance tests with inspectable plots and machine-readable reports.

The result is an engineering validation claim for the declared model and
envelope. It is not a flight-certification claim and it does not establish
accuracy for an unvalidated real vehicle.

## Validation levels

Every family progresses through these levels in order:

1. **Contract**: problem files, tables, units, frames, controls, and required
   outputs are declared and validated.
2. **Plant**: forces, moments, mass, environment, and actuators are active and
   numerically closed at the initial state.
3. **Mechanics**: a long multi-phase scenario exhibits the expected behavior
   for that vehicle family.
4. **Convergence**: the same scenario is stable under `dt`, `dt/2`, and
   `dt/4`, with phase metrics converging.
5. **Robustness**: bounded perturbations and control disturbances preserve the
   declared invariants or produce an explicitly expected failure mode.
6. **Artifact**: reports, telemetry, plots, manifests, hashes, and diagnostics
   are emitted by pytest and are reviewable without rerunning the model.

No family receives a validated verdict if a required level is blocked or if a
scenario leaves its declared table or operating envelope.

## Shared scenario contract

Each scenario is represented by:

```text
scenario.prb
scenario.yaml
tables/*.tbl
expected_channels.yaml
```

The scenario manifest must declare:

- family and vehicle model;
- source, synthetic, or mixed-data provenance;
- duration, integrator, and nominal time step;
- ordered named phases with start and end conditions;
- required controls and optional controls;
- required standard and family-specific telemetry;
- table validity envelopes and minimum query margins;
- phase-window metrics and pass/fail thresholds;
- expected events such as ignition, bank reversal, touchdown, or shutdown;
- convergence factors and perturbation cases;
- plot groups and artifact output locations.

Assertions are phase-windowed. A trajectory is not judged by comparing every
sample to a single golden array. The tests instead check directional behavior,
boundedness, event ordering, conservation, closure, settling, and converged
summary metrics.

## Family A: unpowered hypersonic glider

### Proof vehicle

Create a dedicated unpowered glider fixture. Do not use the powered X-15
rocket-plane as the glider proof vehicle. The fixture must have aerodynamic
tables, mass properties, atmosphere, gravity, bank/angle-of-attack control,
and zero propulsion.

### Nominal scenario: `glider_entry_pullout_glide_terminal`

Recommended simulated duration is 300--900 seconds, ending at a declared
terminal altitude, impact event, or controlled endpoint.

| Phase | Required behavior |
| --- | --- |
| Entry | Mach and dynamic pressure rise from the initial condition; mass stays constant. |
| Pullout | Flight-path angle becomes less negative; lift opposes gravity. |
| Glide | Downrange grows monotonically while energy decreases through drag. |
| Bank reversal | Crossrange rate changes sign after the commanded reversal. |
| Terminal | Altitude and energy decrease toward the endpoint without envelope violation. |

### Acceptance metrics

- final mass equals initial mass within numerical tolerance;
- propulsion force and propellant flow remain zero;
- downrange is monotonic after entry transients;
- specific mechanical energy does not increase beyond a declared numerical
  tolerance after the entry peak;
- peak Mach, dynamic pressure, heat-rate proxy, alpha, and bank remain inside
  the declared envelope;
- terminal descent has negative altitude rate;
- bank reversal produces the expected crossrange sign change;
- no aerodynamic table query extrapolates.

### Required plots

Altitude/time, Mach/time, airspeed/time, flight-path angle/time,
angle-of-attack/time, dynamic pressure/time, heat-rate proxy/time,
downrange/crossrange, specific energy/time, lift/drag/gravity, and commanded
versus achieved bank.

## Family B: powered fixed-wing

### Proof vehicles

Use the B747 as the transport-jet proof vehicle and the X8 as the smaller
fixed-wing proof vehicle. Their claims remain separate because their tables,
propulsion models, and control semantics differ. The X8 cannot receive a long
powered verdict until its propulsion and control tables are runtime-bound.

### Nominal scenario: `fixedwing_trim_climb_cruise_maneuver_descent`

Recommended duration is 600--1,800 seconds. The scenario must contain a
steady trim hold, a commanded altitude change, a heading maneuver, a cruise
hold, and a descent or go-around.

| Phase | Required behavior |
| --- | --- |
| Trim hold | Lift/weight and thrust/drag residuals remain bounded. |
| Climb | Altitude increases while throttle and flight-path angle have the expected signs. |
| Cruise | Airspeed and altitude settle near their commands. |
| Heading maneuver | Bank and heading change; sideslip remains bounded and recovers. |
| Descent/go-around | Descent or recovery event occurs in the declared order. |

### Acceptance metrics

- fuel mass decreases monotonically when the engine is operating;
- fuel consumed agrees with integrated fuel flow;
- cruise lift/weight and thrust/drag ratios remain within declared bands;
- altitude and airspeed step errors settle within declared times;
- heading and route segments occur in the expected order;
- alpha, beta, bank, rates, throttle, and control surfaces remain bounded;
- actuator achieved positions respect position and rate limits;
- no table query extrapolates.

### Required plots

Altitude/time, latitude/longitude, northing/easting, Mach/airspeed,
alpha/beta/bank, body rates, throttle/engine state, thrust/fuel flow/fuel
mass, lift/drag/thrust/weight, actuator command versus achieved position,
tracking errors, and table margins.

## Family C: quadrotor

### Proof vehicle

Use the Hummingbird model with explicit rotor allocation, motor dynamics,
attitude control, position control, and environment disturbance inputs.

### Nominal scenario: `quadrotor_takeoff_hover_translate_disturb_land`

Recommended duration is 120--300 seconds.

| Phase | Required behavior |
| --- | --- |
| Spin-up | Motors rise toward the commanded operating state without negative rotor speed. |
| Takeoff | Altitude increases and vertical velocity settles. |
| Hover | Position, attitude, and rates remain bounded. |
| Translation | North/east steps produce bounded, damped position responses. |
| Yaw step | Heading reaches the command without discontinuity or rate runaway. |
| Disturbance | Wind or force disturbance causes recovery, not unbounded drift. |
| Return/land | Vehicle returns toward the landing region and descends monotonically. |

### Acceptance metrics

- hover collective thrust approaches weight;
- position error remains bounded during hover;
- translation overshoot and settling time are within declared limits;
- yaw error converges with correct angle wrapping;
- rotor commands remain feasible and allocation residuals remain bounded;
- motor command-to-achieved lag matches the declared actuator model;
- attitude and body rates remain bounded during disturbances;
- landing event occurs after return and before motor shutdown.

### Required plots

North/east/down position, altitude/vertical velocity, position-error norm,
roll/pitch/yaw, body rates, rotor command versus achieved speed, total thrust,
body moments, saturation flags, disturbance state, and controller outputs.

## Numerical and robustness matrix

Every nominal scenario must run at `dt`, `dt/2`, and `dt/4`. The report must
compare at least:

- event times;
- peak and final altitude;
- final northing/easting or downrange/crossrange;
- peak speed or Mach;
- maximum envelope quantity;
- settling time and overshoot;
- fuel consumed or mass change;
- maximum actuator demand;
- maximum force/moment closure residual.

Each family also receives bounded perturbation cases:

- **Glider**: initial energy, atmospheric density, mass, aero coefficient,
  bank-command, and entry-angle perturbations.
- **Fixed-wing**: initial speed/altitude, wind, mass/fuel, trim bias, engine
  lag, and control-surface perturbations.
- **Quadrotor**: mass, inertia, center of gravity, motor time constant, rotor
  coefficient, wind, and one bounded rotor-efficiency degradation.

Perturbations must either satisfy the nominal invariants or produce a named,
expected diagnostic such as envelope exit, actuator saturation, or loss of
control authority.

## Execution order

1. Add the shared manifest schema and phase-window assertion helpers.
2. Add standard trajectory channels for geodetic/local position, velocity,
   acceleration, energy, force decomposition, fuel/mass flow, and actuator
   state.
3. Build and validate the glider nominal scenario and artifacts.
4. Expand the B747 long fixed-wing scenario and close its convergence gates.
5. Bind the X8 powered/control tables, then run its long scenario.
6. Expand the Hummingbird takeoff/hover/translation/disturbance/landing case.
7. Add perturbation scenarios and expected diagnostics.
8. Add Matplotlib artifact tests for every nominal and perturbation case.
9. Generate one family report containing plots, phase metrics, diagnostics,
   convergence results, and input hashes.
10. Mark a family validated only after all required scenarios pass.

## Final family verdict

A family report may say `validated` only when:

- contract, plant, mechanics, convergence, robustness, and artifact levels pass;
- all required phases execute in order;
- all required channels and plots exist;
- no unapproved table extrapolation occurs;
- the result is reproducible from the recorded problem files and table hashes.

Otherwise the report must use `blocked`, `needs-fix`, or `evidence-only`, with
the failing phase and metric identified.
