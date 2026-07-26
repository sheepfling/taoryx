# F-16 S-119 complete reference model

## Purpose

This package is the first end-to-end Taoryx aircraft-model reference implementation. It binds static source models into one canonical evaluator and preserves enough evidence to reproduce every model-specific transformation.

“Complete” means complete for the package contract: nonlinear six-axis aerodynamics, steady propulsion, rigid-body mass properties, direct controls, model packaging, trim, and six-degree-of-freedom compatibility. It does not mean operational-system or training fidelity.

## Source components

| Role | Model | Normalized runtime content |
|---|---|---|
| Aerodynamics | F-16 Mod K subsonic aerodynamic model | 51 variables, 18 tables, 744 values, 17 checks |
| Propulsion | F-16 propulsion model, Stevens & Lewis form | 13 variables, three 6×6 thrust tables, nine checks |
| Mass properties | F-16 constant inertia model | mass, CG offset, six inertia terms, three derived checks |
| Controls | S-119 direct manual mixer subset | four normalized inputs and four algebraic outputs |

Every source record includes repository, path, upstream blob SHA, normalized document SHA-256, role, and model version where available.

## Canonical contract

### Inputs

```text
air-relative body velocity [m/s]
body angular rate [rad/s]
body-to-NED scalar-first quaternion
altitude [m]
local atmospheric state [SI]
normalized throttle, elevator, aileron, rudder, and trim ratios
configuration state
mass, CG offset, and inertia tensor [SI]
```

### Outputs

```text
aerodynamic body force [N]
aerodynamic body moment [N m]
propulsion body force [N]
propulsion body moment [N m]
other and gear contributions [N, N m]
fuel flow [kg/s] with availability diagnostic
domain disposition, warnings, and audit diagnostics
```

Body axes are forward-right-down. Navigation axes are north-east-down. Products of inertia use the standard positive product convention and appear with negative off-diagonal signs in the inertia matrix.

## Propulsion semantics

The source supplies steady thrust as a function of:

```text
power lever angle: 0 to 100 percent
altitude:          0 to 50,000 ft
Mach:              0 to 1.0
```

Power lever values below 50% interpolate between idle and military thrust. Values at or above 50% interpolate between military and maximum augmented thrust.

The model returns body-X thrust only. Propulsion moments are zero in the source model. Fuel flow and engine transient state are unavailable and are not synthesized.

## Mass-property semantics

The source mass is 637.1595 slug, corresponding to 20,500 lbm. The source inertia values are constant:

```text
Ixx =  9,496 slug ft²
Iyy = 55,814 slug ft²
Izz = 63,100 slug ft²
Ixz =    982 slug ft²
Ixy = Iyz = 0
```

The longitudinal CG offset is:

```text
DXCG_ft = 0.01 * 11.32 * (35 - CG_percent_MAC)
```

and is positive forward from the 35%-MAC moment reference center.

## Control semantics

```text
longitudinal = limit(elevator_ratio + pitch_trim_ratio, -1, 1)
lateral      = limit(aileron_ratio + roll_trim_ratio, -1, 1)
directional  = limit(rudder_ratio + yaw_trim_ratio, -1, 1)

elevator_deg = -25.0 * longitudinal
aileron_deg  = -21.5 * lateral
rudder_deg   = -30.0 * directional + 0.008 * aileron_deg
PWR_percent  = 100.0 * throttle_ratio
```

The source file also contains SAS and autopilot logic. Those stateful control functions are intentionally not included in this direct manual baseline.

## Trim solve

The reference trim solves angle of attack and elevator using a central-difference Newton method, then solves throttle using bounded bisection over the propulsion map. The equations close body-X force, body-Z force, and pitch moment for straight, wings-level flight.

The published reference condition is encoded as a reproducible input object rather than hidden constants inside the solver.

## Six-degree-of-freedom validation

The validation propagator evaluates:

```text
position rate in NED
body translational acceleration
quaternion rate
body angular acceleration using the complete inertia tensor
```

The equations include body-rate cross products, gravity transformed from NED into body axes, and moment coupling through angular momentum.

The five-second trim-hold case demonstrates internal consistency. It is not an external trajectory-validation claim. Reproducing a full NASA/NESC scenario requires matching the reference environment, controls, integration settings, and scenario initialization.

## Package evidence

The `.txair` package embeds:

- all three executable normalized DAVE-ML component documents;
- the control-binding contract;
- the runtime-binding contract;
- all component check reports;
- the solved reference trim;
- the trim-hold summary;
- the complete 501-row validation trajectory;
- SHA-256 records for every artifact.

## Known limitations

1. Aerodynamics is subsonic and the package declares Mach 0–1.
2. Steady propulsion does not model spool dynamics or fuel flow.
3. Mass and inertia do not change with fuel or stores.
4. Control surfaces have no actuator state, rate limit, or delay.
5. SAS and autopilot are not bound.
6. No landing gear, ground contact, stores, damage, or flexible modes are present.
7. The local validation propagator is flat-Earth and uses constant local gravity.
8. Normalized source mirrors preserve semantics and provenance but are not claimed to be byte-identical to every upstream source document.
