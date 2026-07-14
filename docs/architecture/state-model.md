# TAOS state model

taoryx models the documented TAOS dynamics as a three-degree-of-freedom
point-mass system. The physical integration state is not the same thing as the
runtime's flexible named-value map.

## Canonical physical state

`taoryx.state.PointMassState` is the typed boundary for the main trajectory
integrator. Its frame is always ECFC and its packed order is:

| Index | Name | Meaning |
|---:|---|---|
| 0--2 | `x_ecfc`, `y_ecfc`, `z_ecfc` | Position |
| 3--5 | `xdot_ecfc`, `ydot_ecfc`, `zdot_ecfc` | Earth-relative velocity |
| 6 | `mass` | Vehicle mass |
| 7 | `path_length` | Integral of air-relative-speed/path speed |
| 8 | `ground_range` | Integral of ground speed |

The first six values are the manual's translational state. The last three are
the documented augmented integration variables. User-defined integral variables
will be added through an explicit extension rather than by changing this order.

## What is not state

Body attitude, aerodynamic angles, angular velocity, moments, and inertia are
not integrated states in the TAOS model. Guidance and force evaluation produce
the attitude/basis needed at each derivative evaluation. A `dt` guidance rule
requests a rate outcome; it does not add a rotational differential equation.

Consequently, adding Euler angles or angular rates to `PointMassState` would be
a 6-DOF product change, not an implementation detail.

## Runtime connection

The generic RK4/RKF45 code consumes `SimulationState` numeric vectors. The
typed state provides `to_simulation_state()` and corresponding unpacking so
the order and frame are validated at the boundary. `RuntimeState` remains the
parser/runtime envelope: it carries named values, segment metadata, derived
variables, and diagnostics. Runtime lowering should construct a
`PointMassState` at the physics boundary, evaluate forces/guidance from a
snapshot, return `PointMassRates`, and adapt the rates to the integrator.

The intended flow is:

```text
problem input
  -> RuntimeState / named values
  -> PointMassState (validated ECFC physical state)
  -> guidance + environment + force evaluation
  -> PointMassRates
  -> RK4 or RKF45
  -> PointMassState
  -> derived outputs / RuntimeState
```

This keeps parser semantics, physical state ordering, and numerical integration
separate and testable.

## Source boundary

This contract follows the reconstructed manual's introduction, equations of
motion, numerical integration, and trajectory-calculation sections. It does not
make a TAOS 96.0 compatibility claim and does not define a future 6-DOF model.
