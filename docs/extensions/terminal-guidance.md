# Terminal-guidance extension

## Selectable terminal owner

TAORYX separates the route command from the terminal ProNav command. The
route status declaration selects the owner:

```text
*runtime status route mode=great-circle terminal-guidance-mode=route ...
```

`terminal-guidance-mode=route` keeps the route/altitude command as the
terminal attitude owner. `terminal-guidance-mode=propnav` (or
`pure-propnav`) makes the native moving-target ProNav demand the terminal
attitude owner after `terminal-start-s`. In both modes the ProNav calculation
is still observable, and its aerodynamic allocation remains subject to the
declared control limits and tables.

The California-to-Hawaii showcase intentionally uses `route` today. Its
synthetic aerodynamic table is a wiring and regression fixture, not real
vehicle data. A pure-ProNav run must therefore be treated as a controller
integration experiment until real aerodynamic data is supplied.

Terminal guidance is layered after the vehicle and atmosphere models:

```text
target state
    → line-of-sight / range observables
    → desired acceleration or velocity
    → attitude / bank / angle-of-attack command
    → actuator and force/moment model
    → rigid-body state integrator
```

Proportional navigation must not directly overwrite position, velocity, or
attitude. A terminal-guidance showcase must identify:

- target motion model;
- guidance activation envelope;
- acceleration and attitude limits;
- wind convention;
- target-hit condition;
- terminal miss-distance tolerance;
- behavior when the target is outside the valid envelope.

Historical `*fly propnav` behavior and a modern TAORYX ProNav controller are
separate claims. Agreement of endpoint position alone does not establish
historical compatibility.

## Guidance-response telemetry

Native artifacts expose the following evidence channels:

- `pro_nav_los_range_m`, `pro_nav_closing_velocity_m_s`;
- `pro_nav_los_azimuth_deg`, `pro_nav_los_elevation_deg` and their rates;
- `pro_nav_acceleration_m_s2` and the ECIC command components;
- `pro_nav_achieved_aero_acceleration_m_s2`;
- `pro_nav_acceleration_response_residual_m_s2`.

The achieved value is deliberately labeled as aero response: it is derived
from the aerodynamic force only, not from total net acceleration. That keeps
gravity, propulsion, and aerodynamic response distinguishable in the
diagnostic record.

The independent 3-DOF reference tests only translational equations and RK4
convergence under controlled constant-force conditions. They do not certify
the 6-DOF attitude, actuator, or aerodynamic model.
