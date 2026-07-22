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

`max-sideslip-deg=N` projects the commanded body-X direction into a bounded
body-Y corridor before moment control. This is a command limit; it does not
alter the measured aerodynamic sideslip or clamp a table query.

`max-bank-deg=N` bounds the alpha/bank acceleration allocator's bank command.
It is useful when a source vehicle's aerodynamic or actuator data does not
support an unrestricted ninety-degree bank. Like the sideslip limit, it is a
command bound and does not alter the measured state or permit table
extrapolation.

`sideslip-gain=N` adds a body-axis yaw feedback moment from measured
air-relative sideslip. `sideslip-rate-damping=N` adds body-yaw-rate damping to
that same loop. Both are disabled by default and are intended for a
vehicle-specific 6-DOF extension where the coefficient deck supports this
closed-loop approximation. The resulting moment remains subject to the
declared actuator maximum.

`rudder-hold-gain-deg-per-deg=N` provides a route-independent beta-hold using
the declared rudder table. Its optional target is
`rudder-hold-target-deg=N`; the default target is zero sideslip. This is a
trim/plant controller and does not replace route or ProNav guidance.

`energy-management=alpha-drag` enables an opt-in energy loop. When
`energy-target-speed-mps=N` is below the current airspeed, the controller
raises the table-query alpha command according to
`energy-alpha-gain-deg-per-mps=N`, bounded by `energy-max-alpha-deg=N`.
This changes the aerodynamic force through the declared coefficient tables; it
does not inject an artificial deceleration or alter the integrated equations
of motion.

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

## Segment-scoped route activation

For staged rigid-body showcases, route attitude steering is activated by the
existing segment `*fly propnav` block. Booster and coast segments without that
block do not receive route steering; no separate segment selector is needed.
This does not clamp aerodynamic angles or authorize table extrapolation.

For a declared payload release, `release-attitude=airflow` on the vehicle
status block acquires body-X along the current air-relative velocity and
chooses a coordinated body-Y plane at the segment boundary. This is an
explicit attitude reset at release, not a table-query clamp.

The route profile also supports explicit powered, coast, and terminal flight
path shaping through `powered-end-s`, `apogee-altitude-m`, `terminal-start-s`,
and the terminal descent-rate controls. Staged problem files should pair that
profile with altitude-based `*when ... goto ...` conditions and a time fallback
for booster cutoff and payload release.

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
