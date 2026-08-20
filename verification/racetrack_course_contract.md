# Fixed-wing racetrack course template and contract

`powered_fixed_wing_racetrack_v1` is the reusable powered-fixed-wing mission
template. Its canonical binding catalog is
`verification/racetrack_templates.yaml`, and its typed resolver is
`taoryx.racetrack_template`. The template is deliberately separate from the
vehicle plant and controller: an X8, B747, Cessna, F-16, or other airbreather
may use the same mission semantics while supplying its own scale, trim,
controller, actuator realization, propulsion, resource model, and fidelity
claim.

The runtime realization is `mode=racetrack`. It is defined in the local
tangent plane at the declared start point and closes at the start/finish point:

```text
outbound climb → outbound level → left semicircle
→ inbound descent → inbound level → right semicircle → start/finish
```

The outbound and inbound straights are intentionally long enough to isolate
the vertical changes from the bank reversals. The turns are semicircles, so the
reference tangent is continuous and the turn directions are opposite.

Required route attributes:

```text
mode=racetrack
racetrack-length-m=...
racetrack-turn-radius-m=...
racetrack-speed-mps=...
racetrack-low-altitude-m=...
racetrack-high-altitude-m=...
racetrack-climb-rate-mps=...
racetrack-descent-rate-mps=...
duration-s=...                 # normally the estimate below, with margin
```

For a vehicle whose attitude response lags the nominal flight-path command,
`racetrack-altitude-capture-gain-per-s` and
`racetrack-altitude-capture-max-mps` provide an explicit bounded vertical
capture term. It is still a guidance velocity request; it does not inject
vertical force or bypass the vehicle controller. The truth altitude gates
remain the acceptance authority.

## Vehicle binding and preflight timing

Each binding supplies vehicle-level values rather than copying X8 numbers into
another model. The typed resolver derives phase windows, gate geometry, runtime
route attributes, and an auditable timing manifest. The first-pass timing
estimate is:

```text
straight_time = length / speed
turn_time = pi * turn_radius / speed
climb_time = (high_altitude - low_altitude) / climb_rate
descent_time = (high_altitude - low_altitude) / descent_rate
total_time = 2 * straight_time + 2 * turn_time
```

The estimator rejects a course when either vertical phase would consume the
whole straight. The result is a route horizon and objective-window estimate,
not a claim that the vehicle can meet the climb, descent, turn radius, or
terminal gate. Those are independently checked from truth telemetry.

The binding may add `simulation_margin_s` for post-gate observation. That
margin extends the simulation horizon but does not stretch the geometric route
or silently turn a missing terminal crossing into a pass.

The current reference bindings make the scaling explicit:

| Binding | Fidelity | Straight | Turn radius | Nominal speed | Vertical delta |
|---|---|---:|---:|---:|---:|
| X8 point-mass | point-mass 3DOF | 700 m | 250 m | 17.9 m/s | 10 m |
| X8 kinematic bridge | pseudo-6DOF / 3+3 | 700 m | 250 m | 17.9 m/s | 10 m |
| X8 reference | rigid-body 6DOF | 700 m | 250 m | 17.9 m/s | 10 m |
| B747 transport | point-mass 3DOF | 24 km | 6 km | 153 m/s | 150 m |
| B747 transport kinematic | pseudo-6DOF / 3+3 | 24 km | 6 km | 153 m/s | 150 m |
| B747 transport rigid | rigid-body 6DOF | 24 km | 6 km | 153 m/s | 150 m |

The B747 rows are scaled realizations of the same semantic course. The point
and kinematic rows use translational route guidance. The rigid row is a
separate source-table/direct-moment witness and does not inherit a physical
surface-allocation claim from the X8.

The complete, explicit run surface is documented in
`docs/verification/airbreathing-racetrack-fidelity-ladder.md` and exposed by
`tools/build_airbreathing_racetrack_ladder.py`.

The route phase index is:

| Index | Phase | Expected behavior |
|---:|---|---|
| 0 | outbound climb | positive flight-path command, zero bank |
| 1 | outbound level | level, straight, high altitude |
| 2 | left turn | high-altitude semicircle, positive route curvature |
| 3 | inbound descent | negative flight-path command, zero bank |
| 4 | inbound level | level, straight, low altitude |
| 5 | right turn | low-altitude semicircle, opposite curvature |

## Fidelity and realization boundary

The X8 fixture uses the existing attitude/LQR and surface-control seam. The
racetrack mode supplies only the geometric velocity and scheduled bank
reference; it does not inject forces or bypass the rigid-body equations. Its
two-elevon allocator is a bounded local realization, not a manufacturer flight
control law. The X8 direct-wrench packet is kept as a separate third-tier
diagnostic and is never presented as an elevon result.

A 3DOF binding uses the same geometry and truth gates but resolves the route
through translational guidance only. It may prove path, altitude, speed, and
energy behavior represented by that model; it must not claim attitude, moment,
or individual-surface behavior. A pseudo-6DOF binding may add a declared
attitude-response law. A rigid-body binding must state whether its bank/pitch
commands resolve through physical surface allocation or a direct/induced
moment path. These are separate tiers in the catalog; the template never
upgrades one into the other automatically.

The shared phase and gate contract is therefore reusable across the fixed-wing
family without making the plants interchangeable.

## Reusing the template for another airbreather

Add a binding to `verification/racetrack_templates.yaml` rather than copying
the X8 or B747 coordinates. The binding supplies only the vehicle-specific
quantities:

```yaml
my-aircraft:
  vehicle_id: my_aircraft
  fidelity: rigid_body_6dof
  straight_length_m: 1800.0
  turn_radius_m: 700.0
  speed_m_s: 42.0
  low_altitude_m: 300.0
  high_altitude_m: 500.0
  climb_rate_m_s: 2.0
  descent_rate_m_s: 1.5
  left_turn_bank_deg: 18.0
  right_turn_bank_deg: -18.0
  simulation_margin_s: 8.0
  source_realization: bounded_surface_allocation
```

The mission definition then references the binding and names the shared gates:

```yaml
racetrack_binding: my-aircraft
objectives:
  - id: high-altitude-level-gate
    objective_type: fly_by_gate
    racetrack_gate_id: high-altitude-level-gate
    tolerance: {corridor_m: 250.0, altitude_m: 20.0, speed_m_s: 5.0}
```

The packet builder resolves `racetrack_gate_id` into canonical local-NED
targets, oriented crossing normals, phase windows, and a preflight timing
manifest. The problem file still selects the plant, tables, controller, and
integrator, but its route attributes must agree with the resolved binding.
This makes route scale and objective geometry auditable while leaving trim,
guidance, control allocation, propulsion, resource use, and fidelity claims
vehicle-specific.
