# ROCKET6G phase-aware Taoryx plug-in

## Scope

`cadac.rocket6g.launch_vehicle` is a source-grounded Python reconstruction of the participating ROCKET6G launch-vehicle plant. It preserves one `HYPER6` vehicle identity across stage and control-mode changes and publishes the active runtime fidelity rather than assigning one misleading realization to every sample.

The common-runner key is:

```text
provider_id = cadac
model_id    = cadac.rocket6g.launch_vehicle
executor_id = cadac.rocket6g.phase_aware.batch
```

## Source-program fidelity

| Source phase | Active realization | Returned fidelity |
|---|---|---|
| `aggregate_rcs` | axis-aggregate RCS force/moment | `rigid_body_6dof_direct_wrench` |
| `physical_tvc` | physical nozzle state and thrust wrench | `rigid_body_6dof_surface_allocated` compatibility tier |
| `mixed_tvc_rcs` | physical TVC plus aggregate RCS | T4 run envelope; aggregate RCS limitation retained in realization telemetry |
| `ballistic_coast` | uncontrolled rigid body | rigid-body slot with `uncontrolled_rigid_body` realization |

The public T4 compatibility string still says `surface_allocated`; ROCKET6G metadata identifies the physical actuator as thrust vectoring and never describes the nozzle as an aerodynamic surface.

## Participating source modules

```text
kinematics      body/inertial DCM state and incidence angles
environment     source atmosphere path, Mach, dynamic pressure, WGS84 gravity
propulsion      stage motor, fuel, mass, center of gravity, inertia
aerodynamics    stage-dependent coefficient tables and body coefficients
rcs             proportional or Schmitt axis-aggregate force/moment
tvc             physical nozzle states, limits, achieved thrust wrench
forces          aerodynamic + propulsion/TVC + RCS body wrench
newton          inertial translation on rotating WGS84 Earth
euler           rigid-body angular-rate integration
```

The source `gps`, `startrack`, `ins`, `guidance`, and `control` positions remain in the module schedule, but this plant realization intentionally does not execute their algorithms. Direct bounded commands enter where the omitted control modules would command TVC and RCS.

The source aerodynamic `actuator` module is also retained as a schedule position but does not participate because the insertion case does not activate aerodynamic control surfaces.

## Source events and stages

The event cursor checks only the next source event before each module pass. A fired event resets `event_time`, mutates supported source variables, and advances the cursor.

ROCKET6G requires support for an empty event body:

```text
IF thrust = 0
ENDIF
```

That event records burnout and resets the event epoch without changing another source variable. Subsequent source events install the second- and third-stage declarations.

Each lowered stage records:

- source aerodynamic and propulsion modes;
- gross and fuel mass;
- initial/final center of gravity;
- initial/final roll and transverse inertia;
- specific impulse and source fuel-flow rate;
- nozzle exit area when declared.

Stage changes remain internal transitions of one launch vehicle. They are not represented as spawned child vehicles because the source case discards lower stages rather than independently propagating them.

## Physical TVC boundary

The TVC path preserves:

```text
control request
  -> source command gain
  -> requested nozzle angles
  -> second-order nozzle position/rate states
  -> travel and rate limiting
  -> achieved nozzle angles
  -> achieved thrust force and moment
```

Requested and achieved nozzle channels are both returned. This is the evidence that supports the physical-effector portion of the T4 run envelope.

## Aggregate RCS boundary

The RCS path preserves:

- proportional moment and side-force modes with saturation;
- on/off moment and side-force modes;
- source dead zone and hysteresis;
- previous-error state used by the Schmitt transition;
- switch-count state;
- axis-aggregate body force and moment.

Individual thruster locations, jet selection, plume interactions, and physical allocation are not represented. The RCS path therefore remains T3 even during a mixed T4/T3 phase.

## Configuration boundary

The portable configuration exposes:

- geodetic launch truth and body orientation;
- initial body rates;
- direct TVC pitch/yaw requests;
- desired body thrust-vector direction for RCS vector-direction mode;
- RCS attitude commands;
- optional direct boost-cutoff time;
- end time and output cadence.

It does not expose arbitrary source-array mutation. Stage/event definitions and source decks remain immutable source-owned data.

## Output boundary

Core truth:

```text
position_inertial_m
velocity_inertial_mps
quaternion_wxyz
body_rates_inertial_rad_s
```

Selectable plant telemetry includes geodetic state, incidence, Mach/dynamic pressure, stage and phase identity, runtime fidelity, control realization, propulsion/RCS/TVC modes, mass/fuel/center of gravity/inertia, requested and achieved nozzle state, RCS wrench, and total body wrench.

## Evidence status

The runtime is registered at development status. Synthetic tests establish parser, stage-lowering, TVC, RCS, event, phase-transition, output-schema, and common-runner behavior. They do not establish numerical equivalence with a compiled CADAC executable.

Promotion requires:

1. exact source-case ingestion using the upstream decks;
2. module-local golden values for WGS84 transforms, propulsion, aero, TVC, RCS, force closure, Newton, and Euler;
3. one-step state parity;
4. event and stage-transition parity;
5. open-loop phase-program trajectory parity;
6. only then, independent addition and validation of LTG/autopilot and navigation-estimator layers.
