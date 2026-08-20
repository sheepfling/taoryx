# CADAC AGM6 plug-in

`cadac.agm6.missile` is the exact runnable AGM6 primary model. It executes one source-ordered `MISSILE6`, moving `TARGET3`, and tracking `AIRCRAFT3` composition.

## Fidelity

- Missile: `rigid_body_6dof_surface_allocated`.
- Moving target: `point_mass_3dof`.
- Tracking aircraft: `point_mass_3dof`.

The missile uses four individually lagged and limited aerodynamic fins. Achieved fin positions participate in the aerodynamic force-and-moment closure, so the model is a physical-effector T4 runtime rather than a direct-wrench approximation.

## Source ownership

The provider owns source actor order, module order, event progression, stored-derivative integration, table extrapolation, tracking cadence, packet refresh, the resulting datalink lag, target-position-norm update detection, launch-relative track extrapolation, and the source-literal 100 m target-plane interception sphere. Callers may override semantic initial conditions, guidance gain, source-case tuning, random seed, end time, and output cadence.

## Parametric source-case variants

`build_default_agm6_configuration` publishes source-backed initialization-time
variants in four typed groups: `actuation` (fin position/rate limits and
second-order dynamics), `seeker` (acquisition range and LOS-filter
gain/frequency/damping), `controller` (structural acceleration limit), and
`propulsion` (constant rocket throttle). Use the matching helper aliases such
as `fin_position_limit_deg`, `seeker_acquisition_range_m`,
`structural_limit_g`, and `propulsion_throttle`.

The configuration schema supplies canonical units plus the source model's
nonnegative or strictly-positive domain for every bounded value. These are not
new live actions: each run receives an immutable copy of the source definition
with the requested tuning applied. The native Taoryx relative-state track
remains the model's generic sensor integration; source seeker settings do not
create a competing sensor interface.

## Result composition

Mission Composition returns three root objects:

```text
agm6-missile-1    cadac.agm6.missile
agm6-ground-target-1     cadac.agm6.ground_target
agm6-tracking-aircraft-1 cadac.agm6.aircraft
```

Target tracks are carrier-owned communication events, not lineage records or hidden missile-only telemetry.

## Persistent composition and native sensor delivery

The executable four-fin realization supports `batch` and persistent `step`
execution. A session retains the complete MISSILE6, TARGET3, AIRCRAFT3,
source-event, IIR, tracker, datalink, controller, actuator, and stochastic
state. It accepts only holds that are exact multiples of the source `0.001 s`
`int_step`; it does not recreate a batch run for each composition step.

Each committed source substep publishes `agm6-native-relative-state` through
the Taoryx `SensorBus`. The packet is a typed geometry-only
`relative-state-track` from MISSILE6 to the actual TARGET3 truth state. It is
additional to—not a replacement for—the source IIR, AIRCRAFT3 target track,
or source datalink lag.

Session observations distinguish requested and achieved roll/pitch/yaw control,
requested and achieved individual fin angles, source normal/lateral commands,
and realized normal/lateral accelerations. These outputs permit finite-run
source-controller trace and comparison analysis. No trim, linear stability, or
frequency-margin claim is made, and source-managed controls are not caller
action ports.

## Evidence boundary

The physical plant, moving actors, track production, datalink, guidance/controller paths, source-shaped IIR state machine, and target-plane intercept are executable. Real-INS error propagation, complete optical corruption/aimpoint/gimbal-head geometry, exact C-rand parity, and compiled-CADAC numerical parity are not yet promoted.
