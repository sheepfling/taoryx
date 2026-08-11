# CADAC ADS6 SAM plug-in

`cadac.ads6.sam` is the exact runnable vehicle-only ADS6 `SAM6` model. `ROCKET5` and `AIRCRAFT3` are independently runnable, while `RADAR0` and the source controller participate through the separate `cadac.ads6.engagement` package model.

## Realizations

- `fin_control`: T4 physical four-fin control.
- `tvc_control`: T4 physical pitch/yaw thrust-vector control.
- `aggregate_rcs`: T3 axis-aggregate direct-wrench RCS.

The provider requires the source `actuator`, `tvc`, and `rcs` modules before advertising all three realizations. Phase, fidelity, realization, and mission-template mismatches fail closed.

## Source ownership

The provider owns source lowering, module order, table interpolation/extrapolation, stored-derivative integration, flat-Earth rigid-body truth, propulsion/mass-property tables, aerodynamic closure, physical fin/TVC states, aggregate-RCS state, and requested-versus-achieved telemetry. Callers provide bounded commands at the source controller-output seam.

## Physical-effector evidence

For fins and TVC, achieved physical positions—not requested commands—participate in force-and-moment closure. The RCS path remains T3 because the source produces aggregate body-axis force and moment rather than allocating individual jets.

## Result composition

Mission Composition returns one independent root object:

```text
ads6-sam-1  cadac.ads6.sam
```

The object reports the selected source phase and actual T3/T4 fidelity on every sample.

## Sensor and persistent-session boundary

The standalone SAM model owns a persistent physical plant, but its RF/IR
controller cannot be promoted as a standalone native-sensor session: it
requires live target truth, RADAR0 intercept data, launch timing, and
vehicle-major packet epochs supplied by `cadac.ads6.engagement`. Its use of the
typed Taoryx `relative-state-track` is therefore a source-module integration,
not a separately advertised `SensorBus` delivery session.

`cadac.ads6.sam` remains a caller-controlled direct-command plant with clear
requested/achieved outputs. The package engagement is the only valid owner of
the promoted source-controller session and its native raw relative-state
packets; a wrapper around this standalone plant would omit necessary source
state.

## Evidence boundary

The standalone plant does not claim ADS6 radar, seeker, guidance, INS, source-event scheduling, SRBM/aircraft propagation, full engagement behavior, or compiled-CADAC numerical parity. The package model now supplies a separately evidenced deterministic source-controller layer; that package capability does not broaden the standalone plant claim.
