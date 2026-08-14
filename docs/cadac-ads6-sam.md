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

## Control and feedback contract

All standalone controls are caller-owned, `available_in_batch`, and held fixed
for the submitted configuration. They are not step actions and are not a
streaming-control claim.

| Realization | Batch controls | Requested evidence | Realized plant evidence |
| --- | --- | --- | --- |
| `fin_control` | `actuator.roll.command`, `actuator.pitch.command`, `actuator.yaw.command` (deg) | `requested_control_deg` | `achieved_control_deg`, `requested_fins_deg`, `achieved_fins_deg` |
| `tvc_control` | `tvc.pitch.deflection`, `tvc.yaw.deflection` (deg) | `requested_tvc_pitch_yaw_deg` | `achieved_tvc_pitch_yaw_deg`, `tvc_effective_gain`, TVC limit flags |
| `aggregate_rcs` | attitude, incidence, lateral/normal-acceleration, and unit-direction RCS coordinates | `requested_rcs_attitude_deg`, `requested_rcs_incidence_deg`, `requested_rcs_acceleration_g`, `requested_thrust_vector_unit_body` | `rcs_moment_body_nm`, `incidence_deg`, `achieved_lateral_normal_acceleration_g`, `rcs_force_body_n` |

The action metadata identifies the exact configuration parameter and its
requested/realized evidence pair, so a UI, agent, or tuning tool can normalize
and inspect the boundary without guessing whether a command is an effector,
aggregate wrench, or a hidden source-controller input. The provider supports
finite-run command/response analysis and like-for-like controller comparison;
local linear stability and frequency margins remain blocked until a declared
operating point and complete closed-loop state model are published.

## Result composition

Mission Composition returns one independent root object:

```text
ads6-sam-1  cadac.ads6.sam
```

The object reports the selected source phase and actual T3/T4 fidelity on every sample.

## Sensor and persistent-session boundary

The standalone SAM model is intentionally batch-only. It does not expose a
persistent native plant/session and `open_session` rejects it rather than
resetting a batch run between pseudo-steps. Its RF/IR controller context
requires live target truth, RADAR0 intercept data, launch timing, and
vehicle-major packet epochs supplied by `cadac.ads6.engagement`. Its use of the
typed Taoryx `relative-state-track` is therefore a source-module integration,
not a separately advertised `SensorBus` delivery session.

`cadac.ads6.sam` remains a caller-controlled direct-command plant with clear
requested/achieved outputs. The package engagement is the only valid owner of
the promoted source-controller session and its native raw relative-state
packets; a wrapper around this standalone plant would omit necessary source
state.

The executable profile is `cadac_compat`: source atmosphere and gravity stay
inside the compatibility runtime for parity work. They are not advertised as a
second Taoryx environment service; a future `taoryx_native` session must inject
the shared host environment before this retained source path can be removed.

## Evidence boundary

The standalone plant does not claim ADS6 radar, seeker, guidance, INS, source-event scheduling, SRBM/aircraft propagation, full engagement behavior, or compiled-CADAC numerical parity. The package model now supplies a separately evidenced deterministic source-controller layer; that package capability does not broaden the standalone plant claim.
