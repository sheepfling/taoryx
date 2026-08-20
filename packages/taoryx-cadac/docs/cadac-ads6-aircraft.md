# CADAC ADS6 AIRCRAFT3 plug-in

`cadac.ads6.aircraft` is the exact runnable ADS6 `AIRCRAFT3` target actor. It
is independent of the ADS6 package composition: `cadac.ads6.engagement` owns
SAM/target/RADAR0 scheduling when that full source context is required.

## Fidelity and result composition

The actor is a `point_mass_3dof` force-model realization. Each batch returns
one independent root:

```text
ads6-aircraft-1  cadac.ads6.aircraft
```

Core truth is only local-NED position and velocity. Bank, load factor, and
specific force are source response telemetry; the plug-in never fabricates a
quaternion, body rates, moments, physical effectors, or a pseudo-6DoF claim.

## Source-program configuration

The source program owns control, so the realization advertises no caller action
channels. At composition/startup a caller can select and parameterize the
source behavior through the typed configuration schema:

| Source behavior | Configuration | Required seam |
| --- | --- | --- |
| steady flight | `guidance_option=0` | none |
| horizontal g-turn | `guidance_option=1`, `turn_load_g`, maneuver window, bank/load response parameters | none |
| threat escape | `guidance_option=2`, guidance gain, maneuver window | an enabled constant-velocity `threat_track` configuration |

The optional threat track is an explicit batch configuration seam, not a
native Taoryx sensor or hidden package dependency. Escape validation fails
closed if it is selected without a nonzero threat velocity.

## Control feedback and analysis

The standard output distinguishes source commands from plant response:

- `commanded_acceleration_ned_mps2` and
  `commanded_acceleration_velocity_mps2`;
- `commanded_bank_deg`, `bank_state_deg`, and achieved `bank_deg`;
- `commanded_load_factor_g`, achieved `normal_load_factor_g`, and limiter
  state; and
- `specific_force_body_mps2` plus source atmosphere/gravity diagnostics.

This makes finite-run source-controller analysis and like-for-like trace
comparison available. Local linear stability and frequency margins remain
blocked until a declared operating point and a complete closed-loop state model
are published.

## Batch, sensor, and environment boundary

AIRCRAFT3 is intentionally batch-only. It has no persistent Mission
Composition session and `open_session` rejects it rather than replaying a batch
from initial conditions. Its sensor integration is `not_applicable`; the
standalone threat configuration does not create a `SensorBus` stream.

The executable profile is `cadac_compat`: source atmosphere and gravity remain
inside the compatibility runtime for source-parity work. They are not a second
Taoryx environment service. A future native session would need to inject the
shared host environment before that retained source path could be removed.

## Evidence boundary

The plug-in does not claim package-level radar/SAM communication, source packet
scheduling, persistent streaming control, rigid-body attitude, physical
effector allocation, or compiled-CADAC numerical parity. Exact dispatch cannot
fall back to a different CADAC actor or fidelity tier.
