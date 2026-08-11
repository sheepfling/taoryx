# CADAC ADS6 AIRCRAFT3 plug-in

`cadac.ads6.aircraft` exposes the ADS6 `AIRCRAFT3` target as a Taoryx Mission Composition batch vehicle.

## Fidelity

```text
point_mass_3dof / force_model
```

Only local-NED position and velocity are core truth. The source bank and normal-load variables are response states used to realize specific force; they are not rigid-body attitude or angular-rate state.

## Source modules

```text
environment → kinematics → guidance → control → forces → newton
```

The executable path supports source steady flight, strict-window horizontal g-turns, and cross-product escape guidance against explicitly supplied missile truth.

## Configuration

The typed schema exposes:

- initial NED position, speed, heading, flight path, and launch delay;
- source guidance option, gain, g-turn command, and maneuver window;
- bank/load response constants and source limits;
- longitudinal acceleration;
- optional external threat truth for standalone escape mode;
- end time and output cadence.

## Output

Core:

```text
position_ned_m
velocity_ned_mps
```

Telemetry includes atmosphere, phase, commanded acceleration, bank command/state/output, normal-load command/achievement, and specific force. Quaternion and body-rate outputs are intentionally absent.

## Exact dispatch

```python
registry.register(
    "cadac",
    "cadac.ads6.aircraft",
    provider.execute_batch,
)
```

No neighboring CADAC model or alternate fidelity is substituted when the exact executor is absent.

## Evidence boundary

The shipped straight-level and 1.5-g-turn source cases run to `50 s` at their original `0.001 s` step in the Python compatibility runtime. Their input bytes are checked against the pinned upstream Git blob IDs. Compiled-CADAC numerical parity and package-level radar/SAM orchestration remain open gates.
