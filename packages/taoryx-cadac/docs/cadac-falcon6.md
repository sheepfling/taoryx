# CADAC FALCON6 physical-plant plug-in

`cadac.falcon6.aircraft` is the exact runnable FALCON6 rigid-body physical
plant. It is intentionally a direct-surface realization: source waypoint
guidance and autopilot command generation are outside this plug-in boundary.

## Fidelity and result composition

The one available realization is
`rigid_body_6dof_surface_allocated`. Each batch returns one independent root:

```text
aircraft1  cadac.falcon6.aircraft
```

Core truth includes local-NED position and velocity, scalar-first attitude
quaternion, and body rates. It is a physical 6-DoF claim, not a reduced-order
or source-history replay.

## Direct control contract

The caller owns three fixed batch controls at the source actuator boundary:

| Action | Configuration seam | Requested/realized evidence |
| --- | --- | --- |
| `actuator.aileron.deflection` | `physical_controls.aileron_command_deg` | component 0 of `requested_surfaces_deg` / `achieved_surfaces_deg` |
| `actuator.elevator.deflection` | `physical_controls.elevator_command_deg` | component 1 of `requested_surfaces_deg` / `achieved_surfaces_deg` |
| `actuator.rudder.deflection` | `physical_controls.rudder_command_deg` | component 2 of `requested_surfaces_deg` / `achieved_surfaces_deg` |

All coordinates use degrees and are `available_in_batch` only: a submitted
command is held for that batch, not treated as a streaming or session action.
`aero_surfaces_deg` records the source-epoch positions consumed by the
aerodynamics module. `surface_position_limited` and `surface_rate_limited`
publish the corresponding boolean aileron/elevator/rudder actuator feedback,
so an agent or UI can distinguish a requested command from an achieved,
limited, or delayed physical response.

## Analysis and execution boundary

Requested/achieved surface traces support finite-run command/response analysis
and like-for-like controller comparison. Local linear stability and frequency
margins remain blocked until a declared operating point and complete closed-loop
state model are published; the existence of actuator dynamics is not a
stability claim.

FALCON6 is batch-only. It does not expose a persistent Mission Composition
session, and `open_session` rejects it instead of resetting a batch run between
pseudo-steps. It has no participating native sensor or `SensorBus` path.

The executable profile is `cadac_compat`: source atmosphere and gravity remain
inside the compatibility runtime for source-parity work. They are not a second
Taoryx environment service. A future native session would need to inject the
shared host environment before this retained source path could be removed.

## Evidence boundary

The plug-in does not claim source autopilot/waypoint guidance, a general
streaming controller, a native sensor, or compiled-CADAC numerical parity.
Exact dispatch cannot fall back to another actor, realization, or fidelity.
