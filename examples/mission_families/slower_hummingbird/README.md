# Hummingbird hover pair

`SV05_3dof.prb` establishes the point-mass route baseline. `SV05_6dof.prb`
uses the native rigid-body integrator and the source deck's dimensional rotor
wrench through the generic `aero-load-mode=direct-wrench` bridge. The source
model remains a tuned public research surrogate, not a hardware claim.

The long hover case is plant validation only. `SV05_basic_waypoint_6dof.prb`
adds a bounded native waypoint/altitude maneuver through the generic
rotorcraft allocation and attitude/collective controller. The maneuver is a
research-surrogate controller demonstration, not a hardware claim.
`SV05_rate_damped_hover_6dof.prb` remains the focused individual-rotor
allocation and rate-damping case.
