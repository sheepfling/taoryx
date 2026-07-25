# Showcase catalog

Showcases are explanatory demonstrations built from normalized run artifacts.
They are not substitutes for requirement-level tests. Each showcase has a
source manifest, expected invariants, plot specification, and explicit claim
boundary.

## Planned ladder

1. `6dof_stage_coast_entry` — two-stage launch, separation, coast, apogee,
   thermal entry, and phase timeline.
2. `6dof_attitude_forces` — body axes, quaternions, angular rates, moments,
   and force decomposition.
3. `6dof_terminal_pronav` — target track, line of sight, guidance command,
   and terminal miss distance.
4. `mode_comparison` — point-mass, kinematic 3+3, and rigid-body 6-DOF on one
   controlled reference case.
5. `verification_sensitivity` — step-size, parameter, and seeded-mutation
   sensitivity views.

Each showcase should emit the same normalized artifact in text, JSON, SQLite,
and plot views. The plot layer consumes `RunArtifact`; it must not parse
printed report text.

## Four-Family Flight Showcase v1

The Alpha 2 flagship layer is defined by
[`Four-Family Flagship Flight Scenarios`](../plan/four-family-flagship-flight-showcase-v1.md).
It is a qualification layer, not a replacement for plant, actuator,
convergence, or envelope tests. The four flagship packs are:

| Family | Mission | Family-appropriate terminal |
| --- | --- | --- |
| AscTec Hummingbird | Pad takeoff, hover, 3D waypoint box, yaw, disturbance recovery, landing | Touchdown and motor disarm |
| Skywalker X8 | Launch/release, bidirectional fixed-wing course, speed/altitude changes, recovery | Recovery gate; landing only when ground physics qualify |
| Boeing 747-100 | Airborne trim, climb, turns, speed changes, descent, arrival capture | Stabilized arrival gate; runway lifecycle is separate |
| X-15 | Air release, powered climb, burnout/coast, bank reversals, energy management | Recovery-energy or aim-region corridor |

The public badge is `Flagship Mission Qualified`. Every pack must include
control coverage, waypoint/terminal evaluation, segment timeline, envelope and
convergence reports, and deterministic reproduction instructions.

The planned expansion adds:

```text
hummingbird_fleet_split_merge_return_p6dof
reference_satellite_deploy_detumble_nadir_pass_6dof
surrogate_tandem_rotor_route_land_p6dof
surrogate_tube_launch_loiter_3dof
surrogate_twin_jet_vtol_dash_return_p6dof
```

Fleet and constellation scenarios require scalar-versus-batched equivalence,
sparse-event determinism, separation/communication metrics, sampled full
traces, wall-clock and memory measurements, and no ordinary all-pairs
interaction path.

Reachability products are maintained separately from showcase flight packets.
See the [Trajectory Reachability Workbench plan](../plan/trajectory-reachability-workbench.md)
for effective kinematic envelopes, terminal footprints, launch-state
acceptability, checkpoint branching, adaptive boundaries, witness trajectories,
and R0–R6 workbench maturity.

The first showcase is currently scaffolded because the stage-specific
propulsion, atmospheric drag/heating, and integrated terminal-guidance oracle
are still being built.
