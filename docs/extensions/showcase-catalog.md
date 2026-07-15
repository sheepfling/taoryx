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

The first showcase is currently scaffolded because the stage-specific
propulsion, atmospheric drag/heating, and integrated terminal-guidance oracle
are still being built.
