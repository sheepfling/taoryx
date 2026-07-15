# 6-DOF stage, coast, and entry showcase

Status: scaffolded TAORYX extension.

This showcase will tell one coherent story: stage 1 powers the launch, stage
1 separates, stage 2 completes the ascent, the vehicle coasts through apogee,
and entry guidance trades angle of attack against thermal limits.

The current repository contains the rigid-body state, phase machine, and
thermal controller contracts needed by this showcase. The executable
launch-to-entry scenario is intentionally not represented as historical TAOS
syntax yet; the current `.prb` parser rejects `rigid-body-6dof` rather than
silently falling back to point-mass behavior.

## Evidence goal

The completed showcase must demonstrate:

- exactly one stage-1 separation;
- stage-2 burnout before apogee;
- a detected apogee event;
- entry thermal limiting;
- conserved and explicitly reported mass accounting;
- reproducible telemetry and plots from one run artifact.

See `showcase.yaml`, `expected.yaml`, and `plot.yaml` for the machine-readable
contract.
