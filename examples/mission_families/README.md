# Canonical mission-family suite

This directory is the successor-side scenario catalog. Each family has a
manifest entry in [`mission_families.yaml`](mission_families.yaml) describing
its current evidence level, dynamics mode, and acceptance oracle.

The catalog deliberately distinguishes three states:

- `verified`: the source mission runs and has independent assertions;
- `scaffolded`: the source shape exists, but runtime or physics coverage is
  incomplete;
- `planned`: the family needs a new fixture and/or runtime capability.

These are TAORYX scenario families, not claims that historical TAOS 96.0
implemented the same vehicle models. The historical TAOS boundary remains the
manual-documented three-degree-of-freedom point-mass simulation unless a source
record explicitly says otherwise.

The first family is the synthetic two-stage point-mass example under
`vehicle_families/staged_rocket/two_stage_demo`. The requested mission families
now have source packs or executable controller fixtures: lifting glide,
ballistic/tumble return, wind-aware terminal guidance, changing-atmosphere
rotorcraft loiter, and a rigid-body 6-DOF staged reentry extension. They remain
`scaffolded` until their full source-to-runtime contracts and independent
numerical oracles are complete.
