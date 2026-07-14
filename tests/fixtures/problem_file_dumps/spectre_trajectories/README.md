# Spectre Trajectory Workspace

This leaf is the full Spectre-style trajectory working set for the
simple-aero bundle.

It covers every observed simple-aero maneuver family plus the two-trajectory
propnav intercept case:

- `ballistic.prb` for the boost/coast baseline
- `cbcr_left.prb` and `cbcr_right.prb` for both CBCR directions
- `crossrange.prb` for heading-error steering
- `marv.prb` for the MARV maneuver
- `phugoid.prb` for oscillatory maneuvering
- `range_extension.prb` for a passive range-extension case
- `skip.prb` for skip-flight steering
- `slalom.prb` for alternating bank guidance
- `weave.prb` for weave guidance
- `propnav.prb` for a two-trajectory intercept example

The checked-in `.prb` files are generated from
[`spec.yaml`](spec.yaml). The workspace is intentionally synthetic and is meant
to be a compact, inspectable trajectory set rather than a full Spectre runtime
reconstruction.
