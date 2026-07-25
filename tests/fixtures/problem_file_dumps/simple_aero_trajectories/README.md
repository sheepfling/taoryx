# Simple Aero Trajectory Workspace

This leaf is the full Simple Aero-style trajectory working set for the
simple-aero bundle.

Every case now shares the same launch spot, climbs through a boost and
coast-to-apogee phase, runs a maneuver segment, then hands off to a terminal
propnav segment about 20 km out. Negative altitude stops are explicit in the
segment bodies so the fixture set behaves like a bounded early-termination
test.

The per-case structured `simple_aero_metadata` blocks retain the solution-specific
knobs from the Simple Aero context dump, including left/right direction, maneuver
thresholds, and the range-to-go or time-to-go values that drive the handoffs.

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
- `propnav.prb` for a two-trajectory intercept example with a common target

The checked-in `.prb` files are generated from
[`spec.yaml`](spec.yaml). The workspace is intentionally synthetic and is meant
to be a compact, inspectable trajectory set rather than a full Simple Aero runtime
reconstruction.
