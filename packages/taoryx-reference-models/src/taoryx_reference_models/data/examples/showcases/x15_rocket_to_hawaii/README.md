# X-15 on a native booster toward Hawaii

This is a standard TAORYX `.prb` migration showcase. It models one rigid-body
vehicle through three explicit segments:

1. a short powered booster phase;
2. booster coast to a timed apogee/release boundary;
3. an X-15 research-surrogate configuration with six-axis tables and native
   ProNav attitude steering toward a Hawaii target.

The segment-3 `*reset` is the source-language staging seam: the booster mass
mass is reset to the X-15 research-surrogate mass before its tables become
active. The runtime applies that reset during the native segment transition;
no Python flight equations are used by the showcase. The current rigid-body
reset contract does not yet expose a separate propellant-state assignment;
that is tracked as a follow-up rather than being smuggled into the problem
file. Booster cutoff and X-15 release now have altitude-triggered `*when`
conditions with time fallbacks, so the pre-release profile is visible in the
problem file rather than hidden in a custom runner.

The result is an attempt, not an engineering performance claim. The supplied
X-15 public research deck is a beta surrogate bounded by Mach 6.7, 80,000 ft,
and its declared angle domains, including a ±10° sideslip grid. The problem
uses `no-extrap` tables, so an out-of-envelope trajectory is reported as a
failed evidence case rather than silently extrapolated. The mission explicitly
delays route steering until segment 3 through the segment's standard `*fly`
block. Controller equations remain available in the TAORYX rigid-body backend,
but are not encoded as X-15-specific problem-file directives. The current
regression records an out-of-envelope failure instead of claiming a Hawaii
intercept. A successful intercept would require a vehicle and propulsion deck
with enough range and a valid high-altitude envelope.

The long bank-reversal evidence case uses `dt=0.0125 s`. This is a declared
numerical requirement for its high-rate attitude transitions: at the coarser
`0.025 s` step, independent inertial force-closure p99 exceeded the release
gate. At the selected step the same continuous, unpowered, event-terminated
trajectory passes the translation-closure gate; this is a convergence choice,
not a relaxation of the physics or an unreported state reset.

The focused Simple Aero-derived X-15 additions are `x15_phugoid_3dof.prb` and
`x15_weave_6dof.prb`. The first is a bounded source-deck alpha/energy profile;
the second performs two native positive/negative bank reversals before a
neutral glide. They are promotion fixtures, not natural-mode, crossrange-
optimization, or flight-performance claims.

Run with:

```bash
PYTHONPATH=src python -m taoryx.cli run \
  examples/showcases/x15_rocket_to_hawaii/mission.prb \
  tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl \
  --output-dir artifacts/x15-rocket-to-hawaii
```
