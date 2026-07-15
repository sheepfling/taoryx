# X-15 on a native booster toward Hawaii

This is a standard TAORYX `.prb` migration showcase. It models one rigid-body
vehicle through three explicit segments:

1. a short powered booster phase;
2. booster coast to a timed apogee/release boundary;
3. an X-15 research-surrogate configuration with six-axis tables and native
   ProNav attitude steering toward a Hawaii target.

The segment-3 `*reset` is the source-language staging seam: the booster mass
and propellant are removed before the X-15 tables become active. The runtime
applies that reset during the native segment transition; no Python flight
equations are used by the showcase.

The result is an attempt, not an engineering performance claim. The supplied
X-15 public research deck is a beta surrogate bounded by Mach 6.7, 80,000 ft,
and its declared angle domains. The problem uses `no-extrap` tables, so an
out-of-envelope trajectory is reported as a failed evidence case rather than
silently extrapolated. A successful Hawaii intercept would require a vehicle
and propulsion deck with enough range and a valid high-altitude envelope.

Run with:

```bash
PYTHONPATH=src python -m taoryx.cli run \
  examples/showcases/x15_rocket_to_hawaii/mission.prb \
  tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl \
  --output-dir artifacts/x15-rocket-to-hawaii
```
