# Slower-vehicle validation tranche v2

The current slower-vehicle bundle is a transparent smoke/evidence bundle, not
an engineering-validity release. The next milestone is one reproducible,
unit-consistent, in-envelope validation case per family.

## Gates

1. Artifact integrity: each case starts in a clean directory and records a
   run UUID, source/problem hashes, resolved-problem hashes, table hashes, and
   repository-relative paths.
2. Initial-state parity: the scenario is defined in SI; legacy 3-DOF inputs
   are converted through their native FPS boundary and back to canonical SI.
   Position, velocity, mass, Mach, dynamic pressure, alpha, beta, and
   quaternion norm are audited before integration.
3. Table-query audit: the exact runtime query binding is evaluated at t=0,
   with named axes, units, and positive boundary margins. No clamp or
   extrapolation is allowed.
4. Trim/hover: B747 uses an exact NASA anchor, X8 uses the published trim
   neighborhood, and Hummingbird solves rotor hover with the runtime wrench
   evaluator.
5. Closure: export separate aerodynamic, propulsion, gravity, and total
   force/moment channels plus absolute and normalized translational and
   rotational closure residuals.
6. Meaningful convergence: compare at least three step sizes over a ten-second
   run, including a disturbed Hummingbird recovery case.

## Current classification

- B747 and X8 3-DOF traces are baselines until they share the 6-DOF model,
  atmosphere, propulsion, mass, controls, and canonical SI initial state.
- B747 and X8 6-DOF cases must fail closed at table-envelope violations; the
  matrix must distinguish preflight rejection from an initial table-query
  rejection and runtime envelope exit.
- Hummingbird has bounded hover, disturbed-response, waypoint-course, and
  return-home/landing evidence with direct force/moment closure and timestep
  convergence. The common controller-manifest case covers the standard
  return-home problem file; it remains a research-surrogate claim, not a
  hardware or flight-control claim.
- CA-HI remains evidence-only and is not part of this family-validation gate.

The goal is a small, reviewable evidence set rather than a larger matrix of
non-comparable or stale runs.

## Controller tranche

`verification/controller_scenarios.yaml` is the controller-layer manifest.
The B747 and X8 entries use ordinary rigid-body problem files with the native
`*fly propnav` route-attitude controller, declared guidance gains, actuator
limits, and the same source-anchored tables used by the plant-golden tests.
They are bounded response probes, not route-completion claims: the fixed-wing
tables are local models and the controller must remain inside their envelope.

The generic Hummingbird controller-contract entry remains explicitly blocked:
the standard runtime exposes four-rotor allocation and rate damping, but a
fully general external attitude/actuator command interface is not yet part of
the canonical vehicle contract. The concrete native waypoint and return-home
cases are covered separately by the common manifest through
`SV05_return_home_land_6dof.prb`, with direct equation residual and shutdown
ordering gates.

The B747 controller probe has been extended to ten seconds with generic
alpha-hold feedback. The X8 control and rate-effect grids are now imported as
verified `.tbl` data. The X8 now has a metadata-generated ten-second powered
longitudinal recovery using the composed static, collective-elevon,
differential-elevon, and thrust tables. Its enlarged moment allowance is
explicitly notional and supports a bounded research-controller test rather
than a published control-law claim. The X8 also has a separate five-second
`SV03_lateral_rate_response_6dof.prb` gate using differential elevon,
restoring sideslip, and body-rate damping; it remains within the local source
envelope without saturation. The older 0.5-second collective-elevon file is
retained as a historical diagnostic; it is no longer the active controller
scenario manifest entry.

The opt-in X8 surface-authority route is intentionally a failing diagnostic at
this stage. Its pitch response leaves the source alpha envelope before a
multi-second recovery can be claimed. Aerodynamic `no-extrap` queries now
reject at that boundary instead of clamping and allowing rotational overflow;
the next tuning work must therefore repair the surface controller at this
lowest failing fidelity before the route is promoted to evidence. The source
authority gate in `tests/unit/test_runtime_table_binding.py` records the
finite local collective/differential control derivatives and their signs used
to guide that repair. The generic `surface-control-inversion=true` extension
now allocates moment demand through those measured derivatives, but its
120-second rectangle candidate still exits the source beta envelope and is
not promoted to the evidence packet.
