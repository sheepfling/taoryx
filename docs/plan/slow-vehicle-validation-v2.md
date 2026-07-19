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
- Hummingbird has preliminary hover evidence, but not yet force/moment closure
  or disturbed-response evidence.
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

The Hummingbird entry is explicitly blocked. The current standard runtime
contract now exposes a generic four-rotor allocation and rate-damping path;
the first closed-loop rate-response case is covered by
`SV05_rate_damped_hover_6dof.prb`. Full position/waypoint control remains
blocked until attitude-command and actuator-state coverage is added.

The B747 controller probe has been extended to ten seconds with generic
alpha-hold feedback. The X8 control and rate-effect grids are now imported as
verified `.tbl` data. The X8 now has a separate five-second
`SV03_lateral_rate_response_6dof.prb` gate using differential elevon,
restoring sideslip, and body-rate damping; it remains within the local source
envelope without saturation. The 0.5-second collective-elevon longitudinal
recovery remains the baseline, and a longer coupled recovery is still blocked
until the two independently bounded responses are combined and re-verified.
