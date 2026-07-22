# B747 and Skywalker X8 verification plan

This is the focused execution plan for the two powered fixed-wing research
surrogates. It supports claims about the declared TAORYX models and their
source-bounded envelopes. It does not establish flight qualification,
historical TAOS compatibility, or a globally valid aircraft model.

## Current evidence

| Gate | B747 | Skywalker X8 | Evidence boundary |
| --- | --- | --- | --- |
| Source table transcription | pass | pass | 25 B747 condition-3 states plus 125 elevator states; 840 X8 zero-control states plus 5,880 collective and 5,880 differential states |
| Registry/table provenance | pass | pass | canonical registry and scoped problem audit |
| Convention firewall | pass locally | pass locally | units, frames, quaternion, margins, closure at initial state |
| Source-anchored local plant | pass | pass locally | B747 local trim; X8 source-composed powered trim with all three body moments closed |
| Long open-loop mechanics | pass locally | pass locally | B747 60 s trim plus 60 s approach/go-around; X8 60 s powered recovery plus a generated 120 s level-settling corridor and 120 s four-leg route |
| Controller recovery | partial | partial | long cases are bounded; explicit route tracking-error and settling metrics remain |
| 3-DOF reduction parity | short-phase pass | short-phase pass | Both reductions share verified source decks; 100 ms phase-history parity passes within explicit free-attitude bands, while longer free-attitude equivalence remains intentionally out of scope |
| Independent closure | pass locally | pass at declared discretization gate | Translation and rotation use independent telemetry finite differences. B747 long trim passes p99 < 1e-4; X8 powered validation passes p99 < 5e-3 at dt=5 ms. |
| Reproducible evidence packet | pass | pass | UUID-scoped packet builder copies inputs, records 13 declared B747/X8 cases including the source-composed X8 trim, hashes packet files, and emits standard plots |

## Exit criteria

The B747 and X8 tranche is complete only when each vehicle has all of the
following:

1. Source-differential report with source/table hashes, sample count, maximum
   absolute and relative error, and worst-case input state.
2. Source-anchored powered trim report using the same evaluator used for
   propagation, with six force/moment residual components and no table
   extrapolation.
3. Independent force and moment closure calculated from saved state,
   environment, and load channels rather than only from the integrator RHS.
   Independent translational and rotational diagnostics are implemented. B747
   source-anchor holds use a 1e-4 p99 gate; the generated B747 20 ms
   descent-recovery maneuver uses a separately reported 1e-3 controlled
   finite-difference gate; X8 uses a 5e-3 p99 gate justified by its 5 ms
   controlled finite-difference discretization.
4. Long nominal scenario with explicit phases:
   trim hold, climb or altitude step, heading/bank maneuver, cruise/recovery,
   and descent or go-around.
5. Bounded disturbance cases for initial speed/altitude, wind, mass or trim,
   and control-surface perturbations.
6. RK4 `dt`, `dt/2`, and `dt/4` convergence plus an adaptive reference or a
   documented reason it is unavailable.
7. A physically meaningful 3-DOF reduction using the same atmosphere, source
   tables, propulsion, mass, controls, and initial state as the 6-DOF case.
8. Initial-derivative parity followed by phase-windowed 3-DOF/6-DOF history
   comparisons under constrained assumptions. The current common window is
   100 ms; longer equality is not expected while 6-DOF attitude is free. The
   acceptance bands are explicit discretization/free-attitude bands, not an
   assertion that the unconstrained trajectories are identical.
9. Standard plots: altitude, ground track, airspeed/Mach, alpha/beta/bank,
   rates, forces/moments, propulsion/fuel, controls, tracking error, and
   table margins.
10. UUID-scoped manifest containing all resolved inputs and output hashes. The
   reproducible packet command is `python tools/dev.py b747-x8-evidence`; its
   manifest is the trust root and is intentionally excluded from its own hash
   map to avoid recursive self-reference.

The first reduction gate is now explicit for both fixed-wing vehicles: the
point-mass case and rigid-body case query the same SI source tables at the same
initial speed, altitude,
alpha, beta, controls, and throttle.  The comparison also verifies the
TAOS-canonical-English-unit to SI table boundary.  A direct propulsion table
with a `throttle` independent variable is treated as already throttle-scaled;
literal thrust expressions remain scaled by the active throttle command.
This prevents the common double-throttle error while retaining the historical
`*prop thrust=...` behavior for scalar commands.

## Vehicle-specific acceptance

### B747-100 research surrogate

Use the NASA CR-2144 condition-3 local model. The primary plant case is a
source-anchored trim hold followed by small elevator and heading/bank
disturbances. The local model must remain inside its Mach, alpha, beta, and
forward-speed limits.

Required mechanics:

- level trim holds airspeed and altitude within declared bands;
- elevator perturbation produces a bounded pitch response and recovery;
- heading change produces bank and route displacement without unbounded beta;
- descent/go-around phases occur in declared order;
- mass remains constant unless a separately declared propulsion/mass model is
  active; the notional fuel table cannot support a source-backed fuel claim.

### Skywalker X8 research surrogate

Use the published flight-identified trim neighborhood and keep the claim local
to the supplied alpha, beta, speed, altitude, rate, and control envelope. The
powered case must explicitly identify whether it uses the published propeller
model or the simplified calibrated thrust model.

The generated `SV03_source_trim_hold_30_6dof.prb` is the current source-trim
anchor. It composes the static, collective-elevon, and differential-elevon
tables at the normal runtime boundary and uses the solved controls
(`alpha=7.8095 deg`, `throttle=0.57734`, `collective=-2.14789 deg`,
`differential=-3.93780 deg`). Its initial normalized body-moment residuals
are below `6.6e-4`; the 30-second guided hold is bounded but remains a
research-surrogate recovery case, not a globally stable trim claim.

Required mechanics:

- powered trim closes with gravity, atmosphere, propulsion, actuators, and all
  six aerodynamic channels active;
- the long powered validation uses explicitly declared step-compatible inner
  loop gains (`attitude-gain=100`, `rate-damping=5`) for its 5 ms RK4 step;
  the previous implicit 2,000 rate-damping default was numerically stiff and
  failed independent rotational closure;
- collective elevon produces a bounded longitudinal response;
- differential elevon produces a bounded lateral response with the declared
  sideslip sign;
- a coupled recovery remains in-envelope and unsaturated;
- the generated 120 s level-settling corridor remains bounded in altitude,
  speed, alpha, beta, mass, and independent closure; it is a local settling
  corridor rather than a global altitude-hold claim;
- a rounded rectangle or waypoint route completes with leg direction and
  altitude/speed bands satisfied. The current 120 s rectangle is a directional
  route demonstration only: independently reconstructed corner misses are
  approximately 110 m, 318 m, 415 m, and 229 m at the four phase boundaries,
  so it is not yet a waypoint-capture success.
- the supporting X8 waypoint and approach files now explicitly activate
  native `*fly propnav=4` guidance and the basic waypoint declares its
  `linear-target` altitude profile. The basic waypoint now stops on its native
  `range_to_target_m<25` event at approximately 16.9 s; this is a single
  waypoint-capture result, not evidence for the still-blocked four-leg route.
- route telemetry records error to each fixed corner
  (`route_corner_0_error_m` through `route_corner_3_error_m`) as well as the
  active moving target; those channels are the evidence source for the blocked
  waypoint-capture claim. The packet records the fixed 25 m capture gate and
  emits an explicit `capture_status` (`pass` or `blocked`) from the four
  phase-boundary errors; a bounded route run cannot silently become a
  waypoint-capture result.
- crosswind and initial-condition perturbations either remain inside the
  declared disturbance corridor or fail with a named envelope/authority
  diagnostic.

## Execution order

1. Keep source-differential reports and table provenance as release inputs.
2. Make the common test harness load vehicle/table bindings from the canonical
   registry/catalog rather than repeating paths in each test module.
3. Add the independent closure calculation and expose component residuals.
   Complete for the B747 trim and X8 powered anchor; the same diagnostics must
   still be applied to route, disturbance, and recovery phases.
4. Re-run B747 trim, X8 powered trim, and their convergence studies. The X8
   convergence evidence uses a declared 10-second source-composed trim window;
   its longer 30/60/120-second cases remain separate bounded-trajectory tests.
   The generated B747 descent-recovery case supplies the first explicit long
   controller phase sequence: 60 seconds descending, followed by 60 seconds
   of level recovery toward a 520 m target.
5. Run the long B747 and X8 nominal maneuvers and generate the standard plots.
6. Run bounded disturbances and controller recovery cases.
7. Extend the X8 initial-query parity gate to derivative and phase-window
   history parity under prescribed-attitude assumptions.
8. Build one evidence packet per vehicle and a combined review index.
9. Update `verification/claims.md` only with the narrow claims supported by
   the packet.

## Stop-ship conditions

- source and runtime table hashes are absent or disagree;
- a test binds a table not named by the vehicle registry/catalog;
- a fixed-wing 3-DOF case uses the former `cl=0`, `cd=0` placeholder;
- the legacy rectangle/climb showcase is mistaken for the source-deck
  reduction; use `SV01_powered_trim_reduction_3dof.prb` for B747 parity;
- closure is reported only from the same derivative path being tested;
- a nominal run leaves its local source envelope without an explicit blocked
  result;
- controller success is claimed from a short response probe alone;
- directional route completion is presented as waypoint capture without
  explicit corner-error telemetry and a declared miss tolerance;
- B747 fuel burn is described as source-backed while using the notional table;
- generated or source-preserved problem provenance is ambiguous.
