# Long validation trajectories v1

The short golden cases are useful convention and plant smoke tests. They are
not sufficient evidence of sustained vehicle behavior. This plan replaces the
generic “run for a few seconds” standard with vehicle-specific trajectories
whose phases, expected trends, and quantitative exit criteria are explicit.

The target claim is not flight qualification. It is:

> TAORYX reproduces the declared research-surrogate vehicle semantics over a
> sustained, source-bounded trajectory, with independently checkable trends,
> equation closure, and numerical convergence.

Each vehicle gets a different validation contract. A fixed-wing aircraft,
multirotor, and unpowered hypersonic glider must not share one generic notion
of a successful flight.

## Common release gates

A long trajectory cannot be called validated unless all of these are present:

1. A source-anchored initial condition and an initial-condition audit.
2. A phase/event timeline recorded from the native problem file.
3. An independent expected-trend oracle that does not call the production RHS.
4. Complete force, moment, mass, actuator, and table-query telemetry.
5. No unexplained table extrapolation, invalid state, or safety termination.
6. `dt`, `dt/2`, `dt/4`, and an adaptive reference comparison.
7. Equation-closure p99 below the declared tolerance away from events.
8. No actuator saturation in the nominal case unless the scenario explicitly
   tests saturation.
9. A reproducible manifest containing problem, table, controller, and runtime
   hashes.
10. Human-readable plots plus machine-readable metrics.

The short case remains a prerequisite, but it no longer counts as the long
trajectory itself.

## Vehicle contracts

### B747: powered fixed-wing transport

Semantics:

- six-axis rigid-body aircraft with powered propulsion;
- local NASA handling-qualities derivative model;
- aerodynamic forces depend on trim perturbations, rates, controls, Mach, and
  altitude validity;
- mass is constant for the current surrogate;
- attitude and rates must respond to control inputs while airspeed and altitude
  remain regulated.

Required sequence, nominal case:

The first completed long plant evidence is the generated 120-second source-
anchor trim hold. It remains near 100 m and 153 m/s with alpha approximately
2.675° and beta zero. A generated 120-second descent/recovery case now adds a
native controller phase sequence: 60 seconds at approximately -3° flight path,
then 60 seconds recovering toward a 520 m level target. The full 480-second
maneuver sequence below remains a future controller target; it is not implied
by either case.

| Phase | Duration | Expected mechanics |
|---|---:|---|
| Source-anchor trim hold | 0–120 s | Airspeed, altitude, alpha, beta, and rates remain bounded and nearly steady. |
| Elevator doublet | 120–150 s | Pitch and alpha respond, altitude changes after the short-period response, then recover. |
| Altitude step | 150–240 s | Altitude approaches a new level without leaving the local alpha/beta deck. |
| Heading change | 240–330 s | Bank and heading change; sideslip returns near zero; airspeed remains controlled. |
| Wind rejection | 330–420 s | Track error grows during the gust and decays after removal. |
| Return to trim | 420–480 s | State returns to the source neighborhood without saturation. |

Success gates:

- airspeed error after recovery: ≤ 2% of trim speed;
- altitude steady-state error: ≤ 10 m;
- heading steady-state error: ≤ 2°;
- `|alpha| ≤ 4°`, `|beta| ≤ 5°` for the entire nominal case;
- no table extrapolation or rate-limit saturation;
- translational and rotational closure p99 ≤ 1e-6 normalized away from
  command discontinuities;
- final state difference between `dt/2` and `dt/4` below 1% of each declared
  maneuver tolerance.

Expected plots:

- altitude and commanded altitude versus time;
- airspeed, Mach, dynamic pressure;
- local roll, pitch, heading and body rates;
- alpha, beta, load factor;
- thrust, drag, lift, side force and all three moments;
- controls, limits, saturation, table margins;
- phase/event timeline.

### Skywalker X8: powered low-speed fixed-wing aircraft

Semantics:

- powered six-axis fixed-wing aircraft with elevon actuation;
- flight-identified model valid only in a local alpha/beta/speed region;
- propulsion and aerodynamic controls are coupled;
- airspeed must remain above the stall-unknown lower validity boundary;
- the published trim neighborhood and the simplified propulsion trim are
  separate claims.

Required sequence, nominal case:

The first completed long plant evidence is the generated 120-second level-
settling corridor. It remains inside 150–190 m altitude, 15–21 m/s speed,
0–12° alpha, and ±5° beta. This is a bounded local settling result, not yet
the full climb/reversal/rectangle/gust sequence below.

| Phase | Duration | Expected mechanics |
|---|---:|---|
| Powered trim hold | 0–60 s | Speed, alpha, beta, and altitude remain near the solved trim. |
| Climb and level-off | 60–150 s | Collective elevon produces climb; throttle and pitch settle at the new altitude. |
| Heading reversal | 150–240 s | Differential elevon produces a bounded turn; beta decays after capture. |
| Wide rectangle | 240–420 s | Four legs complete with turn windows sized to the aircraft speed and bank authority. |
| Gust rejection | 420–480 s | Cross-track and beta excursions remain bounded and recover. |
| Trim return | 480–540 s | Controls and state return to the trim neighborhood. |

Success gates:

- nominal speed remains in the declared 12–27 m/s source region;
- `0° ≤ alpha ≤ 12°`, `|beta| ≤ 5°`;
- altitude error after each level-off: ≤ 5 m;
- rectangle corner miss: ≤ 10 m in the point-mass reduction and ≤ 25 m in
  6-DOF;
- beta returns below 1° within 15 s after each turn or gust;
- throttle/elevon commands remain unsaturated in the nominal case;
- powered propulsion is active and contributes to the force balance;
- convergence and closure gates match the B747 contract, with the X8-specific
  actuator time scale included in the step selection.

The published powered trim and any simplified-engine trim must generate
separate reports and must never be merged into one “X8 validated” label.

### AscTec Hummingbird: multirotor hover and translation

Semantics:

- individual-rotor thrust and reaction torque;
- source-native rotor/world frame mapped through an explicit convention
  adapter;
- motor lag and allocation are active;
- hover is an equilibrium problem, not a fixed-wing lift/drag problem;
- alpha and beta are unavailable below a minimum airspeed and must not be
  interpreted as physical angles during hover.

Required sequence, nominal case:

| Phase | Duration | Expected mechanics |
|---|---:|---|
| Takeoff | 0–8 s | Collective thrust raises the vehicle to 2 m without attitude drift. |
| Hover hold | 8–38 s | Position, yaw, rates, and vertical speed remain bounded. |
| North step | 38–58 s | Vehicle translates north, brakes, and settles. |
| East step | 58–78 s | Vehicle translates east, brakes, and settles. |
| Yaw step | 78–93 s | Yaw changes with near-zero position drift. |
| Return square | 93–123 s | Vehicle returns to the origin and 2 m altitude. |
| Descent checkpoint | 123–135 s | Descends to a declared hover-safe height and stops before contact. |

Success gates:

- hover position error: ≤ 0.10 m;
- waypoint radius: ≤ 0.25 m;
- altitude error during hover: ≤ 0.10 m;
- yaw error after settling: ≤ 3°;
- body-rate steady-state error: ≤ 2°/s;
- no rotor command saturation in the nominal case;
- total thrust and weight agree within 2% during hover;
- individual-rotor allocation reproduces the requested wrench within 1%;
- wind case returns to the waypoint corridor after gust removal;
- no alpha/beta pass criterion is applied below the airspeed validity floor.

Fault cases are separate from nominal validation. A degraded rotor case must
declare the fault magnitude, controller authority, allowable altitude loss,
and termination condition before it is run.

### X-15: unpowered hypersonic glide

Semantics:

- the vehicle is released from a booster or prescribed initial state;
- propulsion is active only before the declared release event;
- after release, acceleration comes from gravity and aerodynamic forces;
- energy is managed through alpha and bank, not through a powered fixed-wing
  throttle loop;
- the public deck is a bounded research surrogate, not a global reentry model;
- thermal output is unavailable unless a certified thermal model is supplied.

The first long validation case must start inside the X-15 table envelope. It
must not use the California–Hawaii mission as the golden plant test.

Required sequence, nominal glide case:

| Phase | Duration/condition | Expected mechanics |
|---|---:|---|
| Release initialization | event | Mass, position, velocity, attitude, and table coordinates match the release audit. |
| Ballistic climb/coast | to apogee | Propulsion is zero after release; altitude rises while vertical speed trends toward zero. |
| Atmospheric interface | declared altitude/Mach | Density and dynamic pressure become nonzero; aerodynamic forces oppose relative motion. |
| Energy-management glide | 120–300 s | Alpha/bank commands trade altitude and speed without table exit. |
| Heading capture | bounded range | Ground-track error decreases while bank remains within the declared authority. |
| Terminal corridor | declared altitude/speed | Capture a benign corridor and stop before any unsupported contact or thermal claim. |

Success gates:

- post-release propulsion force and mass flow are exactly zero;
- mass is constant after release;
- altitude has one clearly identified apogee and then decreases;
- vertical speed changes sign at apogee;
- dynamic pressure is near zero in the high-altitude coast and positive in the
  atmospheric segment;
- Mach and alpha remain inside the X-15 source deck;
- ground-track range decreases during terminal capture;
- no negative altitude, Earth-intersection, or table extrapolation;
- no thermal pass/fail claim is made while thermal policy is `none`;
- endpoint corridor tolerances are declared from the scenario, not chosen after
  seeing the result.

The current Hawaii showcase remains a mission integration experiment. It can
become a long validation trajectory only after this source-bounded glide case
passes independently.

## Independent oracles and evidence

The expected curves must not be generated by the same controller or RHS that
produces the simulation. Use:

- trim force/moment balance and dimensional analysis;
- analytic hover thrust/weight and rotor-wrench calculations;
- independent geodesic and local-NED route calculations;
- event-time and monotonicity checks;
- independently computed energy, dynamic-pressure, and mass-flow balances;
- a separate integrator or high-precision reference for convergence.

Every long run produces:

```text
run-manifest.json
scenario-resolved.prb
source-and-table-hashes.json
initial-condition-audit.json
event-timeline.json
closure-metrics.json
convergence-report.json
plots/
  overview.png
  altitude-and-route.png
  velocity-and-energy.png
  forces-and-moments.png
  attitude-and-controls.png
  table-margins-and-events.png
```

The acceptance report must show the expected trend and the measured trend
side-by-side. A trajectory that is finite but has the wrong altitude, energy,
mass, or route trend is a failure even when every number is numerical.

## Execution order

1. B747 480-second powered fixed-wing case.
2. X8 540-second powered fixed-wing case.
3. Hummingbird 135-second takeoff/hover/waypoint case.
4. X-15 300-second source-bounded unpowered glide case.
5. Wind/fault variants only after each nominal case passes.
6. California–Hawaii and any physical landing scenario only after the
   source-bounded cases pass and the missing model claims are addressed.

The completion flag for this plan is four long, independently checked,
vehicle-specific trajectories—not four short plots and not a single generic
“matrix pass.”

## Current execution ledger

The first implementation tranche deliberately records blockers instead of
promoting them to validation:

| Family | Runtime robustness | Behavioral evidence | Model-fidelity blocker |
|---|---|---|---|
| X-15 glider | 120 s bounded run, convergence, perturbations | Apogee and post-release energy contract pass; bank-reversal and source-anchored great-circle route telemetry contracts pass | Short-range native ProNav remains blocked by alpha/beta departure from the local deck; the unpowered case is diagnostic rather than a terminal-arrival claim |
| B747 | Local trim, bounded maneuver, opposite-elevator sign diagnostic, 60-second controlled descent corridor, 20-second composed route/altitude capture, and table-driven notional mass coupling pass | Source-backed fuel flow, longer route closure, and integrated mission semantics remain open | Route and flight-path guidance now compose in one native runtime path; the throttle-to-`mdot` table is a notional first cut, not a NASA JT9D fuel law |
| Skywalker X8 | 120 s directional rectangle and local response probes | Route directions, bounds, and fixed-crosswind corridor pass | After aligning the rectangle legs to the measured ground speed and adding a capped generic position-capture term, non-origin corner misses are approximately 54/87/124 m; waypoint capture is not established, and the first 120 s climb/level-off probe also exits the local coefficient envelope |
| Hummingbird | Declared direct-wrench hover/waypoint scope passes | Leave the existing verdict unchanged | The scope remains a synthetic research model, not flight qualification |

This ledger is part of the evidence boundary. A finite run, a small residual,
or a convergent integrator is a runtime result; it is not by itself evidence
that the vehicle performed the expected maneuver.
