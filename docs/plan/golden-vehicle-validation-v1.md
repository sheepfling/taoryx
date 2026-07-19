# Golden vehicle validation v1

This plan freezes expansion of the broad slower-vehicle trajectory matrix until
one source-anchored plant case is defensible for each vehicle family. The
purpose is to separate model fidelity from controller robustness and route
demonstrations.

## Release claims

Each golden case must distinguish three claims:

- **Real**: TAORYX reproduces the selected source model, including units,
  frames, coefficients, propulsion, mass properties, and controls.
- **Full**: all applicable gravity, atmosphere, wind, aerodynamics, propulsion,
  actuators, mass properties, controls, and events are active.
- **Valid**: closure, convergence, envelope compliance, controller metrics, and
  independent source comparisons pass.

No vehicle gets a trajectory or controller robustness claim until its plant
case passes the real and full gates.

## Golden cases

### B747

Use one NASA CR-2144 anchor condition, with FC6 as the first candidate:

- nominal clean configuration;
- 20,000 ft and Mach 0.65;
- actual alpha 2.50 degrees, beta 0, gamma 0;
- aerodynamic query alpha offset `alpha - alpha0 = 0`;
- throttle solved against source-model drag;
- trim must satisfy lift/weight, thrust/drag, and pitching-moment balance.

The first run is a 0--10 second controller-disabled trim hold, followed by
small elevator and bank/aileron perturbations. The stabilizer value in the
source record must not be treated as an absolute actuator command unless the
adapter explicitly models that convention.

### Skywalker X8

Use the published powered condition:

- 17.9 m/s at approximately 178 m;
- alpha 7.9 degrees and beta 1.2 degrees;
- collective elevon -2.35 degrees;
- differential elevon -2.16 degrees;
- throttle 0.44;
- gravity, atmosphere, propulsion, and actuator dynamics enabled.

The paper-powered model and the simplified engine model are separate claims.
The latter must not be called the published trim. The current static six-axis
table is an anchor-query fixture, not yet a powered trim plant, because its
control and propulsion tables are not bound into the runtime case.

### Hummingbird

Solve common rotor speed using the exact runtime wrench model, starting near
469 rad/s. Validate separately:

1. 30-second symmetric open-loop hover with identity attitude;
2. closed-loop takeoff, hover, position steps, yaw step, return, and landing;
3. additive wind, mass, inertia, CG, motor-time-constant, and rotor-coefficient
   perturbations.

RotorPy source frames require an explicit tested adapter. A fixed 180-degree
   attitude is not an acceptable substitute for that adapter.

## Canonical plant contract

All vehicle adapters must expose one SI evaluation containing state,
environment, aerodynamic query coordinates and margins, aero wrench,
propulsion wrench and mass flow, actuator command/achieved values, and mass
properties. The integrator consumes only canonical total body force, total body
moment, mass, and inertia.

## Evidence gates

For every golden case, produce:

1. source-differential samples across the declared envelope;
2. a bounded trim report with six residual components and solver diagnostics;
3. initial-condition and table-query audits;
4. force/moment decomposition and direct RHS closure;
5. RK4 `dt`, `dt/2`, and `dt/4` histories plus an adaptive reference;
6. source-validation comparison where source executable/evaluator evidence is
   available;
7. a UUID-scoped manifest hashing every input and output.

Research acceptance targets are translation residual p99 below `1e-8`
normalized, rotation residual p99 below `1e-8`, no nominal actuator saturation,
no table extrapolation, and a minimum table margin above ten percent of every
axis span. These are project gates, not certification claims.

## Current status

- `verify_golden_plant(...)` now emits the twelve-stage firewall report for
  arbitrary standard TAORYX 3DOF/6DOF problem files. Stages report `pass`,
  `fail`, or `blocked` with machine-readable evidence. `plant-golden` means
  stages 1--11 pass; `controller-ready` additionally requires stage 12.
  Missing convergence or controller configuration is therefore visible and
  cannot be mistaken for a successful plant claim.
- The four source-anchored plants now pass stages 1--11 and receive the
  `plant-golden` verdict. The convergence stage performs standard problem-file
  reruns at `dt`, `dt/2`, and `dt/4`; controller readiness remains a separate
  stage and is still blocked until controllers are explicitly tested.
- The first longer open-loop tranche is cataloged in
  `verification/long_vehicle_scenarios.yaml`: a 60-second B747 trim hold, a
  0.5-second X8 source-anchor propagation, and a 30-second Hummingbird hover.
  The X8 duration is intentionally bounded by its verified beta envelope;
  longer open-loop propagation is a controller/scenario task, not permission
  to extrapolate its research deck.
- The shared `tests/e2e/support/golden_plants.py` harness is now the common
  runtime boundary for golden-plant tests. It standardizes TAORYX profile
  selection, `.prb`/`.tbl` binding, immutable `time_s` telemetry snapshots,
  finite-value checks, active-aerodynamics checks, table margins, initial
  closure, and bounded channel assertions.
- The source-anchored catalog now includes the X-15 research surrogate beside
  the B747, Skywalker X8, and Hummingbird. Its standard problem exercises the
  generated X-15 six-axis and XLR-99 tables at Mach approximately 4.96.
- The convention firewall checks runtime mode, ECIC integration, ECFC
  environment metadata, quaternion normalization, positive mass, finite
  aerodynamic angles, and table margins before trajectory behavior is judged.
- The B747 condition-3 importer now filters the authoritative Mach 0.45
  slice; a source-grid differential test covers all 25 B747 six-axis states.
- A source-grid differential test covers all 840 zero-control X8 six-axis
  states. These are transcode claims, not source-executable claims.
- A bounded B747 condition-3 trim has been solved with the elevator control
  deck: alpha offset approximately -0.425 degrees, elevator approximately
  +0.386 degrees, and thrust approximately 179.54 kN. The initial normalized
  force/moment residual is below `1e-11`; the ten-second hold is currently a
  bounded research result, not yet the full convergence/source-validation
  gate.
- Hummingbird has bounded hover and disturbed-response evidence, but not yet
  the full closed-loop maneuver claim.
- B747 and X8 now have in-envelope anchor-query probes and closure telemetry.
- B747 now has a solved source-anchored condition-3 trim hold; its
  convergence and independent source-validation gates remain open.
- X8 still lacks runtime binding of the powered control and propulsion tables.
- California--Hawaii remains evidence-only and is not part of this gate.
- California--Hawaii remains a separate synthetic mission demonstrator; it is
  not a fifth source-anchored vehicle plant.

The broad robustness matrix remains useful as a failure detector, but it is
not a release gate until these three plant cases are green.
