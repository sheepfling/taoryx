# 3-DOF to 6-DOF robustness verification plan

## Purpose

Establish the translational TAOS-compatible baseline before judging the native
TAORYX rigid-body controller. Every 6-DOF scenario must have a corresponding
3-DOF reference case with the same atmosphere, gravity, propulsion, guidance
intent, timing, and target geometry wherever the models support comparison.

This plan verifies bounded engineering behavior. It does not establish
historical TAOS 96.0 runtime compatibility or vehicle-specific flight
qualification.

## Execution order

### Tranche A — 3-DOF baseline

Use standard `.prb` constructs and verified `.tbl` fixtures to establish:

- ECIC/ECFC translational integration;
- gravity and atmosphere behavior;
- staged mass and thrust;
- alpha/bank or flight-condition guidance;
- ProNav demand and target geometry;
- wind-relative versus ground-relative behavior;
- envelope and saturation diagnostics.

### Tranche B — 6-DOF parity and extension

Port each applicable 3-DOF case to `*mode rigid-body-6dof`, adding only the
attitude, moment, actuator, and vehicle-table data required by the mode.
Compare translational outcomes where equivalent, then separately assess
attitude and control behavior.

## Scenario matrix

| ID | Scenario | 3-DOF claim | Additional 6-DOF claim |
| --- | --- | --- | --- |
| D01 | Straight ballistic coast | gravity, frame, RK integration | quaternion and body-rate stability |
| D02 | Constant-thrust ascent with mass loss | thrust and mass-rate correctness | thrust attitude and inertia coupling |
| D03 | Symmetric pitch maneuver | normal acceleration and alpha response | pitch moment, alpha authority, recovery |
| D04 | Pure yaw/sideslip maneuver | lateral guidance response | beta recovery, yaw moment, rudder authority |
| D05 | Banked range turn | lift-vector and cross-range behavior | bank attitude, roll coupling, lateral force |
| D06 | Short-range ProNav intercept | LOS and closing-velocity behavior | attitude allocation and achieved acceleration |
| D07 | Long-range/high-azimuth ProNav | route wrapping and target geometry | large-turn stability and saturation behavior |
| D08 | Wind-offset intercept | air-relative guidance semantics | sideslip, bank, and table-envelope margins |
| D09 | Actuator saturation | bounded guidance failure | moment/rate saturation and recovery |
| D10 | Staged release and reentry | release timing and energy corridor | release attitude, heating, aero-table handoff |

## Required evidence per case

Every case must emit a machine-readable summary containing:

- completion or explicit diagnostic failure;
- final position, velocity, and target miss distance;
- peak altitude, speed, Mach, alpha, and beta;
- table-envelope minimum margins;
- peak body rates and attitude error;
- actuator saturation duration;
- mass and propellant history;
- ProNav demanded versus achieved acceleration;
- energy and heat-rate history where applicable.

Human-readable plots are artifact tests. Long-running, artifact-producing, and
Spectre cases retain the existing `slow`, `artifact`, and `spectre` markers.
Views such as `grammar`, `equations`, `algorithms`, `dof3`, and `dof6` may
overlap; they are not mutually exclusive shards.

The reproducible evidence command is:

```bash
python tools/dev.py dof-matrix
```

It executes the manifest’s ten 3-DOF and nine 6-DOF cases and writes the
machine-readable result to ignored
`artifacts/dof_robustness_v1/summary.json`.

## Acceptance gates

### Gate A — 3-DOF foundation

- D01–D02 pass deterministic reference checks.
- D03–D05 pass bounded maneuver and energy checks.
- D06–D08 pass geometry and wind-semantics checks.
- D09 fails safely and diagnostically when limits are exceeded.
- No unexplained table extrapolation occurs.

### Gate B — 6-DOF kernel

- D01–D02 conserve quaternion norm and maintain positive inertia/mass.
- D03–D05 detect sign, handedness, and moment-direction mutations.
- Translational results agree with the applicable 3-DOF reference within
  documented tolerances when rotational and aerodynamic coupling is disabled.

### Gate C — 6-DOF controller

- D03–D05 recover from bounded attitude disturbances.
- D06–D08 remain inside declared control and table envelopes.
- D09 reports saturation without numerical instability.
- D10 completes the staged handoff or reports a source-bounded limitation.

## Status language

The project may claim “3-DOF baseline verified” only after Gate A. It may claim
“rigid-body 6-DOF kernel verified” after Gate B. It may claim “6-DOF guidance
controller demonstrated over the bounded scenario matrix” only after Gate C.
None of these claims imply TAOS 96.0 historical compatibility.

## Current progress

- Tranche A has an executable baseline covering ballistic coarse/fine,
  level-Mach guidance, predictive and proportional navigation, crosswind,
  table-driven thrust vectoring, wind-speed/heading tables, wind components,
  and manual ballistic reentry.
- The current 3-DOF baseline passes ten cases through the local runtime.
- Acceptance evidence is recorded in
  `tests/fixtures/dof_robustness_v1/manifest.json`, including completion,
  finite-state, duration, vehicle-count, altitude, and guidance-demand
  invariants.
- The initial 6-DOF baseline now passes rocket, glider, drone, and the
  short-range ProNav, actuator saturation, and the long-range
  California–Hawaii route. That route exposed and now guards quaternion norm
  drift at the integrator boundary.
- The rigid-body aero bridge now consumes the existing TAOS `*wind` block,
  resolving geodetic east/north/down components into the ECFC wind vector used
  by the aerodynamic model. The native 6-DOF crosswind case passes and records
  nonzero air-relative sideslip and yaw moment.
- Dedicated native 6-DOF pitch and bank fixtures now exercise the same table
  bridge at a finite atmospheric altitude, with explicit alpha/bank controls
  and nonzero pitch/roll moments.
- The 3-DOF matrix passes ten cases. The 6-DOF matrix now passes ten bounded
  cases, including a true no-propulsion ballistic coast. Gate A evidence is
  substantially assembled; Gates B and C remain
  open until mutation and comparative-evidence work is complete.
