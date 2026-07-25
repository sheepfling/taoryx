# Taoryx Four-Family Flagship Flight Scenarios Plan

**Status:** Proposed Alpha 2 showcase and qualification plan  
**Primary vehicles:** Skywalker X8, X-15, Boeing 747-100, AscTec Hummingbird  
**Primary fidelity:** Rigid-body 6DOF, with optional pseudo-6DOF and 3DOF projections  
**Primary purpose:** Produce four complete, reproducible, family-appropriate flight examples that visibly exercise the vehicle, its controller, its effectors, its mission segments, and its terminal behavior.

---

## 1. Recommendation in one sentence

Build a versioned **Four-Family Flight Showcase** containing one **Flagship Mission Qualification Pack** per vehicle:

1. **Hummingbird:** pad takeoff → hover → 3D waypoint box → yaw maneuver → disturbance recovery → precision landing.
2. **Skywalker X8:** launch/release → stabilize → climb → left/right waypoint course → altitude and speed changes → return → recovery or landing gate.
3. **B747:** source-supported airborne trim → climb → right and left turns → speed changes → descent → stabilized arrival gate; add runway-to-runway only after low-speed, high-lift, landing-gear, and ground-contact models are qualified.
4. **X-15:** air launch → separation → rocket-powered climb and acceleration → control checkout → burnout/coast → energy-management bank reversals → target or recovery corridor.

A family receives the **Flagship Mission Qualified** badge only when its scenario passes a common mission gate and links to the lower-level plant, actuator, controller, numerical, and data-envelope evidence.

---

## 2. Clarify the four archetypes

### Skywalker X8

Treat the X8 as a **powered flying-wing fixed-wing UAV**. Its natural low-level controls are:

- collective elevon;
- differential elevon;
- throttle.

Do not invent an independent rudder channel merely to make its controls resemble a conventional aircraft.

### X-15

Treat the X-15 slot as an **air-launched, rocket-powered hypersonic aircraft with an unpowered energy-management/glide phase**, rather than as a pure glider. This is useful because one mission can demonstrate:

- parent release or air launch;
- separation;
- ignition and powered acceleration;
- mass and propulsion evolution;
- burnout;
- high-energy coast;
- atmospheric descent and glide;
- explicit terminal energy or recovery conditions.

### Boeing 747-100

Treat the current B747 package as a **transport-aircraft local-envelope model** until its data prove more. A convincing airborne route mission is valuable. A claimed runway takeoff and landing is not justified until the package contains and qualifies the required low-speed and ground physics.

### AscTec Hummingbird

Treat the Hummingbird as the cleanest **true ground-to-ground 6DOF capstone**. It can naturally demonstrate all translational directions, yaw, hover, rotor allocation, takeoff, touchdown, and disturbance recovery.

---

## 3. What “full flight” means

A full flight is not necessarily runway takeoff through runway landing. It is a **family-appropriate complete lifecycle**:

```text
valid start contract
    → launch, release, or takeoff
    → stabilization
    → controlled maneuvering
    → waypoint or objective completion
    → terminal approach, target, recovery, or touchdown
    → explicit success or failure event
```

A full-flight scenario must include:

- a documented start contract;
- a segment graph with named entry and exit events;
- meaningful longitudinal, lateral, and vertical motion where applicable;
- a waypoint or objective sequence;
- natural use of the vehicle’s declared controls and effectors;
- an explicit terminal contract;
- a machine-generated evaluation report;
- complete commanded-versus-achieved control telemetry;
- no silent table extrapolation, clamping, or ignored commands;
- deterministic batch and stepwise replay.

A timeout is a truncation, not mission success.

---

## 4. Do not make the hero mission carry all validation by itself

Each flagship mission sits on top of a qualification spine.

```text
source/data checks
      ↓
equilibrium or trim
      ↓
force/moment and numerical closure
      ↓
actuator and control-effect checks
      ↓
body-rate and attitude inner loops
      ↓
speed/altitude/position outer loops
      ↓
segment and transition qualification
      ↓
flagship full mission
      ↓
robustness and replay suite
```

The showpiece answers:

> Can this configured vehicle complete a coherent end-to-end mission using the public tooling?

The supporting tests answer:

> Are the data, equations, controls, numerics, and controller behaviors credible within the declared envelope?

This prevents a visually attractive trajectory from being mistaken for complete model validation.

---

## 5. Common Flagship Mission Qualification contract

Each scenario should declare the following object before controller tuning begins.

```yaml
schema: taoryx.flagship-mission/v1alpha1

scenario:
  id: org.taoryx.showcase.<scenario-id>
  version: 0.1.0
  family: <family-id>
  fidelity: rigid_body_6dof
  claim: <precise claim supported by a passing run>
  nonclaims: []

start_contract:
  type: <grounded | catapult_release | trimmed_airborne | air_release>
  acceptance: {}

segments:
  - id: <segment-id>
    objective: <objective-fragment-id>
    entry_requirements: {}
    success_event: <event-id>
    failure_events: []

required_control_coverage: []
required_waypoints: []

terminal_contract:
  success_event: <event-id>
  final_corridor: {}
  failure_events: []
  timeout_is_success: false

evaluation:
  profile: <evaluation-profile-id>
  acceptance_file: acceptance.yaml

artifacts:
  profile: flagship-flight-v1

robustness:
  profile: qualification-seed-set-v1
```

The acceptance values belong in versioned scenario data. They must be frozen before final controller tuning and must not be loosened merely to turn a failing run green.

---

# Part I — The four flagship scenarios

## 6. Scenario 1: Hummingbird pad-to-pad 3D mission

### Identity

```text
Scenario ID: hummingbird_pad_box_yaw_land_6dof
Display name: Hummingbird takeoff, 3D waypoint box, yaw scan, recovery, and landing
Primary fidelity: rigid_body_6dof
Start contract: grounded_on_pad
Terminal contract: landed_and_disarmed
```

### Claim supported by a pass

> The Hummingbird can initialize on a pad, spool its four motors, take off vertically, stabilize in hover, track a three-dimensional waypoint course, translate in both horizontal axes, climb and descend, execute commanded yaw motion, recover from a declared disturbance, return to the origin, land within the pad corridor, and disarm without leaving its qualified state, actuator, or aerodynamic envelope.

### Segment sequence

1. **Ground idle**
   - Initialize contact and motor state.
   - Confirm zero unintended motion.

2. **Motor spool**
   - Ramp from idle to hover region through motor dynamics.
   - Confirm allocator and motor commands are finite and bounded.

3. **Vertical takeoff**
   - Climb to the first hover altitude.
   - Exercise collective thrust and vertical control.

4. **Hover acquisition**
   - Hold position, altitude, attitude, and heading.
   - Record hover errors and individual rotor margins.

5. **3D waypoint box**
   - Fly north/east/up to one waypoint.
   - Fly west or south at a lower or higher altitude.
   - Include both positive and negative roll and pitch demands.
   - Capture every waypoint in order.

6. **Yaw scan**
   - Execute either a full 360-degree yaw or symmetric positive and negative yaw steps while holding position.
   - Exercise differential rotor torque in both directions.

7. **Disturbance recovery**
   - Apply a declared gust or initial-rate disturbance.
   - Recover to a bounded hover without prolonged saturation.

8. **Return home**
   - Navigate to a point above the launch pad.

9. **Precision descent**
   - Descend inside horizontal and vertical-speed corridors.

10. **Touchdown and disarm**
    - Detect contact.
    - Settle without rebound or tip-over.
    - Ramp motors to disarmed state.

### Required control and effector coverage

- all four commanded and achieved motor speeds;
- collective thrust below, near, and above hover;
- positive and negative roll moment allocation;
- positive and negative pitch moment allocation;
- positive and negative yaw moment allocation;
- actual motor lag and rate-limit behavior;
- at least one bounded allocator saturation test outside the nominal hero run, unless nominal flight naturally reaches a declared saturation test condition.

### Terminal success

Success is a named touchdown-and-disarm event, requiring the configured:

- landing-pad horizontal corridor;
- vertical-position/contact state;
- maximum horizontal speed;
- maximum downward speed;
- maximum roll and pitch magnitude;
- maximum body-rate magnitude;
- stable contact dwell time;
- motor-disarmed state.

### Hero plots

- 3D commanded and achieved path;
- top-down path with waypoint capture markers;
- altitude and vertical speed;
- position and velocity tracking errors;
- roll, pitch, yaw, and body rates;
- four commanded and achieved rotor speeds;
- requested wrench versus allocated and achieved wrench;
- actuator saturation and motor-margin histories;
- gust input and recovery error;
- touchdown corridor and final-state card.

### Deferred claims

Do not claim battery endurance, certified propeller performance, motor thermal behavior, or real-airframe landing survivability unless corresponding data are present and independently qualified.

---

## 7. Scenario 2: Skywalker X8 launch, route, and recovery

### Identity

```text
Scenario ID: x8_launch_figure8_recovery_6dof
Display name: X8 launch, climb, left/right course, energy changes, and recovery
Primary fidelity: rigid_body_6dof
Start contract: catapult_release or source_trimmed_airborne
Terminal contract: recovery_gate_captured
Optional later terminal: touchdown_complete
```

### Claim supported by a pass

> The X8 can initialize from a source-supported powered launch or trimmed airborne condition, stabilize, climb, change speed and altitude, track a waypoint course containing both left and right turns, exercise collective and differential elevon authority in both directions, return to the recovery area, and enter a declared recovery corridor without leaving its identified data or actuator envelope.

### Why not start at zero speed

The X8 should not be forced into a conventional runway takeoff merely for presentation. Use one of these honest starts:

- a catapult-release state with declared launch velocity and attitude;
- a hand-launch surrogate with a documented impulse/handoff;
- a source-trimmed airborne state for the first qualified version.

### Segment sequence

1. **Launch or trimmed release**
   - Enter the model inside the source-supported airspeed and angle envelope.

2. **Launch stabilization**
   - Capture target attitude, airspeed, and climb state.

3. **Powered climb**
   - Increase altitude using throttle and collective elevon.

4. **Left-turn route leg**
   - Capture waypoints requiring one differential-elevon sign.

5. **Right-turn route leg**
   - Use an S-turn, teardrop, or figure-eight section to require the opposite differential-elevon sign.

6. **Energy-change leg**
   - Command a speed increase and decrease.
   - Command an altitude increase and decrease.
   - Keep angle of attack and control positions inside the qualified corridor.

7. **Return-to-home route**
   - Close the route geometrically rather than stopping near an arbitrary distance threshold.

8. **Approach and recovery**
   - Descend toward a recovery gate with explicit position, speed, flight-path-angle, bank, and attitude limits.

9. **Optional flare and touchdown**
   - Add only after low-speed aerodynamics, ground contact, terrain, and touchdown behavior are qualified.

### Required control and effector coverage

- collective elevon above and below trim;
- differential elevon in both directions;
- throttle at low, nominal, and high commanded regions;
- commanded and achieved surface position and rate;
- mixer output to left and right physical elevons when modeled individually;
- surface saturation and rate-margin histories;
- expected sign of pitch and roll response for each control direction.

### Terminal success

For the first release, use an explicit **recovery gate**, not an implied landing. The final state must be inside configured corridors for:

- horizontal and vertical position;
- airspeed;
- heading or course;
- flight-path angle;
- bank angle;
- angle of attack;
- body rates;
- remaining control and data-table margin.

A later `touchdown_complete` terminal may replace the recovery gate after contact and low-speed validation are complete.

### Hero plots

- top-down route with commanded rectangle/figure-eight and actual path;
- 3D route and altitude profile;
- waypoint capture sequence and event timeline;
- cross-track and along-track errors;
- airspeed and altitude commands versus achieved values;
- angle of attack, sideslip, and body rates;
- collective and differential elevon command versus actual;
- left/right surface positions if available;
- throttle and propulsive force;
- saturation, rate, and table-envelope margins;
- final recovery-corridor card.

### Deferred claims

Do not claim post-stall behavior, zero-speed takeoff, precise belly-landing dynamics, or terminal weapon performance from a model qualified only around the identified powered-flight envelope.

---

## 8. Scenario 3: B747 airborne departure-to-arrival mission

### Identity

```text
Scenario ID: b747_trim_route_arrival_6dof
Display name: B747 trim, climb, bidirectional turns, speed change, descent, and arrival capture
Primary fidelity: rigid_body_6dof
Start contract: source_anchored_airborne_trim
Terminal contract: stabilized_arrival_gate
Future scenario: b747_runway_to_runway_6dof
```

### Claim supported by a pass

> Within its declared local flight envelope, the B747 model can initialize at a source-anchored trimmed flight condition, climb and descend, accelerate and decelerate, execute coordinated turns in both directions, track a multi-leg route, exercise elevator, aileron, rudder, and throttle control, and capture a stabilized arrival corridor without table extrapolation, prolonged saturation, or numerical instability.

### Why the first version should be airborne

A runway-to-runway claim requires more than the current cruise/local-flight plant. The full ground lifecycle needs, at minimum:

- low-speed and high-angle aerodynamic coverage;
- flap, slat, spoiler, and gear configuration effects as applicable;
- engine spool behavior in takeoff and landing regimes;
- landing-gear geometry and contact forces;
- runway friction, braking, and steering;
- rotation, flare, touchdown, rebound, and rollout behavior;
- ground-effect treatment when significant.

Until those are present, the honest flagship is a complete **airborne operational route** ending at an arrival or approach gate.

### Segment sequence

1. **Source-anchored trim**
   - Solve attitude, thrust, and pitch control together.
   - Hold the equilibrium before route commands begin.

2. **Climb capture**
   - Command a bounded altitude increase.
   - Exercise elevator and throttle above trim.

3. **Right turn**
   - Capture a route leg using positive lateral-control demand.

4. **Level acceleration**
   - Increase airspeed while holding altitude.

5. **Left turn**
   - Reverse lateral demand and capture a second route leg.

6. **Crosswind or sideslip-correction leg**
   - Exercise rudder naturally through coordinated-flight or crosswind logic.
   - Do not inject a large artificial sideslip outside the local model envelope.

7. **Deceleration and descent**
   - Reduce speed and descend toward the arrival condition.

8. **Arrival capture**
   - Capture position, altitude, airspeed, heading, bank, vertical-rate, and attitude corridors.

### Required control and effector coverage

- elevator above and below trim;
- aileron in both directions;
- rudder in both directions or a declared coordinated/crosswind sequence that produces meaningful bidirectional activity;
- throttle below, near, and above trim;
- commanded and achieved actuator positions and rates;
- surface and propulsion margins;
- expected roll, pitch, and yaw response signs.

### Terminal success

The stabilized arrival gate should include configured corridors for:

- along-track and cross-track position;
- altitude;
- airspeed;
- heading or course;
- bank angle;
- vertical speed or flight-path angle;
- angle of attack and sideslip;
- body rates;
- actuator and table margins.

### Future runway-to-runway scenario

After the missing data and contact model are qualified, add:

```text
runway idle
→ takeoff roll
→ rotation
→ climb
→ route
→ descent
→ approach
→ flare
→ touchdown
→ rollout
→ stop
```

That later scenario should be a separate qualification badge. Passing the airborne route must not silently imply runway capability.

### Hero plots

- top-down commanded and achieved route;
- altitude and speed profile;
- climb/descent and route-event timeline;
- heading, bank, flight-path angle, alpha, and beta;
- elevator, aileron, rudder, and throttle command versus actual;
- cross-track and along-track errors;
- body rates and load-factor histories;
- force and moment decomposition;
- actuator and table margins;
- stabilized-arrival corridor card.

### Deferred claims

Do not claim global-envelope aerodynamics, stall/post-stall handling, runway operations, transport certification, or prediction of a specific real aircraft outside the source-supported local conditions.

---

## 9. Scenario 4: X-15 air-launch, powered climb, and energy-target mission

### Identity

```text
Scenario ID: x15_airlaunch_energy_target_6dof
Display name: X-15 air launch, rocket climb, burnout, coast, bank reversals, and recovery target
Primary fidelity: rigid_body_6dof
Start contract: parent_air_release or prescribed_air_release
Terminal contract: energy_recovery_corridor_captured
Optional alternate terminal: aim_region_reached
```

### Claim supported by a pass

> The X-15 model can initialize from a declared air-release state, separate and stabilize, ignite and complete a powered high-energy climb, exercise its declared pitch, roll, yaw, and propulsion controls within their active regimes, transition through burnout into coast and atmospheric descent, perform energy-management bank reversals, and reach a configured terminal position/energy/attitude corridor without unresolved control chatter, silent data extrapolation, or numerical failure.

### Segment sequence

1. **Parent release**
   - Copy or prescribe release position, velocity, attitude, and rates.
   - Record parent-to-child state lineage when a parent model is used.

2. **Separation stabilization**
   - Establish clearance and bounded attitude/rate conditions before ignition.

3. **Rocket ignition and powered pitch-up**
   - Activate thrust and mass flow.
   - Track a climb or energy command.

4. **Powered lateral maneuver**
   - Include left and right bank demand while dynamic pressure and control authority are appropriate.

5. **Control checkout**
   - Include a small, predeclared longitudinal and/or lateral maneuver sufficient to prove response without leaving the source envelope.

6. **Burnout or commanded cutoff**
   - Emit the exact reason: propellant depletion, thrust-table endpoint, or commanded cutoff.

7. **Coast and apogee transition**
   - Manage attitude and rates through the low-load region.
   - Use reaction controls only when the model actually declares them.

8. **Atmospheric reentry or descent capture**
   - Re-enter the aerodynamic control corridor with explicit event ordering.

9. **Energy-management glide**
   - Execute bank reversals or S-turns to control range and energy.

10. **Terminal recovery or aim corridor**
    - Reach a declared ground-track, altitude, speed/energy, heading, flight-path-angle, attitude, and rate corridor.

### Required control and effector coverage

Use the actual X-15 package declarations. As applicable, require:

- positive and negative longitudinal control relative to trim;
- left and right lateral control;
- yaw-control activity in both directions where available;
- throttle, ignition, or engine-state transitions;
- reaction-control activity only in the regime where modeled;
- command and achieved surface/effector positions and rates;
- propulsion force, mass flow, propellant state, and cutoff reason;
- control-authority and saturation margins through changing dynamic pressure.

### Terminal success

The initial flagship should end at a **recovery-energy corridor** or a well-defined aim region, not at an arbitrary time. The final contract may include:

- horizontal miss or corridor error;
- altitude;
- inertial speed and specific energy;
- heading/course;
- flight-path angle;
- bank, angle of attack, and sideslip;
- body rates;
- remaining control authority;
- positive aerodynamic-table margins.

A physical runway landing should be a later scenario unless the package includes qualified low-speed, gear, contact, flare, and rollout behavior.

### Additional X-15 gates

- detect sustained oscillation or control chatter using a declared frequency/zero-crossing metric;
- report dynamic pressure and every active aerodynamic-table margin;
- report propulsion and mass evolution;
- report any heating metric only when that metric has a validated physical definition;
- classify every transition between propulsion, coast, and aerodynamic-control regimes.

### Hero plots

- 3D ground track and altitude;
- Mach/speed, altitude, and specific energy;
- dynamic pressure and optional validated heating/load channels;
- pitch, bank, heading, alpha, beta, and body rates;
- surface/effector commands and achieved states;
- thrust, mass flow, propellant mass, and cutoff event;
- bank reversals and target-range history;
- chatter/frequency diagnostic;
- aerodynamic and actuator margins;
- final target/recovery-corridor card.

### Deferred claims

Do not claim global hypersonic accuracy, validated thermal protection behavior, complete high-altitude reaction-control behavior, or real-world landing performance unless those data and subsystems are explicitly included and qualified.

---

# Part II — Common tooling and evaluation

## 10. The control-exercise contract

“Exercises the control surfaces” should be machine-checkable, not inferred from a plot.

For every reversible effector or semantic control required by the scenario, the evaluator should verify:

1. The command moves meaningfully above its trim or neutral value.
2. The command moves meaningfully below its trim or neutral value.
3. The achieved actuator follows in both directions.
4. The resulting vehicle response has the expected sign in a declared response channel.
5. The actuator returns toward trim or its next valid operating point.
6. Position, rate, latency, and saturation behavior are recorded.
7. Any clipping or rejection has a reason code.

For nonreversible controls, such as throttle or rotor magnitude, require configured low, nominal, and high operating regions instead of positive and negative signs.

Generate a `control_coverage.json` report such as:

```json
{
  "scenario": "x8_launch_figure8_recovery_6dof",
  "controls": {
    "control.elevon.collective": {
      "required_directions": ["above_trim", "below_trim"],
      "observed_directions": ["above_trim", "below_trim"],
      "response_sign_check": "pass",
      "rate_margin_check": "pass",
      "saturation_check": "pass"
    },
    "control.elevon.differential": {
      "required_directions": ["left", "right"],
      "observed_directions": ["left", "right"],
      "response_sign_check": "pass",
      "rate_margin_check": "pass",
      "saturation_check": "pass"
    }
  },
  "passed": true
}
```

The required excursion, dwell, and response thresholds are scenario data, not hardcoded universal constants.

---

## 11. Common full-mission pass gates

A scenario is **Flagship Mission Qualified** only when all applicable gates pass.

### Plant and data prerequisites

- The source evaluator or normalized data package passes its own checks.
- A source-supported equilibrium, trim, or hover state exists.
- Force and moment decomposition closes within the declared tolerance.
- All active tables have positive in-envelope margin.
- No silent extrapolation or clamping occurs.

### Numerical prerequisites

- The scenario passes the configured fixed-step convergence study.
- An independent or higher-accuracy reference is available where required.
- Quaternion norm, mass/resource bounds, and physical invariants remain valid.
- Batch execution and repeated public `step()` execution agree within declared tolerances.

### Controller prerequisites

- Required inner-loop axes are stable.
- Commanded and achieved actuator histories are available.
- Rise, settling, overshoot, steady-state error, body-rate, control effort, and saturation metrics are reported where applicable.
- Missing, stale, and invalid external-control policies are tested.

### Mission gates

- The documented start contract passes.
- Every selected segment initializes successfully.
- Every segment exits through a named success or failure event.
- All required waypoints or objectives are completed in order.
- Required control coverage passes.
- Cross-track, along-track, altitude, speed, attitude, and other mission-specific errors pass.
- No undeclared state discontinuity occurs.
- The final corridor passes.
- A named mission-complete event occurs before timeout.

### Artifact gates

- The packet is self-contained or references immutable, hashed source resources.
- No local absolute machine paths appear.
- Every reported scalar agrees with the telemetry and plot source.
- All inputs, outputs, tables, versions, seeds, and hashes are recorded.
- One command regenerates the packet.

### Claim gate

The generated report must state:

- what the passing scenario proves;
- the fidelity and exact vehicle package version;
- the qualified envelope;
- which data are measured, source-derived, digitized, synthetic, or assumed;
- what is expressly not claimed.

---

## 12. Required run artifacts

Each flagship scenario directory should produce:

```text
run/<run-id>/
├── manifest.json
├── claim.json
├── resolved_case.json
├── compiled_scenario.json
├── realized_fidelity.json
├── parameter_provenance.json
├── control_schema.json
├── observation_schema.json
├── state_schema.json
├── controller_config.json
├── telemetry.parquet
├── controls.parquet
├── actuators.parquet
├── events.json
├── segment_timeline.json
├── waypoint_report.json
├── control_coverage.json
├── envelope_report.json
├── equation_closure_report.json
├── convergence_report.json
├── evaluation.json
├── terminal_state.json
├── reproduction.txt
└── plots/
```

The top-level showcase should also contain a summary matrix:

| Scenario | Start | Terminal | Waypoints | Control coverage | Envelope | Convergence | Batch/step parity | Result |
|---|---|---|---:|---|---|---|---|---|
| Hummingbird | grounded pad | touchdown/disarm | required | pass/fail | pass/fail | pass/fail | pass/fail | pass/fail |
| X8 | launch/release | recovery gate | required | pass/fail | pass/fail | pass/fail | pass/fail | pass/fail |
| B747 | airborne trim | arrival gate | required | pass/fail | pass/fail | pass/fail | pass/fail | pass/fail |
| X-15 | air release | energy/target corridor | required | pass/fail | pass/fail | pass/fail | pass/fail | pass/fail |

---

## 13. Standard plot suite

Every family should use a common visual grammar while retaining family-specific panels.

### Common plots

1. Commanded and achieved top-down route.
2. Three-dimensional trajectory.
3. Altitude, speed, and energy history.
4. Waypoint and segment event timeline.
5. Attitude and body rates.
6. Commanded versus achieved controls.
7. Actuator positions, rates, saturation, and health.
8. Aerodynamic, propulsion, gravity, and total force.
9. Aerodynamic, propulsion, control, and total moments.
10. Alpha, beta, Mach, dynamic pressure, and model-envelope margins.
11. Cross-track, along-track, altitude, and speed errors.
12. Final success corridor card.

### Family-specific additions

- **Hummingbird:** four motor speeds, allocated wrench, touchdown/contact state.
- **X8:** collective/differential elevon and left/right surface reconstruction.
- **B747:** elevator/aileron/rudder/throttle, coordinated-flight and arrival-gate metrics.
- **X-15:** thrust/mass flow/propellant, bank reversals, dynamic-pressure corridor, chatter diagnostic.

---

## 14. Controller variants over the same physical scenario

Do not duplicate the physical mission for each control mode. Resolve one scenario, then run approved authority profiles:

### Nominal autopilot

The baseline pass used for the public showcase and regression artifact.

### External commanded mode

An AI, script, or player supplies high-level heading, speed, altitude, attitude, or rate commands that the autopilot tracks.

### Residual or overlay mode

An external controller applies bounded corrections to the nominal autopilot. This is the recommended first RL mode.

### Direct-effector mode

Available only where the fidelity and vehicle package support physical effectors. It should use the same actuator limits and telemetry as the autopilot path.

A vehicle may be Flagship Mission Qualified in autopilot mode before direct-effector control is RL-qualified. Report the status separately.

---

## 15. Multi-fidelity versions

The rigid-body 6DOF scenario should be the primary proof of physical effectors. Where the family supports reductions, run the same semantic mission as:

```text
point_mass_3dof
named_pseudo_6dof_profile
rigid_body_6dof
```

The comparison should preserve:

- launch/release intent;
- waypoint sequence;
- environmental inputs;
- mass/loadout selection;
- high-level guidance commands;
- terminal objective;
- common output channels.

It should not pretend that direct surface activity exists in 3DOF. Instead, report how semantic commands are realized:

| Semantic command | 3DOF | Pseudo-6DOF | 6DOF |
|---|---|---|---|
| heading/bank | force/lift-vector command | attitude-response command | autopilot to physical effectors |
| altitude/path | lift or normal-acceleration command | attitude/response command | autopilot to surfaces/thrust |
| throttle | net propulsive force | force with response lag | physical propulsion/actuator model |
| direct surfaces | unsupported | surrogate if declared | physical effectors |

Cross-fidelity success means semantic continuity and explained disagreement, not identical trajectories.

---

## 16. Robustness progression

Use a staged evidence plan.

### R0 — Nominal development

- one deterministic nominal run;
- complete artifacts and plots;
- all strict mission gates pass.

### R1 — Fixed perturbation matrix

Use a small, versioned set covering applicable:

- initial position, velocity, attitude, and rate offsets;
- wind or gust;
- mass and inertia variation;
- propulsion variation;
- actuator lag/rate variation;
- sensor noise or latency when a sensor-based controller is used.

All cases are individually named and failures are classified.

### R2 — Development Monte Carlo

Run at least the project’s declared development sample count with fixed seeds and confidence intervals.

### R3 — Release Monte Carlo

Run the larger release evidence set defined by the maturity plan. Archive every failure artifact and report the lower confidence bound on success probability.

The nominal showcase may be published before R3, but the badge must say whether it is nominal-only, fixed-matrix qualified, or release-robustness qualified.

---

# Part III — Delivery plan

## 17. Recommended implementation order

### Stage 1 — Build the common flagship infrastructure

Deliver once for all four families:

- scenario schema;
- segment/objective event reporting;
- waypoint evaluator;
- final-corridor evaluator;
- control-coverage evaluator;
- commanded-versus-achieved actuator logging;
- envelope and table-margin evaluator;
- common plot specifications;
- self-contained run manifest and reproduction command;
- full-run versus stepwise equivalence test.

**Exit:** a synthetic reference vehicle can generate a complete Flagship Mission Qualification Pack.

### Stage 2 — Hummingbird

Why first:

- clean ground start and touchdown;
- clear four-effector allocation;
- natural XYZ and yaw coverage;
- short simulation horizon;
- easiest visually obvious proof.

**Exit:** pad-to-pad nominal pass plus disturbance recovery and fixed perturbation matrix.

### Stage 3 — X8

Why second:

- small fixed-wing controller stack;
- clear collective/differential elevon semantics;
- short waypoint route;
- proves launch/release and recovery-gate contracts.

**Exit:** source-trimmed launch, bidirectional route, energy changes, and recovery-gate pass.

### Stage 4 — B747

Why third:

- reuses fixed-wing guidance and controller infrastructure;
- validates scaling from UAV to transport aircraft;
- requires careful local-envelope and trim handling.

**Exit:** airborne trim-to-arrival scenario. Runway-to-runway remains a separately gated follow-on.

### Stage 5 — X-15

Why fourth:

- hardest transition and envelope problem;
- combines release, propulsion, depletion, burnout, coast, reentry, and glide;
- requires chatter and dynamic-pressure diagnostics.

**Exit:** air-release-to-energy-corridor mission with explicit transition events and complete control/propulsion telemetry.

### Stage 6 — Cross-family showcase release

- run all four from one command;
- generate a four-card summary page;
- generate a common comparison dashboard;
- validate every manifest and checksum;
- publish exact claim/nonclaim language;
- freeze scenario, controller, and data-package versions.

---

## 18. Suggested repository layout

```text
examples/
└── flagship_flights/
    ├── hummingbird_pad_box_yaw_land/
    │   ├── case.yaml
    │   ├── acceptance.yaml
    │   ├── controller.yaml
    │   ├── README.md
    │   └── plots.yaml
    ├── x8_launch_figure8_recovery/
    ├── b747_trim_route_arrival/
    └── x15_airlaunch_energy_target/

qualification/
└── flagship_flights_v1/
    ├── manifest.yaml
    ├── metric_dictionary.yaml
    ├── seed_sets/
    ├── claims/
    ├── generated/
    └── summary/

src/taoryx/
├── scenarios/
├── objectives/
├── evaluation/
│   ├── waypoints.py
│   ├── terminal_corridors.py
│   ├── control_coverage.py
│   ├── envelope.py
│   └── mission_report.py
└── artifacts/
    └── flagship_pack.py
```

---

## 19. CLI surface

```bash
# Inspect the scenarios
taoryx scenario list --tag flagship-v1
taoryx scenario inspect hummingbird_pad_box_yaw_land_6dof

# Preflight and explain
taoryx scenario preflight examples/flagship_flights/x8_launch_figure8_recovery/case.yaml
taoryx scenario explain examples/flagship_flights/x8_launch_figure8_recovery/case.yaml

# Run and evaluate
taoryx scenario run examples/flagship_flights/hummingbird_pad_box_yaw_land/case.yaml
taoryx scenario evaluate runs/<run-id> --profile flagship-flight-v1

# Verify runtime equivalence
taoryx scenario verify runs/<run-id> --batch-step-parity

# Exercise authority variants
taoryx scenario run <case> --authority autopilot
taoryx scenario run <case> --authority residual --controller scripted:policy.py
taoryx scenario run <case> --authority direct --controller scripted:direct_controls.py

# Build the whole showcase
taoryx showcase build flagship-flights-v1
taoryx showcase verify flagship-flights-v1
taoryx showcase report flagship-flights-v1
```

---

## 20. Public presentation format

The showcase landing page should have one card per vehicle.

Each card shows:

- vehicle and fidelity;
- one-sentence claim;
- start and terminal contract;
- small 3D trajectory image;
- waypoints completed;
- controls/effectors exercised;
- maximum envelope use;
- terminal error/corridor result;
- robustness level;
- green/yellow/red qualification state;
- exact limitation statement;
- reproduction command.

The public headline can be:

> **Four vehicle archetypes, four complete mission lifecycles, one deterministic Taoryx runtime.**

The supporting sentence should remain precise:

> Each scenario demonstrates a source-bounded research model completing a family-appropriate mission with explicit launch/start, waypoint/objective, control-effector, envelope, terminal, numerical, and reproducibility gates. These are engineering testbed qualifications within declared envelopes, not aircraft certification claims.

---

## 21. Flagship Mission Qualified definition of done

> **A Taoryx vehicle scenario is Flagship Mission Qualified when a new user can resolve and execute the versioned case through public tooling; the vehicle begins from a documented family-appropriate start; every segment and transition is explicit; all declared waypoints or objective fragments complete in order; the required semantic controls and physical effectors are demonstrably exercised with command-to-achieved provenance; the vehicle performs the scenario’s required longitudinal, lateral, vertical, and energy maneuvers without leaving the declared data, state, actuator, or numerical envelope; it reaches an explicit family-appropriate terminal corridor or touchdown event rather than timing out; batch and stepwise execution agree; and the self-contained artifact pack reproduces the result, plots, metrics, claim boundary, and failure diagnostics from one command.**

For the four-family v1 showcase, “done” specifically means:

- **Hummingbird:** grounded takeoff through touchdown and disarm.
- **X8:** launch/release through bidirectional waypoint course and recovery gate; touchdown only when separately qualified.
- **B747:** source-supported airborne trim through climb, bidirectional route maneuver, descent, and stabilized arrival gate; runway lifecycle only when separately qualified.
- **X-15:** air release through powered climb, burnout, coast, energy-management descent, and target/recovery corridor.

That is a defensible point at which you can say:

> “These are not isolated response snippets. Each vehicle has a reproducible, end-to-end Taoryx mission that uses its real control contract, completes meaningful objectives, and produces the evidence needed to understand exactly what passed.”
