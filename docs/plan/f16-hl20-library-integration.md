# Taoryx F-16 and HL-20 Library Integration Plan

The same source-grounded integration boundary now includes the planned A320
reference family. Its intake record is maintained separately in
[a320-daveml-integration.md](a320-daveml-integration.md); it remains blocked
until an exact A320 DAVE-ML source package and check cases are supplied.

**Status:** Proposed Alpha 2 / Alpha 3 library expansion  
**Scope:** Incorporate the DAVE-ML-derived F-16 and HL-20 models as source-grounded reference vehicle families, complete flagship missions, multi-fidelity parents, and AI/control benchmarks.  
**Primary decision:** Preserve each accepted DAVE-ML plant as an immutable evidence artifact, then compose separately versioned Taoryx actuator, controller, mission, sensor, and reduction layers around it.

---

## 1. Executive decision

The F-16 and HL-20 should not enter the library as merely two more aircraft rows. They should become **reference-anchor families**.

They have a role that differs from more exploratory or reconstructed vehicles:

- Their aerodynamic functions are represented as formal DAVE-ML graphs.
- Their source packages contain explicit identities, conventions, validity bounds, and check cases.
- Their existing Taoryx packages already exercise nonlinear six-axis loads, rigid-body integration, trim, and deterministic packaging.
- They can serve as parents for controlled 3DOF and pseudo-6DOF reductions whose errors are measured against an executable 6DOF source plant.
- They can anchor the validation of controllers and public-data surrogate families without implying that those surrogates reproduce either aircraft.

The library should therefore distinguish:

```text
source-grounded reference family
        versus
engineering showcase family
        versus
public-data surrogate family
```

The F-16 and HL-20 occupy the first category.

---

## 2. What already exists

### 2.1 F-16 S-119 reference plant

The existing F-16 package is already a useful rigid-body plant baseline. It combines:

- nonlinear six-axis subsonic aerodynamics;
- steady thrust as a function of power setting, altitude, and Mach;
- fixed rigid-body mass properties and a longitudinal CG parameter;
- direct elevator, aileron, rudder, and throttle mappings;
- source/component verification;
- a reproducible straight-flight trim;
- a five-second 6DOF trim-hold regression;
- deterministic `.txair` packaging and provenance.

Its current completion boundary is important:

- Mach is limited to the declared subsonic range;
- engine spool dynamics and fuel flow are not modeled;
- mass and inertia do not evolve with fuel or stores;
- surfaces have no actuator state, rate limit, or latency;
- SAS and autopilot states are not bound;
- ground contact and runway operations are absent;
- the hold case is an internal-consistency regression, not an operational flight validation.

This means the F-16 is presently a **qualified reference plant**, not yet a pickup-ready mission family.

### 2.2 HL-20 Mod K reference plant

The existing HL-20 package is an especially strong aerodynamic reference. It contains:

- an exact pinned DAVE-ML aerodynamic source;
- a large nonlinear expression and table graph;
- Mach-dependent limits and behavior through the declared Mach-4 package envelope;
- angle-of-attack, sideslip, and body-rate effects;
- force and moment coefficients;
- direct deflections for seven aerodynamic surfaces;
- ground-effect and landing-gear aerodynamic increments;
- a separately sourced fixed mass, CG, and inertia binding;
- source static-shot verification;
- an unpowered glide trim;
- a five-second 6DOF glide-hold regression;
- deterministic `.txair` packaging and regeneration.

Its current completion boundary is also important:

- the vehicle is unpowered;
- mass properties are fixed;
- control surfaces are direct algebraic inputs rather than actuator states;
- no SAS, guidance, or autopilot is included;
- no entry-energy-management controller is included;
- aerodynamic gear increments are not a landing-contact model;
- no runway touchdown or rollout model exists;
- the glide hold is internal consistency evidence, not an independent complete-trajectory comparison.

This makes the HL-20 a **qualified unpowered lifting-body plant**, not yet a pickup-ready entry/glide mission family.

---

## 3. The gaps these two families fill

| Library gap | F-16 contribution | HL-20 contribution |
|---|---|---|
| Source-grounded controlled fixed wing | Conventional elevator, aileron, rudder, throttle, moments, and inertia | Not the primary role |
| High-control-authority aircraft | Strong direct-control and body-rate benchmark | Multi-surface lifting-body allocation benchmark |
| Nonlinear force/moment validation | Subsonic fighter-class force/moment plant | Larger nonlinear lifting-body graph through Mach 4 |
| Propelled 6DOF reference | Steady jet thrust model | Unpowered reference |
| Energy-management glide | Secondary | Primary reference |
| Lifting-body behavior | Limited | Primary reference |
| Surface-allocation complexity | Conventional three-axis controls | Seven direct aerodynamic surfaces |
| DAVE-ML conformance | Multi-document aero/propulsion/inertia/control binding | Large exact-source graph and check-case stress test |
| 3DOF reduction parent | Powered fixed-wing performance reduction | Unpowered entry/glide reduction |
| Pseudo-6DOF reduction parent | Fighter attitude/rate response | Lifting-body response and control-allocation surrogate |
| RL control progression | Guidance → attitude/rate → direct surfaces | Energy guidance → attitude/rate → direct multi-surface control |
| Public-surrogate grounding | Generic high-performance autonomous-jet architecture tests | Generic lifting-body or hypersonic-glider architecture tests |

These models should complement, not replace, the existing families:

- The **X8** remains the small flying-wing UAV and launch/recovery example.
- The **B747** remains the transport-scale example.
- The **X-15** remains the air-launched rocket aircraft with powered ascent, burnout, and glide transitions.
- The **F-16** becomes the source-grounded conventional high-performance fixed-wing reference.
- The **HL-20** becomes the source-grounded unpowered lifting-body and energy-management reference.

The HL-20 should not silently replace the X-15. They prove different things:

```text
X-15: air release + rocket propulsion + depletion + cutoff + high-energy transition
HL-20: nonlinear lifting-body aerodynamics + multiple surfaces + unpowered energy management
```

---

## 4. Preserve an immutable plant and add composable overlays

The most important integration rule is:

> Never modify the accepted DAVE-ML-derived plant in place to add convenient actuator, controller, mission, or sensor behavior.

Use this stack instead:

```text
accepted source plant
        ↓
Taoryx convention adapter
        ↓
optional mass/configuration binding
        ↓
actuator and effector dynamics
        ↓
control allocation and mixing
        ↓
SAS / autopilot / guidance
        ↓
sensor and observation profiles
        ↓
segments, objectives, and flagship mission
        ↓
RL task or trajectory-provider facade
```

Each layer gets its own version, provenance, fidelity claim, and verification.

### 4.1 Evidence classification by subsystem

A family manifest should classify each subsystem independently:

```yaml
evidence:
  aerodynamics:
    class: source_checked
    source: daveml

  propulsion:
    class: source_checked_steady

  mass_properties:
    class: source_bound_fixed

  actuators:
    class: taoryx_engineering_assumption

  controller:
    class: taoryx_reference_design

  sensors:
    class: synthetic_testbed_profile

  mission:
    class: taoryx_qualification_scenario
```

This prevents a source-grounded aerodynamic model from making the attached controller or actuator assumptions appear equally authoritative.

### 4.2 Proposed repository layout

```text
families/
├── reference_f16_s119/
│   ├── family.yaml
│   ├── plant/
│   │   ├── f16-s119-reference-v0.7.txair
│   │   └── source-lock.yaml
│   ├── bindings/
│   │   ├── canonical-controls.yaml
│   │   ├── canonical-observations.yaml
│   │   └── envelope.yaml
│   ├── actuators/
│   │   ├── ideal-direct.yaml
│   │   └── reference-first-order-v1.yaml
│   ├── controllers/
│   │   ├── sas-basic-v1.yaml
│   │   ├── attitude-rate-v1.yaml
│   │   └── route-autopilot-v1.yaml
│   ├── reductions/
│   │   ├── performance-3dof-v1/
│   │   └── attitude-response-p6dof-v1/
│   ├── segments/
│   ├── objectives/
│   ├── scenarios/
│   └── qualification/
│
└── reference_hl20_mod_k/
    ├── family.yaml
    ├── plant/
    │   ├── hl20-mod-k-unpowered-v0.10.txair
    │   └── source-lock.yaml
    ├── bindings/
    ├── actuators/
    ├── allocators/
    ├── controllers/
    ├── reductions/
    ├── segments/
    ├── objectives/
    ├── scenarios/
    └── qualification/
```

---

# Part I — F-16 family plan

## 5. F-16 role in Taoryx

The F-16 should become the **canonical conventional controlled fixed-wing reference family**.

Its primary uses are:

1. Validate complete rigid-body force, moment, and inertia plumbing.
2. Validate the elevator/aileron/rudder/throttle control path.
3. Develop reusable fixed-wing SAS, attitude, rate, speed, altitude, and route controllers.
4. Exercise autopilot, commanded, residual, and direct-control authority modes.
5. Generate and validate 3DOF and pseudo-6DOF reductions.
6. Serve as a source-grounded benchmark for generic high-performance autonomous-jet surrogates.
7. Provide a demanding but bounded RL plant for control and waypoint tasks.

The family claim must remain narrow:

> A source-grounded subsonic research reference for nonlinear rigid-body flight, control, trim, and mission-tool validation—not an operational F-16 training model.

---

## 6. F-16 fidelity ladder

### 6.1 `f16.performance_3dof.v1`

Purpose:

- fast route and energy simulation;
- controller prototyping;
- large-batch RL;
- cross-fidelity comparison.

Recommended state:

```text
position
velocity or speed/flight-path/heading
mass as fixed package parameter initially
optional engine power state in a later revision
```

Recommended controls:

```text
bank command
normal-acceleration or lift command
speed or throttle command
heading command
flight-path-angle or altitude command
```

Data should be generated from the source plant rather than hand invented:

- trim grid over Mach, altitude, and normal acceleration;
- lift and drag along trimmed or constrained operating states;
- thrust versus Mach, altitude, and power setting;
- feasible bank and load-factor limits;
- stall or alpha-limited regions;
- energy-rate maps;
- controller-authority margin.

The reduction manifest must state exactly what it preserves and discards.

### 6.2 `f16.attitude_response_p6dof.v1`

Purpose:

- easier player and AI control;
- believable attitude and body-rate response;
- cheaper large-batch control training;
- a bridge between guidance-level and direct-surface control.

Recommended state:

```text
translation
quaternion attitude
body rates
optional actuator aggregate states
```

Recommended controls:

```text
bank or roll-rate command
pitch attitude, alpha, or pitch-rate command
yaw-rate or sideslip command
throttle
```

The response schedules should be identified from the controlled 6DOF plant at a declared trim grid. They should include:

- axis time constants or natural frequencies;
- damping ratios;
- cross-axis coupling;
- rate and acceleration limits;
- dynamic-pressure scheduling;
- alpha and sideslip protection;
- actuator saturation effects;
- validity and fallback behavior.

### 6.3 `f16.rigid_body_6dof.v1`

This remains the source-grounded plant, augmented through explicit overlays:

- ideal direct surfaces for source-plant regression;
- optional reference actuator dynamics;
- SAS and rate control;
- attitude control;
- speed, altitude, and heading loops;
- route/waypoint guidance;
- action and observation profiles for RL.

The ideal-direct configuration must remain available so the source plant can always be tested without added actuator dynamics.

---

## 7. F-16 missing layers to add

### 7.1 Actuator layer

Add a separately versioned reference actuator package for:

```text
elevator
aileron
rudder
throttle or power-lever response
```

Each channel needs:

- position bounds;
- rate limits;
- optional acceleration limits;
- first- or second-order response;
- latency;
- neutral/trim behavior;
- saturation reporting;
- failure and jam states;
- evidence class and source/assumption note.

Do not imply that assumed actuator dynamics came from DAVE-ML unless they actually did.

### 7.2 Controller stack

Recommended controller sequence:

```text
basic damping / SAS
    ↓
body-rate controller
    ↓
attitude / alpha / sideslip controller
    ↓
speed and altitude controller
    ↓
heading / course controller
    ↓
waypoint and segment guidance
```

Required authority profiles:

```text
autopilot
commanded setpoints
bounded residual
mixed per channel
direct physical controls
```

### 7.3 Start contracts

Initial supported starts:

- source-supported straight-and-level trim;
- turning trim where solver support exists;
- arbitrary in-envelope airborne state with preflight checks;
- randomized in-envelope RL reset.

Runway start should remain unsupported until gear, contact, brakes, steering, and low-speed behavior are separately qualified.

---

## 8. F-16 flagship mission

### 8.1 Scenario identity

```text
scenario_id: f16_subsonic_energy_route_arrival_6dof
family: reference_f16_s119
fidelity: rigid_body_6dof
start_contract: trimmed_airborne
terminal_contract: stabilized_arrival_gate
```

### 8.2 Mission sequence

```text
trim hold
→ throttle-up acceleration
→ controlled climb
→ right turn and waypoint capture
→ roll reversal
→ left turn and waypoint capture
→ altitude reduction
→ speed reduction
→ rudder/sideslip coordination fragment
→ final route leg
→ stabilized arrival gate
```

### 8.3 What the mission must visibly prove

- Elevator moves above and below trim.
- Aileron moves in both directions.
- Rudder moves in both directions or performs a symmetric sideslip/yaw task.
- Throttle visits low, nominal, and high operating regions.
- The vehicle climbs and descends.
- The vehicle turns left and right.
- The vehicle accelerates and decelerates.
- Waypoints are captured in order.
- Alpha, beta, rates, controls, and Mach remain inside the declared envelope.
- The run terminates by reaching an arrival corridor, not by timeout.
- Batch and stepwise trajectories agree within the declared numerical tolerance.

### 8.4 Recommended objective fragments

```text
hold_trim
capture_speed
capture_altitude
capture_heading
capture_waypoint
coordinated_turn
bounded_sideslip
energy_change
arrival_corridor
```

### 8.5 Terminal corridor

The initial terminal contract should require bounded:

```text
position error
altitude error
course or heading error
speed error
bank angle
flight-path angle
alpha and beta
body rates
control saturation
model-envelope margin
```

It should not claim a landing.

### 8.6 F-16 nonclaims

The flagship package should state explicitly:

- no runway takeoff or landing;
- no stores, damage, or fuel-dependent mass properties;
- no validated engine transients;
- no Mach-above-one mission;
- no claim of operational or training fidelity;
- no independent full-flight validation unless one is later added.

---

## 9. F-16 RL and control benchmarks

Recommended tasks:

### Task A — body-rate tracking

Action:

```text
roll-rate command
pitch-rate command
yaw-rate command
```

Observation:

```text
body rates
attitude error
alpha/beta
speed
dynamic pressure
actuator state
```

### Task B — residual waypoint flight

The built-in autopilot provides the nominal command. The agent supplies bounded residuals to:

```text
bank command
normal-acceleration command
throttle command
```

### Task C — direct-surface recovery

The agent directly controls:

```text
elevator
aileron
rudder
throttle
```

Start from a bounded disturbed in-envelope state and recover to a corridor.

### Task D — cross-fidelity transfer

Train on:

```text
3DOF → pseudo-6DOF
```

Evaluate on:

```text
rigid-body 6DOF with the same semantic action profile
```

---

# Part II — HL-20 family plan

## 10. HL-20 role in Taoryx

The HL-20 should become the **canonical unpowered lifting-body and energy-management reference family**.

Its primary uses are:

1. Validate large nonlinear DAVE-ML graphs and interpolation behavior.
2. Validate broad-Mach aerodynamic force and moment application.
3. Validate body-rate damping and moment-reference transfer.
4. Develop multi-surface control allocation.
5. Develop unpowered glide, crossrange, and energy-management guidance.
6. Generate 3DOF and pseudo-6DOF reductions for glider and entry research.
7. Support RL tasks involving bank reversals, energy control, and terminal corridors.
8. Anchor generic lifting-body and hypersonic-glider surrogate work without claiming a proprietary or operational model.

The family claim should be:

> A source-grounded, unpowered, fixed-mass lifting-body research reference for nonlinear aerodynamic evaluation, rigid-body glide, multi-surface control, and energy-management tooling.

---

## 11. HL-20 control abstraction

The physical plant exposes seven direct aerodynamic surfaces. The family should provide two control views.

### 11.1 Direct surface view

Expose every physical channel individually for plant verification and advanced RL:

```text
body_flap_upper_left
body_flap_lower_left
body_flap_upper_right
body_flap_lower_right
wing_flap_left
wing_flap_right
rudder
```

The exact source naming and positive-deflection conventions must be retained in the binding metadata.

### 11.2 Logical control view

Most users and controllers should operate through logical channels such as:

```text
symmetric_body_flap
body_flap_differential_roll
body_flap_differential_yaw_or_mix
symmetric_wing_flap
wing_flap_differential
rudder
speed_brake_or_drag_command, when physically supported by a declared mix
```

The allocator must expose:

- requested logical control;
- allocated physical deflections;
- achieved logical control;
- saturation and null-space behavior;
- configuration and envelope validity;
- individual surface health.

The logical mixer must never erase access to the direct-source channels.

---

## 12. HL-20 fidelity ladder

### 12.1 `hl20.energy_glide_3dof.v1`

Purpose:

- fast entry/glide mission planning;
- energy-management guidance;
- large-batch RL;
- cross-fidelity reduction studies.

Recommended state:

```text
position
speed or specific energy
flight-path angle
heading/course
mass as fixed package parameter initially
```

Recommended controls:

```text
bank command
lift or alpha command
optional drag/speed-brake command
waypoint or aim-point command
```

Derived data should include:

- clean and controlled lift/drag maps;
- feasible CL/CD or alpha schedules;
- trim surface schedules;
- maximum and minimum L/D regions;
- energy-rate and range-to-go maps;
- bank-reversal authority;
- dynamic-pressure and alpha margins;
- terminal-corridor reachability maps.

### 12.2 `hl20.attitude_response_p6dof.v1`

Purpose:

- believable attitude and body-rate motion;
- easier control than direct seven-surface 6DOF;
- large-batch energy-management and terminal-guidance RL.

Recommended controls:

```text
bank-angle or roll-rate command
alpha or pitch-rate command
sideslip or yaw-rate command
optional drag/energy command
```

Response schedules should be derived from the 6DOF plant plus a declared allocator/controller:

- roll, pitch, and yaw response;
- cross-axis coupling;
- dynamic-pressure scheduling;
- low-authority behavior;
- rate and acceleration limits;
- saturation and control-margin diagnostics.

### 12.3 `hl20.rigid_body_6dof.v1`

The source-grounded plant is retained and augmented through separate layers:

- direct ideal surface mode;
- reference actuator dynamics;
- logical control allocator;
- SAS/rate controller;
- bank/alpha/sideslip controller;
- energy-management and waypoint guidance;
- sensor and RL profiles.

---

## 13. HL-20 missing layers to add

### 13.1 Surface actuator package

Add explicit states and limits for all seven surfaces:

- position limits;
- rate limits;
- response dynamics;
- latency;
- power/health status if modeled;
- jam and saturation modes;
- command-to-achieved telemetry.

Initial values may be engineering reference assumptions, but must be labeled as such.

### 13.2 Control allocation

The allocator must be tested independently of the mission controller.

Required fragments:

```text
pure pitch request
pure roll request
pure yaw request
combined pitch-roll request
combined roll-yaw request
saturation redistribution
one-surface unavailable
symmetry check
```

### 13.3 Guidance and autopilot

Recommended controller sequence:

```text
body-rate damping
    ↓
attitude / alpha / beta control
    ↓
bank and lift control
    ↓
energy-management guidance
    ↓
crossrange / waypoint guidance
    ↓
terminal corridor capture
```

### 13.4 Start contracts

Initial supported starts:

- solved unpowered glide trim;
- air release near an in-envelope trimmed condition;
- high-energy airborne state inside the declared Mach/alpha/beta envelope;
- randomized in-envelope RL reset.

Booster separation can be added later through the common parent-child handoff contract. It should not be embedded directly into the aerodynamic plant.

---

## 14. HL-20 flagship mission

### 14.1 Scenario identity

```text
scenario_id: hl20_release_energy_management_arrival_6dof
family: reference_hl20_mod_k
fidelity: rigid_body_6dof
start_contract: airborne_release
terminal_contract: energy_arrival_corridor
```

### 14.2 Mission sequence

```text
release stabilization
→ glide-trim capture
→ right bank and crossrange waypoint
→ roll reversal
→ left bank and crossrange waypoint
→ bounded direct-surface checkout
→ energy-management descent
→ final heading and speed alignment
→ approach / aim-region energy corridor
```

The scenario should remain inside the model’s declared source envelope. A first release should not deliberately touch the bounds merely to appear dramatic.

### 14.3 What the mission must visibly prove

- The vehicle captures stable unpowered flight after release.
- It executes both right and left banked maneuvers.
- It changes altitude and energy in a controlled way.
- It completes a crossrange waypoint sequence.
- The logical allocator commands physical surfaces predictably.
- Required physical surfaces move in both directions where their design allows.
- Direct-surface mode can exercise all seven channels through bounded checkout fragments.
- Alpha, beta, rates, dynamic pressure, Mach, and table margins remain valid.
- Surface saturation and control-authority margin are reported.
- The run reaches a final energy/position/attitude corridor rather than timing out.

### 14.4 Recommended objective fragments

```text
capture_glide_trim
hold_alpha
hold_bank
bank_reversal
capture_crossrange_waypoint
manage_specific_energy
limit_dynamic_pressure
limit_alpha
surface_checkout
arrival_energy_corridor
```

### 14.5 Terminal corridor

The first qualified terminal state should bound:

```text
position or aim-region error
altitude
speed or specific energy
heading/course
flight-path angle
bank
alpha and beta
body rates
surface margin
aerodynamic-table margin
```

This is an approach or energy-arrival gate, not a touchdown.

### 14.6 HL-20 nonclaims

The flagship package should state explicitly:

- no propulsion or reaction-control capability;
- no complete entry thermal or structural model;
- no runway contact, touchdown, or rollout;
- no claim that aerodynamic gear increments constitute gear/contact dynamics;
- no independent NASA complete-trajectory reproduction until separately demonstrated;
- no vehicle mass/configuration evolution unless separately bound and qualified.

---

## 15. HL-20 RL and control benchmarks

### Task A — bank-reversal energy management

Action:

```text
bank command
alpha or lift command
```

Objective:

- complete crossrange reversals;
- preserve energy margin;
- stay inside alpha and dynamic-pressure limits;
- reach a final corridor.

### Task B — residual glide guidance

The built-in guidance generates nominal bank and alpha schedules. The agent adds bounded residuals.

### Task C — logical surface allocation

Action:

```text
pitch demand
roll demand
yaw demand
drag demand
```

The allocator maps those commands to seven surfaces.

### Task D — direct multi-surface control

The agent commands the seven physical surfaces directly. This is an advanced task and should include strong action scaling, saturation telemetry, and safety limits.

### Task E — cross-fidelity policy transfer

Train energy guidance in 3DOF, continue in pseudo-6DOF, and evaluate on the rigid-body plant with the same high-level action semantics.

---

# Part III — Common qualification and library integration

## 16. Add a Reference Anchor badge

Introduce a badge separate from general pickup readiness:

# **Source-Grounded Reference Anchor**

A family earns this badge when:

1. Every accepted source document is pinned and hashed.
2. Source functions and check cases pass.
3. Unit, frame, sign, reference-area, and moment-reference transformations are explicit.
4. The runtime package is deterministic and independently reloadable.
5. A trim or equilibrium point closes forces and moments.
6. A short rigid-body propagation demonstrates convention and equation consistency.
7. Every missing subsystem and nonclaim is machine-readable.
8. Added Taoryx overlays remain separable from the accepted plant.

The F-16 and HL-20 can carry this badge before they are M5 pickup-ready mission families.

---

## 17. Maturity status and promotion plan

### 17.1 F-16

| Area | Current state | M5 work |
|---|---|---|
| Source and data | Strong baseline | Preserve and pin in the family registry |
| Plant evaluation | Nonlinear aero, propulsion, mass/inertia, direct controls | Maintain source and held-out regressions |
| Numerics | Trim and short hold | Add step sensitivity, maneuver cases, long-run diagnostics |
| Actuators | Missing | Add ideal and dynamic actuator profiles |
| Controller | Missing from current binding | Add SAS, rate, attitude, speed, altitude, heading, and route layers |
| Segments | Minimal | Add trim, acceleration, climb, turn, descent, arrival fragments |
| Full mission | Missing | Qualify the subsonic route-arrival flagship |
| 3DOF reduction | Missing | Derive and cross-validate performance maps |
| Pseudo-6DOF | Missing | Identify and validate response schedules |
| RL readiness | Plant-level only | Add action/observation profiles, reset ranges, tasks, and replay |
| Pickup readiness | Not yet | Public configuration, artifacts, docs, robustness, no-code mission use |

### 17.2 HL-20

| Area | Current state | M5 work |
|---|---|---|
| Source and data | Exact-source aerodynamic baseline | Preserve and pin; retain separate mass/inertia provenance |
| Plant evaluation | Force/moment graph, seven surfaces, glide trim/hold | Add held-out grids and dynamic maneuvers |
| Numerics | Trim and short hold | Add time-step sensitivity and bank-reversal cases |
| Actuators | Missing | Add seven-surface actuator profiles |
| Allocation | Missing | Add logical-to-physical mixer and failure behavior |
| Controller | Missing | Add rate, attitude, alpha/beta, bank, energy, and route layers |
| Segments | Minimal | Add release, glide capture, bank reversal, crossrange, arrival fragments |
| Full mission | Missing | Qualify release-to-energy-corridor flagship |
| 3DOF reduction | Missing | Derive energy-glide maps and compare with 6DOF |
| Pseudo-6DOF | Missing | Identify scheduled attitude response |
| RL readiness | Direct plant only | Add action/observation profiles, tasks, safety limits, and replay |
| Pickup readiness | Not yet | Public composition, evaluation artifacts, docs, robustness |

---

## 18. Reduction qualification requirements

A reduction is not accepted merely because it looks similar in one run.

Each F-16 and HL-20 reduction package must include:

```text
parent 6DOF package identity and hash
derivation version and settings
sample/trim grid
discarded states and physics
preserved quantities
control mapping
observation mapping
validity envelope
fit residuals
held-out pointwise errors
segment-level trajectory errors
event and terminal errors
known non-equivalences
```

### 18.1 Required cross-fidelity benchmark classes

- trim/equilibrium points;
- force and acceleration responses;
- standard command doublets;
- left and right turns;
- climbs and descents where applicable;
- energy changes;
- waypoint segments;
- event timing;
- terminal-corridor outcome;
- control effort;
- envelope-margin comparison.

The target is not identical trajectories. The target is predictable, bounded disagreement for the quantities the reduced model claims to preserve.

---

## 19. Scenario and fragment library additions

### 19.1 Common fixed-wing fragments enabled by F-16

```text
trimmed_airborne_start
hold_trim
capture_speed
capture_altitude
capture_heading
coordinated_turn
bounded_sideslip
roll_reversal
energy_change
waypoint_leg
stabilized_arrival_gate
```

These fragments should be reusable by X8, B747, and generic high-performance-jet surrogates where their contracts match.

### 19.2 Common lifting-body/glider fragments enabled by HL-20

```text
airborne_release
glide_trim_capture
hold_alpha
hold_bank
bank_reversal
crossrange_leg
specific_energy_management
dynamic_pressure_guard
alpha_guard
surface_checkout
energy_arrival_gate
```

These fragments should be reusable by X-15’s unpowered phase and generic hypersonic-glider families where contracts match.

Qualification remains edge-specific. Reusing a fragment definition does not automatically qualify it for every family.

---

## 20. How these models strengthen public-data surrogates

The F-16 and HL-20 should be used as **physics and tooling references**, not as hidden stand-ins for public surrogate vehicles.

### 20.1 High-performance autonomous jet surrogate

The F-16 can validate:

- fixed-wing controller architecture;
- attitude/rate action profiles;
- direct-surface action scaling;
- energy and route evaluators;
- reduction methods;
- RL task mechanics.

It must not be used to imply that an Anduril-inspired autonomous jet has F-16 aerodynamics, mass properties, or performance.

### 20.2 Lifting-body or hypersonic-glider surrogate

The HL-20 can validate:

- energy-management guidance;
- bank-reversal logic;
- multi-surface control allocation;
- broad-Mach table handling;
- 3DOF and pseudo-6DOF reduction methods.

It must not be used to imply that a generic hypersonic glider has HL-20 geometry or performance.

The surrogate manifest should point to these only as validation references:

```yaml
reference_anchors:
  controller_architecture:
    - reference_f16_s119

  energy_management:
    - reference_hl20_mod_k
```

---

## 21. Showcase organization

Do not discard the original four-family showcase. Expand the public presentation into two rows.

### Core archetype missions

```text
Hummingbird
X8
B747
X-15
```

### Source-grounded reference missions

```text
F-16 S-119
HL-20 Mod K
```

This allows a precise headline:

> **Six complete vehicle missions: four breadth archetypes and two source-grounded reference anchors.**

The two reference cards should visibly include:

- source/evidence badge;
- source-model hash and package version;
- fidelity;
- control and effectors exercised;
- envelope margin;
- full mission result;
- cross-fidelity availability;
- exact nonclaims.

---

## 22. Recommended implementation order

### Stage 1 — Registry and immutable plant integration

- Register both `.txair` packages as reference plants.
- Pin hashes and source manifests.
- Add source-grounded evidence classification.
- Add family-level control, observation, and envelope bindings.

**Exit:** both plants load through the normal Taoryx family registry without altering source packages.

### Stage 2 — F-16 actuator and controller spine

- Add ideal and dynamic actuator profiles.
- Add SAS and body-rate loops.
- Add attitude, speed, altitude, and heading loops.
- Add standard response fragments.

**Exit:** repeatable direct and closed-loop maneuver tests pass.

### Stage 3 — F-16 flagship mission

- Add route guidance and objective fragments.
- Add control-coverage and envelope reports.
- Run the full subsonic energy-route mission.

**Exit:** `f16_subsonic_energy_route_arrival_6dof` is Flagship Mission Qualified.

### Stage 4 — HL-20 allocator and controller spine

- Add actuator profiles.
- Add logical-to-physical allocator.
- Add rate, attitude, alpha/beta, and bank loops.
- Add energy-management guidance.

**Exit:** independent allocation and bank-reversal qualification passes.

### Stage 5 — HL-20 flagship mission

- Add release, glide-capture, crossrange, and arrival fragments.
- Run the complete release-to-energy-corridor mission.

**Exit:** `hl20_release_energy_management_arrival_6dof` is Flagship Mission Qualified.

### Stage 6 — Multi-fidelity reductions

- Generate F-16 3DOF and pseudo-6DOF packages.
- Generate HL-20 3DOF and pseudo-6DOF packages.
- Add cross-fidelity benchmark suites.

**Exit:** both families receive Multi-Fidelity Qualified status for the declared reduction envelopes.

### Stage 7 — RL and surrogate reuse

- Add action and observation profiles.
- Add residual and direct-control tasks.
- Add vectorized reduced-fidelity rollouts.
- Reuse controller/evaluator fragments in generic surrogate families.

**Exit:** both families are RL-Ready for their advertised control levels.

---

## 23. Suggested CLI surface

```bash
# Inspect immutable source plants
taoryx family inspect reference_f16_s119
taoryx family inspect reference_hl20_mod_k

taoryx family evidence reference_f16_s119
taoryx family evidence reference_hl20_mod_k

# Verify source and package baselines
taoryx family verify reference_f16_s119 --tier source,plant
taoryx family verify reference_hl20_mod_k --tier source,plant

# Run flagship missions
taoryx scenario run examples/reference_flights/f16_subsonic_energy_route_arrival/case.yaml
taoryx scenario run examples/reference_flights/hl20_release_energy_management_arrival/case.yaml

# Exercise control authority
taoryx scenario run <f16-case> --authority autopilot
taoryx scenario run <f16-case> --authority residual --controller scripted:policy.py
taoryx scenario run <f16-case> --authority direct --controller scripted:surfaces.py

# Inspect reductions
taoryx family compare-fidelities reference_f16_s119 \
  --levels performance_3dof,attitude_response_p6dof,rigid_body_6dof

taoryx family compare-fidelities reference_hl20_mod_k \
  --levels energy_glide_3dof,attitude_response_p6dof,rigid_body_6dof

# Build the source-grounded reference showcase
taoryx showcase build reference-flight-models-v1
taoryx showcase verify reference-flight-models-v1
taoryx showcase report reference-flight-models-v1
```

---

## 24. Definition of done for incorporation

> **The F-16 and HL-20 are incorporated into the Taoryx library when their accepted DAVE-ML-derived plant packages are immutable, hash-pinned reference artifacts discoverable through the normal family registry; every added actuator, controller, sensor, mission, and reduction layer is separately versioned and classified by evidence; each family exposes complete unit-, frame-, bound-, scaling-, and authority-aware parameter, control, observation, and effector schemas; a new user can select a supported fidelity and controller, set documented parameters and an in-envelope initial condition, compose qualified segments through public configuration, preflight the case, and run it batch or stepwise without modifying implementation code; the F-16 completes a subsonic powered route-and-arrival mission that exercises elevator, aileron, rudder, and throttle; the HL-20 completes an unpowered release-to-energy-corridor mission that exercises logical and physical multi-surface control; both runs produce deterministic artifact packs, explicit finality, control coverage, envelope diagnostics, and exact nonclaims; and any advertised 3DOF or pseudo-6DOF reduction passes a documented cross-fidelity benchmark against its parent rigid-body plant.**

---

## 25. The practical library claim after completion

Once this plan is complete, Taoryx can credibly say:

> **The library contains source-grounded reference plants for a powered conventional fixed-wing aircraft and an unpowered lifting body, each extended through transparent Taoryx actuator, controller, mission, and multi-fidelity layers. Users can run the original 6DOF plants, easier pseudo-6DOF variants, or fast 3DOF reductions; compose qualified flight fragments; operate them through autopilot, residual, or direct controls; and reproduce complete waypoint and energy-management missions with clear evidence and limitation boundaries.**
