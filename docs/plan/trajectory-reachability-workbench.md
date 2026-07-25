# Taoryx Effective Kinematic Reachability Envelope Tool Plan

**Status:** Alpha 3 breadth candidate / execution milestone defined
**Product type:** Application and analysis tool built on trajectory-provider interfaces  
**Initial provider:** Taoryx  
**Long-term provider scope:** Any provider that can initialize, run, and report compatible trajectory cases  
**Primary outputs:** Reachability envelopes, launch-state acceptability maps, boundary confidence, witness trajectories, terminal margins, and failure classifications

---

## 1. Executive decision

This capability should be implemented as a separate application, not as vehicle physics inside Taoryx.

The recommended product is a **Trajectory Reachability Workbench** that computes an **Effective Kinematic Reachability Envelope** for a selected vehicle model, fidelity, configuration, controller class, launch-state family, target-state family, environment, and success definition.

The central promise is:

> **Given a vehicle and a family of launch or branch states, determine which target-relative states have at least one valid trajectory that reaches a declared terminal objective while respecting vehicle, control, environment, and model-validity constraints.**

A user should be able to ask questions such as:

- From each point along this launch route, what target positions and velocities are reachable?
- For this launch state, what combinations of target range, bearing, altitude, speed, and heading are feasible?
- What initial aim directions, launch delays, or controller presets produce successful terminal conditions?
- How does the envelope change from 3DOF to pseudo-6DOF to rigid-body 6DOF?
- Which regions are verified feasible, which are unresolved, and why do candidate trajectories fail?
- How much does the feasible region contract under wind, model uncertainty, target maneuver, actuator limits, or control failures?

The visible result may look like a fan, footprint, funnel, tube, or set of range-speed curves. Internally, all of those are slices or projections of the same reachability study.

### 1.1 Reachability Tool Completion Goal

> **Complete the Alpha 3 Reachability Tool as a reproducible end-to-end
> research workflow: a user can connect a declared vehicle model, define its
> search space and terminal goals, execute a deterministic reachability batch,
> retain and classify every candidate, refine timeout cases, and generate
> inspectable Matplotlib visualizations from the resulting artifacts. Prove the
> workflow with the generic rocket/glide baseline and the staged X-15 example
> across reduced-order fidelity tiers, using selected native X-15 replay only
> to exercise the high-fidelity handoff and validity checks.**

This is a process-completion goal, not a claim that the reduced-order X-15 is a
validated performance model or that a finite search certifies physical
infeasibility. The proof must establish that a new vehicle can be integrated,
run, explained, visualized, and escalated without changing the result contract.

#### Completion Gates

| Gate | Required evidence |
| --- | --- |
| **G1. Vehicle integration** | Generic rocket/glide and staged X-15 providers declare state, phases, parameters, fidelity, validity assumptions, and source provenance. |
| **G2. Search and outcome completeness** | A resolved study records the declared axes, every realized candidate, terminal margins, failure reasons, invalid/model-blocked outcomes, and explicit horizon timeouts. |
| **G3. Goal evaluation** | Impact radius, impact speed, terminal speed, ground-contact, and horizon goals are represented in the versioned success specification and reflected in classifications. |
| **G4. Refinement and escalation** | Timeout-only reruns preserve parent query IDs; selected pseudo-6DOF checkpoints can be preflighted and replayed natively, with out-of-domain cases retained as blocked records. |
| **G5. Visualization** | One command renders flown trajectories, declared-versus-realized search coverage, terminal capability, and cross-fidelity progression from artifact data without rerunning simulations. |
| **G6. Reproducible proof** | Example commands, manifests, plot manifests, focused tests, and integration notes demonstrate serial/parallel agreement and state the claim boundary for each fidelity. |

#### Proof Vehicle Set

| Proof lane | Executable configuration | What it proves | Completion boundary |
| --- | --- | --- | --- |
| **Baseline** | Generic single-stage rocket plus glide vehicle, 3DOF and pseudo-6DOF | Known-footprint behavior, impact radius/speed goals, complete outcome accounting, timeout refinement, and comparable fidelity artifacts. | Reduced-order process proof; not a vehicle-validation claim. |
| **Staged example** | X-15-scaled booster, coast, release, and unpowered glide, 3DOF and pseudo-6DOF | Multi-phase state/resource handling, limited-energy capability, trajectory visualization, and capability contraction or movement across tiers. | X-15-scaled surrogate; source-backed assumptions and nonclaims are recorded. |
| **Handoff probe** | Selected X-15 pseudo-6DOF checkpoints through native rigid-body replay | State/frame conversion, aerodynamic-table domain preflight, native telemetry capture, and explicit blocked outcomes. | Selective replay evidence only; not a native batch envelope. |

The broader reachability profile catalog is an onboarding roster, not an Alpha 3
completion dependency. Additional Taoryx examples become proof vehicles when
they have a provider adapter and can pass the same gates; they do not replace
the generic baseline or X-15 multi-phase proof.

#### Minimum Executable Proof Sequence

The completion report should be reproducible from these commands, with the
resolved criteria and search grid recorded in each artifact:

```bash
taoryx reachability run --fidelity point_mass_3dof \
  --include-trajectories \
  --output artifacts/reachability/generic/point_mass_3dof.json

taoryx reachability run --fidelity pseudo_6dof \
  --include-trajectories \
  --output artifacts/reachability/generic/pseudo_6dof.json

taoryx reachability plot \
  artifacts/reachability/generic/point_mass_3dof.json \
  --compare artifacts/reachability/generic/pseudo_6dof.json \
  --output-dir artifacts/reachability/generic/plots

taoryx reachability x15 \
  --output-dir artifacts/reachability/x15

taoryx reachability x15-native-replay \
  artifacts/reachability/x15/pseudo_6dof.json \
  --output-dir artifacts/reachability/x15/native-replay
```

The exact grid, horizon, step size, criteria, worker count, and source
revisions remain part of the emitted manifests; the commands above define the
proof shape, not an unrecorded set of defaults.

### 1.2 Alpha 3 Completion Milestone: Vehicle Onboarding Proof

Alpha 3 is a process-validation milestone below the full R5 qualified
workbench. Its purpose is to prove what is required to connect a vehicle to
the reachability tool, preserve the evidence boundary, and iterate from cheap
models to selective native replay.

The milestone is complete when the X-15 and generic rocket/glide examples can
be run through the following reproducible ladder:

```text
generic 3DOF baseline
        -> generic pseudo-6DOF control refinement
        -> X-15 staged 3DOF envelope
        -> X-15 staged pseudo-6DOF envelope
        -> selective native X-15 6DOF replay
```

The native replay is evidence for selected checkpoints, not a claim that the
native rigid-body provider is already a batch envelope provider.

#### Alpha 3 Requirements

| ID | Requirement | Completion evidence |
| --- | --- | --- |
| A3-RQ-01 | A vehicle provider declares identity, state, parameters, phases, fidelity, and source provenance. | Self-describing result artifact and provider integration notes. |
| A3-RQ-02 | A study declares terminal speed, impact event, target position, impact radius, and impact-speed goals where applicable. | Versioned `success_spec`, terminal margins, and goal-specific failure reasons. |
| A3-RQ-03 | Every candidate is retained with feasible, infeasible, invalid, and model-blocked evidence distinguished. | Complete `samples` table and summary counts. |
| A3-RQ-04 | Horizon termination is explicit and independently queryable. | Per-sample `timed_out`, timeout query IDs, and timeout-focused rerun artifact. |
| A3-RQ-05 | A timeout rerun can extend the horizon without repeating completed candidates. | Parent-study provenance and a longer-horizon subset result. |
| A3-RQ-06 | 3DOF and pseudo-6DOF use the same search contract and produce comparable artifacts. | Shared search-space identity, cross-fidelity plots, and tier comparison report. |
| A3-RQ-07 | Native 6DOF escalation performs domain preflight before execution. | Mach, altitude, angle, sideslip, mass, control, and other table-domain margins. |
| A3-RQ-08 | Native handoff failures are explicit and never silently clamped into a table. | `native_domain_blocked` records with reasons and bridge provenance. |
| A3-RQ-09 | The example suite includes at least one known synthetic problem independent of X-15. | Known-footprint test with impact radius and impact-speed goals. |
| A3-RQ-10 | Runs are deterministic and operationally reproducible. | Serial/parallel agreement, focused tests, manifests, and one-command examples. |
| A3-RQ-11 | Integration friction is recorded as engineering evidence. | Vehicle onboarding notebook covering assumptions, failed attempts, and nonclaims. |
| A3-RQ-12 | Every proof vehicle produces a standard Matplotlib plot bundle from its result artifact. | Flown trajectories, search coverage, terminal capability, and fidelity comparison plots plus a plot manifest. |
| A3-RQ-13 | The proof result is usable without reading implementation details. | One-command run and plot instructions, artifact schema, example output tree, and claim boundary in the accompanying report. |

#### Alpha 3 Definition Of Done

Alpha 3 is complete when a new vehicle can be connected by declaring its
reduced-order model, search variables, terminal goals, validity domain, and
provenance; run a complete candidate batch; inspect successful, unsuccessful,
timed-out, and blocked candidates; rerun unresolved timeout candidates; render
the standard plots from the saved artifact; and escalate selected in-domain
checkpoints to a higher-fidelity provider without modifying the envelope result
contract.

Alpha 3 does not require formal reachable-set certification, uncertainty
quantification, adaptive boundary extraction, or a full native 6DOF batch
search. Those remain R4/R5 workbench objectives.

---

## 2. Pointed product definition

For one resolved study, define:

- A vehicle model and configuration.
- A dynamics fidelity.
- A launch-state or branch-state family.
- A target-state family and target-motion model.
- A permitted controller or control-policy class.
- A set of launch and guidance decision variables.
- Path constraints.
- Terminal success conditions.
- A maximum time horizon.
- Optional disturbances and uncertainty.

The **effective kinematic reachability envelope** is the set of initial relative conditions for which a successful witness trajectory exists under those declarations.

Conceptually:

\[
\mathcal E =
\left\{
\xi_0\;\middle|\;
\exists\,\pi \in \Pi,\;
\exists\,t_f \le T:\;
\text{trajectory}(\xi_0,\pi)
\text{ satisfies all path constraints and enters the terminal target set}
\right\}
\]

where:

- \(\xi_0\) is the combined launch and target-relative initial state.
- \(\Pi\) is the declared policy, guidance, or control family.
- \(T\) is the allowed horizon.
- The terminal target set includes distance, speed, angle, time, and other success criteria.

The envelope is therefore never just a property of the airframe. Its identity must include:

```text
vehicle and loadout
+ fidelity
+ environment
+ launch-state definition
+ target model
+ controller or policy class
+ control authority
+ success criteria
+ time horizon
+ uncertainty treatment
= one envelope definition
```

Changing any of those may change the answer.

When a study uses sensed-data control, it consumes the [sensor and measurement
orchestration contract](sensor-measurement-orchestration.md) rather than
perturbing truth directly. The study records sensor timing, estimator mode,
latency, validity, and truth-isolation status in its provenance.

---

## 3. Use a domain-neutral name, with optional aliases

The core artifact should be called an:

```text
Effective Kinematic Reachability Envelope
```

or simply:

```text
Kinematic Reachability Envelope
```

Depending on the application, the same mathematical object may be presented as:

- A reachable terminal footprint.
- A launch acceptability region.
- A dynamic engagement or intercept envelope.
- A recovery footprint.
- A rendezvous feasibility region.
- A delivery or arrival envelope.
- A backward reachable set or capture basin.

The implementation should use neutral terms such as `launch_state`, `target_state`, `terminal_objective`, `reachability`, and `success_spec`. Domain-specific applications may add their own vocabulary without changing the core contracts.

---

## 4. The tool needs two complementary views

### 4.1 Forward terminal footprint

Question:

> From this launch state, where can the vehicle successfully terminate?

Inputs are fixed or narrowly distributed launch conditions. The sampled dimensions are target position, target motion, or terminal-state requirements.

Typical outputs:

- Ground footprint.
- Three-dimensional terminal volume.
- Range-versus-bearing fan.
- Reachable altitude-versus-range slices.
- Arrival-time bands.
- Terminal-speed contours.

### 4.2 Launch-state acceptability envelope

Question:

> For this target state, from which launch states can the vehicle succeed?

Inputs are fixed or narrowly distributed target conditions. The sampled dimensions are launch position, speed, heading, altitude, attitude, or time.

Typical outputs:

- Acceptable launch corridor.
- Launch-speed-versus-range curves.
- Heading-offset tolerance.
- Earliest and latest feasible launch point.
- Feasible region along a parent or carrier route.

### 4.3 Along-trajectory dynamic envelope

Question:

> At every checkpoint along this trajectory, what terminal objectives remain reachable?

This is the user's “fan that changes along the path” view.

```text
baseline trajectory
        ↓
checkpoint or sample each station
        ↓
branch many candidate trajectories
        ↓
classify and refine the feasible boundary
        ↓
stack the per-station envelopes
        ↓
reachability fan / funnel / tube
```

The visual envelope may contract as the target gets closer, but the tool must not assume monotonic contraction. It may expand when the vehicle gains speed or altitude, split because of turn constraints, contain holes caused by minimum range, or deform when propulsion, staging, atmosphere, or control authority changes.

The shape is an output of the simulation, not a hard-coded geometric rule.

---

## 5. Separate plant reachability from controller reachability

The result depends strongly on what controls the solver is allowed to use.

The study must declare one of these policy scopes.

### 5.1 Fixed-controller envelope

A single guidance and autopilot configuration is used without retuning.

This answers:

> What can the currently implemented controller accomplish?

It is the most operationally reproducible result and the easiest to validate.

### 5.2 Preset-family envelope

The tool may select among a finite set of controller or guidance presets.

This answers:

> What can the approved controller library accomplish?

### 5.3 Parameter-optimized controller envelope

A bounded set of launch aim, guidance gains, timing parameters, or command profiles may be optimized for each query.

This answers:

> What can the declared controller architecture accomplish after bounded tuning?

### 5.4 Open-loop control-search envelope

The solver may optimize a control history directly.

This estimates a broader plant capability, but it may produce trajectories that no practical controller can reproduce robustly. It must be labeled differently from a closed-loop controller envelope.

### 5.5 Robust-policy envelope

One policy must succeed across a declared disturbance or uncertainty set.

This answers:

> What can the vehicle reliably accomplish without choosing a different solution after the uncertainty is revealed?

The output metadata must always record the policy scope. A broad open-loop-search envelope must not be presented as if it were the verified envelope of the production autopilot.

---

## 6. Core study contract

The application should resolve a `ReachabilityStudy` from the following major objects.

```text
ReachabilityStudy
    provider
    vehicle_case_template
    launch_locus
    launch_options
    target_locus
    target_motion
    policy_scope
    decision_variables
    terminal_success_spec
    path_constraint_spec
    uncertainty_profile
    solver_profile
    output_profile
```

### 6.1 Vehicle case template

This is a normal portable trajectory case or Taoryx-resolved case template containing:

- Vehicle family and version.
- Fidelity.
- Variant and loadout.
- Propulsion and aerodynamic configuration.
- Controller and authority profile.
- Environment model.
- Solver settings.
- Declared model-validity envelope.

The reachability tool applies only the explicitly permitted overrides.

### 6.2 Launch locus

The source of candidate launch or branch states.

Supported forms should include:

```text
single_state
state_list
spatial_grid
state_grid
trajectory_checkpoints
parent_vehicle_release_states
sampled_distribution
```

A launch state may include:

```text
time and epoch
position and reference frame
velocity
attitude
body rates
mass and resource state
vehicle configuration
active segment
controller state
actuator state
parent or release transform
```

For trajectory checkpoints, the application should branch from deterministic checkpoints rather than reconstructing the state from selected telemetry columns.

### 6.3 Launch options

These are decision variables available at release or branch time.

Examples:

```text
launch or branch time
initial heading offset
initial flight-path angle or pitch
initial roll or bank
release orientation
separation impulse
ignition delay
controller preset
aim or lead parameter
guidance mode
```

The permitted ranges, scaling, units, and discrete choices must be declared.

### 6.4 Target locus

The set of target states to evaluate.

Supported forms:

```text
single_target_state
position_grid
relative_position_grid
state_grid
target_trajectory
target_state_distribution
```

A target state may include:

```text
position
velocity
heading or course
acceleration or maneuver state
altitude
size or success radius
required time window
```

### 6.5 Target-motion model

Initial target modes should include:

```text
stationary
constant_velocity
scripted_trajectory
maneuver_library
stochastic_ensemble
bounded_maneuver_set
```

A later differential-game mode may treat target motion as an opposing bounded control. That is a separate claim from evaluating one known or sampled target trajectory.

### 6.6 Decision variables

The inner solver may choose from a bounded set such as:

```text
launch direction
launch timing
controller preset
waypoint or lead point
guidance gains
phase-transition timing
throttle or energy schedule
bounded control-profile coefficients
```

Every result must preserve the best-known decision vector and the resulting witness trajectory.

### 6.7 Terminal success specification

A terminal objective should be a set, not only a point.

Supported terminal conditions should include:

```text
maximum miss or arrival distance
minimum or maximum vehicle speed
minimum or maximum relative speed
arrival or intercept angle corridor
heading or attitude corridor
altitude corridor
time window
remaining energy or propellant
required phase or configuration
target-region dwell time
```

Different tasks require different speed semantics:

- A rendezvous may require low relative speed.
- A terminal-energy task may require a minimum vehicle speed.
- A recovery task may require bounded descent rate.
- A simple aim-region task may care only about position and time.

The contract should support all of them without overloading one `speed_success` field.

### 6.8 Path constraints

A trajectory is not successful merely because its final position is close.

Typical hard constraints include:

```text
model-data envelope
altitude or terrain clearance
Mach range
angle of attack and sideslip
dynamic pressure
normal and lateral acceleration
body rates
control position and rate
thermal or heating proxy
resource bounds
segment and configuration validity
keep-out regions
maximum duration
numerical validity
```

Path constraints may be hard, soft with a budget, or report-only. The distinction must be explicit.

### 6.9 Uncertainty profile

Uncertainty may cover:

```text
initial-state error
wind and atmosphere
aerodynamic coefficients
mass and inertia
propulsion performance
actuator response
target state and maneuver
sensor or estimator state
```

The initial product should support three result types:

1. **Nominal:** one deterministic realization.
2. **Probabilistic:** estimated success probability over a declared distribution.
3. **Robust sampled:** success across all members of a finite declared scenario set.

Formal worst-case guarantees should be a later capability and must be labeled separately.

---

## 7. Use a continuous feasibility value, not only a Boolean

Each candidate trajectory should produce both a binary success classification and a continuous margin.

A normalized terminal-and-path violation score can be defined conceptually as:

```text
score = maximum normalized violation across all required conditions
```

Interpretation:

```text
score < 0    feasible with margin
score = 0    estimated boundary
score > 0    at least one required condition violated
```

A more descriptive result should retain the individual margins:

```text
miss_distance_margin
terminal_speed_margin
arrival_angle_margin
time_window_margin
energy_margin
model_envelope_margin
control_authority_margin
terrain_margin
```

The aggregate score helps boundary finding. The individual margins explain why the boundary has its shape.

For each query, store:

```text
success
aggregate_margin
all_constraint_margins
best decision variables
time to terminal
terminal state
minimum path margins
failure reason
witness trajectory reference
solver effort
```

---

## 8. Do not confuse “not found” with “impossible”

A simulation-and-optimization search can prove feasibility by producing a valid witness trajectory. It generally cannot prove infeasibility merely because an optimizer failed to find one.

Every evaluated point should therefore receive one of these statuses:

```text
verified_feasible
certified_infeasible
unresolved_search
invalid_query
invalid_model_region
numerical_failure
```

Definitions:

- **Verified feasible:** at least one independently replayed witness satisfies every required condition.
- **Certified infeasible:** a supported formal method or exhaustive bounded proof rules out success.
- **Unresolved search:** no witness was found within the configured search budget.
- **Invalid query:** the requested state or configuration is structurally invalid.
- **Invalid model region:** the study requests behavior outside the model's declared validity.
- **Numerical failure:** the solver could not produce a trustworthy result.

The main visualization should distinguish:

```text
verified inner envelope
estimated boundary
uncertain band
certified outside region, when available
```

This makes the product scientifically defensible and prevents an optimizer limitation from masquerading as a physical limitation.

---

## 9. Coordinate systems and standard envelope slices

The tool should store states in canonical frames but expose standard local coordinates for analysis.

### 9.1 Route-aligned frame

For a baseline launch trajectory:

```text
x: along-track tangent
y: cross-track right
z: local vertical or down, as declared
```

This is useful for stacking cross-sections along a route.

### 9.2 Line-of-sight frame

For one launch and target pair:

```text
range
azimuth or bearing
elevation
closing or opening speed
target aspect angle
```

### 9.3 Common output slices

The first release should support:

```text
range × bearing
range × target speed
range × target aspect
cross-range × downrange
altitude difference × range
launch speed × target range
launch station × target bearing
launch station × maximum feasible range
```

Higher-dimensional studies should be stored in a labeled multidimensional dataset and rendered as controlled slices rather than pretending the result is only a two-dimensional polygon.

---

## 10. The “fan” product

The specific visualization the user described should be a first-class output named an **Along-Track Reachability Fan**.

### 10.1 Construction

For each checkpoint \(s_i\) along a baseline trajectory:

1. Restore or initialize the exact vehicle state.
2. Define a route-aligned or line-of-sight frame.
3. Evaluate a target-position, target-velocity, or launch-direction slice.
4. Search permitted launch and guidance decisions.
5. Extract the verified feasible boundary and uncertain band.
6. Save representative witness trajectories.
7. Connect equivalent boundary dimensions across checkpoints.

### 10.2 Visual layers

The report should include:

- A top-down fan from selected launch stations.
- A three-dimensional envelope surface.
- A station-versus-bearing carpet plot.
- A station-versus-maximum-range plot.
- Time-to-terminal contours.
- Terminal-speed contours.
- Failure-reason regions.
- Boundary uncertainty.
- Optional target-motion overlays.

### 10.3 Query behavior

A user should be able to select any point in the map and inspect:

```text
launch state
target state
classification
best decision vector
terminal margins
path margins
failure reason
witness trajectory
control and actuator history
model and fidelity provenance
```

---

## 11. Practical solver architecture

The first implementation should be simulation-based and adaptive.

### 11.1 Outer sampler

The outer sampler chooses launch-target query points.

Supported methods should include:

```text
regular grid
polar grid
Latin hypercube or low-discrepancy design
adaptive quadtree or octree
boundary-focused active sampling
user-provided query set
```

A regular grid is useful for reproducible displays. Adaptive subdivision should concentrate work near changing feasibility classifications and sharp margin gradients.

### 11.2 Inner candidate solver

For each query, the inner solver searches permitted launch and controller decisions.

Recommended staged process:

```text
cheap deterministic coarse candidates
        ↓
retain best candidates
        ↓
local or derivative-free refinement
        ↓
independent witness replay
        ↓
margin and provenance report
```

The exact optimization library should be replaceable. The study artifact must record methods, tolerances, evaluation budgets, seeds, and restart counts.

### 11.3 Boundary extraction

The boundary module should:

- Preserve nonconvex regions.
- Preserve holes and disconnected components.
- Avoid applying a convex hull by default.
- Track unresolved cells.
- Extract contours only from compatible slices.
- Report resolution and interpolation uncertainty.

A radial maximum-range search may be offered as a faster specialized method only when the envelope is known or verified to be star-shaped in the selected slice.

### 11.4 Cache and reuse

The application should cache:

```text
compiled vehicle cases
baseline trajectory checkpoints
target trajectories
environment realizations
candidate solutions
successful witness trajectories
nearby boundary warm starts
```

Nearby target states often have nearby successful decisions. Warm-starting from neighboring witness solutions should substantially reduce the inner search cost.

### 11.5 Branching from checkpoints

When the study asks what remains reachable from points along an existing flight:

```text
run one baseline trajectory
save deterministic checkpoints
branch candidate trajectories from each checkpoint
```

The tool should not rerun the entire common prefix for every branch.

---

## 12. Multi-fidelity computation plan

The fidelity ladder is especially valuable for this application.

### 12.1 3DOF discovery

Use 3DOF to:

- Explore broad state spaces.
- Identify likely feasible regions.
- Estimate range, energy, and timing boundaries.
- Produce dense initial envelope maps.
- Generate warm starts for higher fidelity.

### 12.2 Pseudo-6DOF control-feasibility refinement

Use pseudo-6DOF to add:

- Attitude and rate response.
- Turn and pointing lag.
- Command and actuator-like limits.
- Saturation and low-authority effects.
- More credible heading and terminal-orientation feasibility.

### 12.3 Rigid-body 6DOF validation

Use full 6DOF selectively to:

- Validate boundary and corner points.
- Check physical effectors and moments.
- Check actuator rate and authority.
- Detect control coupling and loss of stability.
- Quantify how much the lower-fidelity envelope overstates capability.

### 12.4 Recommended workflow

```text
3DOF dense search
      ↓
3DOF boundary and uncertainty band
      ↓
pseudo-6DOF refinement around the boundary
      ↓
6DOF validation at representative interior, boundary, and exterior points
      ↓
cross-fidelity shrinkage and disagreement report
```

The tool should output separate envelopes rather than silently replacing one with another:

```text
energy-limited 3DOF envelope
response-limited pseudo-6DOF envelope
actuator-and-moment-limited 6DOF envelope
```

A conservative composite envelope may be produced only under an explicit rule.

### 12.5 Native handoff and table-domain preflight

The higher-fidelity handoff should be an explicit provider operation, not an
implicit upgrade inside the reduced-order solver:

```text
reduced checkpoint
      ↓
state/frame conversion
      ↓
native validity preflight
      ↓
native replay or explicit blocked result
```

Before native execution, check every declared validity axis, including:

```text
Mach or speed
altitude
angle of attack
sideslip
mass and propellant state
control and actuator authority
thermal and dynamic-pressure limits
```

The preflight must return domain margins and a named status such as
`ready`, `blocked`, or `invalid`. It must not silently clamp an out-of-domain
state into an aerodynamic or propulsion table. A conservative initialization
policy may be offered for reference replay, but it must preserve both the
original reduced state and the native replay state so that the result cannot be
misread as exact same-state validation.

### 12.6 Horizon and timeout refinement

Horizon termination is a study outcome, not an omitted candidate. Each sample
must retain a timeout flag and the result must expose the timeout query IDs.
The workbench should support a follow-up study that:

- selects only the prior timeout candidates;
- extends the horizon or changes the termination policy;
- preserves the original study and candidate IDs in provenance;
- reports which candidates resolved, timed out again, or reached another
  terminal failure;
- never silently merges the follow-up result into the original envelope.

---

## 13. Formal reachability research track

A later research track can add Hamilton-Jacobi or other formal reachable-set methods for reduced state models.

Good candidates include:

- Planar constant-speed or bounded-turn models.
- Reduced relative-position and relative-velocity systems.
- Energy-state point-mass models.
- Small-dimensional reach-avoid problems with bounded disturbances.

The formal solver can produce certified inner or outer sets and compare them with the simulation-based workbench.

Full rigid-body 6DOF state spaces, controller states, target states, and environment dimensions are generally too large for a naive dense grid formulation. The initial production tool should therefore not wait for a universal formal solver.

---

## 14. Target motion and uncertainty products

The workbench should produce several clearly different envelope types.

### 14.1 Nominal moving-target envelope

The target follows one declared trajectory or constant-velocity model.

Output:

```text
best achievable terminal margin for that exact target motion
```

### 14.2 Scenario-set envelope

The same launch-target query is evaluated across a finite set of target maneuvers and disturbances.

Output options:

```text
all-scenarios-success inner envelope
success-count or success-fraction map
worst observed margin
scenario-specific failure reasons
```

### 14.3 Probabilistic envelope

Target motion, wind, or model parameters are sampled from declared distributions.

Output:

```text
estimated probability of success
confidence interval
sample count
calibration and convergence diagnostics
```

### 14.4 Bounded-adversary envelope

A future formal or game-theoretic mode may ask whether success is possible against all target controls within a bounded set.

This must not be inferred from a small Monte Carlo sample. It receives its own capability and evidence label.

---

## 15. Output data model

One study should create an immutable artifact bundle.

```text
study_manifest.json
resolved_study.json
resolved_vehicle_case.json
provider_capabilities.json
realized_fidelity.json
solver_profile.json
success_spec.json
constraint_spec.json
uncertainty_profile.json

queries.parquet
candidate_evaluations.parquet
best_results.parquet
constraint_margins.parquet
boundary_components.json
uncertain_cells.json

witnesses/
    <query-id>/trajectory.parquet
    <query-id>/controls.parquet
    <query-id>/events.json
    <query-id>/evaluation.json

multidimensional_envelope.zarr
footprints.geojson
boundary_meshes/
plots/
report.html
reproduction.txt
```

Each query result should include:

```text
query identifiers and coordinates
launch-state reference
target-state reference
policy scope
classification
best aggregate margin
all component margins
best decision vector
time to terminal
terminal state
minimum path margins
failure or unresolved reason
witness reference
provider and model versions
simulation and solver effort
```

---

## 16. Standard visualizations

The first release should generate:

1. **Top-down verified reachability footprint**
   - Feasible interior.
   - Estimated boundary.
   - Uncertain cells.
   - Failure-reason coloring.

2. **Polar fan**
   - Bearing versus maximum verified range.
   - Optional target-speed or altitude slices.

3. **Range-speed-aspect maps**
   - Range versus target speed for fixed aspect angles.
   - Range versus aspect for fixed target speeds.

4. **Along-track fan**
   - Envelope slices at selected checkpoints.
   - Launch station versus feasible range.
   - Launch station versus heading tolerance.

5. **Three-dimensional envelope**
   - Downrange, cross-range, and altitude.
   - Optional animation over launch time or checkpoint.

6. **Margin maps**
   - Terminal speed.
   - Time to terminal.
   - Remaining energy.
   - Control-authority margin.
   - Model-envelope margin.

7. **Witness trajectory gallery**
   - Easy interior case.
   - Near-boundary success.
   - Near-boundary failure.
   - Minimum-range case.
   - Maximum-range case.
   - Highest target-speed case.

8. **Cross-fidelity comparison**
   - Overlay of 3DOF, pseudo-6DOF, and 6DOF boundaries.
   - Boundary displacement and reason codes.

### 16.1 Alpha 3 Required Plot Bundle

The Alpha 3 completion bundle is intentionally smaller than the full R5
visualization catalog. It must be generated from the saved envelope artifact,
not by rerunning the vehicle, and must contain:

```text
flown-trajectories.png
search-coverage.png
terminal-capability.png
fidelity-progression.png    # when two or more tiers are supplied
plot-manifest.json
```

The four views answer distinct completion questions:

| View | Completion question |
| --- | --- |
| Flown trajectories | What did the vehicle actually fly, including boost/glide phase changes and feasible versus failed candidates? |
| Search coverage | Did the run realize the declared search space, or are apparent gaps just missing candidates? |
| Terminal capability | Where did candidates terminate, and how do feasible, infeasible, invalid, and unresolved outcomes separate? |
| Fidelity progression | How did terminal points, feasible count, and maximum capability move between 3DOF and pseudo-6DOF? |

Plot manifests must identify the source artifact, fidelity, plot names, DPI,
and any intentionally omitted view, such as flown trajectories when compact
artifacts do not contain trajectory tables. A plot is not evidence of a
successful run unless its source artifact and classification legend are
available alongside it.

---

## 17. Example study specification

The syntax should remain provider-neutral while allowing provider-specific extensions.

```yaml
schema: trajectory.reachability-study/v1alpha1

provider:
  preferred: taoryx

vehicle_case:
  template: cases/example_vehicle.yaml
  fidelity: point_mass_3dof
  loadout: nominal
  controller:
    authority_profile: commanded
    preset: terminal_guidance_nominal

launch_locus:
  kind: trajectory_checkpoints
  source_case: cases/example_launch_route.yaml
  sampling:
    mode: fixed_time_interval
    interval: ${CHECKPOINT_INTERVAL}

launch_options:
  initial_heading_offset:
    min: ${MIN_HEADING_OFFSET}
    max: ${MAX_HEADING_OFFSET}
  initial_flight_path_angle:
    min: ${MIN_FLIGHT_PATH_ANGLE}
    max: ${MAX_FLIGHT_PATH_ANGLE}
  launch_delay:
    min: ${MIN_LAUNCH_DELAY}
    max: ${MAX_LAUNCH_DELAY}

target_locus:
  kind: relative_position_grid
  frame: route_aligned
  downrange: ${DOWNRANGE_AXIS}
  crossrange: ${CROSSRANGE_AXIS}
  altitude_offset: ${ALTITUDE_AXIS}

target_motion:
  kind: constant_velocity
  speed: ${TARGET_SPEED_AXIS}
  heading_relative_to_route: ${TARGET_ASPECT_AXIS}

success:
  horizon: ${MAX_TIME_TO_TERMINAL}
  terminal:
    distance_max: ${MAX_TERMINAL_DISTANCE}
    vehicle_speed_min: ${MIN_TERMINAL_VEHICLE_SPEED}
    arrival_angle: ${ARRIVAL_ANGLE_CORRIDOR}
  path:
    require_model_validity: true
    require_control_validity: true
    constraints: terminal_mission_limits

policy_scope:
  kind: parameter_optimized_controller
  decision_variables:
    - launch_options.initial_heading_offset
    - launch_options.initial_flight_path_angle
    - launch_options.launch_delay

solver:
  outer_sampler: adaptive_cells
  inner_search: deterministic_multistart
  preserve_unresolved: true
  witness_replay: required

outputs:
  slices:
    - range_bearing
    - range_target_speed
    - launch_station_max_range
  retain_witnesses: boundary_and_extremes

extensions:
  org.taoryx:
    checkpoint_branching: true
```

The actual schema should use typed arrays and quantities rather than textual placeholder substitution. The example illustrates the conceptual boundaries.

---

## 18. CLI and Python surface

### 18.1 CLI

```bash
traj-envelope validate studies/example.yaml
traj-envelope resolve studies/example.yaml
traj-envelope run studies/example.yaml
traj-envelope refine artifacts/example-study
traj-envelope verify artifacts/example-study
traj-envelope render artifacts/example-study

traj-envelope query artifacts/example-study \
  --launch-station <station> \
  --target-range <range> \
  --target-bearing <bearing> \
  --target-speed <speed>

traj-envelope compare-fidelities studies/example.yaml \
  --fidelities 3dof,p6dof,6dof
```

### 18.2 Python

```python
study = resolve_reachability_study("studies/example.yaml")
result = run_reachability_study(study)

query = result.query(
    launch_station=launch_station,
    target_range=target_range,
    target_bearing=target_bearing,
    target_speed=target_speed,
)

assert query.classification in {
    "verified_feasible",
    "certified_infeasible",
    "unresolved_search",
}
```

A later online interface may load a precomputed envelope or surrogate and answer state queries with bounded latency.

---

## 19. Package boundary

The application should depend on portable trajectory contracts rather than import Taoryx internals wherever practical.

```text
applications / reachability workbench
               |
               v
portable trajectory-provider and session contracts
               |
               v
Taoryx provider facade
               |
               v
Taoryx runtime
```

Suggested repository structure:

```text
apps/trajectory-envelope/
    schemas/
    src/
        contracts/
        launch_loci/
        target_models/
        success_specs/
        samplers/
        candidate_solvers/
        boundary/
        uncertainty/
        visualization/
        reports/
        providers/
    examples/
    tests/
    benchmarks/
```

The reachability application owns:

- Study configuration.
- Launch and target grids.
- Optimization and sampling.
- Boundary extraction.
- Uncertainty aggregation.
- Visualizations and reports.

Taoryx owns only generic simulation primitives:

- Resolve and compile a vehicle case.
- Initialize from a declared state or checkpoint.
- Run or step deterministically.
- Branch from checkpoints.
- Execute batches efficiently.
- Apply controls and controller presets.
- Return telemetry, events, validity, and evaluation data.

No envelope-specific logic needs to enter the Taoryx physics runtime.

---

## 20. Validation plan

### 20.1 Analytical reference problem

Start with a simple constant-speed point vehicle and a fixed horizon whose unconstrained terminal footprint is known.

This validates:

- Coordinate transforms.
- Grids and adaptive refinement.
- Boundary extraction.
- Margin sign.
- Visualization.
- Reproducibility.

Then add bounded turn rate or acceleration as a more demanding reference case.

### 20.2 Witness replay

Every point labeled verified feasible must be independently rerun from its resolved initial state and decision vector.

The replay must reproduce:

```text
terminal classification
terminal margins
path margins
event sequence
trajectory hash within declared numerical tolerance
```

### 20.3 Inside/outside boundary probes

For each boundary component:

- Sample points slightly inside.
- Sample points on the estimated boundary.
- Sample points slightly outside.
- Rerun with increased search effort.
- Measure classification consistency and unresolved width.

### 20.4 Resolution convergence

Run at increasing outer-grid resolution and inner-search budget.

Report:

```text
boundary displacement
feasible area or volume change
unresolved-band change
extreme-range change
terminal-margin change
compute cost
```

### 20.5 Multi-start and optimizer sensitivity

The workbench should quantify how often different starts find different solutions. Regions with strong optimizer sensitivity receive a wider uncertainty classification.

### 20.6 Cross-fidelity validation

Select:

```text
interior successes
near-boundary successes
unresolved points
nominal exterior points
extreme target-speed points
extreme turn points
```

and rerun them at the next fidelity.

### 20.7 Model-validity enforcement

A trajectory outside declared aerodynamic, propulsion, actuator, or environment validity may be displayed as a research extrapolation only when explicitly allowed. It may not count toward the qualified envelope by default.

### 20.8 Monotonic properties where applicable

The validator may check properties that should hold under a restricted formulation, for example:

- Increasing the allowed time horizon should not shrink an existential nominal feasible set when all other constraints and policy options are unchanged.
- Adding a hard constraint should not enlarge the verified feasible set.
- Tightening a terminal tolerance should not enlarge the verified feasible set.

These checks must be disabled when changed policies, finite search budgets, or time-dependent dynamics invalidate the assumption.

---

## 21. Initial example suite

### Example A: Known planar footprint

```text
simple_constant_speed_terminal_footprint
```

Purpose:

- Validate the envelope machinery against a known reachable region.
- Exercise inner, boundary, exterior, and unresolved classifications.
- Exercise explicit impact radius and impact-speed goals independently of a
  vehicle-specific table or source mission.

### Example B: Moving-objective range-speed-aspect envelope

```text
spectre_style_moving_objective_envelope_3dof
```

Axes:

```text
initial range
target speed
target aspect angle
```

Outputs:

- Verified success region.
- Time-to-terminal.
- Terminal-speed margin.
- Best initial heading offset.
- Failure-reason map.

### Example C: Along-route launch fan

```text
spectre_style_along_route_reachability_fan_3dof
```

Mission:

- Generate a baseline route.
- Checkpoint at fixed intervals.
- Branch launch or retarget attempts from every checkpoint.
- Produce a fan or tube whose cross-sections show what remains reachable.

### Example D: Multi-fidelity boundary contraction

```text
reference_family_cross_fidelity_reachability
```

Purpose:

- Generate a dense 3DOF envelope.
- Refine with pseudo-6DOF.
- Validate selected cases with 6DOF.
- Explain contraction caused by turn response, actuator limits, moments, and control saturation.

The Alpha 3 realization of this example is the staged X-15 demonstration. It
must include booster burn, coast, release mass drop, unpowered glide, timeout
refinement, native table-domain preflight, and explicit native-blocked results.

### Example E: Robust or probabilistic envelope

```text
moving_objective_with_wind_and_model_uncertainty
```

Purpose:

- Compare nominal, finite-scenario robust, and estimated probability-of-success products.
- Demonstrate that each receives a different label and artifact.

---

## 22. Maturity ladder

| Level | Meaning |
|---|---|
| **R0 — Defined** | Study contracts, terminology, classifications, and output semantics are documented. |
| **R1 — Single-State Footprint** | One launch state, stationary target region, deterministic 3DOF, fixed controller, reproducible grid and plots. |
| **R2 — Moving-Target Slices** | Constant-velocity target, range-speed-aspect studies, bounded decision search, witness trajectories, continuous margins. |
| **R3 — Along-Track Fan** | Checkpoint branching along a trajectory, stacked envelope slices, adaptive refinement, nonconvex boundary preservation. |
| **R4 — Multi-Fidelity and Uncertainty** | 3DOF discovery, pseudo-6DOF refinement, 6DOF validation, nominal/probabilistic/scenario-set products. |
| **R5 — Qualified Workbench** | Independent witness replay, convergence evidence, inner/uncertain/certified classifications, one-command artifacts, provider-neutral contracts. |
| **R6 — Online Envelope Service** | Precomputed or learned envelope surrogate with query latency targets, calibration, versioning, and runtime invalidation rules. |

The long-term initial workbench target remains **R3**, and the first
defensible “good to go” research release remains **R5**. The Alpha 3
completion goal in section 1.1 is a narrower vertical slice: executable
single-study semantics, timeout and outcome handling, reduced-order
multi-fidelity evidence, selected native handoff checks, and artifact-backed
Matplotlib plots. It is not a claim that Alpha 3 has reached R3 along-track
branching or R5 qualification.

---

## 23. R5 definition of done

> **The Trajectory Reachability Workbench is Qualified when a user can select a versioned vehicle case and fidelity; define a launch-state locus, target-state and motion model, controller or policy scope, decision variables, path constraints, terminal success set, time horizon, and uncertainty treatment; generate deterministic and reproducible reachability studies through the portable provider interface; and inspect verified feasible regions, unresolved regions, certified infeasible regions when supported, continuous terminal and path margins, failure reasons, and replayable witness trajectories.**
>
> **For along-trajectory studies, the tool must branch from exact checkpoints, compute and stack reachability slices, preserve nonconvex and disconnected regions, and render the resulting fan or tube without assuming that it must monotonically shrink. The qualified example suite must include a known reference problem, a moving-target range-speed-aspect envelope, an along-route fan, and a multi-fidelity comparison.**

---

## 24. The shortest explanation

The user-facing explanation can be:

> **This tool takes a Taoryx vehicle, a set of possible launch states, a moving or stationary target definition, and precise success criteria. It fans out many candidate trajectories, searches the allowed launch and guidance choices, and builds a map of the relative states for which at least one valid trajectory succeeds. When the launch states come from checkpoints along a route, those maps are stacked into a dynamic reachability fan showing what remains possible at every point in the flight.**

The pointed scientific caveat is:

> **A successful witness proves that a point is feasible for the declared model and controller class. Failure to find a witness does not prove physical impossibility unless the study uses a method capable of certifying infeasibility.**

---

## 25. Archetype-specific reachability profiles

The workbench computes one mathematical object, but every vehicle archetype
registers a profile declaring its state dimensions, phases, decisions,
constraints, controller scope, terminal semantics, coordinates, and products.
Reachability is not maximum distance alone:

The machine-readable registration surface is
[`verification/reachability_profile_catalog.yaml`](../../verification/reachability_profile_catalog.yaml).
Every family in that catalog must select a profile and may remain
`cataloged`, `data_ready`, `3dof_ready`, `pseudo_6dof_ready`, `mission_ready`,
or `profile_qualified` independently of the maturity of neighboring families.

```text
E = E(vehicle state, phase, resources, target-relative state,
      controller class, horizon, success set, path constraints)
```

### Feasible intervals and study semantics

For each slice, retain one or more feasible intervals rather than only a radial
maximum:

```yaml
direction: {bearing_deg: 40, elevation_deg: -3}
solutions:
  - {mode: direct, minimum_range_km: 18, maximum_range_km: 63}
  - {mode: lofted, minimum_range_km: 81, maximum_range_km: 104}
```

This preserves inner dead zones, disconnected regions, trajectory families,
minimum-range holes, and controller-specific solutions. Convex hulls and simple
radial maxima are prohibited as default representations.

Every study declares its semantics: `first_pass`, `bounded_time`,
`bounded_resource`, `bounded_path`, `eventual_capture`, or `robust_capture`.
The default tactical product is first-pass, bounded-time, bounded-resource
reachability with no implicit loiter or reattack.

### Initial vehicle profiles

| Profile | Dominant state/constraint | Primary products |
| --- | --- | --- |
| `reachability.multirotor.capture_bounded_time.v1` | Relative position/velocity, altitude, yaw, battery, wind, stopping distance, tilt/thrust limits | Position capture, low-relative-speed capture, time, battery reserve, wind robustness |
| `reachability.fixed_wing.first_pass_capture.v1` | Velocity-relative bearing, speed, heading, energy, turn radius, stall/load-factor, fuel | Forward crescent/annulus, left/right solutions, terminal heading/speed, first-pass route |
| `reachability.boost_glide.prelaunch_capture.v1` | Attached/boost/coast/separation/glide phase, propellant, energy, minimum burn/separation | Prelaunch, release-state, direct/lofted/boost-glide lobes |
| `reachability.boost_glide.release_state.v1` | Booster-produced release position, velocity, attitude, and energy | Release-state acceptability and handoff envelope |
| `reachability.glider.post_release_retarget.v1` | Glide energy, bank authority, dynamic pressure, terminal speed | Post-release retarget and energy corridor |
| `reachability.high_energy_glider.first_pass.v1` | Specific energy, crossrange, bank reversals, heating, Mach, dynamic pressure | Downrange/crossrange, energy and terminal-speed contours |
| `reachability.ballistic.launch_footprint.v1` | Launch azimuth/elevation, burn/cutoff, atmosphere, payload and terminal angle | Passive terminal footprint with low/high-arc branches |
| `reachability.passive_object.terminal_footprint.v1` | Initial attitude/rates and uncertainty, no meaningful control decisions | Probabilistic impact footprint and terminal distributions |

### Family registration matrix

The initial full-family plan is:

| Family lane | Configurations | Primary profile |
| --- | --- | --- |
| Validated multirotor | Hummingbird | `multirotor.capture_bounded_time` |
| Validated powered fixed wing | B747, Skywalker X8 | `fixed_wing.first_pass_capture` |
| Validated rocket/high-energy aircraft | X-15 | `fixed_wing.first_pass_capture`, `high_energy_glider.first_pass`, `boost_glide.release_state` |
| Source reference fixed wing | F-16 S-119, planned A320 | `fixed_wing.first_pass_capture` |
| Source reference lifting body | HL-20 Mod K | `glider.post_release_retarget`, `high_energy_glider.first_pass` |
| Electric multirotor | Bolt, Bolt-M, Anvil | `multirotor.capture_bounded_time` |
| Electric single-main-rotor | Ghost-X | `rotorcraft.capture_bounded_time` |
| Deployable propeller fixed wing | ALTIUS-600, ALTIUS-700 ISR, ALTIUS-700M | `fixed_wing.first_pass_capture` |
| Fuel turbojet fixed wing | Barracuda-100, -250, -500 | `fixed_wing.first_pass_capture` |
| Fuel vectored jet VTOL | Roadrunner | `jet_vtol.transition_capture` |
| Fuel turbofan fixed wing | Fury/FQ-44 | `fixed_wing.first_pass_capture` |
| Selectable-energy tailsitter | Omen | `jet_vtol.transition_capture` |
| Series-hybrid dual tiltrotor | Thunder/Halo | `tiltrotor.conversion_capture` |
| Planned light propeller aircraft | C172-class | `fixed_wing.first_pass_capture` |
| Planned small business jet | Learjet-24-class | `fixed_wing.first_pass_capture` |
| Planned generic rotorcraft | R44-class | `rotorcraft.capture_bounded_time` |
| Planned generic tiltrotor | XV-15-class, V-22-class | `tiltrotor.conversion_capture` |
| Boost/glide | Kestrel glider, generic rocket-plus-glider | `boost_glide.prelaunch_capture`, `glider.post_release_retarget` |
| Ballistic rocket | Ballistic rocket, two-stage ballistic rocket, ballistic reentry | `ballistic.launch_footprint` |
| Passive/tumbling body | Tumbling ballistic body, passive release body | `passive_object.terminal_footprint` |
| Spacecraft wheel-primary | 6U observer, agile imager, wheel-primary reference | `spacecraft.orbit_visibility`, `spacecraft.attitude_pointing` |
| Spacecraft thruster-primary | SPHERES-like free flyer, thruster-primary reference | `spacecraft.freeflyer_capture`, `spacecraft.attitude_pointing` |
| Spacecraft hybrid | MarCO-like hybrid, hybrid reference | `spacecraft.orbit_visibility`, `spacecraft.attitude_pointing` |
| Fleet reductions | Hummingbird fleet, Bolt fleet, generic multirotor population | `fleet.multi_agent_capture` |

Vehicle-relative coordinates are profile-owned: local ENU/NED for multirotors,
velocity-relative range/bearing/elevation for fixed wing, launch-to-aim
downrange/crossrange/altitude for boost-glide, and range/azimuth/terminal
angle/speed for ballistic bodies.

### Preserve solution modes and limiting factors

For each query, retain several nondominated witnesses where they differ in
arrival time, terminal reserve, control effort, path length, robustness, or
mode. Modes may include direct, left/right turn, lofted, low/high arc,
boost-heavy, glide-heavy, and single/multiple bank reversal.

Every boundary cell and failed query carries limiting-factor attribution, such
as `turn_radius_limited`, `braking_distance_limited`, `fuel_limited`,
`terminal_speed_limited`, `crossrange_limited`, `actuator_rate_limited`,
`stall_limited`, `separation_not_complete`, `aero_table_boundary`, or
`controller_search_unresolved`. This turns the envelope into a diagnostic
artifact rather than a picture alone.

### Phase-aware along-track fans

Each checkpoint stores position/velocity, attitude/rates, flight phase, mass
and resources, active topology, controls, controller mode, and environment.
Branching must preserve exact deterministic checkpoint state. A fan may expand,
contract, split, rotate, or change discontinuously at ignition, staging,
separation, or configuration transitions; monotonic tightening is never
assumed.

### Vehicle-specific profile declaration

```yaml
profile:
  id: reachability.boost_glide.first_pass.v1
  semantics: {capture: first_pass, maximum_time_s: 240, allow_loiter: false}
  modes: [boost, coast, separation, glide, terminal]
  coordinates: {primary: [downrange, crossrange, altitude]}
  decisions: [launch_azimuth, launch_elevation, cutoff_speed, glide_bank_schedule]
  products: [feasible_range_intervals, terminal_speed_contours, limiting_factor_map]
```

The profile is qualified only when its state/phase contract, horizon, decision
scope, terminal conditions, constraints, analytical sanity cases, witness
replay, adaptive-boundary convergence, disconnected-region preservation, and
failure attribution all pass.

## 26. Technical grounding

The neutral control-theory formulation is a reach-avoid or capture-basin problem: identify states from which a controlled system can enter a target set while remaining within state and path constraints. Hamilton-Jacobi methods are a principled option for low-dimensional reduced systems, but conventional gridded formulations scale poorly as state dimension grows. That makes a simulation-based, adaptive, multi-fidelity workbench the practical first architecture for Taoryx, with formal reachability retained as a later reduced-order validation track.
