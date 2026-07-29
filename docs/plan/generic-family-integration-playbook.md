# Generic Taoryx family-integration playbook

Status: Alpha 3 planning and implementation notes

This is the reusable composition-level procedure for bringing a new vehicle
family into Taoryx. It applies to source-grounded aircraft such as the F-16 and
HL-20, native four-family vehicles, and later rotorcraft, rockets, spacecraft,
and surrogates.

The word *integrated* has a strict meaning here. A package is not integrated
because it can be parsed or because a single trajectory runs. Integration is a
progression from immutable source data to a diagnosed, fidelity-specific,
showcaseable executable model.

## 1. The generic integration pipeline

```text
source package and provenance
    ↓
canonical plant adapter
    ↓
mass, geometry, frames, resources, and envelope
    ↓
trim / equilibrium operating-point catalog
    ↓
source-vector and equation diagnostics
    ↓
effector and actuator model
    ↓
controller and guidance binding
    ↓
validated fidelity reductions
    ↓
objective mission and independent evaluator
    ↓
showcase packet and promotion decision
```

Every arrow is an evidence boundary. The downstream layer inherits only the
claims explicitly earned by its upstream inputs.

## 2. Data required from every family

The intake record should distinguish `required`, `optional`, `derived`,
`assumed`, and `unknown`. Unknown values remain unavailable; they are not
replaced with zeros or plausible-looking constants.

### 2.1 Identity and provenance

- Family ID, version, display name, physical family, and source role.
- Source archive/repository, revision, retrieval record, license/notices.
- Exact source/package/aerodynamic/member hashes.
- Source check cases, reference vectors, and known limitations.
- Evidence class for every subsystem: source, derived, correlated, estimated,
  synthetic, or unavailable.

### 2.2 Geometry and mass properties

- Reference area, lengths, span or rotor geometry, and configuration state.
- Dry, nominal, payload, propellant/fuel, and trapped-resource masses.
- Center of gravity and reference point.
- Full inertia tensor with frame, units, symmetry assumptions, and validity
  range.
- Deployment, staging, loadout, or configuration transformations.

### 2.3 Environment, frames, and equations

- Body and navigation frames, handedness, quaternion order, and time epoch.
- Atmosphere, gravity, Earth/orbital model, wind, and contact assumptions.
- Force and moment decomposition, sign conventions, coefficient conventions,
  interpolation, extrapolation, and clamping rules.
- State equations, resource equations, event semantics, and integration hooks.

### 2.4 Aerodynamics and propulsion

- Coefficient tables or source functions with domains and units.
- Static, rate, control, and cross-axis derivatives where available.
- Propulsion thrust/power maps, throttle convention, mass flow or energy draw,
  and burnout/depletion behavior.
- Lift/drag/source-force convention; never assume that a body-axis `Cx` is a
  wind-axis `Cd`.

### 2.5 Controls and effectors

- Semantic controls and physical effectors, including topology.
- Sign, neutral/trim, limits, rates, lag, latency, health, and failure state.
- Effector-to-wrench effectiveness, allocation priorities, and saturation
  behavior.
- Explicit declaration when the plant accepts direct forces/moments instead of
  physical effector commands.

## 3. Fidelity contracts and automatic lowering

Taoryx should support a requested fidelity plus an explicit fallback policy:

```yaml
fidelity_request:
  requested: rigid_body_6dof
  fallback_policy: validated_lower_only
  minimum_acceptable: point_mass_3dof
```

The resolver evaluates candidates in descending order:

```text
requested tier
    ↓ if unavailable or not qualified
validated pseudo-6DOF reduction
    ↓ if unavailable or not qualified
validated 3DOF reduction
    ↓ if unavailable or not qualified
reject with structured capability diagnostic
```

Lowering must never be silent. The resolved case records:

- requested fidelity;
- selected fidelity;
- fallback policy and reason;
- parent plant and reduction artifact hash;
- retained and omitted physics;
- reduction envelope and expected disagreement;
- evidence tier and qualification status.

Automatic lowering is allowed only when the reduction artifact has passed its
own equivalence tests. A planned reduction, a controller response sketch, or a
matrix with compatible dimensions is not executable fallback support.

The provider-neutral selection seam is implemented by
`taoryx.trajectory.select_validated_fidelity`. It returns the requested tier,
the selected tier, the fallback reason, every considered profile, and the
reduction artifact reference. Its eligible evidence statuses are intentionally
closed: `development_screen_passed` and `equivalence_pending` cannot trigger
automatic lowering. The F-16 reduction screen therefore remains useful
development evidence while correctly producing no fallback eligibility.

### 3.1 Required data by tier

| Tier | Minimum data | What it can claim |
|---|---|---|
| 3DOF | mass/resource law, environment, force/lift-drag-thrust model, translational state, envelope, terminal geometry | center-of-mass path, energy, range, altitude, resource use |
| Named pseudo-6DOF | all 3DOF data plus quaternion/body-rate state, named response law, time constants/rate limits, control-intent mapping, calibration sweeps | declared attitude/rate response and limits; no physical moment or effector claim |
| 6DOF direct-wrench | rigid-body mass/inertia, force/moment model, direct wrench interface, frame/sign contract, resource law | coupled translation/rotation under injected or abstract wrench; no physical effector realization |
| 6DOF physical-effectors | all direct-wrench data plus effectors, effectiveness, allocator, actuator limits/dynamics, trim controls, achieved wrench telemetry | physical control authority, allocation, saturation, and nonlinear response inside the validated envelope |

The fourth tier is intentionally separate from direct-wrench 6DOF. A direct
wrench model may be useful and numerically correct while still being a control
screen rather than an actuator-realizable model.

### 3.2 Family-specific additions

- Powered fixed-wing: propulsion/throttle and lift/drag, alpha/beta limits,
  surface geometry or declared direct-wrench boundary, and turn/climb energy
  authority.
- Unpowered lifting body: release state, glide polar/source force graph,
  bank/crossrange authority, energy corridor, and surface allocation if
  physical control is claimed.
- Rotorcraft/multirotor: rotor/motor geometry, thrust and reaction torque,
  RPM/lag, battery or fuel, allocation matrix, and contact model.
- Rocket/booster: stages, thrust/mass-flow law, gimbal or attitude control,
  separation events, propellant resources, and target-state contract.
- Spacecraft: epoch/frame, orbit force model, actuator geometry, momentum or
  propellant resources, disturbance torques, and domain-specific terminal
  states.
- Ballistic/tumbling body: orientation-dependent projected area, inertia,
  aerodynamic moments, release state, and impact contract. Tumble cannot be
  physically proven below rigid-body 6DOF.

## 4. Trim and operating-point setup

Trim is a first-class artifact, not a controller initialization convenience.
Each operating point should contain:

```yaml
operating_point:
  id: f16_subsonic_cruise_01
  environment: {altitude_m: ..., mach: ..., atmosphere: ...}
  target: {flight_path_angle_rad: 0.0, turn_rate_rad_s: 0.0}
  state: {...}
  effectors: {...}
  residuals: {translation: ..., rotation: ..., kinematic: ...}
  solver: {method: ..., tolerances: ..., iterations: ...}
  validity: qualified | extended | failed
```

The generic trim procedure is:

1. Select a declared environment and target motion.
2. Seed state and physical effector values from source data or a nearby
   operating point.
3. Solve the nonlinear equilibrium equations using actual effectors when the
   tier claims physical control.
4. Re-evaluate the full plant at the proposed solution.
5. Report separate translational, rotational, and kinematic residuals.
6. Check mass properties, table domains, resource availability, and envelope.
7. Store the state, effector vector, residuals, solver settings, and hashes.
8. Generate neighboring operating points only after the seed point passes.

For a pseudo-6DOF or 3DOF reduction, the trim must still originate from a
qualified parent or an explicitly declared reduced-order equilibrium. The
reduction may simplify the dynamics, but it may not invent a trim silently.

## 5. Diagnostics before controllers

Every family should have a diagnostic ladder:

1. **Load diagnostic:** package hashes, schemas, units, frames, dimensions,
   and required resources.
2. **Static source diagnostic:** pointwise coefficient/force/moment vectors,
   table domains, interpolation, and source convention checks.
3. **Mass/inertia diagnostic:** positive masses, valid CG, positive-definite
   inertia, quaternion norm, and frame consistency.
4. **Trim diagnostic:** residual decomposition and neighboring-point continuity.
5. **Derivative diagnostic:** centered perturbations at two step sizes,
   derivative consistency, and state/effector ordering.
6. **Authority diagnostic:** requested versus achievable force/wrench,
   allocator residual, rank/conditioning, and saturation.
7. **Resource diagnostic:** actual consumption, no negative balance, and
   capability loss after depletion.
8. **Replay diagnostic:** deterministic batch/step parity, event ordering,
   timestep convergence, and no hidden resets.
9. **Mission diagnostic:** independent truth-based objective results, terminal
   contract, envelope margins, and failure taxonomy.

The diagnostic result should identify the first failing layer. A later mission
failure must not obscure a bad trim or a source-frame mismatch.

## 6. Controller and guidance binding

The generic composition is:

```text
mission objective / waypoint
    ↓
guidance: path, energy, heading, altitude, or orbit error
    ↓
kinematic demand: velocity, flight path, bank, acceleration, or pointing
    ↓
attitude/rate controller: PID, LQR, or declared response law
    ↓
desired wrench or direct effector demand
    ↓
allocator and actuator model, if physical
    ↓
nonlinear plant
```

Controllers are family and fidelity bindings, not properties that a source
package automatically possesses. A 3DOF controller may command achievable
acceleration or bank. A pseudo-6DOF controller may command attitude/rates. A
direct-wrench 6DOF controller may command generalized forces/moments. A
physical 6DOF controller must pass through an allocator and actuator dynamics.

The resolved case must record the controller path, gain/tuning provenance,
state/reference scaling, trim point, authority mode, and whether direct wrench
injection is active.

## 7. Showcase promotion

Showcases are the final integration layer, not the integration test itself.
Each family needs:

- a start contract: trim, release, hover, ground, orbit, or separation;
- ordered segments and physical events;
- independent truth-based objective evaluation;
- terminal state and dwell requirements;
- controls/effectors exercised at the selected tier;
- envelope, resource, convergence, and batch/step evidence;
- exact claim/nonclaim and evidence class;
- reproducible artifacts and plots.

Promotion sequence:

```text
diagnostic packet
    → trim/operating-point packet
    → fidelity reduction packet
    → controller/authority packet
    → nominal showcase
    → fixed perturbations
    → cross-fidelity comparison
    → family promotion
```

The F-16 showcase should begin with subsonic airborne trim, acceleration,
right/left energy turns, and stabilized arrival. The HL-20 showcase should
begin with release, trim capture, bank reversals, energy shaping, and a
terminal energy handoff—not touchdown.

## 8. Concrete next work for F-16 and HL-20

### F-16 S-119

1. Promote the existing source replay operating point into the common trim
   catalog.
2. Add explicit subsonic trim residual and derivative reports.
3. Define bounded elevator/aileron/rudder/throttle actuator overlays.
4. Validate local effectiveness and constrained allocation.
5. Generate a named pseudo-6DOF response reduction and a 3DOF performance
   reduction from the same operating-point sweep.
6. Run the powered-fixed-wing racetrack template at each validated tier.
7. Render the evidence board with direct-wrench versus physical-effector path
   clearly labeled.

### HL-20 Mod K

1. Promote the existing source glide-trim evidence into the common operating-
   point catalog.
2. Validate the seven-surface logical allocator against the source force/moment
   graph.
3. Add bounded surface actuator overlays and allocation residual telemetry.
4. Build the energy-glide 3DOF and attitude-response pseudo-6DOF reductions
   only after parent sweeps pass.
5. Define release, bank-reversal, energy-corridor, and handoff objectives.
6. Run the lifting-body showcase without landing/contact claims.
7. Add cross-fidelity disagreement and boundary witnesses.

## 9. Definition of integrated

A family is generically integrated when:

- its source and conventions are immutable and reproducible;
- at least one operating point trims with diagnosed residuals;
- the requested fidelity resolves explicitly or fails explicitly;
- every lower tier is backed by a validated reduction artifact;
- the controller path and physical/direct-wrench boundary are visible;
- resources, limits, and authority are enforced;
- independent truth evaluation produces a complete mission result; and
- a self-contained showcase packet can be regenerated from one command.

Until then, the family remains source-loaded, source-replayed, contract-ready,
or reduction-pending—whichever is the highest honestly earned state.

####
