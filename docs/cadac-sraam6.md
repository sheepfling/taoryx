# CADAC SRAAM6 → Taoryx vehicle plug-in

## Status

The standard CADAC `SRAAM6` four-fin engagement is an exact runnable Taoryx Mission Composition model:

```text
provider_id: cadac
model_id: cadac.sraam6.missile
source actor: MISSILE6
embedded source actor: TARGET3
fidelity: rigid_body_6dof_surface_allocated
realization: cadac-standard-four-fin
mission: air_intercept
executors: cadac.sraam6.standard_fin.batch, cadac.sraam6.standard_fin.session
```

“Runnable” means the source-grounded Python reconstruction is registered through the exact common runner. It does not mean compiled-CADAC numerical equivalence has been promoted.

## Source boundary

The source case contains one `MISSILE6` followed by one `TARGET3`. The compatibility runtime preserves that vehicle order and the declared module order:

```text
environment
kinematics
aerodynamics
propulsion
seeker
guidance
control
actuator
tvc
forces
euler
newton
intercept
```

The missile therefore consumes the target packet present at the missile's source-order slot, and the target refreshes its packet after its own pass. The target is returned as a separate root trajectory object because it exists at scenario initialization; no release or deployment lineage is invented.

## Persistent composition and native sensor delivery

The executable four-fin realization supports both `batch` and persistent
`step` operation. A session retains the MISSILE6 rigid-body, fin actuator,
source controller, dynamic seeker, TARGET3, source event cursors, and
previous-target-pass packet state. It accepts only holds that are exact
multiples of the source `int_step`; this avoids inventing fractional
controller or seeker passes.

At each committed source substep, the session publishes the typed
`sraam6-native-relative-state` `relative-state-track` through the Taoryx
`SensorBus`. This raw packet is additional to—not a replacement for—the
source seeker acquisition, lock, blind-range, pointing, and LOS-filter state.
Session observations expose missile and target truth, source-managed
normal/lateral commands, realized normal/lateral acceleration, source seeker
state, and the delivered native packet. There are no caller action ports.

The requested/achieved fin channels plus command/acceleration outputs support
finite-run control traces and like-for-like controller comparisons. They do
not establish a trim, a closed-loop linearization, or formal stability or
frequency-margin claims.

## Parametric source-case variants

`build_default_sraam6_configuration` exposes an explicit, typed tuning surface
for source-backed variants. `actuation` contains the fin position/rate limits
and second-order natural-frequency/damping parameters; `seeker` contains
acquisition range and LOS-filter gain/frequency/damping; and `controller`
contains the structural acceleration limit. The corresponding helper aliases
are prefixed `fin_`, `seeker_`, and `structural_limit_g`.

Every public value carries a canonical unit and its source-model lower bound in
the configuration schema. These are initialization-time `variant` inputs, not
session actions: they are copied into a per-run immutable definition and leave
the installed source case untouched. The native Taoryx relative-state track
remains the generic sensor output; seeker tuning configures the CADAC source
model rather than introducing a parallel sensor API.

## Fidelity decision

The executable standard realization is T4 because all of the following participate in the closed loop:

1. full translational state;
2. scalar-first quaternion attitude;
3. body angular rates and inertia;
4. aerodynamic and propulsive body force;
5. aerodynamic body moments;
6. roll, pitch, and yaw control requests;
7. fixed mixing into four physical fin commands;
8. independent second-order fin position and rate states;
9. fin travel and rate limits;
10. achieved fin positions feeding aerodynamic force and moment closure.

The optional source TVC path is also classified as a physical-effector T4 realization, but it remains discoverable and validation-only. A request for the optional TVC phase cannot dispatch the standard-fin executor.

## Source lowering

`Sraam6SourceDefinition` lowers and fingerprints:

- the exact module and vehicle order;
- fixed integration and output cadence;
- missile and target initial state;
- aerodynamic, propulsion, seeker, guidance, control, and actuator parameters;
- ordered missile and target event blocks;
- aerodynamic and propulsion table decks;
- source artifact hashes and byte sizes.

The lowerer and event binder fail closed when:

- the case is not exactly one `MISSILE6` followed by one `TARGET3`;
- required modules are absent or reordered;
- required aerodynamic or propulsion tables are absent;
- the standard executable case requests active TVC;
- the target assignment cannot be resolved;
- an event references or mutates a source variable outside the executable registry;
- participating source values are non-finite or invalid.

## Four-fin physical actuator

The source control coordinates are mixed into four fin commands:

```text
fin1 = -roll + pitch - yaw
fin2 = -roll + pitch + yaw
fin3 =  roll + pitch - yaw
fin4 =  roll + pitch + yaw
```

The inverse mapping reconstructs achieved roll, pitch, and yaw control coordinates from achieved fin positions. Each physical fin carries independent stored-derivative state for position and rate. Source-compatible processing preserves the travel-limit, rate-limit, integration, and anti-windup order rather than replacing the actuator with one aggregate lag. In source mode `mact=0`, the output is position-limited but the dormant dynamic states remain unchanged, so a later transition to `mact=2` does not inherit invented actuator state.

CADAC applies the second-order travel and rate checks before the stored-derivative integration update. A fin arriving just inside its travel boundary at high rate can therefore cross the nominal limit for one accepted epoch; the following source pass detects and corrects the excursion. The compatibility runtime and regression suite preserve this ordering instead of adding a post-integration clip that would change source behavior.

The standard output contract exposes both requested and achieved quantities:

- `requested_control_deg`;
- `requested_fins_deg`;
- `achieved_fins_deg`;
- `achieved_control_deg`.

This requested-versus-achieved distinction is required evidence for the T4 claim.

## Rigid-body plant

The missile plant includes:

- source declaration-time mass, center-of-gravity, and inertia values before the first propulsion module call, followed by source-deck values at the propulsion boundary;
- flat-Earth local North-East-Down translation;
- body-forward/right/down coordinates;
- scalar-first quaternion propagation;
- full body roll, pitch, and yaw rates;
- source stored-derivative modified-Euler/trapezoid integration;
- inverse-square gravity and source atmosphere behavior;
- time-varying mass, center of gravity, roll inertia, and transverse inertia;
- source aerodynamic tables and body-axis force/moment reconstruction;
- rocket thrust with nozzle pressure correction;
- Newton and Euler closure.

The source aerodynamic model evaluates axial, normal, side, roll, pitch, and yaw coefficient families, transforms between aeroballistic and body axes, includes rate damping and fin control effects, and applies center-of-gravity moment corrections.

## Guidance, control, and seeker

The source event program begins in rate control and switches to acceleration control with proportional-navigation guidance. The runtime preserves the ordered source event and reports it as `Sraam6EventTrace`.

Participating control paths include:

- roll-angle control;
- pitch/yaw rate control;
- pitch/yaw acceleration control;
- command limiting;
- source aerodynamic-derivative use where required by the controller;
- physical fin allocation and achieved-effect closure.

The seeker implementation includes:

- enabled, acquisition, lock, and blind-range modes;
- acquisition range and dwell time;
- field-of-view gating;
- body-frame pointing angles;
- line-of-sight rate generation;
- the source stored-derivative second-order filter and pointing-angle state equations;
- transition to compensated terminal proportional navigation after lock.

The complete CADAC optical-error, aimpoint, image-error, and gimbal-head geometry is not yet reconstructed. Until those modules are ported, true LOS-to-current-pointing displacement supplies the participating filter error. This narrower approximation is carried in provider diagnostics and the model claim boundary rather than being silently described as complete optical-seeker parity.

## TARGET3 actor

The target uses the source flat-3-DoF equations and supports the source steady and horizontal-g-turn behavior required by the standard engagement. Its position and velocity are independently sampled and returned as:

```text
model_id: cadac.sraam6.target
fidelity: point_mass_3dof
parent_object_id: null
```

Target lifecycle, position, velocity, speed, heading, flight-path angle, altitude, bank, and normal load are available through the common result.

## Mission Composition output

The missile core channels are:

- `position_ned_m`;
- `velocity_ned_mps`;
- `quaternion_wxyz`;
- `body_rates_rad_s`.

Optional telemetry groups cover:

- air data and incidence;
- requested and achieved controls;
- body force and moment;
- propulsion and mass properties;
- seeker and guidance state;
- achieved acceleration.

The returned result contains two independent roots: the SRAAM6 missile and TARGET3 aircraft. Source events and intercept termination are returned through the shared event contract.

## Validation and evidence

Current automated evidence covers:

- source-case lowering and required-table validation;
- exact fin mixer inverse;
- four independent second-order actuator states;
- requested versus achieved fin telemetry;
- time-deck propulsion, mass, center of gravity, and inertia;
- source rate-control to acceleration-control event;
- target-side time/event binding and fail-closed unsupported event mutation;
- source declaration-time mass-property ordering and dormant mode-0 actuator state preservation;
- finite rigid-body execution;
- target motion under source scheduling;
- exact provider/model common-runner registration;
- separate missile and target root objects;
- core versus all-output selection;
- blocked optional-TVC dispatch;
- CLI lowering and execution.

Still required for numerical promotion:

1. compiled SRAAM6 output from the pinned CADAC revision;
2. table and module-boundary golden comparisons;
3. one-step rigid-body and actuator parity;
4. seeker/guidance/control transition parity;
5. closed-loop miss-distance and intercept-time parity;
6. reconstruction of the omitted optical/aimpoint/gimbal-head seeker details, or a narrower permanent claim boundary.

## CLI

```bash
python -m taoryx.families.cadac sraam6-lower /path/to/SRAAM6/input.asc
python -m taoryx.families.cadac sraam6-run /path/to/SRAAM6/input.asc \
  --end-time 2.0 --sample-step 0.02
```

## Subsequent reuse

AGM6 now reuses the SRAAM6 physical-fin and actor-scheduling substrate as a runnable air-to-ground three-actor composition. GHAME6 subsequently extends the same substrate with atmospheric-to-exo phase changes, physical atmospheric surfaces, aggregate-RCS transfer/interceptor phases, and independent satellite and radar actors.
