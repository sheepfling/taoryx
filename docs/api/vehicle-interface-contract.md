# Vehicle interface contract

## Decision

TAORYX needs one versioned, resolved interface contract between a selected
vehicle realization and every consumer above the plant: composition tools,
human pilots, scripted control, AI/RL policies, telemetry, plots, and
qualification evaluators.

The contract is not a second physics model and it does not erase meaningful
family differences. It makes each difference explicit while providing stable
semantic hooks for the things that are genuinely comparable:

- what a user may configure before an episode;
- what an episode, segment, or step may change;
- what control authority is offered at the selected fidelity;
- which resources, propulsion, actuator, and state values are available;
- what each value means, its unit, bounds, frame, provenance, and evidence
  level; and
- whether a policy sees truth, an explicitly modeled observation, or no value
  at all.

The central rule is:

> A canonical channel identifies a semantic quantity. A vehicle binding
> declares how that quantity is realized, qualified, and observed for one
> family, configuration, fidelity, and authority mode. An unavailable
> quantity is unavailable; it is never silently represented by zero.

This addresses a present gap. The family-adapter and composition registries
already declare native state, control, resource, initialization, and segment
channels. The first composition episodes expose bounded native actions and
committed truth. The common Mission Composition session layer now preserves
that native action union for legacy callers and, when a caller selects an
authority profile, projects only that profile's stable semantic action schema.

## Boundary and terminology

The contract sits above source state-vector layouts and below a mission
composer or policy:

    resolved vehicle + fidelity + configuration + environment
        -> VehicleInterfaceContract
        -> selected authority and observation profiles
        -> episode ActionFrame / ObservationFrame / StatusFrame

It has five distinct views. They must not be conflated.

| View | Question answered | Mutability |
| --- | --- | --- |
| Identity and evidence | What exact model is this, and what may it claim? | Immutable |
| Parameter schema | What may vary, at which scope, and within what limits? | Declared by scope |
| Semantic control schema | What may a caller request? | Step or segment only |
| Effector schema | What physical or bridge inputs actually reach the plant? | Internal, or direct-authority only |
| Status and observation schemas | What is true, what is measured, and what is published? | Read-only |

A guidance waypoint, a body-moment request, an actual elevon deflection, and
an observed airspeed are therefore separate named quantities with explicit
links between them. No controller grades its own route progress, and no
policy is allowed to mutate raw state, aerodynamic tables, mass properties,
or arbitrary model fields.

## Resolved interface artifact

VehicleInterfaceContract is an immutable artifact generated after vehicle
selection and before an execution session opens. It is fingerprinted with the
resolved composition, vehicle package, fidelity profile, control realization,
environment, and observation profile.

Its minimum contents are:

    schema: taoryx.vehicle-interface/v1alpha1
    vehicle identity and source/evidence manifests
    selected family, configuration, fidelity, and control realization
    qualification and capability report
    parameter schema, grouped by mutability scope
    semantic action profiles and their bindings
    internal effector schema and allocation/actuator boundary
    canonical status schema and raw diagnostic sidecar schema
    observation profiles, sensor/noise semantics, and visibility rules
    resource ledger mappings
    event, phase, terminal, and numerical-status vocabulary
    channel provenance, availability, and claim boundaries

The artifact is a declarative contract. Runtime samples only contain values
and availability states that are permitted by this resolved artifact.

### Native source projection

The resolved contract and the native source file have different jobs.  A
native `.prb` carries only grammar-supported scalar runtime attributes.  The
contract retains the richer structured record: actuator dynamics, allocation
maps, sensor definitions, table provenance, and evidence limits.  A lowering
tool must not serialize an arbitrary mapping or list into a native
`key=value` field.  That would be malformed source at best and, at worst,
would make structured metadata look like active plant behavior.

Consequently, a generated native file is successor-parsed before it is
accepted, while a capability becomes runnable only when a named native binding
actually consumes the corresponding structured contract field.  The presence
of a motor-lag record or allocation matrix in the contract is not itself a
claim that the selected fidelity executes it.

## Channel descriptor

Every parameter, action, effector, status value, and observation uses a
single descriptor shape. An implementation may use Pydantic models, but its
serialized fields should contain at least:

| Field | Meaning |
| --- | --- |
| id | Stable dotted semantic ID, for example propulsion.command.fraction. |
| kind | parameter, action, effector, status, observation, resource, or diagnostic. |
| value type | scalar, vector, enum, boolean, or structured event. |
| canonical unit and frame | Unit plus reference frame where applicable. |
| value space | Mathematical topology, representation, equivalence, error, normalization, and permitted interpolation rule. |
| bounds | Hard lower/upper bounds and, where known, qualified and safe-extended bounds. |
| default and transform | Default, normalization, and optimizer/RL transform where applicable. |
| availability | available, available_in_batch, not_applicable, not_available, planned, or unavailable_at_runtime. |
| provenance | Source-backed, derived, engineering surrogate, synthetic, or replayed. |
| evidence/claim boundary | What the value proves and what it does not prove. |
| sampling semantics | Truth boundary, sensor cadence, event step, or held command. |
| binding | Mapping to native state, controller reference, wrench, effector, or resource. |

Bounds are not merely UI hints. A hard-bound violation is rejected or
explicitly projected according to the variant-resolution policy; a qualified
range boundary changes the evidence classification. Projection is recorded
with the original request and projection distance.

### Value-space semantics

Every channel declares its mathematical value space in addition
to its storage shape. `scalar`, `vector3`, and `vector4` only say how a value
is serialized; they do not say which algebra is valid.

| Example | Storage type | Value space | Consequence |
| --- | --- | --- | --- |
| `flight.heading.command` | scalar | periodic \(S^1\) angle | Use shortest wrapped-angle error. \(179^\circ\) and \(-179^\circ\) are close. |
| `body_rate.z` | scalar | linear tangent scalar | Use ordinary signed difference in rad/s. |
| `attitude.quaternion` | vector4 | unit-quaternion representation of \(SO(3)\) | Preserve unit norm, identify \(q\) and \(-q\), use a geodesic/log-map attitude error. |
| `attitude.euler` | vector3 | declared Euler coordinate chart of \(SO(3)\) | Record order and singularity; do not treat all components as globally linear. |
| `flight.alpha` or `effector.elevon_left.command` | scalar | bounded interval | Clamp/project at declared physical limits; do not wrap. |
| `position.ned` or `wrench.moment.command` | vector3 | Cartesian \(\mathbb{R}^3\) in named frame | Use component-wise operations in that frame. |
| `guidance.los.unit_vector` | vector3 | unit sphere \(S^2\) | Normalize and measure angular/geodesic error. |
| `resources.battery.soc` | scalar | bounded fraction \([0,1]\) | Use bounded fraction transforms. |
| `phase.mode` | enum | finite discrete set | Hold/discrete transition only; never interpolate. |

The serialized `value_space` descriptor includes:

```text
kind, topology, representation, period/equivalence when applicable,
error_rule, interpolation_rule, normalization_rule, and coordinate chart.
```

`InterfaceChannel` now emits this descriptor for every public channel and
rejects values that violate immediately testable representation invariants,
including quaternion unit norm and bounded fractions. The independent
truth-objective layer publishes the same descriptor for each target channel
and uses its declared error rule during evaluation: heading targets use a
wrapped \(S^1\) error and fly-by gate normals are constrained to unit
directions on \(S^2\). Current resolved interfaces use the versioned
[`interface_channel_value_space_catalog.yaml`](../../verification/interface_channel_value_space_catalog.yaml)
instead of a code-level identifier switch. The catalog explicitly assigns each
public action, effector, status, observation, resource, diagnostic, and
composition-parameter channel to a profile; `topology-report` fails when any
resolved public channel is absent. Registry-owned input and objective
declarations publish the corresponding descriptors at their own boundaries.
The generic objective vocabulary is separately versioned in
[`truth_objective_channel_value_space_catalog.yaml`](../../verification/truth_objective_channel_value_space_catalog.yaml),
so the independent evaluator does not decide a heading's circular semantics
from a local identifier list. Authors must not infer periodicity or quaternion
semantics from a unit or identifier alone.

`taoryx vehicle topology-report` is the catalog-wide conformance gate. It
audits the registry’s initialization, segment, and variant inputs alongside
every resolved interface channel, observation binding, and the generic
truth-objective schema. It fails on a missing or `topology_pending` public
surface, while deliberately making no claim about execution or qualification.

This contract applies to parameters and controls as well as status and
observations. It determines controller errors, action validation, optimizer
transforms, sensor residuals, terminal-objective checks, replay comparisons,
and the only permissible interpretation of any explicitly derived resampling.
Before a semantic action frame maps to a native control, TAORYX validates the
selected profile's channel type, hard bounds, and declared value-space
invariant. A fraction outside \([0,1]\), an invalid quaternion, or a malformed
discrete value is rejected at the interface boundary; it is not silently
converted to a native command. A native low-level actuator may separately
apply its declared physical saturation, but that realization is logged as an
achieved command rather than changing what the caller requested.
It does not relax the committed-truth rule: sensor values are captured at
declared truth boundaries, not interpolated from later truth states.

## Parameter scopes

Parameter metadata must identify when a value may change. This prevents an AI
policy from changing the vehicle while it is flying and makes a resolved case
reproducible.

| Scope | Examples | Allowed mutation |
| --- | --- | --- |
| Family/model | topology, table assets, reference geometry | Never during a run |
| Variant/configuration | payload, initial fuel/propellant, CG, engine derate | Resolve before run |
| Episode reset | seed, launch state, battery state of charge, wind realization | Reset only |
| Segment | target altitude, waypoint, speed, reserve, controller schedule | Named transition only |
| Step action | thrust command, bank command, body-rate command, effector command | At accepted external step boundaries |
| Derived/status | wet mass, available authority, fuel remaining | Never caller-mutated |

Independent inputs must be distinct from derived values. A caller may request
payload and propellant loading; it does not independently set wet mass,
burnout mass, burn duration, total impulse, and mass flow without a declared
coupled model.

Every parameter also records whether it requires retrim, controller
rescheduling, a capability recomputation, or requalification.

## Control authority ladder

One vehicle can expose several authority modes, but each exact
family/fidelity/episode binding declares which modes are available:

    mission intent
      -> kinematic intent
      -> body-motion intent
      -> desired wrench
      -> physical effector

- Mission intent requests a declared objective, route, or segment target.
- Kinematic intent requests speed, flight-path, acceleration, bank, position,
  or heading behavior.
- Body-motion intent requests attitude, body rates, or angular acceleration.
- Desired wrench requests force and/or moment in an explicitly named frame.
- Physical effector requests a declared throttle, elevon, rotor, gimbal,
  wheel, or thruster command.

An AI or human is offered exactly one selected action profile, not a mixture
of hidden lower layers. A high-level controller may internally traverse lower
layers; that provenance remains visible in status. Direct effector authority
is a deliberate debugging, test, or expert-control mode, never an implicit
replacement for guidance or allocation.

Fidelity, external control authority, and observation profile are independent
selection axes. A high-fidelity plant may offer a high-level wrapper only when
the controller and lowering path are actually registered; a reduced plant may
offer pilot-like or waypoint inputs only when it labels their reduced response
semantics. Fidelity alone neither grants low-level effectors nor forbids a
truthfully implemented high-level controller.

Every authority profile records:

- a cross-family scheme ID/layer, intended consumer roles, streaming
  preference, and UI order;
- `command_owner`: caller, source program, provider controller, or open loop;
- `selection_scope`: batch, session, phase, step, or provider;
- `switching_policy`: locked, explicitly bumpless, or provider-managed;
- the exact semantic action IDs and applicable phases; and
- an ordered lowering chain from the external request to the plant seam.

The reduced A320 and F-16 point-mass/pseudo-6DOF sessions currently implement
three caller-owned surfaces: direct kinematic guidance, normalized reduced
pilot commands, and a live local-navigation waypoint. The F-16 pseudo-6DOF
session additionally implements body-frame roll-, pitch-, and yaw-rate
references. Its roll/yaw path is explicitly labeled as a coupled engineering
surrogate, not a source FCS, moment, actuator, or surface interface.

The Hummingbird pseudo-6DOF session implements a multirotor-specific three-mode
surface: aggregate attitude/thrust, local velocity plus yaw, and a live local
waypoint. All three are caller-owned, session-selected, and explicitly
bumpless. Velocity and waypoint modes lower continuously at the episode's
integration boundaries into the existing five-coordinate aggregate plant
seam; they do not add a second dynamics model or rename fixed-wing controls as
multirotor controls. The physical individual-rotor LQI screen remains a
separate batch-only realization and is not reachable through these profiles.

The four canonical fidelity tiers map to expected, not mandatory, authority:

| Fidelity tier | Primary semantic control | Required status boundary | Explicit nonclaim |
| --- | --- | --- | --- |
| point_mass_3dof | Kinematic/energy intent | Requested and achieved translational control, resources, envelopes | Attitude, moments, physical effectors |
| pseudo_6dof | Kinematic or body-motion intent | Commanded/achieved attitude and rate response, lag, limits | Physical moment balance or effector activity unless separately modeled |
| rigid_body_6dof_direct_wrench | Desired wrench | Requested/achieved force and moment, residual, direct-wrench label | Physical actuator realization |
| rigid_body_6dof_surface_allocated | Body-motion/wrench or declared direct effector | Allocation, commanded and actual effectors, actuator limits, achieved wrench | Fidelity beyond declared data and validation envelope |

### Cross-family scheme discovery

The cross-family `scheme_id` is not the authority-profile ID. The scheme is a
stable UI/autonomy grouping; the authority remains the model-specific action
surface with exact channels, units, bounds, ownership, and lowering. Model
metadata materializes `control_scheme_support` as a flat join over realization,
fidelity, and authority so clients can follow this order without guessing:

    model -> fidelity -> realization -> supported scheme
          -> authority profile -> typed action channels

A scheme may appear at multiple fidelities only when each tier implements a
truthful response. The channel set and response law may differ by tier. A
missing row means unsupported; it is not permission to borrow the neighboring
tier's controls. Reduced tiers never advertise physical surfaces merely to
look symmetrical with a 6DOF model.

The rapid-turn development queue is:

| Priority | Families/models | Schemes to establish first | Deliberate boundary |
| --- | --- | --- | --- |
| P0 | API stressor, analytical ballistic, analytical waypoint, Simple Aero | open loop/provider program, waypoint, flight path, energy, typed debug modes | Contract and streaming behavior before physical qualification |
| P1 | Reduced A320 and F-16, then X8/B747 where executable | waypoint/route, flight path, normalized pilot axes; body rate only for a supporting pseudo-6DOF tier | 3DOF and pseudo-6DOF only |
| P2 | Hummingbird implemented; future helicopter/tiltrotor models remain | Hummingbird attitude/thrust, velocity/yaw, and waypoint now; future family-specific collective/cyclic/pedal or nacelle profiles | No fixed-wing axis relabeling and no rotor-allocation claim |
| P3 | Rockets, missiles, air breathers, hypersonic gliders | destination/waypoint/route, flight path, energy, attitude/rate where implemented | Source programs remain provider-owned unless an external seam exists |
| P4 | Spacecraft and proximity operations | orbit target, relative pose/motion, attitude, body rate | No invented thruster, wheel, or RCS allocation |
| Separate qualification track | Any rigid-body source model | direct wrench and direct effectors | Retain discovery, but do not put these paths in the rapid P0/P1 loop |

Promotion within a row requires discovery metadata, active session projection
when `step` is advertised, requested/applied/lowered telemetry, focused tests,
and an explicit claim boundary. It does not require retesting unrelated
direct-wrench or effector models during each reduced-order API edit.

Fidelity never causes a channel to masquerade as a different thing. For
example, a pseudo-6DOF Hummingbird can publish
propulsion.command.fraction and propulsion.output.thrust.aggregate, but rotor.1.rpm
is not applicable until the selected realization models it.

## Canonical action vocabulary

Semantic actions should use stable domains rather than force every family into
a literal throttle, elevator, or rotor command:

| Domain | Representative IDs |
| --- | --- |
| Propulsion | propulsion.command.fraction, propulsion.enable, propulsion.mode |
| Translation | velocity.north.command, velocity.east.command, velocity.vertical.command |
| Flight path | flight.bank.command, flight.heading.command, flight.path_angle.command, flight.altitude.command, flight.airspeed.command |
| Body motion | attitude.command, body_rate.command, angular_acceleration.command |
| Wrench | wrench.force.command, wrench.moment.command |
| Effectors | effector.<id>.command, including position/rate/enable where declared |

propulsion.command.fraction means the normalized propulsion request accepted
by this realization. Its binding specifies whether that is a jet throttle,
rocket thrust program, collective-like aggregate thrust, motor command, or an
unavailable concept. It must not be shown as a common physical throttle unless
the family binding says it is.

The schema may expose vehicle-specific semantic controls in a namespaced
extension, such as rotorcraft.nacelle_angle.command or spacecraft.dipole.command.
An extension must not reuse a canonical ID with altered physics.

### Package-owned interface additions

When a family needs a semantic control or readback that is not a shared
family-wide primitive, its wheel may register a lazy
`vehicle_interface_extension` contribution. The extension is additive only:
it may supply channels and authority profiles for its declared family, while
the host continues to construct the base contract and validates duplicate
IDs, authority membership, availability, units, bounds, value spaces, and
claim boundaries. A selected plug-in catalog is carried through composition,
batch, session, witness, and parity paths, so resolving one extension never
requires discovery of unrelated vehicle wheels.

For example, the F-16 pseudo-6DOF plug-in adds bounded body-rate references
and committed axis readbacks. Its status binding accepts the exact scalar
episode readback or the indexed batch body-rate vector present at the
committed boundary; it does not infer a physical flight-control system,
effector allocation, or missing telemetry.

## Canonical status and resource vocabulary

Status is a normalized read-only view, not a replacement for the native state
or detailed telemetry. It supplies reliable hooks across families:

| Domain | Core status IDs |
| --- | --- |
| Execution | execution.time, execution.status, phase.segment, phase.mode, event.last |
| Truth state | position.*, velocity.*, attitude.*, body_rate.* where represented |
| Resources | resources.mass.total, resources.<kind>.remaining, resources.<kind>.fraction_remaining |
| Propulsion | propulsion.command.fraction, propulsion.output.thrust, propulsion.output.power, propulsion.resource_rate |
| Control | control.requested.*, control.achieved.*, control.residual.*, control.saturated |
| Effectors | effector.<id>.commanded, effector.<id>.actual, effector.<id>.rate, effector.<id>.saturation |
| Limits | envelope.<id>.margin, authority.<axis>.available, table.<id>.domain_margin |
| Numerical integrity | truth.timestamp, integration.accepted_step, integration.event_boundary, numerical.status |

The resource ledger is generic by design. Fuel is represented where it
exists, but it is not the universal resource:

| Family example | Specific ledger fields | Common hook |
| --- | --- | --- |
| Jet/fixed-wing aircraft | resources.fuel.mass, propulsion.fuel_flow | resources.mass.total and a declared fraction_remaining |
| Rocket | resources.propellant.mass, propulsion.mass_flow | resources.mass.total and resource depletion events |
| Multirotor | resources.battery.energy or resources.battery.soc | resources.battery.fraction_remaining |
| Reaction-wheel spacecraft | resources.wheel_momentum | authority and saturation status, not fuel semantics |
| Passive tumbler | none, or a declared thermal/contact budget | available status with no invented propulsion/resource channels |

An all-family generic resource fraction is optional and only emitted when its
meaning is declared. Consumers must retain the specific resource field and
its provenance.

Every canonical channel has an availability state. Unavailable state,
effector, and resource channels are reported with their reason in the
descriptor; they are omitted from a sample rather than filled with a plausible
number.

`available` means an accepted-truth episode may publish the channel to the
selected observation profile. `available_in_batch` means an exact batch
execution binding emits it in the run artifact, but no policy observation is
advertised. This prevents source replays and open-loop staged witnesses from
being described as interactive merely because their telemetry is inspectable.
The X-15 local direct-wrench screen is the intentionally narrow exception:
its exact source-local composition has a bounded episode binding. It accepts
only `wrench.force.command` and `wrench.moment.command` as **total** body
wrenches, projects them through the declared six-axis authority box, and
publishes source-plant body velocity/rate plus requested, achieved, residual,
and saturation-marked wrench truth at every committed boundary. The declared
source-load bridge bias remains visible in those values; it is not concealed
as an actuator trim. This makes the bridge useful for a human or policy
screen while retaining its strict nonclaims: it is not a physical effector,
surface, rotor, gimbal, thruster, or propulsion interface; not an X-15 flight
mission; and not physical-effector allocation evidence. This availability is
scoped to that exact compiled screen composition. The catalog-wide
family/fidelity descriptor can advertise the bridge as a discovery capability,
but a different X-15 direct-wrench mission does not inherit its episode or
truth-observation availability merely because it shares the same fidelity.
For example, the passive tumbling direct-release witness publishes position,
velocity, fixed mass, and—only at its rigid-body-reuse pseudo tier—quaternion
and body-rate truth as `available_in_batch`. It exposes no action, wrench, or
effector channel, and its pseudo label never implies a response-law controller.

## Truth, observation, and sensor separation

Simulation Runtime must make the following distinction enforceable:

    plant truth at committed time t
      -> declared sensor/observation model at t
      -> policy observation at t
      -> action held to the next accepted external boundary

Truth is available to evaluators, debuggers, and showcase evidence. A policy
receives only the selected observation profile. An observation descriptor
records source truth channel, sensor model, latency, cadence, noise,
quantization, frame, bounds, normalization, and validity mask. It must not
interpolate from a future state to fabricate a measurement between accepted
truth boundaries.

This also permits three legitimate profiles for the same episode:

- truth_debug for plant/controller development;
- declared_sensor for realistic estimator and RL work; and
- sparse_policy for a deliberately limited policy interface.

The profile ID and observation/timestamp history belong in every replay and
training artifact. Rewards, curricula, and learning algorithms remain above
the vehicle interface; the interface supplies state, action, status, terminal
events, and constraints without embedding an RL objective.

### First executable declared-sensor slice

The initial implementation is intentionally narrow and auditable rather than
pretending to supply a generic IMU, GPS, or estimator. A composition may bind
one `declared_sensor` profile by naming portable available status/resource
channels plus a positive cadence and nonnegative latency. The binding changes
the resolved interface fingerprint, so an action or checkpoint from a
truth-debug run cannot be replayed against a different observation contract.

At runtime the episode scheduler divides an external held action at the next
sensor capture or delayed-release boundary. It captures only canonical values
at that committed truth boundary; a delayed policy observation holds the last
released sample and records its `source_time_s`. Before its first release,
every selected channel is `null` with `valid: false`. A policy therefore
cannot mistake current plant truth for a delayed measurement, and no skipped
cadence is repaired with a future-state interpolation.

### Declared scalar measurement transforms

The first portable measurement-error slice is intentionally smaller than a
named IMU, GPS, camera, or estimator.  A composition-bound sensor may declare
an explicit transform for one of its selected **scalar** canonical channels:

```yaml
channel_errors:
  position.altitude:
    bias: 0.5
    gaussian_stddev: 0.2
    quantization_step: 0.1
```

At the committed capture boundary TAORYX applies the constant additive bias,
one deterministic white-noise draw, and then the optional nearest-increment
quantizer. The profile definition participates in the resolved interface
fingerprint. The episode-reset seed selects the deterministic draw sequence;
the same profile, seed, committed capture times, and action history replay
exactly, including through a checkpoint. The batch trace accepts the explicit
seed used for its replay and records it in the artifact.

This is a generic scalar measurement transform, not evidence of a physical
altimeter, air-data system, IMU, GPS receiver, vector attitude sensor, or
estimator. Vector/boolean/enum channels must remain ideal in this first slice
unless a named family sensor model declares their error law. Sensor values can
legitimately exceed a truth channel's physical bounds; the raw committed truth
and the declared measurement transform remain separately visible.

The [X8 sensor composition](../../examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml)
and [Hummingbird sensor composition](../../examples/vehicle_composition/hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml)
are the first witnesses. Their checkpoints serialize the declared sensor's
next capture time, pending delayed samples, and last released reading, then
reject restoration under a different composition or interface fingerprint.
Their aligned batch translators additionally emit `sensor_observations.json`
from the same resolved interface: a capture or release boundary absent from
the committed truth rows fails closed rather than being interpolated after the
run. This is the reusable timing/checkpoint/artifact contract; family-specific
noise, bias, quantization, IMU integration, GPS, and estimator state remain
later declared sensor models rather than hidden truth transforms.

Every runnable batch translator additionally emits `status_trace.json`. This
is not an observation profile: it is the complete portable projection of the
selected interface's `available` and `available_in_batch` status, resource,
and diagnostic channels at each committed truth row. The trace is fail-closed:
if an advertised batch channel cannot be recovered from the native row, the
run fails instead of omitting it or substituting zero. This gives artifacts and
showcase consumers a common status hook for fixed-wing, multirotor, staged,
and passive families while preserving the boundary that batch-only values are
not automatically policy-visible observations. The projection is also checked
against each channel's declared scalar/vector/boolean/enum shape and scalar
bounds; malformed values are a failed artifact boundary, never a renderer-side
repair.

Every normal batch executor also emits `resource_ledger.json`. It is a
composition- and interface-fingerprint-bound subset of the committed status
trace: each declared resource channel carries only its emitted samples,
descriptor, and a descriptive constant/nondecreasing/nonincreasing/variable
trend. The ledger does not integrate mass flow, infer battery energy, or treat
a monotonic trace as a physical depletion validation. The result catalog
rebuilds and compares a supplied ledger against `status_trace.json`; an
altered resource value is invalid evidence rather than a new resource model.

The current reduced A320 and F-16 adapters additionally project common local
navigation hooks (`position.north`, `position.east`, `position.altitude`,
`velocity.speed`, flight path, heading, dynamic pressure, and modeled total
mass). Their pseudo-6DOF variants add the named response-law attitude and body
rate. These are batch-only source-derived or engineering-surrogate values, not
episode observations, a physical control-surface claim, or a fuel-system
ledger; the F-16 mass is explicitly fixed at the retained local source point.

### Semantic action-stream replay

`run_composition_policy` retains a trace of public `ActionFrame`, applied
semantic command, committed truth observation, and status frames. A trace is
bound to both the compiled composition identity and its resolved interface
fingerprint. `replay_composition_policy_trace` opens a fresh episode and
requires the same public frames, including the initial and final observations,
to reproduce exactly. It rejects a different composition, interface, action
mapping, accepted-boundary behavior, or nondeterministic episode transition.

This is deliberately narrower than batch parity. It proves deterministic
replay of one declared scripted or RL-style semantic action stream through the
same episode kernel; it does not establish agreement with a separately
compiled batch mission, physical actuator realization, robustness, or mission
qualification.

### Native control provenance is not a semantic action trace

Language-backed TAOS execution can optionally retain
`control_provenance.json` (`taoryx.runtime-control-provenance/v1alpha1`).  It
records accepted interval counts, controller-evaluation timing, native control
names, and whether any solver-stage evaluation changed the visible control
vector.  It also lists the native names owned by a committed-boundary resolver.
This artifact exists to diagnose the migration to committed-boundary
sample-and-hold controllers.  Provenance alone does not assert that a semantic
command was held through an interval and is never accepted in place of
`semantic_action_trace.json`.  A public action trace becomes available only
when the selected controller resolves every declared semantic action at a
committed truth boundary, holds it unchanged until the next accepted boundary,
and emits the identity-bound trace from those accepted intervals.  The X8
point-mass racetrack is the first language-backed witness: its native route
references are resolved at each committed boundary, while its published
semantic trace remains limited to the separately declared throttle/elevon
bridge actions.

Composition-backed boards must retain this selected profile as well as the
resolved interface fingerprint. `build_showcase_run_artifact_for_composition`
uses the compiled composition rather than a bare family/fidelity lookup; a
sensorized run therefore cannot be relabeled as a truth-debug board by a
renderer. The wrapper always requires `status_trace.json`, and additionally
requires `sensor_observations.json` when a sensor profile is selected. The
former preserves canonical committed truth status; the latter preserves the
selected profile's held samples and source timestamps. Neither may be replaced
by metadata alone.

## Family-specific realization without interface fragmentation

The same canonical ID may bind differently, but every binding is explicit:

| Canonical intent | X8/B747/F-16 | Hummingbird | NESC rocket | Passive tumbler |
| --- | --- | --- | --- | --- |
| propulsion.command.fraction | Engine/throttle map or surrogate | Aggregate thrust or physical motors | Declared thrust/gimbal program if controllable | not_applicable |
| flight.bank.command | Guidance or attitude bridge | not primary; map only if a hover trajectory controller declares it | not primary | not_applicable |
| body_rate.command | Pseudo response or rigid controller | Pseudo attitude/rotor-rate controller | Scheduled attitude bridge or gimbal path | not_applicable |
| wrench.moment.command | Direct-wrench tier only | Direct-wrench or rotor allocator tier | Direct-wrench/gimbal tier if declared | not_applicable |
| effector.*.command | Surface-allocated tier only | Motor/rotor tier only | Gimbal/reaction-control tier only | none |

Native source state, table variables, and diagnostics remain accessible in a
separate raw namespace with their original names and provenance. A generic
consumer should use canonical channels; an advanced family-specific consumer
may opt into the raw sidecar knowing that it is not portable.

## Composition and trajectory configuration

Vehicle composition must consume the same descriptors rather than maintain a
separate inventory:

    registry selection
      -> resolved interface contract
      -> initialization and variant parameter form
      -> compatible segment parameter form
      -> action/observation profile form
      -> compiled composition and episode

Initialization and segment parameters cite canonical IDs and scope. Segment
requirements cite a required semantic control intent, not a native elevon or
motor, unless that physical-effector authority is deliberately required by the
mission. A compile-time capability check then reports why a requested segment,
controller, control profile, or observation profile is unavailable.

The existing vehicle-composition registry remains the source of public
initialization, segment, and mission contracts. FamilyAdapterDescriptor
remains the plant-facing source of native state/control/resource channels.
VehicleInterfaceContract joins and projects them; it does not duplicate
physics data.

## Required runtime frames

Episode APIs should evolve from ad hoc mappings to three explicit immutable
frames:

    ActionFrame
        interface_id, authority_profile_id, requested values, time held

    ObservationFrame
        interface_id, observation_profile_id, timestamp, source timestamp,
        values, valid mask, terminal/constraint status

    StatusFrame
        interface_id, committed truth timestamp, canonical values, raw
        diagnostic references, resource/control/envelope/numerical status

EpisodeStep retains the requested ActionFrame, the actually applied action
after validation/limiting, the resulting ObservationFrame, the StatusFrame
available to debugging/evaluation, and declared events. This makes requested,
achieved, and observed quantities auditable without exposing plant internals
to a policy by accident.

### Session profile negotiation and live transfer

`MissionCompositionOpenSessionRequest.authority_profile_id` opts into the
semantic profile projection. The returned descriptor advertises every profile,
the default and active IDs, command source, ownership, switching policy, and
the active action schema. Each profile also carries its own action schema and
agent action-space projection, so clients can inspect inactive guidance,
body-motion, pilot, or effector modes before a transfer. A composition can set
`TrajectoryConfigurationInstance.startup_authority_profile_id`; an explicit
open-session selection must match it. Omitting both fields preserves the
legacy native-action session contract unless that provider explicitly selects
its default authority.

Every selected-profile step repeats the active authority ID and returns four
separate records: requested semantic action, applied semantic action, lowered
native adapter action, and lowering evidence. The committed observation also
contains `control_authority`, so a remote client can determine who owned the
command and which chain realized it without inferring from channel names.
`control_feedback` adds one stable row per active action, including its unit,
requested/applied presence, disposition, reason codes, and optional committed
achieved-state binding. Runtime availability and the policy mask are read from
`observation.control_authority.available_action_ids`; clients must not infer a
mask from fidelity or from the static profile list.

`MissionCompositionSessionManager.switch_authority(...)` performs an explicit
same-session handoff. It requires the caller's expected sequence, both profiles
to declare `explicit_bumpless`, and an episode-specific state-continuous
transfer hook. The handoff changes neither simulation time nor sequence; it
clears held references, preserves plant state, returns the new schema, and
records the new command source. Locked and provider-managed profiles cannot be
seized through this route.

Live waypoints use the same step route as manual and rate commands. A waypoint
action is held for the request's `duration_s`, may be replaced at the next
accepted boundary, and is lowered through the advertised navigator and response
law. This is in-stream guidance authority, not a second trajectory API. A
transport-level heartbeat/deadman policy is not yet part of this contract and
must not be inferred from held-action timing.

### Hummingbird quadcopter streaming profiles

The runnable Hummingbird `pseudo_6dof` composition opts into its default
`body_motion_response` profile when neither the prepared composition nor the
open request selects another mode. A reusable composition can instead set
`startup_authority_profile_id` to `velocity_yaw_command` or
`live_waypoint_guidance`. The same session can transfer among all three
profiles because each declares `switching_policy="explicit_bumpless"`.

| Profile | Semantic actions | Units and bounds |
| --- | --- | --- |
| `body_motion_response` | roll, pitch, yaw; aggregate thrust fraction; propulsion enable | roll/pitch `rad` in `[-pi/2, pi/2]`; yaw `rad` in `[-pi, pi]`; thrust `[0,1]`; boolean enable |
| `velocity_yaw_command` | north, east, positive-up vertical velocity; yaw; propulsion enable | north/east `m/s` in `[-5,5]`; vertical `m/s` in `[-3,3]`; yaw and enable as above |
| `live_waypoint_guidance` | north, east, altitude, capture radius, horizontal/vertical speed limits; yaw; propulsion enable | north/east `m` in `[-1e6,1e6]`; altitude `m` in `[0,10000]`; radius `m` in `[0.05,1000]`; horizontal speed `m/s` in `[0,5]`; vertical speed `m/s` in `[0,3]` |

The velocity adapter closes a bounded translational response into roll, pitch,
yaw, and battery-compensated aggregate thrust. The waypoint adapter first
maps position error and its speed limits to a velocity target, then uses the
same translational response. `lowered_action` exposes the final
`roll_rad`/`pitch_rad`/`yaw_rad`/`thrust_ratio`/`motors_enabled` request, while
`lowering_evidence` exposes target velocity, desired acceleration, requested
aggregate thrust, battery availability, limiting, and waypoint capture state.

Committed readback includes scalar roll/pitch/yaw, north/east/positive-up
vertical velocity, horizontal speed, propulsion enable, achieved aggregate
thrust and fraction, battery fraction, waypoint range/capture/status, and the
existing contact and body-state channels. `control.controller.method` names
the active attitude response, bounded velocity response, or waypoint/velocity
cascade, and `lowering_evidence` publishes its gains and limits so controller
analysis can compare the actual adapters without implying a tuned or qualified
physical flight-control system. Every compatible action declares a
same-unit achieved binding; capture radius deliberately reports
`achievement_status="not_observed"` because it is a tolerance, not an achieved
state. At or below the declared battery depletion threshold the runtime
authority mask reports `depleted` and reason code `battery_depleted` for every
action instead of accepting a command that the plant cannot realize.

No private sensor model is introduced by these adapters. `truth_debug` exposes
the standard canonical committed status, while `declared_sensor` continues to
sample only the composition-selected canonical channel IDs through Taoryx's
shared cadence, latency, error, validity, and checkpoint machinery. Adding a
guidance output does not silently add it to a declared sensor profile.

Fidelity and control abstraction are independent axes. A higher-fidelity plant
may expose fewer externally selectable modes, while a reduced plant may expose
several well-defined adapters:

| Control abstraction | Typical input | What it does not imply |
| --- | --- | --- |
| Effector | Surface, rotor, gimbal, or thruster position | That every lower tier can realize physical allocation |
| Wrench | Body force or moment | A physical effector or actuator implementation |
| Body motion | Attitude, rate, or angular acceleration | Source flight-control-computer behavior |
| Pilot/kinematic | Normalized axes, speed, path, heading, bank | Physical stick, pedal, or surface evidence |
| Guidance/mission | Waypoint, route, destination, orbit, relative pose | Availability in every phase or for every family |

Do not order these rows as a universal fidelity ladder. They are authority
surfaces that a realization declares independently. In particular, waypoint
guidance often gives a remote user a useful lower-bandwidth stream, but a
ballistic coast, source-owned controller, terminal lifecycle, resource
depletion, or an undeclared navigator can make it absent or temporarily
unavailable.

The provider-neutral low-fidelity witnesses and Hummingbird use the same contract. Analytical
ballistic flight selects a locked, zero-action `open_loop_coast` profile.
Analytical waypoint flight can hand off among configured provider guidance,
three-channel kinematic velocity commands, and five-channel live waypoint
retargeting. The non-physical contract probe separates continuous/vector,
discrete, and event action schemas so consumers must honor types, enum choices,
and event repeat policy. Simple Aero defaults to its provider-generated
schedule and may switch to direct throttle only; its generated bank value is
reported but has no interactive steering claim. Each witness preserves time,
sequence, and checkpointed active-profile state across accepted handoffs.
Hummingbird additionally proves a physical-family-specific velocity/yaw to
live-waypoint handoff with battery-aware availability and no rotor promotion.

CADAC persistent source cases advertise a zero-action
`source_program_control` profile instead of manufacturing external controls.
Their descriptors, steps, and observations identify `cadac_source_program` as
the command source and retain the source guidance/controller/actuator lowering
chain. Existing source command and realized-response outputs remain the
analysis evidence.

`run_composition_policy` is the deliberately small generic execution harness
for this boundary. It supplies a policy only the selected
ObservationFrame plus immutable VehicleInterfaceContract, requires the policy
to return a profile-bound PolicyDecision, and records every resulting
ActionFrame and EpisodeStep. Returning no decision ends the run at the current
truth boundary; exhausting its declared decision limit fails rather than
quietly accepting a partial episode. It is a policy-loop contract, not an RL
reward, learner, or vehicle-specific controller framework.

## Validation and authoring checks

The interface compiler and vehicle intake pipeline should reject or flag:

- duplicate or unversioned semantic IDs;
- a control offered at a fidelity that cannot realize it;
- a source control misrepresented as a physical effector;
- units, frames, vector dimensions, bounds, or transforms missing;
- no stated mapping from canonical intent to native control/wrench/effector;
- a resource-depletion claim without a resource-rate path;
- a policy observation sourced from future or interpolated truth;
- unavailable values represented as defaults or zeros;
- a physical-effector claim lacking commanded/actual/limit/achieved-wrench
  telemetry; and
- a segment requiring a control intent unavailable at the selected fidelity.

The report is both an authoring checklist and an automatic lowering gate. It
should tell a vehicle author what data or adapter implementation is missing
before a controller, showcase, or RL experiment begins.

## Delivery sequence

### I0 — vocabulary and descriptor core

Freeze the canonical IDs, channel descriptor, availability states, parameter
scopes, authority-profile vocabulary, resource-ledger semantics, and
truth/observation separation. Add interface conformance fixtures for a
fixed-wing, a multirotor, a rocket, and a passive body.

Exit: the current registry and adapters can be projected into an interface
report without changing simulation behavior.

### I1 — registry and CLI projection

Add VehicleInterfaceContract resolution to vehicle inspect/schema/endpoints
and a dedicated vehicle interface command. Publish action, parameter, status,
observation, and raw-sidecar schemas for an exact fidelity and authority
profile. Add a fail-closed interface validator.

Exit: callers can discover what they may configure, command, and observe
before opening a run.

### I2 — canonical runtime frames

Add ActionFrame, ObservationFrame, and StatusFrame while retaining
EpisodeChannel compatibility during migration. Map X8 and B747 first, then
Hummingbird, so the implementation proves both fuel/throttle-like propulsion
and battery/aggregate-thrust semantics.

Exit: a generic policy harness can use selected semantic profiles across an
X8 and Hummingbird without parsing native state names or receiving invented
channels.

### I3 — actuator and resource accountability

Map direct-wrench, physical-effector, and resource states into the canonical
status view. Add F-16 or X8 physical-surface evidence as the fixed-wing
witness, Hummingbird motor allocation as the rotor witness, and NESC
propellant/stage status as the resource/event witness.

Exit: every promoted control path records requested, applied, achieved,
limited, and unavailable authority consistently.

### I4 — authoring automation and composition integration

Extend vehicle intake scaffolding to generate the interface worklist. Require
new family manifests to declare mappings or concrete blockers for each
advertised fidelity. Have segment compilation, showcases, plots, and training
exports consume canonical channels.

Exit: adding a vehicle in an existing family is primarily data plus declared
bindings; adding a new family identifies the minimal new semantic extensions
before a bespoke controller or plot is written.

## Current implementation status

I0 and I1 now have an executable first slice:

- VehicleInterfaceContract resolves the current family registry into
  versioned parameter, action, effector, status, resource, diagnostic,
  authority-profile, and observation-profile descriptors;
- the vehicle interface CLI command exports the resolved contract and a
  fail-closed validation report before any episode opens; and
- fixed-wing, multirotor, staged-rocket, and passive-body fixtures verify
  that unavailable authority remains unavailable.

I2 is implemented for the active interactive witnesses while preserving the
legacy EpisodeChannel API. X8/B747 and Hummingbird episodes now accept an
ActionFrame, and emit an ObservationFrame and StatusFrame on every accepted
external truth boundary. `episode-info` now serializes the same explicit
`value_space` descriptor for each legacy native action and observation channel
as the semantic interface: native headings/yaw are circles, attitude is a
roll/pitch/yaw product space, vector telemetry is Cartesian, fractions are
unit intervals, nonnegative magnitudes are half-lines, and booleans remain
discrete. A newly added native channel must declare a `ValueSpaceSpec` or be
added to the centrally reviewed mapping; callers may not infer topology from
its unit or spelling. The contract maps the source-runtime X8 and B747
altitude, speed, and mass values into SI and retains their original source-unit
values only in the raw sidecar. Hummingbird maps bounded aggregate-attitude,
velocity/yaw, and live-waypoint commands into its aggregate-thrust seam and
publishes its engineering battery reserve and achieved-thrust limitation.

The native debug mapping is not a generic name heuristic. For the retained
source-table X8/B747 records it explicitly declares source-degree headings and
longitude as (S^1) circles with period (360), source angle quantities as
bounded coordinates, known positive source magnitudes such as speed, mass,
fuel, dynamic pressure, time, and thrust as half-lines, throttle as a unit
interval, and source state/segment flags as discrete numeric codes. The
canonical interface remains preferred for a portable policy; the legacy
schema is still fully typed so diagnostic consumers cannot accidentally apply
linear heading arithmetic or interpolate a state code.
Each frame step also records the requested semantic action, the actually
applied native action, and the mapped applied semantic action after limiting,
so response-law saturation cannot be hidden behind a clean canonical request.
Every public vehicle-run packet now includes the resolved
vehicle_interface.json artifact and its fingerprint, so a plot or downstream
consumer can inspect the exact action/status boundary used by that run.

The I2 exit witness is executable through `run_composition_policy`: one
profile-bound policy loop operates either X8 or Hummingbird without reading
native control/state names. Its trace retains the canonical request, mapped
native application, mapped semantic application after limits, committed truth
observation, and raw diagnostic sidecar. The active profiles remain
`truth_debug` development views; the planned sensor profile is not synthesized
from future or interpolated truth.

Composition-backed showcase artifacts may now retain a compact,
fingerprinted VehicleInterfaceEvidence record. It binds a board to the exact
interface ID, fidelity, control realization, available authority profiles,
available observation profiles, and claim boundary that applied at execution.
Legacy evidence artifacts remain readable without this record; new
composition-backed board builders should pass the resolved contract rather
than reconstructing a control claim from plot labels.

The catalog-wide command `taoryx vehicle interface-report` is the static
conformance gate for this layer. It resolves every registered family/fidelity
contract, verifies declared channel bindings, and rejects an available action
or observation profile that lacks a matching runnable episode binding. It
does not execute a trajectory; execution and qualification remain separate
evidence gates.

This is intentionally not an effector-promotion milestone:

- X8/B747 action profiles are labeled native_control_bridge, not physical
  elevon or surface allocation;
- Hummingbird remains aggregate thrust-vector pseudo-6DOF, not individual
  rotor allocation;
- direct-wrench episode bindings and surface-allocated runtime telemetry remain
  separate from the Hummingbird pseudo-6DOF stream; declared Hummingbird sensor
  profiles use the shared committed-boundary sensor machinery; and
- staged NESC, the source-pinned X-15-scaled reachability witness, and
  passive-body contracts explicitly expose no invented interactive control
  path. In particular, the X-15 witness's `response_law` label describes its
  internal reduced attitude response, not an external body-moment or surface
  command profile.

## Definition of done

The interface contract is complete for an advertised vehicle realization when
a caller can inspect a versioned model before execution, select only declared
bounded configuration and action values, receive a timestamped canonical
observation/status contract at accepted truth boundaries, distinguish
requested from applied and achieved control, inspect relevant resource and
authority status, and understand exactly which channels are unavailable,
estimated, source-backed, or physically realized. The same resolved contract
must drive composition, interactive control, AI/RL export, telemetry, and
showcase evidence without a hidden vehicle-specific fallback.
