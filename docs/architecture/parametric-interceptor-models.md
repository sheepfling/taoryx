# Parametric interceptor model architecture

`taoryx-parametric-interceptors` is an independent direct plug-in for quickly
turning incomplete ontology records into honest low-fidelity simulation cases.
It is separate from `taoryx-cadac`: a profile may name a CADAC model as a
calibration reference, but it neither imports CADAC code nor copies a CADAC
vehicle/environment/controller stack.

Its source-native channels remain local-NED/NEU where that is the truthful
surrogate contract. The common Mission Composition result nevertheless always
adds `sample.standard_ecef`: a WGS-84 ECFC/ECEF position, Earth-relative
velocity, native-or-finite-difference acceleration, body-frame angular
velocity, and an ECEF-from-body quaternion. Since this plug-in has no
profile-specific global launch datum, that position is
explicitly labeled as the shared equatorial tangent embedding; its orientation
is explicitly labeled kinematic unless a selected realization publishes
physical attitude truth, and its body angular velocity is derived from those
standard quaternions when no physical rate is available. The sidecar is a
portable consumer view, not a claim
that the point-mass surrogate has acquired CADAC-style Earth rotation or
rigid-body dynamics.

The direct developer APIs (`PointMassRun` and `Pseudo6Run`) expose the same
per-sample minimum. The pseudo-6DOF tier supplies its explicit NED-from-body
attitude and body rate to that projection; the point-mass tier honestly uses a
velocity-aligned kinematic orientation and derived rate. Both preserve their
source-native fields alongside the common ECEF sidecar.

## Developer path

The authoring path has four intentionally small stages:

```text
plain mapping / ontology record
    -> sparse InterceptorEvidenceProfile
    -> versioned archetype resolver + assumption case
    -> immutable ResolvedInterceptorProfile + fingerprint
    -> standard Mission Composition provider
```

`interceptor(id, **values)` is the short path. Scalars, strings, booleans, and
target-class tuples are accepted directly. Raw values are labeled
`simulation_assumption`, not `observed`. A developer upgrades individual fields
with `observed(...)` or `reported(...)` as source records become available.
The same small API provides `derived(...)`, `inferred(...)`,
`unavailable(...)`, and `variant_interval(...)`, so Python authors can express
the complete catalogue evidence vocabulary without constructing Pydantic
records directly. Observed, reported, derived, inferred, gap, and interval
helpers accept either one `source_record_id` or an ordered
`source_record_ids` sequence; ambiguous, duplicate, empty, or whitespace-damaged
identifiers fail at authoring time.
Omitted fields remain explicit `archetype_assumption` records containing the
archetype ID, resolution method, confidence, and assumption case.
`ParametricInterceptorMissionCompositionProvider.from_yaml(...)` is the direct
file-to-provider path and accepts multiple profile files plus one optional
assumption-case selection.

Ontology-shaped seeds can use
`load_catalogue_interceptor_record(...)` or
`interceptor_from_catalogue_record(...)`. That adapter maps common catalogue
names such as `launch_mass`, `body_diameter`, and `guidance_architecture` onto
the canonical simulation profile without requiring the ontology itself to
adopt unit-suffixed Python field names.
`ParametricInterceptorMissionCompositionProvider.from_catalogue_yaml(...)`
loads one or several such records directly into an isolated provider.

The installed `taoryx-interceptor` command is the file-first front end to this
same path. `init` creates a valid flat or catalogue-shaped scaffold, `schema`
emits either or both machine-readable authoring contracts, `inspect` emits a
versioned evidence/readiness report, and `run` accepts advertised mission
parameters before invoking the ordinary provider-owned Composition runner. It
does not register ambient directories, add a second runtime, or make discovery
depend on environment-variable ordering.

Catalogue ingestion is loss-intolerant. Pydantic-backed input models forbid
unknown keys at every fixed structural level, and the adapter rejects two
source names that normalize to the same canonical parameter across `evidence`
and `derived`. A populated value cannot also carry a gap status, a null gap
cannot claim an evidence origin, and an unknown `resolution_status` cannot fall
back to `unassessed`. `parameter_set_version` and
`calibration_reference_model_id` are preserved into the sparse and resolved
profiles. This means a typo or format mismatch fails before archetype defaults
could hide the lost author input.

The report preserves the complete resolved profile while also projecting the
facts an agent or UI needs immediately: origins grouped by parameter, evidence
gaps, calibration-target presence, required diagnostics, fidelity and
realization IDs, waypoint and target-track authorities, control channels, and
output channels.
It also groups every resolved parameter by the versioned
`taoryx.parametric-interceptors.parameter-usage/v1` contract. Direct dynamics
inputs and active resolver dependencies identify exact consumers and resolved
sinks. Overridden coarse selectors, runtime-only advisories, calibration
targets, derived summaries, and evidence-only fields are distinct classes.
Unknown unconsumed parameters fail classification rather than silently joining
the metadata inventory.
Its `performance_claim_status` is always `unqualified_surrogate` at this
prototype maturity. A named profile that is both archetype-dominant and carries
required diagnostics is `resolver_only`; CLI execution requires the explicit
`--allow-unqualified` acknowledgement. This keeps the sparse PAC-3 MSE witness
useful for resolver development without turning a successful schema resolution
into an implied performance model.

The flat profile API is canonical-unit-only. The catalogue adapter is the
source-unit boundary: it accepts common mass, length, area, time, speed, angle,
angular-rate, angular-acceleration, and ratio units; converts them with explicit
fixed factors; and retains the original scalar/unit plus method. Family
intervals retain both canonical and source bounds. Incompatible dimensions fail
before the resolver runs. Context-dependent conversions such as Mach are not
accepted without a future operating-condition contract.

Intake tools can inspect the same boundary through
`canonical_unit_for_interceptor_parameter(...)`,
`supported_interceptor_units(...)`, and
`canonicalize_interceptor_value(...)`. A copy-ready mixed-unit record is in
`examples/parametric_interceptors/mixed_unit_authoring_catalogue_seed.yaml`.

This separation is the core data invariant:

- observed/reported values describe the research record;
- derived values state their method and input fields;
- archetype and simulation assumptions remain simulation-only;
- calibrated values may be added later without rewriting source facts.

Evidence origin and model use are independent axes. An observed diameter may
be an active resolution input when it derives reference area, or an inactive
retained value when a source-qualified reference area is already supplied. A
simulation-assumption guidance archetype can be a direct dynamics input, while
an observed guidance family remains evidence-only because the prototype does
not implement that physical seeker/controller architecture. Composition
properties and file-first reports expose both axes.

Unavailable evidence is a separate typed record. A classified or unresolved
value is retained as `EvidenceGap(parameter_id, status, ...)` and is never
inserted into the resolved numeric parameter dictionary. In particular,
reported range, speed, and altitude are optional calibration observations;
archetypes do not fabricate them when the source leaves them unavailable.

`EvidenceSourceValue` retains an original value such as `356 lb` beside its
canonical value. `EvidenceInterval` retains canonical and original family-level
ranges without silently selecting a point. These records, resolver diagnostics,
catalogue identity, variant basis, application context, and resolution status
all participate in the resolved fingerprint and Composition advertisement.

The resolved profile is immutable and exposes a SHA-256 fingerprint over every
parameter and evidence record. Conservative, nominal, and optimistic cases
therefore cannot accidentally share a cache identity.

## Runtime tiers

The default executable tier is `point_mass_3dof`. It owns only interceptor
surrogate forces and bounded waypoint steering. The selectable
`attitude_response_pseudo_6dof` tier adds explicit roll, pitch, yaw, body-rate,
and angular-acceleration response states. It is a bounded second-order response
law, not a rigid-body moment balance or a copied CADAC controller.

Both tiers use the same sparse profile and guidance/control kernel. The default
`fixed_waypoint_intercept` mission uses four fixed/live waypoint channels. The
selectable `constant_velocity_target_intercept` mission uses seven fixed/live
target-reference position, velocity, and capture-radius channels. The
`direct_lateral_acceleration_control` mission exposes three local-NEU
acceleration channels in `m/s^2`. A developer chooses tier and mission
independently:

```python
configuration = provider.configuration(
    profile.model_id,
    fidelity="attitude_response_pseudo_6dof",
    mission_template_id="constant_velocity_target_intercept",
)
```

`provider.configuration(..., startup_authority_profile_id=...)` optionally
binds an authority choice into the Composition. If omitted, a session selects
`live_waypoint_guidance`, `live_target_track_guidance`, or
`live_direct_lateral_acceleration` from the chosen mission. Its projected
action schema includes only that authority's channels; batch execution does
not need a startup authority.

The three command grammars are exclusive under
`taoryx.parametric-interceptors.exclusive-mission-controls/v1`. The portable
schema carries default values for every grammar so one compiled shape can
select any mission. After ordinary schema/unit validation, provider validation
rejects every non-default channel that the selected mission does not consume,
using `inactive-mission-control` and its exact configuration path. This avoids
silently ignored waypoint, target-track, or direct-acceleration commands while
retaining one stable schema fingerprint.

Guidance evidence and executable lowering are separate. `guidance_family`
retains the sourced architecture label—command, SARH, active radar, infrared,
or another catalogue term. `guidance_archetype` independently selects the
simulation law and is normally an archetype or developer assumption. The
implemented choices are `waypoint_pursuit` and `proportional_navigation`; the
latter uses the resolved positive `navigation_constant`, target-relative
closing speed, and inertial line-of-sight rate. Both runtime tiers call the same
pure guidance function before applying their distinct authority/attitude
realizations.

The target-track mission propagates caller-supplied local north/east/up target
truth with constant velocity. At each accepted boundary, the selected
registered `relative-state-track` provider projects that scene into a versioned
`taoryx.tracking.relative-state/v1` packet. Guidance consumes the measured
relative position and velocity rather than reading target truth directly. An
invalid packet makes guidance unavailable with the explicit
`target_track_unavailable` mode; there is no hidden truth fallback. Capture
volume evaluation and event localization remain truth based so measurement
noise cannot redefine the mission outcome.

The mission reports both propagated target truth and the sensor packet's
applicability, validity, reason, sequence, timing, schema, sensor-frame relative
state, range, bearing, elevation, closing speed, and line-of-sight rate. At a
partial live update, the provider first rebases target truth to the current
session time; a velocity-only update therefore cannot teleport it to an earlier
epoch. The standard provider is a direct-geometry measurement abstraction, not
a propagation, signature, gimbal, track-manager, datalink, physical-seeker,
target-dynamics, fire-control, or terminal-hit model.

The direct-control mission is an external controller seam, not a second
dynamics path. Its held local north/east/positive-up demand is projected onto
the plane transverse to current velocity, then lowered through the same
instantaneous force-authority allocator used by waypoint and target-track
guidance. Point mass realizes that bounded force directly. Pseudo-6DOF first
drives its existing bounded attitude-response law and then realizes the same
force authority. Composition reports the external law ID, active direct mode,
raw accepted hold, projected command, achieved response, authority, utilization,
and saturation. The three local-NEU stages are deliberately separate:
`control.lateral_acceleration.accepted.local.*` preserves the caller's held
vector, `guidance.lateral_acceleration.commanded.local.*` is the
velocity-transverse projection consumed by the shared guidance/authority path,
and `guidance.lateral_acceleration.achieved.local.*` is the force-limited or
force-and-attitude-limited realization. Per-control feedback binds to the raw
accepted stage; acceptance is not an achievement claim. No waypoint capture or
target sensor is fabricated, and the interface makes no actuator,
control-surface, autopilot, or rigid-body moment claim.

Each direct axis is bounded symmetrically by the exact resolved
`max_lateral_acceleration_mps2` for that model. The portable configuration
schema, Composition channel, native binding, selected live-session action
schema, and agent action-space projection all repeat that same interval. Agent
normalization is affine to `[-1, 1]` and requires no external statistics. This
is a component-wise authoring envelope, not a promise that the combined vector
or current dynamic-pressure/TVC state can realize the request; the ordinary
commanded/achieved vectors and saturation diagnostics remain authoritative.

Static-waypoint proportional navigation has an explicit live-retarget edge
case. When closing speed is nonpositive, its PN demand would be zero even when
the new waypoint lies off course. The shared law therefore emits a pursuit
demand with runtime mode `capture_fallback`. Composition publishes the resolved
law ID, active mode, closing speed, LOS-rate magnitude, and navigation constant,
so a consumer can identify rather than infer that transition.

Atmospheric samples come from Taoryx's standard `EnvironmentProvider`
boundary, and gravity is injected or evaluated with the core inverse-square
equation. Composition advertises both dependencies as categorical
`runtime.environment_model_id` and `runtime.gravity_model_id` parameters. The
provider owns ID-to-implementation registries, and the exact selections are
part of the prepared-configuration fingerprint. Batch diagnostics, live
lowering evidence, and calibration scenarios repeat the IDs so a replay cannot
silently substitute an unnamed atmosphere or gravity equation. A custom ID
must remain immutable in meaning across runs.

Navigation measurement projection follows the same explicit-dependency rule.
`runtime.sensor_suite_id` selects an `InterceptorSensorSuite` with its own
version and fingerprint. A suite names registered Taoryx providers for
translation-only point-mass truth, pseudo-6DOF truth, and target-relative scene
truth. Construction validates the manifests' supported truth modes, target
selector, and required payload schemas (`taoryx.acceleration.increment/v1`,
`taoryx.imu.increment/v1`, and `taoryx.tracking.relative-state/v1`), so an
incompatible sensor cannot be advertised and then fail ambiguously during
propagation. Providers must support instantaneous accepted-boundary sampling;
the surrogate samples at every accepted integration boundary and does not
silently assume the manifest's default sensor clock. The portable suite selects
the registered `translation-acceleration`, `ideal`, and
`relative-state-track` providers.

The provider creates fresh sensor plug-in instances for each batch execution
and each session initialization or reset. Session checkpoints own the selected
providers' accepted-history and random-generator snapshots. Repeated
observation at an accepted boundary reuses the held packet instead of consuming
another noise draw. Batch diagnostics and live lowering evidence report suite
ID, version, fingerprint, and active provider kinds. Output channels retain the
native measurement envelope—sample and
availability time, delivery latency, fresh-delivery status, monotonic sequence,
and versioned payload schema—alongside validity and typed increments. The
causal projection never exposes a packet before `available_at`; it holds the
latest delivered packet between arrivals and checkpoints every pending packet.
Sensor-suite configuration is a simulation
dependency, not interceptor evidence, and does not assert a seeker or hardware
implementation.

The environment wind contract is consumed, not merely exposed. ECFC
radial/east/north wind is subtracted from local north/east/up vehicle velocity;
the resulting air-relative vector drives airspeed, Mach, active drag
coefficient, dynamic pressure, drag magnitude, and drag direction in both
tiers. The runtime validates finite
thermodynamic/wind values, ECFC wind framing, nonnegative density/pressure, and
positive temperature/speed of sound before force evaluation. Translation-only
truth is projected through the selected compatible registered translation
sensor; it cannot invent attitude or gyro data. Pseudo-6DOF truth is projected
through the selected compatible registered IMU. The portable suite resolves to
Taoryx's `TranslationAccelerationAdapter` and `IdealImuAdapter`, respectively.
The plug-in does not
implement a private sensor model or carry a CADAC atmosphere, gravity deck,
seeker, actuator, or rigid-body subsystem.

Aerodynamic drag is a shared, immutable Mach–coefficient schedule rather than a
constant hidden in each kernel. The ergonomic coarse route resolves
`aero_archetype` plus `drag_class` into a versioned normalized shape and
amplitude. Every generated point remains `archetype_assumption`; assumption
case and `drag_scale` affect all ordinates and the resulting fingerprint.

An authored `drag_coefficient_schedule` replaces both coarse selectors so none
remain as unused resolved baggage. Its contract requires strictly increasing
finite nonnegative Mach points, positive coefficients, explicit provenance,
linear interpolation, and held endpoints. Observed/reported schedules require
source IDs. Scaling an authored curve is represented in the resolved schedule
instead of mutating the source profile; a fitted `drag_scale` produces a
`calibrated` resolved curve. Both runtime tiers call the same interpolation
method. Composition publishes schedule identity, origin, fingerprint, domain,
points, and policies plus the active coefficient at every sample.

Lateral control authority is likewise one shared reduced-order evaluator, not
a constant duplicated in the two kernels. The resolver selects one of three
explicit configurations:

- `aerodynamic`: `q * S * Cn_limit / mass`;
- `thrust_assisted`: `thrust * sin(max_vector_angle) / mass`;
- `mixed`: the sum of those enabled components.

The enabled sum is clipped to the separately resolved structural maneuver
limit. Composition therefore distinguishes the structural envelope from
instantaneous available authority and advertises both. It also publishes the
configuration, each component, the unclipped sum, structural-clipping status,
current-authority utilization, structural-envelope utilization, and stable
configuration-specific saturation reasons. Both point mass and pseudo-6DOF
consume the same evaluator at the same environment/propulsion boundary.

After clipping, one shared, profile-selected policy allocates achieved demand:

- `aerodynamic_first` spends aerodynamic authority before TVC authority;
- `thrust_vector_first` reverses that priority;
- `proportional` uses the instantaneous unclipped component-authority ratio.

The thrust-vector share determines an achieved vector angle and an axial
projection satisfying the scalar thrust-magnitude relation; the runtime does
not add lateral thrust while also retaining full axial thrust. Composition
exposes the active policy, all supported policy IDs, both achieved shares,
achieved angle, and axial thrust. The flat authoring JSON Schema exposes the
same enum. Each policy is a visible simulation assumption and remains distinct
from a physical control allocator. With only one enabled component, all three
policies reduce to that component without inventing another authority source.

Both tiers retain vector direction instead of collapsing guidance to only a
scalar demand. Composition publishes commanded and achieved north, east, and
positive-up components in a dedicated `local_neu` frame. The commanded vector
is the unbounded output of the shared guidance law. The point-mass achieved
vector preserves that direction while force authority scales its magnitude;
the pseudo-6DOF achieved vector follows the response-body direction and may
therefore lag or differ from the command. The advertised scalar channels are
the corresponding vector magnitudes. Unavailable target measurements produce
zero command and achieved vectors; zero force authority preserves a visible
nonzero command but produces a zero achieved vector.

One shared tracking calculation also publishes actual achieved/command
magnitude fraction and the angle between commanded and achieved vectors. The
fraction is bounded to `[0, 1]`; zero demand plus zero achievement reports one,
while an unsupported nonzero command reports zero. Direction error is valid
only when both vectors are nonzero. Its numeric value is zero when invalid, so
consumers must use the adjacent validity channel rather than interpreting the
sentinel as alignment. This common actual-achievement metric is intentionally
different from structural-envelope utilization and the pseudo-6DOF
available-authority support fraction.

The pseudo-6DOF response law is coupled to this same operating-point authority.
Its bounded angular acceleration is multiplied by
`min(current_available_authority / commanded_lateral_acceleration, 1)` for a
nonzero command. Zero demand reports a support fraction of one by convention,
with a separate boolean reporting whether any current authority exists. Zero
authority yields zero response-law angular acceleration; a pre-existing rate
coasts without hidden damping. Composition scopes the availability, support
fraction, and authority-limited channels to the pseudo-6DOF realization. The
point-mass force calculation is unchanged.

Target-track sensor failure follows the same no-fallback boundary for both
guidance and attitude. Range/capture accounting may still inspect objective
truth, but an unavailable measurement cannot rotate the pseudo-6DOF attitude
command toward that truth.

Pseudo-6DOF aerodynamic-state introspection uses the selected standard
environment sample rather than a private atmosphere or wind route. Local
north/east/up air-relative velocity is rotated through the response attitude
into forward/right/down body axes. The published geometric definitions are
`alpha = atan2(w, u)` and `beta = atan2(v, hypot(u, w))`. A nonzero airspeed is
valid even at zero density; exactly zero airspeed reports an invalid flag and
zero components/angles. These are fidelity-scoped response-state outputs only.
They do not feed the coarse force surrogate and do not imply an angle-of-attack
state, coefficient deck, static/dynamic stability derivative, or aerodynamic
moment balance.

The ergonomic route still needs only `maneuverability_class`; the archetype
supplies a labeled configuration and coefficient/angle defaults. Developers
may instead set `control_configuration`,
`normal_force_coefficient_limit`, and/or
`max_thrust_vector_angle_rad` directly. Aerodynamic and thrust-only modes reject
irrelevant authored parameters instead of carrying unused baggage. A sourced
`thrust_vectoring` control feature infers `mixed` only when no explicit
simulation configuration exists, retaining the feature's lineage and an
`inferred` origin. This is a surrogate allocation choice and never promotes a
feature claim into evidence of a controller or actuator implementation.

`maneuverability_scale` scales the normal-force coefficient, while the
structural maneuver envelope remains determined by maneuverability class and
assumption case. Thrust-vector authority naturally disappears whenever the
shared propulsion program produces zero thrust; aerodynamic authority remains
dynamic-pressure dependent. No angle-of-attack state, coefficient deck,
actuator dynamics, physical control allocation, autopilot, or stability
qualification is implied by this force-level boundary.

The achieved aerodynamic share drives one shared maneuver-drag evaluator:
`Cn = m*a_aero/(q*S)`, `Cdm = k*Cn^2`, and `Dm = q*S*Cdm`. The nonnegative
`maneuver_drag_factor` defaults to a visible `0.1` archetype assumption and may
be authored directly (or as catalogue alias `induced_drag_factor`). Zero
dynamic pressure with nonzero aerodynamic force fails as inconsistent; zero
demand produces exactly zero maneuver drag. Thrust-vector allocation never
enters this aerodynamic penalty.

The Mach schedule remains the base coefficient contract. Runtime telemetry
keeps base coefficient/drag, achieved normal-force coefficient, maneuver
factor/coefficient/drag, and total coefficient/drag distinct. `drag_scale`
continues to affect only the base schedule, so fitting cannot silently rewrite
the maneuver penalty. This quadratic relation is a reduced-order load penalty,
not a sourced polar, angle-of-attack state, induced-drag identification, CFD
result, or aerodynamic deck.

Applicability is a separate optional contract. Canonical flat profiles may
declare minimum/maximum altitude and Mach bounds; catalogue input accepts the
corresponding `operating_*` aliases and performs the normal provenance-retaining
unit conversion. Bounds are never archetype-filled. A profile with no bounds
therefore reports `not_declared`, not an invented unbounded or valid envelope.

Both runtime tiers evaluate the same immutable envelope against geometric
altitude and air-relative Mach after the selected standard environment sample.
The result is `within_declared_envelope` or `outside_declared_envelope`, with
deterministic altitude-then-Mach reason ordering. The current enforcement is
explicitly advisory: the runtime exposes extrapolation without terminating the
mission or masking controls. Composition publishes the contract, enforcement,
declared bounds, declaration flag, status, and reasons in model and sample
metadata.

Applicability bounds are model-domain evidence, not reported performance.
Reported altitude, speed, range, and engagement range remain scenario-qualified
calibration observations and are never promoted into applicability limits by
the resolver.

Both tiers also consume the same immutable `PropulsionProgram`. It interprets
the resolved architecture and `ThrustProfileSchedule` instead of embedding
separate motor logic in each dynamics kernel. `neutral`, `regressive`,
`progressive`, and `boost_sustain` classes resolve to versioned schedules.
Each schedule spans normalized active-burn fraction `0..1`; its raw relative
multipliers are integrated and divided by their area so the executable shape
has unit mean and preserves resolved nominal active-burn impulse.
Single-pulse models burn continuously to burnout. A `dual_pulse_solid` model
executes pulse one, an explicit zero-thrust coast, pulse two, and burnout while
depleting only the propellant allocated to the active pulse.

The short authoring path remains three fields:

- `propulsion_architecture`
- `burn_time_class`
- `thrust_profile_class`

Developers can override the built-in shape with a structured
`thrust_profile_schedule` containing only an ID, two or more
`burn_fraction`/`multiplier` points, and optional provenance. The first and last
fractions must be exactly `0` and `1`, intermediate fractions must increase,
and the integral must be positive. `linear` is the ergonomic default;
`step_previous` expresses discontinuous boost/sustain shapes without epsilon
points. Observed or reported shapes require source-record IDs. The exact
normalization policy, raw area, normalized peak, interpolation, points, origin,
method, source IDs, and fingerprint are all Composition properties.

Absolute amplitude is independently authorable as `nominal_thrust_n`, defined
as active-burn mean thrust before assumption-case and `thrust_scale` factors.
The source/assumption value remains unchanged in the resolved profile. Runtime
uses a separate `thrust_n`, with dependency, origin, case, scale, and source
links advertised, and publishes `active_burn_total_impulse_n_s` as a derived
summary. Catalogue aliases `nominal_thrust`, `mean_thrust`,
`average_thrust`, and `mean_active_burn_thrust` accept `N`, `kN`, or `lbf` and
retain the original source scalar.

The explicit schedule replaces only the executable time shape; explicit
nominal thrust replaces only the class-to-thrust-to-weight amplitude fallback.
If both are supplied, the resolver removes its archetype-default class and
rejects an explicitly supplied redundant class. If only one side is explicit,
the exact schedule dependency manifest shows the class reaching only the
missing side. `thrust_scale` and assumption case affect executable amplitude
without overwriting evidence. This prevents a shape-only public curve from
being misrepresented as absolute thrust and prevents calibration from
relabeling its result as source data. The same normalized schedule is applied
independently to each dual-pulse interval, while
`second_pulse_thrust_ratio` and `second_pulse_propellant_fraction` keep pulse
amplitude and propellant allocation explicit.

The source-shaped alternative is `AbsoluteThrustCurve`. It accepts two or more
absolute `time`/`thrust` points plus shared source units (`s`/`ms` and
`N`/`kN`/`lbf`). The compiler converts units and analytically integrates using
the declared `linear` or `step_previous` interpolation. It derives:

- `burn_time_s` from the final time ordinate;
- `nominal_thrust_n` from total impulse divided by duration;
- `thrust_profile_schedule` by dividing time by duration and thrust by mean;
- `active_burn_total_impulse_n_s` through the normal resolved runtime path.

Observed/reported curves require source IDs. Their derived amplitude, duration,
and normalized shape are labeled `derived`, retain those IDs, and never become
observations merely because the input points were observed. Composition keeps
the original source-unit points, units, interpolation, origin, confidence,
method, duration, mean, impulse, source IDs, and fingerprint. The parameter
usage graph shows the curve reaching the existing shared propulsion inputs;
there is no curve-specific runtime.

The v1 curve contract represents one continuous single-pulse burn. It replaces
the coarse duration and shape selectors plus separately authored amplitude and
schedule fields. Any overlap fails authoring validation, and pairing the curve
with `dual_pulse_solid` fails resolution. This boundary avoids silently
interpreting a zero-valued source interval as a dual-pulse coast.

`DualPulseThrustProgram` is the corresponding explicit multi-pulse contract. It
contains first- and second-pulse `AbsoluteThrustCurve` records plus an
independently sourced coast duration. Compilation derives
`dual_pulse_solid`, each pulse duration, combined active-burn mean thrust,
second/first mean-thrust ratio, and one normalized schedule per pulse. The
existing runtime's ratio normalization then exactly reconstructs each curve's
mean while preserving combined impulse; it does not add a second propulsion
kernel. Point-mass and pseudo-6DOF therefore consume identical pulse histories.

The program alone does not infer propellant allocation from impulse. Without a
specific-impulse model, that inference would silently claim equal efficiency
between pulses. `second_pulse_propellant_fraction` consequently remains an
independent evidence/assumption field by default.

An author may instead supply `effective_specific_impulse_s`. This opt-in route
requires explicit launch mass plus explicit thrust amplitude and timing; an
absolute curve/program supplies amplitude and timing together. The resolver
uses nominal active-burn impulse divided by effective Isp and standard gravity
to derive propellant fraction and burnout mass. With a dual-pulse program, the
single effective Isp declares equal effective efficiency across pulses and
therefore also derives the second-pulse propellant fraction from the two
nominal impulse shares. Separate per-pulse efficiencies are not inferred.
Overlapping authored burnout mass, total propellant fraction, or pulse split
fail closed.

Mass lowering uses nominal rather than assumption-case-scaled executable
thrust, so uncertainty cases do not change vehicle mass. The Isp and each
derived mass value are advertised through the normal usage manifest, with
source records and operand dependencies retained. Program, coast, nested
curves, normalized schedules, derived ratio, mean, durations, total impulse,
source-unit points, and fingerprints remain exposed through Composition.
Overlapping coarse timing, amplitude, ratio, or schedule inputs also fail
validation.

Known timing can be added with `burn_time_s` for a single pulse, or
`first_pulse_burn_time_s`, `inter_pulse_coast_time_s`, and
`second_pulse_burn_time_s` for a dual pulse. The optional
`second_pulse_thrust_ratio` and `second_pulse_propellant_fraction` expose the
remaining coarse allocation knobs. A dual-pulse `burn_time_s` is the sum of
active pulse durations and excludes coast time. Inconsistent timing fails at
the resolver boundary. Missing dual-pulse values remain labeled
`archetype_assumption`; a sparse evidence diagnostic such as
`motor_pulse_timing_missing` is not cleared merely because the resolver made a
runnable schedule.

The standard Composition surface advertises:

- launch plus local-NED waypoint, target-track, and local-NEU direct-acceleration
  parameters with canonical units;
- separate fixed/live waypoint, constant-velocity target-track, and direct
  lateral-acceleration authorities at both tiers;
- a versioned exclusive mission-control-scope contract that rejects
  non-default commands owned by an inactive grammar;
- exact configuration/session bindings plus accepted waypoint or target
  reference-state and capture-radius feedback;
- propagated target state, relative velocity, linear closest-approach time, and
  predicted miss distance plus generic objective range/captured status;
- sourced guidance-family metadata kept distinct from resolved simulation-law
  ID, active fallback mode, closing speed, LOS rate, and navigation constant;
- commanded and achieved lateral acceleration, the structural maneuver limit,
  and structural-envelope utilization normalized to `[0, 1]`;
- commanded and achieved local north/east/positive-up acceleration components
  in the explicitly advertised `local_neu` frame, with scalar/vector magnitude
  parity;
- model-specific symmetric direct-acceleration component bounds and a
  normalization-ready `[-1, 1]` agent action-space projection;
- normalized actual command achievement plus explicitly valid/invalid angular
  direction error, shared by both fidelity tiers;
- resolved control configuration, aerodynamic and thrust-vector authority,
  their unclipped sum, current clipped authority, current-authority
  utilization, and structural-clipping status;
- reduced-order allocation policy, achieved aerodynamic and thrust-vector
  shares, achieved vector angle, and remaining axial thrust;
- pseudo-6DOF-only attitude-response authority availability, commanded-demand
  support fraction, and authority-limited status;
- guidance availability, control saturation, and deterministic active-limit
  reason codes (`none` or a stable `+`-delimited set);
- standard mission phase and vehicle-operational status for runtime authority
  introspection;
- optional declared altitude/Mach applicability, advisory enforcement, current
  status, and stable out-of-envelope reasons;
- position, velocity, mass, thrust, selected environment/gravity IDs, density,
  pressure, temperature, speed of sound, ECFC wind, airspeed, Mach, active drag
  schedule coefficient, dynamic pressure, achieved maneuver-drag split, total
  coefficient, and total drag;
- binary internal motor schedule, normalized achieved motor level, remaining
  propellant, thrust availability, propulsion phase, and active pulse index;
- selected sensor-suite identity in configuration, batch diagnostics, and live
  lowering evidence;
- registered target-track applicability, validity/reason, packet timing,
  sequence, schema, target/frame IDs, sensor-frame relative position and
  velocity, range, azimuth, elevation, closing speed, and line-of-sight rate;
- standard translation-acceleration validity, interval, packet timing, payload
  schema, and ECI delta-velocity measurements at the point-mass tier;
- pseudo-6DOF attitude commands and achieved attitude/body response;
- registered IMU validity, interval, packet timing, payload schema,
  delta-velocity, and delta-angle measurements at the pseudo-6DOF tier;
- model version, profile fingerprint, assumption case, and selected evidence
  properties, including the parameter-usage contract and exact resolved usage
  manifest.

Point-mass batch/session execution shares `PointMassKernel`; pseudo-6DOF
batch/session execution shares `Pseudo6Kernel`. Each owns explicit state,
evaluation, and bounded semi-implicit advance primitives. The session manager
supplies standard lifecycle validation, authority selection,
requested/applied/achieved feedback, and interface-fingerprint-bound
checkpointing; the plug-in does not define a second streaming route.
Point-mass and pseudo-6DOF checkpoints include the selected registered sensor
provider's snapshot. Session opening rejects an integration step whose implemented
response-law discretization is unstable. `waypoint_capture` or
`target_intercept` disables achieved guidance while leaving the live authority
available for retargeting. `ground_impact` completes
the episode and the common manager masks every action with
`episode_terminal`. Propulsion burnout is independent: it reports unavailable
thrust and the `burnout` motor phase while coasting waypoint authority remains
available. All executions are plausibility and integration
surrogates, not weapon-performance predictions, terminal-homing qualification,
stability proof, or CADAC parity claims.

Capture detection is continuous within each discrete held-derivative step. The
shared event helper evaluates the same semi-implicit translation polynomial as
the kernels, partitions its squared-range quartic into monotone intervals, and
localizes the first capture-volume entry. Batch propagation commits an exact
event boundary and terminates; an earlier or tied descending ground contact has
precedence. Stateful propagation preserves its requested action-hold duration
but emits the localized event and latches
`guidance.objective.capture_occurred` until retargeting. The separate
`guidance.objective.captured` field remains instantaneous. This prevents
cadence-dependent tunneling without turning capture radius into a fuze,
lethality, or probability-of-kill model.

Most authors should leave the response fields out. They resolve visibly from
`control_bandwidth_class` and `maneuverability_class`. When a calibration task
needs more control, these five optional fields are accepted in the same Python
or YAML profile:

- `attitude_bandwidth_rad_s`
- `attitude_damping_ratio`
- `max_body_rate_rad_s`
- `max_body_acceleration_rad_s2`
- `max_bank_angle_rad`

No subclass, factory registration, or response-law code is required to tune
them.

## Calibration and assumption sensitivity

The calibration layer is a comparison boundary, not another dynamics model.
`InterceptorCalibrationScenario` fingerprints the complete launch, waypoint,
duration, integration step, selected fidelity, standard environment/gravity
identities, exact sensor-suite ID/version/fingerprint, declared targets, source
basis, and claim boundary. The same scenario can therefore be replayed without
relying on notebook state, an unstated definition of “maximum range,” or a
silently changed measurement configuration.

`calibration_scenario_from_reported_profile(...)` is deliberately strict:

- it promotes only reported parameters actually present in the resolved
  profile;
- it requires a non-empty scenario-basis statement;
- it carries target origin and source-record IDs into each scored metric;
- it records requested-but-unavailable targets rather than defaulting them;
- it refuses to build a scored scenario when every requested value is missing
  or classified.

The evaluator runs either `point_mass_3dof` or
`attitude_response_pseudo_6dof` and extracts one common observable vocabulary:
peak speed, maximum altitude, maximum horizontal displacement, elapsed time,
terminal waypoint range, remaining propellant, and waypoint capture. Results
use the core `TrajectoryEvaluation` envelope, so validity, qualification,
feasibility, execution outcome, scored metrics, requested controls, resources,
events, and evidence gates remain separate. Parametric interceptor calibration
results are explicitly `unqualified`; a numerical match alone cannot change
that status.

The evaluator instantiates the scenario-bound navigation and target-track
providers for the selected fidelity. The result repeats the suite identity and
both active provider kinds. Direct use may omit implementations only for the
portable standard environment, gravity, and sensor suite. Provider-mediated
evaluation resolves custom IDs from its registries and rejects missing,
version-drifted, or fingerprint-drifted suites before propagation.

`compare_assumption_cases(...)` repeats the exact scenario for conservative,
nominal, and optimistic resolver cases and ranks normalized target error. This
is useful for sensitivity and bounding. It performs no optimizer update,
produces no calibrated parameter set, and cannot convert the selected case into
observed evidence.

The file-first equivalent is
`taoryx-interceptor compare-cases PROFILE SCENARIO`. All three cases are the
default, while repeatable `--case` arguments select a subset. The versioned
comparison contract binds the immutable sparse-profile fingerprint, scenario
fingerprint, distinct resolved-profile fingerprints, full evaluation results,
and ranking. Resolver-only records require the same explicit
`--allow-unqualified` acknowledgement as direct execution; a successful
sensitivity screen does not make a sparse profile runnable by default.

Developers may author a scenario directly in Python or load the same typed
shape from YAML with `load_interceptor_calibration_scenario(...)`. The
copy-ready example is
`examples/parametric_interceptors/generic_medium_sam_calibration.yaml`.
Applications that already hold the Composition provider can use its
`reported_calibration_scenario(...)` and `evaluate_calibration(...)` methods;
those methods resolve the exact selected model before entering the same typed
evaluator.

### Fit campaigns and application receipts

`InterceptorFitCampaign` extends the comparison layer to multiple scenarios
without weakening the evidence model. Its portable default uses Taoryx's
built-in bounded RQP optimizer; optional registered optimizer backends may be
selected explicitly. The objective is the mean squared normalized target error
across every declared scenario and target.

Fitting resolves each scenario's environment, gravity, and sensor suite before
evaluation. Nonstandard dependencies must be supplied by ID; the fitter fails
closed when one is named but absent. Baseline and fitted results therefore
retain identical runtime-dependency evidence.

The initial fitted coordinates are deliberately narrow, dimensionless
archetype corrections:

- `thrust_scale` multiplies the resolved thrust-to-weight result;
- `drag_scale` multiplies every ordinate in the resolved Mach-drag schedule;
- `maneuverability_scale` multiplies the normal-force coefficient driving
  dynamic-pressure-dependent aerodynamic authority, not the structural
  maneuver envelope;
- `guidance_time_constant_scale` multiplies the resolved guidance response
  time.

These fields default visibly to `1.0`. Resolution propagates a fitted scale's
`calibrated` origin into the affected runtime parameter, such as `thrust_n` or
`drag_coefficient`. A fit never edits geometry, mass, reported performance,
propulsion architecture, or another sourced field. The application boundary
also rejects any scale carrying an observed, reported, derived, or inferred
origin.

`InterceptorFitReceipt` retains:

- the exact base-profile and campaign fingerprints;
- selected backend and unmodified optimizer terminal status;
- initial/final objective and maximum normalized error;
- campaign acceptance threshold and disposition;
- variable bounds and initial/fitted values;
- complete baseline and fitted `TrajectoryEvaluation` results;
- fingerprints for the fitted evidence profile and resolved profile.

Convergence and acceptance are separate by design. A fixed-step objective may
terminate as numerically stalled near a useful minimum. That status remains in
the receipt. Application succeeds by default only if the maximum normalized
error satisfies the campaign threshold; inspecting an unaccepted candidate
requires an explicit override.

`apply_interceptor_fit_receipt(...)` requires the original profile fingerprint,
reconstructs only the fitted scale fields as `calibrated`, and verifies both
resulting fingerprints before returning. The input profile is immutable and
unchanged. The fitted profile can then be passed through the ordinary resolver
and Composition provider with no specialized runtime path.

YAML campaigns and JSON/YAML receipts use
`load_interceptor_fit_campaign(...)`, `write_interceptor_fit_receipt(...)`, and
`load_interceptor_fit_receipt(...)`. The copy-ready campaign is
`examples/parametric_interceptors/generic_medium_sam_fit_campaign.yaml`.

## Prototype resolver witnesses

The package includes three deliberately different evidence seeds:

- AIM-9X Block II is the compact, populated, high-agility witness. Publicly
  unavailable speed and range remain classified gaps.
- AIM-120 C5/C7 is the variant-sensitive medium-range witness. The selected
  mass retains its original imperial source value, while the unresolved family
  mass range remains a separate interval.
- PAC-3 MSE is deliberately sparse. Its dual-pulse propulsion, hit-to-kill
  mechanism, and enlarged-fin features remain observed records; its geometry,
  mass, pulse timing, aerodynamics, and control authority remain explicit gaps
  or archetype assumptions.

Only the first two are registered as built-in runnable prototype models. PAC-3
MSE is a resolver/provenance witness until its source-dominant physical profile
is better populated. An application may still explicitly construct a provider
from it, but the claim boundary remains archetype-dominant.

## Extension rules

New profiles normally require no provider code. An application developer uses
`taoryx-interceptor init` plus
`ParametricInterceptorMissionCompositionProvider.from_yaml(...)` for a flat
profile, or `from_catalogue_yaml(...)` for a provenance-bearing ontology
record. Both paths construct an isolated provider and do not modify global
plug-in discovery. The inspection report is the preflight contract: it lists
unresolved diagnostics, selectable fidelity tiers, controls, outputs, and
response-analysis availability before the model is composed.

A maintained built-in is a different route. Add its named source/evidence
profile in `witnesses.py`, include it in `runnable_prototype_profiles()` only
when it is executable, and prove its discovery, advertisement, fidelity, and
mission selection with a focused vertical test. A sparse record may remain a
resolver/provenance witness without appearing as an executable built-in. This
distinction lets developers iterate locally without turning every research
seed into an ambient, host-wide model.

Add a new archetype only when the existing versioned class maps cannot express
the family. Archetypes must have stable IDs and must mark every supplied value
as an assumption. Never silently replace an observed value.

Add future fidelity tiers as separate realizations, not as reinterpretations of
the point-mass or attitude-response tiers. Reuse the resolved physical profile,
publish new response parameters and achieved feedback, and retain the same
waypoint control family where the lowering is supported.

Calibration should consume scenario-qualified targets such as launch state,
target motion, range definition, atmosphere, and termination criteria. Reported
maximum range or speed is not a direct force-model parameter and must remain a
calibration target with provenance.

## Response-law analysis boundary

The pseudo-6DOF realization uses the same symmetric second-order response law
and rate/acceleration bounds on roll, pitch, and yaw, but not the same angle
topology. Roll uses the resolved bank bound, pitch has the runtime's ±89-degree
coordinate guard, and yaw is periodic. The direct plug-in therefore provides
`analyze_pseudo6_response(...)` and `compare_pseudo6_responses(...)` without a
CADAC dependency. Each request names an axis, and the analysis reports both:

- continuous unsaturated poles of the declared response law; and
- discrete poles of the semi-implicit update implemented by the runtime at the
  requested time step.

This distinction matters because a stable continuous response can be paired
with an unstable numerical update when the selected step is too large. The
provider advertises its default result and performs that discrete check during
pseudo-6DOF configuration validation. Point-mass validation does not inherit an
attitude-response restriction.

The same analysis estimates settling and overshoot and evaluates the requested
step against angular-acceleration, body-rate, and selected-axis angle limits. Those
are local surrogate response metrics. They do not qualify a physical autopilot,
airframe, actuator system, nonlinear envelope, schedule, or robustness. A
comparison intentionally gives separate results for response speed, overshoot,
and headroom and never declares an overall controller winner.

Each request supplies a frozen `command_support_fraction` in `[0, 1]`. The
analysis uses the actual local runtime equation
`angle_ddot = f * (wn^2 * error - 2*zeta*wn*angle_dot)`, so its effective
natural frequency is `sqrt(f)*wn` and its effective damping ratio is
`sqrt(f)*zeta`. Continuous and semi-implicit discrete poles, settling,
overshoot, and rate demand all follow those effective coefficients. The report
retains the pre-support acceleration demand because runtime acceleration
clipping occurs before the support multiplier. At zero support it reports two
continuous poles at zero, two discrete poles at one, marginal/unresponsive
status, and no settling or finite sample-time-limit claim.

A second typed route removes the need to hand-select that fraction.
`Pseudo6ResponseOperatingPoint` contains dynamic pressure, current mass,
current thrust, and commanded lateral acceleration, with an ID, fingerprint,
and source basis. `from_sample(...)` captures those values from a pseudo-6DOF
runtime sample without importing unrelated state. The operating-point resolver
uses the same shared aerodynamic/TVC authority evaluator and resolved
configuration, geometry, coefficient, vector-angle, and structural-limit
parameters as runtime. Its result retains each component, source parameter
trace, force-availability status, ordinary command-support fraction, and the
runtime-equivalent response-support fraction.

The distinction matters at zero command. `commanded_support_fraction(0)` is one
because no force is requested, but pseudo-6DOF response dynamics are enabled at
zero error only when some force authority is currently available. The
operating-point bridge reproduces that convention explicitly before calling
the existing pole analysis. It is a reproducible local snapshot, not an
environment sweep, schedule, or controller qualification.

`compare_pseudo6_responses_at_operating_point(...)` uses one identical
operating point and analysis request for a baseline and candidate, then resolves
each profile's authority support independently. It publishes candidate-minus-
baseline aerodynamic, thrust-vector, available-authority, and response-support
deltas separately from settling, overshoot, discrete-pole, and headroom
deltas. This avoids flattening different vehicle authority into a guessed shared
fraction while also avoiding an overall controller winner or physical
qualification. The file-first equivalent is
`taoryx-interceptor compare-operating-response BASELINE CANDIDATE CASE`.

The standalone bounded-step witness accepts the same frozen fraction and
applies it after the same pre-support acceleration bounds. Mission propagation
still recomputes the fraction from dynamic-pressure- and thrust-dependent
authority at every accepted boundary. Frozen analysis is therefore a local
operating approximation, not proof of achievable response throughout a
trajectory, a gain schedule, or a physical controller qualification. Provider
configuration preflight deliberately uses full support as the fastest nominal
response when checking the requested integration step.

The analytic rate check uses the exact continuous second-order step peak, not
the looser `command × bandwidth` characteristic scale. Axis command limiting is
applied before acceleration and rate demand are computed, while both requested
and effective commands remain in the report. `run_pseudo6_attitude_step(...)`
then supplies the nonlinear witness: it reuses the mission runtime's bounded
acceleration, body-rate, semi-implicit integration, and angle handling and
reports which limit actually activated. The witness deliberately excludes all
translation, propulsion, changing environment/authority, guidance, and sensor
behavior.

The developer route is a small Python model or one YAML file. See
`examples/parametric_interceptors/generic_medium_sam_response_analysis.yaml`
for a manually frozen fraction and
`generic_medium_sam_operating_point_response.yaml` for a shared-authority
operating point. The latter runs through
`taoryx-interceptor analyze-operating-response PROFILE CASE`.

The file-first inspection report projects this same capability as
`composition.controller_analysis`, including the response contract, supported
axes, provider operations, and copy-ready analysis commands. This gives a UI
or automation agent the same discoverable route without recasting a local
surrogate metric as physical-controller qualification.

## Focused verification

During development, run only this plug-in's vertical gate:

```bash
python -m pytest -q tests/families/parametric_interceptors
ruff check packages/taoryx-parametric-interceptors/src tests/families/parametric_interceptors
pyright packages/taoryx-parametric-interceptors/src/taoryx_parametric_interceptors
```

That gate covers sparse authoring, provenance retention, assumption cases,
entry-point isolation, complete advertisement, configuration validation, both
common-runner fidelity paths, shared single-/dual-pulse propulsion, and standard
registered translation-acceleration/ideal-IMU projection. It also checks
control-configuration resolution, feature lineage, shared point/pseudo force
authority, burnout loss of thrust-vector authority, structural versus
instantaneous Composition metadata, optional applicability ingestion and
parity, pseudo-6DOF vacuum/aerodynamic/TVC response coupling, ballistic-rate
continuity, target-track no-truth-fallback behavior, forward/right/down
air-relative projection, angle-of-attack/sideslip sign and zero-speed validity,
commanded/achieved local-vector magnitude parity, force-authority loss,
pseudo-6DOF direction lag, normalized command achievement, direction-error
validity, and live-lowering parity,
fail-closed parameter-usage classification, active dependency tracing,
overridden-selector detection, authoring/Composition manifest parity,
achieved-load maneuver drag and straight-flight invariance, and unit
conversion. Vertical tests
additionally cover suite truth/payload compatibility, suite-version binding,
causal delayed delivery, pending-packet checkpoint replay, and both tiers'
batch/session parity, partial held-waypoint retargeting, standard control
feedback, direct-acceleration partial holds and force-limited readback,
deterministic checkpoint replay, fidelity-scoped sensor continuity,
and session integration-step stability preflight. They also cover scenario
fingerprinting, classified-target rejection, both-fidelity calibration
observables, and assumption-case sensitivity ranking. Fit tests recover a
hidden scale from multiple scenarios,
verify bounds and protected evidence, replay a serialized receipt, and prove
calibrated-origin propagation. Response tests cover analytic poles, exact
implemented-discrete stability, saturation headroom, like-for-like comparison,
axis-specific angle topology, frozen-authority pole scaling, zero-authority
marginality, bounded step-trace agreement, YAML loading, and pseudo-6DOF
time-step preflight. Unit-ingestion tests cover
mixed public-source units, retained originals, converted intervals,
Composition projection, and incompatible-dimension rejection. The gate
also verifies the strict catalogue contract, duplicate and contradictory field
rejection, source-profile version propagation, and CLI JSON Schema discovery.
It deliberately does not revalidate CADAC, DAVE-ML, or unrelated vehicle
families.
