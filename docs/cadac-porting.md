# CADAC source-backed family integration

The CADAC work is organized as a provider family and evidence source inside Taoryx, not as a line-for-line Python translation of the CADAC C++ class hierarchy.

## Data flow

```text
input.asc ──> typed source AST ───────┐
                                      ├─> fingerprinted source bundle
*_deck.asc ─> immutable CADAC tables ┘          │
                                                ├─> family lowerer
                                                │      │
                                                │      ├─> cadac_compat execution
                                                │      └─> later taoryx_native execution
                                                │
                                                └─> source/output parity evidence

package inspection ─> actor/phase fidelity manifest ─> Taoryx capability claims
```

## Source compatibility semantics

`cadac_compat` currently preserves the semantics that affect AIM5 numerical behavior:

- fixed integration step;
- vehicles executed in source order;
- modules executed in source declaration order for each vehicle;
- one sequential event watchpoint checked before each vehicle module pass;
- communication-bus packet refresh after that vehicle's module pass;
- therefore a missile earlier in the source vehicle list observes the previous target packet until the target receives its own pass;
- stored-derivative trapezoid integration;
- linear lower-end table extrapolation;
- constant/clamped upper-end extrapolation.

These are compatibility rules, not recommendations for the eventual native runtime.

## Host integration contract

Every catalog model also publishes a CADAC-specific integration record through
`provider.get_model_integration_contract(model_id)`. It is deliberately stricter
than presence of a batch executor. The record answers three separate questions:

- Is a persistent Mission Composition session actually registered, including
  hidden integrator, event, actuator, seeker, and controller state?
- Does the runtime consume the Taoryx-owned atmosphere and gravity providers,
  or is it executing the source environment strictly for compatibility?
- Which controller analyses are supported by explicit command/response outputs,
  and which still require a family adapter and local linearization?

Most reconstructed runtimes are correctly reported as `batch_only`. They are
not advertised as `step`: restarting a batch for each call would reset hidden
state, and replaying precomputed samples would not accept held actions. A model
is promoted only when its persistent native state implements the standard
session lifecycle and its step controls have native bindings.

The installed AIM5 engagement, ADS6 SRBM, AGM6, and SRAAM6 providers are the
current exceptions. Each advertises `step` alongside `batch`, owns its complete
source integrator and controller state across calls, and accepts only durations
that are integral multiples of the source `int_step`. Their guidance and
controllers remain source-owned, so their sessions advertise no caller action
channels; they instead publish explicit command/response and sensor observation
ports at each committed boundary. The remaining CADAC models stay `batch_only`
until they meet the same state-ownership condition.

The current environment profile is `cadac_compat`. Its source US76/NASA,
inverse-square, and WGS84 paths remain provider-owned only to preserve source
execution and parity evidence. They are not exposed as a competing host
environment service. A future `taoryx_native` session must inject the shared
Taoryx atmosphere and gravity providers; that profile is not available while
the compatibility environment still executes internally.

## Native sensor harmonization

CADAC seekers and RADAR0 paths are layered on Taoryx's native
`relative-state-track` sensor rather than maintaining a parallel, untyped LOS
calculation. The typed measurement publishes sensor-frame range, azimuth,
elevation, relative position and velocity, closing speed, unit LOS, and LOS
rate. It is direct committed geometry only: it does not claim propagation,
target signature, image formation, gimbal dynamics, tracking, or fire
control.

```text
CADAC committed local-NED truth
          │
          ├── local orthonormal adapter ──> Taoryx relative-state-track
          │                                      │
          │                                      └── typed raw observation
          │
          └── CADAC source gimbal / acquisition / lock / filter / scheduler
                                                 │
                                                 └── standard output telemetry
```

The adapter is deliberately geometry-only. It embeds a CADAC local NED vector
in the sensor API's vector contract; the API's historical `*_eci` field names
do not turn that local source state into ECI, nor do they import CADAC gravity
or atmosphere into Taoryx. A source DCM that has accumulated numerical drift
is projected to its nearest proper rotation before entering the physical sensor
interface.

Current raw-native integrations are AIM5, ADS6 SRBM, SRAAM6, AGM6, and the
ADS6 engagement package. CADAC-specific gimbal, acquisition, lock, filtering,
source RADAR0 noise sequencing, track management, and launch scheduling remain
explicit source-compatibility state above the native observation. They are not
hidden inside the standard tracker.

`provider.get_model_sensor_integration_contract(model_id)` exposes this
boundary to composition tooling. It identifies the native provider, the source
state retained above it, and whether a persistent `SensorBus` binding exists.
For installed AIM5, ADS6 SRBM, ADS6 engagement, AGM6, and SRAAM6 source cases,
the contract reports
`sensor_bus_status="available"`: their sessions publish the native raw
relative-state packet at every committed source boundary, including every
native substep inside a longer caller hold, through the standard bus with
normal packet sequencing and delivery semantics. These packets are additional
external observations. They do not replace AIM5's intentional
previous-target-pass lag, ADS6 SRBM's source seeker enable and phase gates,
AGM6's source IIR, AIRCRAFT3 tracking, and datalink scheduling, SRAAM6's
source seeker acquisition/lock/filter state and previous-target-pass lag, or
the ADS6 package's RF/IR and RADAR0 controller/scheduling state. Other CADAC
models with no participating source sensor correctly report
`sensor_bus_status="not_applicable"`. The standalone ADS6 SAM is the one
sensor-bearing actor that reports `"blocked"`: its source controller needs
the package's target, RADAR0, launch-latch, and vehicle-major scheduling
context, so only `cadac.ads6.engagement` can own that persistent SensorBus.

### Sensor/session readiness matrix

The following matrix is the current installed-runtime boundary. `cadac_compat`
means the source atmosphere and gravity equations remain internal to the
compatibility runtime for source behavior; they are not advertised as a second
Taoryx host environment service.

| Exact model | Standard lifecycle | Native sensor boundary | Controller evidence | Environment profile |
| --- | --- | --- | --- | --- |
| `cadac.aim5.missile` | persistent `step`; source-step holds | available `relative-state-track` | source-owned; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.ads6.srbm` | persistent `step`; source-step holds | available `relative-state-track` | source-owned; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.sraam6.missile` | persistent `step`; source-step holds | available `relative-state-track` | source-owned; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.agm6.missile` | persistent `step`; source-step holds | available `relative-state-track` | source-owned; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.ads6.engagement` | persistent `step`; source-step holds | one available `relative-state-track` per exact SAM/target pair | source-owned; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.ads6.sam` | batch-only direct-command plant | blocked; package-owned context required | direct batch controls; finite-run analysis available where requested/realized channels are selected; stability blocked | `cadac_compat` |
| `cadac.ads6.aircraft` | batch-only source-program point-mass plant | not applicable; optional threat is a configuration seam, not a native sensor | source-owned commanded-bank/load versus achieved response; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| `cadac.falcon6.aircraft` | batch-only direct-surface 6-DoF plant | not applicable | caller-owned aileron/elevator/rudder with requested/achieved and limit feedback; finite-run analysis and comparison available; stability blocked | `cadac_compat` |
| All other current CADAC actors | exact advertised batch or validation boundary | not applicable unless a participating source sensor and persistent owner are added | only their advertised command/response surface; no inferred stability claim | `cadac_compat` when executable |

### AIM5 persistent-session API

The session uses the ordinary Mission Composition request/descriptor/step
types. Its `integration_step_s` must exactly equal the source case's
`int_step`; that restriction preserves the source controller and communication
ordering instead of silently inventing fractional source passes.

```python
from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)

prepared = provider.validate_configuration(configuration)
session = provider.open_session(
    MissionCompositionOpenSessionRequest(
        session_id="aim5-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        integration_step_s=0.01,  # exact source int_step
    )
)
step = provider.step_session(
    MissionCompositionSessionStepRequest(
        session_id=session.session_id,
        duration_s=0.10,  # an integral number of source timesteps
    )
)
track = step.observation.values["native_relative_state_track"]
packet_history = provider.session_sensor_packets(session.session_id)
```

The descriptor has no action schema because the AIM5 source controller is
internal. Its observation schema labels standard local-NED state, source-owned
normal/lateral commands, source seeker state, and the delivered native sensor
packet. The batch output schema additionally publishes target-relative NED
position, body-frame unit LOS and LOS rate, and realized normal/lateral
acceleration. These do not establish closed-loop stability or compiled-CADAC
parity; they make the control and sensor evidence inspectable.

The focused regression suite includes a 250-step persistent-session smoke
budget (`tests/families/cadac/test_aim5_performance.py`). It guards against a
future change that accidentally replays a whole batch or rebuilds heavyweight
sensor state on every interactive source step. It is a responsiveness guard on
the CI host, not a vehicle-throughput or real-time performance claim.

### ADS6 SRBM persistent-session API

`cadac.ads6.srbm` uses the same standard session request/descriptor/step
surface as AIM5. It retains the ROCKET5 source translation, propulsion,
alpha/beta response-law, phase, terminal, and fixed-target state rather than
turning a fresh batch run into a pretend interactive loop. The session accepts
only integral `int_step` holds and publishes a standard
`relative-state-track` packet named `ads6-srbm-native-relative-state` at each
committed source step.

Its observation and batch schemas label the source-managed normal/lateral
commands separately from realized normal/lateral specific force. This makes
finite-run control traces and like-for-like controller comparisons available;
it does not provide a trim state, linear closed-loop model, or stability
margins. The native track remains raw fixed-target geometry, while source
seeker enable and endo/exo state remain visible CADAC behavior. The analogous
250-step responsiveness guard is
`tests/families/cadac/test_ads6_srbm_performance.py`.

### SRAAM6 persistent-session API

`cadac.sraam6.missile` now exposes the standard persistent Mission
Composition lifecycle for the executable four-fin phase. It retains the full
MISSILE6 rigid-body, actuator, controller, dynamic seeker, TARGET3, event,
and source-packet state across calls. The session accepts exact `int_step`
multiples only and emits `sraam6-native-relative-state` at every committed
source substep.

The session publishes missile and target truth, source-managed
normal/lateral command and realized acceleration, source seeker state, and the
native raw track as distinct labeled ports. Its source controller accepts no
caller action channels; finite-run source-control analysis remains available
from the published command and response evidence, while local stability and
frequency claims remain blocked without a declared trim/linearization model.
The 200-step responsiveness guard is
`tests/families/cadac/test_sraam6_performance.py`.

### AGM6 persistent-session API

`cadac.agm6.missile` supports the same persistent lifecycle for its complete
three-actor source engagement. It retains MISSILE6 rigid-body, actuator,
controller, IIR, TARGET3, AIRCRAFT3 tracker, datalink, event, stochastic, and
source-packet state across calls. Holds must be integral multiples of its
`0.001 s` source `int_step`, and the session emits
`agm6-native-relative-state` at every committed substep.

The raw native packet reports actual committed MISSILE6-to-TARGET3 geometry.
It neither feeds nor replaces the source IIR, the independently propagated
AIRCRAFT3 target-track producer, or the source datalink lag. Session outputs
label requested versus achieved roll/pitch/yaw controls and four fin angles,
alongside normal/lateral command and realized acceleration. Consequently,
finite-run source-controller comparison is available without claiming trim,
local stability, or frequency margins. The 250-step responsiveness guard is
`tests/families/cadac/test_agm6_performance.py`.

### ADS6 package sensor-session boundary

The ADS6 standalone SAM physical plant is already stepper-capable, but its
source RF/IR controller requires the live target, RADAR0 intercept point,
launch latch, and source packet epochs owned by `cadac.ads6.engagement`.
The standalone direct-command plant is therefore not a valid host for a
source-controller or SensorBus session. `cadac.ads6.engagement` is the package
owner now promoted for that work: it retains the full vehicle-major scheduler
and RADAR0 state across the standard lifecycle, advertises `step`, and publishes
one native raw relative-state SensorBus stream per exact SAM/target pair. The
streams expose committed geometry only; source RF/IR and RADAR0 behavior stays
inside the package. The standalone SAM's precise blockers remain explicit.

## Controls, outputs, and controller analysis

Every externally available CADAC control has a semantic quantity, canonical
unit when dimensional, exact native configuration binding, and an
`output_evidence` link to requested and realized standard-output channels.
Vector component indices preserve scalar control identity without duplicating
telemetry. Direct RCS realizations publish their attitude, incidence,
acceleration, and thrust-direction requests alongside resulting response or
wrench channels.

Source-owned controllers remain provider-internal controls; their commands and
responses are telemetry, not caller action channels. The analysis helpers can
still evaluate or compare them:

```python
from taoryx.families.cadac import (
    analyze_closed_loop_state_matrix,
    analyze_controller_trace,
    cadac_controller_trace_from_samples,
    compare_controller_traces,
)

source_trace = cadac_controller_trace_from_samples(
    result.objects[0].samples,
    controller_id="ads6-source-controller",
    model_id="cadac.ads6.engagement",
    reference_channel_id="normal_command_g",
    response_channel_id="achieved_normal_acceleration_g",
    unit="g",
)
source_report = analyze_controller_trace(source_trace)
comparison = compare_controller_traces(source_trace, candidate_trace)

local_stability = analyze_closed_loop_state_matrix(
    controller_id="source-controller",
    model_id="cadac.ads6.sam",
    operating_point_id="mach-2-alt-10km",
    state_names=("alpha", "q", "actuator_position", "actuator_rate"),
    closed_loop_state_matrix=closed_loop_a,
)
```

`cadac_controller_trace_from_samples` accepts typed standard trajectory samples
or their JSON-shaped `{time_s, values}` representation. It requires explicitly
selected finite scalar channels and rejects vectors rather than silently
choosing or reducing a component. Trace comparison requires identical model,
channel, units, time grid, and reference. It reports tracking, settling,
overshoot, control realization, and saturation metrics but makes no formal
stability claim. Matrix analysis makes only a local linear pole claim at the
named operating point. Frequency margins, gain-schedule coverage, nonlinear
stability, uncertainty robustness, and envelope qualification remain blocked
until the full plant/controller/actuator state and timing contract is published
through a CADAC `StandardFamilyAdapter`.

## Explicit local source binding

The package entry point intentionally discovers the CADAC catalog without
searching the host for a source checkout. A caller that owns a standard
upstream checkout can bind every reconstructed vehicle explicitly:

```python
from taoryx.families.cadac import CadacSourceCaseBindings

provider = CadacSourceCaseBindings.from_standard_checkout(
    "/path/to/missiondesignsolutions-CADAC"
).build_provider()
```

`CadacSourceCaseBindings` also accepts individual case paths, so an
integration can install only the source-backed vehicles it is authorized to
use. The provider retains the rest of the catalog as validation/discovery
metadata, without scanning the environment or substituting a neighboring
vehicle implementation. Register its exact installed runtimes with
`provider.register_runnable_models(registry)`.

The standard ADS6 binding deliberately uses the RF aircraft engagement case
both as the package composition and as the direct SAM source. The latter names
`MISSILE6` actor index `0` explicitly; it is a direct vehicle-plant boundary,
not a hidden extraction or replay of the aircraft/radar package participants.

## AIM5 executable slice

The first executable source definition lowers a single `AIM5` plus `AIRCRAFT3` engagement.

### AIM5 fidelity

- Missile: `pseudo_6dof` / `response_law`.
- Target: `point_mass_3dof` / `force_model`.

The missile carries translational state plus reduced-order alpha/beta and rate-loop dynamics. It does not integrate the complete rigid-body attitude/body-rate/moment closure required for Taoryx's two rigid-body tiers.

### Participating source modules

```text
environment
kinematics
aerodynamics
propulsion
seeker
guidance
control
forces
newton
intercept
```

The lowerer rejects missing, unknown, or reordered vehicle assumptions rather than silently degrading the source schedule.

### Executable physics and logic

- flat-Earth NED translational state;
- source US-1976 atmosphere implementation;
- inverse-square gravity used by the AIM5 source;
- source aerodynamic lift/drag deck lookup and alpha/beta conversion;
- mass and sea-level thrust histories with nozzle pressure correction;
- LOS kinematic seeker;
- proportional-navigation acceleration command;
- optional spiral command path when source parameters are present;
- reduced-order pitch/yaw PI + lag response;
- aerodynamic/propulsive specific-force closure;
- AIRCRAFT3 steady and commanded-turn paths;
- source closest-approach intercept gate.

### Parity instrumentation

`run_aim5_source_compatibility(..., trace_steps=N)` records state immediately after every source module for the first `N` execution epochs. This is intended to support the promotion sequence:

1. module-local checks;
2. one-step equivalence;
3. open-loop trajectory equivalence;
4. closed-loop engagement equivalence.

`parse_cadac_plot_file()` reads the legacy plot stream, and `compare_aim5_source_plot()` reruns the Python compatibility slice at source plot cadence and compares shared channel names.

No equivalence status is promoted until a compiled CADAC output file is compared.

## Multi-engagement AIM5

`aim5-scenario-run` lifts the same source-compatible actor kernels into a source-ordered multi-actor loop. It assigns stable `m1..mN` and `a1..aN` identities by source type order, retains the complete vehicle loop order separately, resolves missile decks by vehicle role/source binding, and validates each missile `tgt_num` against the target population.

Communication snapshots are refreshed only after the corresponding target actor pass. A missile therefore observes whichever target packet is present at its own source-order slot, including the previous epoch when the target appears later in the vehicle list. Per-missile samples and intercept results remain separate so simultaneous engagements do not collapse into one trajectory.

The strict single-engagement `aim5-run` remains useful for C++ parity because it has the smallest comparison surface; the scenario runner is the integration path toward Mission Composition.

## Source provenance

Each bundle records, for `input.asc` and each resolved deck:

- normalized resolved path;
- SHA-256;
- byte size.

Shared deck resources are parsed once, while vehicle-to-deck bindings retain the source vehicle line and role. Multiple vehicles of the same model may therefore reuse the same deck without collapsing their binding identities.

## Event model

The parser supports source-ordered blocks of the form:

```text
IF <watch-variable> <|=|> <criterion>
    <variable> <value>
    ...
ENDIF
```

The runtime cursor intentionally checks **only the next event**. When it fires, its mutations are applied and the cursor advances to the next source event. This matches the CADAC sequential event model and matters when two events use the same watch condition. Event bodies may also be empty: ROCKET6G uses a condition-only `thrust = 0` event to advance the cursor and reset `event_time` without changing a source variable.

AIM5 binds its executable source-variable registry to this cursor. Supported source inputs and participating runtime states may therefore be mutated before the actor module pass, and every fired event is emitted as an `Aim5EventTrace`. ROCKET6G uses the same cursor to drive source stage/control transitions and emits a `Rocket6gPhaseEvent` containing the pre/post phase, stage, and runtime fidelity. Unsupported participating mutations fail closed rather than silently claiming behavior.

## Fidelity classification

Fidelity is declared per actor and phase, not per source directory. Runtime promotion is a separate property from source classification.

| Package | Actor / phase | Taoryx mapping | Runtime state |
|---|---|---|---|
| ADS6 | source package / SAM fins / TVC / aggregate RCS | mixed roots / T4 / T4 / T3 | runnable composition / runnable exact realizations |
| ADS6 | `ROCKET5` SRBM / `AIRCRAFT3` / radar | T2 / T1 / static | runnable / runnable / embedded package sensor |
| AGM6 | missile / carrier / moving ground target | T4 / T1 / T1 | runnable / embedded / embedded |
| AIM5 | missile / target | T2 / T1 | runnable / embedded |
| CRUISE5 | source / translation-only | T2 / T1 | runnable / validate-only |
| FALCON6 | `PLANE6` physical plant | T4 | runnable physical-plant boundary |
| GHAME3 | `CRUISE3` | T1 | runnable |
| GHAME6 | atmospheric surfaces / transfer-interceptor aggregate RCS / SAT3 / RADAR0 | T4 / T3 / T1 / static | runnable phase-aware / embedded / non-trajectory |
| MAGSIX | trajectory / restricted attitude | T1 / T2 | runnable / validate-only |
| ROCKET6G | aggregate RCS / TVC / mixed / coast | T3 / T4 / T4-mixed / uncontrolled rigid body | runnable phase-aware plant |
| SRAAM6 | fins / optional TVC / target | T4 / T4 / T1 | runnable / validate-only / embedded |

Two source corrections are important for the catalog:

- GHAME3 is a source **3-DoF** `CRUISE3` vehicle. Prescribed alpha/bank and Q-hold propulsion do not promote it to pseudo-6DoF.
- MAGSIX's source actor is `ROTOR`; the package's restricted attitude equations remain one-way coupled and separate from its independently runnable planar trajectory.

Taoryx currently names its highest canonical tier `rigid_body_6dof_surface_allocated`. CADAC-specific metadata separately identifies aerodynamic surfaces, TVC gimbals, aggregate RCS, and mixed effectors so that the tier string does not erase the physical realization.

## Actor plug-in architecture

The important plug-in unit is the **source actor**, not the source directory. Mixed packages such as ADS6, GHAME6, and ROCKET6G therefore remain compositions of independently classified actor/phase realizations.

`CadacVehiclePluginDescriptor` is immutable discovery metadata. `CadacPluginRegistry` separately records installed runtime implementations. A model can therefore be discoverable and validatable without being executable. Exact runtime lookup never substitutes a neighboring actor, lower tier, or source replay.

Current inventory:

- 19 source actor descriptors;
- 17 trajectory-capable actor-level Mission Composition models;
- 2 static/sensor actor descriptors outside standalone trajectory registration;
- 12 exact batch-runtime primary actor plug-ins;
- 5 embedded actor models (AIM5 target, SRAAM6 target, AGM6 target, AGM6 tracking aircraft, and GHAME6 satellite);
- 1 additional exact ADS6 package-composition model;
- 18 provider models when an ADS6 package source case is installed;
- 0 remaining planned dynamic actors; all 17 trajectory-capable actor models are runnable or embedded.

### Exact runnable common-runner models

```text
cadac.ads6.engagement             mixed-fidelity source-ordered Mission Composition
cadac.ads6.aircraft               point_mass_3dof
cadac.ads6.sam                    T4 fins/TVC or T3 aggregate RCS, selected exactly
cadac.ads6.srbm                   pseudo_6dof
cadac.agm6.missile                rigid_body_6dof_surface_allocated
cadac.aim5.missile                pseudo_6dof
cadac.cruise5.cruise_vehicle     pseudo_6dof
cadac.falcon6.aircraft             rigid_body_6dof_surface_allocated
cadac.ghame3.hypersonic_vehicle   point_mass_3dof
cadac.ghame6.hypersonic_vehicle    T4 run envelope; phase-reported T4→T3
cadac.magsix.vehicle              point_mass_3dof
cadac.rocket6g.launch_vehicle      T4 run envelope; phase-reported T3/T4
cadac.sraam6.missile              rigid_body_6dof_surface_allocated
```

All use provider ID `cadac` and exact `(provider_id, model_id)` executor registration.

AIM5, SRAAM6, and AGM6 return source actors as independent root objects rather than children of the missile:

```text
m1  cadac.aim5.missile    pseudo_6dof
a1  cadac.aim5.target     point_mass_3dof

m1  cadac.sraam6.missile  rigid_body_6dof_surface_allocated
t1  cadac.sraam6.target   point_mass_3dof

m1  cadac.agm6.missile         rigid_body_6dof_surface_allocated
t1  cadac.agm6.ground_target   point_mass_3dof
a1  cadac.agm6.aircraft        point_mass_3dof

h1  cadac.ghame6.hypersonic_vehicle  T4 run envelope; sample-level T4/T3
s1  cadac.ghame6.satellite           point_mass_3dof
r1  cadac.ghame6.ground_site         static
```

The target exists at scenario initialization, so no release/deployment lineage is invented.

### ADS6 package composition and SAM multi-realization plant

`cadac.ads6.sam`, `cadac.ads6.srbm`, and `cadac.ads6.aircraft` remain independently promoted actor models. `cadac.ads6.engagement` is an additional exact package model that installs one source case containing one to three SAM/target pairs plus one RADAR0. It does not turn RADAR0 into a standalone trajectory executor and does not replace the actor models.

The package runtime preserves persistent actor state, exact `VEHICLES` order, immediate per-actor packet publication, positional `m1->a1/r1` pairing, and radar-to-missile next-epoch latency. Aircraft mode uses lethal-range launch latches and measured-target IP uplinks. SRBM mode uses apogee detection plus the source SAM/SRBM trajectory decks for launch prediction and IP refinement. All participants are independent roots and retain actor-specific fidelity.

The runtime follows the documented RADAR0 latched launch schedule rather than reproducing the apparent unconditional launch-delay overwrite in the shipped executive. The default package path now interleaves truth-aligned INS, deterministic RF/IR seeker state, radar-IP line guidance, terminal proportional navigation, and adaptive rate/acceleration control with the physical SAM plant. RF glint/thermal-noise, complete IR focal-plane/aimpoint behavior, exact stochastic sequences, final compiled-CADAC intercept parity, and bug-for-bug executive behavior remain outside the package claim.

The package has a persistent, source-owned session with no caller action
channels. It retains the SAM/target/RADAR0/controller/latch/packet state across
integral source-step holds, and resets to its original deterministic state.
Its standard batch and session outputs label requested/achieved controls and
fins, source normal/lateral commands, and achieved lateral/normal acceleration.
Native `relative-state-track` packets are additional Taoryx observations, not
substitutes for the package's RF/IR seeker or RADAR0 track manager. Finite-run
controller comparison is available from those labeled command/response
channels; stability and frequency margins are not claimed without a trim and
linearization interface.

The installed standalone SAM provider exposes three exact realizations:

- physical cross-fin control at T4;
- physical pitch/yaw TVC at T4;
- axis-aggregate RCS force/moment at T3.

The source case must include `actuator`, `tvc`, and `rcs` before the provider advertises the combined model. Fin and TVC paths preserve independent second-order physical-effector states and source-order limits. Achieved effectors enter the aerodynamic or thrust force-and-moment closure. The RCS path preserves proportional/Schmitt control, hysteresis, relay history, side-force parasitic moments, and RCS fuel accounting, but remains T3 because no individual-jet allocation is resolved.

The standalone SAM provider still accepts commands at the source controller-output seam so the physical plant can be studied independently. The package provider owns the separately evidenced source-controller layer and interleaves it with that same plant; standalone plant availability does not imply package GNC parity.

### CRUISE5 middle-rung evidence

CRUISE5 is the first port for which the upstream repository ships a legacy `plot1.asc` reference. The regression suite checks the Python source-compatible runner at 0.0, 0.5, and 1.0 seconds across shared source channels, including force, dynamic pressure, Mach, altitude/speed, propulsion, L/D, reduced incidence/bank response, guidance commands, and waypoint range.

That evidence is intentionally narrower than a general equivalence claim. It establishes a concrete golden-trace anchor for the reusable Round3 and pseudo-6DoF substrate.

### FALCON6 highest-tier boundary

The FALCON6 executable plug-in covers the participating physical plant:

- source `PLANE6` rigid-body state;
- quaternion attitude and body rates;
- translational/environment loop;
- turbojet spool/thrust;
- second-order aileron/elevator/rudder actuator states and limits;
- achieved physical surfaces feeding the aerodynamic force/moment closure;
- source-ordered one-step actuator/effect latency.

The full CADAC waypoint guidance/autopilot program is **not** included in this runtime claim. That mission layer can be added without changing the plant's T4 classification.

### GHAME3 T1 correction

GHAME3 executes through the reusable Round3 substrate as `point_mass_3dof`. Its alpha and bank values are prescribed source data/event values rather than integrated response states. The plug-in therefore fails closed rather than advertising pseudo-6DoF simply because those angles appear in the force model.

### GHAME6 atmospheric-to-exo T4→T3 composition

GHAME6 is the second runnable phase-changing CADAC `HYPER6`, but its control realization is materially different from ROCKET6G. Source inspection corrected an earlier tentative assumption: GHAME6 has no `tvc` module. Its source progression is physical atmospheric surfaces followed by aggregate RCS transfer/interceptor phases. An input that adds an invented TVC module fails closed during lowering.

The participating runtime preserves:

- the exact `HYPER6`, `SAT3`, `RADAR0` actor order and source module order;
- WGS84 rigid-body truth, Earth rotation, source gravity, and stored-derivative integration;
- atmospheric left/right elevon and rudder mixing with three independent second-order states;
- achieved-surface aerodynamic closure through the source 22-table deck;
- US76 and the source NASA-Marshall atmosphere extension through 1000 km;
- fixed/Q-hold hypersonic propulsion and transfer/interceptor rocket modes;
- fuel, mass, inertia, aerodynamic-mode, and integration-step changes through five sequential source events;
- proportional and Schmitt-trigger axis-aggregate RCS moment/side-force paths;
- independent SAT3 two-body truth and rotating-Earth RADAR0 track production;
- three root objects and source/radar-owned events without invented carrier lineage.

The model advertises a T4 run envelope because physical surfaces participate before release. `transfer_angle_rcs`, `transfer_vector_rcs`, `interceptor_glideslope_rcs`, and `interceptor_terminal_rcs` samples remain T3 `rigid_body_6dof_direct_wrench`. Full arc/LTG/glideslope command generation, GPS/INS/star-tracker and RF-EKF estimation, unsupported weather/wind/turbulence modes, exact stochastic parity, discarded-carrier propagation, and compiled-CADAC parity remain explicit blockers.

### MAGSIX split-tier plug-in

MAGSIX exposes one model with two explicit source phases:

- `trajectory_only` -> T1 `point_mass_3dof`, exact batch runtime;
- `restricted_attitude` -> T2 `pseudo_6dof`, validate-only.

The T1 runtime preserves the source Dynamic Normalized Time equations, changing DNT-to-seconds scale, Magnus spin state, local-level trajectory, US76 atmosphere, inverse-square gravity, and ground-impact termination. A T2 request cannot dispatch the T1 executor.

### ADS6 `ROCKET5` pseudo-6DoF SRBM

The ADS6 SRBM actor is executable under the exact model ID `cadac.ads6.srbm`. Its source actor token is `ROCKET5`; “SRBM5” remains the semantic role rather than the dispatch identity.

The runtime preserves:

- Flat3 local-NED translation and NASA-Marshall US76 atmosphere;
- source pressure-corrected rocket thrust and continuous mass depletion;
- lift/drag lookup in total incidence and Mach;
- pitch/yaw PI, response-rate, alpha, and beta states below the endo boundary;
- the sticky exo flag and repeated response-state reset above `alt_endo`;
- reentry response from the reset states;
- optional fixed-coordinate seeker, proportional navigation, and decaying spiral maneuver;
- ground-impact and closest-approach termination.

Only position and velocity are core state. Alpha, beta, and response-rate values remain telemetry because the source has no quaternion, full body angular velocity, physical effector, or body-moment closure. The source actor is therefore T2, not T3/T4. Source event/stochastic execution and compiled-CADAC golden parity remain explicit blockers.

### ADS6 `AIRCRAFT3` point-mass target

The ADS6 aircraft actor is executable under the exact model ID `cadac.ads6.aircraft`. It remains T1 `point_mass_3dof` despite carrying source bank and normal-load response states: those states delay and limit the specific-force model but do not close rotational attitude, body-rate, inertia, or moment equations.

The runtime preserves:

- Flat3 local-NED position and velocity truth;
- NASA-Marshall US76 atmosphere and inverse-square gravity;
- source-order `environment -> kinematics -> guidance -> control -> forces -> newton`;
- steady, horizontal g-turn, and first-SAM escape guidance behaviors;
- the strict open maneuver interval `man_start < time < man_stop`;
- first-order or ideal bank/load response according to the source time constants;
- source bank and dynamic-pressure/alpha load limits;
- achieved-bank force-axis rotation and normal-load specific-force closure;
- an explicit constant-velocity threat-track seam for standalone escape execution;
- ground-impact and finite-state termination.

Only position and velocity are core state. Bank angle, load factor, maneuver mode, environment, and optional threat range remain telemetry. Actor-local events and stochastic declarations fail closed in the standalone runtime. Full radar/SAM communication scheduling and compiled-CADAC parity remain package-level promotion gates.

### AGM6 three-actor source-closed T4 engagement

AGM6 extends the physical-fin missile substrate into a source-ordered `MISSILE6`, moving `TARGET3`, and tracking `AIRCRAFT3` composition. Its runtime preserves:

- the exact source actor order and module order;
- a physical four-fin missile with independently lagged and limited actuator states;
- achieved-fin aerodynamic force/moment closure and full flat-Earth rigid-body propagation;
- continuous rocket fuel and mass, pressure-corrected thrust, and fixed principal inertias;
- source weather-deck atmosphere, wind, and source-shaped turbulence options;
- moving target and tracking-aircraft point-mass dynamics;
- AIRCRAFT3 target-track production, source packet refresh, and missile datalink update/extrapolation;
- midcourse line/proportional-navigation guidance, terminal sensor guidance, and rate/acceleration control;
- IIR acquisition, lock, blind-range, pointing, and source-shaped LOS-filter state;
- target-plane intercept termination;
- three independent Mission Composition roots with no invented lineage.

The source can request a real INS, but the current participating navigation solution is truth-aligned. Complete focal-plane corruption, aimpoint modulation, gimbal-head optical geometry, exact C-rand sequence parity, and compiled-CADAC numerical equivalence remain explicit promotion blockers.

### SRAAM6 source-closed-loop T4 engagement

SRAAM6 reuses the flat-Earth rigid-body substrate but promotes a wider execution boundary than FALCON6. Its standard-fin runtime preserves:

- the source `MISSILE6` then `TARGET3` vehicle order and one-pass communication lag;
- the source module sequence from environment through intercept;
- the default `time > 0.25 s` transition from rate control to acceleration control and midcourse proportional navigation;
- the seeker acquisition/lock/blind-range state machine, source stored-derivative filter states, and pointing-angle integration;
- source rate and acceleration autopilot equations;
- fixed roll/pitch/yaw-to-four-fin mixing;
- four independent second-order actuator position/rate states with travel and rate limiting;
- time-deck thrust, mass, center of gravity, and principal inertia;
- achieved-fin aerodynamic force/moment closure, rigid-body propagation, and closest-approach termination;
- an independently propagated T1 `TARGET3` root object.

The model advertises two physical-effector realizations. `cadac-standard-four-fin` is registered for batch and persistent step execution. `cadac-optional-tvc` is configuration-validatable but blocked for execution. Exact dispatch rejects the TVC phase instead of substituting the fin runtime. Complete optical corruption/aimpoint/gimbal-head geometry and compiled-CADAC numerical equivalence remain outside the current claim; the current dynamic seeker uses true LOS-to-current-pointing displacement as its explicit reduced optical-error boundary.

### ROCKET6G phase-changing T3/T4 plant

ROCKET6G is the first runnable CADAC actor whose active realization changes during one source program. It keeps one `HYPER6` vehicle identity while returning the active source phase, stage, Taoryx fidelity, and control realization on every sample.

The participating plant includes:

- source-ordered pre-step events, including the condition-only burnout event;
- WGS84 geodetic/inertial initialization, transformations, and gravity;
- inertial translation, body attitude, and body-rate propagation;
- source stage-specific aerodynamic tables;
- three solid-motor stages with fuel, mass, center-of-gravity, and principal-inertia evolution;
- physical TVC with requested and achieved nozzle states, second-order dynamics, travel/rate limits, and thrust wrench;
- proportional and Schmitt-trigger aggregate RCS modes;
- source aerodynamic, propulsion, TVC, and RCS force/moment closure.

The source program is advertised through a T4 run envelope because physical TVC participates. `aggregate_rcs` samples remain `rigid_body_6dof_direct_wrench`; `physical_tvc` and `mixed_tvc_rcs` samples report the physical-effector tier. Mixed phases do not promote the RCS path beyond axis-aggregate force/moment realization.

The direct-command boundary excludes LTG/autopilot command generation and GPS/INS/star-tracker estimation. Weather/turbulence modes beyond source `mair=0` and compiled-CADAC numerical parity are also outside the current claim.

## Control and optimization publication

Mission Composition separates a source-owned controller from a caller-owned
plant command boundary. This prevents a generic composer from mistaking a
CADAC autopilot diagnostic for an external actuator command.

- `cadac.ads6.sam` publishes exact batch-only controls for each selected
  direct realization: cross-fin roll/pitch/yaw coordinates, physical-TVC
  pitch/yaw coordinates, or aggregate-RCS attitude/incidence/acceleration and
  thrust-vector coordinates.
- `cadac.falcon6.aircraft` publishes batch-only direct aileron, elevator, and
  rudder controls.
- `cadac.ghame6.hypersonic_vehicle` and `cadac.rocket6g.launch_vehicle`
  publish their reconstructed direct atmospheric-surface, TVC, and
  aggregate-RCS seams.

Every published control carries its semantic ID, primitive type and shape,
canonical/display unit, frame where applicable, value topology,
operation-specific sampling semantics, and an exact native binding back to its public
configuration path. A body-axis thrust-vector command is explicitly a
nonzero-vector input normalized at the provider boundary; it is not silently
advertised as three independent force components.

Source-managed controllers remain `internally_generated` authority rather than
caller action channels. AIM5, ADS6 SRBM, SRAAM6, AGM6, and the ADS6 package
expose that authority through persistent stepping and explicit
command/response observations. CRUISE5, GHAME3, MAGSIX, and ADS6 AIRCRAFT
remain batch-only. In every case provider-internal intent is visible to
composition clients but cannot be selected as an external interactive control
surface.

The action metadata is suitable for a batch optimizer or policy adapter to
discover the input coordinates and units. It does **not** assert an optimized
trajectory, a controller-tuning objective, or a finite qualified control
envelope. Such a claim requires a source-bound objective, explicit bounds
where the source supplies them, and numerical evidence against a compiled
CADAC result. External-action `step`/episode control remains blocked for every
CADAC model, even where a source-owned persistent session is available.

## Mission Composition boundary

A numerical source reconstruction is not labeled `source_replay`. Runnable CADAC actors participate in state propagation and are projected to Taoryx's normal result contracts. Scenario-owned source scheduling remains explicit where needed, including AIM5 vehicle-major packet semantics and the ADS6 source-order package scheduler.

The ADS6 package publishes mixed-fidelity independent roots rather than applying the SAM envelope to every participant. Packet traces expose the epochs consumed and published by each actor pass. The package result carries the diagnostic `launch_schedule_semantics=documented_radar_latched_next_epoch` so the intended RADAR0 contract is not confused with bug-for-bug executive replay.

Per-model configuration distinguishes source phases from dynamics fidelity. Lower or alternate phases that have not received independent execution lowering remain validate-only. This prevents, for example, CRUISE5 T1 or MAGSIX T2 requests from silently executing a different realization.

## Promotion gates

1. **Intake** — every participating source symbol is mapped or explicitly rejected and resources are fingerprinted.
2. **Table parity** — dimensions, axes, row-major layout, interpolation, lower extrapolation, and upper clamping agree.
3. **Primitive parity** — integration, frames, atmosphere/gravity, actuators, stochastic primitives, and event semantics agree.
4. **Module parity** — source and Python agree at participating module boundaries.
5. **One-step parity** — integrated and diagnostic channels agree after one source epoch.
6. **Open-loop parity** — prescribed-command trajectories agree.
7. **Closed-loop parity** — guidance/control/seeker/events/termination agree where those modules are in scope.
8. **Fidelity promotion** — the evidence supports the advertised tier and validity envelope.

Runtime registration may occur before gate 8 only with an explicit `development` evidence status and a narrow claim boundary, as done for the current source reconstructions.

## Remaining model trenches

- Close compiled-CADAC parity for AIM5 and multi-engagement AIM5 when executable output is available.
- Close module/event/trajectory parity for the phase-aware ROCKET6G plant, then add LTG/autopilot and navigation estimators as separately evidenced layers.
- Close module/event/trajectory parity for AGM6, then add full real-INS and optical-IIR evidence layers.
- Add MAGSIX `restricted_attitude` T2 on top of the now-stable T1 trajectory runtime.
- Close module/event/trajectory parity for GHAME6, then add full source GNC/estimation and discarded-carrier evidence as separate layers.
- Close one-step/module/closed-loop parity for the ADS6 source-controller path against compiled aircraft- and SRBM-defense outputs; then add RF glint/thermal noise, complete IR focal-plane behavior, and INS error-state propagation as independently evidenced layers.
- Keep `step`/episode execution blocked until state ownership and committed-boundary semantics are independently proven.

## Runtime onboarding recipe

For each remaining actor:

1. retain the existing actor/phase descriptor;
2. lower the source case and decks into an immutable typed definition;
3. reuse a neutral substrate (Flat3, Round3, rigid-body, physical-effector, aggregate-RCS) where the source equations actually align;
4. preserve source module/event/communication order required for parity;
5. publish truth and requested/achieved control telemetry appropriate to its tier;
6. register only the exact runnable phase/model;
7. keep adjacent phases validate-only until independently implemented;
8. attach golden-source evidence independently from runtime availability.
