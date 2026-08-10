# Taoryx sensor and measurement orchestration backlog

**Status:** Alpha 3 breadth candidate / architecture workstream  
**Scope:** committed truth, synthetic measurements, sensor timing, estimator
interfaces, truth isolation, and multi-rate closed-loop execution

Taoryx sits at the orchestration boundary between an equation-of-motion truth
producer and sensed-data consumers. It must not live inside the EOM or inside a
particular IMU, focal-plane, radar, or electronics library.

```text
EOM / environment
      ↓ committed truth point or segment
ideal sensor projection
      ↓ ideal measurement
sensor errors / electronics / timing
      ↓ timestamped measurement packet
estimator / tracker
      ↓ estimate
guidance / control / decision logic
      ↓ actuator command
EOM propagation
```

Use `synthetic measurement`, `sensor observation`, or `sensed data`. Do not call
measurements “corrupted truth”; an EOM increment, an IMU increment, and an
estimator error state are different quantities.

## Canonical causal step ordering

At the committed boundary `t_k`, the previous interval has already propagated
using its latched command. Taoryx first publishes the truth associated with the
achieved actuator state at `t_k`, then executes:

```text
COMMIT_TRUTH
  → SAMPLE_SENSORS
  → APPLY_SENSOR_ERRORS
  → SCHEDULE_MEASUREMENT_DELIVERY
  → RELEASE_AVAILABLE_MEASUREMENTS
  → UPDATE_ESTIMATORS_AND_TRACKERS
  → MAKE_DECISIONS
  → LATCH_ACTUATORS
  → ADVANCE_EOM
```

The dependency graph is:

```text
TruthSegment[k-1,k] → Measurement[k] → Estimate[k] → Command[k]
                                                  → TruthSegment[k,k+1]
```

The measurement sampled at `t_k` must not depend on the new command computed
from that measurement. The truth commit is atomic: position, attitude,
velocity, acceleration, transforms, target geometry, environment fields,
forces, moments, achieved actuators, and resource state share one timestamp,
convention set, and integration commit status.

Solver trial states and RK stages are computational only. They are never
sensor-visible. A sensor rate that does not align with the EOM step creates an
explicit scheduler boundary or an accepted-segment accumulator; it does not
authorize post-hoc interpolation between published vehicle states.

The Taoryx problem-language boundary is the successor-only declaration:

```text
*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous delivery-s=0 truth=boundary rate-policy=split
*runtime sensor camera kind=camera cadence-s=0.1 sample=interval delivery-s=0.05 truth=accepted-segment rate-policy=accumulate
```

This declaration registers a `SensorClockSpec` and constrains the next accepted
truth boundary. It does not invent a sensor measurement provider. Measurement
models, noise, mounting, validity, and estimator delivery remain separate
components and must consume the committed point or accepted segment named by
the declaration.

## Truth contracts

The EOM publishes physical truth, not sensor-specific truth:

```python
@dataclass(frozen=True, slots=True)
class TruthPoint:
    time_s: float
    R_eci_from_body: FloatArray
    position_eci_m: FloatArray
    velocity_eci_mps: FloatArray
    acceleration_eci_mps2: FloatArray
    angular_rate_body_radps: FloatArray
    angular_acceleration_body_radps2: FloatArray
    gravity_eci_mps2: FloatArray
```

```python
@dataclass(frozen=True, slots=True)
class TruthSegment:
    start: TruthPoint
    end: TruthPoint
    trajectory: TruthTrajectory | None = None
```

`TruthSegment` is required for interval sensors: IMU delta angles/velocities,
camera exposure, radar dwell, and other measurements that integrate over time.
It contains accepted truth over the interval and may carry a sensor-specific
integral accumulator. It must not be created by linearly interpolating two
published states after the fact, and solver intermediate stages are never
exposed as physical truth.

## Measurement contract

```python
@dataclass(frozen=True, slots=True)
class MeasurementPacket[MeasurementT]:
    sampled_at_s: float
    available_at_s: float
    interval_start_s: float | None
    payload: MeasurementT
    valid: bool = True
```

```python
class SensorModel[MeasurementT](Protocol):
    def sample(
        self,
        truth: TruthSegment,
        *,
        sample_time_s: float,
        rng: np.random.Generator,
    ) -> MeasurementPacket[MeasurementT]: ...
```

Taoryx owns timing, ports, scheduling, delivery, validity, and reproducible
random streams. An external IMU or sensor package owns its error physics.

## IMU adapter rules

The IMU truth adapter converts ECI rigid-body truth into the exact external
library convention. The external package calls this generic input frame
"world"; Taoryx supplies ECI and does not silently substitute ECEF or a local
navigation frame. The adapter must document `R_eci_from_body`, gravity
treatment, sensor mounting transform, lever arm, and interval convention. For a
start-of-interval body-frame contract it may derive:

```text
R_delta,k = R_EB(t[k-1])^T R_EB(t[k])
delta_theta_k = log(R_delta,k)
delta_v_k^B[k-1] ≈ R_EB(t[k-1])^T (v_no_g^E(t[k]) - v_no_g^E(t[k-1]))
```

Required invariants:

- exactly one `ImuModel.measure()` per IMU sampling event;
- strictly chronological calls;
- explicit invalid/initial history behavior on the first call;
- mounting and gravity conventions applied before the external model;
- persistent bias/random-walk state;
- independent reproducible RNG streams per sensor.

### Initial External Binding: `imu-error-model`

The first external implementation is `imu-error-model==0.1.3`, installed by
Taoryx's optional `sensors` extra. It is intentionally not a core runtime
dependency. Its public `ImuModel` satisfies the adapter shape:

```text
reset()
measure(timestamp, velocity_without_gravity_eci,
        orientation_eci_from_body, temperature_celsius=None)
    -> ImuOutput(delta_v, delta_theta, start_time, end_time, temperature)
```

The binding is compatible with the current contract because the package:

- consumes ECI velocity with gravity excluded, passed through the package's
  generic world-frame argument;
- consumes a proper `R_eci_from_body` matrix, passed through the package's
  generic world-frame argument;
- computes interval increments from strictly chronological calls;
- emits increments in the body frame at the interval start; and
- owns stochastic, bias, misalignment, thermal, clipping, and quantization
  effects without calculating dynamics, gravity, or navigation.

The Taoryx adapter owns the following decisions and evidence:

- derive ECI gravity-excluded velocity from the committed truth snapshot rather
  than from solver stages or post-hoc interpolation;
- retain ECI position, velocity, and attitude as the source truth so a body
  fixed to Earth produces the expected Earth-rotation gyro increment;
- apply the declared sensor mounting transform and lever-arm correction before
  calling the external model, or explicitly record them as unsupported;
- map the package's first zero-length baseline result to an explicit invalid or
  initialization packet rather than delivering a false zero measurement;
- wrap `ImuOutput` in `MeasurementPacket`, preserving sampled and available
  times, validity, interval bounds, body-frame payload, and configuration
  provenance; and
- inject one persistent model instance and one reproducible RNG stream per
  sensor, resetting only at a declared run boundary.

For the current free-flight tranche, the explicit gravity-excluded ECI
velocity is sufficient. Ground contact, pad restraint, wheel/track reaction,
and other support-force states are deliberately deferred; when those are
added, the truth contract must expose the support acceleration or force rather
than infer it from a zero velocity derivative.

The adapter proof uses a committed rigid-body truth sequence with known
translation and rotation, verifies increment/frame/unit conventions, checks
first-sample behavior and strict chronology, and feeds the resulting
measurement packets into a deliberately small navigation consumer.
Perfect-navigation mode remains a separate, explicit comparison.

The initial adapter, ideal comparison adapter, dead-reckoning example, and
15-state MEKF example are implemented in `taoryx.sensors` and
`taoryx.navigation`. They are research examples, not flight-qualified
estimators. The runtime now exposes its
committed `TruthPoint`/`TruthSegment` payloads through
`RuntimeVehicle.truth_provider` and delivers packets through the deterministic
`SensorBus` in batch and interactive execution. Scenario sidecars, packet-only
estimators, deterministic drops, timeout manifests, source hashes, and
checkpoint rebind metadata are covered by
[`sensor-scenario-integration.md`](sensor-scenario-integration.md); neither
path bypasses the accepted-truth bus.

Channel-selective orientation work is also explicit. `rotation-only` uses a
body-frame angular-rate packet and an attitude-only dead-reckoner, so a
three-axis table can exercise gyro orientation without requiring translational
excitation. `hybrid-6dof` combines that rotational source with simulated or
substituted ECI translation for full-IMU tests. The artifact records whether
translation is used, unavailable, accepted, synthesized, or externally
substituted; it does not silently treat a table's lack of translation as zero
vehicle acceleration.

The sidecar boundary is typed rather than mode-string-only. Provider, truth,
and attitude-policy definitions use Pydantic discriminated unions and are
normalized into JSON metadata before runtime attachment. Adapter and attitude
implementations are selected through registry-backed factories behind small
interfaces. This keeps vehicle-specific policies, such as distinct quadcopter
and tilt-rotor forward-axis rules, separate from generic scheduling, packet,
and navigation contracts; unsupported variants or missing state channels fail
closed.

The external profile corpus is deliberately not copied into the vehicle
library. `resources/sensors/imu_profiles/catalog.json` pins both the upstream
repository commit and the required package version and labels its hardware
estimates as notional. A sidecar may select a repository file with
`ImuErrorModelAdapter.from_profile(...)` or a packaged profile with
`package:hardware_estimates/hg9900.yaml`; both paths record the source, model
name, declared sample period, metadata, and output-scale conversion in run
provenance. Package `0.1.3` also supplies a versioned `snapshot()` / `restore()`
checkpoint protocol. Taoryx delegates that state and checkpoints the
post-sensor observation stages as one JSON-compatible payload.

## Observation models and controller feedback

Sensor error physics and controller feedback are separate contracts. The base
IMU model produces a timestamped packet first; an optional typed observation
pipeline then applies ordered scale/misalignment, bias, Gaussian noise,
quantization, and dropout stages. Each stage returns a new packet, so committed
plant truth and the upstream model's internal state are not modified. The
sidecar form is:

```yaml
observation:
  stages:
    - kind: scale-misalignment
      accelerometer_matrix: [[1.0, 0.001, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    - kind: bias
      gyroscope_bias_radps: [0.00001, 0.0, 0.0]
    - kind: dropout
      every_n: 20
```

Navigation feedback is explicit and is available only after packet delivery:

```yaml
feedback:
  source: mekf             # plant-truth, dead-reckoning, or mekf
  availability: delivered
  stale_policy: hold       # hold or fail
  max_age_s: 0.05          # required with stale_policy=fail
```

Before the first delivered estimate, the controller uses its ordinary plant
state as an explicit startup fallback. With `source: mekf`, only the
controller observation is replaced; mass, propellant, thermal channels, and
the integration history remain plant truth. A `fail` stale policy fails closed
when no estimate has arrived or the last delivered estimate exceeds
`max_age_s`. Artifacts include the selected source, delivery timestamp,
startup fallback, checkpoint support, and a `plant_truth_immutable` marker.

The Hummingbird HG1700 sidecar demonstrates a higher-fidelity IMU with MEKF
controller feedback. The X8 HG9900 sidecar demonstrates the same packaged
profile/checkpoint path with MEKF recorded in parallel but plant-truth
controller feedback because the supplied X8 aerodynamic table is evidence
bounded at approximately +/-5 degrees beta and +/-12 degrees alpha. Raw
estimator-driven X8 control is intentionally retained as a diagnostic case:
it can exceed that declared table envelope under realistic sensor drift and
must report a validity failure rather than silently extrapolating.

The scenario-level integration sequence for Hummingbird and Skywalker X8 is
defined in [`sensor-scenario-integration.md`](sensor-scenario-integration.md).
It deliberately makes Hummingbird hover the diagnostic first case and X8 the
second free-flight/glide case.

## Focal-plane, seeker, and other sensor pipelines

Sensors are pipelines, not universal truth-plus-Gaussian-noise operators:

```text
truth geometry → mount/boresight → ideal projection
  → distortion/jitter/blur/obscuration → photon/detector noise
  → quantization/saturation/dead pixels → detection/centroiding
  → measurement packet → tracker → guidance
```

Boresight error belongs before projection, read noise at the detector,
quantization at the digital boundary, centroid error after processing, and
latency in packet availability. Lower-fidelity sensors may emit azimuth/
elevation or focal-plane coordinates; higher-fidelity sensors may emit arrays.

## Multi-rate execution

For EOM, IMU, controller, and focal-plane rates that differ, support:

1. **Event-aligned propagation:** stop at every sensor/decision event.
2. **Committed dense output:** query sensor sample times inside a committed
   interval, while stopping at every actuator-decision boundary.
3. **Common event grid:** use a deterministic common grid when practical.

Taoryx must never provide a 10 ms endpoint as ten invented 1 ms IMU samples,
and must never interpolate backward through a controller event. The chosen
policy is part of run provenance.

## Truth isolation and ports

Truth ports are available only to sensor simulators, environment models, truth
logging, evaluation, and visualization. Flight-like models consume ports such
as:

```text
vehicle.imu.raw
vehicle.navigation_solution
seeker.focal_plane
seeker.track
vehicle.air_data
vehicle.control_command
```

Guidance and control receive estimates rather than truth by default. Supported
execution modes bind the same guidance contract to:

```text
perfect-navigation: truth → ideal navigation adapter → guidance
sensor-closed-loop:  truth → sensors → estimator → guidance
hardware/SWIL:       hardware/flight-software measurements → guidance
```

Perfect-information mode must be explicit and visibly labeled.

Spacecraft sensorized studies commonly bind gyros, star trackers, Sun sensors,
magnetometers, and relative-navigation sensors to wheel/thruster control
loops. Their timestamps, latency, validity, and estimator dependencies use the
same measurement bus; sensor models do not directly mutate orbital or
attitude truth.

## Disturbances versus measurement errors

Physical disturbances enter the EOM and affect every sensor through changed
truth. Measurement errors enter only the sensor pipeline and change reported
measurements without changing truth. Wind, turbulence, actuator disturbances,
structural vibration, and thrust variation are process/environment paths;
IMU bias, detector noise, boresight error, quantization, and communication
latency are measurement paths.

## Backlog gates

| Gate | Exit condition |
| --- | --- |
| S0 — Contracts | TruthPoint, TruthSegment, MeasurementPacket, sensor ports, validity, and timing semantics are published. |
| S1 — Commit boundary | Atomic truth staging and immutable committed segments work in batch and stepwise execution. |
| S2 — IMU adapter | External IMU error model receives correct chronological, mounted, gravity-consistent intervals with persistent RNG state. |
| S3 — Multi-rate scheduler | Event-aligned or committed-dense policies pass causal timing and replay tests. |
| S4 — Measurement bus | Delayed packets, validity, drops, multiple sensors, and independent streams work deterministically. |
| S5 — Estimator isolation | Guidance/control cannot consume truth unless explicit perfect-information mode is selected. |
| S6 — Focal-plane pipeline | Ideal projection, detector/electronics effects, processing, and latency are separately inspectable. |
| S7 — Research-ready | IMU, air-data, seeker, and estimator closed-loop examples produce truth, measurement, estimate, command, and applied-control provenance. |

Current implementation status: S0 through S5 are exercised by the sensor
contract, adapter, batch-bus, interactive-bus, observation-pipeline, and
feedback tests. The sensor-family interface is now an internal registry with
plug-in-owned configuration, explicit language-family compatibility, and
versioned payload codecs. The existing IMU providers, bearing-only and
point-source focal-plane IR proof models, and a receiver-level GNSS fix proof
model use the same accepted-truth bus. Ideal/bias/noise witnesses demonstrate
that immutable committed truth is projected into deterministic corrupted
measurements rather than modified in place. S5 is demonstrated at
the navigation consumer boundary: estimators receive packets only, while the
runtime truth provider is used solely by the sensor/artifact path. The runtime
IMU/navigation example exercises the S7 artifact shape for an IMU pipeline.
Guidance integration, a complete S6 focal-plane pipeline, raw GNSS
observables, multisensor fusion, and broader S7 sensor families remain future
work. The bearing-only IR model proves scene-context projection but does not
by itself satisfy S6.

####
