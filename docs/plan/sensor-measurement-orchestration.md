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
    R_world_from_body: FloatArray
    position_world_m: FloatArray
    velocity_world_mps: FloatArray
    acceleration_world_mps2: FloatArray
    angular_rate_body_radps: FloatArray
    angular_acceleration_body_radps2: FloatArray
    gravity_world_mps2: FloatArray
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

The IMU truth adapter converts generic rigid-body truth into the exact external
library convention. It must document `R_world_from_body`, gravity treatment,
sensor mounting transform, lever arm, and interval convention. For a start-of-
interval body-frame contract it may derive:

```text
R_delta,k = R_WB(t[k-1])^T R_WB(t[k])
delta_theta_k = log(R_delta,k)
delta_v_k^B[k-1] ≈ R_WB(t[k-1])^T (v_no_g^W(t[k]) - v_no_g^W(t[k-1]))
```

Required invariants:

- exactly one `imu.step()` per IMU sampling event;
- strictly chronological calls;
- explicit invalid/initial history behavior on the first call;
- mounting and gravity conventions applied before the external model;
- persistent bias/random-walk state;
- independent reproducible RNG streams per sensor.

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

####
