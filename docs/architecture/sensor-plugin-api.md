# Sensor plug-in API

Taoryx sensor models are internal, typed plug-ins behind a stable runtime
boundary. The host owns causal execution and persistence; each plug-in owns
its family-specific projection, configuration, physics, and payload.

This is currently an in-process extension API. It is deliberately separate
from installable Python entry-point discovery until the contract has been
exercised by more sensor families.

## Runtime boundary

```text
accepted EOM boundary
        |
        v
committed SensorContext / SensorContextSegment
        |
        v
registered sensor-family implementation
        |
        v
typed MeasurementPacket envelope
        |
        +--> delayed delivery and packet subscribers
        +--> versioned payload codec and run artifact
        +--> estimator, tracker, or image processor
```

The generic contracts live in `taoryx.sensor_api`. `SensorBus` remains the
owner of sample clocks, delivery, drop accounting, sequence numbers, and one
deterministic random stream per binding. Solver trial states and future truth
are not exposed through the context.

`MeasurementPacket` is a common envelope. Its `payload` is not a universal
numeric vector; it is a family-specific immutable type identified by
`schema_id`. The payload codec registry provides JSON encoding, checkpoint
restoration, and a concise semantic contract without a central type switch.

## Language, runtime, and plug-in ownership

The sensor system spans three layers, but they do not own the same semantics:

| Layer | Owns | Does not own |
| --- | --- | --- |
| Successor `.prb` language | sensor identity, broad family, cadence, phase, point/interval mode, delivery latency, and committed-truth timing policy | concrete provider, payload schema, bias/noise law, target selection, image formation |
| Runtime host | accepted-boundary scheduling, immutable context, per-binding RNG, packet sequencing/routing, latency release, checkpoint state | family physics or interpretation of a plug-in payload |
| Sensor plug-in and sidecar | ideal measurement projection, family configuration, bias/noise/outage/gating, typed payload, versioned codec | mutation of plant truth, solver stepping, delivery scheduling |

For example, the language declaration

```text
*runtime sensor nose_ir kind=infrared cadence-s=0.02 sample=instantaneous delivery-s=0.01 truth=boundary rate-policy=split
```

declares a provider-neutral infrared clock. A sidecar with the same sensor name
may select `ir-bearing` or `ir-point-source`. Plug-in manifests advertise the
language families they accept, and attachment rejects a mismatch. Thus
`kind=ir-bearing` is a plug-in identifier and is intentionally not valid as a
language sensor family.

When a matching language clock exists, that clock is authoritative for timing
and retains its provider-neutral family kind. When no language clock exists,
the sidecar supplies timing and the manifest supplies its standalone runtime
clock kind and default cadence. The plug-in provider never silently replaces a
declared language clock.

This split is deliberate. Stable causal concepts belong in the language;
open-ended physical models and error parameters belong in plug-ins. Adding a
new detector model should not require changing the `.prb` grammar.

## Plug-in declaration

Each `SensorPluginDescriptor` provides:

- a versioned `SensorPluginManifest`;
- the sensor family, compatible language families, and runtime clock kind;
- supported point or interval sampling modes;
- supported truth modes and required truth capabilities;
- declared output ports and payload schema IDs;
- a plug-in-owned Pydantic configuration model; and
- a construction factory receiving mounting, seed, services, and resource
  references from the host.

Provider configuration is an open envelope:

```yaml
provider:
  kind: ir-bearing
  config:
    target_id: target
    angular_noise_stddev_rad: 0.0005
```

The generic envelope is validated first. The selected descriptor then
validates `config`; unsupported kinds and unknown family-specific fields fail
closed. Existing flat IMU sidecars remain accepted and are normalized through
the same registry.

## Truth projection and corruption rule

"Corrupting truth" means deriving a measurement from immutable committed
truth; it never means modifying the plant state. Every basic model follows:

```text
committed host/scene truth x(t)
    -> ideal family projection h(x, mount, context)
    -> declared bias/noise/gating/outage
    -> typed measurement z(t)
    -> packet latency and delivery
```

An all-zero error configuration is the ideal projection rung. Nonzero error
settings use the runtime-owned seeded random stream. The RNG state, model
state, and packet sequence are checkpointed for host-only scenarios, so replay
or checkpoint continuation produces the same corruption sequence.

The basic models make this rule concrete:

- IR bearing: `z = [azimuth, elevation] + bias + N(0, sigma^2 I)` after
  mounting, range, and field-of-view projection.
- Point-source focal plane: a pinhole projection maps line of sight to
  `[column, row]`, followed by pixel bias and `N(0, sigma_px^2 I)`.
- GNSS fix: `z_position = position_ECI + bias_position + N(0, sigma_p^2 I)`
  and equivalently for velocity, with an optional independent outage draw.
- IMU: committed point/interval truth first becomes ideal increments; the IMU
  error provider or ordered observation stages then apply scale, misalignment,
  bias, noise, quantization, or dropout.

The tests compare ideal and corrupted configurations against the same truth,
verify exact bias offsets where noise is zero, and verify seeded replay where
noise is enabled. The executable witness is
[`basic_corruption_demo.py`](../../examples/sensors/basic_corruption_demo.py).

## Committed context and services

`TruthPoint` remains host-vehicle kinematics. It is wrapped in a
`SensorContext` rather than expanded with every future scene property. A
context contains:

- a committed snapshot identity;
- host-vehicle truth;
- zero or more `EntityTruth` records; and
- an immutable environment property snapshot.

Heavy functionality is injected separately through `SensorServices`.
`CommittedSceneQuery` is the boundary for snapshot-bound ray queries, and
`SensorArtifactStore` is the boundary for large arrays. `ArrayFrameReference`
allows a future focal-plane model to put an immutable content-addressed frame
reference in a packet instead of embedding megabytes of pixels into JSON
manifests and checkpoints.

A callback-supplied scene context is intentionally not reconstructed during
automatic checkpoint restore. Such scenarios require an explicit context
rebind. Host-only sensors retain automatic checkpoint behavior, including the
runtime-owned random-stream state.

## Bundled families

### Inertial

The existing `imu-error-model`, ideal IMU, ideal gyroscope, and translation
acceleration implementations are registered descriptors in
`taoryx.sensor_plugins.imu`. Their established measurement behavior and
navigation consumers are retained. IMU, gyro, and acceleration payloads now
use versioned codecs rather than scenario-level `isinstance` serialization.

The older IMU profile fields on `SensorScenarioSpec` remain as a schema-v1
compatibility surface. New plug-in configuration belongs under
`provider.config`.

### Infrared bearing and point-source focal plane

`ir-bearing` is the first non-inertial scene sensor. It projects a selected
committed entity through the declared sensor mount and emits azimuth/elevation
in the sensor frame. It supports FOV/range rejection, angular bias, white
noise, deterministic replay, latency, and standard packet validity.

It does not emit target world position. `ir-point-source` is the next
deliberately small rung: a pinhole projection emits a target centroid in
focal-plane pixels with declared centroid bias, Gaussian pixel noise,
covariance, image dimensions, and FOV/range invalidation. It still does not
emit a raw frame or claim radiometric fidelity.

Higher rungs remain in the same family:

```text
bearing-only
  -> point-source focal-plane centroid
  -> radiometric focal plane
  -> rendered scene and detector/electronics pipeline
```

### GNSS fix

`gnss-fix` emits a receiver-level ECI position/velocity fix with covariance,
bias, white error, outage, latency, and deterministic replay. ECI is explicit
in both its payload and artifact contract.

It is not a constellation or RF model. Per-satellite pseudorange, range rate,
carrier phase, visibility, clock, ephemeris, and atmospheric effects belong
in a later `gnss-observables` payload and receiver-processing pipeline.

Example sidecars are provided for
[`ir-bearing`](../../examples/sensors/ir_bearing_v1.yaml),
[`ir-point-source`](../../examples/sensors/ir_point_source_v1.yaml), and
[`gnss-fix`](../../examples/sensors/gnss_fix_v1.yaml).

## Current limitations and next extensions

- A sensor instance currently routes one declared output port through one bus
  binding. Multi-output focal-plane pipelines need port-aware subscriptions
  before a single instance emits both raw frames and detections.
- `SensorContextSegment` currently contains committed endpoints. Exposure,
  dwell, and rolling-shutter models still need an accepted trajectory or
  accumulator contract; post-hoc interpolation remains forbidden.
- Scene-query and array-artifact protocols are defined but have no bundled
  renderer, radiometric model, raw-array provider, or artifact-store
  implementation yet. The point-source plug-in emits only a centroid.
- Installed third-party sensor discovery is deferred. The internal registry
  is the compatibility test bed for that later boundary.
- The generic ordered observation pipeline currently transforms inertial
  payloads only. IR and GNSS use their plug-in-owned basic error parameters;
  a future payload-generic corruption API must preserve units and schema
  semantics rather than treating every payload as an anonymous vector.
- Navigation consumers remain inertial-specific. IR detections and GNSS fixes
  are delivered as typed packets but do not yet have bundled tracker/fusion
  consumers.

These limitations keep the extraction honest: the abstraction is proven
against inertial increments, angular detections, focal-plane centroids, and
receiver fixes without claiming radiometric imagery, constellation physics,
or multisensor-estimation work is complete.

####
