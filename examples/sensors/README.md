# Basic sensor plug-in examples

These examples demonstrate one rule: plant and scene truth remain immutable,
while a sensor plug-in projects that truth into its own observable and then
adds declared bias, noise, gating, outage, or latency. A corrupted measurement
is a new packet; it is never written back into the plant state.

Run the deterministic ideal-versus-corrupted witness from the repository root:

```bash
PYTHONPATH=src python3 examples/sensors/basic_corruption_demo.py
```

The output intentionally includes truth beside ideal and corrupted packets for
debugging. Normal sensor artifacts do not expose target world position through
IR payloads.

## Language versus plug-in configuration

The successor language owns the provider-neutral clock and causal sampling
contract:

```text
*runtime sensor nose_ir kind=infrared cadence-s=0.02 sample=instantaneous delivery-s=0.01 truth=boundary rate-policy=split
*runtime sensor gnss kind=gnss cadence-s=1 sample=instantaneous delivery-s=0.15 truth=boundary rate-policy=split
```

The language `kind` is a broad family. It does not select an implementation or
define its error law. The sidecar selects a registered plug-in and validates
plug-in-owned configuration:

```yaml
sensor:
  name: nose_ir
  provider:
    kind: ir-bearing
    config:
      target_id: target
      azimuth_bias_rad: 0.0002
      angular_noise_stddev_rad: 0.0005
```

The shared sensor name binds the language clock to the sidecar. Attachment
fails if the declared family is incompatible with the selected provider. For
example, an `infrared` clock accepts `ir-bearing` or `ir-point-source`, while a
`gnss`/`gps` clock accepts `gnss-fix`.

If the `.prb` declares that clock, its timing is authoritative. The cadence and
delivery fields in a standalone sidecar are used when no matching language
clock exists; the plug-in never replaces an existing language clock.

## Bundled minimal models

| Provider | Input projection | Basic corruption | Output |
| --- | --- | --- | --- |
| `ideal` | committed rigid-body truth over a point or interval | none | IMU increments |
| `imu-error-model` | the same IMU projection | configured IMU error model | IMU increments |
| `ir-bearing` | host/target geometry through sensor mounting | angular bias and Gaussian noise; FOV/range invalidation | azimuth/elevation detection |
| `ir-point-source` | pinhole projection of a point target | centroid bias and Gaussian pixel noise; FOV/range invalidation | focal-plane centroid |
| `gnss-fix` | host ECI position and velocity | vector bias, Gaussian error, and outage probability | receiver-level ECI fix |

With every bias, noise, and outage setting at zero, the models form an ideal
projection rung. Nonzero settings use the runtime-owned seeded random stream,
so replay and checkpoint continuation are deterministic. Scene-backed IR
checkpoints still require the caller to rebind its committed-context callback.

The point-source model is intentionally not an IR image simulator. It proves
pixel-coordinate projection and corruption. Radiometry, extended targets,
optics, detector/electronics response, and full arrays belong to later focal-
plane plug-ins using the scene-query and array-artifact service contracts.

Sidecars:

- [`ir_bearing_v1.yaml`](ir_bearing_v1.yaml)
- [`ir_point_source_v1.yaml`](ir_point_source_v1.yaml)
- [`gnss_fix_v1.yaml`](gnss_fix_v1.yaml)

See the full [sensor plug-in architecture](../../docs/architecture/sensor-plugin-api.md)
for ownership, timing, payload, and checkpoint rules.
