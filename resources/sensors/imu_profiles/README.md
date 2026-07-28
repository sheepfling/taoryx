# IMU Profile Source

Taoryx consumes the validated profile document format from
[`imu-error-model`](https://github.com/sheepfling/imu-error-model). The
currently pinned upstream commit, package version, and profile names are recorded in
[`catalog.json`](catalog.json).

Taoryx currently requires `imu-error-model==0.1.3`. Profiles bundled by that
release can be selected without copying files, for example:

```yaml
sensor:
  provider: imu-error-model
  profile: package:hardware_estimates/hg9900.yaml
```

The adapter exposes the upstream `snapshot()` / `restore()` checkpoint
protocol, and observation-stage state is included when a sidecar composes
additional measurement effects.

The upstream hardware-oriented profiles are explicitly notional estimates.
They are useful for exercising profile loading, output-scale conversion,
temperature terms, and navigation drift comparisons; they are not vendor
specifications, certification data, or flight qualification evidence.

Use an explicit path with `ImuErrorModelAdapter.from_profile(...)`. Do not copy
an estimate into a Taoryx family library without preserving its source path,
upstream commit, profile metadata, and non-authority disposition.
