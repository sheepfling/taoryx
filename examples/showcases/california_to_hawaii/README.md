# California-to-Hawaii synthetic long-range flight

Status: runnable synthetic showcase using the native TAORYX 6-DOF runner.

This example invents a high-speed, high-altitude vehicle and asks it to follow
a great-circle route from coastal California to Honolulu. It is intended to
exercise the showcase and telemetry pipeline, not to describe a real vehicle,
weapon, aircraft, or achievable mission.

The route is a force-integrated inertial reference scenario with rigid-body
attitude channels and five explicit mission phases:

| Phase | Time window | Purpose |
| --- | ---: | --- |
| Stage 1 powered | 0–180 s | Initial ascent and first-stage burn |
| Stage 2 powered | 180–360 s | Climb above the atmosphere |
| Coast to apogee | 360–700 s | Exoatmospheric coast and apogee |
| Hypersonic glide | 700–1,650 s | Drag- and attitude-controlled descent |
| Terminal ProNav | 1,650 s–Earth intersection | Native route capture and target hit |

The native acceptance test requires the phases to occur in order, the
trajectory to exceed 100 km altitude for a sustained interval, the glide to
remain hypersonic, and the Earth-intersection state to fall within a 40 km
target tolerance of Honolulu at no more than 100 m altitude. This is a
synthetic target-hit criterion, not a validated vehicle or thermal design.
The model is intentionally an invented showcase rather than a validated
vehicle or thermal design.

## Invented scenario parameters

| Parameter | Value |
| --- | ---: |
| Start | 34.70° N, 120.60° W |
| Destination | 21.31° N, 157.86° W |
| Nominal apogee target | 220,000 m |
| Integrated glide range | Mach 4.5–13.0 |
| Initial mass | 12,000 kg |
| Cruise mass | 8,000 kg |
| Wind | 35 m/s east, 8 m/s north |
| Sample interval | 10 s |

The output uses the normal `RunArtifact` contract and writes text, JSON,
SQLite, and PNG views under the requested artifact directory.

Run it from the repository root:

```bash
python tools/dev.py showcase-california-hawaii
```

The portable task runner uses the repository virtual environment when it is
available, so the command works from a clean checkout without relying on a
global `PYTHONPATH`. To run the artifact acceptance test separately:

```bash
python tools/dev.py test-artifacts
```

The force model includes central gravity, a native altitude-aware exponential
atmosphere and ECFC wind provider, table-shaped aerodynamic drag, staged
propellant flow, a bounded control-force guidance law, and terminal
proportional navigation, with prepared aerodynamic force and moment tables. The
generated panels show route geometry, altitude and range, altitude/Mach,
mass/propulsion, thermal exposure, and attitude/body-rate channels.
`generate()` executes `mission.prb` and `aero.tbl` through `run_files()` and
renders directly from the native `RunArtifact`.

The native route uses declared propulsion, aerodynamic force/moment tables,
attitude moments, mass flow, and terminal route guidance. It contains no
showcase-only force callback or second integrator.

Validate the acceptance contract with:

```bash
python -m pytest tests/unit/test_california_to_hawaii_showcase.py -q
```
