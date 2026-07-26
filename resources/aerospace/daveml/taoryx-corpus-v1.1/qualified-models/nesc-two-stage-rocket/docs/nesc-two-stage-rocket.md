# NASA/NESC Scenario 17 two-stage rocket

## Scope

This document defines the v0.9 source, runtime, event, mass-property,
trajectory, validation, and packaging contract for
`nasa-nesc-two-stage-rocket-scenario17`.

The model is a conceptual, axisymmetric, two-stage launch vehicle created for
flight-simulation verification. It is not a representation of an operational or
certified launch system.

## Pinned source lineage

```text
repository: nasa/simupy-flight
commit:     70754e6916afc206e8c0abb386d1a9c98bf8f561
license:    NASA Open Source Agreement Version 1.3
```

Upstream source records:

| Role | Path | Git blob SHA |
|---|---|---|
| Aerodynamics | `NESC_data/All_models/two-stage_package/twostage_aero.dml` | `8415a58f0cd1ec5c0ae3a9881813e5e473c25b13` |
| Propulsion | `NESC_data/All_models/two-stage_package/twostage_prop.dml` | `a8050f8673b224f32d9fb85991fdba224cb2948a` |
| Mass and inertia | `NESC_data/All_models/two-stage_package/twostage_inertia.dml` | `5e1a1d5a16daf6ebaf3cd8e305d95f2690011155` |
| Model README | `NESC_data/All_models/two-stage_package/README.html` | `e684ae988496d5d71fb668ba6f3128a7ade64eec` |

Reference trajectories are identified by pinned paths and Git blob SHAs for
simulation participants 04, 05, and 06.

The package contains normalized semantic DAVE-ML reconstructions. It does not
claim that those XML bytes are identical to the upstream files. Exact retrieval
identifiers are retained in `source/nesc/provenance.json`.

## Coordinate, unit, and state contract

| Quantity | Convention |
|---|---|
| Units | SI |
| Body axes | Forward-right-down |
| Local navigation axes | North-east-down |
| Trajectory frame | Earth-centered inertial |
| Attitude | Scalar-first unit quaternion, body to ECI |
| Position origin | Earth center |
| Force/moment application | Center of gravity after MRC transfer |
| Products of inertia | Standard symmetric tensor convention |

## Aerodynamics

Reference geometry:

```text
Sref = 7 m²
cbar = 3 m
bspan = 3 m
```

The source tables provide:

```text
CL(alpha)
CD(sqrt(alpha² + beta²))
CY(beta)
Cm(alpha)
Cn(beta)
Cl = 0
```

Force tables are defined over ±10°. Moment tables are defined over ±20°. The
runtime records table-domain clamping rather than silently calling it an
in-envelope evaluation.

The wind-axis coefficient vector is:

\[
\mathbf C_W = [-C_D,\ C_Y,\ -C_L]^T.
\]

The body force is:

\[
\mathbf F_B = qS R_{BW}(\alpha,\beta)\mathbf C_W,
\qquad q = \tfrac12\rho V^2.
\]

Moments about the source moment-reference center are:

\[
\mathbf M_{MRC} = qS[bC_l,\ \bar c C_m,\ bC_n]^T.
\]

The inertia source reports the vector from MRC to CG in body coordinates. The
runtime transfers the source moment to the current CG with:

\[
\mathbf M_{CG}=\mathbf M_{MRC}+\mathbf r_{CG\rightarrow MRC}\times\mathbf F_B.
\]

## Propulsion

| Stage | Thrust | Specific impulse | Propellant |
|---|---:|---:|---:|
| 1 | 17,000,000 N | 360 s | 180,000 kg |
| 2 | 5,000,000 N | 390 s | 80,000 kg |

The source defines constant thrust and specific impulse during each firing
phase. Propellant consumption is:

\[
\dot m = \frac{T}{I_{sp}\,9.8066}.
\]

The model has no throttle, mixture, gimbal, chamber-pressure, startup, shutdown,
or residual-propellant state.

## Mission event state machine

| Phase | Time range | Thrust | Vehicle configuration |
|---|---|---:|---|
| Stage 1 burn | 0 to 37.380451765 s | 17 MN | Full two-stage stack |
| Stack coast | 37.380451765 to 134.170451765 s | 0 | Empty Stage 1 remains attached |
| Stage 2 burn | 134.170451765 to 195.363635765 s | 5 MN | Stage 1 dry hardware jettisoned |
| Orbit coast | 195.363635765 to 200 s | 0 | Empty Stage 2 |

The coast lasts exactly 96.79 seconds. At Stage 2 ignition the model applies an
instantaneous 35,000 kg jettison and switches to the Stage 2 CG, MRC, and inertia
schedule.

Integrator steps are split at every discontinuity so a fixed step never smears
burnout, ignition, or staging across a time interval.

## Mass, CG, and inertia

Source endpoints:

| State | Mass | CG aft of nose | Ixx | Iyy = Izz |
|---|---:|---:|---:|---:|
| Liftoff stack | 314,000 kg | 16.918790 m | 353,250 kg·m² | 33,501,637.473461 kg·m² |
| Stage 1 burnout stack | 134,000 kg | 9.421642 m | 150,750 kg·m² | 10,886,636.572139 kg·m² |
| Stage 2 ignition | 99,000 kg | 4.797980 m | 111,375 kg·m² | 941,063.762626 kg·m² |
| Stage 2 burnout | 19,000 kg | 3.947368 m | 21,375 kg·m² | 212,384.868421 kg·m² |

Mass, CG, and principal inertias interpolate linearly with remaining propellant
within each stage. All products of inertia are zero. During burn, the rigid-body
rotational equation includes the scheduled inertia-rate contribution.

## Environment and equations of motion

The trajectory model includes:

- WGS-84 ellipsoid geometry;
- Earth rotation;
- central gravity plus J2;
- ECI position, velocity, attitude, and body angular rate;
- a full symmetric rigid-body inertia matrix;
- variable mass and inertia;
- event-aligned classical RK4.

The atmospheric implementation follows US Standard Atmosphere 1976 geopotential
layers through 84.852 km. Above that altitude, density and pressure continue
with a 7 km exponential scale height at the top-layer temperature. This is an
explicit approximation, not a claim of complete US76 thermosphere fidelity.

## Initial condition

The packaged baseline starts:

```text
latitude:                 0 deg
longitude:                0 deg
altitude:                 0 m
yaw:                     90 deg
pitch:                   55.220 deg
roll:                     0 deg
body/local initial speed: 0.1 ft/s upward
body rates wrt launch site: zero
```

The ECI velocity and body angular rate include Earth rotation. Source-era README
text and some later participant traces differ on whether the near-zero initial
speed is 0.1 ft/s or approximately 0.1 m/s. The selected package baseline uses
0.1 ft/s and records the discrepancy.

## Source-equivalence verification

The direct runtime is compared with the three compiled normalized DAVE-ML
components across:

```text
aerodynamics:  multiple alpha/beta combinations and all six coefficients
propulsion:    Stage 1, coast, and Stage 2 outputs
inertia:       full, intermediate, empty, and staged configurations
```

Result:

```text
comparisons:                 162
maximum absolute error:      2.7755575615628914e-17
```

## Trajectory verification

The default acceptance uses:

```text
duration:                    200 s
fine RK4 step:               0.01 s
fine output step:            0.1 s
coarse RK4 step:             0.02 s
coarse output step:          0.2 s
selected reference points:   11
```

Fine result:

```text
final altitude:              242792.707962 m
final true airspeed:           8358.683206 m/s
final pitch:                     12.429563 deg
final mass:                   19000 kg
maximum dynamic pressure:    395807.616397 Pa
maximum-q time:                  23.7 s
```

Selected checkpoint errors:

```text
altitude RMSE:                 2609.929275 m
maximum altitude error:        8315.707962 m
pitch RMSE:                       1.528583 deg
maximum pitch error:              3.368958 deg
speed RMSE:                       7.758304 m/s
maximum speed error:             22.016794 m/s
```

Fine/coarse endpoint deltas:

```text
altitude:                       0.060348843 m
pitch:                          0.000045169 deg
true airspeed:                 -0.000142669 m/s
```

## Acceptance meaning

`RocketVerificationReport.passed` requires:

1. Source equivalence within `1e-12`.
2. Exact event masses within `1e-8 kg`.
3. Selected checkpoint maxima within 10 km, 4°, and 30 m/s.
4. Fine/coarse endpoint agreement within 1 m, 0.002°, and 0.01 m/s.
5. Final altitude within the 234–252 km participant envelope.

It does **not** require the osculating perigee to exceed 125 km. The current
200-second osculating perigee is below that value. The source README's qualitative
perigee statement is carried as a separate failed diagnostic so it cannot be
mistaken for a reproduced result.

## Package contents

```text
manifest.json
runtime/binding.json
runtime/table-inventory.json
source/nesc/*.normalized.dml
source/nesc/provenance.json
tables/aerodynamic-force-coefficients.csv
tables/aerodynamic-moment-coefficients.csv
tables/mass-properties-schedule.csv
tables/propulsion-stages.csv
tables/mission-events.csv
validation/acceptance.json
validation/fine-trajectory-summary.json
validation/coarse-trajectory-summary.json
validation/scenario17-trajectory.csv
validation/nasa-sim06-checkpoints.csv
validation/nasa-sim06-comparison.csv
docs/model-contract.md
checksums.sha256
```

Every declared artifact is checked before the runtime package is loaded.

## CLI

```bash
tx-aircraft inspect-nesc-rocket

tx-aircraft export-nesc-rocket-source --output-dir build/source

tx-aircraft export-nesc-rocket-tables --output-dir build/tables

tx-aircraft simulate-nesc-rocket \
  --integration-step-s 0.01 \
  --output-step-s 0.1 \
  --trajectory-csv build/trajectory.csv \
  --output build/summary.json

tx-aircraft accept-nesc-rocket --output build/acceptance.json

tx-aircraft build-nesc-rocket build/rocket.txair

tx-aircraft verify-nesc-rocket \
  build/rocket.txair \
  --rerun-trajectory \
  --output build/package-verification.json
```
