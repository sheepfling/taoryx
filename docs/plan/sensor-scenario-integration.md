# Sensorized Scenario Integration Plan

Status: implementation complete for the committed free-flight tranche; residual
Earth-rate and 3-DOF attitude limitations remain explicitly fail-closed.

Sensor integration follows the same ordered fidelity ladder as the plant. T0
uses acceleration-only translation, T1 may use explicitly synthesized attitude,
T2 requires physical rotational truth for full IMU/MEKF evidence, and T3 adds
effector/resource telemetry. See [Fidelity-first vehicle integration program](fidelity-first-integration-program.md).

## Objective

Prove that a real Taoryx scenario can be run through the complete sensor path:

```text
scenario source
  -> lowered vehicle and accepted truth
  -> sensor clock and persistent model
  -> delayed/valid measurement packets
  -> dead-reckoning or MEKF estimator
  -> standard run artifact and Matplotlib evidence
```

The proof must retain the existing vehicle-family evidence. A sensorized run
must not replace plant validation, silently change the dynamics, or claim that
a notional IMU profile is hardware qualification.

## Vehicle Order

The vehicle order is also a fidelity order: complete the smallest translation
case and its artifact contract before attaching the next sensor or estimator
channel. Higher-tier sensor runs may be diagnostic, but they do not promote a
vehicle past a blocked lower-tier plant contract.

### 1. Hummingbird diagnostic integration

Use Hummingbird first because it has the most revealing sensor truth case:

- a point-mass reduction and native rigid-body 6-DOF path;
- hover, rate-damping, disturbance, waypoint, and route scenarios;
- explicit rotor allocation and body rates;
- existing force/moment closure and convergence evidence; and
- a clean opportunity to test Earth rotation while the vehicle is nearly
  stationary in the local frame.

The first case is the 10-second rigid-body hover, not takeoff, landing, or pad
contact. The rate-damped hover is the second case because it exercises angular
motion and mounting/frame handling. Landing and touchdown remain out of scope
until support-force truth is modeled explicitly.

Hummingbird is also a deliberate canary for the current gravity-excluded
velocity assumption. A vehicle that is hovering has nonzero thrust-supported
specific force even when its velocity is nearly zero. If the external IMU
adapter produces zero accelerometer increment for that case, the result is a
truth-contract failure to fix, not a successful sensor demonstration.

### 2. Skywalker X8 operational integration

Use X8 second because it is a clean free-flight demonstration with:

- 3-DOF reduction and native rigid-body 6-DOF cases;
- powered trim and bounded recovery;
- route and crosswind scenarios;
- source-composed aerodynamic/control tables; and
- a meaningful glide/free-flight energy history without ground-support
  semantics.

The initial X8 sequence is a short source-trim hold, followed by the long
powered recovery or rectangle route. The 3-DOF case is a scheduler and
artifact smoke test; the 6-DOF case is the authoritative IMU and estimator
integration. A 3-DOF versus 6-DOF sensor comparison is valid only for shared
quantities such as position, velocity, and declared frame metadata. Attitude
and body-rate comparisons must be marked unavailable for the 3-DOF model.

## Integration Boundary

### Scenario declaration

Keep the `.prb` surface provider-neutral. A scenario may declare:

```text
*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous \
  delivery-s=0.0 truth=boundary rate-policy=split
```

The model, profile, estimator, and seed belong in a sidecar/run
specification, not in the historical problem grammar. The sidecar must name:

- sensor binding and vehicle ID;
- model provider and profile path;
- profile source repository, commit, authority, and output scales;
- random seed and run identifier;
- mounting rotation and lever arm;
- delivery latency and packet-drop policy;
- truth convention (`ECI`, gravity treatment, and free-flight/support mode);
- estimator mode and initialization covariance; and
- artifact output and plotting policy.

The runner should accept this specification through an explicit API/CLI
argument. A run that declares a clock but supplies no provider remains
clock-only and must say so in its metadata.

### Truth provider

Lowered rigid-body vehicles must receive a standard truth provider rather than
each scenario inventing one. The provider maps an accepted `RuntimeState` to:

- ECI position and velocity;
- ECI-from-body attitude;
- body angular rate;
- ECI gravity;
- gravity-excluded velocity, only when the EOM has produced that quantity;
- ECI acceleration or body specific force when available; and
- temperature/support metadata when declared.

The existing rigid-body state is ECIC and carries quaternion, body-rate,
position, and velocity fields. The implementation must verify the ECIC/ECI
relationship and Earth-rate convention before labeling the sensor artifact
ECI. Existing family baselines use `omega=0`; sensor overlay scenarios must
set the physical Earth rate explicitly without mutating the baseline fixtures.

### Runner and artifact

Add one shared scenario sensor runner rather than separate Hummingbird and X8
notebooks. It should:

1. load and hash the `.prb`, tables, sidecar, profile, and source commit;
2. lower the scenario and attach the truth provider and `SensorBus`;
3. run the unchanged plant with accepted sensor boundaries;
4. deliver packets to an estimator through a packet-only subscriber;
5. record invalid, delayed, dropped, and timed-out samples;
6. run the same scenario in ideal/perfect-information comparison mode only
   when explicitly selected; and
7. emit the standard `RunArtifact` plus sensor-specific packet and estimator
   records.

## Artifact Contract

Every sensorized scenario produces one manifest with these sections:

```text
artifact_schema
scenario_identity
source_inputs
run_configuration
truth_contract
sensor_bindings
measurement_summary
measurements/<sensor>.jsonl
estimator_summary
estimates/<estimator>.jsonl
comparison
plots
limitations
```

Each measurement record must include sampled time, available time, interval
bounds, validity, payload frame/units, provider, seed, and source profile
identity. The summary must include emitted, valid, invalid, delivered,
dropped, and still-queued counts. A timeout is a first-class termination
reason, not a missing result; the manifest must identify the last accepted
truth time and the next requested sensor time so it can be rerun with a larger
time budget.

The comparison section must distinguish:

- no-sensor plant baseline versus sensorized plant;
- ideal measurement versus error-model measurement;
- dead reckoning versus MEKF;
- 3-DOF versus 6-DOF shared channels; and
- nominal profile versus alternate profile or seed.

Required plots are flown trajectory/attitude, packet cadence and latency,
truth-versus-estimate position/velocity/attitude, estimator error growth,
packet validity/drop timeline, and scenario phase/segment boundaries. The
plot manifest must point back to the exact run and source hashes.

## Execution Phases

### S0 - Contract and profile fixture

- Keep the existing `TruthPoint`, `TruthSegment`, `MeasurementPacket`, and
  `SensorClockSpec` contracts.
- Add a committed synthetic profile fixture for deterministic tests.
- Pin the external profile corpus separately and retain its notional status.
- Add a sidecar schema with strict validation and deterministic serialization.

Exit: a sidecar can be loaded without importing vehicle-specific code, and a
clock-only run is distinguishable from a model-bound run.

### S1 - Generic lowered truth provider

- Add rigid-body truth projection for Hummingbird and X8.
- Add explicit frame and Earth-rate diagnostics.
- Populate acceleration/specific-force fields or fail closed when the chosen
  IMU model requires data that the EOM does not provide.
- Preserve transition pre/post truth and never sample solver stages.

Exit: the same provider test passes for synthetic, Hummingbird, and X8 states;
position, velocity, attitude, body rate, and gravity conventions are recorded.

### S2 - Hummingbird hover sensor run

Run:

- `SV05_hover_validation_6dof.prb` with the deterministic synthetic profile;
- the same case with `hg1700ag58.yaml` or another explicitly selected
  upstream profile; and
- `SV05_rate_damped_hover_6dof.prb` for body-rate and attitude excitation.

Exit conditions:

- baseline and sensorized plant histories agree at common accepted times;
- packet count equals the declared cadence, subject to the initial invalid
  baseline and explicit drops;
- ECI Earth-rate behavior is visible in the static/low-motion case;
- hover accelerometer semantics are physically explained and nonzero when the
  truth contract says lift support is present;
- DR and MEKF consume packets only and remain finite; and
- artifacts contain profile, seed, clock, truth, packet, estimator, and plot
  provenance.

### S3 - Hummingbird disturbance and lifecycle boundary

Run the disturbed-response case after S2. Exercise a declared event or phase
boundary without landing/contact first. Verify sensor re-baselining, packet
validity, estimator continuity, and transition metadata. Defer touchdown until
support acceleration/contact truth is first-class.

Exit: a committed transition cannot produce a false IMU jump or a stale
pre-transition interval.

### S4 - X8 powered/glide integration

Run:

- `SV03_source_trim_hold_30_6dof.prb` for the short integration gate;
- `SV03_long_powered_recovery_6dof.prb` or the long rectangle route for
  sustained flight; and
- `SV03_3dof.prb` or the reduction case for shared-channel comparison.

Exit conditions:

- accepted sensor timing remains causal through powered control;
- profile scaling and mounting are unchanged between X8 cases;
- 6-DOF attitude/body-rate packets are present and labeled unavailable in the
  3-DOF comparison;
- route, speed, altitude, alpha, sideslip, and force/moment closure gates
  remain within their existing declared limits; and
- sensorized results retain the existing X8 table and scenario provenance.

### S5 - Replay, estimator, and failure handling

- Re-run Hummingbird and X8 with identical source hashes and seed.
- Compare batch and interactive stepping.
- Inject deterministic packet drops and latency.
- Record timeout cases and provide a focused rerun command with a larger
  horizon/step budget.
- Exercise checkpoint save/load with an explicit sensor/estimator rebind
  requirement; never imply that metadata-only checkpoint loading restores a
  stochastic external model.

Exit: two identical runs have identical packet hashes and estimator outputs;
changed seed/profile/drop policy changes only the declared affected outputs.

### S6 - Standard visualization and release packet

- Add the sensor plots to the existing Matplotlib artifact bundle.
- Add one CLI command for each scenario family through the shared runner.
- Add focused Hummingbird/X8 tests and artifact tests; preserve existing family
  tests unchanged.
- Update the sensor plan and architecture/LaTeX catalog with the integration
  result and known limitations.

Exit: a reviewer can identify exactly what was run, how it was parameterized,
which samples succeeded or failed, and whether the result is plant,
sensor-model, estimator, or comparison evidence.

### S7 - Lower-fidelity sensor contracts

- Add an acceleration-only translation contract for point-mass vehicles. It
  publishes ECI specific-force increments and a translation navigator without
  manufacturing an attitude or gyro channel.
- Add a pseudo-6DOF projection for point-mass vehicles. Body-forward follows
  velocity, controller bank is applied about that axis, and low-speed/nadir
  framing uses hold, deterministic heading, and parallel-transport fallback
  rules.
- Derive body rate from continuous quaternion differences and reject MEKF
  attachment when orientation/body-rate truth is unavailable.
- Record whether orientation and body rate are accepted truth, synthesized
  truth, or unavailable in the standard artifact.

Exit: Hummingbird 3-DOF translation-only and pseudo-6DOF sidecars complete
through the shared runner, and artifacts distinguish physical channels from
synthesized channels without random attitude flips.

### S8 - Channel-selective SWIL/HWIL truth

Treat translational and rotational truth as independent authorities. A
three-axis table can excite a real IMU's rotational channels while the vehicle
translation is supplied by a simulated or otherwise substituted ECI state;
this is not a malformed full-6DOF run. The runtime now exposes a
`hybrid-6dof` composition seam: `attach_sensor_scenario` accepts an injected
rotational truth provider containing ECI-from-body attitude and body angular
rate, and combines it with the vehicle's accepted translation provider. The
artifact records both channel sources and allows the translation source to be
substituted explicitly.

The external provider contract is strict: timestamps must match accepted
boundaries, attitude must be a proper ECI-from-body rotation, and angular rate
must be in body axes. The table adapter remains an integration boundary rather
than a hidden YAML magic source; a future HWIL adapter can stream the table's
measured or commanded rotation into this callback without changing the IMU
model or estimator.

Exit: a synthetic table callback passes through the full IMU adapter path,
artifacts distinguish simulation/substituted translation from external
rotation, and missing or mis-timestamped rotational truth fails closed.

### S9 - Rotation-only IMU evidence

Add a gyro-only path for table excitation and rotational sensor qualification.
The `rotation-only` sidecar uses an angular-rate packet and an
attitude-only dead-reckoner; translation remains explicitly unused rather than
being fabricated. A rigid-body vehicle may provide accepted orientation and
body rate directly, while a point-mass or external-table setup can use the S8
rotational provider composition. This gives SWIL/HWIL tests a way to validate
gyro increments and ECI attitude propagation before full accelerometer
translation substitution is connected.

Exit: the Hummingbird rotation-only sidecar produces gyro packet artifacts,
attitude estimates, and no translation estimate dependency; full IMU and
translation-only scenarios remain unchanged.

### S10 - Typed provider and vehicle-policy contracts

Keep sidecar configuration extensible without making the runtime depend on a
growing set of mode strings. Provider, truth, and attitude-policy definitions
are parsed as Pydantic discriminated unions. Runtime adapters and attitude
projections are created through registry-backed factories behind callable
interfaces, so a new vehicle family can add a provider or policy without
changing the scheduler or packet schema.

The first vehicle-specific policy is `rotorcraft`, with a required
`vehicle_type` discriminator for `quadcopter` or `tilt-rotor` and an explicit
`forward_source` of `velocity` or `body-axis`. Body-axis projection requires
quaternion state channels and fails closed when they are absent; it does not
silently reuse the velocity-aligned rocket/aircraft policy. The legacy scalar
sidecar fields remain accepted and normalize to the typed velocity-aligned
contract for compatibility.

Exit: artifacts preserve the normalized provider/truth/policy definitions,
unknown variants are rejected before execution, default and rotorcraft policy
paths are covered by unit tests, and custom registry entries have a stable
interface for later vehicle families.

### S11 - Source-pinned profile and Earth-rate release evidence

The deterministic `taoryx_demo.yaml` fixture remains the regression baseline.
The catalog-pinned upstream checkout may be compared explicitly, but its
hardware-estimate profiles remain notional. Run:

```bash
python tools/compare_imu_profiles.py \
  --baseline tests/fixtures/imu_profiles/taoryx_demo.yaml \
  --candidate ../imu-error-model/examples/imu_profiles/hardware-estimates/hg1700ag58.yaml \
  --upstream-root ../imu-error-model \
  --output artifacts/imu_navigation/profile_comparison_hg1700ag58.json
```

The command fails if the upstream checkout does not match
`resources/sensors/imu_profiles/catalog.json`. The artifact hashes both
profiles and the catalog, records the seed and common deterministic ECI truth,
and reports packet-rate and increment metrics. It is a model comparison, not a
vendor-performance or hardware-qualification claim.

The existing omega=0 Hummingbird/X8 family fixtures must not be edited to make
this pass. A nominal overlay is expected to fail closed when its aerodynamic
table envelope cannot support the induced transport velocity. The isolated
transport evidence is executable with:

```bash
python examples/taoryx/rotating_earth_sensor_demo.py \
  --output-dir artifacts/sensor-scenarios/rotating-earth-sensor-v1
```

Its artifact identifies ECI, nominal Earth rate, the transport-only truth
source, and `does-not-modify-omega-zero-family-fixtures`.

The expected fail-closed family check is:

```bash
python examples/taoryx/sensor_scenario_runner.py \
  hummingbird-hover-rotating-earth \
  --output-dir artifacts/sensor-scenarios/hummingbird-hover-rotating-earth-v1
```

Do not turn that failure into a pass by widening or mutating the source table;
create a dedicated Earth-rate table/initial-condition family when physical
rotating-Earth vehicle evidence is required.

### S12 - SWIL/HWIL rotational replay

`RotationalTruthReplay` consumes timestamped JSONL frames containing
`time_s`, `orientation_eci_from_body`, and `angular_rate_body_radps`.
`RotationalTruthRecorder` validates every returned timestamp and writes an
accepted/rejected request log. Missing, unsorted, or mis-timestamped frames
fail closed. The complete hybrid path is demonstrated with:

```bash
python examples/taoryx/rotational_truth_replay_demo.py \
  --replay examples/sensors/rotational_truth_replay_v1.jsonl \
  --output-dir artifacts/sensor-scenarios/rotational-truth-replay-v1
```

The replay fixture is a deterministic stand-in for a three-axis table. A real
HWIL adapter can replace the provider callback without changing the packet,
estimator, or artifact contracts. Translation remains simulated or explicitly
substituted and is recorded separately from rotational authority.

## Explicit Non-Goals

- No pad, touchdown, wheel, track, or contact IMU semantics in this tranche.
- No claim that an upstream hardware estimate is a vendor specification.
- No guidance/controller closure using hidden truth.
- No adaptive/rejected-step claim until sensor sampling is proven against those
  execution paths.
- No automatic profile discovery that makes a run irreproducible.

## Recommended First Work Package

Implement S1 and S2 together as `hummingbird_sensorized_hover_v1`. It is small
enough to debug quickly but difficult enough to expose frame, gravity, body
rate, Earth rotation, and estimator-isolation mistakes. Once that artifact is
correct, reuse the exact runner and sidecar for X8 rather than creating a
second notebook-specific integration.

## Completion Record

The shared runner now implements the recommended work package and the first
operational X8 gate:

- `hummingbird_sensorized_hover_v1.yaml` runs the rigid-body hover through the
  external `imu-error-model` profile, accepted-truth `SensorBus`, delivery
  latency, DR, MEKF, source hashes, JSONL records, and six Matplotlib plots.
- `hummingbird_sensorized_hover_ideal_v1.yaml` provides the explicit
  perfect-information comparison without making it the default provider.
- The rate-damped hover and disturbed-response cases use the same binding and
  complete without packet discontinuity errors.
- `x8_sensorized_powered_v1.yaml` runs the short source-trim powered 6-DOF
  gate through the same runner and artifact contract.
- `hummingbird_sensorized_translation_v1.yaml` runs the existing Hummingbird
  3-DOF point-mass case with acceleration-only ECI packets and a translation
  navigator; its artifact marks orientation and body rate unavailable.
- `hummingbird_sensorized_pseudo6dof_v1.yaml` runs the same 3-DOF case with a
  declared velocity-aligned, zero-speed-hold, parallel-transport attitude
  policy and supports DR plus MEKF as synthesized-truth evidence.
- `hummingbird_sensorized_rotation_v1.yaml` runs the rigid-body Hummingbird
  through an ideal gyro-only packet path and attitude dead reckoning; its
  artifact marks translation as not consumed and includes attitude evidence.
- `hybrid-6dof` composition accepts independent rotational truth for a table or
  HWIL adapter while retaining simulated or substituted ECI translation.
- Deterministic drop policies rebase packet-only estimators at the next
  accepted packet boundary and record the skipped interval. Step-limit
  termination records `timeout`, the last accepted truth time, and the next
  requested sensor time. Checkpoint loading records that the external provider
  and estimator must be explicitly rebound.
- `tools/compare_imu_profiles.py` verifies the catalog-pinned
  `imu-error-model` checkout and produces a baseline-versus-HG1700 packet
  comparison with profile and catalog hashes.
- `rotating_earth_sensor_demo.py` produces isolated nominal-Earth-rate ECI
  transport evidence without changing the source omega=0 family fixtures.
- `rotational_truth_replay_demo.py` proves timestamp-validated external
  rotation, hybrid sensorization, estimator isolation, and request logging.
- `verification/imu_sensor_free_flight_release.yaml` is the canonical inventory
  of these commands, expected artifacts, source provenance, and limitations.

The remaining limitation is deliberate and visible: the existing Hummingbird
and X8 family fixtures declare `omega=0` and their aerodynamic tables are
bounded around that baseline. An explicit `earth.rate_mode: nominal` or
`explicit` sidecar override is applied only to an in-memory copy and fails
closed if the unchanged table envelope cannot support it; the authoritative
family evidence therefore uses `earth.rate_mode: source` and labels the
source ECIC convention in the artifact. A physical Earth-rotating overlay
requires a dedicated table/initial-condition family rather than silently
changing these baselines. Translation-only 3-DOF sensorization is now
supported, but it is acceleration-only and does not claim a physical body
frame or gyro. Pseudo-6DOF sensorization is supported as an explicitly
synthesized velocity-aligned attitude policy; its DR/MEKF results are
comparison evidence, not rigid-body attitude truth.

The free-flight IMU tranche is complete when S0-S12 have passing focused tests,
the commands above produce manifests and plots/logs, and the full regression
remains green. Pad, touchdown, wheel, track, and support-force
semantics remain a separate follow-on milestone; they must introduce an
explicit support/contact truth provider before accelerometer measurements are
interpreted as ground reaction or vehicle support.
