# Slower-vehicle 3-DOF/6-DOF robustness tranche

This tranche introduces three coherent public research surrogates without
claiming flight qualification or historical TAOS compatibility:

| Vehicle | Primary use | Deliberate boundary |
|---|---|---|
| Boeing 747-100 | subsonic jet trim, thrust lapse, control authority | local handling-qualities derivatives; no stall/global envelope |
| Skywalker X8 | powered fixed-wing trim, wind, elevon coupling | flight-identified near-trim model; no separated-flow claim |
| AscTec Hummingbird | hover, rotor wrench, allocation and saturation | tuned research rotor model; not a hardware certification |

The original CSV, workbook, notices, validation report, and SHA-256 records
are preserved under `tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1`.
`tools/import_slower_6dof_tables.py` generates the parser-validated TAORYX
tables. Large control grids remain source CSV and can be promoted later with
the same generator once a scenario needs them.

## Scenario ladder

Every family gets a 3-DOF route/trim baseline first. A corresponding 6-DOF
case then consumes the same initial condition, atmosphere, propulsion table,
and route duration while adding attitude, moments, and actuator state.

| ID | Family | 3-DOF baseline | 6-DOF follow-up | Evidence |
|---|---|---|---|---|
| SV01 | B747 | trim and level acceleration across two Mach/altitude points | elevator capture, thrust lapse, and bounded pitch response | static table bounds, thrust monotonicity, finite state |
| SV02 | B747 | climb/descent route with configuration switch | coupled elevator/aileron/rudder disturbance recovery | control-envelope diagnostics and rate bounds |
| SV03 | X8 | powered trim and short loiter leg | wind-coupled elevon response and saturation | trim residual, cross-track bound, control trace |
| SV04 | X8 | throttle step and glide segment | propeller thrust plus roll/yaw coupling | thrust monotonicity, energy trend, quaternion norm |
| SV05 | Hummingbird | altitude-hold kinematic route | hover, lateral translation, yaw step, and wind | hover force balance, wrench sign, allocation residual |
| SV06 | Hummingbird | racetrack ground-track scaffold | motor lag and rotor-speed saturation | lap closure, actuator state, bounded attitude |

The first implementation slice promotes the data and table validation for all
three families. SV01, SV03, and SV05 are the recommended next executable
scenarios because they test distinct force/moment pathways with the least
interpretive risk. They must use standard `.prb` files and existing `*aero`,
`*when`, `*fly`, and runtime-control constructs; no vehicle-specific grammar
directive is needed.

## Trajectory-example delivery plan

The trajectory examples are paired experiments, not independent showcases.
The 3-DOF and 6-DOF files for a pair must share the same initial position,
velocity, mass, atmosphere, propulsion schedule, duration, and commanded
inputs. The 6-DOF version adds attitude, moments, actuator state, and the
vehicle-specific coefficient deck. This makes the difference between the two
models reviewable instead of comparing unrelated missions.

Each pair is delivered as a self-contained directory:

```text
examples/mission_families/<family>/
    README.md                 # purpose, source boundary, expected behavior
    3dof.prb                  # point-mass route/trim baseline
    6dof.prb                  # native rigid-body trajectory
    tables.tbl                # only when a small local table is needed
    expected.yaml             # tolerances and qualitative invariants
    plot.yaml                 # common comparison plots
```

The large vehicle decks remain under `tests/fixtures/.../tables/`; problem
files refer to them through the existing table-ingestion/lowering path. A
scenario must not copy or silently alter a source-derived table.

### B747-100: trim, climb, and disturbance recovery

Pair `SV01` first. Start at the supplied nominal reference condition, run a
short level segment, apply a documented throttle step, and then command a
small climb. Use only the local B747 envelope represented by the selected
reference condition; do not seek stall, landing flare, or a global transport
model claim.

- 3-DOF: verify thrust-lapse interpolation, acceleration sign, altitude trend,
  and trim persistence.
- 6-DOF: initialize the same trim with a consistent quaternion, apply the same
  throttle and elevator commands, and verify bounded pitch response and finite
  aerodynamic moments.
- Follow-up `SV02`: add a small lateral disturbance and recover with the
  existing aileron/rudder controls; this promotes the source control grids only
  when the baseline is stable.
- Plots: altitude, Mach, alpha/beta, throttle, elevator, body rates, six
  coefficients, forces, moments, and 3-DOF/6-DOF altitude and speed overlays.

### Skywalker X8: powered trim, wind, and glide

Pair `SV03` next. Use the published near-trim region as the initial condition,
run a powered straight leg, introduce a bounded steady wind, and finish with a
short loiter or glide segment. Keep alpha, beta, velocity, and control inputs
inside the supplied identification envelope.

- 3-DOF: establish powered trim, throttle response, wind-relative velocity,
  and short-leg cross-track behavior.
- 6-DOF: use the same route and throttle schedule with elevon control, then
  verify roll/pitch/yaw coupling, actuator limits, and quaternion norm.
- Follow-up `SV04`: execute a throttle step into a glide and compare energy
  decay, thrust, and control saturation. The differential-elevon sign mapping
  remains a recorded verification item until independently adjudicated.
- Plots: ground track, altitude, airspeed, alpha/beta, throttle, collective and
  differential elevon, body rates, six coefficients, force/moment components,
  and energy overlays.

### AscTec Hummingbird: hover, translation, and racetrack

Pair `SV05` first. Start from the supplied ideal hover speed, hold altitude,
command a small lateral translation, and return to hover. Wind is introduced
only after the no-wind wrench balance passes.

- 3-DOF: verify hover force balance, commanded translation, altitude hold, and
  route closure using the existing point-mass/kinematic path.
- 6-DOF: consume the rotor static/wrench tables, add motor lag and attitude
  state, and verify force/moment signs, allocation residual, and bounded rotor
  speeds.
- Follow-up `SV06`: run a two-lap racetrack with a yaw step, crosswind, and
  actuator saturation. This is an opt-in slow/artifact test, not part of the
  default fast loop.
- Plots: ground track, altitude, rotor speeds, collective thrust, force and
  moment components, Euler/quaternion attitude view, and saturation events.

### Shared execution and evidence order

For every pair, the implementation sequence is:

1. Parse the `.prb` and all referenced tables, preserving source locations.
2. Run the 3-DOF case and record its initial/final state and route metrics.
3. Run the 6-DOF case with the same scenario inputs.
4. Compare only common observables first: time, position, velocity, mass,
   altitude, and route completion.
5. Inspect 6-DOF-only observables: attitude, body rates, forces, moments,
   coefficients, actuator states, and saturation.
6. Generate the paired text/JSON result and opt-in plots under the ignored
   `artifacts/` directory.

The acceptance record for each pair must include the source fixture ID,
table-catalog entries, problem-file hashes, solver settings, tolerance units,
3-DOF result, 6-DOF result, and any out-of-envelope diagnostic. A trajectory
that completes numerically outside its source envelope is useful as a stress
test, but it is not evidence of model validity.

### Current implementation status

The first three paired smoke trajectories are now executable:

| Pair | Problem files | Table connection | Status |
|---|---|---|---|
| SV01 | `examples/mission_families/slower_b747/SV01_{3dof,6dof}.prb` | B747 nominal six-axis deck | passing |
| SV03 | `examples/mission_families/slower_x8/SV03_{3dof,6dof}.prb` | X8 zero-control six-axis deck | passing |
| SV05 | `examples/mission_families/slower_hummingbird/SV05_{3dof,6dof}.prb` | Hummingbird direct-wrench deck | passing |

The pair manifest is
`examples/mission_families/slower_vehicle_trajectory_matrix.yaml`, and the
executable assertions are in
`tests/e2e/test_slower_vehicle_trajectories.py`. These are bounded trajectory
smoke tests, not complete trim or route-validation cases. SV02, SV04, and SV06
remain the next expansion tranche for lateral disturbance recovery, wind/glide
behavior, and racetrack/actuator saturation respectively.

### Planned marker and view assignment

The paired tests belong to the overlapping `dof3` and `dof6` views. B747 and
X8 baseline pairs are ordinary table/runtime tests when short; Hummingbird
racetrack and all plot-producing comparisons are marked `slow` and/or
`artifact`. They are not `simple_aero` tests unless they are deliberately added to
the Simple Aero problem/segment corpus.

## Acceptance gates

1. **Data gate:** source validation passes, generated tables parse without
   errors, and every generated table is prepared by the table explorer.
2. **3-DOF gate:** route or trim completes with finite state and documented
   tolerances before any 6-DOF result is interpreted.
3. **6-DOF gate:** quaternion normalization, force/moment signs, actuator
   bounds, and comparison-to-3-DOF evidence pass for the same case.
4. **Claim gate:** the result is labeled as a public research surrogate; no
   global aerodynamic, hardware, or historical-compatibility claim is inferred.

Artifact plots are opt-in with `pytest -m artifact`; ordinary table and source
validation stays in the default fast test set.
