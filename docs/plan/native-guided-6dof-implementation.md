# Native TAORYX guided 6-DOF implementation plan

## Scope and claim boundary

This plan builds a TAORYX successor capability: a force- and moment-integrated
rigid-body vehicle that can use propulsion, atmosphere, aerodynamic tables,
guidance, and attitude control in one runtime graph.

It does not change the historical TAOS 96.0 claim. The TAOS-compatible mode
remains the documented three-degree-of-freedom point-mass model. The native
6-DOF mode is an explicit TAORYX extension selected by `*6dof` under the
`taoryx` grammar profile.

The California–Hawaii scenario is a migration and verification fixture. It is
not a historical TAOS case and its invented vehicle data is not an engineering
validity claim.

## Target execution graph

```text
problem/document profile
        ↓
typed vehicle, stage, table, and controller configuration
        ↓
RuntimeProblem / RuntimeVehicle
        ↓
state → environment → guidance demand → control allocation
        ↓                              ↓
 atmosphere, wind, aero tables       desired attitude / rates
        ↓                              ↓
 body forces and moments  ← attitude controller
        ↓
 ECIC rigid-body equations and mass/thermal rates
        ↓
 normalized run artifact, diagnostics, and plots
```

The integration state remains ECIC position and inertial velocity, quaternion
attitude, body rates, mass, propellant mass, heat load, and peak heat rate.
Earth-fixed atmosphere and wind evaluation must cross the explicit
`EarthRotationAdapter` boundary before loads are formed.

## Phase 1 — Freeze the contracts

Define typed contracts before adding more vehicle-specific logic:

- `MassProperties`: mass, dry mass, center of gravity, inertia tensor, and
  reference points;
- `PropulsionOutput`: body thrust vector, body moment, positive propellant
  flow rate, exhaust metadata, and active stage;
- `AerodynamicOutput`: body force, body moment, coefficients, dynamic
  pressure, angle branches, and table provenance;
- `GuidanceDemand`: desired ECIC acceleration, desired velocity direction,
  terminal target data, and limiting status;
- `AttitudeCommand`: desired attitude, body-rate command, angle of attack,
  bank, and command limits;
- `ControlOutput`: actuator commands, achieved body loads, and saturation;
- `RuntimeDiagnostics`: source IDs, table envelope status, solver status, and
  unresolved ambiguity IDs.

Every provider must be deterministic for a state and immutable configuration.
No provider may mutate mass, attitude, or other state as a side effect.

Acceptance tests:

- contracts are slotted and type checked;
- units and sign conventions are explicit;
- body/ECIC/ECFC frame conversions have round-trip tests;
- unavailable data produces a structured diagnostic rather than a fallback.

## Phase 2 — Mass properties, propulsion, and staging

Replace the minimal rigid-body `*prop` reader with a native vehicle/stage
configuration that can still ingest ordinary TAOS propulsion tables.

Required behavior:

- thrust and mass flow may be constants, expressions, or resolved tables;
- positive `mdot` means propellant consumption and produces negative mass
  derivatives;
- total mass cannot fall below dry mass;
- propellant mass cannot become negative;
- thrust shuts down at depletion unless an explicit dry-fire policy exists;
- stage separation is an event with a mass-property discontinuity;
- center of gravity and inertia may change at stage transitions;
- thrust-vector direction and application point produce body moments;
- mass-flow and thrust telemetry are recorded at every accepted state.

The rigid-body derivative should use a single `MassProperties` snapshot for
each force evaluation. It must not independently reconstruct mass or inertia
from unrelated named variables.

Acceptance tests:

- analytical constant-`mdot` mass benchmark;
- dry-mass and propellant clamps;
- two-stage ignition, burnout, and separation;
- mass and inertia discontinuity event trace;
- thrust-vector moment sign test;
- RK4 mass-flow convergence test.

## Phase 3 — Native environment and aerodynamic loads

Create a rigid-body aerodynamic provider using the existing environment and
table infrastructure.

The provider must:

1. Convert ECIC inertial state to Earth-fixed position and air-relative
   velocity.
2. Evaluate atmosphere, wind, density, sound speed, Mach, dynamic pressure,
   angle of attack, and sideslip using the documented frame conventions.
3. Resolve `CA/CD/CL/CN/CS` or equivalent table families with explicit
   interpolation and extrapolation policy.
4. Apply reference area, reference length, center-of-pressure, and center of
   gravity to produce body forces and moments.
5. Apply control-effector coefficient increments rather than bypassing the
   aerodynamic model with arbitrary force commands.
6. Return table envelope status and source provenance in telemetry.

Analytical drag-table generators remain valid as data preparation tools, but
their outputs must enter through the same prepared-table/provider interface as
source-faithful `.tbl` data.

Acceptance tests:

- zero-wind stationary-Earth air-relative velocity;
- density, Mach, and dynamic-pressure reference points;
- coefficient-table node and interpolation round trips;
- positive drag and lift direction tests;
- alpha/beta branch-boundary tests;
- aerodynamic force/moment finite-difference checks;
- table extrapolation diagnostics;
- analytical-table and serialized-`.tbl` equivalence.

## Phase 4 — Guidance demand and control allocation

Use the canonical guidance algorithms as demand generators, not as direct
force injectors.

### ProNav

The guidance layer calls `proportional_navigation()` with interceptor and
target states. Its ECIC acceleration demand is recorded with:

- navigation constant;
- closure velocity;
- line-of-sight rates;
- pitch/yaw demand components;
- requested acceleration vector;
- saturation and infeasibility status.

Target motion and target-frame conventions must be explicit. A custom
showcase-specific line-of-sight formula is not an acceptable substitute for
the canonical algorithm in the migrated scenario.

### Acceleration allocation

Convert the requested acceleration into achievable vehicle commands:

- separate gravity-compensation, thrust-direction, lift, drag, and lateral
  components;
- solve for angle of attack and bank using current dynamic pressure and
  aerodynamic tables;
- respect alpha, beta, bank, dynamic-pressure, heating, and actuator limits;
- report the residual between demanded and achievable acceleration;
- handle infeasible or low-dynamic-pressure conditions explicitly.

### Attitude control

Implement a bounded controller that turns the allocated orientation into body
rate and moment commands. The controller must include:

- quaternion attitude error;
- body-rate feedback;
- inertia-aware moment demand;
- actuator rate and deflection limits;
- control saturation telemetry;
- optional thermal-entry command limiting.

The rigid-body model then integrates the resulting moments. Roll and pitch
must come from actual attitude state, not from a plot-only reconstruction of
velocity heading.

Acceptance tests:

- stationary target and closing-target ProNav benchmarks;
- canonical ProNav output versus independent implementation;
- demanded-versus-achieved acceleration residuals;
- bank reversal and alpha limit behavior;
- quaternion tracking and rate damping;
- torque sign and inertia scaling;
- controller saturation and low-q fallback;
- thermal limiter precedence.

## Phase 5 — Native language lowering

Extend the TAORYX profile without changing TAOS 96 parsing behavior.

The first native configuration should be explicit and typed, for example:

```text
*6dof
*vehicle ...
*mass-properties ...
*propulsion ...
*aerodynamics ...
*guidance propnav ...
*attitude-controller ...
```

The exact syntax requires a grammar contract, source-page/extension
provenance, diagnostics, and lossless recovery tests. Lowering should produce
the provider graph described above, not a second private physics path.

Required lowering rules:

- all referenced tables resolve before runtime execution;
- stage and controller IDs are unique;
- required frame and unit declarations are present;
- unsupported combinations are rejected with source locations;
- defaults are explicit in the resolved runtime manifest;
- every lowered provider retains source and provenance IDs.

The current minimal `*6dof` form remains a compatibility bridge until this
typed vehicle configuration exists.

## Phase 6 — Migrate California–Hawaii

Convert the showcase in this order:

1. Preserve the existing expected artifact contract.
2. Replace its synthetic direct control acceleration with canonical ProNav.
3. Introduce a synthetic but serialized aerodynamic coefficient deck.
4. Add stage propulsion and mass-property manifests.
5. Add acceleration allocation and attitude control.
6. Run through `parse → ingest → lower → RuntimeProblem → runtime.engine`.
7. Publish demanded/achieved acceleration, alpha, beta, bank, moments,
   saturation, and table-envelope channels.
8. Retain route, altitude, heating, forces, torques, and orientation plots.

The scenario acceptance oracle should include:

- both stage transitions;
- dry-mass and propellant conservation;
- exoatmospheric interval;
- hypersonic glide envelope;
- bounded heat rate and heat load;
- ProNav activation and target motion;
- demanded/achieved acceleration residual;
- final island-hit tolerance;
- no table-envelope or controller-saturation violations unless expected.

## Phase 7 — Verification and release gates

Add independent verification layers:

- equation and algorithm catalog links for every canonical algorithm;
- independent ProNav and rigid-body reference calculations;
- property tests for conservation and quaternion normalization;
- mutation tests for mass-flow signs, frame transposes, ProNav signs, and
  alpha/bank limits;
- artifact tests for JSON, SQLite, text, and plots;
- traceability report from syntax and requirements to runtime channels/tests.

Stop-ship conditions:

- direct force injection bypasses the aerodynamic/control chain;
- mass falls below dry mass or propellant becomes negative;
- ProNav is claimed while a custom unverified formula is used;
- roll/pitch plots do not represent integrated attitude;
- coefficient tables are used without frame, units, or envelope metadata;
- historical TAOS compatibility is claimed for native 6-DOF behavior.

## Deliverable order

The implementation should proceed as these reviewable increments:

1. Contracts and reference fixtures.
2. Mass/propulsion/staging provider.
3. Rigid-body aerodynamic provider with serialized synthetic tables.
4. Canonical ProNav demand provider.
5. Acceleration allocator and attitude controller.
6. Typed TAORYX lowering.
7. California–Hawaii migration.
8. Verification baseline and release artifact.
