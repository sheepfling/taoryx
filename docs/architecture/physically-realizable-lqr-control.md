# Physically realizable LQR control

An LQR that stabilizes an inertia-only moment bridge is a useful numerical
screen. It is not evidence that a vehicle can create those moments with its
real surfaces, rotors, wheels, gimbals, or thrusters. This note defines the
promotion path from that screen to a physically accountable controller.

## Control path and evidence boundary

The required path is:

```text
declared operating condition
        -> nonlinear trim with declared effectors
        -> plant-derived local linearization
        -> LQR or other regulator
        -> requested generalized wrench or declared direct-effector command
        -> constrained allocation
        -> actuator position/rate/lag dynamics
        -> nonlinear vehicle load evaluation
        -> requested-versus-achieved evidence
```

Direct body-force or body-moment injection is permitted only as
`direct_wrench_screen`. It remains a diagnostic for stabilizability, scaling,
and guidance wiring. It must not be presented as actuator realization,
nonlinear controller validation, or a qualified showcase result.

Taoryx records the following monotonic evidence tiers.

| Tier | Name | Minimum evidence |
| --- | --- | --- |
| T0 | structural | State, controls, units, frames, and model boundaries are declared consistently. |
| T1 | trimmed | A bounded equilibrium passes its declared residual contract using the model's actual declared effectors. |
| T2 | linearized | `A` and `B` are centered derivatives of that nonlinear plant at the trim; state/control order, steps, units, and a second-step comparison are retained. |
| T3 | linearly controlled | The regulator stabilizes the recorded linear model with declared scaling, poles, conditioning, and command demand. |
| T4 | physically allocated | A requested wrench is mapped through bounded effectors, with allocation residual, travel/rate/lag state, and actual nonlinear load evidence. |
| T5 | nonlinearly validated | The nonlinear plant recovers from declared perturbations without bypassing the effectors. |
| T6 | envelope validated | The same chain passes a scheduled set of operating points and schedule-transition cases. |

The typed realization contract rejects a direct-wrench or unconstrained
allocation path that attempts to claim T4 or above.

## The three control spaces

The controller and plant must not conflate these quantities:

```text
state/reference error
        -> controller output
        -> desired wrench w_d
        -> physical effector coordinates u
        -> actual nonlinear wrench w_a
```

For a regulator expressed in generalized body moments,

\[
u_c = -K\left(x-x_{\mathrm{trim}}\right),
\qquad
w_d = \mathcal{C}(u_c),
\]

where `\(\mathcal{C}\)` is explicit. A direct-effector LQR is also allowed,
but its controller definition must say that it bypasses wrench allocation; it
does not make a direct-moment bridge physical by itself.

Around the current physical operating point, an adapter supplies

\[
w \approx w_0 + G(x)\left(u-u_0\right).
\]

The common allocator solves the bounded weighted least-squares problem

\[
u^* = \arg\min_u
\left\|W_w\bigl(G(x)u-w_d\bigr)\right\|_2^2
+ \lambda\left\|W_u\bigl(u-u_{\mathrm{preferred}}\bigr)\right\|_2^2
\]

subject to position and, when declared, rate bounds:

\[
u_{\min}\le u\le u_{\max},
\qquad
-\dot u_{\max}\Delta t\le u-u_{k-1}\le\dot u_{\max}\Delta t.
\]

The result is never silently clipped. It records feasible, near-limit,
partially-achievable, infeasible, singular, or solver-failure status. Axes
that are deliberately not controlled are recorded separately from controlled
axes, so a flying wing's coupled yaw residual cannot disappear inside a
roll/pitch result.

Actual actuator positions advance as either declared ideal response or a
first-order lag with travel/rate limits. The family adapter then evaluates the
nonlinear plant at those **actual** positions. Thus `w_a` is not merely the
linearized `G u` prediction.

## Common implementation seam

`taoryx.control_allocation` supplies the reusable numerical layer:

- `EffectorLimits` for position, rate, response, and availability;
- `EffectorEffectiveness` for the affine local effectiveness contract;
- `allocate_bounded_weighted_wrench` and `advance_actuators`;
- `PhysicalAllocationStep`, including requested, predicted, and actual
  residuals;
- `DerivativeProvenance` and a two-step centered finite-difference check;
- `ControlPlantAdapter`, the family-specific plant contract.

Every physical adapter owns only vehicle-specific facts:

```text
state_derivative(state, effectors, environment)
trim(target, initial_guess)
linearize(trim, options)
effectiveness(state, effectors)
allocate(state, desired_wrench, previous_effectors, dt)
```

The shared allocator does not contain an elevator, rotor, wheel, or gimbal
sign convention. It treats those as declared adapter data.

The canonical allocation telemetry includes requested, predicted, and actual
wrench; controlled/uncontrolled axes; commanded and actual effector positions;
rates; position/rate limits; effectiveness matrix/rank; solver status;
allocation residual; and actual nonlinear residual. A report that only shows
surface motion is therefore insufficient.

## Current X8 vertical slice

The X8 is the first proof vehicle because its source deck exposes two useful
control coordinates and throttle. The new
`skywalker_x8_table_coordinate_trim_6dof.prb` anchor carries the accepted
source trim without any direct force, moment, or hold-law bridge. The local
runtime plant adapter then:

1. evaluates the same lowered aerodynamic and thrust tables used by runtime;
2. solves the declared axial/normal-force and three-moment source trim
   contract at the fixed source attitude/velocity condition;
3. computes a nine-state source-relative attitude, body-velocity, and
   body-rate `A/B` twice by centered finite differences;
4. derives a local table-effectiveness matrix from actual nonlinear moments;
5. projects the actual table-coordinate derivative into independently
   requested roll/pitch moment axes, then synthesizes a scaled LQR from that
   plant-derived matrix;
6. allocates requested roll/pitch moments through bounded collective and
   differential **source table coordinates**; and
7. evaluates the resulting nonlinear moment and nonlinear local recovery
   after actual coordinate lag/rate advancement.

The public X8 bundle explicitly says that the physical left/right elevon
differential sign mapping needs reconciliation before hardware use. Therefore
the present X8 result must say `collective/differential table-coordinate
allocation`, not `hardware-ready left/right elevon allocation`. The bundle
declares 20 degree travel, a 120 degree/s rate limit, and a 0.05 s
implementation first-order response for the table coordinates. Those values
are active in the local validation; their source classification remains
`PUBLISHED_LIMITS_WITH_IMPLEMENTATION_APPROXIMATION`, not a claim of an
identified production servo.

The X8 source table coordinates provide two independently allocated axes at
the local trim. Differential elevon also produces a coupled yaw moment. A
roll/pitch allocation can declare yaw unweighted, but the yaw residual remains
logged as an uncontrolled consequence; it is not claimed as controlled yaw.

The focused generator is:

```bash
PYTHONPATH=src python3 tools/validate_x8_physical_lqr.py
```

It writes `verification/generated/x8_table_coordinate_physical_lqr.json`.
That packet records the trim, derivative provenance, projected LQR, real
table-coordinate allocation, declared actuator behavior, requested versus
actual nonlinear moments, and a coupled roll/pitch recovery. The model-level
nonlinear evidence is retained, but the typed X8 controller remains at T3
until the left/right physical sign mapping is source-resolved. This avoids
promoting a model-coordinate result into a hardware-elevator claim.

The reusable X8 racetrack is intentionally a different evidence case. Its
scenario declares `surface-control-response=ideal-declared`, so it can show
bounded table-coordinate allocation in a mission integration run without
silently inheriting the source rate/lag evidence. The local tool above is the
only current X8 packet that activates the declared 120 deg/s and 0.05 s
source-coordinate actuator contract.

The existing runtime LQR remains T0 `direct_wrench_screen`: its default `B`
is still an inertia/direct-moment bridge and cannot inherit the table-derived
result merely because a later scenario uses the same vehicle.

## Current Hummingbird individual-rotor slice

The Hummingbird source package contains a stronger physical control contract
than the legacy rate-loop screen: four rotor locations, alternating yaw-torque
directions, the quadratic thrust and reaction-torque coefficients, frame and
rotor drag, translational lift, a 0--1500 rad/s motor range, and a 5 ms
first-order motor time constant. The source equations are retained in
`quadcopter_hummingbird/scripts/wrench_evaluator.py` and reimplemented in the
typed `QuadRotorAllocation.source_force_moment` adapter.

The physical local anchor is
`hummingbird_individual_rotor_hover_6dof.prb`. Its
`rotor-force-model=individual-rotor-source` declaration replaces the legacy
common-speed-table-plus-differential-moment shortcut for this proof only. At
every accepted controller sample:

1. the LQR requests roll, pitch, and yaw body moments;
2. bounded weighted allocation maps the request to all four motor-speed
   commands;
3. each actual motor speed advances through the documented 5 ms lag;
4. the source rotor equations evaluate individual thrust, local rotor drag,
   force-arm moment, reaction torque, and frame drag at those actual speeds;
5. the nonlinear rigid-body plant receives only those resulting loads.

There is no direct body-moment injection in this path. The artifact logs the
commanded and actual speed of each rotor, motor-lag activity, local
effectiveness matrix and rank, requested/predicted/actual wrench, and both
allocation and nonlinear achieved residuals. The initial lag transient is
visible as a requested-versus-achieved mismatch; it is not relabeled as a
saturation or hidden by plotting the commanded rotor values alone.

Run the proof with:

```bash
PYTHONPATH=src python3 tools/validate_hummingbird_physical_lqr.py
```

It writes `verification/generated/hummingbird_individual_rotor_physical_lqr.json`.
The packet earns T5 only for the declared local hover attitude/rate recovery.
It does **not** promote the existing waypoint, landing, or direct-wrench rate
screen: there is still no qualified position loop, ground/contact proof,
battery ledger, rotor blade/inflow model, or scheduled flight envelope.

Run the focused checks with:

```bash
PYTHONPATH=src python3 -m pytest \
  tests/unit/test_control_allocation.py \
  tests/unit/test_runtime_control_adapter.py \
  tests/unit/test_physical_lqr.py
```

## Current B747 condition-3 source-surface slice

The B747 now has one deliberately narrow physical-control proof at the NASA
CR-2144 condition-3 source anchor. The anchor
`b747_condition3_surface_trim_6dof.prb` declares the static, elevator,
aileron, rudder, and installed-JT9D thrust tables. The aerodynamic runtime
composes the absolute source control-conditioned decks as each current minus
zero-control increment about the static deck, so it does not double-count
absolute control data.

The local adapter uses the same lowered nonlinear force and moment closure as
the runtime. It first solves the fixed-condition force/moment trim through
elevator, aileron, rudder, and throttle. It then derives the full nine-state
source-relative attitude, body-velocity, and body-rate `A/B` matrices twice
by centered differences. Throttle participates in the trim and plant
derivative, while the local LQR requests roll, pitch, and yaw body moments;
the bounded allocator maps those requests only through elevator, aileron, and
rudder. The installed-engine table receives current altitude, airspeed, and
Mach from the same truth-state aerodynamic evaluation; no constant-thrust
substitute is used in this proof.

The generic tuning utility evaluates normalized gentle, standard, and
aggressive profiles against fixed nonlinear acceptance gates. The complete
artifact retains each candidate, including rejected candidates, rather than
quietly preserving only the selected gain. This matters for a transport-scale
local mode: raw mixed-unit state norms are diagnostic only; the acceptance
test uses the declared LQR state scales.

The selected controller earns T5 for the one declared local recovery case:
one-degree roll/yaw, half-degree pitch, and half-degree-per-second body-rate
perturbations. Requested moments are allocated through bounded physical
surface coordinates, the nonlinear plant evaluates the achieved moments, and
the artifact exposes the brief partially-achievable/saturated interval instead
of relabeling it as exact authority.

Run it with:

```bash
PYTHONPATH=src python3 tools/validate_b747_physical_lqr.py
```

It writes `verification/generated/b747_condition3_physical_surface_lqr.json`.
The result does **not** claim a B747 mission controller, gain scheduling,
source servo rate/lag dynamics, high-lift/approach control, fuel flow,
engine-spool dynamics, engine-out behavior, or a broader transport envelope.
The older direct-wrench racetrack remains T0 integration/screen evidence and
cannot inherit this T5 local source-surface result.

## Current X-15 physical-control readiness gate

The X-15 deliberately stops before LQR synthesis. The source aerodynamic
package supplies symmetric stabilator, differential stabilator, and rudder
coordinates, but a physical controller requires an actual equilibrium of the
same nonlinear plant before it can claim a plant-derived `A/B` model.

`validate_x15_physical_lqr.py` regenerates and records two bounded attempts:

1. an unpowered source-release directional-glide trim; and
2. a frozen-time full-thrust attempt at the start of the XLR99 history.

Both fail their declared residual contracts. The latter cannot be repaired by
calling the grammar-facing `throttle` a physical command: the available XLR99
deck is a time-indexed full-thrust/mass-flow history, not an altitude/Mach/
throttle engine map. In addition, the source package explicitly leaves the
stabilator left/right gearing unresolved, supplies no RCS geometry or impulse,
and omits hard rate limits for differential stabilator and rudder.

The generated `verification/generated/x15_physical_lqr_readiness.json` is
therefore a **successful blocked prerequisite** with T0 structural evidence.
It records `blocked_before_T1_trim`, sets direct body-moment injection to
false, and declares that LQR synthesis, allocation, and nonlinear controller
validation were not attempted. This is the desired behavior: no control tool
should turn a missing source trim into a direct-wrench fallback.

Run it with:

```bash
PYTHONPATH=src python3 tools/validate_x15_physical_lqr.py
```

## Follow-on vehicle order

1. **X8:** preserve the local source-coordinate result at T3 until the public
   left/right elevon sign mapping is resolved; then promote it through a true
   physical-elevon proof before broad scheduling.
2. **Hummingbird:** carry the local individual-rotor T5 proof upward into
   position, yaw, waypoint, disturbance, and landing/contact loops without
   reintroducing a direct-wrench bridge.
3. **B747:** extend the current condition-3 T5 source-surface local proof to
   adjacent cruise and descent conditions, then validate schedule transitions
   before claiming any transport envelope.
4. **X-15:** remain at T0 structural evidence until an actual-effectors,
   source-bounded trim/reference point exists. The release-glide directional
   attempt and a frozen full-thrust `t=0` attempt both currently fail their
   declared residual contracts. The XLR99 source deck is a time-indexed
   full-thrust/mass-flow history—not a throttle map—and the package lacks RCS
   geometry/impulse data, a reconciled left/right stabilator mapping, and
   hard rate limits for two surface coordinates. `validate_x15_physical_lqr.py`
   records this as `blocked_before_T1_trim`; it does not synthesize an LQR,
   allocate a wrench, or use a direct-moment substitute. Only after a valid
   trim/reference and the missing actuator data are supplied may the work
   proceed to state-dependent effectiveness and low-authority/blended-control
   transitions.

At no point may a lower-tier direct wrench screen be used to rescue a failed
physical allocation or nonlinear validation case.
