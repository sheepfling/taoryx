# LQR extension

TAORYX provides a declarative LQR configuration and a numerical solver. The
problem file names the state/control contract and the sources of the matrices;
prepared `output` tables may supply flattened `A`, `B`, `Q`, and `R` values. A
six-value `Q` or three-value `R` table is interpreted as a diagonal matrix;
full matrices use row-major values.

```text
*runtime lqr attitude
  states=alpha,beta,p,q,r
  controls=fin-pitch,fin-yaw
  linearization=vehicle-trim
  a-table=attitude-a
  b-table=attitude-b
  q-table=attitude-q
  r-table=control-r
  method=continuous
  update=segment
```

Inspect the declaration before execution:

```python
program.inspect_lqr()
```

Solve a supplied linearization through the runtime API:

```python
from taoryx.runtime import LqrController, solve_continuous_lqr

result = solve_continuous_lqr(A, B, Q, R,
                              state_names=("alpha", "q"),
                              control_names=("elevator",))
controller = LqrController(
    result,
    state_trim={"alpha": 0.0, "q": 0.0},
    lower={"elevator": -0.4},
    upper={"elevator": 0.4},
)
command = controller.command({"alpha": alpha, "q": q})
u = command.controls["elevator"]
```

The solver validates dimensions, finite values, symmetry and definiteness of
`Q` and `R`, controllability, and reports closed-loop eigenvalues. Matrix
inversion is intentionally not exposed; the implementation uses linear solves.
`LqrController` maps named runtime state values to named, optionally bounded
commands and reports which controls saturated.

This page is the operational LQR reference: it owns exact syntax, API names,
telemetry channels, and copyable commands. The companion
`docs/latex/taoryx_extensions_and_verification.tex` document explains the
design rationale, verification sequence, and claim boundaries. The composite
PDF packages both documents.

For a rigid-body vehicle, a problem may opt into dimensionless design scales:

```text
*runtime lqr attitude states=attitude-error-x,attitude-error-y,attitude-error-z,wx,wy,wz controls=moment-x,moment-y,moment-z q-angle=1 q-rate=1 r-moment=1 state-angle-scale-rad=0.1745329252 state-rate-scale-rad-s=0.5 control-moment-scale-nm=20
```

The runtime solves the normalized system and maps the gain back to radians,
radians/second, and N·m before applying it. The three weight styles for the
four standard vehicle families live in `verification/lqr_scaling_profiles.yaml`.
Their dimensionful scales resolve from the canonical vehicle registry and
source table axes when the profile is loaded: attitude scales use alpha/beta
limits, rate scales use actuator or rate-table limits, and moment scales use
the registered mass, reference length, and principal inertia. The resolved
contract retains provenance for each choice. A problem can select a style
directly:

```text
*runtime lqr attitude profile=b747-gentle ...
*runtime lqr attitude profile=b747-standard ...
*runtime lqr attitude profile=b747-aggressive ...
```

`gentle`, `standard`, and `aggressive` are controller-design starting points,
not guarantees of stability or handling quality. The runtime still rejects a
non-Hurwitz design, and the vehicle harness must still check saturation,
table margins, and source-specific limits.

## Changing mass and inertia

The direct-moment attitude LQR uses inertia in the rotational input matrix;
mass is not silently converted into inertia. The shared vehicle contract also
exposes a nominal mass and derives dimensionful conditioning scales:

```text
mass scale                 = nominal mass
force scale                = nominal mass * 9.80665 m/s^2
weight-moment scale        = force scale * reference length
inertia-moment scale       = max(principal inertia) * rate scale / response time
control-moment scale       = max(weight-moment scale, inertia-moment scale)
```

The actuator's maximum moment remains a saturation limit and is reported
separately; it is not silently substituted for the plant's mass/inertia scale.
The auto-tune report records all five scales and provenance for each vehicle.

A vehicle may declare an explicit mass-property schedule when its source
provides dry-mass endpoints:

```text
*runtime status vehicle inertia-mass-model=linear-dry-mass dry-mass-kg=100 inertia-x=12 inertia-y=18 inertia-z=25 inertia-x-dry=9 inertia-y-dry=14 inertia-z-dry=20
*runtime lqr attitude update=mass ...
```

The runtime linearly interpolates between the declared reference and dry-mass
inertias, clamps outside the declared mass interval, and rebuilds the cached
LQR when the operating point changes beyond its tolerance. When a catalog
profile declares `mass-scaling=nominal-ratio`, the normalized control-moment
scale is adjusted by current mass divided by nominal mass; the rotational
input matrix still uses the independently supplied current inertia. This is a
conditioning rule, not an inferred inertia law. The interpolation is an
explicit approximation—not a claim that inertia scales with mass in general.
If tank, payload, or stage geometry is available, provide a more specific
source-backed schedule instead.

The current mass and inertia used by the plant are published as
`mass_kg`, `propellant_mass_kg`, and `inertia_{x,y,z}_kg_m2` telemetry.

For a real plant, build the matrices with
`finite_difference_dynamics_linearization`. It requires the evaluator to
return named state derivatives, producing a `DynamicsLinearization` artifact
that can be passed to `auto_tune_lqr_profiles(linearization=...)`. The older
residual Jacobian helper remains useful for trim diagnostics but must not be
passed to LQR as though force/moment residuals were state derivatives.

## Estimated aerodynamic data

Aerodynamic coefficients and derivatives are model estimates unless their
source contract explicitly says otherwise. A scenario may declare a bounded
model uncertainty and source-quality label:

```text
*runtime status vehicle aero-uncertainty-fraction=0.15 aero-source-quality=estimated
*runtime lqr attitude a-uncertainty-fraction=0.15 b-uncertainty-fraction=0.20 uncertainty-samples=25 uncertainty-policy=fail-closed ...
```

For a supplied source-trim `A/B` pair, the runtime screens the fixed LQR gain
against deterministic bounded derivative perturbations. A fail-closed policy
rejects a candidate whose sampled closed-loop poles become unstable. This is a
robustness screen, not a formal proof or a confidence interval.

Runtime artifacts expose `aero_model_uncertainty_fraction`,
`aero_model_estimated`, and `aero_table_operational_margin`. The operational
table margin subtracts the declared uncertainty from the normalized lookup
margin, so a controller can leave reserve before reaching the hard table
boundary. Do not invent an uncertainty value merely to make a mission pass;
record its source or engineering-assumption basis in the scenario manifest.

## Controller profiles and table validity

Profiles tune controller response; they never expand the vehicle model's
validity envelope. Coefficient tables remain strict: an out-of-range query is
rejected rather than extrapolated. This is the absolute validity boundary for
the loaded table deck.

Rigid-body telemetry exposes both the per-axis table margins and the minimum
dimensionless margin across all queried force/moment tables:

```text
aero_table_valid
aero_table_min_normalized_margin
aero_table_normalized_margin_<table>.<axis>
```

The normalized margin is the distance to the nearest endpoint divided by the
declared axis span. A value of `0` is on the boundary; a value below a chosen
operational reserve (for example `0.10`) should cause a controller to soften,
hold, or exit the maneuver before the hard lookup boundary is reached.
`aero_table_margin_*` channels retain the same distance in the source axis
units for diagnostics. These are validity/proximity signals, not additional
aerodynamic semantics.

This is a TAORYX extension, not historical TAOS syntax. Automatic trim,
source-plant linearization, and online Riccati updates remain separate runtime
features. Mass-property scheduling and uncertainty screening are explicit
TAORYX extensions, not historical TAOS behavior.

## Generic profile synthesis

The standard vehicle registry exposes a common profile-synthesis entry point:

```text
python tools/auto_tune_controllers.py --vehicle all
```

The command writes one JSON report per registered vehicle under
`artifacts/controller_autotune/`. It resolves the vehicle's table limits,
nominal mass, reference geometry, inertia, and controller response fallback
into the same dimensionful scale contract used by runtime LQR. It then evaluates the
`gentle`, `standard`, and `aggressive` profiles against a common
attitude-error/moment bridge.

The default report is deliberately labelled
`runtime-inertia-attitude-bridge` and `synthesis-only`. That bridge proves
matrix conditioning and closed-loop pole placement across the catalog; it does
not prove a source model, a trim, table margins during a maneuver, or waypoint
tracking. A vehicle-family adapter should pass the source-trim `A`/`B`
linearization and an evaluator returning metrics such as
`table_margin_min_normalized`, `control_saturation_fraction`,
`control_rate_abs_max`, and `tracking_error` before promoting a candidate into
mission evidence. The autotuner fails closed when those metrics violate their
declared limits, so a stable pole set cannot hide table extrapolation or
actuator saturation.

This keeps the workflow generic for new vehicles: register the physical
contract and table bindings, solve or import a source-backed trim, provide one
local derivative adapter, and reuse the same profile sweep and score schema.

## Plant-bound trim

Use `taoryx.trim.TrimSpec` and `solve_trim` to make the operating point an
explicit, reusable artifact instead of embedding it in a vehicle runner:

For new vehicles, start with [verification/trim_specs.yaml](/Users/rick/LocalStorage/GIT_LOCAL/active/taoryx/verification/trim_specs.yaml). The catalog generates the `TrimSpec`; only the source-backed residual adapter remains Python.

```python
from taoryx.trim import TrimSpec, solve_trim

spec = TrimSpec(
    state_names=("alpha", "beta", "q"),
    control_names=("elevator", "rudder"),
    residual_names=("ax", "ay", "az", "pitch_accel"),
    state_initial={"alpha": 0.05, "beta": 0.0, "q": 0.0},
    control_initial={"elevator": 0.0, "rudder": 0.0},
    state_lower={"alpha": -0.1, "beta": -0.1, "q": -1.0},
    state_upper={"alpha": 0.2, "beta": 0.1, "q": 1.0},
    control_lower={"elevator": -0.4, "rudder": -0.3},
    control_upper={"elevator": 0.4, "rudder": 0.3},
    residual_scales={"ax": 10.0, "ay": 10.0, "az": 10.0, "pitch_accel": 1.0},
)

result = solve_trim(spec, plant_residual)
assert result.success
```

`plant_residual(state, controls)` is the only vehicle-specific part. It must
evaluate the same source tables, mass properties, frames, actuators, and
propulsion used during propagation. The result records named solved state and
controls, unscaled residuals, bounds/solver status, and iteration count.

Use a tight `residual_tolerance` for optimizer convergence and a separately
declared `acceptance_tolerance` when source-table digitization or published
coefficients impose a larger evidence bound. Do not hide that distinction in a
vehicle-specific solver.

`finite_difference_dynamics_linearization` produces the local state-derivative
Jacobian used by LQR. Its evaluator must return derivatives named exactly like
the trim states. The older `finite_difference_linearization` helper remains
available for residual diagnostics; a force/moment Jacobian is not
automatically an `A,B` dynamics pair.

## Body-frame trim equations and linearization

For a full rigid-body trim, resolve every load about the center of gravity and
in body axes. With body velocity `v_B = [u, v, w]`, body rate
`omega_B = [p, q, r]`, and inertia tensor `I`, the residual adapter evaluates:

```text
m * (v_dot_B + omega_B x v_B) = F_aero_B + F_thrust_B + F_gravity_B
I * omega_dot_B + omega_B x (I * omega_B) = M_aero_B + M_thrust_B + M_control_B
```

The residuals are left side minus right side. For straight, level, constant-
speed flight with zero rates, they reduce to total force and total moment equal
to zero. A coordinated turn, hover, or powered climb retains its target
acceleration and rate terms; do not force every derivative to zero by default.

The adapter should assemble the loads in this order:

1. Compute relative air velocity from vehicle velocity and wind.
2. Derive the source-defined `alpha`, `beta`, Mach, and dynamic pressure.
3. Query coefficient tables and retain interpolation brackets and table margins.
4. Dimensionalize and transform aerodynamic force/moment into body axes.
5. Add gravity, propulsion, actuator maps, and moment transfers about the CG.
6. Apply achieved actuator values, not merely requested commands.
7. Return named SI derivatives such as `u-dot`, `v-dot`, `w-dot`, `p-dot`,
   `q-dot`, and `r-dot`.

For example, with `m=1200 kg`, `V=60 m/s`, `rho=1.225 kg/m^3`, and
`S=16 m^2`, the dynamic pressure is `2205 Pa`. Straight-level flight requires
approximately `C_L = 1200*9.81/(2205*16) = 0.334`. That is a dimensionalization
sanity check, not the trim result. The full residual still includes drag,
thrust direction, gravity projection, moment reference transfer, and control
effects.

For each accepted trim, freeze the complete context before linearizing:
mass, CG, inertia, atmosphere, wind, table hashes, actuator state, coefficient
uncertainty, state order, and control order. Then:

1. Evaluate the unperturbed state-rate vector `f(x0, u0)`.
2. Perturb one state or control using a physical step that stays inside the
   same table, actuator, and branch region.
3. Recompute air data, coefficients, forces, moments, mass-dependent
   accelerations, and inertia for both plus and minus evaluations.
4. Form each central-difference column:

   ```text
   A[:,j] = (f(x0 + dx_j, u0) - f(x0 - dx_j, u0)) / (2*dx_j)
   B[:,k] = (f(x0, u0 + du_k) - f(x0, u0 - du_k)) / (2*du_k)
   ```

5. Repeat with smaller and larger steps where the source table is smooth.
6. Check that the expected physical scales appear before synthesizing gains.

The direct signatures are:

```text
partial(u-dot) / partial(Fx) ~= 1/m
partial(p-dot) / partial(Mx) ~= 1/Ix
```

At `m=1200 kg` and `Ix=4200 kg m^2`, those are approximately `8.33e-4` and
`2.38e-4` in SI units. If those sensitivities are missing or off by orders of
magnitude, inspect frames, units, mass properties, and moment reference before
tuning `Q` or `R`.

The reusable artifact path is:

```python
from taoryx.trim import finite_difference_dynamics_linearization

linearization = finite_difference_dynamics_linearization(
    trim_spec,
    plant_state_derivatives,  # returns derivatives named like trim states
    trim_result,
    state_step=1.0e-6,
    control_step=1.0e-6,
    metadata={
        "mass-kg": trim_result.state["mass"],
        "inertia-source": "vehicle-provider",
        "aero-source-quality": "estimated",
    },
)
```

The helper's steps are fractional scales applied to each variable. Verify the
resulting absolute perturbations for angles, rates, controls, and dimensional
states. If mass, CG, or aerodynamic conditions change materially, repeat the
trim and local linearization or provide a schedule of local designs. A stable
gain from estimated aerodynamics remains an estimated-model result.

## Automated weight search

For route development, do not hand-tune `q-angle` and `r-moment` on a linear
scale. Vehicle inertias can differ by many orders of magnitude, so the native
rectangle tuner accepts positive logarithmic search bounds:

```yaml
log_bounds:
  q-angle: [1.0e8, 1.0e13]
  q-rate: [1.0e-2, 1.0e2]
  r-moment: [1.0e-3, 1.0e1]
```

Candidates are injected into ordinary `.prb` attributes and executed by the
normal parser/runtime path. Each candidate is ranked using completion, route
error, altitude and speed error, table-envelope violations, and attitude
controller saturation. A candidate with stable poles but persistent
saturation is therefore not promoted as a good controller.

```text
python tools/tune_rectangle_controller.py \
  --case b747 --method differential-evolution --evaluations 48
```

The search is a development aid, not evidence by itself. The winning
candidate must be copied into a fixed problem file and rerun with the full
convergence, closure, source-parity, and mission gates.
