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

This is a TAORYX extension, not historical TAOS syntax. Automatic trim,
linearization, gain scheduling, actuator saturation, and online Riccati updates
remain separate runtime features.

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

`finite_difference_linearization` can then produce a local Jacobian around the
trim. The adapter must still map residuals to true state derivatives before
passing matrices to LQR; a force/moment Jacobian is not automatically an
`A,B` dynamics pair.
