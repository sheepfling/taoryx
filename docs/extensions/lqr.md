# LQR extension

TAORYX provides a declarative LQR configuration and a numerical solver. The
problem file names the state/control contract and the sources of the matrices;
the runtime or a Python model adapter supplies the numeric `A`, `B`, `Q`, and
`R` matrices.

```text
*runtime lqr attitude
  states=alpha,beta,p,q,r
  controls=fin-pitch,fin-yaw
  linearization=vehicle-trim
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
from taoryx.runtime import solve_continuous_lqr

result = solve_continuous_lqr(A, B, Q, R,
                              state_names=("alpha", "q"),
                              control_names=("elevator",))
u = -result.gain @ (x - x_trim)
```

The solver validates dimensions, finite values, symmetry and definiteness of
`Q` and `R`, controllability, and reports closed-loop eigenvalues. Matrix
inversion is intentionally not exposed; the implementation uses linear solves.

This is a TAORYX extension, not historical TAOS syntax. Automatic trim,
linearization, gain scheduling, actuator saturation, and online Riccati updates
remain separate runtime features.
