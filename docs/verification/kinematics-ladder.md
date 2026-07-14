# Kinematics ladder

The public kinematics showcases are deliberately small and layered. Each
physics case should eventually have two representations:

1. a direct `RuntimeProblem` case that isolates integration, dynamics, and
   events from parsing; and
2. a `.prb` case that exercises parsing, lowering, runtime execution, and
   output together.

Both are checked against an analytic or invariant-based oracle that is
implemented outside TAORYX.

## Current executable tranche

The local runtime now applies the manifest-declared analytic oracles to:

- `p001_linear_ecfc_zero_force` — constant Cartesian velocity;
- `p002_constant_thrust_table` — constant thrust with fixed mass.

Run only this tranche with:

```text
python -m pytest tests/e2e/test_local_runtime_analytic.py
```

The existing `p004`–`p006` refinement group remains historical/metamorphic
evidence for now. Their `.prb` lowering currently uses adaptive RKF45 and the
printed samples conceal the internal accepted-step history, so the manifest's
fixed-step convergence relation must not be silently relabeled as a local RK4
claim. A direct `rk4_step` convergence test and an explicitly configured
file-runtime RK4 route are separate follow-up work.

## Planned levels

| Level | Case | Independent evidence |
| --- | --- | --- |
| 1 | Constant velocity | Exact position and constant velocity history |
| 2 | Constant acceleration | Exact position and velocity polynomials |
| 3 | Uniform free fall | Impact time, impact velocity, and event refinement |
| 4 | Variable-mass thrust | Rocket-equation mass, velocity, and position history |
| 5 | Flat 2-D projectile | Parabola, apex, range, and impact event |
| 6 | Spherical vacuum ballistic | Energy and angular-momentum invariants |
| 7 | Drag fall | Terminal-velocity solution or trusted independent reference |

These are analytically verified TAORYX runtime showcases. They are not claims
of historical TAOS 96.0 compatibility.
