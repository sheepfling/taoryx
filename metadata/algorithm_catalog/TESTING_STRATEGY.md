# Testing Strategy

## Required traceability

Every implementation symbol should declare the algorithm IDs and manual equations it implements.
Every test should name at least one algorithm ID. The catalog's `equation_algorithm_map.csv`
provides the bridge back to all numbered equations.

## Test layers

1. **Formula unit tests** — direct numeric examples and hand-computed special cases.
2. **Property tests** — round trips, orthogonality, conservation, monotonicity, and dimensional identities.
3. **Finite-difference tests** — compare analytic rates and derivatives with perturbed states.
4. **Algorithm regression tests** — fixed inputs/outputs for searches, atmosphere layers, gravity models, and table programs.
5. **Fixture tests** — use every Chapter 3/4 snippet from the Version 22 snippet corpus.
6. **Trajectory tests** — run complete Chapter 4 problems once the runtime is available.
7. **Historical equivalence tests** — only possible if the TAOS 96.0 executable or trusted output baselines become available.

## Highest-value property families

- coordinate transform forward/inverse round trips;
- orthonormality and determinant +1 of bases;
- finite-difference agreement for position, rate, and radar derivatives;
- RK order-of-accuracy under step halving;
- atmosphere continuity at layer boundaries;
- gravity acceleration as the negative numerical gradient of potential;
- direct/inverse Sodano consistency;
- table interpolation exactness at knots and for affine functions;
- guidance residual convergence and control-bound enforcement;
- event refinement that never skips an earlier crossing;
- partial trajectory restart preserving unaffected history.

## Suggested pytest layout

```text
tests/
    unit/
        test_coordinates.py
        test_atmosphere.py
        test_gravity.py
        test_aerodynamics.py
        test_searches.py
        test_tables.py
    properties/
        test_coordinate_roundtrips.py
        test_derivative_finite_differences.py
        test_interpolation_properties.py
    fixtures/
        test_chapter03_snippets.py
        test_chapter04_snippets.py
    integration/
        test_derivative_pipeline.py
        test_trajectory_runtime.py
        test_chapter04_problems.py
```

## Catalog test-plan coverage

The catalog contains explicit test-plan fields for all 107 algorithms.
