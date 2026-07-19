# Slower air-breathing and multirotor 6-DOF public research bundle

Version 1.0.0, generated 2026-07-16.

This bundle contains three coherent, slower vehicle models:

1. **Boeing 747-100** — a subsonic jet model based on NASA CR-2144 handling-qualities derivatives.
2. **Skywalker X8** — a non-weapon cruise-class fixed-wing UAV with a 2025 flight-test-identified nonlinear 6-DOF model.
3. **AscTec Hummingbird** — a RotorPy quadcopter model with rotor, motor-lag and aerodynamic-wrench effects.

A fourth folder records the **NASA Generic Transport Model** as an alternate small twin-turbine jet lead. Its exact aerodynamic database is a binary MATLAB file and is not falsely represented as included.

## Safety and scope

The X8 is intentionally used instead of a cruise-missile-specific model. This bundle contains no target selection, terminal guidance, warhead, terrain-following, survivability or mission-performance data.

## Axes

- B747 and X8 generated coefficient tables: body `+X forward`, `+Y right`, `+Z down`.
- Hummingbird: RotorPy source-native Cartesian body frame, with `+Z` along positive rotor thrust; X and Y lie in the rotor plane.

## Fidelity

Every file carries a source ID and fidelity label. Generated dense grids are deterministic evaluations of source equations; a generated grid is not equivalent to a wind-tunnel measurement at every row.

Run `python scripts/validate_dataset.py` from the bundle root to repeat the compact integrity checks.
