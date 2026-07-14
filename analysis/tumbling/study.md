# Tumbling Aero Coefficients Study

This note collects the practical ideas for modeling drag on tumbling bodies.

## Core point

For flight dynamics, the useful quantity is usually drag area:

`K_D = D / q_inf = C_D S_ref`

where `q_inf = 1/2 rho V^2`.

That form avoids confusion when different sources use different reference areas.

## Geometry versus aerodynamics

Projected area is only a first approximation. The actual drag depends on:

- separation;
- base pressure;
- Reynolds number;
- Mach number;
- surface roughness and seams;
- angle of attack or body orientation.

For tumbling bodies, drag alone is usually not enough. A practical database often needs:

- three force coefficients;
- three moment coefficients;
- rate damping terms for rotation.

## Orientation requirements

A static flow condition is set by the flow direction in body coordinates, which has two degrees of freedom.

Useful orientation classes:

- sphere: no orientation dependence;
- spheroid or cylinder: one angle;
- cone: one angle over `0` to `180 deg`;
- triaxial ellipsoid: a full direction vector on the unit sphere.

## Useful models

### Cylinder

An engineering interpolation for a symmetric cylinder is:

`K_D(alpha) ~ K_0 |cos(alpha)|^3 + K_90 |sin(alpha)|^3`

### Cone

For a cone, separate the nose-first, base-first, and broadside anchors:

`K_D(alpha) ~ K_nose max(cos(alpha), 0)^3 + K_base max(-cos(alpha), 0)^3 + K_90 |sin(alpha)|^3`

## Next scripts

The first script in this folder plots projected area and drag-area models versus orientation angle for a cylinder and cone. Future scripts can add:

- sphere and ellipsoid projected-area curves;
- isotropic-orientation averages;
- coefficient database surface plots;
- damping-term sweeps for angular-rate sensitivity.

Those follow-on plots now live beside this note as:

- [plot_drag_area_models.py](./plot_drag_area_models.py)
- [plot_sphere_ellipsoid_sweeps.py](./plot_sphere_ellipsoid_sweeps.py)
- [plot_isotropic_averages.py](./plot_isotropic_averages.py)

Generated `.tbl` decks now come from:

- [generate_tbl_data.py](./generate_tbl_data.py)
- [generated/](./generated/)

The ballistic launch workflow now also includes:

- [generate_launch_sweeps.py](./generate_launch_sweeps.py)
- [simulate_ballistic_cone.py](./simulate_ballistic_cone.py)
- [generated/ballistic_cone_launch.prb](./generated/ballistic_cone_launch.prb)
- [generated/ballistic_cone_history.tbl](./generated/ballistic_cone_history.tbl)
- [artifacts/ballistic_cone_profiles.png](./artifacts/ballistic_cone_profiles.png)

## Four-shape trajectory comparison

The working comparison uses four assumed bodies with the same launch conditions:

- sphere: orientation-independent reference case;
- cylinder: axisymmetric body with broadside and end-on drag-area anchors;
- cone: nose-first, broadside, and base-first drag-area anchors;
- triaxial ellipsoid: non-axisymmetric projected-area surrogate.

Run [simulate_ballistic_shapes.py](./simulate_ballistic_shapes.py) to generate one history table per body under [generated/four_shape_histories/](./generated/four_shape_histories/) and the aligned comparison plot [artifacts/four_shape_ballistic_comparison.png](./artifacts/four_shape_ballistic_comparison.png). Each history includes downrange position, altitude, speed, flight-path angle, tumble orientation `alpha`, dynamic pressure, and `C_D`.

## Coupled attitude prototype

[simulate_coupled_tumble.py](./simulate_coupled_tumble.py) integrates planar position, velocity, attitude `theta`, and angular rate `omega`. It recomputes `alpha` and `C_D` at every integration stage, and records center-of-pressure, rate-damping, optional static-moment, and total torque separately. The model is deliberately illustrative until calibrated coefficient and moment data are supplied.

The reusable full-angle command is [tools/aero_drag_analysis.py](../../tools/aero_drag_analysis.py):

```sh
python tools/aero_drag_analysis.py --config examples/aero_drag/cone.json --output-dir build/aero-drag/cone
python tools/aero_drag_analysis.py --shape triaxial-ellipsoid --output-dir build/aero-drag/ellipsoid
```

Axisymmetric outputs include full-angle CSV data and a `0` to `180 deg` `cd(alphat)` table. Triaxial output uses `cd(alphat,phi)` with `phi` changing fastest in the flattened table values.
