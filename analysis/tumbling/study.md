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
