# Tumbling Aero Coefficients

Working area for tumbling-body aerodynamic coefficient studies.

- Study note: [study.md](./study.md)
- First visualization script: [plot_drag_area_models.py](./plot_drag_area_models.py)
- Sphere/ellipsoid sweeps: [plot_sphere_ellipsoid_sweeps.py](./plot_sphere_ellipsoid_sweeps.py)
- Isotropic averages: [plot_isotropic_averages.py](./plot_isotropic_averages.py)
- Table generator: [generate_tbl_data.py](./generate_tbl_data.py)
- Launch sweep generator: [generate_launch_sweeps.py](./generate_launch_sweeps.py)
- Ballistic simulator: [simulate_ballistic_cone.py](./simulate_ballistic_cone.py)
- Four-shape simulator: [simulate_ballistic_shapes.py](./simulate_ballistic_shapes.py)
- Coupled rigid-body simulator: [simulate_coupled_tumble.py](./simulate_coupled_tumble.py)
- Full-angle coefficient CLI: [tools/aero_drag_analysis.py](../../tools/aero_drag_analysis.py)
- Reusable full-angle models: [taoryx.aero_drag_tables](../../src/taoryx/aero_drag_tables.py)
- Generated tables: [generated/](./generated/)
- Ballistic launch harness: [generated/ballistic_cone_launch.prb](./generated/ballistic_cone_launch.prb)

Planned additions in this folder should stay scriptable and focused on:

- drag-area versus projected-area comparisons;
- sphere and ellipsoid projected-area sweeps;
- isotropic-orientation average plots;
- generated `.tbl` decks with explicit provenance;
- a concrete ballistic launch harness using the cone coefficient deck;
- a swept family of launch PRBs;
- a simulated time-history output deck and plot;
- a four-shape comparison of position, flight-path angle, body orientation, speed, and `C_D`;
- angle sweeps for cylinders, cones, and other axisymmetric bodies;
- coefficient interpolation models for tumble dynamics.
- center-of-pressure and angular-rate torque histories for coupled attitude studies.

## New dump integration boundary

The imported full-angle analysis is represented by
`src/taoryx/aero_drag_tables.py`, `tools/aero_drag_analysis.py`, and focused
tests in `tests/unit/test_tumbling_analysis.py`. It provides projected-area
models, illustrative front/broadside/rear closures, the two-angle body-
direction map for triaxial ellipsoids, periodic interpolation, CSV output, and
TAOS-compatible `.tbl` generation.

These are synthetic analysis fixtures. They do not establish a universal
aerodynamic correlation, historical TAOS behavior, or validated CFD/test data.
The coupled planar script is a research prototype with illustrative moment
terms; it is not a complete six-degree-of-freedom runtime.

Run the complete gallery with:

```sh
python tools/aero_drag_analysis.py --all --output-dir build/aero-drag
```

This creates per-shape CSV/`.tbl` files and plots, the triaxial orientation
map, `normalized-full-angle-comparison.png`,
`normalized-full-angle-polar.png`, and a JSON comparison summary.
