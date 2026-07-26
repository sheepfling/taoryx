# DAVE-ML conformance suite

The official examples are treated as language fixtures:

- `simple_aero.dml`: legacy one-dimensional table form.
- `twoD_table.dml`: regular two-dimensional interpolation.
- `twoD_ungridded.dml`: partially nonorthogonal scattered data.
- `threeD_ungridded.dml`: fully scattered three-dimensional data.
- `atmos_76.dml`: large environment tables and 42 embedded checks.
- `F16_aero.dml`: nonlinear six-axis aircraft aerodynamics.
- `HL20_aero.dml`: large shared-table polynomial aerodynamic buildup.

The first required runtime expansion is named, deterministic ungridded interpolation. No implicit SciPy default should become part of the Taoryx interchange contract without an explicit policy identifier and test vectors.
