# Robustness acceptance matrix

`robustness_matrix_v1.yaml` is the claim-control boundary for the paired
trajectory examples. It separates nominal completion, numerical convergence,
model-envelope and actuator safety, bounded perturbation robustness, and
evidence-only CA-HI results.

The variation lists are parameter axes, not a license to combine arbitrary
out-of-envelope values. Every run must record its rendered problem inputs,
source table IDs, solver settings, and failure classification.

The bounded-robustness gate requires at least a 95% pass rate, but a family
cannot claim success if a failure is unclassified, silently extrapolated, or
outside the declared source envelope. CA-HI remains evidence-only until a
separate endpoint requirement and independent route oracle exist.

To generate the full Matplotlib evidence bundle, including every evaluated
robustness pair, the CA-HI DOF stress gamut, and the long-route showcase, run:

```bash
python tools/dev.py verification-artifacts
```

Outputs are ignored under `artifacts/verification/robustness_matrix_v1/`,
`artifacts/dof_robustness_v1/`, and
`artifacts/showcases/california_to_hawaii/`. Each plotted run includes a
`plot-manifest.json` recording rendered and unavailable channels.

Nominal family composites are generated at
`artifacts/showcases/slower_vehicle_composites/`: one nine-panel figure per
vehicle and one three-column all-family dashboard comparing 3-DOF with 6-DOF.
