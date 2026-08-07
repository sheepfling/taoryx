# Product 2 numerical quality and fidelity gates

Status: M4 implementation contract

Product 2 separates a runnable trajectory from a numerically bounded result.
The run manifest records the inputs, runtime, artifact identity, and outcome;
the M4 report records evidence about the emitted artifact. A completed run or
a successful plot never substitutes for the M4 report.

## Catalog declaration

Every entry in `verification/product_two_scenario_catalog.yaml` carries a
`quality` block. It declares:

- an isolated `fidelity.lane` and a second claim-boundary statement;
- the response law, omitted physics, controls, envelope, and operation
  availability for pseudo-6DOF entries;
- the telemetry channels that must be finite;
- optional state-continuity tolerances and explicitly declared conservation
  invariants;
- fixed-input repeatability channels and tolerances; and
- selected coarse/fine step multipliers, integrators, channels, and event-time
  tolerance.

The loader rejects an incomplete pseudo-6DOF declaration. The catalog audit
also requires separate lanes for NESC, X-15, HL-20, and synthetic
California–Hawaii evidence and requires at least two refinement cases.

## Report contract

`taoryx.product_two_quality.evaluate_product_two_quality` emits
`taoryx.product-two-numerical-quality/v1alpha1` for one scenario. The report
contains a disposition for each gate:

| Disposition | Meaning |
| --- | --- |
| `pass` | The declared evidence was evaluated and stayed within its bound. |
| `development` | The lane is explicit, but the common artifact or selected gate is not yet part of the evidence. |
| `blocked` | A required artifact, channel, or precondition is missing. |
| `bounded-failure` | Evidence was evaluated and exceeded a declared numerical or identity bound. |

The evaluator checks finite selected telemetry and strict time ordering,
nondecreasing event streams, continuity outside declared events, and only the
mass/energy invariants named by the scenario. Repeatability compares selected
telemetry plus scenario/model identity. Refinement interpolates the fine
history onto the coarse history and separately compares event names and event
times; spawned-child histories may have a small accepted-boundary offset that
must be declared in the scenario tolerance.

## Running the gate

The full machine-readable suite report can be generated with:

```bash
python tools/validate_product_two_quality.py --output artifacts/verification/product_two_quality/report.json
```

The repository check executes the two source fixed-step cases and the
interactive accepted-boundary case:

```bash
python tools/validate_product_two_quality.py \
  --execute \
  --scenario two-stage-ballistic \
  --scenario two-stage-demo \
  --scenario interactive-california-hawaii
```

The report still contains all canonical scenarios. Cases outside this
selected common-artifact execution set remain explicitly `blocked` or
`development`; they are not promoted because a nominal or family-owned
trajectory exists. Family-native quality reports remain authoritative until a
common Product 2 artifact adapter is declared.

