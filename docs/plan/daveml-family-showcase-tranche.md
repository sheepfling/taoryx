# DaveML Family Showcase Tranche

**Status:** implemented evidence-board generator and initial qualification packs
**Scope:** Alpha 3 showcase products for promoted DaveML families
**Out of scope:** B747/X-15 parallel showcase work, LaTeX, flight qualification,
and promotion of source-only NESC object records as executable families

The catalog is defined in
`verification/daveml_showcase_catalog.yaml` and generated with:

```text
python tools/dev.py daveml-showcase
```

Outputs are written beneath `artifacts/showcases/daveml-families/`. Each board
contains a manifest, evidence summary, telemetry payload, rendered evidence
board, and an object-lineage artifact when the recipe requires lineage.

## Boards

| Board | Family/evidence boundary | Required distinction |
|---|---|---|
| F-16 maneuver and trim | `reference_exact`, source-bounded 6-DOF | Maneuver evidence is not flight qualification |
| HL-20 glide and entry | `reference_exact`, unpowered source channels | No controller or actuator claim |
| NESC two-stage launch | `reference_exact`, source-retained Scenario 17 | Stage lineage is evidence-backed, not independent participating-simulation equivalence |
| A320 comparison | `derived_exact` OpenAP versus `surrogate_composite` OpenAP+JSBSim | Fidelity products remain separate and are not treated as identical trajectories |
| NESC synthetic passive child | source-backed parent plus `synthetic` cylinder | Child aerodynamics are an explicit radial-ballistic witness, not NESC source truth |

The generator consumes existing verified reports rather than rerunning a hidden
vehicle-specific simulation. The NESC parent telemetry and mission event table
are extracted from the pinned evidence package. The synthetic child is derived
only in the deployment board using the declared `radial-ballistic-witness-v1`
policy, and its lineage is separate from the source parent lineage.

Every generated run uses the common `taoryx.showcase/v1alpha1` contracts for
fidelity, claim/nonclaim text, evidence-board modules, artifact hashes, and
object lineage. Missing source channels remain unavailable rather than being
filled with fabricated values.
