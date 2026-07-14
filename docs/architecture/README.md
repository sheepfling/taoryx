# taoryx simulation architecture

These notes describe the implementation boundaries for the TAOS successor.
The manual describes a trajectory-analysis system with several distinct
responsibilities; the replacement should preserve those boundaries instead of
turning the `.prb` parser into the simulator.

## System layers

| Layer | Manual source | Planned package boundary |
| --- | --- | --- |
| Source language | Chapters 3–4; `grammars/` | `taoryx.language` |
| Tables and constants | Chapter 3; `metadata/table_*.yaml` | `taoryx.tables` |
| Problem resolution | Chapter 4 data blocks | `taoryx.scenario` |
| State and reference frames | Chapter 2 §§1–2 | `taoryx.coordinates`, `taoryx.state` |
| Forces and rates | Chapter 2 §3; `metadata/acceleration_*.yaml` | `taoryx.models` |
| Integration and events | Chapter 2 §§2.1, 6; Chapter 4 `*Integ`, `*When` | `taoryx.integration`, `taoryx.events` |
| Guidance/search/optimization | Chapter 2 §§5–6; Chapter 4 `*Search`, `*Optimize`, `*Survey` | `taoryx.guidance`, `taoryx.search` |
| Outputs and reports | Chapter 2 §4; Chapter 4 `*Print`, `*Summarize` | `taoryx.outputs` |

The runtime now provides an executable, evidence-bounded subset of this
architecture: parsed `.prb`/`.tbl` files lower into runtime problems and
tables, the engine integrates trajectories, applies supported events and
controls, and renders outputs and summaries. It does not claim numerical
equivalence with historical TAOS 96.0, and unsupported or ambiguous language
shapes are diagnosed rather than silently executed.

## Related catalog pages

- `metadata/algorithm_catalog/` holds the reviewed planning catalog and the
  generated implementation ledger.
- [`docs/architecture/algorithm-catalog.md`](algorithm-catalog.md) explains the
  catalog as an architecture layer, not as a second equation registry.
- [`docs/architecture/state-model.md`](state-model.md) defines the canonical
  ECFC point-mass state and its runtime/integrator boundaries.
- [`docs/architecture/vehicle-data-model.md`](vehicle-data-model.md) groups
  the successor-side geometry, mass, propulsion, and effector data families
  that sit above the current TAOS `.tbl` abstraction.
- [`docs/architecture/telemetry.md`](telemetry.md) defines the structured
  runtime artifact consumed by reports and visualization backends.

## Execution shape

```text
.prb/.tbl files
      │
      ▼
parse → validate → resolve scenario
                        │
                        ▼
              state + model inputs
                        │
                        ▼
             derivatives + events
                        │
                        ▼
             integration / segments
                        │
                        ▼
              outputs / search results
```

## Implementation order

1. Extend typed parser defaults and cross-block validation.
2. Complete remaining unit-dimension projections and parser/runtime semantic
   coverage while preserving the lossless source boundary.
3. Expand trajectory-level regression baselines and resolve large synthetic
   optimization performance.
4. Establish historical comparison evidence when a trusted TAOS 96.0
   executable or output corpus is available.

Every implemented equation should link back to its canonical ID in
`metadata/equations.csv`; implementation coverage is separate from manual
transcription coverage.
