# taoryx simulation architecture

The manual describes a trajectory-analysis system with several distinct
responsibilities. The replacement should preserve those boundaries instead of
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

The initial runtime scaffold is deliberately generic. It does not claim
numerical equivalence with historical TAOS 96.0 and does not implement the
manual's equations yet.

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

1. Stabilize parser ASTs and table resolution.
2. Define units, frames, state variables, and scenario resolution contracts.
3. Implement coordinate transforms and equation registry bindings.
4. Add derivative contributions and a validated integrator.
5. Add segment/event execution and output evaluation.
6. Add search, optimization, guidance, and regression fixtures.

Every implemented equation should link back to its canonical ID in
`metadata/equations.csv`; implementation coverage is separate from manual
transcription coverage.
