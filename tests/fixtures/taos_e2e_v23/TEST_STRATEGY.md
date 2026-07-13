# Higher-order TAOS test strategy

## What this suite validates now

- Whole-document `.prb` parsing, including multiple problems, trajectories, and segments.
- Whole-file `.tbl` parsing for simple, full, skewed, output, wind, aerodynamic,
  propulsion, and mass-flow tables.
- Problem-to-table dependency closure.
- Complete application launch commands and output-file contracts.
- Analytic dynamics in deliberately simplified zero-force and constant-force cases.
- Search and optimization behavior on linear problems with known solutions.
- Guidance, atmosphere, radar, IIP, wind, staging, branch, and multi-vehicle smoke tests.
- Metamorphic equivalence where exact historical output is unavailable.
- At least one positive integration input for every documented scoped Chapter 4 block,
  every Chapter 3 table type, and every Chapter 3 full-table operation.
- Opt-in rejection tests, historical/exploratory runs, and a large-table/many-segment stress case.

## Recommended next tiers for the live codebase

1. Add a deterministic `taos_math` reference engine and use it as a differential oracle.
2. Capture golden outputs from a licensed/historical TAOS 96.0 executable.
3. Add randomized AST generation constrained by the semantic model.
4. Add mutation tests for every block option and every table operation.
5. Add restart/recovery tests for optimization and search failure modes.
6. Add long-duration stress cases, large tables, many trajectories, and memory limits.
7. Add platform portability tests for path names, line lengths, case folding, and exponent formats.
8. Add parser round-trip tests once a canonical formatter exists.
9. Add dimensional-analysis checks for every `*units/fmt` conversion.
10. Add equation-to-runtime traceability: every runtime oracle should cite the governing equation IDs.
