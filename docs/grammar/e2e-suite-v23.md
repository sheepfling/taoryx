# TAOS v23 end-to-end corpus

The checked-in corpus at [`tests/fixtures/taos_e2e_v23/`](../../tests/fixtures/taos_e2e_v23/)
was imported from the v23 inbox bundle. It is an evidence-bearing integration corpus,
not a claim of historical runtime equivalence.

## Inventory

- 63 complete cases: 49 positive and 14 negative;
- 63 `.prb` inputs and 82 `.tbl` inputs;
- 9 metamorphic groups;
- positive representation coverage for all 33 scoped Chapter 4 blocks, 19 Chapter 3
  table types, and 28 Chapter 3 full-table operations.

The original bundle's checksum verification and frozen-parser static suite passed
before import. The frozen parser and wheel were deliberately not copied into the
canonical repository; the corpus must run against `taoryx.language` instead.

## Manual-backed parser coverage

The live parser now accepts the three formerly unresolved positive forms directly:

- user atmospheres may name kinematic viscosity as `nu` or dynamic viscosity as
  `visc`;
- `*file` after completed trajectory definitions is attached at problem scope and
  accepts two-subscript relative/radar output variables;
- limited-state `cg` and wind tables reject non-limited state variables while
  preserving documented user-defined independent variables such as `config`.

Problem-level `define`, `file`, and `print` blocks that occur after completed
trajectory segments remain physically ambiguous because the manual documents those
keywords at both scopes. The parser preserves the chosen attachment and emits an
`ambiguous-dual-scope-block` warning; the manifest contract still counts the
documented problem-level productions explicitly.

The regression tests retain the source locations and raw text for malformed forms.
Any future evidence gap must be added to `tests/e2e/live_compatibility.py` with an
explicit rationale rather than silently weakening the positive-case checks.

## Runtime boundary

Static tests run by default. Runtime and metamorphic tests are marked `runtime` and
require `TAOS_EXE` to point to a historical or separately verified compatible
executable:

```sh
python3 tools/dev.py e2e
python3 -m tests.e2e.cli --json
TAOS_EXE=/absolute/path/to/taos python -m pytest -m runtime tests/e2e
```

No runtime result is inferred from the presence of analytic oracles. Runtime output
column conventions, tolerances, and external table behavior must be calibrated
against the executable before those tests become release evidence.
