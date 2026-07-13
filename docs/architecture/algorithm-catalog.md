# Algorithm Catalog Architecture

The v1 catalog decomposes the manual into 107 implementation-sized records. It
is a planning and traceability layer, not a second equation registry and not a
claim of TAOS 96.0 runtime compatibility.

## Boundaries

1. `manual/` and `metadata/equations*` remain the historical source and
   equation provenance layer. Every formula implementation continues to cite
   canonical equation identifiers there.
2. The algorithm catalog describes implementation-sized work: formula groups,
   numerical methods, table semantics, and runtime workflows. It may reference
   equations, but an algorithm is not required to correspond one-to-one with an
   equation.
3. `taoryx.catalog` is a typed read-only index over the generated catalog. It
   provides stable ID, dependency, phase, domain, target, and equation-map
   queries without forcing the proposed `taos_math` / `taos_runtime` names into
   the current package layout.
4. `taoryx.equations`, `taoryx.language`, and `taoryx.simulation` remain the
   current implementation packages. A catalog target is an intended binding,
   not evidence that the symbol exists or that the algorithm is complete.

## Promotion rule

The reviewed catalog is promoted under `metadata/algorithm_catalog/`. The
ignored `INBOX/taos-algorithm-catalog-v1` bundle remains an intake copy and is
not a runtime dependency. `algorithm_status.csv` is generated from the tracked
JSON catalog and the implementation ledger by
`tools/generate_algorithm_status.py`.

## Implementation status

The catalog's `cataloged` status means the work item has provenance, steps, and
test ideas. `algorithm_status.csv` separately records whether a documented
typed binding exists; `typed_binding` still does not claim full historical
runtime semantics. Completion of a runtime algorithm requires its lowering,
execution behavior, focused tests, provenance links, and repository gates.
