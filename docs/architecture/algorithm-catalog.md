# Algorithm Catalog Architecture

This page explains the catalog as an implementation-planning layer. The v1
catalog decomposes the manual into 107 implementation-sized records, but it is
not a second equation registry and it is not a claim of TAOS 96.0 runtime
compatibility.

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
test ideas. The generated `algorithm_status.csv` records separate evidence:
roadmap completion, documented binding, target importability, verification-path
existence, implementation stage, and historical-equivalence status.
`unit_verified` means that the documented target imports and every named test
file exists. It does not claim that every input branch has been exercised,
that the runtime semantics are complete, or that behavior matches TAOS 96.0.
`roadmap_status=complete` is a stronger project decision and must only be
marked after the algorithm's execution behavior, focused tests, provenance
links, and repository gates are complete. Historical equivalence remains a
separate status because it requires the historical executable or trusted
output baselines.
