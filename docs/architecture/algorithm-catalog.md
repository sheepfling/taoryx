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

The ignored `INBOX/taos-algorithm-catalog-v1` bundle is reviewed and validated
as a unit. Once its schema and source relationships are accepted, the canonical
YAML and generated views can be promoted into tracked `metadata/` paths. Until
then, implementation code may consume it through an explicit path, but no
runtime default should depend on an ignored inbox file.

## Implementation status

The catalog's `cataloged` status means the work item has provenance, steps, and
test ideas. It does not mean the target function is implemented. Completion
requires a real `taoryx` binding, focused tests, provenance links, and the
repository validation gates.
