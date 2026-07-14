# Claim registry

```yaml
claims:
  - id: D
    name: documentary_fidelity
    scope: reconstructed 1995 manual edition
    excludes: executable behavior not printed by the source
  - id: S
    name: semantic_coherence
    scope: adopted equations, constants, defaults, restrictions
    excludes: unresolved Class D ambiguities in normal execution
  - id: P
    name: parser_conformance
    scope: explicitly evidenced .tbl/.prb constructs
    excludes: undocumented syntax and incomplete manual excerpts treated as full files
  - id: N
    name: numerical_correctness
    scope: individually benchmarked mathematical kernels
    excludes: historical iteration sequence or vehicle-table completeness
  - id: H
    name: historical_compatibility
    scope: TAOS 96.0 behavior
    evidence_required: historical executable, source, or trusted output corpus
    status: not_established

The numerical-verification work in this repository is scoped to TAOS 1995.
TAOS 96.0 compatibility remains a separate claim and must not be implied by
manual fidelity or by passing unit tests alone.
```

The status of a claim is determined per release and per scope. A passing unit
test may support a requirement; it does not promote the whole claim to verified.

####
