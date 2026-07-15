# Problem-file documentation guide

Every documented `.prb`/`.tbl` fixture must have a neighboring `README.md`
and a machine-readable manifest such as `expected.yaml` or `family.yaml`.

## Required README sections

```text
# Scenario title
Status and claim boundary
Purpose
Dynamics mode
Phase/segment schedule
Input files
Expected events and invariants
Output channels and plots
Validation commands
Known ambiguities and exclusions
Provenance
```

The README explains intent in human terms. The manifest records exact values,
units, tolerances, source hashes, and claim labels. Do not hide a source
correction or an unresolved convention in comments alone.

## Required manifest fields

```yaml
schema_version: 1
id: example-name
status: scaffolded
claims:
  manual_bounded: false
  taoryx_extension: true
inputs: []
invariants: []
plots: []
limitations: []
```

Each input should identify its path and role. Each invariant should provide a
unit, tolerance, and evidence class. Each plot should identify the normalized
telemetry channels it consumes.

## Validation commands

Use the narrowest useful command during development, then the repository gates
before handoff:

```bash
python tools/dev.py check
python -m pytest tests/unit/test_<scenario>.py
python -m pytest -m artifact
```

Generated plots, SQLite files, and summaries belong under ignored
`artifacts/`; source fixtures, manifests, and documentation remain tracked.
