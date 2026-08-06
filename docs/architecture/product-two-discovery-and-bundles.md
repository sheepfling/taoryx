# Product 2 discovery, diagnosis, and bundles

M1 and M2 provide one consumer-facing route from “which model?” to “what
evidence did this run produce?” The canonical source is
[`verification/product_two_scenario_catalog.yaml`](../../verification/product_two_scenario_catalog.yaml).
It contains stable IDs and aliases, input paths, family/model identity,
fidelity, realization, operation, expected disposition, commands, artifacts,
and claim boundaries for the Product 2 regression set.

## Discover a scenario

Run these commands from the repository root:

```bash
taoryx scenario list --json
taoryx scenario search hawaii --json
taoryx scenario show hl20-pseudo6dof --json
```

Aliases are deliberate onboarding shortcuts; the stable ID is the value that
belongs in reports and automation. Search is case-insensitive and searches IDs,
aliases, descriptions, family, fidelity, realization, operation, and tags.

## Run the doctor ladder

```bash
taoryx doctor hl20-source-release-pseudo6dof --json
```

`doctor` executes the checks in this order:

1. catalog input existence and SHA-256 identity;
2. `.tbl` structure, names, axes, and bounds where tables are declared;
3. source or composition compilation;
4. source resolution or composed interface validation;
5. source lowering or exact composition lowering; and
6. family-owned capability preflight for compositions.

Every blocking finding has a stable `code`, `location`, `message`,
`remediation`, and `blocking: true` field. A doctor `passed` result means the
setup boundary is ready for the declared operation. It does not execute the
model or qualify the resulting trajectory. `expected_disposition` remains
separate, so pseudo-6DOF and HL-20 entries can be runnable while still marked
`development`.

## Produce a reproducible bundle

```bash
taoryx scenario bundle hl20-source-release-pseudo6dof \
  --output-dir artifacts/product2/hl20-source-release-pseudo6dof

taoryx scenario manifest validate \
  artifacts/product2/hl20-source-release-pseudo6dof/run-manifest.json

taoryx artifact validate \
  artifacts/product2/hl20-source-release-pseudo6dof/run/run-artifact.json
```

A bundle contains:

- `inputs/` — copied catalog inputs with their original repository-relative
  layout;
- `catalog-entry.json` — the exact discovery metadata used;
- `composition.json` or source-run outputs when the entrypoint requires them;
- `run/` — normalized telemetry, native reports, status/event traces, and
  family-owned evidence;
- `plots/` when a normalized `RunArtifact` is available;
- `reproduction.txt` — setup and run commands rooted at the bundle directory;
- `run-manifest.json` — the schema-versioned Product 2 identity, runtime,
  integration, accepted time, termination, artifact hashes, and claim boundary;
  and
- `bundle-index.json` — a compact handoff pointer to the manifest and
  reproduction record.

The manifest’s `run_identity` is derived from the scenario ID, source-input
hashes, operation, fidelity/realization, integration settings, and requested
time. Artifact hashes bind the produced evidence separately. The manifest
reader rejects unknown schema identifiers or unsupported schema versions and
preserves additive fields within the supported version.

`taoryx run` and `taoryx vehicle run` also emit `run-manifest.json` beside
their output artifacts. Interactive consumers can use the canonical
`examples/showcases/california_to_hawaii/run_interactive.py` witness or the
scenario bundle command; both produce the normalized `RunArtifact` shape.

## Claim boundary

Discovery, doctor, manifest validation, and bundle creation establish setup,
identity, artifact shape, and reproducibility metadata. They do not establish
numerical accuracy, source-exact historical fidelity, physical-effector
behavior, target intercept, landing, or vehicle qualification. Those claims
remain in the family-specific evidence and qualification reports.
