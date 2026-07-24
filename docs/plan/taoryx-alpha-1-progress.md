# TAORYX Alpha 1 progress log

This is the running implementation log for the Alpha 1 release plan. It is
deliberately evidence-bounded: a green gate records the current repository
contract, not historical TAOS 96.0 compatibility or vehicle certification.

## 2026-07-24 — Alpha 1 release gate closed

The current audit is now:

```text
R0 pass  R1 pass  R2 pass  R3 pass  R4 pass  R5 pass  R6 pass  R7 pass
R8 pass  R9 pass
overall: pass; lowest unmet gate: none
```

This is a real Alpha 1 milestone: the release now has an executable proof that
new composition cases can be authored as metadata and promoted through the
standard builder, native syntax compiler, parser, and runtime. It does not
imply historical TAOS compatibility.

The X-15 trim tranche is now explicit and fast. The common trim solver uses a
single loaded program and a cached rigid-body residual evaluator rather than
rebinding tables for every least-squares evaluation. The resulting report is
`artifacts/golden_plants/x15_release_glide_trim_report.json`: it reaches the
declared alpha lower bound with a normalized residual of approximately 0.792,
so the equilibrium gate remains blocked. This is the correct result for the
current source evidence: the adjudication record says that the source release
state is a finite, nonzero-wrench glide initialization, not a static trim.
The report therefore preserves `source_only` and excludes an equilibrium
claim instead of forcing a false trim.

The Alpha 1 R7 scope is now recorded separately in
`verification/alpha1_vehicle_evidence.yaml`. R7 covers source-backed plant and
convention evidence for all four families; stronger route/controller cases
remain visible as deferred blockers in `family_validation_execution.yaml` and
do not silently become Alpha 1 claims. The release audit validates this
narrower, explicit scope.

R8 is now closed by `dist/taoryx-alpha1-evidence-v1.zip`. The packet contains
the four manual inputs, Alpha 1 registries, source tables, baseline problems,
trim/source-differential reports, long-scenario reports, plots, and the
sanitized `bundle-manifest.json`. The manifest records both source and packet
hashes; the audit rejects missing ZIP members and workstation-local paths.

R9 is now closed by `dist/taos-manual-codex-handoff-v21.zip`. The handoff
contains the source/tooling tree, the normalized manual and equation audit,
the current TAORYX extension PDFs, and the installable wheel. Its builder now
excludes the local virtual environment, scratch directories, and the large
generated trajectory-artifact tree; those generated vehicle results remain in
the dedicated Alpha 1 evidence packet. The handoff is 20,733,247 bytes with
SHA-256 `4ab4f56d238e82daa9af1bf23aefa5052a13fcfa22872376c65a1422a1d6b3d8`.
The Alpha 1 evidence packet is 52,783,321 bytes with SHA-256
`aa76825b07afcd96964770796d9f2522447f6d8dbfbff42782cf203af39e4f29`.

The completed release claim remains deliberately bounded. Alpha 1 closes the
documented language/runtime/composition/fidelity contracts and source-backed
baseline evidence for B747, Skywalker X8, Hummingbird, and X-15. It does not
claim historical TAOS 96.0 compatibility, global vehicle validity, flight
qualification, or successful higher-order B747/X8/X-15 missions. Those remain
the next development tranche.

## 2026-07-23 — release-gate baseline

Completed:

- Added the Alpha 1 plan and machine-readable release inventory.
- Added `tools/audit_alpha1_release.py` and the `audit-alpha1` development task.
- Added the generated 80-row `verification/alpha1_feature_matrix.yaml`.
  The matrix covers 33 Chapter 4 scoped blocks, 19 Chapter 3 table types, and
  28 Chapter 3 table operations. Each row links grammar, semantic registry,
  implementation owner, fixture cases, tests, and a requirement group.
- Added the four-family manual-example execution report runner:
  `python tools/dev.py alpha1-manual-examples`.
- Added the metadata-driven new-case composition proof:
  `python tools/dev.py alpha1-composition-case`. It resolves a two-segment
  vehicle case through `TrajectoryBuilder` and the standard template registry,
  lowers it to native problem syntax, parses it, and executes it through the
  common runtime. The report records generated hashes, the resolved case, and
  the run artifact without a vehicle-specific runner.
- Added focused tests for the release audit and feature matrix.
- Required repository checks passed after the tranche: manual rebuild,
  equation audit, `tools/dev.py check`, and standalone pytest.

Current gate status:

| Gate | Status | Evidence-bounded meaning |
| --- | --- | --- |
| R0 | pass | Claims, exclusions, and release inventory are recorded. |
| R1 | pass | The bounded language surface has a generated feature matrix. |
| R2 | pass | Current positive corpus coverage is 33/33, 19/19, and 28/28. |
| R3 | pass | The canonical runtime, public step path, and parity contracts exist. |
| R4 | pass | All four manual example families have claim-bounded companion executions; source optimization qualification remains separately reported. |
| R5 | pass | A new case is metadata-resolved through the standard composition registry, lowered to native syntax, parsed, and run through the common runtime. |
| R6 | pass | 3-DOF, pseudo-6-DOF, and rigid-body contracts have ladder tests. |
| R7 | pass | `verification/alpha1_vehicle_evidence.yaml` records source-backed baseline plant evidence for all four families; higher-order route/controller blockers remain deferred. |
| R8 | pass | The self-contained, path-sanitized Alpha 1 evidence packet and manifest agree. |
| R9 | pass | The current handoff ZIP, final products, and checksums were produced successfully. |

The generated report is written to:

`artifacts/verification/alpha1/alpha1-status.json`

The manual-example report is written to:

`artifacts/verification/alpha1/manual_examples/report.json`

The composition proof report is written to:

`artifacts/verification/alpha1/composition_case/report.json`

The current report intentionally retains these deferred blockers:

- `p033_manual_ballistic_rocket_synthetic`: the declared synthetic tables do
  not satisfy the optimization equality endpoint;
- `p034_manual_air_intercept_synthetic`: the fixed optimization trajectory does
  not reach its qualified endpoint;
- vehicle-specific route/controller blockers remain outside the Alpha 1
  baseline-plant claim and must be resolved before stronger mission claims are
  made.

## Post-Alpha tranche

1. Decide whether the two synthetic optimization cases should receive coherent
   source-bounded stand-in tables or remain parser/runtime-blocked fixtures.
2. Continue the B747/X8 route and X-15 terminal blockers as post-Alpha mission
   work; do not widen the Alpha 1 claim matrix while doing so.

## Reproduce the current log

```bash
python tools/dev.py alpha1-feature-matrix
python tools/dev.py alpha1-manual-examples
python tools/dev.py alpha1-composition-case
python tools/dev.py alpha1-packet
python tools/dev.py handoff
python tools/dev.py audit-alpha1
python -m pytest tests/unit/test_alpha1_release_audit.py tests/unit/test_alpha1_feature_matrix.py tests/unit/test_alpha1_composition_case.py
```
