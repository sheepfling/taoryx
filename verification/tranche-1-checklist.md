# Tranche 1 execution checklist

This checklist converts the first verification tranche into a gap-driven work
list. Each item is tied to an existing baseline gap so progress is evidence
based rather than aspirational.

## 1. Anchor vectors and handedness

Close the remaining anchor-vector review gap.

Current evidence:

- `verification/spec/anchors.yaml`
- `verification/spec/frame_matrices.yaml`
- `verification/spec/sign_matrices.yaml`
- `tests/unit/test_verification_baseline.py`
- `tests/unit/test_coordinates.py`
- `tests/unit/test_frames_equations.py`

Remaining gap:

- independent reviewer approval for anchor vectors, handedness, frame conventions, and sign conventions

Acceptance evidence:

- reviewer approval recorded in the baseline or review log;
- mutation coverage that flips sign, axis, or branch conventions and fails.

## 2. Exact constants and external source-page audit

Close the remaining constant provenance gap.

Current evidence:

- `verification/spec/constant_matrices.yaml`
- `verification/spec/constants.yaml`
- `tests/unit/test_gravity.py`
- `tests/unit/test_gravity_equations.py`
- `tests/unit/test_atmosphere.py`
- `tests/unit/test_atmosphere_equations.py`
- `tests/unit/test_verification_baseline.py`

Remaining gap:

- external reviewer approval for the constant provenance matrix

Acceptance evidence:

- each constant is checked against the manual page and table anchor;
- the source decimal remains exact in the registry;
- any source/runtime mismatch stays in an explicit ambiguity record.

## 3. Branch behavior and diagnostic matrices

Close the remaining specific-error-matrix gap for branch and singularity cases.

Current evidence:

- `verification/spec/diagnostic_matrices.yaml`
- `manual/chapters/chapter02/01_07_body_fixed.tex`
- `manual/chapters/chapter02/01_09_wind.tex`
- `manual/chapters/chapter02/01_11_tangent_plane.tex`
- `tests/unit/test_frames_equations.py`
- `tests/unit/test_coordinates.py`
- `tests/unit/test_verification_baseline.py`

Remaining gap:

- external reviewer approval for the branch/singularity and diagnostic matrices

Acceptance evidence:

- one source-located diagnostic per documented branch/singularity class;
- negative tests that assert the diagnostic text or code references the source
  passage or ambiguity record.

## 4. Requirement coverage for tranche 1

Make the tranche traceable through the requirement registry.

Current evidence:

- `verification/spec/ambiguity_matrices.yaml`
- `verification/spec/chapter4_normative_inventory.yaml`
- `verification/spec/equation_matrices.yaml`
- `verification/spec/matrix_inventory.yaml`
- `verification/spec/mutation_matrices.yaml`
- `verification/spec/review_inventory.yaml`
- `verification/spec/requirements.yaml`
- `verification/traceability.yaml`
- `verification/spec/regression_matrices.yaml`
- `tests/unit/test_verification_baseline.py`

Remaining gap:

- traceability entries still marked partial for the tranche-1 requirements and
  reviewer approval remains outstanding

Acceptance evidence:

- tranche-1 requirement IDs mapped to the corresponding tests and registries;
- unresolved gaps remain visible rather than being hidden behind an overall
  green status.

## Suggested order

1. reviewer approval for anchor vectors;
2. frame matrix review;
3. sign matrix review;
4. branch/singularity matrix review;
5. constant provenance matrix review;
6. equation derivation matrix review;
7. regression matrix review;
8. diagnostic matrix review;
9. ambiguity matrix review;
10. Chapter 4 inventory review;
11. reviewer inventory review;
12. mutation matrix review;
13. inventory review;
14. traceability updates for the tranche-1 requirement set.

####
