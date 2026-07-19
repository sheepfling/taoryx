# Verification Baseline

This directory records what taoryx is allowed to claim and what evidence supports
each claim. It is deliberately separate from the manual reconstruction and from
the executable implementation.

`baseline-v1.md` is the human-readable entry point. The YAML files under
`spec/` are the machine-readable seed registries for requirements, ambiguities,
coordinate frames, and constants. They are not a replacement for the canonical
manual metadata; they link to it.

`numerical_goal.md` states the operational goal for the derivation and
verification work: source-linked, unit-checked, ambiguity-preserving numerical
behavior with explicit diagnostics.

`tranche-1.md` is the first concrete work slice for that goal. It focuses on
coordinate conventions, exact constants, branch behavior, and source-located
diagnostics.

`tranche-1-checklist.md` turns that slice into a gap-driven execution list tied
to the current baseline and traceability state.

`spec/branch_matrices.yaml` records the explicit branch and singularity cases
that the numerical layer must preserve as source-located evidence.

`spec/constant_matrices.yaml` records family-level constant provenance for the
Earth and atmosphere models so the registry can be audited against the manual
tables and source text.

`spec/equation_matrices.yaml` records the topic-level derivation groups for the
equations that are independently rederived in the numerical layer.

`spec/regression_matrices.yaml` records the source-linked regression anchors for
the canonical numeric scenarios and the manual snippet corpus.

`spec/diagnostic_matrices.yaml` records source-linked diagnostic families for
malformed and ambiguous cases so recovery behavior stays explicit.

`spec/ambiguity_matrices.yaml` groups the unresolved and provisionally resolved
ambiguity records into source-located families.

`spec/sign_matrices.yaml` records the sign and orientation conventions that are
asserted independently of the frame and branch matrices.

`spec/frame_matrices.yaml` groups the reviewed frame cards into explicit
verification families tied to the anchor vectors.

`spec/matrix_inventory.yaml` is the master index of the matrices, their
purposes, and their requirement links.

`spec/chapter4_normative_inventory.yaml` is the chapter-level inventory for
Chapter 4 normative coverage and requirement IDs. It is split along the manual's
real section families: file format, segment blocks, trajectory blocks, problem
blocks, output/search blocks, and example scenarios.

`spec/review_inventory.yaml` records the reviewer roles required for each major
matrix and inventory artifact.

`spec/mutation_matrices.yaml` records the mutation families that must fail under
the verification baseline.

`gates.md` is the release threshold for the numerical verification layer. It
states what must be true before the layer is considered strong enough for a
release claim.

`acceptance/robustness_matrix_v1.yaml` is the executable claim boundary for
the paired slower-vehicle examples. Run it with `python tools/dev.py
robustness-matrix`; the ignored report is written below
`artifacts/verification/robustness_matrix_v1/`. It records nominal completion,
RK4 half-step convergence, declared-envelope filtering, bounded perturbation
pass rates, worst-case records, and failure classifications. The CA-HI case is
intentionally evidence-only until it has a declared endpoint requirement and
an independent route oracle.

The current baseline supports documentary fidelity (D), semantic coherence (S),
and bounded parser conformance (P) in stated areas. It does not establish
historical compatibility (H) or engineering validity. The historical TAOS
scope is the three-degree-of-freedom point-mass model; taoryx also contains a
separately labeled native 6-DOF extension exercised by public research
surrogates, without claiming that extension was part of TAOS 95/96.

####
