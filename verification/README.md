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

`fidelity_ladder.yaml` and `tests/e2e/test_fidelity_ladder.py` define the
common four-family progression: source 3-DOF, derived kinematic 3+3-DOF, and
source-anchored rigid-body 6-DOF. The kinematic tier must preserve the same
translational history as its source 3-DOF case while publishing a normalized
attitude sidecar; it is a development bridge, not a full rigid-body claim.

`fidelity_parity.yaml` is the stricter shared-experiment catalog. It records
the physical inputs that must be identical before a 3DOF/3+3/6DOF comparison
is called reduction parity. `python tools/dev.py check-parity` validates all
problem/table inputs and writes the hashed generated contract at
`verification/generated/fidelity_parity_contracts.json`. All four families now
have explicit candidate contracts; a candidate may still fail its execution
gate. Candidate execution currently stops at the unit firewall:
the point-mass artifacts still expose native FPS-style telemetry while the
rigid-body artifacts expose SI telemetry. That transcode must be made explicit
before a parity result can be marked pass. The parity runner now performs that
transcode in its report; the B747, X8, and Hummingbird 0.1-second
translational windows pass initial-state, history-difference, and continuity
gates. The X-15 release-glide window is 0.01 seconds because its high-speed
initial acceleration is more strongly conditioned; its explicit tolerance is
recorded in the catalog. These are reduction-window results, not
long-duration or engineering-validity claims. The long X-15 release-glide
mission remains separate fidelity-separation evidence: over that horizon the
free rigid-body trajectory diverges from the point-mass reduction as attitude
and rate dynamics are released.

`staged_completion_matrix.yaml` is the per-vehicle completion ledger for that
progression. It names the source dataset, problem or derivation for each tier,
the recovery and long-validation cases, and any remaining blockers. Keep a
vehicle at its lowest failing tier: a passing bridge or short propagation does
not promote the corresponding rigid-body or mission claim. Validate the ledger
with `tests/unit/test_staged_completion_matrix.py` before packaging evidence.

The reproducible packet entrypoint is `python tools/dev.py fidelity-packet`.
It creates a UUID-scoped directory and ZIP containing the resolved source
problems, table inputs, tier summaries, and SHA-256 manifest for all four
families. The packet is evidence of staged execution and provenance; it does
not promote a family to engineering validity when a lower-tier or source
differential gate remains open.

The current baseline supports documentary fidelity (D), semantic coherence (S),
and bounded parser conformance (P) in stated areas. It does not establish
historical compatibility (H) or engineering validity. The historical TAOS
scope is the three-degree-of-freedom point-mass model; taoryx also contains a
separately labeled native 6-DOF extension exercised by public research
surrogates, without claiming that extension was part of TAOS 95/96.

####
