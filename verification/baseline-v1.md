# Verification Baseline v1

Status: Phase 0 governance baseline; review required before a release claim is widened.

## Product boundary

TAOS is treated here as a three-degree-of-freedom point-mass simulator. Vehicle
attitude is an input/result of guidance and aerodynamic/propulsive models. This
baseline does not verify rotational dynamics, moments, inertia tensors, or a
six-degree-of-freedom vehicle model.

## Claims

| ID | Claim | Current status | Evidence required |
|---|---|---|---|
| D | Documentary fidelity to the 1995 manual | partial, section-scoped | source-page review, transcription checks, editorial ledger |
| S | Semantic coherence of adopted equations, constants, defaults, and restrictions | partial | dimensional checks, derivations, contradiction review, registries |
| P | Parser conformance for the documented and explicitly bounded `.tbl`/`.prb` surface | partial, evidence-bounded | positive/negative fixtures, diagnostics, recovery, traceability |
| N | Numerical correctness of an adopted mathematical kernel | partial, function-scoped | independent implementation, high-precision references, properties, mutation tests |
| H | Historical compatibility with TAOS 96.0 | not established | historical executable/source or trusted output corpus |

“Manual-compatible numerical implementation” is not a historical compatibility
claim. Any Class D ambiguity affecting normal execution blocks that claim until a
mode or explicit exclusion is implemented.

## Evidence authorities

Three authorities are kept separate:

- Documentary: rendered source scan, reviewed visual readings, extracted text as a
  search aid, then reconstructed LaTeX.
- Historical behavior: original source, known historical executable, original
  output/test records, manual descriptions, then inference.
- Scientific: primary standard or original paper, official reference
  implementation, manual, secondary source, then inference.

Conflicts are recorded; they are not silently normalized. Documentary and
scientifically adjudicated forms may coexist as `source_form` and
`adopted_form`.

## Baseline artifacts

- [Numerical verification goal](numerical_goal.md)
- [Numerical verification gates](gates.md)
- [Claims and exclusions](claims.md)
- [Evidence policy](evidence_policy.md)
- [Review policy](review_policy.md)
- [Ambiguity registry](spec/ambiguities.yaml)
- [Requirement registry](spec/requirements.yaml)
- [Coordinate-frame cards](spec/frames.yaml)
- [Independent anchor vectors](spec/anchors.yaml)
- [Tolerance policy](spec/tolerances.yaml)
- [Constant/model registry](spec/constants.yaml)
- [Initial traceability](traceability.yaml)

## Acceptance gates

| Gate | Exit criterion |
|---|---|
| G0 | Release claims and exclusions are written |
| G1 | Every in-scope P0 source passage has a stable requirement ID |
| G2 | Every known inconsistency has an ambiguity record |
| G3 | Frame, sign, unit, and angle anchor tests pass independently |
| G4 | Earth/gravity constants have exact source decimals and benchmark evidence |
| G5 | Normative language rules have linked positive/negative tests |
| G6 | Canonical integrated scenarios pass with justified tolerances |
| G7 | Role-specific reviewers approve P0 items |
| G8 | Clean pinned-environment build and test are reproducible |

Stop-ship examples: silent source correction, unresolved handedness, unproven
constant, parser behavior without a requirement link, surviving sign/branch
mutation, missing inherited-state cycle detection, or a historical claim without
an oracle.

## Current phase

The repository has completed the governance inventory and the frame-card/tolerance
scaffold. Existing parser, equation, corpus, and runtime tests are evidence inputs,
not blanket proof of the five claims. Independent anchor vectors now cover ECFC,
local-horizon, ECIC, wind, body, and tangent-plane cases, but handedness review,
mutation review beyond the basic sign/axis sentinels, and scenario-specific
tolerances remain before Phase 1 can exit. The constant/model registry now has
seeded exact-decimal entries for WGS-84, WGS-72, the TSAP zonal constants, the
full WGS-84 degree-4 coefficient table, and atmosphere constants, and the
verification baseline now asserts the exact source page/table anchors for those
gravity constants. The branch/singularity matrix now records the explicit
evidence cases for aerodynamic, velocity, geodetic, and geocentric boundary
behavior, and the baseline now ties that matrix back to a dedicated
requirement/traceability entry. The constant provenance matrix now records the
family-level source tables and decimal sets for Earth and atmosphere constants,
and the baseline ties that matrix to a dedicated requirement/traceability entry,
but external review is still pending.
The equation derivation matrix now records the topic-level grouping for the
coordinate, aerodynamic, atmosphere, geodesy, gravity, and optimization
families, and the baseline ties that matrix to a dedicated
requirement/traceability entry.
The regression matrix now records the source-linked canonical scenarios and the
manual snippet corpus, and the baseline ties that matrix to a dedicated
requirement/traceability entry.
The diagnostic matrix now records the source-linked malformed and ambiguous case
families, and the baseline ties that matrix to a dedicated
requirement/traceability entry.
The ambiguity matrix now groups the unresolved and provisionally resolved
ambiguity records into source-located families, and the baseline ties that
matrix to a dedicated requirement/traceability entry.
The sign matrix now records the source-linked orientation and coefficient-sign
families, and the baseline ties that matrix to a dedicated
requirement/traceability entry.
The frame matrix now groups the reviewed frame cards and anchor vectors into
explicit verification families, and the baseline ties that matrix to a
dedicated requirement/traceability entry.
The matrix inventory now lists the full set of explicit verification matrices
and their requirement links, giving the baseline a single audit index for the
evidence structure.
The Chapter 4 normative inventory now maps the manual's file-format, segment,
trajectory, problem, output/search, and example families to requirement IDs so
the P0 coverage is represented as a structured section inventory rather than a
prose checklist alone.
The reviewer inventory now records the required reviewer roles for each major
matrix and inventory artifact, so the remaining approval work is also
structurally explicit.
Taken together, the chapter inventory and traceability work are complete; the
remaining open item is external reviewer approval, not missing registry
coverage.
The mutation matrix now groups the defect families that the baseline must reject
when signs, branches, coefficients, or diagnostics are flipped or erased.
The baseline test suite now explicitly asserts every matrix registry entry in
the source-linked verification layer, and the runtime-lowering module test
includes an autouse parser-state reset to avoid suite-order cross-talk in the
multi-problem output path.

## Phase 0 exit checklist

- [x] Five claims and their exclusions are written.
- [x] Documentary, historical-behavior, and scientific authorities are separated.
- [x] Ambiguity records exist for known equation and coordinate-convention issues.
- [x] Historical optimizer compatibility is explicitly excluded without an oracle.
- [x] All 11 documented Chapter 2 coordinate systems have verification cards.
- [x] A scoped numeric tolerance policy is recorded.
- [x] A constant/model registry seed with exact decimal strings is recorded.
- [x] Fixed ECFC, local-horizon, ECIC, wind, body, and tangent-plane anchor vectors are recorded.
- [x] Anchor vectors have full mutation coverage.
- [ ] Anchor vectors have independent reviewer approval.
- [x] Every P0 Chapter 4 normative passage has a requirement ID.
- [x] Every Earth/gravity constant has an exact source decimal and baseline page/table anchor check.
- [ ] Role-specific reviewers have approved the baseline.

####
