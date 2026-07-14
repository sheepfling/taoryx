# Numerical verification gates

This document states the release threshold for the numerical verification layer.
It does not replace the requirement registry; it constrains when the registry is
considered strong enough to support release claims.

## Gate sequence

| Gate | Exit criterion |
|---|---|
| N0 | Claim boundaries, exclusions, and operating scope are recorded. |
| N1 | Equation, constant, frame, and ambiguity registries have stable IDs and source links. |
| N2 | Representative equations are source-anchored by labels and independently rederived. |
| N3 | Frame conventions, singularities, and branch rules have source-located diagnostics. |
| N4 | Gravity constants and table anchors have exact source decimals and manual table labels. |
| N5 | Negative tests cover documented singularities and malformed cases. |
| N6 | Mutation checks fail when a sign, branch, or convention is flipped. |
| N7 | External review confirms unresolved ambiguities remain explicit. |
| N8 | The verification baseline passes cleanly in a pinned environment. |

## Stop-ship conditions

- A source correction is applied without an ambiguity record.
- A documented singularity lacks a source-located diagnostic.
- A gravity constant is admitted without a table anchor.
- A representative equation is used without a manual label anchor.
- A sign or branch mutation survives the focused verification suite.
- Historical-compatibility language is used without an oracle.

## Release statement

The numerical verification layer is not considered release-ready until N0 through
N8 are all satisfied for the in-scope claims. Partial progress may support
development, but it does not widen the claim boundary.

####
