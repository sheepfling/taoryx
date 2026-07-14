# Numerical verification goal

Workstream objective: build an evidence-bounded numerical verification layer
for TAOS 1995 that independently derives, checks, and records the equations,
constants, frame conventions, and boundary behaviors documented in the manual,
with source-linked tests and diagnostics that preserve ambiguity instead of
silently resolving it.

This is a verification layer, not a silent repair pass. It must preserve source
form, adopted form, and ambiguity records separately; attach stable source links
to every adopted numerical rule; and produce source-located diagnostics for
malformed, singular, or ambiguous cases.

The layer must:

- keep every normative numerical requirement tied to a stable source location;
- validate units, dimensions, sign conventions, branch conditions, defaults,
  and singularities before the implementation is treated as accepted;
- preserve unresolved or disputed cases as explicit ambiguity records rather
  than resolving them implicitly;
- attach positive and negative tests to each numerical requirement;
- prove that sign flips, branch flips, and other seeded mutations fail;
- keep TAOS 96.0 historical-compatibility language separate and unasserted
  unless a historical executable, source, or trusted output corpus exists;
- make recovery behavior part of the evidence record instead of collapsing it
  into generic failure.

Claims tracked by this goal:

- Documentary fidelity: the reconstructed 1995 manual matches the source scan
  and editorial record.
- Semantic coherence: the equations, constants, signs, branches, and frame
  conventions are internally consistent once adopted.
- Parser conformance: the `.tbl` and `.prb` language accepts and rejects only
  the documented forms, with source-located diagnostics for malformed input.
- Numerical correctness: the mathematical kernels reproduce the adjudicated
  TAOS 1995 derivations and boundary behavior.
- Historical compatibility: TAOS 96.0 behavior is only claimed when an actual
  historical oracle exists.

This goal excludes silent normalization of disputed equations or conventions.
It also excludes widening the scope into a six-degree-of-freedom vehicle model
or claiming historical compatibility without an oracle.

Done means:

1. the relevant requirements are registered with stable IDs and source links;
2. the derivations are reproducible from the source-linked registry and can be
   independently audited;
3. boundary, recovery, and mutation tests fail when a sign, branch, or
   convention is flipped;
4. every unresolved ambiguity is explicit in the registry;
5. the accepted numerical behavior is traceable to the manual or to a recorded
   external authority;
6. no TAOS 96.0 compatibility claim is made without a matching historical
   oracle.

Recommended execution order:

1. lock the claim boundaries, exclusions, and evidence policy;
2. register the equations, constants, frames, and ambiguities with stable IDs;
3. derive and test the coordinate and sign conventions first;
4. verify constants and gravity models against source-linked fixtures;
5. add branch, boundary, and mutation tests for aerodynamic and guidance cases;
6. expand into integrated scenario tests only after the local numerical pieces
   are source-anchored.

####
