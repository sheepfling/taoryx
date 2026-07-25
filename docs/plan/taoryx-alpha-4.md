# TAORYX Alpha 4: parameterized experiments, backend breadth, and scale

**Status:** Planned after Alpha 3 domain qualification  
**Predecessor:** [Alpha 3 vehicle breadth and qualification expansion](taoryx-alpha-3.md)  
**Boundary:** Scale validated family and variant contracts without weakening
model validity, qualification, or evidence claims.

## Purpose

Alpha 4 is the experimentation and scale release. Alpha 2 establishes bounded
variants and Alpha 3 proves them across new vehicle families. Alpha 4 makes
those contracts useful for large parameter studies, multiple execution
backends, system identification, and AI/search workflows.

Alpha 4 does not turn arbitrary parameter mutation into a vehicle model. Every
candidate still follows:

```text
semantic decision space
    -> bounded modifiers
    -> derived physics and resource checks
    -> qualification classification
    -> segment feasibility
    -> backend execution
    -> objective vector and evidence report
```

## Alpha 4 entry conditions

Alpha 4 may begin only when:

- Alpha 2.1 bounded variant resolution is reproducible and fail-closed;
- Alpha 3 has at least three new M4 family packages;
- resource depletion, achieved-control projection, and segment outcomes are
  exercised by representative missions;
- at least one family has source-dynamic correlation and one has a hybrid mode
  transition qualification;
- objective reports distinguish validity, qualification, feasibility, and
  execution outcome; and
- the evidence packet can reproduce a candidate without hidden local state.

## Alpha 4 workstreams

### A4-W1 — Tool-agnostic trajectory parameterizer

Implement reusable `TrajectoryTemplate`, `DecisionSpace`,
`TrajectoryParameterizer`, `SegmentAssembler`, `ObjectiveSet`, and
`ObjectiveEvaluator` contracts that do not import a Taoryx integrator or native
state layout.

The parameterizer must support design, loadout, mission, segment, controller,
and step-control scopes with conditional variables, units, transforms, seeds,
and provenance. It must return structured resolution failures rather than
crashing a batch.

### A4-W2 — Backend conformance and adapter breadth

Add at least two independent backend bindings, or one production backend and a
conformance/reference backend. Compare capabilities, control intents, event
semantics, resource behavior, batch/step transitions, and objective results.
Unsupported concepts must fail during compilation; approximations must be
explicitly recorded.

### A4-W3 — Search, optimization, and robustness

Add conditional normalized decision spaces, candidate projection distance,
cache keys, deterministic seeds, multi-start tuning, Pareto and lexicographic
objectives, and large robustness campaigns. Search may vary semantic modifiers
and mission/controller decisions, never raw table cells or runtime state.

Every candidate retains its resolved variant, segment graph, controller,
backend, seed, objective vector, failure classification, and evidence hashes.

### A4-W4 — Model morphing and reduced-model generation

Support bounded smooth aerodynamic morphs, reduced-order identification, and
pseudo-6DOF parameter generation from qualified 6DOF responses. Morphs must
preserve declared symmetry, smoothness, positivity, table-domain behavior, and
qualification limits.

Geometry scaling, topology changes, new propulsion mechanisms, and arbitrary
control layouts create new family recipes rather than ordinary scalar
variants.

### A4-W5 — Corpus, fleet, and experiment operations

Generate paired 3DOF, pseudo-6DOF, and rigid-body editions with leakage-safe
truth/observation splits, population execution, checkpoint branching, and
artifact stores suitable for AI and system-identification workflows. Include
uncertainty bands and evidence grades in every corpus record.

## Alpha 4 exit gates

Alpha 4 closes only when:

- one backend-neutral trajectory template resolves through two backend
  bindings or a production/reference conformance pair;
- thousands of mixed valid, projected, infeasible, and failed candidates can
  execute without process crashes or hidden invalid physics;
- no candidate receives an improved score through negative resources,
  extrapolation, numerical failure, or unachievable commands;
- Pareto/constraint-first reports preserve the full objective vector and
  validity/qualification/feasibility/outcome classifications;
- a bounded model morph and a generated pseudo-6DOF response pass their
  qualification probes and convergence checks; and
- every corpus, fleet, and optimization result is reproducible from immutable
  variant, segment, controller, environment, backend, and software hashes.

## Explicit non-goals

Alpha 4 does not establish flight certification, proprietary aircraft truth,
global validity outside declared envelopes, historical TAOS runtime
equivalence without an oracle, or unrestricted AI control of physical state.
