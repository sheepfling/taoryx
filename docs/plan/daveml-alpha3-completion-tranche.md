# DAVE-ML Alpha 3 Completion Tranche

**Status:** execution registry active
**Scope:** finish the remaining DAVE-ML family-library integration without
collapsing source fidelity, derived exactness, surrogate composition, or
synthetic demonstration into one claim
**Claim boundary:** source-bounded research integration; not flight
qualification, manufacturer validation, or historical TAOS compatibility

## Pinned Plan

This is the pinned remaining-work plan for the current DAVE-ML tranche. The
catalog round trip, canonical export, fresh-process release gate, family
readiness records, and runtime replay for the currently promoted corpus are
complete. The remaining work is qualification depth and explicit disposition,
not another broad importer rewrite.

### Priority 1: Promote the Existing Executable Families

1. **F-16:** add multipoint operating evidence, then qualify controller,
   actuator, mission, and reduced-order overlays independently.
2. **HL-20:** add wider energy-glide and attitude-response evidence; keep the
   unpowered source boundary and do not infer controller authority from it.
3. **NESC two-stage:** complete an independent full-trajectory comparison,
   preserve staging history, and keep any deployment child as a separate
   lineage artifact.
4. **A320:** maintain the derived-exact OpenAP and surrogate-composite
   OpenAP-plus-JSBSim lanes; compare them at matched operating points without
   presenting either as an authoritative Airbus 6-DOF source.

### Priority 2: Qualify Reductions and Demonstrations

For every promoted 3-DOF or pseudo-6-DOF reduction, record the immutable
parent, omitted physics, operating-point bounds, comparison window, metrics,
pass/fail rule, and known nonclaims. Finish the synthetic NESC deployment
witness only as a demonstration of parent/child lineage and passive
aero-ballistic behavior; it is not source-equivalence evidence.

### Deferred Until Triggered by Authoritative Input

- Expand the six source-only catalog families into executable packages,
  beginning with atmospheric spheroid/brick cases and then orbital packages.
- Implement typed vector table-function semantics.
- Revisit the quarantined official 2-D ungridded interpolation case only when
  a valid authoritative fixture or an explicit legacy triangulation policy is
  available.
- Acquire and qualify an authoritative Airbus 6-DOF source package.

### Pinned Exit Conditions

The tranche is complete when the applicable F-16, HL-20, NESC, and A320 lanes
pass A3-1 through A3-7; each non-applicable or unavailable lane has an
explicit machine-readable disposition; reduction and deployment artifacts
carry lineage and nonclaims; and the deterministic release report, focused
tests, CI job, and catalog registries agree. The known semantic gaps remain
documented rather than silently waived.

## Completion Strategy

The Alpha 3 completion work is split into five independently auditable lanes:

1. **F-16 S-119:** expand source operating-point evidence, then qualify
   separately versioned actuator, controller, mission, and reduced-order
   overlays generated from the immutable rigid-body parent.
2. **HL-20 Mod K:** qualify direct-surface and logical-allocation contracts,
   retain the unpowered source boundary, and add energy-glide and
   attitude-response reductions without making controller claims from the
   source package.
3. **NESC two-stage:** preserve launch, staging, and trajectory evidence,
   compare the retained history against any independent trajectory source,
   and derive reduced models only from the pinned parent. Deployment children
   are separate lineage nodes.
4. **A320:** keep OpenAP as `derived_exact` and OpenAP plus JSBSim as
   `surrogate_composite`. The missing authoritative Airbus 6-DOF package is
   a source-acquisition gap, not a blocker for either executable lane.
5. **Synthetic deployment witness:** use the NESC parent plus a clearly
   labeled passive cylinder or spheroid child. The child has no source-
   equivalence claim and cannot change the parent qualification status.

## Gates

| Gate | Exit condition |
|---|---|
| A3-1 registry | Every remaining family layer has an owner, parent, evidence, status, and nonclaims. |
| A3-2 source-channel expansion | F-16 and HL-20 operating points identify their source hashes, envelope, residuals, and unqualified axes. |
| A3-3 overlay qualification | Actuator and allocator overlays are separate artifacts with command/achieved, bounds, and provenance evidence. |
| A3-4 reduction qualification | Each promoted reduction names its parent, omitted physics, comparison window, metrics, and pass/fail rule. |
| A3-5 trajectory and lineage | NESC staging history is source-backed; independent comparison and synthetic child behavior are separately reported. |
| A3-6 A320 comparison | Derived-exact and surrogate-composite outputs are compared at matched points without merging authority. |
| A3-7 release | Deterministic validators, focused tests, full applicable DAVE-ML gates, and provenance reports pass. |

## Explicit Non-Blockers

- LaTeX and PDF work is deferred to the Mac workflow.
- The legacy ungridded DAVE-ML fixture remains source-preserving with a
  source-hash-pinned compatibility overlay and one narrow quarantine.
- The exact Airbus-authoritative A320 6-DOF lane remains unavailable pending
  source acquisition, while the derived and surrogate family lanes continue.
- A reduction is not promoted merely because a single trace looks similar.
  Until its parent comparison passes, its registry status remains
  `contract_ready` or `equivalence_pending`.

## Execution Order

The execution agent should update the registry first, then work in parallel on
F-16 and NESC, followed by HL-20 and A320 comparison, and finally run the
release gate. Every generated artifact must point back to the canonical
repository resources tree; temporary intake locations are never valid
provenance.
