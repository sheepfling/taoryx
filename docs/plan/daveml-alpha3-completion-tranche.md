# DAVE-ML Alpha 3 Completion Tranche

**Status:** execution registry active
**Scope:** finish the remaining DAVE-ML family-library integration without
collapsing source fidelity, derived exactness, surrogate composition, or
synthetic demonstration into one claim
**Claim boundary:** source-bounded research integration; not flight
qualification, manufacturer validation, or historical TAOS compatibility

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
