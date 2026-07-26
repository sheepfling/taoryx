# DaveML Family Operational Qualification Tranche

**Status:** baseline complete; multi-point envelope expansion is the follow-on
**Scope:** family-library contracts, certified operating points, applicable
linearization and tuning, objective/scenario runtime, and release evidence
for the promoted DaveML families
**Out of scope:** LaTeX, B747/X-15 parallel work, manufacturer-authoritative
A320 source acquisition, flight certification, and promotion of source-only
NESC records

This tranche is the next executable phase after the initial DaveML round-trip
and showcase work. The round-trip pipeline establishes that source documents
can be preserved, represented in a deterministic semantic IR, exported, and
fresh-process re-imported. This tranche proves that the resulting family
artifacts integrate into the Taoryx library without silently turning static
DaveML functions into dynamics, controllers, missions, or objectives.

## Governing Claim Boundary

Every artifact carries its source and qualification class:

| Family | Qualification | Operational boundary |
|---|---|---|
| F-16 S-119 | `reference_exact` | Source-bounded aerodynamic, propulsion, inertia, atmosphere, trim, and controller-overlay evidence; not flight qualification |
| HL-20 Mod K | `reference_exact` | Source-bounded lifting-body glide and entry evidence; no controller or actuator claim |
| NESC two-stage | `reference_exact` | Open-loop launch, staging, and trajectory evidence; trim and tuning are not applicable |
| A320 OpenAP | `derived_exact` | Exact reproduction of the pinned public OpenAP performance source; no manufacturer-truth claim |
| A320 OpenAP + JSBSim | `surrogate_composite` | Deliberate public-source composite with an authority map; no source-exact Airbus claim |

Synthetic children remain `synthetic` and must retain separate lineage from
their source-backed parent.

## Current Execution State

| Gate | Current state | Evidence |
|---|---|---|
| OQ-1 typed family contracts | verified for all five lanes | `verification/daveml_operational_contracts.json` |
| OQ-2 operating-point catalog | verified baseline for four applicable lanes plus NESC checkpoint | `verification/daveml_operating_point_catalog.json` |
| OQ-3 linearization and tuning | existing applicable F-16 and A320 evidence is included; envelope expansion remains follow-on | `verification/daveml_release_gate.json` |
| OQ-4 objective/scenario runtime | existing clean-process smoke and trajectory evidence is included | `verification/daveml_release_gate.json` |
| OQ-5 cross-family release gate | verified with 26 stages and 26 hashed artifacts | `verification/daveml_release_gate.json` |

The baseline catalog intentionally records the next expansion axes rather than
claiming envelope coverage. The next implementation increment is multi-point
F-16 operating-point evidence, followed by wider HL-20 glide and matched A320
comparison points.

## Work Packages

### OQ-1: Typed Family Contract Closure

Complete and validate one integration manifest per promoted family. Each
manifest binds:

- aerodynamic coefficients, derivatives, geometry, frames, and schedules;
- propulsion, mass flow, ignition, cutoff, staging, and throttle behavior;
- mass properties, center of gravity, inertia, and separation events;
- atmosphere, gravity, Earth model, units, and validity envelope;
- state, control, output, telemetry, and capability mappings; and
- source hashes, package identity, provenance, and nonclaims.

The loader must fail closed on hash drift, missing roles, unsupported units,
ambiguous frames, and out-of-envelope inputs.

**Exit:** all promoted families load through typed manifests and produce a
contract report with zero unresolved required bindings.

### OQ-2: Certified Operating-Point Catalogs

Add deterministic operating-point catalogs with source/package hashes,
environment, state, controls, bounds, solver settings, seeds, residual
definitions, validity envelopes, and independent residual checks.

- **F-16:** expand beyond the existing certified point across speed, altitude,
  angle of attack, and load factor.
- **HL-20:** add glide and entry points using the explicit mass, atmosphere,
  and load overlays; do not create controller evidence.
- **NESC:** add staging and trajectory checkpoint operating records; retain
  `not_applicable_no_declared_controls` for trim.
- **A320:** compare matched OpenAP and pseudo-6DOF points while preserving
  `derived_exact` and `surrogate_composite` as separate products.

**Exit:** each applicable family has at least one reproducible certified
operating point, and every unavailable dimension has an explicit disposition.

### OQ-3: Linearization And Tuning Qualification

Generate source-linked local linearizations around certified operating points,
including perturbations, state/control ordering, signs, frames, conditioning,
and finite-difference or analytic provenance.

- F-16 receives multi-point linearization, direction probes, bounded LQR, and
  gain-scheduling evidence.
- A320 pseudo-6DOF may receive controller and tuning overlays only when the
  authority map and surrogate claim are repeated in every report.
- HL-20 remains `not_applicable_no_declared_controls` unless a separate
  external control overlay is deliberately introduced.
- NESC remains `not_applicable_current_open_loop_contract`.

**Exit:** every promoted controller references a certified operating point,
passes direction probes, and has bounded command and conditioning evidence.

### OQ-4: Objective, Scenario, And Runtime Contracts

Expose family-specific objectives and scenarios through clean-process runtime
contracts:

- F-16 trim capture, maneuver, attitude hold, and energy-management smoke;
- HL-20 glide/entry and terminal-condition smoke without controller claims;
- NESC launch, staging, trajectory checkpoint, and parent-child deployment;
- A320 matched performance and bounded pseudo-6DOF comparison scenarios.

Each scenario declares channels, units, targets, tolerances, validity,
termination behavior, source/package identity, and evidence artifacts. Runtime
selection must fail closed on stale provenance, unsupported extrapolation, and
missing trim/controller contracts.

**Exit:** a clean process loads each family/fidelity lane, executes its
applicable scenario, scores its objectives, and emits a provenance-complete
artifact.

### OQ-5: Cross-Family Release Gate

Add a single deterministic release report over OQ-1 through OQ-4. It must
include source and package hashes, contract status, operating-point records,
linearization/tuning reports when applicable, objective scores, runtime
replay, showcase links, typed diffs, and explicit nonclaims.

Run the gate twice and compare manifests and report hashes. The release gate
must distinguish a source-exact failure, a derived-product failure, a
surrogate-composite failure, a synthetic-child failure, and a non-applicable
layer.

**Exit:** all applicable gates pass, all non-applicable gates have evidence,
and no generated artifact can overwrite immutable source authority.

## Execution Order

1. OQ-1 family contracts and fail-closed loader checks.
2. OQ-2 operating-point catalogs, starting with F-16 and HL-20.
3. OQ-3 F-16 multi-point tuning and A320 surrogate overlay qualification.
4. OQ-4 family runtime objectives and scenarios, including NESC lineage.
5. OQ-5 deterministic release gate and CI publication.

The vector table-function gap and quarantined legacy ungridded fixture remain
separate semantic work. They are resumed only when an authoritative corpus
member requires those semantics; they do not justify weakening this tranche's
family-library claims.

## Required Artifacts

- typed family integration manifests and hash reports;
- operating-point catalogs and independent residual reports;
- source-linked linearization and tuning reports when applicable;
- objective and scenario contracts with clean-process evidence;
- family-level typed diff and provenance reports;
- deterministic cross-family release report; and
- updated readiness and layer-disposition registries.

## Completion Definition

This tranche is complete when OQ-1 through OQ-5 pass for the applicable
F-16, HL-20, NESC, and A320 lanes, with explicit non-applicable dispositions,
deterministic artifacts, focused tests, and a committed/pushed release report.
