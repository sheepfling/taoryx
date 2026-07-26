# Taoryx ↔ DAVE-ML round-trip workstream

This workstream treats DAVE-ML as an interchange format, the Taoryx
collection as the editable semantic authority, and `.txair` as a derived
runtime artifact.

```text
DAVE-ML source documents
        ↓
loss-aware source identities
        ↓
Taoryx .txcollection
        ↓
compiled .txair
        ↓
regenerated DAVE-ML
        ↓
fresh import and equivalence report
```

## Current implementation status

| Milestone | Status | Evidence |
| --- | --- | --- |
| M0 collection schema and deterministic container | Complete for F-16 fixture | `src/taoryx/trajectory/collections.py` |
| M1 lossless source layer | Complete for local F-16 inputs | Exact source hashes and package members are embedded by the builder |
| M2 multi-document binder | F-16 and HL-20 initial slices complete | `tools/build_daveml_collection.py` |
| M3 source-preserving exporter | Complete for embedded source bytes | `tools/roundtrip_daveml_collection.py` reports L0/L1 |
| M3 canonical DAVE-ML exporter | In progress | Deterministic source-anchored semantic IR and exporter are implemented in `src/taoryx/trajectory/daveml_semantic.py` |
| M4 round-trip verifier | Source-layer, collection, package-runtime, and fresh-import structural gates in progress | Canonical export and fresh-process structural/numeric-literal comparison are implemented for collections and `.txair` packages; graph evaluation remains separate |
| M5 four-model qualification | Pending | F-16 and HL-20 collection fixtures are now available; rocket and A320 later |
| M6 offline CI integration | Pending | Requires stable M3–M5 reports |

## Standard collection contracts

`CollectionManifest`, `SourceDocument`, `ComponentBinding`,
`ContributionAuthority`, `TransformRecord`, and
`StatefulComponentContract` are provider-neutral. They preserve source IDs
and hashes separately from canonical IDs and declare who owns each physical
contribution.

The deterministic ZIP writer fixes member ordering, timestamps, permissions,
and the checksum ledger. It does not rewrite source DAVE-ML bytes.

## F-16 first fixture

Rebuild from the external corpus with:

```text
python tools/build_daveml_collection.py \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output build/f16-s119.txcollection
```

The first fixture contains:

- `aerodynamics.dml`, `propulsion.dml`, and `inertia.dml` source bytes;
- the verified `.txair` runtime artifact;
- source identity and hash records;
- control and runtime bindings;
- contribution ownership and frame transforms;
- 29 source validation cases: 17 aerodynamic, 3 mass-property, and 9 propulsion;
- trim-hold telemetry and an explicit dynamics sidecar;
- a pending canonical/runtime L0–L4 round-trip report.

The generated collection is a derived external artifact. Source distribution
remains governed by the corpus policy; the repository stores the builder,
schemas, and provenance contracts rather than silently copying the external
corpus into the source tree.

## Source-preserving round trip

Run the source-layer check against either collection:

```text
python tools/roundtrip_daveml_collection.py \
  --collection build/f16-s119.txcollection \
  --output-dir build/f16-s119-daveml
```

The command validates the archive checksum ledger, exports the exact embedded
DAVE-ML bytes, reloads those bytes in a fresh file read, and writes
`roundtrip-report.json`. A verified report means L0 byte preservation and L1
source identity, not canonical DAVE-ML regeneration or runtime replay.

Build the second source-grounded fixture with:

```text
python tools/build_daveml_collection.py \
  --family-manifest families/reference_hl20_mod_k/family.yaml \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output build/hl20-mod-k.txcollection
```

HL-20 deliberately records the aerodynamic DAVE-ML source separately from its
fixed mass-property binding. Its collection also preserves the generated
tables and validation evidence under `evidence/`; this is a reference-anchor
boundary, not a claim that those engineering overlays are contained in the
source DAVE-ML.

## Next gates

1. Preserve original DAVE-ML XML identity and unknown extension nodes in a
   lossless source representation.
2. Implement canonical DAVE-ML regeneration with generated IDs and provenance.
3. Re-import generated documents and compare structure, check cases, knots,
   boundaries, and seeded interior samples.
4. Run `tools/canonical_daveml_roundtrip.py` for each promoted collection and
   retain the typed structural/provenance report.
5. Add evaluator-backed numeric replay comparison for seeded values, boundaries,
   and source check cases; the current verifier covers literal preservation.

The collection-level runtime handoff is now covered by
`tools/replay_daveml_collection.py`. It verifies the `.txcollection` checksum
ledger and manifest first, then replays the declared embedded `.txair` artifact
with collection and family provenance in the report. This is still distinct
from canonical DAVE-ML regeneration and semantic equivalence.

## Ingestion And Round-Trip Plan

The machine-readable promotion authority is
`verification/daveml_roundtrip_promotion.yaml`; this document defines the
workflow and claim boundary, while the registry records current rung status.

The target is a repeatable cycle, not a one-time converter:

```text
INBOX source + catalog
        |
        v
verify acquisition and hashes
        |
        v
lossless source package + canonical semantic IR
        |
        v
Taoryx .txcollection
        |
        v
compiled .txair + host/runtime bindings
        |
        v
runtime replay and qualified evidence
        |
        v
canonical DAVE-ML export
        |
        v
fresh import -> structural diff -> numeric replay diff
        |
        +---- pass: promote cycle report and retain regenerated source
        +---- fail: preserve diff, do not overwrite source authority
```

### Phase 1: Intake And Source Lock

Build the source inventory from `INBOX/taoryx-daveml-nesc-model-catalog-v1.0`
without moving or editing source bytes. For every document and qualified
package, record:

- catalog ID, family, source path, upstream revision, and license;
- SHA-256 for raw, normalized, embedded, and exported bytes;
- parser/compiler status and all embedded check-case counts;
- source frame, units, quaternion order, and validity envelope; and
- acquisition gaps or normalized-source disclaimers.

Exit artifact: `intake-manifest.json` plus the existing catalog verification
report. A missing or mismatched source blocks the cycle; it is never replaced
with a guessed or newly normalized file.

### Phase 2: Lossless Source Package

Import each source document into the collection as an immutable
`SourceDocument`. Preserve XML comments, unknown elements, namespace prefixes
where practical, original ordering, and raw bytes. The lossless layer is the
fallback when canonical export cannot represent an extension.

Exit artifacts:

- `source/` members with source hashes;
- `source/source-hashes.json`;
- source provenance and license records; and
- an L0/L1 report proving byte identity and source identity.

### Phase 3: Canonical Semantic IR

Project supported DAVE-ML constructs into explicit Taoryx-owned records rather
than exporting directly from XML nodes. The first IR must cover:

- variable definitions, units, descriptions, and initial values;
- functions, constants, algebraic dependencies, and check-case references;
- gridded and ungridded tables with axes, knots, interpolation, and boundary
  policy;
- component inputs/outputs and contribution authority;
- source-to-canonical transforms and frame conventions; and
- unsupported extensions as retained opaque nodes with a diagnostic.

The IR must be deterministic and independently serializable. Every canonical
record keeps its source document ID and source node ID so an export or numeric
diff can point back to the source.

Exit artifact: `canonical/ir.json` with a schema version, stable IDs, source
anchors, and an unsupported-feature ledger. No runtime compilation is allowed
from an IR that has unresolved required features.

### Phase 4: Collection Build And Runtime Binding

Build `.txcollection` from the lossless source layer, canonical IR, bindings,
transforms, contribution authorities, stateful-component contracts, runtime
artifact, and validation evidence. Compile or attach `.txair` only through an
explicit binding; DAVE-ML static functions do not silently become equations of
motion, atmosphere, staging, controllers, or event logic.

The collection replay command must verify the outer checksum ledger, replay the
declared runtime artifact, and emit a report containing collection identity,
family, runtime member, source hashes, load-contract status, and evidence type.

Exit gate: F-16, HL-20, and NESC two-stage rocket collections all replay with
their declared evidence families. The current repository has this gate for the
three qualified packages.

### Phase 5: Canonical DAVE-ML Export

Generate DAVE-ML from the canonical IR, not by editing or copying the original
XML. The exporter must:

- emit deterministic namespaces, ordering, numeric formatting, and generated
  IDs;
- preserve canonical IDs and source anchors in provenance annotations or the
  sidecar report;
- emit supported functions, tables, units, and check-case bindings;
- fail or quarantine opaque unsupported nodes instead of silently dropping
  them; and
- write `exported/<document>.dml` and `canonical/export-manifest.json`.

The first exporter scope is the 22 catalog documents that parse successfully,
with the seven official conformance fixtures used as serializer tests before
vehicle packages are promoted.

### Phase 6: Re-Import And Comparison

Fresh-process import of the generated DAVE-ML must produce a new IR. Compare in
layers:

| Layer | Comparison | Required result |
| --- | --- | --- |
| L0 | Exact source bytes when source-preserving export is selected | Equal |
| L1 | Source identity, document roles, and provenance | Equal or declared normalized-source mapping |
| L2 | Canonical structure, IDs, variables, functions, tables, knots, and boundary policies | Equal modulo approved generated-ID map |
| L3 | Seeded interior, boundary, extrapolation, and embedded check-case values | Within declared absolute/relative tolerances |
| L4 | Runtime loads, observables, hold/benchmark evidence, and trajectory checkpoints | Within package/family acceptance tolerances |

Every difference becomes a typed record with source anchor, canonical path,
old value, new value, tolerance, and disposition. A numeric pass does not erase
a structural loss, and a structural pass does not imply runtime equivalence.

Exit artifact: `roundtrip-report.json`, `structural-diff.json`,
`numeric-diff.json`, and, when applicable, `runtime-replay-report.json`.

### Phase 7: Corpus Promotion Order

Promote in increasing integration complexity:

1. Official seven conformance fixtures: serializer/parser behavior. These are
   preserved and hash-pinned under
   `resources/aerospace/daveml/official-conformance-v1/`.
2. Normalized simple 2D/3D and atmosphere examples: tables, units, and
   boundary policies. The simple lift-curve and 1976 atmosphere fixtures are
   included in the hash-pinned official corpus and pass the canonical gate.
3. F-16 S-119: multi-document aero, propulsion, inertia, and control bindings.
4. HL-20 Mod K: unpowered lifting-body source and glide evidence.
5. NESC two-stage rocket: scheduled mass, propulsion, staging, and benchmark
   evidence.
6. Remaining NESC atmospheric models and scenarios: host environment and
   trajectory qualification, not merely source compilation. The incorporated
   corpus contains no additional NESC DAVE-ML members beyond the promoted
   two-stage package; this rung is therefore recorded as not applicable to
   the current corpus rather than silently skipped.

Each promotion produces a separate collection and report. A failure in a
later family must not weaken the acceptance status of an earlier family.

## Cycle Exit Conditions

The initial round-trip tranche is complete when:

- the INBOX catalog and all selected source/package hashes are verified;
- the three qualified packages build or load as `.txcollection` and replay;
- the canonical IR is deterministic and source-anchored;
- the seven conformance fixtures round-trip structurally and numerically;
- F-16, HL-20, and NESC rocket pass their declared L4 runtime evidence;
- regenerated DAVE-ML re-imports in a fresh process;
- every loss, unsupported extension, tolerance, and normalization is reported;
- repeated cycle runs produce identical manifests and diff results; and
- the source authority remains immutable and no result is promoted solely from
  an unqualified numerical coincidence.

The current implementation has completed intake, lossless collection build,
collection/runtime replay, and qualified-package evidence for the three
packages. The canonical exporter and fresh-import structural/numeric-literal
comparison are implemented. The next implementation gate is evaluator-backed
   comparison of seeded values, boundaries, and declared check cases. The
   official-fixture gate now preserves and compares `checkData` signatures;
   numerical execution of those cases remains a separate runtime adapter.

The catalog-wide source/package cycle is executable:

```text
python tools/canonical_daveml_roundtrip.py \
  --catalog-root INBOX/taoryx-daveml-nesc-model-catalog-v1.0 \
  --output-dir build/daveml-full-cycle
```

It processes all 22 normalized source documents and all 3 qualified packages,
writing per-source IR/export files, per-package reports, and
`catalog-roundtrip-report.json`. A verified result proves deterministic
structural and numeric re-import of the emitted XML. It does not by itself
prove that a DAVE-ML graph produces identical host force, moment, mass, or
trajectory outputs; those require independent evaluator vectors and runtime
qualification evidence.

## Family-Library Incorporation

`tools/import_daveml_catalog.py` is the catalog-to-library boundary. It cycles
all normalized `source.dml` documents, cycles every qualified `.txair` package,
and emits `verification/daveml_catalog_import.json`. The report keeps the full
catalog visible while distinguishing `qualified_reference_family` targets
from `catalog_source_record_only` records.

Each promoted reference family points at `plant/daveml-import.json`. The
sidecar records package-member hashes, package-to-catalog lineage when a
source match is unambiguous, canonical IR/export hashes, and Taoryx runtime
replay evidence. `ReferenceFamilyManifest` validates the sidecar's family and
package identity during load, so a family cannot silently drift away from its
imported package.

The first promoted library members are `reference_f16_s119`,
`reference_hl20_mod_k`, and `reference_nesc_two_stage_rocket`. The other six
catalog families remain source records until they have an executable package
and the host dynamics/environment evidence required by their integration spec.

Current family-library readiness is tracked separately in
`verification/daveml_family_readiness.yaml`; it deliberately distinguishes
source/runtime qualification from pending trim, controller, objective, and
scenario work.

## Extended Completion Plan

The initial round-trip tranche is the foundation, not the end of DaveML work.
The extended program below is the authority for completing semantic execution
and incorporating qualified models into Taoryx family libraries. Every phase
produces a report and a typed contract; no phase promotes a model merely
because its XML can be parsed.

### Track A: Typed Semantic Execution

1. Replace the loss-aware XML tree projection with typed records for variables,
   constants, functions, table definitions, table references, units, limits,
   interpolation, extrapolation, and check cases.
2. Resolve references across all documents in a package and reject duplicate,
   missing, cyclic, or dimensionally inconsistent definitions.
3. Preserve opaque extensions in the lossless source layer and emit a typed
   unsupported-feature record whenever the evaluator cannot execute one.
4. Version the IR schema and provide deterministic migration fixtures for every
   schema change.

Exit gate: a fresh process can load every promoted package into a typed graph,
with zero unresolved required features and a complete unsupported-feature
ledger.

### Track B: Numeric Evaluator And CheckData

Implement the DAVE-ML evaluation semantics needed by the corpus:

- scalar and vector constants;
- algebraic/function definitions and dependency ordering;
- 1-D, 2-D, and N-D gridded tables;
- ungridded tables;
- interpolation, limiting, and extrapolation policy;
- units and dimension checks; and
- `checkData` input, output, and internal-value vectors.

For each case, record inputs, expected values, evaluated values, absolute and
relative errors, declared tolerance, source anchor, and disposition. The
evaluator report must distinguish exact table-knot checks, interpolated checks,
boundary checks, extrapolation checks, and unsupported cases.

Exit gate: all applicable official and qualified-package check cases pass in a
fresh process, or are explicitly quarantined with a reproducible unsupported
feature report.

### Track C: Family-Library Contracts

For each promoted family, create a typed integration manifest that binds the
DaveML graph to Taoryx contracts without changing source ownership:

- aerodynamic coefficients, derivatives, reference geometry, frames, and
  control schedules;
- propulsion thrust, mass flow, staging, ignition, cutoff, and throttle;
- mass properties, center of gravity, inertia, and separation events;
- atmosphere/environment source, units, altitude domain, gravity, and Earth
  model assumptions;
- state, control, output, telemetry, and capability mappings; and
- provenance, source hashes, validity envelope, and non-claims.

Static DaveML functions must not silently become equations of motion,
atmosphere models, controllers, event logic, or mission objectives. Those
connections require explicit bindings in the family manifest.

Exit gate: loading a family fails closed on missing bindings, hash drift,
unsupported units, frame ambiguity, or out-of-envelope inputs.

### Track D: Trim And Operating Points

Add family-specific trim catalogs backed by the imported model graph. Each trim
record declares state variables, controls, environmental inputs, bounds,
residual definitions, solver settings, seeds, validity envelope, and source
provenance. Validate solutions with independent force, moment, and rate
residual calculations rather than solver status alone.

Required outputs include airspeed/Mach, altitude, angle of attack, sideslip,
control deflections, throttle, load factor, residual norms, solver status, and
source/package hashes.

Exit gate: every promoted family has at least one reproducible certified trim
and an explicit reason for every unavailable trim dimension.

### Track E: Linearization, Tuning, And Controllers

1. Generate local linearizations around certified trims using declared
   perturbations and finite-difference or analytic provenance.
2. Map DaveML derivatives into Taoryx state/control ordering and validate signs,
   frames, and control directions.
3. Add controller-design catalog entries for LQR, gain scheduling, and any
   family-specific tuning method.
4. Record conditioning, weights, saturation limits, actuator mappings, and
   tuning provenance.

Exit gate: each controller references a certified trim, has matching state and
   control contracts, passes direction probes, and carries a bounded tuning
   report.

### Track F: Objectives, Scenarios, And Runtime

Expose model-family objectives such as trim capture, attitude hold, glide,
flight-path tracking, energy management, staging, and terminal conditions
through the existing objective and scenario contracts. Each objective declares
channels, units, target, tolerance, validity envelope, termination behavior,
and evidence artifact.

Add runtime/CLI paths to select a family, fidelity, source/package identity,
trim, controller, objective, and scenario. The CLI must fail closed on stale
provenance, unresolved bindings, unsupported extrapolation, and missing trim or
controller contracts.

Exit gate: a clean process can load each promoted family, execute its declared
smoke scenario, score its objectives, and emit a provenance-complete artifact.

### Track G: Promotion, CI, And Release

Promotion remains ordered by semantic complexity: official fixtures, simple and
atmosphere models, F-16, HL-20, NESC rocket, then any additional qualified
families. Each rung requires intake, lossless identity, typed IR, canonical
export, fresh re-import, structural diff, numeric/checkData diff, runtime
replay, family-library integration, trim, tuning, objective, and provenance
gates that are applicable to that family.

CI must run hash verification, deterministic export twice, fresh-process import,
numeric evaluation, family loading, trim smoke tests, controller direction
probes, objective scoring, and report schema validation. Release bundles must
include source manifests, canonical IR/export manifests, typed diffs, runtime
evidence, family manifests, and explicit non-claims.

The extended program is complete only when every applicable corpus rung has a
passing report, every non-applicable rung has an evidence-backed disposition,
and no generated artifact can overwrite immutable source authority.

## Execution Ledger

The current long-running implementation is deliberately staged. The following
items are complete and reproducible:

- The bounded evaluator executes the official atmosphere, F-16, and HL-20
  check-data vectors, plus F-16 propulsion and inertia package vectors, in a
  fresh Python process.
- F-16 propulsion placeholder `initialValue` records are correctly overridden
  by their function sources; the package now passes 54/54 propulsion checks.
- `DAVEMLGraph` is a reusable typed scalar execution view, and
  `load_daveml_family_graph` verifies package and source-member hashes before a
  family component can execute.
- `tools/validate_daveml_family_readiness.py` produces
  `verification/daveml_family_readiness.json`; its applicable source,
  round-trip, graph, check-data, and runtime gates pass for F-16, HL-20, and
  NESC while retaining pending derived-layer status.
- `DAVEMLTrimBinding` provides the explicit solver-facing seam for mapping
  family state, controls, environment inputs, and graph outputs into the
  existing trim evaluator; it is an adapter contract, not a certified trim.
- `DAVEMLFunctionChannel` and `load_daveml_function_channel` now provide a
  hash-verified typed channel boundary for individual aerodynamic, propulsion,
  and inertia functions. F-16 `cxt` and HL-20 `CL0A0` are covered by focused
  tests with source-unit and output checks.
- `load_daveml_trim_binding` now composes that boundary with the existing trim
  solver. A source-backed F-16 pitch-coefficient probe solves elevator trim at
  fixed air data with a residual below `1e-9`; this is a certified channel
  probe, not yet a complete force/thrust operating-point qualification.
- `DAVEMLCompositeTrimBinding` now combines independently hash-verified
  component bindings. A focused F-16 probe evaluates aerodynamic pitch
  coefficient and propulsion thrust together without collapsing their source
  provenance; force-balance residual composition remains the next gate.
- `DAVEMLFixedWingLoadBinding` now converts hash-verified F-16 body-axis
  coefficients and lbf thrust into explicit SI force and moment channels using
  declared reference geometry and conversion factors. The focused probe
  verifies the channel arithmetic without applying hidden mass or gravity
  assumptions; HL-20 wind-axis/lifting-body mapping remains separate work.
- `tools/validate_daveml_trim.py` now emits the reproducible
  `verification/daveml_f16_trim_evidence.json` artifact. It certifies the
  source-backed F-16 pitch-channel elevator trim and a local residual Jacobian
  with package/document hashes; it is intentionally not yet a full dynamics
  linearization or six-degree-of-freedom equilibrium.
- The same evidence task now emits
  `verification/daveml_hl20_trim_evidence.json`, certifying the HL-20
  source-backed pitch-channel angle-of-attack trim at a declared Mach/airspeed
  point. The artifact retains the same non-claim boundary until the full
  vehicle load, gravity, and atmosphere adapters are integrated.
- `DAVEMLAtmosphereBinding` and `tools/validate_daveml_atmosphere.py` now bind
  the official 1976 atmosphere fixture by SHA-256 and emit normalized SI
  temperature, pressure, density, and speed-of-sound samples. The adapter
  preserves source ratios and rejects unresolved channels; it does not
  extrapolate beyond the source domain.
- `DAVEMLFixedWingLoadBinding.evaluate_with_atmosphere` now composes that
  environment contract with the F-16-style body-load adapter using the
  declared dynamic-pressure equation `q = 0.5*rho*V^2`; the coupled probe
  remains explicit about speed units and still applies no hidden mass or
  gravity terms.
- `DAVEMLInertiaBinding` now loads the F-16 mass-properties graph through its
  verified package/member hash, converts slug and slug-ft2 values to SI, and
  exposes a signed symmetric body inertia matrix. This is the source-backed
  inertia seam required before a six-degree-of-freedom dynamics linearization.
- `DAVEMLFixedWingDynamicsBinding` now provides the true body-axis Newton-Euler
  derivative contract for `u,v,w,p,q,r`, consuming explicit total loads, mass,
  inertia, and optional body gravity. Its focused test verifies translational
  and rotational accelerations; this is the dynamics seam, not yet a certified
  operating-point linearization or controller design.
- `tools/validate_daveml_linearization.py` now emits
  `verification/daveml_f16_linearization_evidence.json` with a source-linked
  6x6/6x1 local dynamics Jacobian and declared central-difference steps. It is
  a verified channel operating-point artifact, not an equilibrium trim or
  controller qualification.
- `tools/validate_daveml_tuning.py` now consumes that Jacobian through the
  existing named LQR factory and emits
  `verification/daveml_f16_tuning_evidence.json`. The reduced five-state screen
  is controllable and Hurwitz with bounded surface commands; longitudinal speed
  is explicitly excluded because the source-channel model has rank five.
- `tools/validate_daveml_scenario.py` now drives the existing unit-aware
  objective scorer with a hashed `ScenarioContract`, producing
  `verification/daveml_f16_scenario_evidence.json`. Stability, bounded command,
  and controller-event objectives pass in a clean-process smoke run; this is
  not a flight or mission qualification.
- The runtime CLI now exposes `taoryx daveml smoke --family ...`. It fail-closes
  on unsupported family selection, missing/stale evidence, family mismatch, or
  unresolved source roles, and emits a deterministic
  `taoryx.daveml-cli-smoke/v1` envelope for F-16 and HL-20.
- `tools/validate_daveml_release.py` now executes the fresh-process promotion
  chain and emits `verification/daveml_release_gate.json`. The current gate
  runs readiness, atmosphere, trim, linearization, tuning, scenario scoring,
  and both family CLI smoke paths, then hashes the resulting artifacts. Its
  claim boundary remains source integration, not flight qualification.
- The F-16 readiness entry now promotes only the verified source-channel
  layers (`source_channel_dynamics_verified`,
  `source_channel_reduced_lqr_verified`, and
  `source_channel_smoke_verified`); the registry continues to leave full
  equilibrium trim pending and leaves HL-20/NESC derived layers unchanged.
- Family manifests now repeat the trim, linearization, tuning, objective, and
  scenario evidence dispositions, and readiness validation fails if those
  declarations drift.

The following are intentionally not marked complete:

- The official two-dimensional ungridded fixture has three failed local
  numeric checks after one pass. Its interpolation rule remains unresolved;
  tolerances are not weakened, and NASA DAVEtools does not qualify that legacy
  fixture either. This is a numeric disposition, not a silent pass.
- Full typed package IR reference resolution, units/dimensions, vector graph
  execution, and complete ungridded interpolation remain Track A/B work.
- F-16 and HL-20 certified trim artifacts, trim Jacobians, controller tuning,
  objective catalogs, and scored smoke scenarios remain Tracks D-F work.
  Their readiness entries say `source_package_regression` or `pending` rather
  than implying a certified trim or controller.

The next executable gates are, in order:

1. Resolve and independently test the official ungridded interpolation
   disposition without changing source tolerances.
2. Compose typed function channels into source-backed F-16 and HL-20 trim
   records and residual adapters, then
   certify at least one operating point per applicable family.
3. Generate trim-local linearizations, run control-direction probes, and add
   controller/tuning records only after the trim contracts pass.
4. Add family objective and scenario artifacts, expose them through the
   runtime selection path, and emit provenance-complete smoke reports.
5. Promote only after the CI bundle runs intake, deterministic export twice,
   fresh import, numeric/check-data evaluation, graph loading, trim, tuning,
   objective scoring, and runtime replay gates.
