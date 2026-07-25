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
| M3 canonical DAVE-ML exporter | Pending | Requires canonical-to-DAVE-ML writer |
| M4 round-trip verifier | Source-layer gate complete; semantic levels pending | Fresh source reload is explicit; L2/L4 remain pending |
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
  --source-root INBOX/taoryx-aerospace-data-corpus-v1.1 \
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
  --source-root INBOX/taoryx-aerospace-data-corpus-v1.1 \
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
4. Add the Taoryx runtime replay adapter only after static round-trip evidence
   is stable.
