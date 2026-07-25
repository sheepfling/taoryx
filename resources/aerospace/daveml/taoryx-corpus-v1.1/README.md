# Taoryx DAVE-ML reference corpus v1.1

This is the minimal offline input required to rebuild the Alpha 2 F-16 S-119
and HL-20 Mod K source-grounded reference collections. The archive is kept as
one immutable payload so the exact Taoryx-normalized F-16 inputs and compiled
`.txair` package remain reproducible.

## Integrity

```text
SHA-256: 9dbb38f23924f2cd2775cb37b1b87476d9ddda672ecab9b84828b253931ebd31
```

The expected package and source hashes are recorded in `manifest.json` and in
the family source locks under `families/`.

## Extract and validate

From the repository root:

```bash
rm -rf /tmp/taoryx-aerospace-data-corpus-v1.1
mkdir -p /tmp/taoryx-aerospace-data-corpus-v1.1
unzip -q resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip \
  -d /tmp/taoryx-aerospace-data-corpus-v1.1
python tools/verify_daveml_reference_inputs.py \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1
```

Build the source collections:

```bash
python tools/build_daveml_collection.py \
  --family-manifest families/reference_f16_s119/family.yaml \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output /tmp/f16-s119.txcollection
python tools/build_daveml_collection.py \
  --family-manifest families/reference_hl20_mod_k/family.yaml \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output /tmp/hl20-mod-k.txcollection
```

The generated collections are derived artifacts and should remain outside the
authoring tree unless a release specifically packages them.

## Contents

- F-16 S-119 normalized DAVE-ML, propulsion, inertia, validation evidence, and
  verified `.txair` package.
- HL-20 Mod K byte-pinned DAVE-ML, fixed mass-property binding, tables,
  validation evidence, and verified `.txair` package.
- Embedded provenance and package checksum ledgers.

This corpus does not establish operational, certification, or real-aircraft
predictive claims. See `NOTICE.md` and the family integration records.
