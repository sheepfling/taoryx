# CADAC table normalization

CADAC source decks can be normalized into provider-neutral Taoryx persisted table resources.

Canonical schemas:

- `taoryx.table.v1`
- `taoryx.table_bundle.v1`

The importer preserves source provenance, original axis order/direction, units when defensible, multilinear interpolation, and CADAC's lower-linear/upper-clamp extrapolation policy. Descending source axes are stored ascending with the value tensor reordered consistently.

Commands:

```bash
python -m taoryx.families.cadac convert-deck <deck.asc> --output <directory>
python -m taoryx.families.cadac convert-bundle <input.asc> --output <directory>
python -m taoryx.families.cadac convert-tree <CADAC-root> --output <directory>
python -m taoryx.families.cadac validate-table <directory>
```

Original CADAC source values are not part of this repository overlay. Generate canonical bundles from a local source checkout when the data is available and appropriate to use.
