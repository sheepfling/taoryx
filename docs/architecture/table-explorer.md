# Table Explorer foundation

TAORYX treats table inspection as a data-contract boundary rather than as a
plotting command. The parser remains the source-faithful layer; the explorer
adds a renderer-independent artifact that can be consumed by terminal, JSON,
HTML, notebook, and static-plot views.

```text
.tbl source
    -> TableDocument
    -> TableInspectionArtifact
       -> terminal / JSON / future HTML and plot renderers
```

## Phase 1 contract

`taoryx.table_explorer.inspect_table_file()` creates an artifact with:

- a compact catalog for every declared table;
- the declared type, format, axes, shape, options, and table dependencies;
- raw assignments and full-table operations with their source locations;
- parser and semantic diagnostics plus recovered source records;
- an optional `PreparedTable` only when simple-table preparation succeeds.

The artifact deliberately keeps raw and prepared views together. A source
excerpt or malformed table can therefore be displayed and diagnosed without
being silently sorted, repaired, averaged, or treated as executable.

```python
from taoryx.table_explorer import inspect_table_file

artifact = inspect_table_file("vehicle.tbl")
print(artifact.format_catalog())
payload = artifact.to_dict()  # suitable for JSON serialization
```

## Status semantics

| Status | Meaning |
| --- | --- |
| `prepared` | A simple regular grid passed current runtime preparation. |
| `source-only` | The source is inspectable, but is a full program or otherwise has no prepared grid. |
| `partial` | The parser identified documented omissions, so executable completeness is not claimed. |
| `invalid` | Diagnostics or cardinality/axis checks prevent safe preparation. |

The status is an evidence boundary, not a quality score. In particular,
`source-only` does not mean that full-table semantics have been implemented or
that historical TAOS behavior has been reproduced.

## Planned next layers

The core artifact is intentionally free of Matplotlib, Plotly, browser, or
notebook imports. The next renderer-independent APIs should explain regular
interpolation brackets and full-table execution traces. Only after those
contracts are stable should optional visualization extras render lines,
heatmaps, skewed-table slice families, control-flow graphs, and runtime-usage
overlays.
