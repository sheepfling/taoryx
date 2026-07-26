# Legacy toolkit adoption review

Reviewed source:
the preserved legacy toolkit acquisition archive

Review status: quarantine analysis only. No legacy source has been promoted into
the package.

## Evidence from the legacy project

- Its isolated regression suite passes **394 tests**.
- It contains 38 evidence-bounded EBNF files, typed grammar profiles, JSON
  Schemas, an evidence ledger, and lossless source records.
- Its own documentation distinguishes verified, structural, observed, and
  blocked claims. That boundary discipline is valuable.
- It explicitly does not execute trajectory integration, optimization,
  summarize/survey reduction, unit conversion, or EGS database writes.

## Baseline compatibility check

The legacy lexical parser was run against the current verified fixture baseline
using the TAOS 96 dialect:

- 5 Chapter 3 `.tbl` fixtures: byte-preserving round trip passed;
- 4 Chapter 4 `.prb` fixtures: byte-preserving round trip passed;
- 1 Chapter 4 `.tbl` fixture: byte-preserving round trip passed;
- all 10 fixtures retained their original bytes after parse/render.

This establishes a safe preservation seam, not semantic equivalence. On these
fixtures the legacy parser still emits many `raw` records; the current parser
has stronger verified structure for the established examples.

## Keep / adapt / defer

### Keep as reference material

- `resources/manual_evidence/ledger.json` and its evidence notes;
- grammar profiles and their verified/structural status model;
- the lossless `SourceSpan`, newline, comment, raw-record, and render concepts;
- focused tests that encode evidence-bounded behavior;
- EBNF documents that correspond to reviewed manual pages.

### Adapt gradually into `taoryx`

1. Compare the legacy lexical IR with `taoryx.language` diagnostics and source
   locations.
2. Add a lossless source layer behind the current verified parser rather than
   replacing its AST.
3. Port only tests using the current `examples/chapter03/` and
   `examples/chapter04/` fixtures first.
4. Translate useful canonical records into `taoryx` models only after their
   evidence status and relationship to `metadata/` are explicit.

### Do not promote wholesale

- the `taos_legacy` package namespace;
- its wheel, `dist/`, generated analysis JSON, or release/checksum artifacts;
- synthetic/profile-example fixtures as official TAOS fixtures;
- structural `*summarize`, `*survey`, `*radar`, `*search`, or other blocked
  syntax as verified grammar;
- any preservation analyzer as a simulation implementation.

## Recommended next slice

Port the legacy lossless lexical concepts into a small compatibility module and
write differential tests that assert, for each verified baseline fixture:

1. the current parser accepts it;
2. the legacy lexical layer round-trips it;
3. source spans and comments remain available;
4. semantic diagnostics remain owned by the current verified parser.

Only after that differential seam is stable should we consider porting selected
evidence-aware catalogs or block analyzers.
