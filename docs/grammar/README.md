# `.tbl` and `.prb` grammar validation

This is the working area for validating the TAOS table and problem languages.
The grammar is reconstructed from the manual; it is not yet a complete
lossless grammar for every historical construct.

## Sources of truth

- [Table EBNF](../../grammars/taos_table.ebnf) — documentary grammar for `.tbl` files
- [Problem EBNF](../../grammars/taos_problem.ebnf) — documentary grammar for `.prb` files
- `src/taoryx/language/` — parser, AST models, diagnostics, and semantic validation
- `examples/chapter03/` and `examples/chapter04/` — extracted manual fixtures
- `metadata/fixture_provenance.yaml` — fixture provenance and expected status

## Validation layers

1. **Lexical/expression validation** checks tokens, operators, literals, and table references.
2. **Structural parsing** builds typed documents and dispatches documented block keywords.
3. **Semantic validation** checks references, block placement, trajectory and segment relationships,
   table cardinality, and known historical constraints.
4. **Fixture validation** runs the parser over extracted manual examples and records diagnostics
   and completeness boundaries.

The parser must preserve documentation-only fixtures that contain omissions or historical printout
excerpts. “Parses successfully” and “executable-complete” are intentionally separate outcomes.

## Commands

```sh
python tools/dev.py grammar
python tools/dev.py check
```

When changing a grammar rule, parser model, or fixture, update the focused tests and fixture
provenance together. Do not silently rewrite established manual examples to fit the current parser.

## Planned work

- turn the EBNF into an executable grammar or generate parser checks from it;
- add positive and negative grammar fixtures independent of manual examples;
- model units and defaults explicitly;
- close complex block-body and lossless-formatting gaps.
