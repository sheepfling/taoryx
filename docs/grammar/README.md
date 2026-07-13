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
- `metadata/legacy_grammar_registry.json` — quarantined legacy grammar candidates
- `tests/fixtures/taos_manual_corpus_v22/` — 164 source-faithful Chapter 3/4
  displays, 149 synthetic wrappers, manifests, and lexical/parser baseline evidence

## Validation layers

1. **Lexical/expression validation** checks tokens, operators, literals, and table references.
2. **Structural parsing** builds typed documents and dispatches documented block keywords.
3. **Semantic validation** checks references, block placement, trajectory and segment relationships,
   table cardinality, and known historical constraints.
4. **Fixture validation** runs the parser over extracted manual examples and records diagnostics
   and completeness boundaries.

The currently canonicalized subset is represented by the evidence-linked contracts in
`src/taoryx/language/grammar_contracts.py`: free-field lexing, problem framing, hierarchy,
and the problem-level block catalog. `src/taoryx/language/lexical.py` provides the shared token
stream with source offsets, numeric/reference classification, and inline-comment boundaries.
The legacy EBNF registry remains an archived research record; it does not expand the accepted
grammar automatically.

The parser must preserve documentation-only fixtures that contain omissions or historical printout
excerpts. “Parses successfully” and “executable-complete” are intentionally separate outcomes.

## Recovery and diagnostics contract

Parsing is a file-level operation, not a fail-fast operation. For every `.tbl` or `.prb` file, the
ingestion route returns:

- the lossless source and lexical records;
- the typed portion of the document that could be established safely;
- zero or more source-located diagnostics, in encounter order; and
- recovery records containing the original rejected line and its diagnostic code.

Malformed input must synchronize at a documented boundary such as a problem, trajectory, segment,
or block header. A malformed construct may be retained as raw/recovered text, but it must not be
given guessed semantics and must never raise an implementation exception. The command-line route
continues across all supplied paths and can write an aggregate report:

```sh
taoryx-validate --report build/grammar-diagnostics.json examples/chapter03/*.tbl examples/chapter04/*.prb
```

The process exits nonzero when any file has an error diagnostic, while still reporting every file.
This makes recovery behavior testable in fixtures and usable by CI/editor integrations.

The current typed problem-body coverage includes:

- coordinate/reference assignments for `*dwn/crs`, `*iip`, `*initial`, and `*tangent`;
- assignment syntax for `*aero`, `*constants`, `*cg`, `*integ`, `*prop`, `*reset`, and `*increment`;
- relationship lists for `*limits`;
- direct, `vrs`, interpolation, and rule-only forms for `*fly`; and
- `launch`/`sled` mode validation for `*rail`; and
- typed `*optimize` constraints, endpoint qualifiers, reference factors, and documented control
  assignments; and
- documented `*inertial` alignment forms and coordinate-specific assignments.
- `*survey` headers plus incremental (`lo`/`hi`/`inc`) and explicit (`vals`) settings, both inline
  and on continuation lines.
- `*search` objectives with endpoint relationships, multiline `=` continuations, and documented
  search controls.
- `*radar` station identities, earth-shape selectors, and documented station parameters.
- `*units/fmt` variable/unit/format records, including inline and continuation-line forms.
- `*atmos` standard/user/site model headers, user/site column headers, and numeric data rows.
- `*earth` documented model families and earth-parameter assignments.
- `*wind` coordinate-system headers, the mutually exclusive speed/heading and
  east/north component sets, and table-or-constant assignments.
- `*file`, `*egs`, and `*print` output-variable lists, including indexed
  variables and the special `*egs summary file` form.
- `*summarize` operation names, operand-bearing versus unary operations,
  special trajectory functions, and segment/trajectory-qualified operands.
- `*initial` coordinate-specific position/velocity variables, required mass
  input, mutually exclusive `wt`/`mass` and `vel`/`mach`, and both the
  authoritative `from segment N, trajectory M` framing and retained legacy
  ordering.

The parser still deliberately retains some documented complex bodies as raw statements while
their typed contracts are being reconstructed. The next evidence-backed slices are the remaining
block-specific variable restrictions. Raw retention in those areas is a supported preservation
boundary, not an assertion that the runtime semantics have been implemented. A table is not marked
executable-complete when it contains syntax or semantic errors, even if recovery produced a partial
AST.

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
