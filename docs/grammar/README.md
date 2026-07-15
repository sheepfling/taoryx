# `.tbl` and `.prb` grammar validation

This is the working area for validating the TAOS table and problem languages.
It is not the canonical manual source and it is not the runtime implementation.
The grammar is reconstructed from the manual. Documented constructs are
accepted, source-preserved, and diagnosed within their evidence-backed
boundaries; complex bodies may remain raw statements when typed semantics would
require unsupported inference.

For the fastest route into the validator and parser corpus:

```bash
python tools/dev.py grammar
taoryx-validate examples/chapter04/ballistic-reentry.prb
python tools/dev.py test-grammar
python tools/dev.py test-spectre
```

## Sources of truth

- [Table EBNF](../../grammars/taos_table.ebnf) — documentary grammar for `.tbl` files
- [Problem EBNF](../../grammars/taos_problem.ebnf) — documentary grammar for `.prb` files
- [taoryx extensions](../../grammars/taoryx_extensions.ebnf) — successor-only mode syntax,
  explicitly separate from the 1995 TAOS grammar
- [Grammar profiles](profiles.md) — selectable `taos96` and `taoryx` claim boundaries
- `src/taoryx/language/` — parser, AST models, diagnostics, and semantic validation
- `examples/chapter03/` and `examples/chapter04/` — extracted manual fixtures
- `metadata/fixture_provenance.yaml` — fixture provenance and expected status
- `metadata/legacy_grammar_registry.json` — quarantined legacy grammar candidates
- `tests/fixtures/taos_manual_corpus_v22/` — 164 source-faithful Chapter 3/4
  displays, 149 synthetic wrappers, manifests, and lexical/parser baseline evidence
- `tests/fixtures/taos_e2e_v23/` — complete application-level `.prb`/`.tbl` cases,
  metamorphic metadata, and documented-surface coverage; see
  [`docs/grammar/e2e-suite-v23.md`](e2e-suite-v23.md)
- `tests/fixtures/grammar_baseline/` — independent positive and negative
  `.tbl`/`.prb` fixtures for syntax acceptance and multi-diagnostic recovery

The manual corpus currently declares 131 fragment displays. The corpus checker dispatches every
one through its declared direct fragment route, in addition to validating the 149 synthetic
whole-file wrappers; this keeps excerpt parsing and complete-file validation as separate,
auditable claims.

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
Tests compare the parser’s problem-block, table-type, and table-operation catalogs directly with
the reviewed Chapter 4 grammar-reference registries, so catalog drift fails the parser gate.
Both documentary EBNF files now have closed nonterminal inventories; a regression test rejects
undefined productions. They remain structural documentation rather than an executable grammar
until a grammar backend is selected.
`taoryx.language.ebnf` is the first executable layer for that notation: it parses both files into
typed rule trees and reports malformed grammar syntax or undefined productions with locations.
It does not yet replace the canonical TAOS source parser or assign semantics to broad historical
block bodies.
The Chapter 4 guidance-rule registry is likewise compared with the parser contract, and every
documented guidance rule has a positive acceptance case.
The grammar-contract tests also instantiate every reviewed table type and every reviewed full-table
operation, including special `if`, `csto`, `goto`, and `end` forms, so registry membership is backed
by a positive parser acceptance test rather than a catalog-only comparison.
Table header parameters are type-checked against the manual: coefficient tables accept numeric
`sref`, thrust and mass-flow tables accept their documented `units`, and other table types reject
unsupported parameters while preserving the original header.
Full-table control flow validates simple `if` relationships, rejects undocumented nested `if`
operations, and preserves name-or-value labels for `goto` references.
Full-table semantic validation also diagnoses duplicate operation labels, duplicate storage
register names, storage names that shadow documented TAOS state variables, and `goto` destinations
with no matching label; the parsed operations remain available for inspection and source-preserving
recovery.
Simple-table independent axes enforce the manual's strict monotonicity rule; duplicate and
unordered values are retained but reported as errors.
Duplicate simple-table assignments and duplicate assignments within a full-table interpolation
group are likewise retained and diagnosed instead of being silently overwritten during semantic
validation.
Table identification names and problem-file table references are compared case-insensitively, as
required by the manual's lower-case normalization rule; their original spelling remains in the
lossless source and typed models.
When a typed table catalog is supplied to problem ingestion, role-bearing references such as
`*aero ca=(...)`, `*prop thrust=(...)`, and `*wind windv=(...)` are checked against the referenced
table's declared type. Name-only catalogs remain supported and retain external-reference warnings.
An optional table-variable catalog also diagnoses user-defined assignments in `*aero`, `*cg`, and
`*prop` that are provably absent from all tables referenced by that block. Without that metadata,
the parser preserves the assignment rather than guessing that it is invalid.
Table declarations and interpolation calls enforce the manual's maximum of five independent
variables.
Full-table `csto` storage names are checked against the documented state-variable registry: a
reserved state name is diagnosed, while the original operation remains parsed and recoverable.
The legacy EBNF registry remains an archived research record; it does not expand the accepted
grammar automatically.

## User-facing entry points

- `taoryx-validate` is the direct problem/table validator; pass `--profile taoryx`
  when validating successor extensions.
- `python tools/dev.py grammar` runs the parser, fixture, and manual-corpus
  checks together.
- `python tools/dev.py test-grammar` runs the grammar view from pytest.
- `python tools/dev.py test-spectre` runs the Spectre corpus that exercises
  problem, segment, and trajectory coverage.
- `tests/fixtures/grammar_baseline/` contains the smallest independent syntax
  fixtures for positive and negative grammar checks.

The parser must preserve documentation-only fixtures that contain omissions or historical printout
excerpts. “Parses successfully” and “executable-complete” are intentionally separate outcomes.
Independent grammar fixtures exercise that boundary without relying on the manual-derived corpus:
valid fixtures must round-trip through lossless ingestion, while malformed fixtures must retain
their source and report multiple located diagnostics.

## Recovery and diagnostics contract

Parsing is a file-level operation, not a fail-fast operation. For every `.tbl` or `.prb` file, the
ingestion route returns:

- the lossless source and lexical records;
- the typed portion of the document that could be established safely;
- zero or more source-located diagnostics, in encounter order; and
- recovery records containing the original rejected line, including indentation
  and inline comments, and its diagnostic code.

Malformed input must synchronize at a documented boundary such as a problem, trajectory, segment,
or block header. A malformed construct may be retained as raw/recovered text, but it must not be
given guessed semantics and must never raise an implementation exception. The command-line route
continues across all supplied paths and can write an aggregate report:

```sh
taoryx-validate --report build/grammar-diagnostics.json examples/chapter03/*.tbl examples/chapter04/*.prb
```

The process exits nonzero when any file has an error diagnostic, while still reporting every file.
This makes recovery behavior testable in fixtures and usable by CI/editor integrations.
A unified ingestion result returns parser and semantic diagnostics together in stable physical
source order; recovery records are sorted by the same locations, so later validation cannot move
an earlier source error behind a finding discovered during model traversal.
A deterministic malformed-input probe also exercises both parsers over representative token noise,
requiring zero exceptions and exact physical-line recovery for every retained record.
The required problem terminator is also checked: a missing `*end` is reported at the next problem
declaration or end of file while the already parsed problem remains available.
Semantic validation diagnostics returned through unified ingestion also receive recovery records
with the original source line, so unresolved references and ambiguous cross-block relationships
remain traceable in the same way as syntax errors.
An unknown block header closes the preceding typed-block context; following continuation-looking
lines become `orphan-line` recovery records rather than being attached to the previous block.
Malformed `*trajectory` headers close the active trajectory, while malformed `*segment` headers
close only the active segment; subsequent valid framing can recover without inheriting stale
hierarchy semantics.
Block-level `RawStatement` records likewise retain the original physical line, including
indentation and inline comments; typed interpretations are additive and do not replace that
source evidence.
File ingestion decodes with `surrogateescape` and retains arbitrary undecodable source bytes in
the lossless document, while the semantic layer reports any resulting unsupported construct with
source-located diagnostics rather than silently replacing the bytes.
Table definitions, assignments, and full-table operations expose the same original source line
through `source_text`, including nested operations, so direct `.tbl` parsing does not require a
separate normalization pass to recover formatting evidence.
Manual displays classified as operation fragments use `parse_table_operation_fragment`; this
parses the documented operation sequence without inventing a table header or reporting omitted
interpolation data as an error. The fragment result retains source text, typed operations,
source-located diagnostics, and recovery records, while whole-file `parse_table_text` continues
to enforce table-level completeness and semantic validation.
Documented full-table bodies use `parse_table_body_fragment`, which treats `start` and `end` as
body markers, and standalone numeric assignment displays use `parse_table_assignment_fragment`.
Neither route invents omitted table metadata or performs whole-table cardinality checks.
Standalone `table ...` declarations use `parse_table_header_fragment`; the typed result contains
the declared table type, axes, and header options without fabricating an identification name.
Problem-language displays with an explicit manifest scope use `parse_problem_fragment`; it adds
only temporary framing needed by the existing block parser, shifts locations back to the raw
display, and suppresses contextual completion checks whose prerequisites are outside the excerpt.
Those checks remain observable through `deferred_diagnostics` and
`deferred_recovered_records`, so context deferral is source-located rather than silently dropped.
The documented optimize constraint/control displays have an explicit route:
`parse_optimize_body_fragment` returns only their typed constraints and controls, never the
synthetic optimize header used internally for parsing. No other continuation-only fragment
category is currently declared by the reviewed corpus.
Problem block nodes expose their original header line through `source_text` while retaining a
normalized `header` for grammar checks; comments and spacing therefore remain available without
being mistaken for header semantics.
Problem, trajectory, and segment framing nodes preserve their original header lines in the same
way, including malformed framing that remains attached for later diagnostics and recovery.
The dual-scope `*define`, `*file`, and `*print` keywords are retained using the established
trajectory/problem boundary behavior when they follow completed segments, but receive an
`ambiguous-dual-scope-block` warning and recovery record: the manual permits these keywords at
both trajectory and problem scope, so the parser does not silently claim that physical placement
proves the author's intended scope.
Problem names are checked as nonempty historical identifiers; malformed names remain attached to
their problem record with an `invalid-problem-name` diagnostic so later problems are still parsed.
Trajectory headers require a nonempty title; an empty title is retained with an
`invalid-trajectory-header` diagnostic so following segments remain recoverable.
Segment-only blocks are not silently promoted to trajectory or problem scope when misplaced;
historically evidenced pre-segment `*inertial` forms remain accepted as a compatibility exception.

The current typed problem-body coverage includes:

- coordinate/reference assignments for `*dwn/crs`, `*iip`, `*initial`, and `*tangent`;
- assignment syntax for `*aero`, `*constants`, `*cg`, `*integ`, `*prop`, `*reset`, and `*increment`;
- relationship lists for `*limits`;
- direct, `vrs`, interpolation, and rule-only forms for `*fly`; and
- `launch`/`sled` mode validation for `*rail`; and
- typed `*optimize` constraints, endpoint qualifiers, reference factors, and documented control
  assignments; and
- optimization endpoints reject undocumented wildcard semantics while preserving malformed source;
- documented `*inertial` alignment forms and coordinate-specific assignments.
- `*survey` headers plus incremental (`lo`/`hi`/`inc`) and explicit (`vals`) settings, both inline;
  incomplete incremental triplets and surveys without values are diagnosed while their source is retained
  and on continuation lines; every malformed continuation diagnostic retains
  the original source line as a recovery record.
- `surv-N` parameter references are checked against the corresponding `*survey N` declaration;
  unresolved references remain in the expression tree and receive a source-located diagnostic.
- `srch-N` parameter references are checked against the corresponding `*search N` declaration
  through the unified semantic-ingestion route; fragment parsers preserve unresolved references
  without claiming that a surrounding search block exists.
- Complete `*search` blocks are required to provide the manual's `xlo`, `xhi`, `xest`, and `dx`
  controls; header-only manual excerpts remain available through fragment parsing.
- Complete `*optimize` blocks require at least one initial `par-N` value, and optional `lo-N`,
  `hi-N`, and `ref-N` controls are checked against their matching parameter number; fragment
  excerpts remain available without speculative completeness diagnostics.
- Documented `*search` and `*optimize` control names require numeric values; nonnumeric
  controls remain represented with source-located diagnostics and recovery records.
- Optimization placeholders such as `opta-N` are checked against both the matching optimize-loop
  letter and its declared `par-N` values through unified semantic ingestion.
- Optimization parameter numbers must be sequential from `par-1`, as required by the manual.
- Optimization loop identifiers are restricted to `a` through `e`, and a problem may contain
  no more than five optimization loops; excess declarations are preserved with
  `too-many-optimize-loops`.
- Duplicate optimization loop letters and duplicate optimization control assignments are
  diagnosed as ambiguous; all source assignments remain in encounter order and are not
  silently reduced to last-write-wins semantics.
- Survey names are checked against output-variable names and collisions are diagnosed without
  discarding either block.
- Duplicate survey/search identifiers, duplicate survey settings (including `vals`), and
  duplicate search-control assignments are diagnosed as ambiguous while all source
  declarations remain available.
- Survey and search identifiers are required to be positive, matching the documented
  `surv-1`/`srch-1` reference vocabulary.
- `*search` objectives with endpoint relationships, multiline `=` continuations, and documented
  search controls, including standalone `min` and `max` search functions. The first term must be an output variable rather than a numeric literal, and its required location is checked; a
  trajectory qualifier written on either side is propagated to the other side
  when the manual permits that shorthand.
- `*radar` station identities, earth-shape selectors, and documented station parameters.
- `*radar` earth-shape selectors and fixed station parameters may continue onto
  subsequent lines; fixed-vocabulary continuation assignments are validated and
  retained on error. Station numbers must be positive and unique, and duplicate
  station parameters are diagnosed without discarding source assignments.
- Direct radar earth geometry requires `reqtr` and exactly one of `rpolr`, `ecc`,
  or `flat`; named WGS-72/WGS-84 selection cannot be mixed with direct geometry.
- `*units/fmt` variable/unit/format records, including inline and continuation-line forms.
- `*atmos` standard/user/site model headers, user/site column headers, and numeric data rows.
- `*earth` documented model families and earth-parameter assignments.
- User/site atmosphere rows retain source locations and diagnose duplicate or
  non-increasing altitude samples; earth headers diagnose conflicting polar-shape
  parameters (`rpolr`, `ecc`, and `flat`). Duplicate global atmosphere/earth
  declarations and duplicate earth parameters are diagnosed as ambiguous while
  preserving all declarations.
- `*wind` coordinate-system headers, the mutually exclusive speed/heading and
  east/north component sets, and table-or-constant assignments.
- Valid `*wind` assignments expose the selected speed/heading or east/north
  component form in the typed model; mixed or incomplete sets retain assignments
  without selecting a form.
- `*file`, `*egs`, and `*print` output-variable lists, including indexed
  variables and the special `*egs summary file` form.
- Each `*print` block is limited to the ten output variables documented by
  the manual; continuation lines count toward the same limit. The manual's
  approximate `*file` line-width limit remains unmodeled rather than being
  converted into an unsupported hard grammar rule.
- `*summarize` operation names, operand-bearing versus unary operations,
  special trajectory functions, and segment/trajectory-qualified operands.
- `*initial` coordinate-specific position/velocity variables, required mass
  input, mutually exclusive `wt`/`mass` and `vel`/`mach`, and both the
  authoritative `from segment N, trajectory M` framing and retained legacy
  ordering. Copied-initial forms accept the manual-documented `t_0` and `omega_0`
  continuation assignments; other continuation assignments are retained and diagnosed
  rather than assigning them direct-initial semantics.
- Repeated assignments in a direct `*initial` block are retained and reported
  as `duplicate-initial-parameter`, avoiding ambiguous last-write-wins meaning;
  recovery records point to the repeated source line.
- Continued `*limits` lines are parsed as additional relationships, while
  header-only `*when` forms preserve and diagnose any following body-looking
  text instead of silently treating it as assignments.
- `*define` simple assignments, C-style `if`/`else` controls with braced
  assignment bodies, nested braced controls, integral headers with initial values,
  and recovery for unmatched or unclosed braces.
- Top-level `*define else` statements must follow a top-level `if`; orphaned
  `else` text remains represented but is diagnosed instead of receiving control-flow meaning.
- Trajectory numbers and names are checked for uniqueness within a problem,
  and segment numbers are checked for uniqueness within each trajectory; duplicate
  declarations remain in the AST with source-located recovery diagnostics.
- Each segment is checked for at least one `*when` final-condition block, as
  required by the manual; missing termination is diagnosed at the segment header
  while the segment and its other blocks remain preserved.
- Each trajectory is checked for the required `*initial` block and at least one
  `*segment`; missing children receive separate source-located diagnostics while
  the trajectory remains preserved.
- `*egs summary` is checked for both prerequisites documented by the manual:
  at least one `*survey` loop and one populated `*summarize` block.
- Radar and relative-vehicle output variables receive the manual’s exact
  subscript count: one in trajectory-scoped output blocks and two in
  problem-scoped `*file`/`*print` blocks; extra subscripts are diagnosed.
- `*define` is dispatched only at problem or trajectory scope; after a segment
  header it closes that segment context instead of becoming a segment block.
- Populated `*define` blocks must assign their declared target variable; incomplete
  fragments remain recoverable without this completion-level diagnostic.
- `*define` temporary assignments must not shadow documented state variables or
  another user-defined target; repeated temporary assignments and duplicate
  user-defined targets are diagnosed while all equations remain in source order.
- Definite forward references to variables assigned later in the same `*define`
  assignment sequence receive `define-variable-used-before-assignment`; control-flow
  branches and names supplied outside the block remain preserved without speculative
  name-resolution claims.
- `*define` expressions accept the manual-documented function vocabulary; unknown
  function calls remain represented but receive an `unsupported-define-function`
  diagnostic and a recovered source record.
- `*define` function calls enforce documented arity, and `table()` is restricted
  to its one table-identification-name argument; malformed calls remain in the
  expression tree with diagnostics and recovery records.
- Problem-scope `*define` blocks diagnose `table()` calls because the manual
  requires table evaluation to be associated with a specific trajectory;
  trajectory-scope definitions retain the documented form.
- Ordinary block assignments retain parenthesized table references but reject
  function-call expressions with `unsupported-assignment-call` rather than
  assigning them `*define` semantics.
- `*when`, `*limits`, and `*summarize` operands reject nested function calls
  outside their explicitly documented forms, while retaining the source for
  recovery and diagnostics.
- `*when` conditions enforce the manual's single-relationship form using `=`,
  `<`, or `>`; compound boolean or nested relational conditions are retained
  with `unsupported-when-condition` rather than being assigned runtime meaning.
- `*limits` accepts the documented `l/d-max` spelling and rejects the guidance
  rules the manual excludes from limiting (`intercept`, `prop`/`propnav`,
  `downria`, and `upria`).
- `*fly` guidance-table continuation rows as typed independent/value points,
  with row-level source locations and recovery for malformed rows.
- documented direct/condition guidance variables and `interp-1` through
  `interp-3` are checked explicitly; unknown guidance vocabulary is preserved
  with a diagnostic.
- `*units/fmt` unit tokens are checked against the manual's allowable-unit
  table, format-only records are represented with an absent unit, and unknown
  units remain represented with diagnostics.
- `*units/fmt` permits format changes for variables declared by `*define` but
  diagnoses attempted unit changes; variables not provably declared in the
  current file remain preserved without speculative semantic rejection.
- `*reset` and `*increment` enforce the manual's mutually exclusive `wt`/`mass`
  rule, coordinate-system grouping for position and velocity state variables,
  and the prohibition on absolute-time discontinuities in multiple-trajectory
  problems; invalid assignments remain preserved in their blocks.
- `*units/fmt` continuation records validate variable names and preserve the
  original line when a variable, unit, or format is malformed.
- `*rail` headers validate the documented `cfstat` and `cfslid` parameters;
  unknown rail parameters are preserved with diagnostics.
- `*integ` validates `dt`, `dtprnt`, and `dtguid`; `*reset` and `*increment`
  validate the documented state-variable vocabulary, including deployment
  variable `velibx`, on both headers and continuation lines.
- Fixed-vocabulary continuation records for `*dwn/crs`, `*earth`, `*iip`,
  `*inertial`, `*rail`, and `*tangent` are diagnosed without discarding their
  parsed assignments.
- `*aero` preserves arbitrary user-defined assignments but diagnoses mixing
  the documented coefficient families CA/CN, CL/CD/CS, and CX/CY/CZ.
- User-defined assignments in `*constants`, `*aero`, `*cg`, and `*prop` cannot
  shadow documented state-variable names; fixed block parameters remain accepted
  and invalid user-variable assignments are preserved with `reserved-user-variable`.
- `*prop` preserves arbitrary user-defined assignments while validating the
  fixed `thr_units` and `mdt_units` controls against the manual's propulsion
  unit tables.
- multiline `*title` bodies as title text rather than ignored block lines.
- full-table `if` operations with the manual-required `then` keyword and a
  single `<`, `>`, or `=` relationship; compound or incomplete conditions are
  diagnosed while their nested operation remains recoverable.
- full-table `csto` and `goto` operations require identifier destinations;
  invalid numeric or table-call operands are retained with diagnostics.
- full-table `start` is a body delimiter only; repeated or in-body occurrences
  are preserved as unparsed text with a recovery diagnostic.
- Full-table `end` is enforced as the final operation; trailing text is synchronized
  at the next table declaration and reported without losing that following table.
- Full-table format detection distinguishes a standalone `start` delimiter from a
  simple-table assignment named `start`.
- full-table table calls require a nonempty parenthesized list of identifier
  arguments; malformed calls are retained with an `invalid-table-call`
  diagnostic, and the `end` delimiter cannot be used as a nested operation.
- Search and optimization endpoint syntax rejects simultaneous bracketed and
  keyword trajectory qualifiers, and optimization headers require both endpoint
  numbers.
- table names and operation labels preserve the manual's historical
  alphanumeric/hyphen/dot naming forms (including `1st-stage`), while malformed
  punctuation and independent-variable names receive diagnostics.
- table headers reject simultaneous `extrap` and `no-extrap` options while
  retaining the first source-declared setting.
- table-header option values are restricted to numeric or identifier atoms;
  malformed parenthesized values synchronize at the next table declaration.
- simple and interpolation data assignment names are checked as identifiers;
  malformed names remain represented with an `invalid-table-assignment-name`
  diagnostic.
- assignment-based `*aero`, `*constants`, `*cg`, `*prop`, `*integ`,
  `*inertial`, `*reset`, and `*increment` bodies diagnose residual text after
  otherwise valid assignments instead of silently discarding it.
- Data-block assignments retain undocumented compound or relational operators for
  inspection but report `unsupported-assignment-operator`; only the documented
  `=` assignment form receives unambiguous assignment semantics.
- Fixed numeric parameter blocks (`*dwn/crs`, `*earth`, `*iip`, `*integ`,
  `*inertial`, `*radar`, `*rail`, and `*tangent`) reject names and table references
  as numeric values while allowing numeric literals and numeric survey placeholders;
  repeated fixed parameters are diagnosed without last-write-wins reduction.
- body-attitude `*fly` variables are checked as a segment-level set against
  the nine consistent angle sets documented in Manual Table 4-3.
- `*fly` blocks are limited to four per segment, and wildcard values are
  rejected in a trajectory's first segment because there is no preceding
  segment value to inherit; the wildcard remains represented in the AST.
- A segment using `intercept` or `propnav` is limited to three `*fly` blocks
  because each special rule consumes two control variables.
- simple-table independent-variable assignments are checked against the order
  declared in the table header; source examples with reversed order are
  preserved with a semantic diagnostic.
- full-table interpolation groups likewise check that their independent-value
  assignments follow the order in the table-call argument list.

The parser deliberately retains some documented complex bodies as raw statements rather than
inventing typed semantics. Raw retention in those areas is the supported preservation boundary,
not an assertion that runtime semantics have been implemented. A table is not marked
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
- extend the explicit unit-dimension registry to newly recovered state variables;
- deepen typed projections and runtime semantics without weakening the
  source-preserving acceptance boundary.
