# TAOS reconstruction - Codex handoff

## Handoff state

This repository is ready to become a live Codex codebase. The documentation reconstruction is complete at the semantic level and the equation transcription/provenance layer is closed.

### Completed

- All 307 physical pages of the 1995 source are represented.
- All 326 numbered equations are transcribed, labeled, source-page mapped, TeX-location mapped, and display-hashed.
- All 72 numbered figures are editable semantic graphics or semantic listings; the final manual contains no raster image objects.
- The Appendix contains 171 machine-readable output-variable entries.
- References, historical Index, and Distribution pages are reconstructed.
- Ten extracted `.tbl` / `.prb` fixtures are parser-backed and structurally validated.
- `src/taoryx.language` provides an installable Python 3.12 parser/validator with typed Pydantic AST nodes and CLI diagnostics.
- `src/taoryx.runtime` lowers parsed `.prb`/`.tbl` files into runtime problems and tables, executes supported trajectories through `taoryx run`, and writes deterministic print, file, and summary outputs.
- All 107 algorithm-catalog rows are promoted with executable bindings and unit verification paths; historical equivalence remains explicitly unverified in the generated status ledger.
- The full manual, QA reports, original source PDF, parser wheel, scripts, metadata, tests, and generated products are included in the handoff bundle.

### Explicitly not complete

- The parser does not assign typed runtime semantics to every complex block body; unsupported or
  ambiguous bodies are source-preserved and diagnosed under the evidence-bounded grammar contract.
- The 326 equations are not all independently implemented as executable Python functions; the catalog and equation registries distinguish implemented subsets from transcription-only records.
- No runtime-equivalence claim has been established against TAOS 96.0.
- The manual remains a semantic edition rather than a complete word-for-word diplomatic transcription outside the completed fidelity tranches.

## Canonical starting points

- Manual root: `manual/manual.tex`
- Parser package: `src/taoryx.language/`
- Language grammars: `grammars/`
- Original source: `TAOS_manual_1995.pdf` (referenced by provenance; not included
  in this repository drop)
- Equation registry: `metadata/equations_provenance.json`
- Source-page registry: `metadata/source_page_coverage.csv`
- Build and QA commands: `tools/dev.py`
- Agent instructions: `AGENTS.md`

## Recommended next milestones

1. Extend defaults, unit dimensions, cross-block restrictions, and lossless
   formatting in `taoryx.language`.
2. Complete parser defaults, unit dimensions, cross-block restrictions, and
   lossless formatting.
3. Add trajectory-level regression baselines and resolve large synthetic
   optimization performance.
4. Establish historical comparison evidence when an executable reference or
   trusted output set is available.

## Release boundaries

The equation provenance audit proves transcription and traceability, not numerical implementation completeness. Section-level validators contain selected algebraic and numerical checks only. Historical output listings are transcriptions from the source manual and were not regenerated with the original executable.
