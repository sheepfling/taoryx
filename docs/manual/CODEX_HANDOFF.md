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
- `src/taoryx.language` provides an installable Python 3.12 parser/validator foundation with typed Pydantic AST nodes and CLI diagnostics.
- The full manual, QA reports, original source PDF, parser wheel, scripts, metadata, tests, and generated products are included in the handoff bundle.

### Explicitly not complete

- The parser is not yet a full lossless grammar and semantic implementation for every complex block body.
- The 326 equations are not all implemented as executable Python functions and do not each have an independent unit test.
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

1. Fully type the internal languages of `*define`, `*fly`, `*optimize`, `*search`, `*survey`, and `*summarize`.
2. Add defaults, unit dimensions, cross-block restrictions, and lossless formatting to `taoryx.language`.
3. Map every equation record to an implementation symbol or an explicit
   documentation-only status under `src/taoryx/equations/`.
4. Add property tests for coordinate transforms, geodesy, atmosphere, guidance, searches, and inverse relationships.
5. Add trajectory-level regression tests when an executable reference implementation or validated result set is available.

## Release boundaries

The equation provenance audit proves transcription and traceability, not numerical implementation completeness. Section-level validators contain selected algebraic and numerical checks only. Historical output listings are transcriptions from the source manual and were not regenerated with the original executable.
