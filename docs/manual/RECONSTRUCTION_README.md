# TAOS 1995 Manual Reconstruction and Language Tooling

This repository reconstructs the 1995 *Trajectory Analysis and Optimization System (TAOS) User's Manual* (SAND95-1652, Version 96.0) as semantic, maintainable LaTeX and provides the foundation of a typed `.tbl` / `.prb` parser and validator.

## Version 21 handoff milestone

The repository is prepared for transition to a live Codex codebase.

### Documentation coverage

- All **307 physical source pages** are represented.
- The front matter, nomenclature, Chapters 1-4, Appendix, References, historical Index, and Distribution pages are reconstructed.
- All **326 numbered equations** are editable LaTeX and have complete provenance records.
- All **72 numbered figures** are editable semantic graphics or semantic listings.
- The normalized manual is text- and vector-based and contains no raster image objects.
- The Appendix provides **171 machine-readable output-variable entries**.
- The original source PDF is referenced by the provenance records, but is not
  present in this repository drop; retain it as an external source input.

### Equation provenance closure

`metadata/equations_provenance.csv` and `metadata/equations_provenance.json` map every equation to:

- its canonical equation number;
- LaTeX label;
- source manual page and physical PDF page;
- source PDF SHA-256;
- reconstructed page and section;
- TeX file, line range, and label line;
- SHA-256 of the exact LaTeX display;
- verification scope and implementation boundary.

The canonical sequence is:

- 1-1 through 1-2;
- 2-1 through 2-315;
- 3-1;
- 4-1 through 4-8.

Run `python tools/dev.py equation-audit` to regenerate and verify the registry and the six-page provenance report.

External source references:

- [Direct UNT PDF](https://digital.library.unt.edu/ark:/67531/metadc623738/m2/1/high_res_d/162896.pdf)
- [Archive.org record](https://archive.org/details/taos-users-manual-1995)

### Language tooling

`src/taoryx.language` is an installable Python 3.12 package with:

- typed Pydantic AST nodes for problem files, problems, trajectories, segments, tables, operations, and all 33 documented data-block keywords;
- a precedence-aware expression parser;
- hierarchical `.prb` parsing;
- simple and full `.tbl` parsing;
- source locations and diagnostics;
- structural and selected semantic validation;
- a `taoryx-validate` CLI;
- pytest coverage for all extracted manual fixtures and block dispatch.

This is the canonical parser and runtime foundation. `taoryx run` executes the
supported evidence-bounded `.prb`/`.tbl` subset; it is not yet a complete
lossless language implementation or a historically equivalent TAOS runtime.

## Build

See [BUILDING.md](../BUILDING.md) for environment details.

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py check
```

The normalized manual is written to `build/manual.pdf`.

## Parser examples

```bash
python -m taoryx.language.cli examples/chapter04/ballistic-reentry.prb
python -m taoryx.language.cli --json examples/chapter03/stmi-full.tbl
taoryx-validate file.prb file.tbl
```

## Final Codex handoff bundle

```bash
python tools/dev.py handoff
```

The handoff bundle contains:

- the original source PDF;
- all LaTeX sources and editable figures;
- all machine-readable metadata and grammars;
- the parser package, tests, and tools;
- source-page renders and QA assets;
- the reconstructed manual and audit reports;
- the parser wheel;
- a complete file-level SHA-256 manifest.

Start with [CODEX_HANDOFF.md](CODEX_HANDOFF.md) and [AGENTS.md](../../AGENTS.md) when opening the repository in Codex.

## Validation boundaries

All numbered equations are transcribed and provenance-mapped. The promoted
algorithm catalog records which equation groups have executable bindings and
focused tests; remaining equation records are documentation-only or planned.
Chapter 2 validators provide selected section-level algebraic and numerical
checks.

The extracted `.tbl` and `.prb` files are parser-validated documentation fixtures. They have not been run against the historical TAOS 96.0 executable. Historical printouts are source transcriptions, and reconstructed trajectory plots were not regenerated with the historical binary.

The manual is not a diplomatic transcription across the whole source. It is a semantic technical edition rather than a complete word-for-word transcription outside the completed fidelity tranches. Consult the original scan for exact historical wording and appearance.

## Repository map

- `manual/manual.tex`, `manual/frontmatter/`, `manual/chapters/`, `manual/backmatter/`: canonical manual sources
- `manual/styles/`, `manual/math/`, `manual/figures/`: shared LaTeX and editable graphics
- `metadata/`: canonical semantic registries and generated provenance
- `grammars/`: documentary EBNF grammars
- `examples/`: extracted TAOS fixtures and historical listings
- `src/taoryx.language/`: parser and validator package
- `tests/`: pytest suite
- `tools/`: build, QA, audit, and packaging scripts
- `qa/`: generated reports, source renders, and comparison assets
- `dist/final/`: principal release products

Release-specific details are recorded in `RELEASE_NOTES_v21.md`.
