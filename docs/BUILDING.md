# Building and validating the TAOS reconstruction

This page is the practical command reference for rebuilding the manual,
auditing equations, and running the validation routes that keep the source,
grammar, runtime, and analysis layers aligned.

## System dependencies

The complete documentation and QA workflow uses:

- Python 3.12+
- `latexmk` and a TeX Live installation with the packages used by `manual/styles/taos.sty`
- `qpdf`
- Poppler tools: `pdfinfo`, `pdftotext`, and `pdftoppm`
- `pandoc` and `xelatex` for the composite extension PDF and selected QA reports

A typical Debian/Ubuntu installation requires TeX Live's recommended, science, pictures, and extra collections in addition to the PDF and Python tools.

## First setup

The portable bootstrap path is:

```bash
python3 tools/dev.py bootstrap   # macOS/Linux
py -3.12 tools/dev.py bootstrap  # Windows
```

On macOS/Linux, activate with `source .venv/bin/activate`; on Windows, use
`.venv\\Scripts\\activate`.

This creates `.venv` and attempts to install the package with all development,
parser, manual-QA, packaging, and analysis dependencies from `pyproject.toml`.
When the network is unavailable, the bootstrap falls back to an offline
editable install that reuses the system site packages already present on the
machine. Once `.venv` exists, `tools/dev.py` prefers it automatically even if
you do not activate it first.

## Source PDF

The original TAOS scan is intentionally not vendored. When you need a local
copy for provenance audits, use one of the external references below and place
the file at `TAOS_manual_1995.pdf` in the repo root:

- [Direct UNT PDF](https://digital.library.unt.edu/ark:/67531/metadc623738/m2/1/high_res_d/162896.pdf)
- [Archive.org record](https://archive.org/details/taos-users-manual-1995)

The repository treats that PDF as an external input, not a long-term source
artifact.

To fetch the scan into the local cache and create the ignored root symlink:

```bash
python3 tools/dev.py source-pdf
```

To inspect an existing environment without changing it:

```bash
python3 tools/dev.py doctor
python3 scripts/doctor.py --strict
```

For the complete PDF workflow, use the documentation-specific diagnosis:

```bash
python3 tools/dev.py docs-doctor
```

This checks Python packages, both LaTeX engines, `latexmk`, Pandoc, Poppler,
`qpdf`, and the frozen manual plus successor LaTeX sources. A missing tool is
reported with its PATH status. On macOS, the external tool bundle is typically
installed with `brew install mactex-no-gui pandoc poppler qpdf`; on Debian or
Ubuntu, install `texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended
latexmk pandoc poppler-utils qpdf`.

The strict form fails when required Python packages or core manual-build tools
are unavailable. Optional audit/report tools are reported as warnings.

Manual installation remains equivalent to:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
```

The repository uses `pyproject.toml` as the single dependency source.

The dependency-free runtime uses the TAOS reference integrators by default.
Explicit Euler is available as `--integrator euler` for fast smoke tests and
diagnostic runs; use RK4 for fixed-step production work. To
enable optional SciPy `solve_ivp` backends for performance comparisons, install
the extra and inspect the available methods:

```bash
python -m pip install -e '.[scipy]'
taoryx integrators list
taoryx run path/to/problem.prb --integrator scipy-dop853
```

SciPy is optional and does not replace the reference integrator in validation
or historical-equivalence claims. The current adapter preserves the runtime's
step and event boundaries, so it is a method-comparison backend rather than a
guaranteed performance improvement.

## Common routes

The portable task runner is the authoritative interface:

```bash
python3 tools/dev.py doctor
python3 tools/dev.py lint
python3 tools/dev.py typecheck
python3 tools/dev.py test
python3 tools/dev.py test-views
python3 tools/dev.py manual
python3 tools/dev.py check
```

The older, source-heavy validation workflow remains available as
`python tools/dev.py check`.

For selective pytest runs, including the `slow`, `artifact`, and `simple_aero`
markers, see [BUILDING_TESTS.md](BUILDING_TESTS.md).

For the most common project entry points:

```bash
python tools/dev.py grammar          # parser, lexer, corpus, EBNF, and manual-fixture validation
python tools/dev.py test-simple_aero     # Simple Aero problem / segment / trajectory corpus
python tools/dev.py test-equations   # equation catalog and provenance checks
python tools/dev.py test-algorithms  # algorithm catalog and runtime binding checks
taoryx-validate examples/chapter04/ballistic-reentry.prb
taoryx table inspect examples/chapter03/stmi-full.tbl --html build/table.html
```

## Rebuild the manual

```bash
python tools/dev.py manual
```

The normalized document is written to `build/manual.pdf`.

## Build the TAORYX successor guide

The reconstructed manual remains historically scoped. Successor-side features
are documented in a separate LaTeX guide so that fidelity modes, plant data,
controller synthesis, trajectory scoring, and verification policy can evolve
without changing the 1995 reconstruction:

```bash
python tools/dev.py successor-guide
```

The normalized guide is written to
`output/pdf/taoryx_extensions_and_verification.pdf`. Its source is
`docs/latex/taoryx_extensions_and_verification.tex`.

The manual-parallel language and mathematics reference is built separately:

```bash
python tools/dev.py language-reference
```

Its normalized output is
`output/pdf/taoryx_language_reference.pdf`, sourced from
`docs/latex/taoryx_language_reference.tex`. It is organized around successor
mathematics, problem grammar, and table syntax to parallel the historical
manual's Methods, Problem Files, and Table Files coverage.

## Junior PDF workflow

Run the doctor first, then choose either the complete build or one target:

```bash
python tools/dev.py docs-doctor
python tools/dev.py all-pdfs
```

The individual targets are:

| Command | Output |
| --- | --- |
| `python tools/dev.py manual` | `build/manual.pdf` — frozen historical manual rebuild |
| `python tools/dev.py successor-guide` | `output/pdf/taoryx_extensions_and_verification.pdf` |
| `python tools/dev.py language-reference` | `output/pdf/taoryx_language_reference.pdf` |
| `python tools/dev.py taoryx-extension-pdf` | `output/pdf/taoryx_extensions_composite.pdf` |
| `python tools/dev.py all-pdfs` | all four outputs above |

`all-pdfs` never edits the historical manual source. It rebuilds the existing
frozen manual PDF, the separate successor LaTeX documents, and their composite.
If a build fails, rerun `docs-doctor`, then run the individual target named in
the failing command to isolate the missing tool or source problem.

## Build the composite TAORYX extension PDF

The repeatable composite stage combines the LaTeX verification guide, the
manual-parallel language reference, and the canonical Markdown extension
references, then normalizes the merged PDF:

```bash
python tools/dev.py taoryx-extension-pdf
```

The result is written to
`output/pdf/taoryx_extensions_composite.pdf`. The stage rebuilds the base guide,
renders the extension appendix with Pandoc/XeLaTeX, merges the PDFs in a fixed
order, and removes its temporary build directory. The full `check` task runs
this composite stage as its documentation build gate.

## Rebuild the equation provenance registry

```bash
python tools/dev.py equation-audit
```

This verifies all 326 numbered equations against the compiled AUX file, the TeX
sources, the 307-page source registry, and the original PDF. It regenerates:

- `metadata/equations_provenance.csv`
- `metadata/equations_provenance.json`
- `qa/equation_provenance_audit_v21.md`
- `qa/TAOS_equation_provenance_audit_v21.pdf`
- `qa/equation-source-pages/`

## Run validation

```bash
python tools/dev.py check
python -m pytest
```

The validation suite covers the reconstructed manual, source-page and figure
registries, parser fixtures, equation provenance, PDF interoperability, and
selected numerical relationships.

## Build the parser wheel

```bash
python -m build --wheel
```

The current wheel is also distributed under `dist/`.

## Create the Codex handoff bundle

```bash
python tools/dev.py handoff
```

The handoff script creates a clean, checksum-manifested ZIP containing source
data, LaTeX, metadata, parser code, tests, tools, QA assets, build products,
and final deliverables.
