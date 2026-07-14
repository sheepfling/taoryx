# taoryx

Python successor to the 1995 Trajectory Analysis and Optimization System
(TAOS). The repository combines the reconstructed manual, its machine-readable
metadata and examples, and the emerging Python implementation.

## Repository map

- `manual/manual.tex`, `manual/frontmatter/`, `manual/chapters/`, `manual/backmatter/`, `manual/figures/`,
  `manual/styles/`, and `manual/math/` — canonical editable manual sources
- `metadata/` — equation, figure, source-page, fixture, and domain registries
- `grammars/` — documentary EBNF for `.tbl` and `.prb` files
- `docs/grammar/` — grammar-validation workflow and coverage boundaries
- `verification/` — claim definitions, evidence policy, ambiguity control, and
  requirement-to-test traceability
- `examples/chapter03/` and `examples/chapter04/` — extracted TAOS fixtures
- `tests/fixtures/taos_manual_corpus_v22/` — source-faithful Chapter 3/4
  snippet corpus and parser/lexer baseline wrappers
- `src/taoryx/` — the installable Python package
- `tests/parser/` and `tests/unit/` — parser and package tests
- `tools/` — build, audit, generation, and validation tooling
- `docs/manual/` — editorial notes, handoff guidance, and reconstruction context for the manual sources
- `docs/architecture/` — design notes for the implementation boundaries and catalog layers
- `docs/grammar/` — parser/validator workflow notes and evidence boundaries for `.tbl` / `.prb`
- `docs/qa/` — retained QA guidance and visual review reports
- `build/` and `qa/` — local generated manual and audit products
- `.venv/` — local development environment; `INBOX/` — ignored temporary intake
- `tools/fitz_compat.py` and `tools/fitz.py` — local PDF compatibility shim
  used by historical audit tools when PyMuPDF is unavailable

Generated build products, QA renders, release bundles, checksums, and other
handoff artifacts are intentionally not part of the long-term source tree.
The ignored `INBOX/` directory remains available for future drops.

## Development

Recommended first-time setup on macOS/Linux:

```sh
python tools/dev.py bootstrap
source .venv/bin/activate
python tools/dev.py doctor
```

The portable Python entry point is:

```sh
python3 tools/dev.py bootstrap   # macOS/Linux
py -3.12 tools/dev.py bootstrap  # Windows
```

On Windows, use `python`/`py` directly and activate with `.venv\\Scripts\\activate`.

The bootstrap tries a full editable install first and falls back to an offline
editable install that reuses system site packages if the network is unavailable.

The doctor is read-only. Use `python3 tools/dev.py doctor` when missing
required packages or manual-build tools should fail the command.

To fetch the external source PDF into the local cache and create the ignored
root symlink, run `python3 tools/dev.py source-pdf`.

Common workflows:

```sh
python3 tools/dev.py lint
python3 tools/dev.py typecheck
python3 tools/dev.py grammar
python3 tools/dev.py test
python3 tools/dev.py manual
python3 tools/dev.py check
taoryx-validate examples/chapter04/ballistic-reentry.prb
```

Once `.venv` has been bootstrapped, the task runner automatically prefers it
even without activation. `taoryx.language` parses and validates source files;
`taoryx.runtime` lowers the supported `.prb`/`.tbl` subset into executable
trajectories and writes outputs through `taoryx run`. The remaining work is
expanding the planned P1/P2 catalog surface, language defaults, and historical
comparison evidence.

For the manual build and provenance workflow, see
[docs/BUILDING.md](/Users/rick/LocalStorage/GIT_LOCAL/active/taoryx/docs/BUILDING.md).
