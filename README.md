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
- `tests/fixtures/problem_file_dumps/` — staging area for future problem-file
  dumps and segment-family fixture collections. Current leaves:
  `spectre_segments/` and `spectre_trajectories/`
- `tests/fixtures/taos_manual_corpus_v22/` — source-faithful Chapter 3/4
  snippet corpus and parser/lexer baseline wrappers
- `tests/fixtures/spectre_simple_aero_v1/` — data-only Spectre simple-aero
  seed corpus with shared scaffold and maneuver-family variants
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
python3 tools/dev.py test-views
python3 tools/dev.py test-grammar
python3 tools/dev.py test-equations
python3 tools/dev.py test-algorithms
python3 tools/dev.py test-spectre
python3 tools/dev.py manual
python3 tools/dev.py check
taoryx-validate examples/chapter04/ballistic-reentry.prb
```

Test selection is documented in
[docs/BUILDING_TESTS.md](/Users/rick/LocalStorage/GIT_LOCAL/active/taoryx/docs/BUILDING_TESTS.md).
The grammar, equations, and algorithms selections are overlapping views;
`slow`, `artifact`, and `spectre` are cost/output categories.
Bare `python -m pytest` uses the fast default and excludes those three
categories; use `test-all` when you want every test.

Once `.venv` has been bootstrapped, the task runner automatically prefers it
even without activation. `taoryx.language` parses and validates source files;
`taoryx.runtime` lowers the supported `.prb`/`.tbl` subset into executable
trajectories and writes outputs through `taoryx run`. The catalog's 107 typed
algorithm contracts now have executable bindings and unit verification; remaining
work is deeper parser semantics, trajectory-level performance/regression coverage,
and historical comparison evidence.

The runtime keeps the dependency-free TAOS integrators (`rk4` and `rkf45`) as
the reference path. Explicit Euler (`euler`) is also selectable for fast smoke
tests and derivative-pipeline diagnostics, but is not recommended for
production accuracy. If the optional SciPy extra is installed, `solve_ivp`
backends such as `scipy-dop853` are selectable for numerical cross-checks and
method experiments:

```bash
taoryx integrators list
taoryx run problem.prb --integrator scipy-dop853
```

SciPy-backed results are not yet claimed historically equivalent, and the
current stepwise event boundary contract may make them slower rather than
faster. Compare them against the reference integrator before validation.

For the manual build and provenance workflow, see
[docs/BUILDING.md](/Users/rick/LocalStorage/GIT_LOCAL/active/taoryx/docs/BUILDING.md).
For pytest marker selections and artifact output handling, see
[docs/BUILDING_TESTS.md](/Users/rick/LocalStorage/GIT_LOCAL/active/taoryx/docs/BUILDING_TESTS.md).
