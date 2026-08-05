# taoryx

`taoryx` is the Python reconstruction of the 1995 TAOS User's Manual and the
typed `.tbl` / `.prb` toolchain that grows out of it. The repository keeps the
manual, equation provenance, grammar, runtime, analysis tools, and tests tied
together so each layer can be rebuilt and checked from the same source tree.

## Big Picture

The project is larger than a manual rebuild. The manual is the historical
anchor, but the active work is to turn that source into a typed, testable, and
evidence-bounded toolchain that can:

- reconstruct the manual as editable LaTeX with provenance;
- parse and validate `.tbl` and `.prb` files with source-located diagnostics;
- map equations and algorithms to executable Python bindings;
- provide runtime, inspection, and analysis entry points for real workflows;
- keep regression coverage around the parser, runtime, Simple Aero corpus, and
  numeric helpers; and
- preserve the boundary between what is documented, what is executable, and
  what still needs historical confirmation.

## What lives here

- `manual/` - canonical editable LaTeX for the reconstructed manual
- `metadata/` - equation, figure, source-page, fixture, and catalog registries
- `grammars/` - documentary EBNF for TAOS table and problem files
- `src/taoryx/` - the installable Python package
- `tests/` - parser, runtime, equation, algorithm, Simple Aero, and artifact tests
- `tools/` - build, audit, validation, and reporting commands
- `analysis/` - focused numerical studies and generated analysis helpers
- `docs/` - user-facing guidance for the manual, grammar, equations, runtime, and verification layers
- `examples/` - extracted fixtures and historical example material

Generated build products, QA renders, and release bundles are intentionally
kept out of the long-term source tree.

## Start Here

Set up the local Python environment first:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
python -m tools.dev doctor
```

If you already have a working environment, the portable runner will use `.venv`
automatically when it exists.

## Core Workflows

Audit the equation provenance registry:

```bash
python -m tools.dev equation-audit
```

Run the main validation gates:

```bash
python -m tools.dev check
python -m pytest
```

Validate the grammar and manual-backed parser corpus:

```bash
python -m tools.dev grammar
python -m tools.dev test-grammar
taoryx-validate examples/chapter04/ballistic-reentry.prb
```

Inspect a table and, when helpful, render a standalone HTML view:

```bash
taoryx table inspect examples/chapter03/stmi-full.tbl
taoryx table inspect examples/chapter03/stmi-full.tbl --html build/table-explorer.html
```

Run the Simple Aero corpus tests when you want the problem/segment/trajectory
fixtures:

```bash
python -m tools.dev test-simple_aero
```

Rebuild the reconstructed manual when you need the published PDF or want to
refresh the page-normalized source build:

```bash
python -m tools.dev manual
```

For the complete junior-friendly documentation PDF workflow, diagnose tools
first and then rebuild all historical and successor PDFs:

```bash
python -m tools.dev docs-doctor
python -m tools.dev all-pdfs
```

Individual PDF targets and outputs are listed in [docs/BUILDING.md](docs/BUILDING.md).

Use the analysis tree for focused studies and generated helpers:

```bash
python tools/aero_drag_analysis.py --all --output-dir build/aero-drag
```

## What To Read Next

- [Agent workflows](docs/AGENT_WORKFLOWS.md)
- [Product 2 onboarding: find, set up, step, and diagnose](docs/PRODUCT_TWO_ONBOARDING.md)
- [Three-product showcase guide](docs/THREE_PRODUCT_SHOWCASE.md)
- [Product 2 maturity plan](docs/plan/product-two-maturity.md)
- [Manual build and provenance workflow](docs/BUILDING.md)
- [Test selections and markers](docs/BUILDING_TESTS.md)
- [Manual reconstruction notes](docs/manual/README.md)
- [Grammar and validation boundary](docs/grammar/README.md)
- [Equation registry](docs/equations/registry.md)
- [Equation implementation bindings](docs/equations/implementation-bindings.md)
- [Runtime architecture](docs/architecture/README.md)
- [Table explorer / table plotter boundary](docs/architecture/table-explorer.md)
- [Simple Aero corpus description](docs/grammar/e2e-suite-v23.md)
- [Tumbling analysis workspace](analysis/tumbling/README.md)

## Scope

This repository aims to be source-linked and evidence-bounded. It reconstructs
the manual, provides typed parsing and validation for the documented language,
and implements a growing subset of the runtime and analysis stack.

It does not claim historical TAOS 96.0 runtime equivalence without a trusted
historical executable and output corpus.
