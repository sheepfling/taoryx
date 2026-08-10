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
- `packages/` - separately buildable model and provider plug-ins
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
python -m tools.dev install-check
python -m tools.dev doctor
```

Bootstrap installs the core plus all four official model/provider plug-ins for
contributors. Core-only, model-suite, full-suite, wheelhouse, optional sensor,
and Windows instructions are in [Installing Taoryx and its model packages](docs/INSTALLATION.md).
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

Inspect installed vehicle/model plug-ins without constructing a plant:

```bash
taoryx plugins list
taoryx plugins list --json
taoryx model list
taoryx model assess --output build/model-assessment.json
taoryx model plan taoryx.registry.mission-composition hummingbird --fidelity pseudo_6dof
```

`taoryx model assess` is the compact all-model readiness matrix: it shows
advertisement, control, adapter, tuning, lowering, and declared-blocker status
for every realization. Generate a plain-value mission draft, validate it
through its exact provider, or run a model-owned campaign through the common
automatic-tuning pipeline with `taoryx model scaffold`, `taoryx model compile`, and
`taoryx model tune`. See
[Model-to-mission authoring and automation](docs/architecture/model-authoring-automation.md)
for the complete data, waypoint, segment, controller, and plug-in workflow.
Model plug-ins can generate common LQR/LQI campaigns from the compact
`ControlAutomationDeclaration`; `taoryx model tune` content-addresses reports
under `build/controller-cache` unless `--no-cache` is selected.

The core wheel contains the language and simulation host. Optional wheels own
DAVE-ML, Simple Aero, reference vehicles, and the reachability workbench. The
reachability commands become available when `taoryx-reachability` is installed;
without it the core CLI fails closed with an installation hint.

The plug-in contract and extraction status are documented in
[Installable model and provider plug-ins](docs/architecture/plugins.md).

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

- [Installation and package selection](docs/INSTALLATION.md)
- [Agent workflows](docs/AGENT_WORKFLOWS.md)
- [Mission Composition front door](docs/MISSION_COMPOSITION.md)
- [Model-to-mission authoring and automation](docs/architecture/model-authoring-automation.md)
- [Simulation Runtime onboarding: find, set up, step, and diagnose](docs/SIMULATION_RUNTIME_ONBOARDING.md)
- [Authoring → Runtime → Composition showcase guide](docs/AUTHORING_RUNTIME_COMPOSITION_SHOWCASE.md)
- [Simulation Runtime maturity plan](docs/plan/simulation-runtime-maturity.md)
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
