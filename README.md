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

## First-class API products

The repository's reusable contracts are products in their own right, not only
interfaces to the bundled vehicle models. In particular, the
[Vehicle Composition Advertisement API](docs/architecture/vehicle-composition-advertisement-api.md)
lets any trajectory backend publish a closed, JSON-safe account of its mission
authoring, execution, graph, node, transition, rearrangement, runtime-change,
multi-entity, and state-transfer capabilities. A backend may wrap a native
simulator, remote service, analytical model, or recorded data without adopting
the Taoryx runtime internally.

The broader [Mission Composition Provider API](docs/architecture/mission-composition-provider-api.md)
adds typed configuration, exact operation selection, batch results, failures,
and persistent sessions. Together they are the provider-neutral integration
surface for catalogues, planners, UIs, agents, and independent trajectory
providers.

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

Bootstrap installs the core plus the fourteen direct model/overlay plug-ins used
for development. The compatibility aggregate is an explicit opt-in profile for
existing catalogue consumers, rather than a dependency of new vehicle work.
Core-only, model-suite, compatibility, full-suite, wheelhouse, optional sensor,
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
python -m tools.dev quality
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
taoryx model plan taoryx.hummingbird.mission-composition hummingbird --fidelity pseudo_6dof
taoryx model overview --provider taoryx.hummingbird.mission-composition --model hummingbird --output build/hummingbird-model-card.md
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
For a reviewer-facing model/fidelity/provenance/tuning/segment-parameter
summary that stays tied to those installed contracts, use
[`taoryx model overview`](docs/architecture/vehicle-model-cards.md).

The core wheel contains the language and simulation host. Optional wheels own
DAVE-ML, development-only debug providers, Simple Aero, the standalone A320,
F-16, Hummingbird, and X-15 families, the remaining reference vehicles, and the reachability
workbench. The reachability commands become available when
`taoryx-reachability` is installed; without it the core CLI fails closed with
an installation hint.

The plug-in contract and extraction status are documented in
[Installable model and provider plug-ins](docs/architecture/plugins.md).
The repository-level core/direct-package/compatibility topology and focused
verification model are documented in
[Repository and package architecture](docs/architecture/repository-architecture.md).

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
- [Mission Composition and interactive-control front door](docs/MISSION_COMPOSITION.md) — the consumer API for model discovery, typed configuration, batch execution, and persistent live-control sessions
- [Vehicle Composition Advertisement API](docs/architecture/vehicle-composition-advertisement-api.md) — the standalone capability-negotiation contract for Taoryx and independent trajectory backends
- [Mission Composition Provider API reference](docs/architecture/mission-composition-provider-api.md)
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
- [Repository and package architecture](docs/architecture/repository-architecture.md)
- [Table explorer / table plotter boundary](docs/architecture/table-explorer.md)
- [Simple Aero corpus description](docs/grammar/e2e-suite-v23.md)
- [Tumbling analysis workspace](analysis/tumbling/README.md)

## Scope

This repository aims to be source-linked and evidence-bounded. It reconstructs
the manual, provides typed parsing and validation for the documented language,
and implements a growing subset of the runtime and analysis stack.

It does not claim historical TAOS 96.0 runtime equivalence without a trusted
historical executable and output corpus.
