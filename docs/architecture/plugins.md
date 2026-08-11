# Installable model and provider plug-ins

Taoryx separates the language and simulation host from optional vehicle,
model-format, and provider packages. The host contract lives in
`taoryx.plugins`; packages publish implementations through the standard Python
entry-point group `taoryx.plugins`.

This is a software-discovery boundary. It does not weaken the existing source,
fidelity, qualification, or execution claims. A discovered adapter still has
to pass the same registry, semantic preflight, conformance, and evidence gates
as a bundled adapter.

## Installation

The copy-paste core, model-suite, full-suite, contributor, wheelhouse, and
optional-dependency recipes live in
[Installing Taoryx and its model packages](../INSTALLATION.md). A full install
can be audited without source fallbacks or plant construction with:

```bash
taoryx plugins check --profile full
```

## Package boundary

The intended distribution shape is:

```text
taoryx
  language, compiler, simulation engine
  units, frames, state/control/telemetry contracts
  typed registries and plug-in discovery
  common CLI and result envelopes

taoryx-simple-aero
  analytical Simple Aero providers and workflow assets

taoryx-daveml
  DAVE-ML parsing, semantic import, evaluation, and replay

taoryx-reference-models
  family adapters, source assets, missions, registered tuning inputs,
  and model qualification records

taoryx-reachability
  reachability envelopes, terminal criteria, plotting, continuation
  X-15/HL-20/passive-body mission overlays and reachability qualification

taoryx-cadac
  optional source-bound CADAC actor catalog, compatibility runtimes,
  and canonical table conversion
```

A package may contribute several coherent models. Package boundaries should
follow dependency, evidence, maintenance, or ownership boundaries; Taoryx does
not require one wheel per vehicle.

## Entry point and API version

An external package declares one entry point:

```toml
[project.entry-points."taoryx.plugins"]
"example.flight-models" = "example_flight_models.plugin:PLUGIN"
```

The entry-point name must exactly match `PluginMetadata.id`. The loaded value
may be a `TaoryxPlugin` object or a zero-argument factory returning one.

```python
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar


def register(registrar: PluginRegistrar) -> None:
    registrar.register_family_adapter(MY_ADAPTER_REGISTRATION)
    registrar.register_trajectory_provider(MY_PROVIDER)
    registrar.register_mission_composition_provider(MY_COMPOSER_PROVIDER)
    registrar.register_controller_tuning_campaign(MY_TUNING_CAMPAIGN)


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="example.flight-models",
        package="example-flight-models",
        version="2.3.0",
        api_version="1",
        description="Example source-backed flight models.",
    ),
    register_callback=register,
)
```

`api_version` versions the host/plug-in registration contract independently
from the package version. An incompatible API is rejected before any staged
contribution is merged.

## Typed registry aggregation

The plug-in catalog is a registry of typed registries, not one arbitrary
dictionary:

```text
installed entry points
        |
        v
  per-plug-in staging registrar
        |
        +-- model_format
        +-- reachability_provider
        +-- family_adapter
        +-- model
        +-- mission_capability_adapter
        +-- semantic_preflight_handler
        +-- execution_factory
        +-- episode_factory
        +-- batch_episode_parity_verifier
        +-- controller_tuning_campaign
        +-- trajectory_provider
        +-- mission_composition_provider
        |
        v
validated immutable PluginCatalog
```

Identity is unique within a contribution kind. Two model contributions may not
claim the same model ID, and two family-adapter contributions may not claim the
same family ID. Different kinds do not collide merely because their IDs match.
Installation or callback order never selects a winner: collisions fail closed.

Registration is atomic per plug-in. The host stages all contributions in a
private `PluginRegistrar`, validates compatibility and collisions, and only
then merges them. A failing callback cannot leave half of a package installed
in the active catalog.

Every public contribution record retains:

- contribution kind and stable ID;
- plug-in ID;
- distribution name and version; and
- a catalog fingerprint over loaded identities and ownership.

Executable values are intentionally absent from the serialized catalog.

## Discovery behavior

```python
from taoryx.plugins import discover_plugins

catalog = discover_plugins()
provider = catalog.contribution(
    "trajectory_provider",
    "reference.point_mass",
).value
```

Reachability is another exact typed registry rather than a core singleton:

```python
workbench = catalog.build_reachability_provider_registry().provider(
    "taoryx.reachability.workbench",
)
```

Mission Composition providers have their own typed composer registry. Model
identity is resolved inside the selected provider namespace:

```python
composer = catalog.build_mission_composition_provider_registry()
models = composer.list_models("taoryx.registry.mission-composition")
f16 = composer.model("taoryx.registry.mission-composition", "f16_s119")
```

Model-owned controller inputs are projected into a separate typed registry;
the numerical campaign sequence remains a core host service:

```python
campaigns = catalog.build_controller_tuning_campaign_registry()
campaigns.validate_against(composer)
report = campaigns.registration(
    "hummingbird-pseudo-hover-attitude-v1"
).run()
```

This split is described in
[Model-to-mission authoring and automation](model-authoring-automation.md).

The reference-model plug-in registers both the full 11-model production
registry (`taoryx.registry.mission-composition`) and the two-model analytical
reference provider (`taoryx.reference.mission-composition`). The production
provider therefore crosses the same installed plug-in boundary as its model
and execution contributions; a host does not need a repository-local import
to reach the complete composer advertisement.

Discovery loads installed Python code, but registration must not construct a
plant, parse a large model asset, or execute a trajectory. Register lightweight
providers and factories; defer expensive work until an exact model/fidelity
operation is selected.

Strict discovery raises on load, compatibility, registration, and collision
failures. Diagnostic surfaces can use `strict=False`; the failing plug-in is
then omitted atomically and appears in `PluginCatalog.diagnostics`.

Hosts may disable exact plug-in IDs or disable bundled/external discovery. The
CLI exposes the same diagnostic view:

```bash
taoryx plugins list
taoryx plugins list --json
taoryx plugins inspect taoryx.reference-models
taoryx plugins list --disable example.flight-models
```

`--no-builtin` and `--no-external` make installation/removal behavior directly
testable. Python plug-ins are trusted installed code; this mechanism is not a
sandbox for arbitrary uploaded model archives.

## Assets and provenance

An extracted package must load its own read-only data through
`importlib.resources`. It must not calculate paths from the Taoryx repository
root. Immutable source hashes, family IDs, adapter IDs, model IDs, execution
factory IDs, schema versions, and claim boundaries must survive extraction.

Moving a model is not evidence promotion. Existing fixtures and qualification
records move with their authority, or remain in the canonical repository with
an explicit cross-package provenance reference. A package may be installed and
discoverable while all of its execution operations remain development or
blocked.

## Current package set

The monorepo builds five independent wheels:

| Distribution | Plug-in ID | Owned implementation |
| --- | --- | --- |
| `taoryx` | host only | language, parser/compiler, engine, generic contracts, plug-in discovery, typed registry aggregation, and a delegating reachability CLI surface |
| `taoryx-daveml` | `taoryx.daveml` | DAVE-ML collections, import, semantic IR, evaluation, atmosphere/inertia binding, and replay |
| `taoryx-simple-aero` | `taoryx.simple-aero` | Simple Aero builder, validation, fixtures, fidelity ladder, Mission Composition schema, and `reference.point_mass` provider |
| `taoryx-reference-models` | `taoryx.reference-models` | X-15, HL-20, NESC, passive body, X8, B747, A320, F-16, and Hummingbird source models, non-reachability planners/adapters/executors, registered tuning campaigns, parity verifiers, Mission Composition providers/sessions, and model assets |
| `taoryx-reachability` | `taoryx.reachability` | generic envelope solver, catalogs, terminal criteria, timeout continuation, plotting, X-15/HL-20/passive-release studies, four reachability planners/preflights, three execution factories, and `taoryx.reachability.workbench` |
| `taoryx-cadac` | `taoryx.cadac` | source-bound CADAC actor catalog, compatibility runtimes, `taoryx.table.v1` conversion, and source-hash provenance; upstream CADAC values are not redistributed |

The reference-model distribution depends on the DAVE-ML and Simple Aero
distributions because its combined Mission Composition registry advertises
those formats and workflows. The reachability distribution depends on the
reference-model distribution because it composes studies over those source
models. The dependency is intentionally one-way: reference models do not
depend on reachability and remain discoverable and executable without it.
Each distribution has its own `pyproject.toml` and standard entry point.

All executable vehicle/model and reachability-workbench modules have been
removed from the core source tree and core wheel. The shared `taoryx` namespace
preserves established imports when the owning wheel is installed; importing an
absent optional capability fails at the package boundary. A core-only
installation can import every shipped core module, reports an empty plug-in
catalog, and rejects `taoryx reachability ...` with an instruction to install
`taoryx-reachability`.

Source-checkout development adds the four sibling `src` roots and exposes
the same plug-ins through a checkout-only fallback. Installed wheels use only
entry-point discovery, and an installed distribution suppresses its matching
source fallback so a plug-in cannot be registered twice.
