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

## Direct developer route

New vehicle and model work starts from the `developer` profile, not the
compatibility aggregate. It installs the core, every direct family/model-format
plug-in, and the reachability overlay, but deliberately omits
`taoryx-reference-models`. A developer therefore selects the provider owned by
the package under work instead of accidentally exercising the historic
aggregate catalog.

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
taoryx plugins check --profile developer
python tools/dev.py check-developer-plugins

# Examples of normal vertical loops.
python tools/dev.py check-vehicle f16_s119
python tools/dev.py check-vehicle hummingbird
python tools/dev.py test-vehicle cadac_aim5
```

Use `compatibility` only to install the aggregate and its declared dependency
closure for an existing consumer. It excludes the independent debug,
passive-body, and reachability packages; use `full` when a release or
migration explicitly needs both broad direct and aggregate paths. The
compatibility package owns no vehicle runtime; new plug-ins must neither import
it nor depend on it.

Direct providers use the core's
`taoryx.trajectory.catalog_mission_composition.CatalogMissionCompositionProvider`
when a packaged catalog fragment supplies the standard Composition API. The
older registry-named host remains solely a compatibility import for existing
aggregate consumers.

The current exemplar packages are deliberately complementary:

- `taoryx-debug-models` is the smallest development-only provider;
- `taoryx-a320` is a focused reduced vehicle with an advertised batch-only
  controller screen;
- `taoryx-f16` is a source-backed family with a shared DAVE-ML dependency;
- `taoryx-reachability` is an optional typed overlay; and
- `taoryx-nesc` with `taoryx-passive-bodies` demonstrates an explicit
  cross-plug-in deployment binding.

They are reference shapes, not mandatory feature inventories: publish only the
controls, factories, evidence, and dependencies a model actually supports.

### Remaining compatibility seam

The only intentionally retained aggregate surface is the provider ID
`taoryx.registry.mission-composition`, supplied by
`taoryx-reference-models`. Its factory now receives the selecting
`PluginCatalog` explicitly. The old registry-named core class and unscoped
vehicle-catalog resource resolver remain a migration seam for existing direct
callers; new plug-ins must receive or preserve a selected catalog instead.
They are not part of the direct developer profile, package dependency graph,
or focused vehicle test loop.

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

taoryx-dual-launch
  synthetic dual-launch glider provider, source lowering, batch projection,
  and workflow assets; it does not consume the compatibility aggregate

taoryx-daveml
  DAVE-ML parsing, semantic import, evaluation, and replay

taoryx-debug-models
  development-only analytical ballistic/waypoint and contract-probe Mission
  Composition providers, selectable control/session fixtures, and package-owned
  workflow endpoint witnesses

taoryx-a320
  OpenAP point-mass and explicitly surrogate pseudo-6DOF A320 products,
  focused Composition provider, reduced controls, local named-coordinate LQI
  screen, and A320-only data/evidence; it does not consume the reference-model
  aggregate

taoryx-f16
  F-16 S.119 source assets, reductions, focused Composition provider, reduced
  guidance controls, and bounded local physical-controller screens; it consumes
  DAVE-ML but not the reference-model aggregate

taoryx-hummingbird
  AscTec Hummingbird source assets, family adapter, Composition provider,
  control campaigns, physical rotor screens, and family-owned evidence

taoryx-nesc
  NASA/NESC Scenario 17 source-replay assets, adapters, focused Composition
  provider, and stage-separation parent contract; it consumes DAVE-ML only

taoryx-passive-bodies
  reusable tumbling/released-body assets, direct-release provider and witnesses,
  and independently selectable child propagation runtimes

taoryx-x15
  X-15 source assets, local direct-wrench and source-surface controller screens,
  focused Composition provider, controller campaigns, and family-owned evidence

taoryx-hl20
  HL-20 Mod K DAVE-ML source fixture, local direct-wrench and seven-surface
  controller screens, focused Composition provider, controller campaigns, and
  family-owned trim/evidence bindings; it consumes DAVE-ML but not the reference-model
  aggregate

taoryx-source-table-fixed-wing
  shared Skywalker X8 and B747 source-table plants, route catalogs, focused
  Composition providers, package-owned route episode binding, local controller
  screens, and family-owned witnesses; it consumes only core host services

taoryx-reference-models
  compatibility aggregate provider and package-scoped compatibility data only;
  core owns generic maturity, parity, and integration audit utilities. It consumes the independently owned
  source-table fixed-wing, A320, F-16, Hummingbird, NESC, Simple Aero, Dual Launch, X-15,
  and HL-20 fragments

taoryx-reachability
  reachability envelopes, terminal criteria, plotting, continuation
  X-15/HL-20/passive-release mission overlays and reachability qualification;
  it consumes the X-15 and HL-20 family packages for those overlays

taoryx-cadac
  optional source-bound CADAC actor catalog, compatibility runtimes,
  and canonical table conversion
```

A package may contribute several coherent models. Package boundaries should
follow dependency, evidence, maintenance, or ownership boundaries; Taoryx does
not require one wheel per vehicle.

The adopted ownership rule, focused package split, test boundary, and criteria
for subsequent vehicle splits are documented in
[Vehicle plug-in isolation](vehicle-plugin-isolation.md).
For the developer recipe for a new vehicle package—including controls,
authority profiles, factories, data ownership, and verification—start with
[Authoring a vehicle plug-in](vehicle-plugin-authoring.md).
Its per-realization fidelity definition of done is
[Fidelity tiers and vehicle plug-in requirements](fidelity-data-requirements.md).

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

### Declaration-first discovery

The `taoryx.plugins` entry-point group is the sole discovery authority. Its
name establishes the plug-in ID, its target establishes the import target, and
the owning distribution establishes the package/version identity. Discovery
orders plug-ins by that declared ID, rejects malformed or duplicate names
before importing a target, and rejects a loaded plug-in whose metadata ID,
package, or version disagrees with its declaration. A selected plug-in ID must
have a declaration; there is no name-based import fallback or first-winner
ordering rule.

The shared `taoryx` namespace remains an import-compatibility mechanism for
family-owned implementation modules. It is not scanned to discover plug-ins.
Likewise, Taoryx does not use Pluggy for this boundary: the typed registrar is
the extension contract, while standard Python entry points provide packaging,
installation, and deterministic discovery semantics.

In a source checkout, the fallback reads each active sibling package's same
`pyproject.toml` entry-point declaration rather than maintaining a separate
module list. Installed distributions take precedence over an identically named
source declaration. This makes a wheel and its checkout fallback advertise the
same identity and target while still allowing a focused source path to expose
only its selected package.

Inspect declarations without importing plug-in targets:

```bash
# Installed distribution metadata only.
taoryx plugins entry-points --no-builtin --json

# Every sibling source declaration, even with only core on PYTHONPATH.
taoryx plugins entry-points --no-external --json

# Installed profile: distribution, declaration, loaded metadata, and registration.
taoryx plugins check --profile full --json
```

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
        +-- trim_evidence_binding
        +-- deployment_child_runtime
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

## Version and revision detection

Discovery has two deliberately different update signals. `PluginMetadata.version`
is the package-maintained release or compatibility identifier. The catalog also
publishes `plugin_revisions`: one `taoryx.plugin-revision/v1` record per loaded
plug-in with its package/API versions, owned contribution count, and a
SHA-256 fingerprint over that plug-in's identity and contributions only. A
focused client can compare `catalog.plugin_revision(plugin_id)` (or the JSON
record) without invalidating its view just because another plug-in changed.
The existing catalog `fingerprint` remains the token for a complete host
environment.

Mission Composition metadata uses the same distinction. Every provider and
model has a nonempty `version`; serialized provider and model metadata also
carry a derived `metadata_fingerprint`. The provider catalog adds a complete
catalog fingerprint, a provider revision, and a compact model `revision`
record containing the model version plus its configuration/output schema IDs
and fingerprints. This lets UI, agent, and cache clients distinguish a
compatible model release from any advertised-metadata update.

Increment a model's `version` for an intentional compatibility or behavioral
release (physics, controls, supported operations, source-data meaning, or
contract change). The derived fingerprint still changes for every precise
metadata correction, including documentation and provenance. Neither value is
evidence of numerical qualification; provenance and focused witnesses retain
that role.

Use the non-executing discovery views to obtain the values:

```bash
taoryx plugins entry-points --json
taoryx plugins list --json
taoryx plugins inspect taoryx.a320 --json
taoryx model list --provider taoryx.a320.mission-composition
```

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
models = composer.list_models("taoryx.f16.mission-composition")
f16 = composer.model("taoryx.f16.mission-composition", "f16_s119")
```

Select `taoryx.registry.mission-composition` in the same way only after
installing the `compatibility` or `full` profile.

Selected multi-object releases use a separate exact runtime registry. A parent
advertises the deployment boundary; the composition explicitly names the child
plug-in, runtime, model, fidelity, and state-transfer policy. The common host
never guesses a child model or falls back to a catalogue-wide discovery scope.
If the selected child runtime is absent or owned by a different plug-in, the
operation fails before it can claim a child trajectory.

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

`taoryx-debug-models` registers the two development-only analytical and
contract-probe providers. `taoryx-a320` registers the focused
`taoryx.a320.mission-composition` provider, its OpenAP/reduced surrogate
adapters, named-control LQI screen definition and static screen advertisement,
and A320-only runtime contributions. `taoryx-f16` registers the focused
`taoryx.f16.mission-composition` provider, its F-16 source reductions, local
physical screens, associated controller campaigns, and a lazy
`vehicle_interface_extension` for pseudo-6DOF body-rate reference/readback
semantics. Core joins that additive contract and validates it, but does not
name F-16 controls. `taoryx-hummingbird` registers the focused
`taoryx.hummingbird.mission-composition` provider and all Hummingbird runtime
contributions. `taoryx.nesc` registers the focused
`taoryx.nesc.mission-composition` provider, its source-replay adapter and
batch endpoint, and the stage-separation parent contract.
`taoryx-passive-bodies` registers the focused `taoryx.passive-bodies.mission-composition` provider and the reusable
`tumbling_body` child runtime. `taoryx-x15` registers the focused
`taoryx.x15.mission-composition` provider, source-local direct-wrench and
source-surface screens, controller campaigns, and X-15-owned source/evidence
assets. Its focused provider intentionally excludes the optional staged
reachability overlay; that endpoint is exposed only by the aggregate when
`taoryx-reachability` is installed. `taoryx-hl20` registers the focused
`taoryx.hl20.mission-composition` provider, source-local direct-wrench and
seven-surface screens, controller campaigns, and its lazy
`reference_hl20_mod_k` trim-evidence binding. `taoryx-source-table-fixed-wing`
registers the focused `taoryx.x8.mission-composition` and
`taoryx.b747.mission-composition` providers, their source-table adapters,
route assets, package-owned episode binding, local screens, and campaigns.
`taoryx-dual-launch` registers the focused
`taoryx.dual-launch.mission-composition` provider, its synthetic launch-form
schema, native lowering, normal batch projection, and endpoint fragment. It
does not require the compatibility aggregate or another workflow package.
`taoryx-reference-models` retains the compatibility aggregate
(`taoryx.registry.mission-composition`), but it consumes installed
non-overlapping family fragments rather than owning every model's source assets
or factories. The minimal `compatibility` profile exposes its ten declared
family/workflow dependencies; `full` adds the optional passive-body model for
the eleven-model aggregate. A host therefore has a small focused
provider for vertical A320, F-16, HL-20, Hummingbird, NESC, X8, B747, X-15, Dual Launch, or passive-body work and a
backward-compatible aggregate for catalogue consumers.

Discovery loads installed Python code, but registration must not construct a
plant, parse a large model asset, or execute a trajectory. Register lightweight
providers and factories; defer expensive work until an exact model/fidelity
operation is selected.

### Deferred family registration

When a contribution's concrete object imports numerical dependencies, parses a
catalog, or constructs Pydantic schemas, publish its stable identity first and
use the deferred registrar forms:

```python
registrar.register_family_adapter_factory("example_family", build_family_registration)
registrar.register_mission_composition_provider_factory("example.provider", build_provider)
registrar.register_semantic_preflight_handler_callback("example.translator.v1", preflight)
registrar.register_execution_factory_request_v1("example.batch.v1", run_batch)
registrar.register_trim_evidence_binding_factory("example_source_family", trim_binding)
```

The host records these IDs during discovery and validates a concrete family
identity when its adapter registry is built, its source trim evidence is
selected, or a concrete provider identity when its provider API is selected.
This lets a UI list or select plug-ins without importing unrelated plant code;
it does not defer or weaken validation once a caller selects the adapter,
trim binding, provider, preflight, or batch endpoint.

For Mission Composition, a deferred provider also advertises its stable
provider ID to the registry. Creating the registry can therefore index a
focused provider without importing its catalog, schemas, or numerical runtime;
accessing that provider's metadata, models, or configuration API resolves it
and checks the advertised ID against its concrete metadata.

If a host discovers a focused catalog, it should pass that catalog to the
public preflight, lowering, batch, episode, and policy-replay APIs. Core
execution preserves the scope for nested capability, factory, witness, and
parity resolution, preventing an isolated host from rediscovering unrelated
installed plug-ins during runtime. A deferred focused Mission Composition
provider also binds its creating catalog, so the common runner and session
manager preserve that scope for canonical vehicle routes.

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

The direct `developer` profile installs the core plus thirteen independent
plug-in distributions. The `full` profile adds the separate compatibility
aggregate for migration/release integration; the source-bound CADAC integration
is an additional optional wheel:

| Distribution | Plug-in ID | Owned implementation |
| --- | --- | --- |
| `taoryx` | host only | language, parser/compiler, engine, generic contracts, plug-in discovery, typed registry aggregation, and a delegating reachability CLI surface |
| `taoryx-daveml` | `taoryx.daveml` | DAVE-ML collections, import, semantic IR, evaluation, atmosphere/inertia binding, and replay |
| `taoryx-debug-models` | `taoryx.debug-models` | development-only analytical `taoryx.reference.mission-composition` and contract-probe providers; no physical vehicle assets |
| `taoryx-a320` | `taoryx.a320` | OpenAP point-mass and explicitly surrogate JSBSim/OpenAP pseudo-6DOF A320 assets, catalog fragments, reduced adapters, `taoryx.a320.mission-composition`, native-coordinate LQI screen definition/advertisement, and A320-only execution/episode/parity witnesses; it consumes only core host services |
| `taoryx-f16` | `taoryx.f16` | F-16 S.119 source assets, catalog fragments, 3DOF/pseudo-6DOF reductions, a lazy F-16-only semantic-interface extension for pseudo-6DOF body-rate control/readback, `taoryx.f16.mission-composition`, local physical-control screens, controller campaigns, and F-16-only execution/episode/parity witnesses; it consumes DAVE-ML only |
| `taoryx-hummingbird` | `taoryx.hummingbird` | AscTec Hummingbird source tables/assets, fidelity and endpoint fragments, native and pseudo adapters, `taoryx.hummingbird.mission-composition`, control campaigns, and Hummingbird-only execution factories/witnesses |
| `taoryx-nesc` | `taoryx.nesc` | NASA/NESC Scenario 17 source-replay assets, catalog fragments, point/pseudo adapters, `taoryx.nesc.mission-composition`, source-replay preflight/execution, and the stage-separation parent contract |
| `taoryx-passive-bodies` | `taoryx.passive-bodies` | reusable tumbling/released-body source fragments, direct-release adapters and witnesses, and the `taoryx.passive-bodies.local-atmosphere-release.v1` child runtime; it has no parent-vehicle import dependency |
| `taoryx-simple-aero` | `taoryx.simple-aero` | Simple Aero builder, validation, fixtures, fidelity ladder, `reference.point_mass`, the focused `taoryx.simple-aero.mission-composition` provider, and its workflow endpoint fragment/witness |
| `taoryx-dual-launch` | `taoryx.dual-launch` | synthetic dual-launch family metadata, source-problem lowering, `taoryx.dual-launch.mission-composition`, normalized batch projection, and its workflow endpoint fragment/witness; it has no compatibility-aggregate dependency |
| `taoryx-x15` | `taoryx.x15` | X-15 source tables, catalog fragments, direct-wrench and source-surface local screens, `taoryx.x15.mission-composition`, controller campaigns, and X-15-only execution/episode/parity witnesses; its focused API excludes the optional reachability-owned staged overlay and it consumes only core host services |
| `taoryx-hl20` | `taoryx.hl20` | HL-20 Mod K DAVE-ML source fixture, catalog fragments, direct-wrench and seven-surface local screens, source trim evidence, `taoryx.hl20.mission-composition`, controller campaigns, and HL-20-only witnesses; its focused API excludes the reachability-owned overlays and it consumes DAVE-ML only |
| `taoryx-source-table-fixed-wing` | `taoryx.source-table-fixed-wing` | shared Skywalker X8 and B747 source-table plants, catalogs, source mission/table assets, `taoryx.x8.mission-composition` and `taoryx.b747.mission-composition`, package-owned route episode binding, local controller screens, campaigns, and family-only witnesses |
| `taoryx-reference-models` | `taoryx.reference-models` | compatibility aggregate provider/session and package-scoped compatibility data only; generic maturity, parity, and integration audit utilities live in core. It consumes source-table fixed-wing, A320, F-16, Hummingbird, NESC, Simple Aero, Dual Launch, X-15, and HL-20 fragments rather than carrying a vehicle runtime |
| `taoryx-reachability` | `taoryx.reachability` | generic envelope solver, catalogs, terminal criteria, timeout continuation, plotting, X-15/HL-20/passive-release studies, three lazy reachability planners/preflights, two lazy composition execution factories, and `taoryx.reachability.workbench` |
| `taoryx-cadac` | `taoryx.cadac` | source-bound CADAC actor catalog, compatibility runtimes, `taoryx.table.v1` conversion, and source-hash provenance; upstream CADAC values are not redistributed |

The F-16, NESC, and HL-20 distributions depend on DAVE-ML because their
source-backed runtimes retain the published DAVE-ML parser boundary. The
reference-model distribution depends on source-table fixed-wing, DAVE-ML,
A320, F-16, Hummingbird, NESC, Simple Aero, Dual Launch, X-15, and HL-20 because its
compatibility aggregate advertises their installed workflows. The debug-provider,
A320, Hummingbird, passive-body, Simple Aero, Dual Launch, source-table fixed-wing, and
X-15 plug-ins consume no sibling Taoryx model plug-in. All focused family
packages remain discoverable and executable without the reference aggregate. A NESC
composition can select a passive child only when that separately installed
plug-in is present; this is a runtime binding, not a NESC-package dependency.
The reachability distribution depends directly on the X-15 and HL-20
distributions because it composes studies over those source families; it has no
compatibility-aggregate dependency. Its entry-point registration advertises
stable capability/preflight/execution identities without importing the source
planners or simulators. These dependencies are intentionally one-way: family
packages do not depend on reachability and remain discoverable and executable
without it. Each distribution has its own `pyproject.toml` and standard entry
point.

All executable vehicle/model and reachability-workbench modules have been
removed from the core source tree and core wheel. The shared `taoryx` namespace
preserves established imports when the owning wheel is installed, but it never
participates in plug-in enumeration; importing an absent optional capability
fails at the package boundary. A core-only
installation can import every shipped core module, reports an empty plug-in
catalog, and rejects `taoryx reachability ...` with an instruction to install
`taoryx-reachability`.

Source-checkout development exposes only sibling plug-in `src` roots that are
explicitly active on `sys.path`, then reads those packages' own entry-point
declarations. Installed wheels use standard entry-point metadata, and an
installed distribution suppresses its matching source fallback so a plug-in
cannot be registered twice.
