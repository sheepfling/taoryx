# Repository and package architecture

TAORYX is one repository with three deliberately different extension surfaces:
the standalone provider-neutral **trajectory contracts**, the language/runtime
**core**, and independently installable **model plug-ins**. The historical
manual, semantic parser, generic simulation contracts, and plug-in host belong
to core. A vehicle's plant, source data, controls, composition endpoints, and
focused evidence belong to the package that advertises that vehicle.

This page is the top-level map. Use
[vehicle plug-in isolation](vehicle-plugin-isolation.md) for the detailed
ownership and verification evidence, and
[authoring a vehicle plug-in](../developer/vehicle-plugin-authoring.md) for the contributor
workflow. The provider-neutral capability product shared by every direct
package is the
[Vehicle Composition Advertisement API](../api/vehicle-composition-advertisement-api.md).

## Target topology

```text
          taoryx-trajectory-contracts wheel
     discovery + batch + optional streaming + ECEF
                               ▲
                               │ TAORYX is one provider through an adapter
                               │
                  taoryx core wheel
 language + simulation + TAORYX-native schemas + discovery + generic execution
                               |
                               | declares host API / loads entry points
                               v
                   selected PluginCatalog
                               |
              +----------------+----------------+
              |                                 |
              v                                 v
      direct vehicle plug-in             optional overlay plug-in
  plant + data + provider + tests      append-only endpoint/data additions
              |                                 |
              +----------------+----------------+
                               v
              selected catalog fragment resources
                               |
                               v
              generic composition / batch / session APIs

    compatibility aggregate (explicit opt-in, legacy consumers only)
              |
              v
    historical unscoped catalogue and legacy facade adapters
```

A normal developer or deployment host selects exactly the plug-ins it intends
to offer. Discovery turns their entry points into an immutable `PluginCatalog`.
That catalog carries plug-in identity, version, revision fingerprint, and typed
contributions. A focused provider receives both its resolved vehicle catalog
and the active `PluginCatalog`; nested generic work therefore remains in the
same selected scope.

## Ownership and layout

| Location | Owns | Must not own |
| --- | --- | --- |
| `packages/taoryx-trajectory-contracts/` | versioned provider-neutral composition, batch, streaming-control, ECEF, and conformance interfaces | TAORYX language/runtime imports, vehicle data, or model execution |
| `src/taoryx/` | language/toolchain, simulation engine, generic schemas, discovery, generic composition/compiler/session contracts | a package-specific plant, source deck, vehicle endpoint, or family test fixture |
| `packages/taoryx-*/` | one independently installable contribution, its source/data/assets, adapter, provider, controls, focused tests, and package README | hidden imports of sibling model packages or the compatibility aggregate |
| `packages/taoryx-reference-models/` | the explicit compatibility aggregate and adapters for older catalogue consumers | new vehicle ownership or the normal authoring path |
| `verification/` | canonical editable registry material that package extractors project into owned package data | an implicit runtime dependency for an installed focused wheel |
| `tests/families/<family>/` | the family’s narrow vertical proof | catalogue-wide revalidation as an inner-loop requirement |
| `tests/unit/` | core contracts, discovery rules, compatibility behavior, and cross-package deployment rules | ownership of a family-specific numerical claim |

The namespace package layout is an implementation detail for sharing the
`taoryx` import namespace. Distribution discovery is performed by Python entry
points, not by namespace scanning or module-name ordering. Every plug-in
distribution must declare exactly one `taoryx.plugins` entry point, stable
metadata, and an independently buildable package.

## Selected scope versus compatibility scope

New code follows these rules:

1. A host discovers the desired entry points and retains the resulting
   `PluginCatalog`.
2. A direct vehicle provider constructs
   `CatalogMissionCompositionProvider` with its own
   `ResolvedVehicleCompositionCatalog`. This constructor rejects an omitted
   catalog.
3. Generic resource, preflight, batch, and session helpers receive or inherit
   that selected catalog. Missing package data is a failure; it never borrows a
   sibling wheel or the aggregate registry.
4. Cross-plug-in behavior is modeled as an explicit typed dependency or an
   append-only overlay. A released/tumbling child is the current example: a
   parent names the passive-body runtime it needs rather than importing all
   vehicle packages.

The following surfaces are compatibility-only and remain supported for existing
callers while migration continues:

- `RegistryMissionCompositionProvider()` with no catalog;
- unscoped `load_resolved_vehicle_composition_catalog()` calls;
- unscoped `vehicle_catalog_resource(s)` calls, which delegate to
  `taoryx.compatibility.vehicle_catalog_resources`; and
- legacy convenience vehicle symbols exposed through `taoryx.trajectory`.

They are not examples for a new package. The compatibility adapter is kept
separate so selected-path imports do not need its historical static ordering.
No new direct package may depend on `taoryx-reference-models` or import the
legacy aggregate provider module or the broad `taoryx.trajectory` facade; the
developer-route validator enforces this. Direct packages may still import a
narrow core module such as `taoryx.trajectory.catalog_mission_composition`.

## Package profiles

The bootstrap profiles express intent rather than accidental installation
state:

| Profile | Purpose |
| --- | --- |
| core | language and generic simulation host only |
| models | direct vehicle/model packages, excluding optional overlays |
| developer | all direct packages, including optional overlay development |
| compatibility | only the aggregate and its declared dependencies |
| full | developer plus the explicit compatibility aggregate |

Use `taoryx plugins check --profile developer` to verify that installed entry
points match the intended direct route. `taoryx plugins entry-points` exposes
declarations without constructing model implementations, which makes version
and revision changes visible to tooling before a model is materialized.

## Verification model

The test pyramid is intentionally vertical and scoped:

1. `python tools/dev.py check-developer-plugins` checks entry-point metadata,
   dependency direction, legacy-import exclusion, and the installed developer
   profile.
2. `python tools/dev.py test-vehicle <family>` runs one fast composition slice.
   `check-vehicle <family>` adds that family’s interface, witness, and relevant
   parity checks.
3. `python tools/dev.py check-plugin-contract <plugin>` is the fast inner-loop
   check: it validates only that plug-in’s direct-route dependency rules and
   TAORYX-universal Mission Composition contract. `python tools/dev.py check-plugin <plugin>`
   adds its focused installed-wheel proof. The universal check
   constructs only that plug-in’s providers, audits their advertised schemas,
   requires every host-facing TAORYX model to have matching registered common
   `batch` and `step` tuples, and verifies the mandatory standard ECFC/ECEF
   result/session types. The reusable provider protocol remains capability
   based, so an external provider may honestly advertise batch-only execution.
   It treats a plug-in with no trajectory provider as not applicable rather
   than loading unrelated providers. Its wheel command,
   `python tools/verify_plugin_wheels.py --plugin <plugin> --python
   .venv/bin/python`, builds fresh source/wheels, installs only the selected
   boundary and declared direct dependencies in a temporary target, removes
   source fallback, and exercises the installed result. It is the package
   installation proof.
   The compatibility aggregate is intentionally different: because its
   provider composes the direct package catalog, its `check-plugin-contract`
   path validates that declared aggregate scope.
4. Catalogue-wide tests and the full `pytest` suite are integration/release
   work, not the default evidence required to change a single plug-in.

A family change should normally run levels 2–3 for that package, then broaden
only when a typed shared contract or declared cross-package dependency changes.

## Migration state and rules of engagement

The direct packages already own the physical families, debug workflows,
Simple Aero, Dual Launch, DAVE-ML, CADAC, passive bodies, and reachability
overlay work. The remaining architectural work is intentionally bounded:

- migrate core-facing callers away from unscoped compatibility helpers where a
  selected catalog is available;
- reduce the oversized `taoryx.trajectory` convenience facade to a documented
  compatibility surface as package-owned imports become the normal route; and
- eventually rename or supersede the `taoryx-reference-models` distribution
  with an explicitly named compatibility distribution after downstream users
  have migrated.

Do not resolve these by reintroducing a root-level static package list or by
making every focused gate import the aggregate. New work should add a typed
contribution, its version/revision metadata, a focused vertical test, and an
installed-wheel proof instead.

## Related documents

- [Installable model and provider plug-ins](plugins.md)
- [Vehicle plug-in isolation](vehicle-plugin-isolation.md)
- [Vehicle plug-in authoring](../developer/vehicle-plugin-authoring.md)
- [Mission Composition Provider API](../api/mission-composition-provider-api.md)
- [Standalone trajectory contracts](../api/trajectory-contracts.md)
- [Model-to-mission authoring and automation](../developer/model-authoring-automation.md)
- [Installation and package profiles](../INSTALLATION.md)
