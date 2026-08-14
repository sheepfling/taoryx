# taoryx-source-table-fixed-wing

The source-table fixed-wing plug-in owns the Skywalker X8 and B747 research
surrogates as one coherent boundary. They share table-backed local plants,
language-backed racetrack lowering, catalog fragments, and source witnesses;
separating them into two thin distributions would leave both coupled through
the same runtime.

The package supplies two focused Mission Composition providers:

- `taoryx.x8.mission-composition`
- `taoryx.b747.mission-composition`

Each exposes only its own family’s models, fidelity tiers, controls, feedback,
and endpoint metadata. The local source-table controller screens remain
batch-only development evidence; they are not route-level flight qualification.
The legacy `taoryx-reference-models` package retains the aggregate provider and
consumes this plug-in’s fragments rather than owning these plants.

Install it with the core package:

```bash
python -m pip install -e . -e packages/taoryx-source-table-fixed-wing
```

For focused lower-tier development use:

```bash
python tools/dev.py check-vehicle skywalker_x8
python tools/dev.py check-vehicle b747
python tools/verify_plugin_wheels.py --plugin source-table-fixed-wing --python .venv/bin/python
```

Both focused family gates begin by verifying that the shared package-data tree
is an exact fresh extract of the canonical X8/B747 inputs:

```bash
python tools/extract_source_table_fixed_wing_plugin_assets.py --check
```

This is a package-boundary check only; changing X8 does not execute B747's
runtime suite (or vice versa). The shared static data remains intentional
because both providers depend on the same source-table lowering runtime.

The X8 and B747 gates are intentionally limited to the public point-mass and
pseudo-6DOF racetrack interfaces, their four batch/episode witnesses, their
two parity traces, and their focused provider/compiler/batch/session proof.
They do not rerun the direct-wrench, source-elevon, or transport local-controller
screens; those remain separate physical-controller evidence. B747's fast
vertical proof uses a clearly labeled bounded batch prefix, while its normal
execution witnesses remain the full lower-tier mission-completion evidence.

Focused hosts pass their selected plug-in catalog to composition compilation
and execution. Each family provider retains that catalog and reports the
package version together with the model's version and metadata fingerprint, so
UI, agents, and caches can distinguish a package update from a model revision.

`taoryx.source-table-fixed-wing.vehicle-catalog` is the typed package-data
root. A selected package may resolve the shared X8/B747 catalogs, witnesses,
and parity replays, but generic loaders cannot fall back to a sibling or the
compatibility aggregate; each focused provider still filters that shared data
to its one public family.
