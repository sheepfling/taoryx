# taoryx-reference-models

This package is the backward-compatible aggregate Mission Composition provider
for hosts that intentionally install the full Taoryx model profile. It owns no
vehicle plant, source table, controller screen, workflow endpoint witness,
execution factory, or family-specific capability adapter.

The generic completion, maturity, parity-witness, and integration-report
utilities live in the core host. Installing this aggregate is therefore never
required to run a focused family gate or its shared validation helpers.
Its wheel build also clears its private staging tree before copying declared
sources, preventing an earlier aggregate-era namespace module from being
carried into a later compatibility-only wheel.

Vehicle families are owned by their focused packages. In particular,
`taoryx-source-table-fixed-wing` owns the Skywalker X8 and B747 source-table
plants, route assets, focused providers, local controller screens, campaigns,
and route episode binding. This aggregate consumes that package together with
DAVE-ML, A320, F-16, Hummingbird, NESC, Dual Launch, Simple Aero, X-15, and
HL-20 fragments to retain `taoryx.registry.mission-composition` for catalogue
consumers. The provider resolves only through the exact plug-in catalog that
selected it; it does not use an unscoped fallback to discover additional
families.

Install this compatibility route explicitly from the repository root:

```bash
python scripts/bootstrap.py --profile compatibility
source .venv/bin/activate
taoryx plugins check --profile compatibility
taoryx model list --provider taoryx.registry.mission-composition
```

That profile contains only this aggregate's declared plug-in dependency
closure. It intentionally leaves the independent debug providers, passive-body
runtime, and reachability overlay out; use `full` only when a migration or
release must exercise those routes together.

The normal developer bootstrap deliberately excludes this package:

```bash
python -m tools.dev bootstrap
taoryx plugins check --profile developer
```

For new UI, agent, or composition work, prefer the focused provider owned by
the selected family—for example, `taoryx.x8.mission-composition` or
`taoryx.b747.mission-composition`. The aggregate is a compatibility view, not
a hidden dependency of those packages.

See the [installation guide](../../docs/INSTALLATION.md) for source and wheel
installation details, and [vehicle plug-in isolation](../../docs/architecture/vehicle-plugin-isolation.md)
for ownership and focused verification rules.
