# Developer guides

This section is for people extending TAORYX itself. It deliberately separates
the external host/provider API from the internal package authoring experience.

- [Developer interface layers](interface-layers.md) — choose the external
  contracts package or the internal TAORYX plug-in boundary.
- [Authoring a vehicle plug-in](vehicle-plugin-authoring.md) — package shape,
  entry-point discovery, controls, factories, and focused evidence.
- [Model-to-mission authoring and automation](model-authoring-automation.md)
  — configuration scaffolding, lowering, and registered tuning flows.

For exact commands for one installed distribution, run:

```bash
python tools/dev.py plugin-focus <wheel-selector>
```

Then read that distribution's package-owned README and any notes under
`packages/<distribution>/docs/`, as catalogued in the
[plug-in directory](../plugins/README.md). Shared public contracts are under
the [API reference](../api/README.md); do not put a new public contract in a
model package or a package-specific design note in `docs/architecture/`.
