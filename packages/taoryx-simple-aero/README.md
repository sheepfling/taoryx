# taoryx-simple-aero

Installable Simple Aero plug-in for Taoryx. It owns the analytical
`reference.point_mass` trajectory provider, the fixed-L/D model builder,
Mission Composition model, fidelity ladder, validation helpers, and semantic
fixture adapters.

The implementation occupies the shared `taoryx` namespace so established
imports remain valid without keeping Simple Aero code in the core wheel.

## Install from this checkout

Run from the repository root so the core dependency is supplied locally:

```bash
python -m pip install -e . -e packages/taoryx-simple-aero
taoryx plugins inspect taoryx.simple-aero --no-builtin
```

See the [installation guide](../../docs/INSTALLATION.md) for the complete model
suite, release wheels, and contributor setup.
