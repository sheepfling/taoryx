# taoryx-simple-aero

Installable Simple Aero plug-in for Taoryx. It owns the analytical
`reference.point_mass` trajectory provider, the fixed-L/D model builder,
`taoryx.simple-aero.mission-composition` focused provider, fidelity ladder,
validation helpers, semantic fixture adapters, and its workflow-endpoint
catalog/witness package data.

The focused provider advertises one `simple_aero` point-mass model. Batch
execution uses package-generated commands. A persistent session starts in the
provider-owned `generated_mission_commands` profile, whose bank schedule is
telemetry rather than caller steering; a caller may switch to
`direct_throttle_command`, which exposes only
`propulsion.command.fraction` and reports requested/achieved control feedback.
It makes no physical-vehicle, controller-stability, or flight-performance
claim.

The implementation occupies the shared `taoryx` namespace so established
imports remain valid without keeping Simple Aero code in the core wheel.

## Install from this checkout

Run from the repository root so the core dependency is supplied locally:

```bash
python -m pip install -e . -e packages/taoryx-simple-aero
taoryx plugins inspect taoryx.simple-aero --no-builtin
python tools/dev.py test-vehicle simple_aero
python tools/verify_plugin_wheels.py --plugin simple-aero --python .venv/bin/python
```

The focused command begins by verifying an exact package-data extract:

```bash
python tools/extract_simple_aero_plugin_assets.py --check
```

The package retains its analytical fixture/segment data but carries only the
`simple_aero` row from the shared Alpha 2 family catalog; Dual Launch and
other workflow families are not shipped as Simple Aero package data.

See the [installation guide](../../docs/INSTALLATION.md) for the complete model
suite, release wheels, and contributor setup.
