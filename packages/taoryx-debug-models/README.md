# Taoryx debug models plug-in

`taoryx-debug-models` contains development-only, low-fidelity Mission
Composition fixtures. It is deliberately separate from physical vehicle
packages and makes no source-vehicle, atmosphere, controller-performance, or
qualification claim.

The plug-in publishes two versioned providers:

- `taoryx.reference.mission-composition`
  - `reference_ballistic_3dof`: an explicit `open_loop_coast` zero-action
    session. It accepts `{}` only; no invented actuator or controller surface
    is exposed.
  - `reference_constant_velocity_waypoint_3dof`: default provider-owned
    waypoint guidance plus selectable `kinematic_velocity_command` and
    `live_waypoint_guidance` session modes. Both expose normalized action
    metadata, runtime availability, and requested/applied/achieved feedback.
- `taoryx.debug.mission-composition-contract-probe`
  - `contract_probe_vehicle`: a synthetic API stress fixture with selectable
    `debug_guidance_control`, `debug_discrete_control`, and
    `debug_event_control` modes. It exists to exercise typed controls,
    telemetry, multi-object lineage, event semantics, and feedback—not flight
    physics.

The package owns three checked-in batch endpoint witnesses and their package
data fallback:

- `reference-ballistic-3dof-batch`
- `reference-waypoint-3dof-batch`
- `debug-contract-probe-batch`

For the focused development loop, use:

```bash
python tools/dev.py test-debug-models
python tools/verify_plugin_wheels.py --plugin debug-models --python .venv/bin/python
```

The first command begins by verifying the exact package-data extract:

```bash
python tools/extract_debug_models_plugin_assets.py --check
```

It then runs only this package's advertisement, control/session, and endpoint
route. The wheel check builds an isolated distribution with core, discovers
only this plug-in, and executes all three package-owned batch witnesses.
