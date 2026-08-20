# API reference

These are the canonical, reusable interface documents. They are organized by
the boundary a consumer or provider adopts—not by a bundled vehicle family.

## Composition and vehicle contracts

- [Trajectory contracts](trajectory-contracts.md) — standalone discovery,
  configuration handoff, batch results, optional streaming, and mandatory
  standard ECEF state.
- [Vehicle Composition Advertisement](vehicle-composition-advertisement-api.md)
  — JSON-safe capability/reflection artifact for hosts, UIs, and providers.
- [Mission Composition Provider](mission-composition-provider-api.md) —
  TAORYX-native typed configuration, batch, and session API.
- [Vehicle interface contract](vehicle-interface-contract.md) — shared model
  parameters, action/status/observation semantics, and transfer rules.
- [Vehicle Composition Registry](vehicle-composition-registry.md) — caller
  discovery of supported vehicle/fidelity/operation tuples.
- [Control contracts](control-contracts.md) and
  [public value spaces](public-value-spaces.md) — typed values, control
  semantics, and authority-safe UI/client behavior.

## Runtime and extension contracts

- [Telemetry](telemetry.md) and [route tracking](route-tracking-contract.md)
  — structured runtime and route artifacts.
- [EOM timing and committed truth](eom-timing-contract.md) — accepted-state,
  force-evaluation, and sensor timing boundary.
- [Run manifest](simulation-runtime-run-manifest.md) — portable run identity,
  result disposition, and artifact manifest.
- [Sensor plug-in API](sensor-plugin-api.md) — typed sensor extension and
  observation ownership boundary.

## Contract rule

Use these pages for normative public behavior. Package-specific capabilities,
data, calibration, source-deck limits, and model witnesses belong in the
owning package's documentation, listed in the [plug-in directory](../plugins/README.md).
For the choice between this external surface and a TAORYX-internal plug-in,
start with the [developer interface-layer guide](../developer/interface-layers.md).
