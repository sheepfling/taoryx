# TAORYX Trajectory Contracts

`taoryx-trajectory-contracts` is the dependency-light, versioned public
contract for trajectory composition hosts and providers. It defines provider
discovery, opaque provider-owned configuration handoffs, batch execution,
stateful control streaming, standard ECEF kinematics, and conformance checks.

It deliberately contains no TAORYX model data, language parser, plug-in
discovery implementation, or simulation runtime. A host such as TITAN can
depend on this package alone; TAORYX and other providers adapt their native
models to these schemas.

The base protocol is capability-based. A provider may implement batch only or
batch plus streaming. TAORYX's stronger batch-and-step promise is a provider
profile validated by the TAORYX integration package, not a requirement imposed
on external providers.

## Provider-selected runnable defaults

`DefaultConfigurationProvider` is an optional public capability. Its
`build_default_configuration(model_id)` call returns a deterministic,
provider-selected configuration that is ready for the ordinary prepare and run
path. A model advertises the stable identity through
`TrajectoryModelDescriptor.default_configuration_id`.

“Default” here means a batteries-included starting configuration selected by
the provider. It does not mean that every authoring field is optional, nor does
it claim a vehicle's canonical physical condition, calibration, or
qualification point. A foreign provider may omit this capability entirely and
remain an honest batch-only or streaming provider. TAORYX's universal profile
requires it for every host-facing model.

The normative repository-facing reference is
[Trajectory contracts](../../docs/api/trajectory-contracts.md). For the choice
between this external contract and adding a model to TAORYX, see
[Developer interface layers](../../docs/developer/interface-layers.md).

## Choose your role

- **Host or consumer:** depend on this package, obtain a provider through your
  own discovery/transport mechanism, inspect its descriptor, then preserve its
  opaque configuration payload through preparation and execution.
- **Independent trajectory provider:** implement `BatchCompositionProvider`;
  add `StreamingCompositionProvider` only when you truly own a persistent
  control lifecycle. Add `DefaultConfigurationProvider` when you choose to
  offer a provider-selected runnable starting configuration. Publish a
  `StandardEcefState` on every returned sample.
- **TAORYX vehicle plug-in author:** use TAORYX's internal `taoryx.plugins`
  integration path. The TAORYX host adapts its common model result to this
  contract, so the vehicle does not maintain a second solver API.

## Minimum external-provider checklist

1. Return a truthful `TrajectoryProviderDescriptor` with the exact available
   operation tuples for each model.
2. Validate a provider-owned `CompositionConfiguration` into a fingerprinted
   `PreparedCompositionConfiguration`; hosts must be able to round-trip the
   opaque payload without knowing its language.
3. Return a `BatchRunResult` whose every `TrajectorySample` contains
   `StandardEcefState`: ECEF/ECFC position, Earth-relative velocity and
   acceleration, body angular velocity, and scalar-first ECEF-from-body
   quaternion.
4. Put provider-specific telemetry in `values` or extensions rather than
   replacing the common state.
5. Call `audit_batch_provider(provider)` in the provider's focused test. Test
   streaming separately only if it is advertised.

```python
from taoryx_trajectory_contracts import audit_batch_provider

assert audit_batch_provider(provider).status == "pass"
```

The rich composition, control, and reflection metadata is carried by the
versioned Vehicle Composition Advertisement artifact. It complements this
execution contract; it does not make batch-only providers pretend to offer
streaming control.

```python
from taoryx_trajectory_contracts import (
    BatchCompositionProvider,
    BatchRunRequest,
    StreamingCompositionProvider,
    StandardEcefState,
)
```

The package uses Pydantic v2 models and emits JSON Schema through each model's
`model_json_schema()` method. Schema identifiers are stable within major
contract version `v1`.
