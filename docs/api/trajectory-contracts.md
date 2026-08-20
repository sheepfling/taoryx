# Trajectory contracts

`taoryx-trajectory-contracts` is the versioned, dependency-light interface
product for trajectory composition. It contains portable Pydantic schemas,
structural provider protocols, and inexpensive conformance audits. It depends
only on Pydantic; it does not import the TAORYX language, plug-in discovery,
simulation engine, vehicle data, or any model package.

That lets a host such as TITAN depend on this package alone, and lets a new
trajectory provider implement the same public seam without adopting TAORYX as
its runtime.

If you are deciding whether to use this external seam or to add a model inside
the TAORYX host, begin with [Developer interface
layers](../developer/interface-layers.md). A TAORYX vehicle plug-in is authored
once through the internal path and then exposed here through the host adapter.

## Package boundary

```text
taoryx-trajectory-contracts
  discovery + opaque configuration handoff + batch result
  optional streaming-control lifecycle + mandatory ECEF state
  structural protocols + conformance helpers
                 ▲                         ▲
                 │                         │
        external batch-only provider   TAORYX adapter
                                           │
                                           ▼
                         taoryx language + composition runtime + plug-ins
```

The current `taoryx` distribution remains the language/toolchain and runtime
host. `taoryx.trajectory.contracts_adapter.TaoryxTrajectoryContractsAdapter`
is TAORYX's implementation of the standalone protocol. It wraps an existing
`ConfigurableTrajectoryProvider`; it does not duplicate a model's plant,
configuration language, or execution bindings.

## Portable minimum and optional extensions

Every batch `TrajectorySample` and every streaming `StreamingObservation`
contains a required `StandardEcefState`:

- ECEF/ECFC position, Earth-relative velocity, and acceleration;
- body-frame angular velocity in radians per second; and
- scalar-first `ecef_from_body_wxyz`, mapping body forward/right/down axes into
  the ECEF world frame.

Projection and orientation provenance are required too. A consumer therefore
does not infer a local-axis convention or silently relabel an inertial frame as
Earth-fixed. The standard state is shared by identity with TAORYX's native
Mission Composition result and session contracts.

`values` contains a provider's selected model-specific output channels.
`extensions` bags retain optional provider-specific information such as
segments, lineage, control lowering, presentation details, and source
provenance without weakening the common minimum.

## Capability model

The base protocol is capability based:

- `BatchCompositionProvider` supports discovery, preparation, and batch runs.
- `DefaultConfigurationProvider` is an optional onboarding extension that can
  return one deterministic, provider-selected runnable default configuration
  for an advertised model.
- `StreamingCompositionProvider` adds open, inspect, step, authority switch,
  reset, and close for persistent control sessions.

A foreign provider may truthfully be batch-only. It must not emulate an empty
session just to satisfy a TAORYX feature. TAORYX's stricter rule—every
host-facing TAORYX model advertises matching common batch and step tuples—is a
TAORYX provider profile, not a requirement of this interface package.

The fast local command keeps that distinction practical:

```bash
# Checks only this plug-in's public projection and TAORYX-universal matrix.
python tools/dev.py check-plugin-contract taoryx.hummingbird

# Runs only that plug-in's runnable defaults through one public streaming step.
python tools/validate_mission_composition_provider_contract.py \
  --plugin taoryx.hummingbird --contract-profile taoryx-universal --execute-defaults

# Adds the selected plug-in's full public batch routes. Use this focused probe
# when those routes are expected to finish within the local test budget.
python tools/validate_mission_composition_provider_contract.py \
  --plugin taoryx.hummingbird --contract-profile taoryx-universal --execute-batch-defaults

# Builds and imports the standalone package with no TAORYX runtime import.
python tools/dev.py check-trajectory-contracts
```

It does not construct unrelated providers or require a catalogue-wide test
run. Full vehicle execution remains a separate vertical/release gate.

## Consumer flow

An interface-only consumer installs just the contract package:

```bash
python -m pip install taoryx-trajectory-contracts
```

It can use only the stable public objects:

```python
from taoryx_trajectory_contracts import (
    BatchCompositionProvider,
    BatchRunRequest,
    DefaultConfigurationProvider,
    StreamingCompositionProvider,
    audit_batch_provider,
)

provider: BatchCompositionProvider = obtain_provider_from_your_host()
assert audit_batch_provider(provider).status == "pass"

if isinstance(provider, DefaultConfigurationProvider):
    # This is provider-selected and runnable, not a claimed physical nominal condition.
    configuration = provider.build_default_configuration("model-id")
else:
    configuration = authored_configuration

prepared = provider.prepare_configuration(configuration)
result = provider.run_batch(BatchRunRequest(request_id="run-42", prepared_configuration=prepared))
pose = result.entities[0].samples[-1].standard_ecef

if isinstance(provider, StreamingCompositionProvider):
    # Open/inspect/step/reset/close are available using the same prepared
    # configuration and the same required ECEF observation contract.
    ...
```

The configuration payload is deliberately opaque to the common package. A
provider may use its own language document, JSON schema, typed tree, remote
request, or compiled scenario. Hosts preserve it while the provider owns its
authoring semantics.

## TAORYX use

TAORYX authors its richer typed configuration first, then wraps it at the
independent boundary:

```python
from taoryx.trajectory.contracts_adapter import TaoryxTrajectoryContractsAdapter
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx_trajectory_contracts import BatchRunRequest

adapter = TaoryxTrajectoryContractsAdapter(RegistryMissionCompositionProvider())
public_configuration = adapter.build_default_configuration("model-id")
prepared = adapter.prepare_configuration(public_configuration)
result = adapter.run_batch(BatchRunRequest(request_id="run-42", prepared_configuration=prepared))
```

The adapter verifies the envelope identity, schema fingerprint, and prepared
payload before dispatch. A host cannot accidentally execute a configuration
for a different provider, model revision, realization, or mission template.
TAORYX advertises one provider-selected runnable default for every host-facing
model. The default is traceable to a package-owned configuration or witness and
is deliberately not encoded as a fake schema default for a launch state, route,
or operating point.

For TAORYX-specific authoring semantics and the legacy/native contract types,
see the [Mission Composition Provider API](mission-composition-provider-api.md).
