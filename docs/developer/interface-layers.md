# Developer interface layers

TAORYX has two related but deliberately different developer boundaries. Pick
the boundary based on **where the model must run and be discovered**, not on
its dynamics fidelity or whether it supports live control.

```text
external host or foreign trajectory backend
    │
    │  provider-neutral discovery, batch results, optional streaming,
    │  standard ECEF kinematics and orientation
    ▼
taoryx-trajectory-contracts
    ▲
    │ adapter
    │
TAORYX language/runtime host
    ▲
    │ Python entry point, typed registry contribution, package-owned assets
    │
TAORYX vehicle/model plug-in
```

The layers meet at the same standard trajectory state, but they are not two
ways to implement the same solver. An internal TAORYX plug-in supplies its
native facts and execution bindings once; the TAORYX host adapts the common
result to the external contract. A foreign provider can adopt the external
contract without installing TAORYX or registering a Python plug-in.

## Choose the right layer

| You are trying to… | Start with | Do not assume |
| --- | --- | --- |
| Build a globe, planner, UI, agent, or gateway that consumes trajectory providers | [Trajectory contracts](../api/trajectory-contracts.md) | A host needs the TAORYX language/runtime or any vehicle package |
| Offer a native solver or remote service to generic hosts | [Trajectory contracts](../api/trajectory-contracts.md) and the [Vehicle Composition Advertisement API](../api/vehicle-composition-advertisement-api.md) | Every provider must support persistent stepping; batch-only is valid |
| Render a rich model picker, composition editor, or control form | [Mission Composition](../MISSION_COMPOSITION.md) and the [Vehicle Composition Advertisement API](../api/vehicle-composition-advertisement-api.md) | A UI may inspect private model classes or infer controls from labels |
| Add a model, control surface, source asset, or execution factory to TAORYX | [Authoring a vehicle plug-in](vehicle-plugin-authoring.md) | The compatibility aggregate is a dependency or that every repository test is needed for an inner loop |
| Do both: put a model in TAORYX and expose it to external hosts | The internal plug-in path first, then the TAORYX adapter | You need to maintain a second trajectory solver API |

## 1. External provider and consumer contracts

`taoryx-trajectory-contracts` is the installable, dependency-light public
package. It contains Pydantic schemas, structural protocols, and conformance
helpers; it does not import the TAORYX parser, engine, plug-in discovery, or
vehicle data.

```bash
python -m pip install taoryx-trajectory-contracts
```

Use it when the integration boundary is a host/provider relationship rather
than an extension of the TAORYX runtime.

| Need | Public contract | Minimum promise |
| --- | --- | --- |
| Discover provider and model identities | `TrajectoryProviderDescriptor` | Exact provider/model revision and available operation tuples |
| Hand a provider its own authored configuration | `CompositionConfiguration` → `PreparedCompositionConfiguration` | The host preserves the opaque provider payload; the provider validates it |
| Start a known-runnable onboarding case | optional `DefaultConfigurationProvider` | The returned default is provider-selected and prepared normally; it is not a physical/default-condition claim |
| Run a trajectory | `BatchCompositionProvider` and `BatchRunResult` | Every returned sample has `StandardEcefState` |
| Keep a model alive for control | optional `StreamingCompositionProvider` | Every observation has the same `StandardEcefState` |
| Describe mission, control, graph, and topology capabilities | `TrajectoryCompositionAdvertisement` JSON artifact | Closed, truthful feature rows and explicit claim boundaries |

`StandardEcefState` is the common globe/kinematics minimum: ECEF/ECFC position,
Earth-relative velocity and acceleration, body-frame angular velocity, and a
scalar-first `ecef_from_body_wxyz` quaternion. A provider can add its native
channels in `values` or extensions, but must not replace or ambiguously relabel
that common state.

The standalone execution package intentionally stays capability based:

- `BatchCompositionProvider` is the base contract.
- `DefaultConfigurationProvider` is optional for the broad external contract;
  it gives a host a deterministic, preparable runnable default when a provider
  chooses to publish one.
- `StreamingCompositionProvider` is optional; a foreign provider may honestly
  be batch-only.
- The richer capability, control, configuration, and reflection vocabulary is
  published through the Vehicle Composition Advertisement and Mission
  Composition artifacts. Consumers use declared authority, value-space, frame,
  and claim-boundary metadata rather than guess from a field name.

The Python reference implementation of the rich TAORYX-native metadata lives
in `taoryx` today. The advertisement itself is a versioned JSON-safe artifact,
so a service or non-Python backend can publish it without adopting TAORYX's
runtime. See the [Advertisement API](../api/vehicle-composition-advertisement-api.md)
for its semantic rules.

### External development loop

```python
from taoryx_trajectory_contracts import audit_batch_provider

report = audit_batch_provider(provider)
assert report.status == "pass"
```

Then test the exact provider-owned configuration and batch result shape,
including at least one `StandardEcefState`. Test streaming separately only when
the provider advertises it. In this repository, the package boundary is kept
honest by:

```bash
# Builds the standalone wheel and imports it with no TAORYX runtime fallback.
python tools/dev.py check-trajectory-contracts
```

That check proves the public package boundary, not an unrelated TAORYX vehicle
plug-in.

## 2. Internal TAORYX vehicle/model plug-ins

Use the internal plug-in path when the new model must be discoverable and
executable through TAORYX. A plug-in owns its source data, evidence,
configuration/fidelity facts, control metadata, model-specific lowering and
factories, focused witnesses, and Python entry point. Core owns generic
registries, session lifecycle, common result envelopes, and the adapter to the
external contracts.

Start from the direct developer profile, which does not install the historical
compatibility aggregate:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
taoryx plugins check --profile developer
```

Then make the implementation visible through the standard `taoryx.plugins`
entry-point group and a focused Mission Composition provider. Use the
[vehicle plug-in authoring guide](vehicle-plugin-authoring.md) for the package
shape and the [plug-in architecture](../architecture/plugins.md) for discovery rules.

### Internal development loop

The commands are intentionally ordered from cheap and package-scoped to
broader execution evidence:

```bash
# List registered package identities without materializing a model plant.
taoryx plugins list --json
taoryx plugins inspect taoryx.hummingbird --json

# Fast inner loop: one direct package, its providers, universal batch/step
# registration, a preparable provider-selected runnable default, and the mandatory standard
# ECEF result/session state.
python tools/dev.py check-plugin-contract hummingbird

# Consumer-level smoke: only the selected plug-in's defaults and one common
# streaming step.
python tools/validate_mission_composition_provider_contract.py \
  --plugin taoryx.hummingbird --contract-profile taoryx-universal --execute-defaults

# Add its full public batch routes only when they are expected to be bounded.
python tools/validate_mission_composition_provider_contract.py \
  --plugin taoryx.hummingbird --contract-profile taoryx-universal --execute-batch-defaults

# Package boundary before handoff: the fast check plus an isolated wheel install.
python tools/dev.py check-plugin hummingbird

# One model's executable vertical evidence. The stronger form adds selected
# interface, witnesses, and parity checks for physical vehicle families.
python tools/dev.py test-vehicle hummingbird
python tools/dev.py check-vehicle hummingbird
```

For any installed wheel selector, ask the repository to print only the exact
commands relevant to it:

```bash
python tools/dev.py plugin-focus hummingbird
```

It resolves the real plug-in ID from the wheel-boundary registry and lists only
that package's contract/install checks and its owned vehicle slices. Adding a
new plug-in therefore requires adding its focused evidence mapping; it does not
make a catalogue-wide execution sweep the default contributor workflow.

## Relationship to TAORYX's universal model profile

All host-facing TAORYX models use the stronger `taoryx_universal` profile:
every advertised common batch tuple has a matching step tuple. A model with a
native live episode keeps it; a batch-native model receives the core read-only
replay session. In either case every trajectory sample and step observation
carries the common `StandardEcefState`.

This is a TAORYX-host guarantee, not an external interoperability requirement.
The external contract remains honest about batch-only providers, while TAORYX
remains easy for a host to step uniformly. See the [Mission Composition
Provider API](../api/mission-composition-provider-api.md) for the exact profile and
adapter semantics.

## Fast reference

| Question | Answer |
| --- | --- |
| “Can an outside provider be batch-only?” | Yes, under `taoryx-trajectory-contracts`. |
| “Must a TAORYX-hosted model be batchable and steppable?” | Yes, through the TAORYX universal profile; a replay session is valid for a batch-native model. |
| “How does a host start a model with required mission inputs?” | Use TAORYX's advertised `DefaultConfigurationProvider` path, then prepare it normally; do not treat schema placeholders as defaults. |
| “Where do ECEF position, velocity, orientation, and body rate belong?” | `StandardEcefState`, on every batch sample and streaming observation. |
| “Where does a generic UI learn controls and composition options?” | From declared schemas, control/authority metadata, value-space metadata, and the composition advertisement. |
| “Where do I add a new TAORYX vehicle?” | A direct package with a `taoryx.plugins` entry point, not the compatibility aggregate. |
| “What is the first check after changing one TAORYX package?” | `python tools/dev.py check-plugin-contract <wheel-selector>`. |
| “How do I find the narrow commands for a package?” | `python tools/dev.py plugin-focus <wheel-selector>`. |
