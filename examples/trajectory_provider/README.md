# Mission Composition provider example

This directory has six complementary Mission Composition walkthroughs.

`composition_advertisement.py` treats capability negotiation as a standalone
product. It discovers any installed provider/model pair, validates the
embedded `taoryx.trajectory-composition-advertisement/v1` artifact through a
JSON round trip, prints its usable feature matrix, or exports the structural
JSON Schema for a non-Python gateway:

```bash
python examples/trajectory_provider/composition_advertisement.py
python examples/trajectory_provider/composition_advertisement.py --include-unavailable
python examples/trajectory_provider/composition_advertisement.py --json
python examples/trajectory_provider/composition_advertisement.py --json-schema
```

Use `--provider` and `--model` to inspect another installed trajectory
backend. The JSON Schema covers the portable field shape; the semantic
partition and cross-record conformance rules are documented in the
[Vehicle Composition Advertisement API](../../docs/architecture/vehicle-composition-advertisement-api.md).

`mission_composition_catalog.py` exercises the self-describing configuration
contract for all nine canonical vehicle families and the Simple Aero workflow:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --model hummingbird
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --model simple_aero
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --model x15 --json
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --audit
```

It demonstrates provider/model discovery, the typed `Parameter` / `Group` /
`Choice` / `Sequence` / `Optional` grammar, the independent core-state and
telemetry `OutputSchema`, exact mission operation matrices, and
evidence-bounded fidelity transitions. It validates configurations but does
not dispatch an execution binding.

The `simple_aero` schema demonstrates launch and endpoint choices, initial
mass/boost/aero parameters, open per-occurrence segment composition, and named
ballistic, phugoid, skip, slalom, and weave templates. Every reviewed template
and the separately advertised `Custom Composition (Open Sequence)` binding has
a real generic configuration-to-fixed-L/D batch path. This remains a synthetic
workflow demonstration, not a vehicle-performance or qualification claim.

`mission_composition_simple_aero.py` is the ergonomic counterpart: it creates
the full typed mission form, uses every source segment constructor once,
validates the resulting portable configuration tree, and runs it through the
same registry batch runner:

```bash
PYTHONPATH=src:packages/taoryx-daveml/src:packages/taoryx-simple-aero/src:packages/taoryx-reference-models/src:packages/taoryx-reachability/src \
  python3 examples/trajectory_provider/mission_composition_simple_aero.py \
  --output /tmp/mission-composition-simple-aero.json
```

`mission_composition_reference.py` is the runnable analytical provider walkthrough:

1. publish provider metadata and two native 3DOF vehicle models;
2. inspect initialization, capability, configuration, core-state, telemetry,
   and segment contracts;
3. validate and fingerprint a typed configuration tree with canonical units and a variable-length segment sequence;
4. dispatch the selected model through `MissionCompositionRunnerRegistry`; and
5. emit the discriminated common trajectory-or-failure response.

Run it from the repository root after installing the package, or with
`PYTHONPATH=src` in a source checkout:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_reference.py \
  --output /tmp/mission-composition-trajectory.json
```

The two advertised models are intentionally analytical fixtures. They show
the shape of a source-owned provider without claiming historical TAOS
compatibility, vehicle fidelity, or qualification evidence.

`mission_composition_contract_probe.py` is the full-surface development proof,
including typed telemetry and parent/child/grandchild lineage.
It advertises every configuration node/value type, structured model properties,
reference frames, core and grouped telemetry channels, output interpolation
modes, fidelity and operation states, and deployment state. It then requests
all telemetry and returns a valid multi-object trajectory with a first-class
spawn relationship and initial-state snapshot, plus a deliberately triggered
structured failure:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py \
  --output /tmp/mission-composition-contract-probe.json
```

`mission_composition_consumer_models.py` is the end-to-end smoke path for the
three leading deterministic consumer fixtures: basic ballistic, two-leg
waypoint, and the full-contract debug probe. It discovers the installed
providers, uses the reference provider's friendly SI launch and waypoint
builders for the two analytical configurations, revalidates each prepared
configuration through the common consumer seam, and emits all three standard
responses. The builder API is the recommended normal path for these two small
fixtures; the generic configuration grammar remains available when a consumer
needs to exercise the underlying tree directly:

```bash
PYTHONPATH=src:packages/taoryx-daveml/src:packages/taoryx-simple-aero/src:packages/taoryx-reference-models/src:packages/taoryx-reachability/src \
  python3 examples/trajectory_provider/mission_composition_consumer_models.py \
  --output /tmp/mission-composition-consumer-models.json
```
