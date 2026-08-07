# Mission Composition provider example

This directory has three complementary Mission Composition walkthroughs.

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
ballistic, phugoid, skip, slalom, and weave templates. Its model metadata
distinguishes the runnable fixed-L/D baseline from maneuver templates that are
currently fixture-ready but do not yet have a generic configuration-to-runtime
adapter.

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
