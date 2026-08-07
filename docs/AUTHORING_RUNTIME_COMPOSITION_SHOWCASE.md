# TAORYX Authoring → Runtime → Composition showcase guide

TAORYX is easiest to understand as three functional layers that meet at
explicit artifacts. The layers are related, but they are not interchangeable:

```text
Model Authoring: source and language successor
    .tbl/.prb -> parse -> validate -> lower -> provenance

Simulation Runtime: simulation and stepping platform
    resolved plant -> accepted truth -> controls/stepper -> telemetry/artifact

Mission Composition: vehicle and mission composition
    discover -> configure -> compile -> bind -> run -> evaluate -> showcase
```

The most useful demonstration is to walk through all three layers with one
vehicle. Authoring proves that the source is understood. Runtime proves that
the resolved model can execute and expose truth at the correct time
boundaries. Composition proves that a
caller can select a vehicle, fidelity, initialization, and mission without
writing a new route runner.

In one sentence: **Authoring defines the model. Runtime executes the model.
Composition lets users choose and assemble a mission.**

Compatibility identifiers remain in historical roadmap records, Python import
paths, and versioned schema identifiers where changing them would break an
existing consumer. New documentation and interfaces use Model Authoring,
Simulation Runtime, and Mission Composition.

This guide is a practical index to the checked-in examples and evidence. It
does not promote a nominal run to a qualification claim. The words
`declared`, `composable`, `adapter_bound`, `factory_bound`, `executable`, `evaluated`, and
`qualified` remain distinct; see the [Authoring → Runtime → Composition roadmap](plan/authoring-runtime-composition-execution-roadmap.md).

## The three layers in one view

| Layer | User question | Primary interface | Evidence to inspect |
| --- | --- | --- | --- |
| Model Authoring | “Does TAORYX understand this source and preserve its meaning?” | `.tbl`, `.prb`, grammar profiles, diagnostics | parser report, lowering report, source hashes, equation/provenance audits |
| Simulation Runtime | “Can this resolved plant step reproducibly with truthful timing and bounded controls?” | `run`, `LoadedProgram`, `InteractiveSession`, artifacts | accepted truth, controls, events, sensor timing, replay, plots |
| Mission Composition | “Can I select a family, fidelity, setup, and mission and get an exact runnable binding?” | `taoryx vehicle ...`, composition YAML/JSON, interface contract | compiled composition, preflight, interface, execution, status trace, objective report |

The composition layer is deliberately above the plant. It may select a
point-mass model, a named pseudo-6DOF response model, a rigid-body direct-
wrench bridge, or a rigid-body physical-effector realization. It must report
which one was selected and must fail closed when the requested realization is
not available.

## Model Authoring — source and language successor

Start with source validation. Use the historical profile for historical
fixtures and the successor profile for TAORYX extensions:

```bash
taoryx-validate --profile taos96 examples/chapter04/ballistic-reentry.prb
taoryx-validate --profile taoryx path/to/taoryx-extension.prb

python tools/dev.py grammar
python tools/dev.py test-grammar
python tools/dev.py equation-audit
```

Model Authoring is showcased when the result is more than “the parser
accepted the file.” Inspect located diagnostics, table/problem source hashes,
the resolved structured manifest, the native projection boundary, and the
distinction between grammar coverage and executable runtime coverage.

For a complete release check, rebuild the manual and run the repository
checks:

```bash
python tools/dev.py manual
python tools/dev.py check
python -m pytest
```

TAORYX does not claim runtime equivalence with TAOS 96.0 without the
historical executable and a trusted output corpus.

## Simulation Runtime — simulation, stepping, and artifacts

Simulation Runtime has two equivalent entry styles: a deterministic batch
run and an external stepping session. Both use the same resolved plant and
the same accepted-truth boundary.

### Batch source run

```bash
taoryx run examples/chapter04/ballistic-reentry.prb \
  --profile taoryx \
  --output-dir artifacts/examples/ballistic-reentry
```

For a controlled numerical comparison, choose the integrator explicitly and
retain the output directory as an artifact:

```bash
taoryx run path/to/mission.prb path/to/aero.tbl \
  --profile taoryx \
  --integrator rk4 \
  --max-steps 20000 \
  --output-dir artifacts/runs/example \
  --report artifacts/runs/example/run-report.json \
  --artifact artifacts/runs/example/run-artifact.json
taoryx artifact inspect artifacts/runs/example/run-artifact.json
```

The complete Simulation Runtime scenario map, pseudo-6DOF recipes,
California–Hawaii distinctions, and external stepping example are in the
[Simulation Runtime onboarding guide](SIMULATION_RUNTIME_ONBOARDING.md).

### Interactive and AI/RL stepping

The same runtime can be driven through `InteractiveSession`. A policy or
external controller supplies only declared actions at accepted external
step boundaries. It cannot mutate raw state or bypass the selected plant.
The runtime records pre-step and post-step truth, force/load evaluation state,
control activation time, segment and event transition pairs, and sensor
captures only at committed truth boundaries.

Read the [EOM timing contract](architecture/eom-timing-contract.md),
[interactive engine guide](architecture/interactive-engine.md), and
[sensor orchestration plan](plan/sensor-measurement-orchestration.md) before
adding a sensor, estimator, or RL observation.

### What a Simulation Runtime showcase must say

Every runtime artifact should identify the exact source, resolved model,
environment, and integrator; accepted truth timestamps and event boundaries;
requested and realized controls; resource, envelope, and numerical status;
and any direct-wrench, pseudo-6DOF, or physical-effector limitation.

A clean trajectory is not sufficient evidence of physical control. The
control path and the truth/measurement boundary are part of the result.

## Mission Composition — vehicle and mission composition

The composition CLI is the main public showcase. It exposes discovery,
semantic compilation, capability preflight, exact binding, and execution as
separate steps.

For the smaller provider plug-in path—metadata publication, typed parameters,
variable-length segments, and a standard trajectory—start with the [Mission
Composition front door](MISSION_COMPOSITION.md).

### 1. Discover the catalog

```bash
taoryx vehicle catalog --detail summary
taoryx vehicle catalog --detail full
taoryx vehicle interface-report
taoryx vehicle topology-report
taoryx vehicle describe skywalker_x8
taoryx vehicle endpoints skywalker_x8
taoryx vehicle schema skywalker_x8 initialization
taoryx vehicle schema skywalker_x8 segments
taoryx vehicle parameters skywalker_x8 --scope segment
taoryx vehicle segment skywalker_x8 fly_by_turn
taoryx vehicle authoring skywalker_x8
taoryx vehicle authoring-template skywalker_x8 powered_fixed_wing_racetrack_v1 point_mass_3dof
taoryx vehicle authoring-all
taoryx vehicle maturity-report
taoryx vehicle mission inspect skywalker_x8 powered_fixed_wing_racetrack_v1
taoryx vehicle mission create skywalker_x8 powered_fixed_wing_racetrack_v1 point_mass_3dof
```

The registry reports the physical family, available fidelity tiers,
initialization contracts, mission templates, semantic controls, resources,
status channels, and evidence boundary. A registry entry is not automatically
a runnable mission.

`vehicle authoring` is the practical addition checklist. It joins each
declared mission/fidelity with its interface validation, available batch or
episode bindings, graph-execution status, and remaining Mission Composition
next steps.
It does not claim that source intake, model physics, or qualification is
complete; use the vehicle-integration intake/readiness workflow for those
separate authorities.

`vehicle authoring-all` preserves the same per-family evidence and next-step
records in one catalog response. It deliberately does not collapse missing
interfaces, adapters, graph semantics, and qualification gates into one
cross-family score.

`vehicle maturity-report` is the corresponding release-facing audit. It
reports value-space findings, parameter/variant maturity, graph and capability
coverage, runnable batch/episode operations, and parity disposition as
separate counts. Use `--check-execution-witnesses` when the slower checked-in
composition-witness sweep should also compile every endpoint and open each
advertised episode. Neither mode converts any count into model qualification.

`vehicle authoring-template` is the handoff from discovery to composition
authoring. It emits required inputs with their units and value spaces, ordered
segment occurrences, graph/state-transfer shape, selected control interface,
and known runtime/evidence work. Its values are intentionally null placeholders;
authors must supply source-backed values rather than inherit hidden geometry or
controller settings. It also lists the exact selected batch/episode endpoints
and emits only commands supported by that tier; a batch-only or planned tier is
not handed a misleading interactive or run command.

Read the [public value-space contract](architecture/public-value-spaces.md)
before building a UI, optimizer, policy, or controller against these fields.
It distinguishes periodic values such as heading from their linear rates,
quaternions from componentwise vectors, bounded controls from generic scalar
overrides, and discrete modes/events from interpolable numeric values.

`vehicle mission inspect` is the read-only mission view; `vehicle mission
create` exports that same kit for one selected fidelity, and `vehicle mission
validate <request>` runs the immutable compiler, interface check, graph check,
preflight, and lowering report without executing a vehicle. A semantic
validation pass is deliberately not a run, controller, or qualification pass.

For a catalog-wide authoring check, run:

```bash
taoryx vehicle validate-examples
taoryx vehicle validate-examples --require-preflight
```

The first verifies every checked-in composition request and reports its
interface, lowering, and preflight disposition. The strict form fails for a
valid-but-unlowerable witness, keeping legacy or planned requests visible
without treating them as runnable endpoints.

The same query surface exposes independently selectable vehicle variants. A
variant is runnable only when its declaration names the exact native runtime
input it changes. For example, A320's `operating_mass_kg` is bounded by the
pinned clean OpenAP tables and triggers a new trim:

```bash
taoryx vehicle parameters a320_openap_3dof --scope variant_configuration
taoryx vehicle compose \
  examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml \
  --output generated/a320-racetrack-mass-variant.json
```

It is deliberately not a generic field override and does not claim that every
mass in the table domain has passed mission qualification.

The Hummingbird has a separate, grounded-start mass binding for its named
aggregate-thrust pseudo-6DOF witness:

```bash
taoryx vehicle parameters hummingbird --scope variant_configuration
taoryx vehicle variant validate examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml
taoryx vehicle variant resolve examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml --output generated/hummingbird-grounded-mass-variant.json
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml \
  --output generated/hummingbird-grounded-mass-variant.json
```

Its hard limit comes from the surrogate's declared collective-thrust capacity
and is checked again by hover-capability preflight. This only proves that the
variant reaches the bounded pseudo plant; it is not a payload, inertia, motor,
or endurance qualification.

### 2. Compose a semantic request

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml \
  --output generated/x8-racetrack-3dof.json

taoryx vehicle compose \
  examples/vehicle_composition/x8_racetrack_capability_compose.yaml \
  --output generated/x8-racetrack-pseudo6dof.json
```

The compiled JSON is the immutable semantic handoff. It records normalized
values, derived route geometry, the selected interface contract,
source/evidence references, and the requested fidelity. It does not yet
prove that the native runtime can execute the mission.

### 3. Preflight and resolve the exact interface

```bash
taoryx vehicle preflight generated/x8-racetrack-3dof.json
taoryx vehicle interface-composition generated/x8-racetrack-3dof.json
taoryx vehicle lower generated/x8-racetrack-3dof.json
taoryx vehicle episode-info generated/x8-racetrack-3dof.json --seed 7
taoryx vehicle result generated/x8-racetrack-run --composition generated/x8-racetrack-3dof.json
```

Preflight checks that semantic segments map to the declared translator.
Lowering binds the exact adapter and returns a structured blocker when the
adapter, source, fidelity, initialization, or mission capability is absent.
There is no generic fallback plant or controller.

### 4. Materialize and run

```bash
taoryx vehicle materialize generated/x8-racetrack-3dof.json \
  --output-dir generated/x8-racetrack-native-inputs

taoryx vehicle run generated/x8-racetrack-3dof.json \
  --output-dir artifacts/composition/x8-racetrack-3dof
```

The run output should include the exact execution binding, interface
contract, committed status trace, truth telemetry, events, the native
`objective_report.json`, and a normalized `evaluation.json`. The latter is a
Mission Composition projection with common validity, feasibility, outcome, gate, and
dimensionless objective-margin fields; it retains `unqualified` unless the
family evidence says otherwise. It never replaces the native objective report
or family-specific telemetry. For episode-capable compositions, inspect the
declared action/observation contract before opening the episode:

```bash
taoryx vehicle interface-composition \
  generated/x8-racetrack-sensor-episode.json
taoryx vehicle replay-policy \
  generated/x8-racetrack-sensor-episode.json \
  path/to/policy-trace.json
```

The endpoint matrix does not infer parity. Every descriptor separately reports
whether parity is `registered`, `not_registered`, or `not_available`; the
first separate witness is intentionally narrower. A
persisted Hummingbird aggregate-thrust pseudo-6DOF policy trace can be sent
through the independently owned batch loop and compared at every committed
truth/status boundary:

```bash
taoryx vehicle batch-episode-parity \
  generated/hummingbird-hover-yaw-sensor.json \
  generated/hummingbird-policy-trace.json \
  --output generated/hummingbird-batch-episode-parity.json
```

The command accepts only the exact registered mission, fidelity, authority
profile, interface fingerprint, batch factory, episode factory, and declared
integration step. The current witnesses are family-specific (Hummingbird,
X8/B747, A320, F-16, and the local X-15 direct-wrench bridge); it fails closed for every other combination rather
than quietly replaying a different model. A passing report
proves one semantic action trace has matching committed status projections in
the two paths; it does not prove a closed-loop mission, individual motors,
physical allocation, robustness, or family qualification.

`taoryx vehicle results <directory>` additionally emits a comparison-safe
`evaluation_summary` for each valid normalized result: required-objective and
gate dispositions, worst supplied normalized objective error, and evidence
channel availability. It is a typed-envelope projection only—never a
recalculation of trajectory physics, a synthetic terminal result, or a
substitute for family telemetry.

The [vehicle interface contract](architecture/vehicle-interface-contract.md)
defines the common parameter, action, status, resource, and observation
vocabulary. The [vehicle composition registry](architecture/vehicle-composition-registry.md)
defines how a family publishes its capabilities and execution bindings.

## Showcase recipes by layer boundary

The checked-in composition examples are intentionally small, named witnesses.
Use them to demonstrate the architecture without implying that every one is a
full physical qualification.

| Showcase | Runtime / Composition lesson | Example |
| --- | --- | --- |
| X8 racetrack | A common powered-fixed-wing mission can resolve at 3DOF and pseudo-6DOF. | `examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml`, `x8_racetrack_capability_compose.yaml` |
| B747 racetrack | The same semantic route scales to transport geometry and response limits. | `examples/vehicle_composition/b747_racetrack_capability_3dof_compose.yaml`, `b747_racetrack_capability_pseudo6dof_compose.yaml` |
| A320/F-16 | Existing-family members use the same seam with different source/reduced adapters. | `examples/vehicle_composition/a320_racetrack_capability_*.yaml`, `f16_racetrack_capability_*.yaml` |
| Hummingbird | Episode composition exposes hover, yaw, translation, sensor, and landing contracts. | `examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml` |
| X-15 | Direct-wrench and staged/reachability paths remain explicitly labeled evidence tiers. | `examples/vehicle_composition/x15_*_compose.yaml` |
| Tumbling body | Passive bodies use the same composition interface while declaring no control authority. | `examples/vehicle_composition/tumbling_body_direct_release_*.yaml` |
| NESC staged rocket | Stage/release semantics are composed without pretending that all tiers expose the same attitude physics. | `examples/vehicle_composition/nesc_staged_source_replay_*.yaml` |

For the full nominal showcase catalog, use the generated Alpha 2 packet and
its reproduction record under `artifacts/showcases/alpha2/final-catalog-v1`.
Its status is an honest evidence summary: a nominal result may be present
while family qualification, physical-effector evidence, robustness, or
terminal closure remains pending.

## Verification commands for the Authoring → Runtime → Composition handoff

```bash
python tools/validate_vehicle_interface_catalog.py
python tools/validate_vehicle_execution_witnesses.py --variants-only
python tools/validate_vehicle_execution_witnesses.py --execute-batch
PYTHONPATH=src python tools/validate_showcase_artifact_boundary.py
python tools/dev.py check
```

These checks answer different questions: interface consistency, exact
execution-witness resolution, showcase claim boundaries, and repository-wide
language/catalog/runtime health.

## How to present the layers to a coworker

Use this order in a demo:

1. Show a `.prb`/`.tbl` source and the language diagnostics/provenance.
2. Run the same resolved source through the batch runtime and open its
   telemetry/artifact report.
3. Show the vehicle registry entry and its fidelity choices.
4. Compose the X8 racetrack, preflight it, inspect its interface, and run it.
5. Open the objective and status artifacts, then state the exact claim and
   nonclaim: route execution and truth evaluation are not automatically
   physical-effector qualification.
6. Repeat the composition request for the B747 or Hummingbird to show that
   the mission interface is shared while the family adapter and capability
   envelope change.

The core message is simple: Authoring preserves meaning, Runtime executes
truth, and Composition makes supported vehicles and missions discoverable and
reproducible. The contracts between them are the system—not a collection of
unrelated showcase scripts.

For the staged plan that matures Mission Composition from the current registry and
template compiler into bounded vehicle configuration and general semantic
mission authoring, see the [Mission Composition maturity plan](plan/mission-composition-maturity.md).
