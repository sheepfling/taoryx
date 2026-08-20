# Agent workflows

This is the shortest route from a new task to a traceable TAORYX change. Read
this page first, then follow the detailed architecture page for the workflow.

Before running a workflow in a fresh checkout, install and verify the complete
contributor package profile:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
python -m tools.dev install-check
```

The test configuration can import plug-ins directly from sibling source trees,
so a passing test alone does not prove that package metadata or entry points
are installed. See [Installing Taoryx and its model packages](INSTALLATION.md)
for smaller profiles, optional dependencies, wheel installs, and diagnostics.

Before adding a new source-grounded plant, DAVE-ML model, OpenAP model,
NASA/NESC scenario, JSBSim aircraft, orbital source, or public-data surrogate,
follow the [model integration workflow](plan/model-integration-workflow.md).
Start with the integration record and source hashes; keep the immutable plant
separate from Taoryx actuator, controller, mission, and RL overlays. This is
the contributor-facing Tier 0–4 process for new model work.
Use [Model-to-mission authoring and automation](developer/model-authoring-automation.md)
to turn that advertisement into plain-value initialization, segments,
waypoints, modes, and a registered no-manual-gain-search campaign.

For sensors, estimators, seekers, or RL observations, also follow the
[sensor plug-in architecture](api/sensor-plugin-api.md), the
[basic executable sensor examples](../examples/sensors/README.md), and the
[sensor and measurement orchestration backlog](plan/sensor-measurement-orchestration.md).
It defines the committed-truth boundary, measurement timing, multi-rate event
ordering, and truth-isolated decision ports.

## First orientation

For a layer-level tour before choosing a contributor workflow, read the
[Authoring → Runtime → Composition showcase guide](AUTHORING_RUNTIME_COMPOSITION_SHOWCASE.md).
It connects Model Authoring, Simulation Runtime, and Mission Composition
through the commands and artifacts used in this repository.

For the consumer-facing entry point to provider discovery, typed setup, and
variable-length mission sequences, read the [Mission Composition front door](MISSION_COMPOSITION.md).

Before choosing an external provider/consumer integration or contributing a
vehicle to the TAORYX host, read [Developer interface
layers](developer/interface-layers.md). It separates the
provider-neutral contract from the TAORYX-specific plug-in path and names the
narrow validation loop for each.

For the consumer or junior-engineer path through Simulation Runtime, read the
[Simulation Runtime onboarding guide](SIMULATION_RUNTIME_ONBOARDING.md). It is the scenario
index for source `.prb`/`.tbl` cases, pseudo-6DOF compositions, the
California–Hawaii variants, external time stepping, and the diagnostic ladder.

```text
.tbl/.prb source -> parse/validate -> lower -> runtime model -> artifact/plots
                                      ^                  ^
                              composition patches   controls/controllers
```

- Model Authoring preserves source text and reports diagnostics. Parsing
  successfully does not imply executable runtime coverage.
- Mission Composition resolves typed overrides and scenario metadata. It is
  a TAORYX extension, not historical TAOS syntax.
- Simulation Runtime integrates the lowered model in batch or through an external
  timestep loop.
- Artifacts are the common output boundary for telemetry, events, replay,
  reports, and plots.

## Choose the workflow

| Task | Start here | Primary command/API |
| --- | --- | --- |
| Choose an external contract or internal plug-in path | [Developer interface layers](developer/interface-layers.md) | `python tools/dev.py interface-guide` |
| Integrate a host or foreign trajectory provider | [Trajectory contracts](api/trajectory-contracts.md) | `BatchCompositionProvider`, optional `StreamingCompositionProvider`, `StandardEcefState` |
| Validate grammar | [Grammar guide](grammar/README.md) | `taoryx-validate file.prb file.tbl` |
| Add reusable segments | [Segmentation](architecture/declarative-segmentation.md) | `python tools/dev.py segment-lint` |
| Apply typed scenario changes | [Scenario runtime](architecture/README.md) | `ScenarioCompiler`, `ScenarioRequest` |
| Run a trajectory | [Runtime architecture](architecture/README.md) | `run_files(...)` or `LoadedProgram` |
| Drive timesteps | [Interactive engine](architecture/interactive-engine.md) | `InteractiveSession.step(...)` |
| Find and run Simulation Runtime scenarios | [Simulation Runtime onboarding](SIMULATION_RUNTIME_ONBOARDING.md) | `taoryx run ...` / `taoryx vehicle compose → preflight → lower → run` |
| Build plots | [Telemetry](api/telemetry.md) | `RunArtifact`, `render_run_artifact_plots(...)` |
| Add control above trim | [Controller stack](architecture/controller-stack.md), [Control contracts](api/control-contracts.md), and [LQR](extensions/lqr.md) | `TrimSpec`, `solve_trim`, controller/allocator |
| Introduce data for a new model | [Model-to-mission automation](developer/model-authoring-automation.md) and [Model integration workflow](plan/model-integration-workflow.md) | Mission Composition advertisement → `taoryx model plan` |
| Assess all advertised model/control readiness | [Model-to-mission automation](developer/model-authoring-automation.md) | `taoryx model assess --output build/model-assessment.json` |
| Generate initialization, modes, and segments | [Model-to-mission automation](developer/model-authoring-automation.md) | `taoryx model scaffold` → edit plain YAML → `taoryx model compile` |
| Run automatic controller candidate tuning | [Generic controller tuning](architecture/generic-controller-tuning.md) | plug-in campaign registration → `taoryx model tune` |
| Publish caller, provider, or common-tuned control for a new vehicle | [Vehicle plug-in authoring](developer/vehicle-plugin-authoring.md) | authority profile → native lowering/readback; optional campaign registration |
| Verify one mature vehicle endpoint vertically | [Model-to-mission automation](developer/model-authoring-automation.md) | `taoryx vehicle endpoint-specs` → `taoryx vehicle verify <endpoint-id>` |
| Validate an LQI candidate through native model controls | [Model-to-mission automation](developer/model-authoring-automation.md) | `LocalNativeCoordinateLqiScreenConfig` → exact batch Composition screen for response-law/guidance coordinates; use physical wrench validation when allocation is declared |
| Add an airbreathing vehicle mission | [Mission-composition automation](plan/mission-composition-automation.md) | `compile_powered_fixed_wing_racetrack(...)` |
| Add a vehicle or topology | [Vehicle plug-in authoring](developer/vehicle-plugin-authoring.md) and [generic family integration playbook](plan/generic-family-integration-playbook.md) | `taoryx vehicle intake existing-family ...` or `taoryx vehicle intake new-topology ...` |
| Define or assess one vehicle fidelity tier | [Fidelity tiers and vehicle plug-in requirements](architecture/fidelity-data-requirements.md) | `python3 tools/validate_fidelity_readiness.py --vehicle <id> --tier all` |
| Iterate on one runnable vehicle | [Building and testing](BUILDING_TESTS.md) | `python tools/dev.py test-vehicle <family>` |
| Verify one physical vehicle plug-in | [Building and testing](BUILDING_TESTS.md) | `python tools/dev.py check-vehicle <family>` |
| Check one TAORYX plug-in's universal public trajectory contract | [Standalone trajectory contracts](api/trajectory-contracts.md) | `python tools/dev.py check-plugin-contract <plugin>` |
| Discover the focused test/check commands for one plug-in | [Developer interface layers](developer/interface-layers.md) | `python tools/dev.py plugin-focus <plugin>` |
| Validate catalogue declarations | [Building and testing](BUILDING_TESTS.md) | `python tools/dev.py vehicle-catalogue` |
| Expose a parameter, control, status, or objective value | [Public value-space contract](api/public-value-spaces.md) | `taoryx vehicle topology-report` |
| Demonstrate the three layers | [Authoring → Runtime → Composition showcase guide](AUTHORING_RUNTIME_COMPOSITION_SHOWCASE.md) | `taoryx vehicle maturity-report` → `catalog` → `mission inspect`/`mission create`/`mission validate` → `preflight` → `run` |
| Publish schema-driven Mission Composition | [Mission Composition Provider API](api/mission-composition-provider-api.md) | `list_models()` → `get_model_schema()` → `validate_configuration()` |
| Execute a Mission Composition batch | [Mission Composition Provider API](api/mission-composition-provider-api.md) | `MissionCompositionRunRequest` → `MissionCompositionRunnerRegistry.run()` → trajectory or failure response |
| Drive a Mission Composition episode | [Mission Composition Provider API](api/mission-composition-provider-api.md) | `MissionCompositionSessionManager.open()` → `inspect()` / `step()` / `reset()` → `close()` |

### Mission Composition verification ladder

Use the narrowest audit that proves the change you made. These are separate
evidence levels, not interchangeable green checks:

```bash
# While changing the cross-family control taxonomy or the first streaming
# implementations, run only the API stressor, ballistic, waypoint, Simple
# Aero, and generic session tests. This does not execute direct-wrench or
# physical-effector qualification paths.
python tools/dev.py test-control-api-pilot

# Start a model/plugin change with one explicit vertical endpoint instead of
# the repository-wide maturity sweep. The default check compiles the endpoint
# witness, validates its generic capability advertisement and interface,
# confirms its exact factory and controller-adapter operations, and checks its
# selected public core-state and telemetry IDs.
taoryx vehicle endpoint-specs
taoryx vehicle verify hummingbird-source-hover-rotor-lqi

# Opt into only the work relevant to the change. --execute drives that one
# exact factory; --tune runs its registered campaign. Campaign results use a
# content-addressed cache unless --no-cache is chosen. --results-dir retains
# the exact packet, verifies finite samples for each required advertised output,
# and writes a controller/tuning provenance sidecar.
taoryx vehicle verify hummingbird-source-hover-rotor-lqi --execute
taoryx vehicle verify hummingbird-source-hover-rotor-lqi \
  --execute --tune --cache-dir build/controller-tuning-cache \
  --results-dir build/vehicle-endpoints/hummingbird-hover-lqi

# Read readiness in order: contract_valid → batch_executed → tuner_bound →
# tracking_passed → disturbance_or_mass_screened. A candidate-ready tuner is
# not bound until the runtime names the exact campaign node/profile/configuration
# fingerprint it actually applied. `records.performance` gives local phase
# timings and the campaign cache disposition; use a stable --cache-dir when
# iterating. `records.controller.tuning_application_contexts` exposes resolved
# gains for an exactly coordinate-compatible runtime, but is not proof of
# application until that runtime emits the returned tuning_binding receipt.
# Each endpoint also declares mass/offset/wind screen cases and thresholds;
# `not_executed` is intentional incomplete evidence, never a robustness pass.

# The corresponding cross-contract regression set is intentionally slow and
# opt-in; it exercises the four current physical-controller endpoint slices
# without invoking every vehicle's witnesses.
python -m pytest tests/unit/test_vehicle_endpoint_spec.py -m slow

# Fast executable proof for one vehicle: advertisement, Composition batch,
# interactive episode where that fidelity supports one, and any registered
# controller campaign. Current focused slices: f16_s119, a320_openap_3dof,
# hummingbird, x15, hl20_mod_k, skywalker_x8, b747,
# reference_nesc_two_stage_rocket, tumbling_body, all Simple Aero fixed-L/D
# batch/session templates, and both source-generated point-mass dual-launch forms.
python tools/dev.py test-vehicle skywalker_x8

# Stronger family-scoped host-contract gate. This validates only the selected
# vehicle's interfaces, endpoint witnesses, parity replay, and vertical tests.
python tools/dev.py check-vehicle hummingbird

# Fast package-scoped TAORYX-universal gate. This constructs only the selected
# plug-in's providers and checks standard ECEF state types plus every exact
# registered batch-to-step seam; it does not build a wheel or execute a mission.
python tools/dev.py check-plugin-contract hummingbird

# Simple Aero is a non-physical fixed-L/D workflow with a common batch runner
# and a persistent point-mass session. Its default provider-owned generated
# schedule has an empty caller action schema; the selectable caller-owned
# direct-throttle profile lowers exactly to command.throttle. Bank remains
# generated schedule telemetry and is not advertised as interactive steering.
python tools/dev.py test-vehicle simple_aero

# Dual launch executes both air-release and attached-booster source forms,
# reports the primary trajectory, and preserves the event-only/no-child-
# propagation boundary. Its common session is a read-only core replay: it has
# no caller action schema and does not create an independently propagated child.
python tools/dev.py test-vehicle dual_launch_glider

# These two non-physical models have typed workflow endpoint witnesses rather
# than physical Vehicle Composition endpoints. The verifier checks the exact
# checked-in draft, installed provider advertisement, common batch
# registration, and (with --execute) normalized result surface. Simple Aero's
# native persistent session is tested by its vehicle slice; dual launch uses
# the same common session lifecycle through the read-only core replay adapter.
taoryx model endpoint-specs
taoryx model verify simple-aero-fixed-ld-batch --execute
taoryx model verify dual-launch-attached-booster-batch --execute

# Run the exact checked-in compose → preflight → public batch-run → normalized
# result-artifact witnesses for one family. This installed command is the
# focused endpoint smoke for agents; it does not substitute a local screen or
# another vehicle.
taoryx vehicle witness-report \
  --family hl20_mod_k --execute-batch

# Before executing a new or blocked composition, use one read-only report to
# inspect its exact interface plus preflight capability advertisement and
# lowering disposition. This makes source-domain and runtime-admission gaps
# visible without treating semantic compilation as a runnable model.
taoryx vehicle mission validate \
  examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml

# If a family has several batch endpoints, keep the smoke even narrower with
# its stable witness ID from verification/vehicle_execution_witnesses.yaml.
taoryx vehicle witness-report \
  --witness hl20-local-direct-wrench-screen-batch --execute-batch

# Keep generated packets instead of discarding the bounded smoke workspace.
# The target must be empty. The command writes one isolated packet per
# selected batch witness plus release-catalog.json; feed that same directory
# into maturity-report for a validated retained-result projection. This is an
# artifact inventory, not a qualification or expanded controller claim.
taoryx vehicle witness-report \
  --family hl20_mod_k --execute-batch \
  --results-dir /tmp/taoryx-hl20-witness-results
taoryx vehicle maturity-report \
  --results-dir /tmp/taoryx-hl20-witness-results

# Every committed batch status trace now carries the typed
# control.controller.method diagnostic. It is lqr or lqi only when that exact
# screen ran the corresponding reusable controller; other modes report
# not_applicable. A passing local-controller witness also includes a
# controller_metadata record that checks the runtime and screen evaluation
# agree, so controller selection is executable evidence rather than a
# plan-only advertisement. `taoryx vehicle result <output-dir>` also projects
# control_execution_evidence for any executed realization (including a
# source-surface allocator) and controller_execution_evidence for LQR/LQI:
# realization, physical-allocation flag, method/integral outputs when present,
# and committed-trace sample count. Both fail closed on disagreement and do
# not turn a local screen into qualification.

# X-15 and HL-20 each also expose a distinct source-local body-speed LQI
# screen. These use the registered tuner result through bounded generalized
# wrench coordinates, emit integral-error telemetry, and remain batch-only:
# they are not physical-effector or interactive-control endpoints.
python tools/validate_vehicle_execution_witnesses.py \
  --witness x15-local-direct-wrench-lqi-screen-batch --execute-batch
python tools/validate_vehicle_execution_witnesses.py \
  --witness hl20-local-direct-wrench-lqi-screen-batch --execute-batch

# X-15 also exposes a separate frozen-fixture source-table surface-authority
# screen. It allocates only the declared symmetric stabilator, differential
# stabilator, and rudder; it is not a full X-15 trim or flight mission.
python tools/validate_vehicle_execution_witnesses.py \
  --witness x15-source-surface-authority-screen-batch --execute-batch

# The X8 physical source-table paths are separate eight-second local LQR and
# LQI recovery screens. Both report actual bounded collective/differential
# source coordinates, not inferred individual left/right servo positions.
python tools/validate_vehicle_execution_witnesses.py \
  --witness x8-local-physical-surface-lqr-screen-batch --execute-batch
python tools/validate_vehicle_execution_witnesses.py \
  --witness x8-local-physical-surface-lqi-screen-batch --execute-batch

# The X8 direct-wrench racetrack is a separate source-autonomous nominal route.
# It has a batch endpoint and action-free interval trace; it is not an external
# six-axis control endpoint or a physical-elevon allocation claim.
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_direct_wrench_compose.yaml \
  --output build/x8-direct-wrench.composition.json
taoryx vehicle run build/x8-direct-wrench.composition.json \
  --output-dir build/x8-direct-wrench

# The B747 condition-3 source-table paths are eighty-second local three-axis
# LQR and LQI recoveries. They report actual bounded elevator, aileron, rudder,
# and throttle source coordinates without claiming servo dynamics or a
# racetrack. The LQI witness also emits a typed robustness sidecar for its
# bounded, matched ±5% pitch-moment screen; that is not wind or mass robustness.
python tools/validate_vehicle_execution_witnesses.py \
  --witness b747-condition3-local-physical-surface-lqr-screen-batch --execute-batch
python tools/validate_vehicle_execution_witnesses.py \
  --witness b747-condition3-local-physical-surface-lqi-screen-batch --execute-batch

# Verify the same LQI path through the common endpoint contract. --execute
# replays the exact batch witness and reads the typed result and robustness
# sidecars; --tune may be added when candidate-tuning provenance is needed.
taoryx vehicle verify b747-condition3-source-surface-attitude-rate-lqi --execute

# The B747 direct-wrench racetrack is a complete 665-second, 66k-step source
# route. It is intentionally marked slow: use the local-screen witnesses for
# ordinary edits and reserve this exact common-runner proof for promotion.
python -m pytest tests/unit/test_mission_composition_native_bridge.py \
  -m slow -rA

# A320's point-mass and named pseudo-6DOF routes are separate runnable
# reduced products. The pseudo path advertises OpenAP thrust, throttle, mass,
# fuel-flow, and surrogate attitude-response telemetry without an effector claim.
python tools/validate_vehicle_execution_witnesses.py \
  --witness a320-pseudo6dof-batch --execute-batch

# The distinct A320 local LQI endpoint runs the retained common-host candidate
# through its exact named aileron/elevator/rudder response coordinates. It is
# batch-only: native controls and integral telemetry are recorded in
# local_screen.json. It also emits a composition-bound convergence_report.json.
# Its endpoint contract explicitly declares that no mass, wind, or persistent-
# offset seam exists at this pseudo-6DOF fidelity; that is not robustness
# evidence and no physical surface-allocation claim is made.
python tools/validate_vehicle_execution_witnesses.py \
  --witness a320-local-native-coordinate-lqi-screen-batch --execute-batch
taoryx vehicle verify a320-pseudo6dof-native-coordinate-lqi --execute

# NESC is an exact source-history replay with a pseudo-6DOF response-law
# projection. It has no participating derivative, guidance, gimbal, or
# robustness seam, so the endpoint verifies its explicit controller-free
# disposition instead of fabricating one.
taoryx vehicle verify nesc-source-history-replay-pseudo6dof --execute

# Passive tumbling has two intentionally uncontrolled reductions. Both expose
# the standard read-only replay session with no caller actions; this checks the
# exact point-mass and native-rigid-body-reuse release paths.
python tools/validate_vehicle_execution_witnesses.py \
  --family tumbling_body --execute-batch
taoryx vehicle verify tumbling-body-passive-release-pseudo6dof --execute

# F-16 physical control has one-second LQR route-entry screens plus a distinct
# five-second fixed-altitude physical LQI velocity-recovery screen. The surface
# paths write actual actuator-overlay positions and allocation diagnostics; the
# direct-wrench lane remains an explicit comparison screen.
taoryx vehicle compose examples/vehicle_composition/f16_local_physical_surface_screen_compose.yaml \
  --output build/f16-surface-screen.composition.json
taoryx vehicle run build/f16-surface-screen.composition.json \
  --output-dir build/f16-surface-screen
taoryx vehicle result build/f16-surface-screen \
  --composition build/f16-surface-screen.composition.json
taoryx vehicle compose examples/vehicle_composition/f16_local_physical_surface_lqi_screen_compose.yaml \
  --output build/f16-surface-lqi-screen.composition.json
taoryx vehicle run build/f16-surface-lqi-screen.composition.json \
  --output-dir build/f16-surface-lqi-screen

# The F-16 schedule-interior endpoint executes two retained body-w LQR
# recoveries at each of four independently retrimmed source nodes. It is a
# held-node campaign, not a continuous gain scheduler or a route.
taoryx vehicle compose examples/vehicle_composition/f16_local_physical_surface_lqr_schedule_interior_screen_compose.yaml \
  --output build/f16-surface-lqr-schedule-interior-screen.composition.json
taoryx vehicle run build/f16-surface-lqr-schedule-interior-screen.composition.json \
  --output-dir build/f16-surface-lqr-schedule-interior-screen

# The separate F-16 transition screen time-marches four retained altitude-
# coordinate cases. It linearly blends only the validated source endpoint
# derivatives/effectiveness and allocates every scheduled wrench through actual
# bounded elevator, aileron, rudder, and throttle. Its bound robustness sidecar
# replays all four transitions at nominal and under +/-5% of the shared declared
# pitch-wrench scale (500 N m). The offset enters the post-source-derivative,
# full-inertia dynamics seam—not the controller request or allocator output.
# It is not navigation, wind or mass robustness, a full envelope, or flight
# qualification.
taoryx vehicle compose examples/vehicle_composition/f16_local_physical_surface_lqr_schedule_transition_screen_compose.yaml \
  --output build/f16-surface-lqr-schedule-transition-screen.composition.json
taoryx vehicle run build/f16-surface-lqr-schedule-transition-screen.composition.json \
  --output-dir build/f16-surface-lqr-schedule-transition-screen

# The companion LQI screen uses the same four held source nodes but retains
# only the source-feasible +/-0.25 m/s body-w interior. It proves bounded
# offset-free local recovery and allocation, not interpolation, node transfer,
# wind/mass rejection, a route, or qualification.
taoryx vehicle witness-report \
  --witness f16-local-surface-lqi-schedule-interior-screen-batch \
  --execute-batch

# Hummingbird's local source-hover screen closes the controller loop through
# its four bounded, lagged rotor-speed effectors. It is an LQI attitude/rate
# recovery screen with requested-versus-achieved moment allocation evidence.
# Its result also retains nonlinear_validation.json from the shared physical-LQI
# runner, including the integral state, realized allocation, and derivative
# environment used for the exact local run.
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_individual_rotor_lqi_screen_compose.yaml \
  --output build/hummingbird-lqi-screen.composition.json
taoryx vehicle run build/hummingbird-lqi-screen.composition.json \
  --output-dir build/hummingbird-lqi-screen
taoryx vehicle result build/hummingbird-lqi-screen \
  --composition build/hummingbird-lqi-screen.composition.json

# The vertical companion keeps the same source plant and four actual rotors,
# but extends its LQI design to collective body-z force plus roll/pitch/yaw
# moments. Its bounded source-local route is climb, hover capture, descent,
# and return hover. It is deliberately not a wind, contact, landing, or
# full-route qualification endpoint.
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_vertical_translation_lqi_screen_compose.yaml \
  --output build/hummingbird-vertical-lqi-screen.composition.json
taoryx vehicle run build/hummingbird-vertical-lqi-screen.composition.json \
  --output-dir build/hummingbird-vertical-lqi-screen
taoryx vehicle result build/hummingbird-vertical-lqi-screen \
  --composition build/hummingbird-vertical-lqi-screen.composition.json
taoryx vehicle verify hummingbird-source-vertical-translation-rotor-lqi --execute

# The horizontal companion has a separate source-local forward/yaw/lateral/
# rearward/return capture proof. Its robustness status is explicitly
# not_applicable because it has no declared persistent disturbance injection.
taoryx vehicle verify hummingbird-source-horizontal-translation-rotor-lqi --execute

# The companion direct-wrench screen uses the same pinned source-hover plant
# for a bounded six-axis LQR recovery comparator. Its batch-visible requests
# are controller-generated; it is not an external episode-control, rotor-
# allocation, motor-lag, or flight-mission endpoint.
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_local_direct_wrench_screen_compose.yaml \
  --output build/hummingbird-direct-wrench-screen.composition.json
taoryx vehicle preflight build/hummingbird-direct-wrench-screen.composition.json
taoryx vehicle run build/hummingbird-direct-wrench-screen.composition.json \
  --output-dir build/hummingbird-direct-wrench-screen
taoryx vehicle result build/hummingbird-direct-wrench-screen \
  --composition build/hummingbird-direct-wrench-screen.composition.json

# Cross-family catalogue contracts for controller-slice coverage, model
# membership, interfaces, and onboarding data. Planned tiers remain blockers.
python tools/dev.py vehicle-catalogue

# Verify that every composition-managed maturity record still names a concrete
# catalog family with a checked-in public batch witness.
python tools/dev.py check-vehicle-maturity

# Fast declaration and value-space/catalog audit.
taoryx vehicle topology-report
taoryx vehicle maturity-report

# Compile every declared endpoint witness, preflight it, lower it, and open
# each advertised episode at its committed-truth boundary.
taoryx vehicle maturity-report --check-execution-witnesses

# Execute every batch witness and replay every registered batch/episode pair.
# This is intentionally slower because it exercises actual source-owned
# factories and artifact contracts.
taoryx vehicle maturity-report \
  --check-execution-witnesses \
  --execute-batch-witnesses \
  --execute-parity-witnesses

# To retain the entire batch matrix in one release-ready evidence corpus, use
# this separate empty destination. The command indexes the generated corpus in
# the same maturity report; it remains packet evidence, not qualification.
taoryx vehicle maturity-report \
  --execute-batch-witnesses \
  --retain-batch-results-dir /tmp/taoryx-all-batch-witness-results

# Reconcile the authoritative asset inventory, every advertisement and exact
# common batch/session registration, then verify the generated coverage matrix.
python tools/dev.py mission-composition-completion
```

When batch execution is enabled, each witness record includes an
`interface_trace` summary: the exact interface fingerprint and the status,
resource, and diagnostic channel IDs projected at committed truth boundaries.
Its control evidence remains separately represented by the semantic action
trace. This is an execution-coverage report, not controller or qualification
promotion. The containing maturity report also provides
`execution.batch_interface_trace_conformance`, a compact cross-witness count
and channel inventory; it is `not_checked` until batch witnesses are executed.

The maturity command proves only the declared composition/execution and parity
contracts. The completion command additionally reconciles discovery and exact
common-interface coverage. Neither promotes a direct-wrench screen, replay,
pseudo-6DOF response law, or nominal mission to physical-effector or family
qualification. The generated completion matrix lists every family and
realization, including explicit blockers and intentionally deferred physics.

## Model-to-mission automation

Do not start a new model by hand-coding a bespoke mission graph or copying
controller gains. First exercise the installed advertisement:

```bash
taoryx plugins check --profile models
taoryx model list
taoryx model assess --output build/model-assessment.json
taoryx model plan <provider-id> <model-id> --output build/model-plan.json
taoryx model scaffold <provider-id> <model-id> --output build/model-draft.yaml
```

The plan joins the model's data contract, frames, controls, authorities,
control intents, navigation parameters, initialization modes, mission
templates, segments, family adapter, and tuning registrations. Fill only the
generated `<REQUIRED>` and `<SELECT>` sites, then validate the ordinary YAML
through the exact owning provider:

`focused_endpoint_verification` names each checked-in vertical proof for that
model and gives its exact static and `--execute` command. Its selected-match
field is deliberately narrow: a proof for a different mission or fidelity is
discoverable, but is not presented as evidence for the selected configuration.

When a caller supplies `--fidelity` without `--mission`, `model plan` keeps
that fidelity and selects its first runnable compatible mission. This prevents
a physical local-control-screen default from silently overriding a requested
reduced tier. Supplying both an incompatible fidelity and mission remains a
hard error. A registered local tuning campaign can still be invoked for a
blocked end-to-end realization; its report is local design evidence, not a
new runnable mission or qualification claim.

```bash
taoryx model compile build/model-draft.yaml \
  --output build/model-configuration.json
```

For a controlled realization, the plug-in must supply state derivatives,
trim/linearization semantics, authority, operating points, scales, and
family-specific allocation before it registers a tuning campaign. Once it
does, the host runs the same bounded sequence for every topology:

```bash
taoryx model tune <provider-id> <model-id> \
  --campaign <campaign-id> \
  --output build/tuning-report.json
```

A missing campaign, route coordinate, control axis, or effector is an explicit
integration gap. Do not synthesize a value to make the command pass. See the
automation architecture page for the Python helpers and the exact host versus
plug-in ownership table.

## Grammar validation

Validate source before attempting to run it:

```bash
taoryx-validate --profile taos96 path/to/file.prb path/to/file.tbl
taoryx-validate --profile taoryx path/to/extension.prb
python tools/dev.py grammar
python tools/dev.py test-grammar
```

Use `taos96` for historical-language fixtures and `taoryx` for successor
extensions. For malformed input, inspect located diagnostics and recovery
records rather than discarding source evidence. Add independent positive and
negative fixtures under `tests/fixtures/grammar_baseline/` when changing
grammar behavior.

For junior-friendly corpus regeneration, use the batch command from the
repository root:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family all --execute --output artifacts/examples/all
```

This writes AST/parser reports for both profiles, attempts the TAOS96-compatible
local runtime where bundled tables permit execution, executes Taoryx full
examples, and regenerates the indexed runtime showcases. Each successful case
gets a run report, normalized artifact, telemetry CSV, and plots. A short run
may be incomplete at the step budget; that is distinct from a runtime failure.

## Segments and composition

For a normal waypoint or segment course, use the high-level composition
builder. It keeps the source problem and native compiler boundary intact while
removing most of the graph bookkeeping:

~~~python
from pathlib import Path

from taoryx.composition import TrajectoryBuilder, WaypointSpec

builder = TrajectoryBuilder(
    "demo-course",
    vehicle="skywalker_x8",
    family="fixed-wing-uav",
    source_problem="examples/mission.prb",
)
builder.use(
    "trim_hold",
    "trim",
    duration_s=20.0,
    target={"speed_m_s": 18.0},
    tolerance={"speed_m_s": 1.0},
)
builder.waypoint_course(
    (
        WaypointSpec(
            id="north",
            target={"north_m": 100.0},
            tolerance={"north_m": 15.0},
            duration_s=30.0,
        ),
        WaypointSpec(
            id="east",
            target={"east_m": 100.0},
            tolerance={"east_m": 15.0},
            duration_s=30.0,
        ),
    )
)
scenario = builder.build()
problem, manifest, audit = builder.compile(Path("repo-root"))
~~~

Use `SegmentCompositionRegistry.standard().names()` to discover the reviewed
templates: `trim_hold`, `hover`, `waypoint`, `altitude_capture`,
`heading_capture`, and `moving_target_intercept`. The last one is intentionally
more demanding than a waypoint: it requires a target reference plus explicit
LOS/closure targets and tolerances.
builder.evaluate() returns a structural report; warnings are visible and
invalid capture goals, graph edges, or termination policies fail before
compilation. Use the lower-level catalog only when a component needs custom
events, state transitions, or non-time entry/exit expressions.

Use native `.prb` `*segment`, `*when`, `goto`, and `stop` constructs when the
change belongs to the documented source language. Use the external
segmentation catalog when the task needs reusable orchestration metadata,
controller bindings, goals, events, or transition policies.

### Simple Aero-style specialized segments

For the synthetic Simple Aero corpus, start with
[`docs/architecture/simple_aero-segments.md`](architecture/simple_aero-segments.md) and
[`verification/simple_aero_segment_catalog.yaml`](../verification/simple_aero_segment_catalog.yaml).
The reusable templates are `powered_ascent`, `ballistic_coast`,
`bank_maneuver`, `alpha_profile`, `skip_maneuver`, `terminal_pronav`, and
`moving_target_intercept`. They apply to point-mass 3-DOF, kinematic
pseudo-6-DOF, and—after additional plant gates—rigid-body 6-DOF.

The Simple Aero source vocabulary is deliberately separate from vehicle
promotion. Each reviewed workflow template now has a real bounded
point-mass batch lowering: its source-shaped values map to a generated
fixed-L/D alpha, bank, and duration profile. That proves a typed
compose-to-run path and its standardized telemetry—not a vehicle's thrust,
aero tables, bank sign, alpha response, target closure, terminal behavior, or
historical Simple Aero runtime. Reuse the phase contract and retune the
vehicle-specific controls, tables, limits, and time-to-go values; never copy
those values blindly from the surrogate.

Inspect the same workflow through the provider-independent Mission Composition
advertisement with:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py \
  --model simple_aero
```

Audit every registry-backed advertisement and run the common analytical
trajectory interface with:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_catalog.py --audit
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_reference.py \
  --output /tmp/mission-composition-trajectory.json
PYTHONPATH=src python3 examples/trajectory_provider/mission_composition_contract_probe.py \
  --output /tmp/mission-composition-contract-probe.json
```

The audit proves configuration- and output-schema publication integrity. The
reference run proves the typed configuration + requested core/telemetry
selection → common runner → standardized trajectory/failure lifecycle. The
contract probe additionally proves typed scalar/vector/structured telemetry,
first-class spawned-entity relationships, and accepted-boundary initial-state
snapshots. `python tools/dev.py mission-composition-completion` reconciles the
authoritative inventory and exact common batch/session registrations with the
generated coverage matrix. None of these commands promotes vehicle fidelity
evidence.

That schema exposes launch and endpoint geometry, initial mass and burnout
checkpoints, fixed-L/D surrogate inputs, and the reviewed ballistic, phugoid,
skip, slalom, and weave recipes. It declares only point-mass fidelity. The
templates are runnable fixed-L/D lowerings, not vehicle execution or
qualification claims.

For a minimal typed request for any reviewed template, use the plug-in helper:

```python
from taoryx.trajectory import build_simple_aero_template_configuration

request = build_simple_aero_template_configuration("phugoid")
```

Validate that request with the registry provider, then submit it to the normal
batch runner; parameters can be replaced in the typed tree before validation.

For readable typed authoring that still exercises the full configuration
grammar, prefer `SimpleAeroMission` and the named `SimpleAeroSegment`
constructors. The mission can use either endpoint type, all launch/target,
surrogate, checkpoint, and runtime fields, and any ordered combination of the
twelve published segment variants:

```python
from taoryx.trajectory import SimpleAeroMission, SimpleAeroSegment, prepare_simple_aero_mission

prepared = prepare_simple_aero_mission(
    SimpleAeroMission(
        configuration_id="custom-simple-aero",
        segments=(
            SimpleAeroSegment.powered_ascent(duration_s=4.0),
            SimpleAeroSegment.phugoid(frequency_hz=0.04, amplitude_deg=1.5),
            SimpleAeroSegment.terminal_pronav(capture_range_m=50.0),
        ),
    )
)
```

The default `Custom Composition (Open Sequence)` operation profile registers
the generic fixed-L/D batch lowering without calling a caller-authored sequence
a reviewed template. Use `SimpleAeroMission.template("phugoid")` when the
published phugoid sequence itself is the intended starting point.

Before composing a Simple Aero phase into a vehicle route, run the isolated fixture
ladder:

```bash
python tools/dev.py test-simple_aero-segments
```

This Simple Aero-specific runtime view proves grammar, completion, telemetry,
finite samples, time ordering, and isolated segment span. It does not promote
the phase: vehicle-quality gates remain deferred until a vehicle adapter
supplies bounded aero/plant, control, convergence, and terminal evidence.

For the shortest path to a runnable reduced-order case, use the parameter
builder instead of hand-writing four native segments:

```python
from taoryx.simple_aero_builder import build_fixed_ld_3dof

build = build_fixed_ld_3dof(
    vehicle_id="generic-3dof",
    vbo_m_s=900.0,
    apogee_altitude_m=20_000.0,
    pitch_over_angle_deg=75.0,
    target_range_m=100_000.0,
    target_bearing_deg=90.0,
    initial_heading_offset_deg=8.0,
    lift_to_drag=4.0,
)
build.write("build/simple_aero-demo.prb", "build/simple_aero-demo.manifest.json")
```

Validate the generated extension with `taoryx`, then run it through the same
profile explicitly:

```bash
taoryx-validate --profile taoryx build/simple_aero-demo.prb
```

The generated manifest shows the computed phase durations, initial heading,
target location, and fixed-L/D assumptions. `vbo_m_s`, apogee, pitch-over,
heading offset, and `lift_to_drag` are convenience parameters—not claims that
the resulting trajectory exactly reaches those values. For a promoted vehicle,
override phase durations as needed, bind vehicle tables/controllers, and run
the segment evidence ladder before route verification.

For the X-15, the focused follow-on fixtures are the bounded 3-DOF
`x15_phugoid_3dof.prb` alpha/energy profile and the 6-DOF
`x15_weave_6dof.prb` two-cycle bank reversal. Their machine-checked gates live
in `tests/e2e/test_glider_family_validation.py`; the current claim boundary is
segment response, not natural-mode identification, crossrange optimization, or
route promotion.

For a low-code X-15 menu, use
[`verification/x15_maneuver_catalog.yaml`](../verification/x15_maneuver_catalog.yaml):

```python
from taoryx.x15_maneuvers import load_x15_maneuver_catalog

catalog = load_x15_maneuver_catalog("verification/x15_maneuver_catalog.yaml")
settled = catalog.select("settled")
```

Use only `settled` rows as inputs to new composition work. Inspect each row's
`focused_test`, `tables`, and `quality_gates` before changing parameters. The
catalog currently settles five X-15 maneuvers; its trim-to-terminal ProNav row
is still a candidate pending terminal miss-distance evidence. This menu is a
segment-quality checkpoint, not a route-promotion or flight-certification
shortcut.

Use the smallest test slice for the change. The X-15 module marks isolated
segment tests with `segment` and their dynamics tier with `dof3` or `dof6`:

```bash
python tools/dev.py test-x15-catalog
python -m pytest tests/e2e/test_glider_family_validation.py \
  -m 'x15 and segment and dof3' -k phugoid -o addopts=''
python -m pytest tests/e2e/test_glider_family_validation.py \
  -m 'x15 and segment and dof6' -k weave -o addopts=''
```

Use `python tools/dev.py test-plots` for visualization-only checks and
`python tools/dev.py test-grammar` for parser work. Reserve
`python tools/dev.py test-x15` and the full `check`/`pytest` gates for a
promotion checkpoint or a handoff; they intentionally include much more
vehicle and artifact coverage. The complete slice map is in
[`docs/BUILDING_TESTS.md`](BUILDING_TESTS.md).

After execution, score the composed run instead of inspecting plots by eye:

```python
from taoryx.composition import RuntimeEvaluationOptions, evaluate_run

evidence = evaluate_run(
    scenario,
    artifact,
    options=RuntimeEvaluationOptions(
        channel_aliases={"altitude_m": "position.altitude.geodetic"},
        transition_tolerances={"mass.total": 1.0e-6},
        max_saturation_fraction=0.05,
    ),
)
evidence.raise_for_failure()
```

This is a segment-level validation ladder: entry state, goal capture and
dwell, transition event/continuity, exit or terminal coverage, finite required
telemetry, and optional saturation. Missing channels produce `blocked`, a
contract violation produces `fail`, and omitted optional channels produce a
visible `warning`. Use canonical semantic output names or provide explicit
aliases; the evaluator never guesses that `altitude_m` or `north_m` means a
particular vehicle channel. For non-time segments, require runtime spans so
the evaluator does not infer boundaries from source text.

### Segment promotion before route verification

Do not promote a controller directly into a route. The segment promotion
matrix at `verification/segment_promotion.yaml` is the intermediate gate. Each
row names one vehicle/scenario/segment pair, its goal kind, the evidence
categories that must pass, and the focused test that produced the evidence.
Validate its coverage against the segmentation catalog with:

```python
from taoryx.segment_promotion import (
    load_segment_promotion_catalog,
    validate_promotion_coverage,
)
from taoryx.segmentation import load_catalog

segmentation = load_catalog("verification/segmentation_catalog.yaml")
promotions = load_segment_promotion_catalog("verification/segment_promotion.yaml")
errors = validate_promotion_coverage(promotions, segmentation)
assert not errors, errors
```

After running a focused segment scenario, pass its runtime report through
`evaluate_promotion_catalog(...)`. A segment is promoted only when every
required category is present and every required check is `pass`; a missing
channel, missing event, incomplete termination, or untested quality category
remains `blocked`. The resulting evidence hash is the stamp consumed by route
review. The matrix is coverage metadata, not a manual “green” override: the
existing family tests are evidence inputs, while the promotion report is the
decision boundary.

```bash
python tools/dev.py segment-lint
python tools/dev.py segment-build
python tools/dev.py segment-run
```

The compiler produces a generated `.prb`, resolved manifest, and transition
audit. Review the YAML catalog and source problem, not generated outputs.
Compilation proves composition and syntax, not plant or trajectory validity;
add closure, convergence, envelope, and controller tests.

For moving-target guidance, compose the segment with an explicit reference and
score the native guidance channels rather than treating a route waypoint as an
intercept:

```python
builder.moving_target_intercept(
    "terminal-intercept",
    duration_s=10.0,
    reference="target-2",
    target={
        "pro_nav_los_range_m": 25.0,
        "pro_nav_closing_velocity_m_s": 0.0,
    },
    tolerance={
        "pro_nav_los_range_m": 25.0,
        "pro_nav_closing_velocity_m_s": 5.0,
    },
)

evidence = evaluate_run(
    scenario,
    artifact,
    options=RuntimeEvaluationOptions(
        required_channels=(
            "pro_nav_active",
            "pro_nav_los_range_m",
            "pro_nav_closing_velocity_m_s",
            "pro_nav_acceleration_response_residual_m_s2",
        ),
    ),
)
```

The required-channel list is a hard evidence contract. Missing or non-finite
channels produce `blocked`; aliases must be declared explicitly. Consult
`verification/segment_capability_matrix.yaml` for the current vehicle
boundary: X15 has partial source-trim-to-ProNav evidence, while Hummingbird
has a bounded focused fixture but remains a candidate until the remaining
vehicle-quality gates and promotion hash exist.

For typed initialization/configuration changes, use `ScenarioCompiler` instead
of editing state tuples or source text in place:

```python
from taoryx.scenario import ParameterOverride, ScenarioCompiler

scenario = ScenarioCompiler().compile(
    "mission.prb",
    patches=(ParameterOverride("launch_altitude", 30_000.0, unit="m"),),
)
artifacts = scenario.run(output_dir="artifacts/mission")
```

Composition patches are ordered, unit-aware, recorded in resolution metadata,
and must not bypass declared control or actuator routes.

## Vehicle addition and the validation ladder

Treat a new vehicle as a staged evidence problem. Each stage consumes the
artifacts from the previous stage; a later green plot does not promote an
earlier blocked convention or data check.

| Stage | Question | Required evidence |
| --- | --- | --- |
| Registry/data | Can the model be discovered and loaded? | SI metadata, family contract, table bindings, source provenance/hash, generated problem profile, onboarding report |
| Grammar/lowering | Does the declared source and composition mean what the author intended? | `taoryx-validate`, segment lint/build, resolved manifest, transition audit |
| Convention firewall | Are frames, axes, table orientation, coefficient signs, and controls correct? | table inspection, in-range/boundary queries, frame/quaternion tests, signed control-direction probes |
| Plant validity | Does the model satisfy its own equations? | initial-condition audit, trim residuals, force/moment dimensionalization, independent closure, bounded propagation |
| Numerical quality | Is the result reproducible and time-step credible? | `dt`, `dt/2`, `dt/4`, adaptive comparison, event-aware exclusions, output hashes |
| Controller validity | Does the controller stabilize the local plant without violating the interface? | named `A/B` provenance, controllability, LQR poles, uncertainty screen, actuator/slew/allocation telemetry |
| Segment validity | Do entry, handoff, and exit contracts hold? | segment manifest, inherited/reset state audit, controller reset events, goal/termination evidence |
| Checkpoint mission | Does the complete scenario meet bounded objectives? | generated fidelity-ladder packet, objective gates, quality metrics, plots, claim status |

Use the following commands as the normal progression:

```bash
taoryx vehicle catalog --detail full
taoryx vehicle describe new_vehicle
taoryx vehicle authoring new_vehicle
python tools/validate_vehicle_onboarding.py --vehicle new_vehicle --strict
python tools/dev.py generate-problems
python tools/dev.py vehicles
taoryx table inspect path/to/vehicle.tbl --html build/table-explorer.html
python tools/dev.py control-directions
python tools/dev.py trim-vehicles
python tools/dev.py segment-lint
python tools/dev.py segment-build
python tools/dev.py segment-run
python tools/dev.py fidelity-packet
python tools/dev.py maneuver-matrix
```

The full catalogue card must expose a summary and taxonomy, explicit typed
geometry/mass/envelope/effector alternatives (including `not_represented` or
`not_applicable` where appropriate), all four fidelity meanings, per-tier
control authority, semantic initialization/variant/segment inputs, and a
complete class-normal segment plan. Keep the typed catalogue contracts through
provider and authoring code; JSON cards are an API boundary, not an argument
bag to deserialize internally. A mission template either names a nonempty
fixed order or publishes an `open_segment_sequence` grammar with its node,
allowed segment IDs, and cardinality; never encode caller-authored order as an
empty fixed tuple. A maximum Mach, speed, or altitude in this card
is a declared model-validity bound, not a performance or qualification claim.

The onboarding validator answers whether the metadata path is complete; it
does not prove trim or mission behavior. `vehicles` checks registry and
provenance coverage, the control harness checks signed responses, and the
fidelity packet owns the multi-tier trajectory evidence. Keep the first
failing stage visible in the report instead of replacing it with a score.

### Data and convention firewall

Before tuning, make a small model packet and inspect it by hand. It should
state the state/control order, body/wind/world bases, SI conversions, reference
area/span/chord, mass/CG/inertia behavior, table axis order and bounds,
interpolation policy, coefficient source status, and actuator bounds. Query a
nominal point, every relevant boundary, and one deliberately out-of-range
point. Out-of-range behavior must be an explicit block or diagnostic; never
reverse an axis or negate a coefficient merely to make a trajectory look right.

The control-direction harness is a firewall, not a tuning test. It perturbs one
declared control around an identical baseline and records the signed body
force/moment response, antisymmetry error, expected source sign, and achieved
command. For example, the B747 elevator should produce the declared negative
body-`y` moment, the X8 collective and differential elevon probes should test
body-`y` and body-`x` independently, and the Hummingbird rotor-speed probe
should test the declared body-`z` force sign. Frame transforms and
force/moment reference transfers must be checked separately from table lookup.

### Trim, LQR, and mass-property gates

The accepted trim must identify the exact state/control ordering and carry
unscaled force and moment residuals. Linearize the state-rate evaluator at that
trim with the same tables, atmosphere, propulsion, mass, CG, and inertia used
by propagation. Record perturbation sizes, `A/B` names, `Q/R`, controllability,
closed-loop eigenvalues, table margins, and actuator/slew/allocation behavior.
Require the nominal poles to be Hurwitz and fail closed if an allowed mass,
CG, inertia, or aerodynamic uncertainty corner produces an unstable pole or a
missing table query. A mass-dependent inertia provider can update attitude
gains, but it does not replace a fresh plant-bound `A/B` design when mass also
changes translation, propulsion, CG, or aerodynamic derivatives.

### Generated trajectories and segments

The checkpoint trajectories are catalog-driven, not hand-selected after the
fact. The fidelity ladder and long-validation catalogs parameterize vehicle,
source problem/table set, duration, step factors, bounds, objectives, and
termination rules. The generated packet must include a manifest, initial
condition audit, event timeline, closure metrics, convergence report, plots,
and hashes. The usual order is source/static trim hold, 3-DOF anchor, derived
kinematic bridge, short rigid-body 6-DOF run, time-step convergence, source
differential, bounded recovery/control maneuver, then long checkpoint mission.

For each segment, validate the entry state and inherited/reset fields before
integrating; validate continuity or an explicit impulse/mass change at the
handoff; and validate the exit condition, target/action, final state, and
termination reason. The compiler rejects duplicate IDs, missing targets,
cycles, invalid `stop`/`goto` forms, missing source segments, and missing
integration blocks. The controller must reset at the first segment and each
transition. Compilation is only composition evidence; plant closure,
convergence, envelope, and controller gates still apply.

## Batch runs, timesteps, and plots

For a deterministic batch run, use the shared runner and choose the integrator
explicitly when numerical method matters:

```python
from taoryx.runtime.runner import run_files

report = run_files("mission.prb", ("vehicle.tbl",), max_steps=10_000,
                   integrator="rk4", output_dir="artifacts/mission")
```

For an external controller, player, notebook, or learning agent, use
`InteractiveSession`. Each call accepts a duration and named bounded commands;
it advances numerical time and records requested/applied commands:

```python
session = scenario.interactive_session()
snapshot = session.step(0.02, {"fin_pitch": 0.1, "throttle": 0.7})
artifact = session.to_run_artifact()
```

For a Vehicle Composition model, prefer the common Mission Composition
session contract and select one advertised authority profile at open. Reduced
A320/F-16 sessions can expose kinematic, normalized pilot, live-waypoint, and
(for F-16 pseudo-6DOF) body-rate profiles through the same step route.
Hummingbird's pseudo-6DOF session exposes aggregate attitude/thrust,
north/east/positive-up velocity plus yaw, and live local-waypoint control; its
lowered action remains the five-coordinate aggregate response-law seam and
never individual rotor commands. Use
`MissionCompositionSessionManager.switch_authority` only for profiles that
advertise `explicit_bumpless`; inspect each observation's `control_authority`
and each step's lowering evidence. See
[Vehicle interface contract](api/vehicle-interface-contract.md#session-profile-negotiation-and-live-transfer)
and
[Model-to-mission automation](developer/model-authoring-automation.md#select-a-streaming-control-profile).

The low-fidelity conformance ladder uses that same route before promotion to
the F-16: the ballistic fixture is explicit zero-action open loop; the
constant-velocity waypoint fixture switches among configured guidance,
kinematic velocity commands, and live waypoint retargeting; the debug contract
probe exercises continuous/vector, discrete/enum/boolean, and one-shot event
profiles; and Simple Aero switches between its generated schedule and direct
throttle. All four retain one session clock and sequence across allowed
profile handoffs.

Render from the artifact rather than reparsing source:

```python
from taoryx.visualization import render_run_artifact_plots
render_run_artifact_plots(artifact, "artifacts/mission/plots")
```

Use standard observations for live consumers, declared status channels for
model-specific telemetry, and deep named state only for diagnostics.

The equivalent CLI routes are:

```bash
taoryx run mission.prb vehicle.tbl --profile taoryx --integrator rk4 --output-dir artifacts/mission
taoryx scenario compile mission.prb vehicle.tbl --profile taoryx --output artifacts/mission/scenario.json
taoryx artifact plot path/to/artifact.json --output-dir artifacts/mission/plots
taoryx integrators list
```

Use `python3` instead of `python` on systems where the `python` alias is not
installed. The repository's required gates are still the commands named in
`AGENTS.md`.

## Common traps

- Do not edit generated files under `build/`, `qa/`, or `artifacts/` as source.
  Change the YAML, TeX, fixture, or Python input that generates them.
- Do not use `docs/plan/` as an API reference without checking the current
  implementation. Plans may describe target APIs; current public runtime
  entry points are listed in this page and the architecture docs.
- Do not treat `parse`, `compile`, or `run` as equivalent evidence. Grammar
  validation, composition validation, numerical execution, and historical
  parity are separate claims.
- Do not mutate physical state through a controller command. Commands must
  pass through declared controls, bounds, slew limits, and allocators.
- Do not add a new `.prb` dialect for orchestration metadata. Use the
  segmentation catalog or scenario composition layer for TAORYX extensions.
- Check the existing worktree before editing. Preserve unrelated user changes
  and avoid modifying established fixtures silently.

## Controls above trim

The safe path is:

```text
plant -> TrimResult -> local A/B -> controller -> demand -> allocator -> plant
```

`plant_residual` in `solve_trim` must use the same frames, tables, mass
properties, actuators, and propulsion as propagation. After trim, use
`finite_difference_dynamics_linearization` with a state-rate evaluator to
obtain true state-derivative Jacobians; `finite_difference_linearization` is
for residual diagnostics, and force/moment derivatives are not automatically
an `A,B` pair. Bind named states and controls exactly to the trim artifact,
then apply bounds, slew limits, and family-specific allocation through the
control contracts.

For changing mass properties, provide the rigid-body model's
`inertia_provider(state)` from the vehicle adapter. The runtime has an explicit
`linear-dry-mass` interpolation for declared reference and dry-mass inertias,
but it never infers inertia from mass. Pair that provider with
`*runtime lqr attitude update=mass` when the direct-moment attitude bridge is
appropriate. A full source-trim `A/B` design is still required when changing
mass also changes translational, propulsion, CG, or aerodynamic derivatives.

```bash
python tools/dev.py trim-vehicles
python tools/dev.py control-directions
python -m pytest -m algorithms
```

## Definition of done

1. Add or update the nearest README and machine-readable manifest.
2. State whether the result is manual-bounded, source-backed, or a TAORYX
   extension.
3. Add focused tests; keep generated artifacts under ignored `artifacts/` or
   `build/` paths.
4. Run the required repository gates from `AGENTS.md`.
5. Report unresolved grammar, numerical, provenance, or historical-runtime
   limitations explicitly.

## Further reading

- [Build and validation](BUILDING.md)
- [Test views](BUILDING_TESTS.md)
- [Extensions documentation contract](extensions/problem-file-guide.md)
- [Codex handoff](manual/CODEX_HANDOFF.md)
