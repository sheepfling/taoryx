# Vehicle Composition Registry

The vehicle composition registry is the user-facing index for composing a
trajectory. It does not replace a vehicle's source data, nonlinear plant, or
qualification record. Instead, it joins those authorities so a caller can ask
one question:

> For this vehicle family and fidelity, what may I initialize, chain, and
> control?

The source overlay is [vehicle_composition_registry.yaml](../../verification/vehicle_composition_registry.yaml).
It is validated against the horizontal fidelity registry and the unified
family-manifest join before it can be inspected.

## Discovering a vehicle

```bash
taoryx vehicle list
taoryx vehicle inspect skywalker_x8
taoryx vehicle schema skywalker_x8 initialization
taoryx vehicle schema skywalker_x8 segments
taoryx vehicle schema skywalker_x8 missions
taoryx vehicle endpoints skywalker_x8
```

`list` is intentionally compact. `inspect` returns the full composition
contract, including the four canonical fidelity tiers:

1. `point_mass_3dof` — force-model translation.
2. `pseudo_6dof` — named attitude/rate response law.
3. `rigid_body_6dof_direct_wrench` — rigid-body plant with explicitly
   labeled generalized-wrench control.
4. `rigid_body_6dof_surface_allocated` — rigid-body plant with requested
   wrench allocated through declared physical effectors.

Each entry reports whether the profile is declared, its promotion status,
required adapter operations, and remaining blockers. A declared profile is
not automatically a runnable or qualified vehicle claim.

`endpoints` reports a separate and stricter capability: the exact source-owned
batch runner or accepted-truth episode factory currently registered for a
family/mission/fidelity tuple. The declaration lives in
[vehicle_execution_bindings.yaml](../../verification/vehicle_execution_bindings.yaml).
An entry can be `runnable` or `planned` with concrete blockers. Omission means
there is no execution path; TAORYX never falls back to a nearby family or a
generic force/moment model.

## Compile a selected mission

The registry is also an executable semantic boundary. A user selects one
vehicle, canonical fidelity tier, initialization contract, mission template,
and the template's exact ordered segment instances in a compact YAML request:

```bash
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_compose.yaml \
  --output generated/x8-racetrack-semantic.json
```

The compiler rejects unknown inputs, missing required inputs, noncanonical
units, a fidelity not declared by the family, an initialization not allowed by
the chosen mission, or a reordered/skipped segment sequence. Each following
segment receives the preceding segment's terminal truth state unless the
segment explicitly declares a physical transition.

The result is an immutable **semantic adapter handoff**, not a completed
simulation. It records the selected native adapter, fidelity profile, required
adapter operations, promotion blockers, backing templates, and exact
inputs/units. Runtime lowering remains family-adapter work: this layer does
not create hidden forces, moments, effectors, or qualification claims.

## Bind the native adapter

The second explicit boundary binds a compiled scenario to the adapter declared
by the selected family and fidelity:

```bash
taoryx vehicle lower generated/x8-racetrack-semantic.json
```

There is no physical-family fallback. If the selected adapter factory has not
been moved into the runtime registry, the command returns `blocked` with that
specific integration gap. If it can construct the adapter, it returns
`adapter_bound`, the adapter state/control schema, and its operation-level
capabilities. `adapter_bound` still is not a mission run: the remaining
adapter-specific step is a semantic-segment translator that maps the compiled
segments to trim, controller, and runtime requests.

## Preflight a native mission translation

Before a composition can run, a family translator must prove that the chosen
semantic values have an unambiguous mapping to its native mission geometry:

```bash
taoryx vehicle preflight generated/x8-racetrack-semantic.json
```

The initial vertical slices are the capability-scaled X8, B747, A320, and
F-16 racetracks. Their preflight independently derives the selected speed,
bank, turn radius, straight length, altitude gates, NED gate centers, and
start/finish heading from `verification/powered_fixed_wing_mission_profiles.yaml`.
A composition is `translation_ready` only when each input matches that derived
route. A nearby or legacy hand-authored route returns `blocked`; the runtime
may not silently substitute a different geometry.

The legacy [x8_racetrack_compose.yaml](../../examples/vehicle_composition/x8_racetrack_compose.yaml)
remains a deliberately loose composition example and correctly fails this
preflight. Use [x8_racetrack_capability_compose.yaml](../../examples/vehicle_composition/x8_racetrack_capability_compose.yaml)
when a capability-derived native route handoff is required.

`translation_ready` is deliberately narrower than `adapter_bound`: it says
only that the composition has a route representation for the selected native
translator. Adapter construction, trim, controller execution, independent
truth evaluation, and qualification remain subsequent gates. Families without
a registered translator return `not_applicable`, not an optimistic success.

## Materialize exact native inputs

The same translation is reusable by the language-backed execution path:

```bash
# Point-mass 3DOF
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml \
  --output generated/x8-racetrack-3dof-semantic.json

# Named pseudo-6DOF attitude-response bridge
taoryx vehicle compose examples/vehicle_composition/x8_racetrack_capability_compose.yaml \
  --output generated/x8-racetrack-pseudo6dof-semantic.json

taoryx vehicle materialize generated/x8-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/x8-racetrack-native-inputs
```

Use the analogous 3DOF compiled artifact with `materialize` to produce its
point-mass route inputs.

The exact interface is selected by the compiled composition rather than only
by a family/fidelity lookup. This matters when an interactive-only composition
declares a sampled observation profile:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml \
  --output generated/x8-racetrack-sensor-episode.json
taoryx vehicle interface-composition generated/x8-racetrack-sensor-episode.json
```

The command prints the fingerprinted parameter/action/status/resource and
observation contract before an episode opens, including sensor cadence and
latency. The X8 and Hummingbird sensor fixtures also have aligned batch
translators: their `sensor_observations.json` artifact replays the declared
sensor only at committed truth rows and fails if a required capture or release
boundary is absent. That artifact is observation-timing evidence, not a
sensor-noise, estimator, or batch-showcase qualification claim.

This command is currently registered for the X8 and B747 powered-fixed-wing
racetrack slices. It refuses an unready composition and writes three
disposable inputs: the native problem with exact route attributes, a retimed
independent-truth mission catalog, and a route catalog containing the same
resolved binding. It never edits a checked-in baseline `.prb`, mission
catalog, or template. Execution and qualification are deliberately separate,
later commands.

`verification/vehicle_execution_witnesses.yaml` is the companion onboarding
matrix. Every runnable exact family/mission/fidelity/operation binding must
have one checked-in composition request. The validation gate compiles each
request, requires translation-ready preflight, resolves the exact factory, and
opens interactive endpoints once. It does not substitute for a batch mission
run or qualification evidence. Use
`python tools/validate_vehicle_execution_witnesses.py --execute-batch` for
the slower public compose-to-run smoke of every batch witness. The
source-table fixed-wing factories use a declared eight-row translation smoke;
their full transport-sized racetracks remain separate nominal-mission runs.
Every batch smoke also requires and validates `status_trace.json` beside
`execution.json` and `vehicle_interface.json`. The validator checks the exact
interface ID and fingerprint, declared batch-visible channel set, committed
time ordering, and per-row completeness. It proves that the selected
interface's status/resource channels project from committed truth rather than
being present only as static catalog metadata.

## Execute an immutable nominal-run artifact

`vehicle run` is the public execution step for the current airbreather and
source-replay slices. X8 and B747 re-run translation preflight, materialize native inputs in
a temporary workspace, then execute the normal language-backed runtime. A320
and F-16 bind the same resolved racetrack directly to their declared OpenAP or
source-reduced execution adapter. Each path writes the compiled composition,
preflight, runtime report, truth telemetry, independent objective report,
envelope report, `status_trace.json`, and stable `execution.json`; the reduced
paths additionally write their trim and model-provenance artifacts:

```bash
taoryx vehicle run generated/x8-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/x8-racetrack-pseudo6dof-execution
```

The command refuses a non-`translation_ready` composition and never selects a
generic substitute vehicle or a different route. A zero exit status is only a
nominal truth-objective and declared-envelope pass. It does **not** imply
controller physical-effector realization, timestep convergence, accepted-step
replay parity, robustness, or family qualification; those additional gates
remain explicit evidence consumers of the same execution inputs.

For example, the B747 request uses the identical segment vocabulary but its
capability profile derives a transport-scaled geometry:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml \
  --output generated/b747-racetrack-pseudo6dof-semantic.json

taoryx vehicle preflight generated/b747-racetrack-pseudo6dof-semantic.json
taoryx vehicle materialize generated/b747-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/b747-racetrack-native-inputs
taoryx vehicle run generated/b747-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/b747-racetrack-pseudo6dof-execution
```

The B747 pseudo-6DOF input uses a named route-lag attitude response sidecar.
It exposes commanded/achieved bank, pitch, heading, and body-rate response;
it does not establish a physical B747 surface allocation or moment balance.

The same user flow runs the OpenAP A320 and source-local F-16 reduced
profiles. There is no fallback from one aircraft to another, and the selected
semantic speed is not silently replaced by a model default:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml \
  --output generated/a320-racetrack-pseudo6dof-semantic.json
taoryx vehicle run generated/a320-racetrack-pseudo6dof-semantic.json \
  --output-dir generated/a320-racetrack-pseudo6dof-execution

taoryx vehicle compose \
  examples/vehicle_composition/f16_racetrack_capability_3dof_compose.yaml \
  --output generated/f16-racetrack-3dof-semantic.json
taoryx vehicle run generated/f16-racetrack-3dof-semantic.json \
  --output-dir generated/f16-racetrack-3dof-execution
```

At present these adapters support only their declared point-mass 3DOF and
named pseudo-6DOF profiles. The A320 pseudo profile includes a response-law
and policy-surface diagnostics; the F-16 pseudo profile is a source-local
attitude/rate bridge. Neither result is a direct-wrench, physical-surface,
actuator, or full-controller-equivalence claim.

## Open a composition episode

The composition layer also has a narrow interactive projection. It does not
introduce a second numerical kernel: the language-backed airbreathers reuse
`InteractiveSession`, and the Hummingbird pseudo witness calls its existing
bounded aggregate-thrust model at fixed accepted substeps.

```python
from pathlib import Path

from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        Path("examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
)
episode = open_vehicle_composition_episode(composition)
observation = episode.reset()
transition = episode.step({"throttle": 0.60}, duration_s=0.10)
episode.save_checkpoint("generated/x8-racetrack.checkpoint.json")
episode.close()
```

Every episode reports its action and observation schema, exposes only the
selected fidelity's declared controls, and returns truth committed at the end
of the requested external duration. An adaptive integrator may take smaller
inner steps, but it may not silently shorten `step(..., duration_s)`; it
continues to the requested boundary or a declared terminal boundary. A
checkpoint is bound to the immutable composition fingerprint and restores
into a newly built executable callback graph.

For a deterministic policy or RL-style action stream, retain the public
`CompositionPolicyTrace` with the compiled composition rather than a native
control history. `write_composition_policy_trace` persists that trace, and the
public command reopens the exact composition and compares every newly produced
public committed-boundary frame against the saved artifact:

```bash
taoryx vehicle replay-policy \
  generated/x8-racetrack-semantic.json \
  generated/x8-policy-trace.json \
  --output generated/x8-policy-replay.json
```

The replay rejects a changed composition fingerprint, interface fingerprint,
semantic action mapping, accepted-boundary result, or final public frame. It
is a same-episode-kernel replay check, not evidence that a separately compiled
batch mission, a physical effector model, or a robustness suite agrees with
the trace.

The initial episode witnesses are X8/B747 language-backed racetracks and the
Hummingbird named pseudo-6DOF hover/yaw composition:

```python
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        Path("examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )
)
episode = open_vehicle_composition_episode(composition, seed=7)
transition = episode.step(
    {"roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 1.57, "thrust_ratio": 0.10},
    duration_s=0.10,
)
```

The Hummingbird episode is explicitly an aggregate-thrust-vector response
law: it does not claim individual rotor allocation. A320, F-16, and other
families return an explicit unavailable-adapter error until their episode
binding is implemented; they do not borrow the X8 or Hummingbird runtime.

The same Hummingbird pseudo-6DOF composition also has a source-owned nominal
batch path. It translates the declared hover, yaw, translation, contact, and
post-shutdown settle segments into the existing aggregate-thrust response
model, then writes the standard composition, preflight, plan, truth,
objective, envelope, and transition artifacts:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml \
  --output generated/hummingbird-pseudo-semantic.json
taoryx vehicle run generated/hummingbird-pseudo-semantic.json \
  --output-dir generated/hummingbird-pseudo-execution
```

This is an executable nominal pseudo-6DOF witness, not a rotor-resolved
showcase or a promotion to individual-motor allocation.

The NESC two-stage rocket has an equally explicit but different batch binding:
the retained source translation is replayed only when the composed launch,
staging delay, and terminal kind match the pinned witness.  The pseudo-6DOF
case adds its named scheduled attitude response; neither path turns source
history into an active guidance, thrust-vector, or stage-separation model.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml \
  --output generated/nesc-source-replay-semantic.json
taoryx vehicle preflight generated/nesc-source-replay-semantic.json
taoryx vehicle run generated/nesc-source-replay-semantic.json \
  --output-dir generated/nesc-source-replay-execution
```

The resulting artifact contains the source-replay provenance, stage event
times, independently evaluated stage/cutoff/terminal objectives, and data-
integrity envelope. It is not a substitute for a participating rocket runtime
or a future vehicle-control episode.

The X-15 registry deliberately exposes two different mission contracts. The
existing `rocket_aircraft_high_energy_v1` remains the planned local
direct-wrench X-15 bridge. The runnable
`x15_staged_booster_reachability_v1` is a different, local-frame, source-pinned
reduced witness at 3DOF and pseudo-6DOF:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml \
  --output generated/x15-staged-reachability-semantic.json
taoryx vehicle preflight generated/x15-staged-reachability-semantic.json
taoryx vehicle run generated/x15-staged-reachability-semantic.json \
  --output-dir generated/x15-staged-reachability-execution
```

The latter pins the retained X-15 source staging mass, speed magnitude,
cutoff, and release timing; it then evaluates booster deployment, the
high-energy corridor, a declared open-loop atmospheric handoff witness, and
passive impact. Its `open_loop` or `response_law` control realization is
intentional: it exposes no external action or physical-effector authority.
It is not a native X-15 rigid-body mission, a controlled terminal handoff, or
the separate synthetic California-to-Hawaii showcase.

The X-15 also exposes a deliberately narrower rigid-body direct-wrench
endpoint, `x15_local_direct_wrench_screen_v1`. It executes the retained local
source-plant LQR recovery screen from one source release/glide point. The
artifact records the local state and requested-versus-achieved force/moment
truth, residual, and saturation disposition at committed rows:

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml \
  --output generated/x15-local-direct-wrench-screen-semantic.json
taoryx vehicle preflight generated/x15-local-direct-wrench-screen-semantic.json
taoryx vehicle run generated/x15-local-direct-wrench-screen-semantic.json \
  --output-dir generated/x15-local-direct-wrench-screen-execution
```

Its passing result is `screen_pass`, not `mission_pass`. It establishes only
that the bounded local source-linearized screen recovers its declared local
perturbation. It has no runnable external action/episode binding (the generic
direct-wrench profile remains planned) and does not establish trim, carrier
release, ignition, propulsion, navigation, handoff, physical effectors, or
X-15 family qualification.

The passive tumbling family has a separate direct-release path. It intentionally
has no controller or allocation profile: 3DOF uses a declared orientation-
averaged projected area, and pseudo-6DOF explicitly reuses the native
rigid-body passive-tumble equations. Both requests pin the current engineering
cylinder fixture, its release state, and its body rates; a different geometry
or release condition is a future bounded-variant request, not a silent model
morph.

```bash
taoryx vehicle compose \
  examples/vehicle_composition/tumbling_body_direct_release_3dof_compose.yaml \
  --output generated/tumbling-3dof-semantic.json
taoryx vehicle run generated/tumbling-3dof-semantic.json \
  --output-dir generated/tumbling-3dof-execution

taoryx vehicle compose \
  examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml \
  --output generated/tumbling-pseudo-semantic.json
taoryx vehicle run generated/tumbling-pseudo-semantic.json \
  --output-dir generated/tumbling-pseudo-execution
```

The 3DOF artifact must not be used as a tumble proof. The pseudo artifact can
prove only the declared native-rigid passive rotation of this fixture; it does
not provide a pseudo response law, a wrench command, an effector allocation,
or a source-specific spent-stage claim.

## Composition model

A trajectory has one initialization contract and an ordered segment graph:

```text
vehicle + fidelity
    ↓
initialization contract and parameters
    ↓
mission template
    ↓
ordered segment instances with target parameters
    ↓
compiled scenario and independent evaluation
```

An initialization contract describes the one-time state setup, such as
`airborne_trim`, `grounded_idle`, `air_launch_release`, `high_altitude_release`,
or `pad_launch`. It declares the required fields and their canonical units.

A segment contract describes semantic intent rather than an implementation
shortcut. It declares compatible fidelities, user parameters, requested control
intents, and permitted physical transition events. Examples include
`climb_level_gate`, `fly_by_turn`, `hover_dwell`, `stage_separation`, and
`atmospheric_handoff`.

State is continuous by default across segments: position, velocity, attitude,
rates, mass, and resources carry forward. Only a named physical transition
event—such as release, stage separation, contact, or motor shutdown—may alter
that rule, and it must be recorded in the run artifact.

## Authority boundaries

The composition registry is a projection of existing authorities:

| Concern | Authority |
| --- | --- |
| Family identity, physical family, adapter, four fidelity slots, blockers | `verification/horizontal_fidelity_registry.yaml` |
| Profile control realization and evidence boundary | `verification/pseudo6dof_profiles.yaml` |
| Native source data and local plant details | `verification/vehicle_models.yaml` or `families/*/family.yaml` |
| Existing qualification scenario details and truth objectives | `verification/family_qualification_missions.yaml` |
| Reusable powered-fixed-wing geometry | `verification/racetrack_templates.yaml` |
| User-visible initialization, segments, and mission recipes | `verification/vehicle_composition_registry.yaml` |

The registry may advertise a **planned** or **development** mission template;
that is discoverability, not a promotion. Runtime selection must still apply
the existing fail-closed fidelity-lowering and qualification checks.
