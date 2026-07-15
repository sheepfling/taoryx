# Interactive stepping engine

`taoryx.runtime.interactive.InteractiveSession` provides an externally driven
execution boundary over the existing runtime state, derivative, integrator,
event, and telemetry contracts.

This is the live-flight counterpart to batch execution, not a separate physics
engine. The same lowered `RuntimeProblem` can be run open-loop for a batch
study or handed to a human player, controller, or learning agent that supplies
commands at accepted time boundaries. A caller can pause, inspect state,
replace the command policy, resume, and branch with `RuntimeProblem.clone_at()`.

Scenario composition uses `ScenarioRuntimeContract` as the serializable
declaration boundary. `ResolvedScenario.interactive_session()` projects its
typed controls, statuses, and output subscriptions into `InteractiveSession`,
while batch `RunArtifact` metadata retains the same declaration. Python
callbacks remain live runtime objects and are never placed in the cache.

For source-oriented workflows, `taoryx.runtime.program.LoadedProgram` is the
program/emulator boundary. `LoadedProgram.inspect()` exposes the parsed
problem, trajectory/segment structure, table inventory, controls, and lowered
state names before stepping. `case()`, `copy_case()`, and `clone_case_at()`
provide the corresponding executable graph views. `inspect_case()` exposes live
state and history, while `set_control()` and `set_parameter()` apply bounded
runtime mutations without changing the source document.

```python
session = InteractiveSession(
    problem,
    controls=(
        ControlSpec("fin_pitch", unit="normalized", lower=-1, upper=1),
        ControlSpec("throttle", unit="fraction", lower=0, upper=1),
    ),
)
snapshot = session.step(0.02, {"fin_pitch": 0.1, "throttle": 0.7})
```

## Command semantics

Commands are named and bounded. Optional slew limits are applied at each
accepted step. The requested value and the applied value are both recorded.
Commands are exposed to the derivative through the state’s named-value view,
or can be transformed into actuator values with a `control_model` callback.

The callback is the correct place to model fin gearing, throttle limits,
actuator dynamics, or controller mappings. Interactive commands do not directly
overwrite position, velocity, attitude, mass, or other physical state.

## Lifecycle

```text
created → running ↔ paused → running
                 ├→ interrupted
                 ├→ completed
                 └→ failed
```

`step(duration, commands)` advances an explicit numerical duration. It does
not read a wall clock. `pause`, `resume`, and `interrupt` affect session
lifecycle only; a caller or player owns the external timing loop.

## Replay and artifacts

Each accepted step records a `ReplayFrame` containing the requested command
stream and duration. Replaying those frames re-applies bounds and slew limits,
which tests the command semantics rather than merely copying final values.

`InteractiveSnapshot` is a human-readable step view. `InteractiveArtifact`
serializes the session directly, while `InteractiveSession.to_run_artifact()`
projects the accumulated state history into the same `RunArtifact` telemetry
contract used by batch execution and visualization.

The shared observation contract also includes:

- `StatusSpec` for named, unit-labeled model status channels;
- `EventSpec` and `EventAction` for deterministic `stop`, `transition`, and
  `signal` boundaries;
- `RuntimeEvent` records with accepted time, vehicle, action, and signal name;
- `OutputSubscription` for renderer-neutral channel and event requests.

Interactive commands, structured runtime events, and output subscriptions are
included in the projected `RunArtifact`. A signal is recorded once at the
accepted numerical boundary and does not implicitly stop or mutate the model.

## Checkpoint and restart

Use `LoadedProgram.save_checkpoint(path)` to pause a point-mass emulator and
persist its source documents, case metadata, controls, events, state, and
history. `LoadedProgram.load_checkpoint(path)` rebuilds the executable model
from those documents and restores the saved state:

```python
program.save_checkpoint("flight.checkpoint.json")
resumed = LoadedProgram.load_checkpoint("flight.checkpoint.json")
resumed.case()  # continue with the same runtime model
```

The checkpoint includes integrator/timing configuration, vehicle execution
settings, source/table SHA-256 fingerprints, schema versioning, and is written
atomically. It deliberately serializes the program and state, not Python
callback closures. Kinematic sidecar and interactive-session checkpoint
support remain planned extensions; the current checkpoint contract rejects
kinematic sidecars explicitly.

The same `event` declarations are lowered into the batch `EventCondition`
engine. Batch `stop` and `signal` rules therefore use the existing event
crossing/refinement and artifact recording path; `transition` remains an
explicit model adapter concern.

## Tiered observations

Callers do not need to inspect `RuntimeState.named` for ordinary flight data.
`RuntimeProblem.observe()` and `LoadedProgram.observe()` expose three levels:

1. `standard`: a compact `StandardRuntimeOutput` containing time, ECFC
   position/velocity/acceleration, optional attitude quaternion/body rate,
   mass, and current segment;
2. `status`: declared model-specific values such as thermal state, throttle,
   stage, mode, or health flags;
3. `deep`: the complete named/internal state, only when explicitly requested.

```python
observation = program.observe(vehicle="1")
observation.standard.position_ecfc
observation.status
deep = program.observe(vehicle="1", include_deep=True).deep
```

The standard tier is intended for controllers, players, telemetry loops, and
UIs. The status tier is the stable model-owned inspection surface. The deep
tier is for diagnostics, debugging, and research tooling rather than the
default runtime payload.

## Scope

The session supports generic derivative models and existing point-mass,
kinematic 3+3, and rigid-body 6-DOF runtime records. It does not claim that a
3-DOF model has rotational dynamics, and it does not establish historical TAOS
96.0 behavior.
