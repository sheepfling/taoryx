# Checkpoint and restart goal

Checkpoint/restart is a primary TAORYX product capability. A user must be
able to pause a loaded problem-file program, inspect its source and emulator
state, persist it, terminate the process, reload it later, and continue from
the same accepted numerical boundary.

This is a TAORYX successor capability, not a claim about historical TAOS 96.0
behavior.

## Contract

The checkpoint must preserve or explicitly reject:

- source problem and table documents;
- grammar/profile and schema versions;
- source/table integrity fingerprints;
- selected case and parameters;
- current state and retained history;
- active segment, event state, and vehicle dependencies;
- control values and runtime mutations;
- integrator, tolerance, step-size, and timing-boundary settings;
- model-specific sidecar state, or a clear unsupported-state diagnostic.

Executable callbacks are rebuilt from source. Raw Python closure pickling is not
the persistence contract.

## Acceptance tests

The release gate must retain tests for:

- exact point-mass save/load/restart equivalence;
- restart at a segment transition;
- restart immediately before and after an event;
- modified controls surviving restart;
- source/table tampering rejection;
- atomic checkpoint replacement;
- batch and interactive replay equivalence;
- multi-vehicle and dependency-state restoration;
- kinematic 3+3 sidecar restoration;
- rigid-body attitude, inertia, controller, and actuator restoration;
- interactive session command-history restoration.

Current implementation covers point-mass and rigid-body source reconstruction,
runtime configuration, integrity verification, atomic writes, cloning, and
restart tests in `tests/unit/test_runtime_branching.py`. Kinematic 3+3 attitude
sidecars are serialized as explicit position/velocity/quaternion data and are
restored with their parent runtime vehicle. Interactive sessions expose the
same boundary through `InteractiveSession.save_checkpoint()` and
`InteractiveSession.load_checkpoint()`: the caller supplies the executable
problem and callback/controller factory, while the checkpoint restores the
runtime graph, sidecar state, controls, snapshots, events, and command history.
Python callbacks are never pickled. A model fingerprint may be supplied to bind
the checkpoint to a resolved case.

## Completion gate

This goal is complete only when a checkpoint/resume run produces the same
accepted state, event sequence, control history, and terminal artifact as an
uninterrupted run within the documented numerical tolerance for every supported
dynamics mode. The source-program and interactive checkpoint tests are the
executable evidence for that claim; event/controller callback rebinding is
explicit at the load boundary rather than inferred from serialized Python.
