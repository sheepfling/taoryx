# CADAC ADS6 package composition

`cadac.ads6.engagement` exposes one installed ADS6 SAM/target/RADAR0 source
case as an exact Taoryx Mission Composition batch model and persistent session.

## Root objects and fidelity

The result contains independent roots in source order:

```text
MISSILE6   T4 physical fins/TVC or T3 aggregate RCS
AIRCRAFT3  T1 point-mass target, or
ROCKET5    T2 pseudo-6DoF SRBM
RADAR0     static sensor/site
```

No participant is represented as a spawned child. Rigid-body channels are null for target and radar roots rather than fabricated.

## Scheduling semantics

The executor retains persistent actor state and runs the `VEHICLES` list sequentially. Each actor refreshes its packet after its own pass. A radar placed after the missiles and targets can observe their current-epoch packets, while an earlier SAM consumes the retained radar and target packets from the previous epoch.

Target pairing is exact and positional:

```text
m1 → a1/r1
m2 → a2/r2
m3 → a3/r3
```

The package follows the documented latched RADAR0 schedule. It does not reproduce an apparent unconditional launch-delay overwrite in the shipped executive. The result diagnostic records:

```text
launch_schedule_semantics = documented_radar_latched_next_epoch
```

## RADAR0 modes

Aircraft defense uses source-cadence target measurements, independent lethal-range launch latches, and measured target coordinates as the current intercept-point uplink.

SRBM defense detects apogee, combines `SRBM_DECK` descent prediction with `SAM_DECK` ascent timing, applies independent launch biases, and refines the intercept point from measured target and launched-SAM altitude errors.

## Source SAM controller

The default command law is:

```text
source_controller
```

It interleaves a persistent SAM controller at the source module boundaries:

```text
truth-aligned INS
  → deterministic RF/IR seeker
  → radar-IP line guidance
  → terminal proportional navigation
  → adaptive rate/acceleration autopilot
  → physical SAM effectors
```

Parsed source events are evaluated before each SAM module pass. Controller commands produced at `control` are consumed by the later physical-effector modules in the same pass. Seeker locks, source events, and control/guidance/sensor mode transitions are returned as actor-qualified Mission Composition events.

The legacy `hold` and `line_of_sight` command laws remain available only as plant/scheduler comparison seams.

## Configuration

The installed source case fixes:

- target family;
- source actor order and count;
- SAM physical realization;
- radar mode and resources;
- source controller module order and events.

Runtime configuration exposes:

- `command_law`: `source_controller`, `hold`, or `line_of_sight`;
- legacy direct-command gain and limit;
- deterministic radar seed;
- end time and sample cadence.

## Output

The standard output schema labels the following primary-SAM controller and
physical-response channels explicitly (rather than leaving them only in an
untyped telemetry map):

- `requested_control_deg` and `achieved_control_deg`;
- `requested_fins_deg` and `achieved_fins_deg` in source fin order;
- `normal_command_g` and `lateral_command_g`;
- `achieved_lateral_acceleration_g` and `achieved_normal_acceleration_g`.

The batch result additionally returns:

- actor-specific trajectories and telemetry;
- radar tracks and launch commands;
- launch, phase, controller-event, mode-transition, seeker-lock, and termination events;
- packet observation/publication traces;
- controller modes, target packet epoch, target range, LOS rates, pointing states, guidance commands, and total body wrench in the actor telemetry map;
- source provenance and a narrow claim boundary.

## Exact dispatch

```python
registry.register(
    "cadac",
    "cadac.ads6.engagement",
    provider.execute_batch,
)
```

No actor-level model, alternate target family, or neighboring fidelity can substitute for the exact installed package.

## Persistent-session API

The package owns a native `step` lifecycle. Opening a session checkpoints and
resetting it reconstructs the complete vehicle-major scheduler: SAM
plant/controller, target runtime, RADAR0 stochastic and track-manager state,
launch latches, source packets, event cursor, and actor lifecycle state. A
hold must be an integral multiple of the source `int_step`; caller actions are
empty because the source controller remains package-owned.

At the initial state and after every committed package epoch, each exact
SAM/target pair emits a native `relative-state-track` packet with ID
`ads6-engagement-<sam-id>-native-relative-state`. These are standard Taoryx
raw geometry packets with normal SensorBus sequence/delivery accounting. They
do not feed back into, replace, or re-time the source RF/IR seeker or RADAR0
paths.

Session observations label the primary SAM's requested and achieved controls
and fins, command and achieved acceleration evidence, package controller
state, source events, and latest delivered native tracks. This supports
finite-run controller analysis and like-for-like trace comparison. In
particular, `cadac_controller_trace_from_samples` can build a trace directly
from a primary-SAM batch object's standard `normal_command_g` and
`achieved_normal_acceleration_g` samples. A comparison still needs an
identical time/reference trace; local stability and frequency-margin claims
remain blocked without a declared trim, linearization coordinate set, and
complete closed-loop state model. The 251-epoch responsiveness guard is
`tests/families/cadac/test_ads6_engagement_performance.py`.

## Evidence boundary

Synthetic source-shaped regressions cover package scheduling, the complete
one-aircraft source-controller transition sequence, session reset, standard
output advertisement, native packet delivery, and interactive persistence.
Complete RF noise/glint parity, complete IR focal-plane parity, source INS
error-state propagation, exact stochastic sequences, and compiled-CADAC
numerical equivalence remain open gates.
