# EOM timing and committed-truth contract

## Decision

Taoryx uses accepted integration boundaries as the only physical truth
boundaries. At the beginning of a step, the runtime has a committed state and
must publish the corresponding truth snapshot before a controller or sensor
can produce the command for the next interval.

The canonical sequence is:

```text
t_k committed state and achieved actuators
        |
        +--> evaluate current truth/load snapshot
        |
        +--> sample instantaneous sensors
        |
        +--> deliver due measurements and update estimators
        |
        +--> compute next command
        |
        +--> latch command/actuator transition for [t_k, t_{k+1}]
        |
        +--> integrate EOM through private solver stages
        |
        +--> accept and atomically commit t_{k+1}
```

This makes the IMU causal. A measurement at `t_k` cannot see a command that
was generated from that same measurement, and it cannot see a solver stage
that has not yet been accepted as physical state.

The machine-readable contract is
[`verification/eom_sensor_timing_contract.yaml`](../../verification/eom_sensor_timing_contract.yaml).

## Selecting the next accepted boundary

The runtime chooses the next step from the minimum of the active model cadence
and every required boundary after the current time:

```text
dt_next = min(
    active_model_step,
    next_truth_timestamp - t,
    next_sensor_timestamp - t,
    next_output_timestamp - t,
    next_table_knot - t,
    next_event_or_final_timestamp - t,
)
```

The current runtime implements the model/output/table/final-time part and the
successor `SensorClockSpec` boundary provider. Per-segment `*Integ dt=...`
values are resolved when a `*When ... goto` transition commits the target
segment, so a coarse coast phase can hand off to a finer terminal phase without
changing the historical language shape. A `*runtime sensor ...` declaration
registers a clock that participates directly in the minimum-boundary decision;
its measurement provider is still a separate Alpha 3 component. The explicit
`RuntimeProblem.required_truth_times` stream remains available for externally
scheduled truth consumers. Neither path permits post-step sensor interpolation.

The declaration's `kind` is a provider-neutral family, not a plug-in ID. A
same-named sensor sidecar selects the concrete provider and its error model;
the provider manifest must advertise compatibility with the declared family.
The language clock remains authoritative when both are present. See the
[sensor plug-in API](sensor-plugin-api.md) for the complete ownership rule.

## What is committed

At every accepted boundary, the runtime must commit one coherent truth point:

- time;
- position and velocity;
- attitude and body rates where applicable;
- acceleration and angular acceleration;
- gravity and environment values;
- force and moment decomposition;
- achieved actuator state;
- mass, CG, inertia, and resource state;
- active mode, segment, and event status.

These values share the same timestamp, frame, units, and accepted-step status.
The force snapshot must record which achieved actuator state and environment
were used. A derivative callback evaluated on demand is not a substitute for a
committed truth snapshot because it may be recalculated with different
controls, tables, or stage metadata.

## Segment-transition truth

Segment and event transitions are runtime boundaries, not optional reporting
features. The runtime records a `TransitionTruthPair` for every applied event:

- `pre` is the accepted truth at the event time before the handler runs;
- the event handler applies the declared reset, increment, or segment change;
- `post` is the truth after the handler, including the new segment and
  achieved actuator values;
- both snapshots retain the same event timestamp and are immutable copies.

The legacy `event_history` remains available for compatibility and contains
the same pair under `pre_truth` and `post_truth`. Consumers that need to audit
state continuity should use `transition_history` and inspect its explicit
`state_discontinuity` flag. A discontinuity is valid only when the event
declares a physical reset, impulse, staging, or other state-changing action.
The `.prb` language declares the transition behavior; it does not opt out of
this runtime truth record.

## What is not physical truth

RK4 stages, adaptive trial states, finite-difference probes, controller
predictions, and rejected solver steps are computational artifacts. They may be
used internally by the EOM, but they must not be exposed to IMUs, cameras,
radar, estimators, controllers, or telemetry as if they were measurements at a
physical time.

`RuntimeState.interpolated(...)` remains useful for replay and branch tooling.
It is not a permitted sensor source.

## IMU timing

An instantaneous IMU model samples the committed truth point at its timestamp.
An interval IMU model consumes accepted truth over a closed interval and
computes delta angle, delta velocity, coning, sculling, bias, and noise through
its own sensor integration rules.

If the IMU rate is faster than the EOM step, the scheduler must split or
substep the EOM at the IMU timestamp. If the IMU rate is slower, the scheduler
accumulates accepted truth segments until the delivery boundary. It must never
linearly interpolate two published vehicle states to manufacture a sensor
measurement.

The first IMU sample has an explicit policy: either an initialized history is
provided or the measurement is marked invalid. There is no implicit
zero-duration interval.

## Current implementation status

The current runtime has the accepted-truth shape:

- `InteractiveSession.step` gives controllers the state at the beginning of
  the step before calling the EOM;
- `integrate_active_vehicles` commits `vehicle.state` only after the selected
  integrator accepts the step;
- solver-stage states are not appended to normal vehicle history.

The runtime now records immutable pre/post transition truth pairs in addition
to accepted vehicle history. The clock contract constrains accepted boundaries
and exposes timing metadata. `RuntimeVehicle.truth_provider` projects each
accepted state into a `TruthPoint`, and `SensorBus` owns model calls, interval
accumulation, delivery latency, validity, drops, and estimator subscribers in
both batch and interactive execution. Sensor models never receive solver-stage
states or post-step interpolations. RKF45 regression cases now force rejected
trial steps for both instantaneous and interval IMUs, then verify that the
measurement and interval endpoints remain accepted history boundaries in both
batch and interactive paths. Every runtime environment and right-hand-side
load evaluation now records its evaluated state timestamp, the accepted
boundary at which its achieved controls became active, and one of four phases:
`solver_stage_environment`, `solver_stage_rhs`,
`committed_truth_environment`, or `committed_truth_rhs`. Solver-stage records
remain computation provenance only; they are not emitted as accepted truth or
sensor input. Declared `SensorScenarioSpec` checkpoints now reconstruct their
registered provider, queued/delivered latency packets, interval baseline,
and packet-only estimator state in both loaded-program and interactive flows.
Custom callback-only sensor/estimator integrations remain deliberately
non-checkpointable rather than being silently reattached with different
behavior. Remaining gaps are richer actuator/support-force truth and named
checkpoint factories for additional physical sensor/estimator families.

## Required Alpha 3 implementation gates

1. [x] Preserve immutable committed truth points at every accepted boundary
   and both sides of every segment transition.
2. [x] Return accepted truth segments to the sensor bus.
3. [x] Tag all runtime load evaluations with state time, achieved-control time, and
   step phase.
4. [x] Add a scheduler that creates explicit sensor boundaries.
5. [x] Implement IMU ideal/error-model measurements before navigation use.
6. [x] Test instantaneous and interval IMUs across adaptive and rejected
   steps in batch and interactive execution.
7. [x] Prove batch and stepped sensor runs use identical accepted truth.

The acceptance condition is not merely that an IMU plot looks smooth. It is
that every measurement can be traced to a committed truth boundary or an
accepted truth segment, with no solver-stage or post-hoc interpolation path.
