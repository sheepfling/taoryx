# Transition truth: language and runtime specification

This document defines how TAORYX exposes truth at `.prb` event and segment
transitions. It is a successor-runtime contract; it does not change the
historical TAOS 96 grammar.

## Language boundary

Authors use the existing transition constructs:

```text
*segment 1 powered
  *when tseg>2 goto 2
*segment 2 coast
  *increment mass=-5
  *when tseg>10 stop
```

`goto` and `stop` select the event action. `reset` and `increment` describe an
explicit state change applied when the destination segment is entered. The
language does not contain a `record-truth` switch: transition truth capture is
mandatory for every lowered runtime event.

## Runtime sequence

For an event at accepted time `t`:

1. The scheduler accepts the boundary at `t`.
2. The runtime copies the pre-transition state and achieved controls.
3. The event handler applies the segment action, reset, increment, or stop.
4. The runtime refreshes derived channels and active-segment metadata.
5. The runtime copies the post-transition state and achieved controls.
6. The pair is appended atomically to `RuntimeProblem.transition_history`.
7. The backward-compatible `event_history` entry is appended with the same
   pair under `pre_truth` and `post_truth`.

The pre and post snapshots share the event timestamp. They may differ in
segment number, active status, named channels, or physical values. No RK4
stage, rejected adaptive trial, or post-hoc interpolation may appear in a
transition pair.

## Typed record

`TransitionTruthPair` contains:

| Field | Meaning |
| --- | --- |
| `event_name`, `signal`, `action` | Source event identity and action |
| `event_time` | Accepted transition timestamp in seconds |
| `segment_from`, `segment_to` | Source and resulting segment numbers |
| `pre`, `post` | Immutable `TransitionTruthSnapshot` values |
| `residual` | Event residual at the refined crossing |
| `source` | Optional source-file location |
| `state_discontinuity` | Whether physical state values changed beyond tolerance |

Each snapshot contains state values, named channels, value names, frame,
segment number, active status, and sorted achieved-control values. These are
the achieved values, not merely commanded values.

## Continuity policy

A plain segment handoff should preserve physical state and report
`state_discontinuity=false`. A reset, increment, staging impulse, jettison, or
other physical discontinuity must be represented by source syntax and will
report `state_discontinuity=true`. Consumers should reject an undeclared jump,
not smooth it away.

Sensors and controllers observe committed truth only. An instantaneous sensor
may sample the boundary; an interval sensor may consume the accepted segment.
Neither may sample between the pre and post snapshots or interpolate across a
transition.

## Inspection and artifacts

The records are available through:

```python
case.transition_history
program.inspect_case()["transition_history"]
problem.event_history[index]["pre_truth"]
problem.event_history[index]["post_truth"]
```

Checkpoints serialize and restore the typed transition history. Run artifacts
retain the legacy event list so existing consumers continue to work while new
consumers can follow `transition_index` to the richer record.

## Required verification

The implementation must test at least:

- a continuous `goto` with identical physical pre/post values;
- a `stop` with post-state inactive;
- a reset or increment with a detected discontinuity;
- achieved controls retained on both sides;
- checkpoint round-trip preservation;
- no transition records for rejected solver stages.

The executable example is
[`examples/composition/staged_signals.py`](../../examples/composition/staged_signals.py),
and the source syntax fixture is
[`examples/taoryx/syntax_fragments/12_transition_truth.prb`](../../examples/taoryx/syntax_fragments/12_transition_truth.prb).
