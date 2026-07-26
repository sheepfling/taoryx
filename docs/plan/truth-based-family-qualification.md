# Truth-Based Family Qualification

Status: active implementation plan; Hummingbird and X8 nominal packets regenerated with qualification gaps visible

The existing four-family composites are retained as `integration evidence`. They
must not receive a family-qualification badge until required mission objectives
are verified independently from truth telemetry.

## Four facts that must remain separate

1. **Mission objective** — the physical result the scenario requires.
2. **Guidance reference** — the point, path, gate, or steering target supplied
   to the controller.
3. **Controller transition** — the controller's internal decision to advance,
   skip, abort, or terminate.
4. **Independent truth result** — a result computed from the recorded truth
   state, without trusting the controller's capture flag.

A controller transition is diagnostic evidence. It cannot create a physical
objective pass.

## Objective vocabulary

The evaluator supports these explicit objective types:

| Type | Physical meaning |
| --- | --- |
| `fly_over` | Truth position enters the declared region. |
| `dwell` | State remains in the region for the declared duration. |
| `fly_by_gate` | Truth crosses an oriented gate within the declared corridor. |
| `path_corridor` | The complete sampled path remains inside bounds. |
| `loiter` | Truth remains in the declared loiter region for a duration. |
| `event` | A declared physical event occurs. |
| `energy_corridor` | Truth enters the declared multidimensional energy/state set. |
| `terminal_state_gate` | Position, velocity, attitude, and rates meet the terminal set. |
| `touchdown` | Contact and post-contact state satisfy the landing contract. |

Transitions are classified as `CAPTURED`, `GATE_CROSSED`,
`DWELL_COMPLETE`, `EVENT_COMPLETE`, `TIMEOUT_SKIP`, `FORCED_ADVANCE`,
`ABORT`, or `NUMERICAL_TERMINATION`. A required objective reached by a
timeout, forced advance, abort, or numerical termination is a failure.

## Hard mission result

The mission result is a conjunction, not a scalar score:

```text
mission_pass =
    all required truth objectives passed
    and all required transitions are valid
    and the terminal contract passed
    and hard envelope limits passed
    and numerical checks passed
```

Quality scores remain useful for ranking a passing or failing trajectory, but
they cannot compensate for a missed required objective.

## Qualification order

The four current families are requalified in this order:

1. Hummingbird: pad-to-pad capture, dwell, disturbance recovery, touchdown.
2. X8: physically feasible fly-by gates or declared fly-over corridors,
   bilateral turns, and terminal closure.
3. B747: climb, cruise, opposing large-radius turns, descent, and a
   multidimensional arrival gate.
4. X-15: release, burn, cutoff, coast/apogee, descent, energy management, and
   a declared approach or handoff terminal gate.

The reusable strict probe remains a negative-control baseline. The dedicated
Hummingbird packet now evaluates a true rectangular corner-dwell sequence,
altitude gates, and a yaw step. The touchdown and post-touchdown-settle
objectives fail because the current fixture has no contact reaction model and
motor shutdown precedes geometric ground crossing; the packet reports the
critical altitude error, post-ground maximum speed, and unavailable contact
state directly on the board. It is therefore a **nominal case pass — terminal
contact pending** case, not a landing qualification.

The X8 figure-eight packet independently evaluates two oriented vertical gate
planes with lateral, altitude, and speed requirements, plus truth-observable
positive/negative bank and pitch events. Both gate planes pass; the terminal
state gate fails (the best terminal north error is about 362 m against a 100 m
limit), and the diagnostic controller transitions precede several truth
crossings. Its bank-command/achieved response and flat elevon channels remain
diagnostic rather than physical-control qualification evidence. It is
therefore a **nominal case pass — overall qualification pending** regression
case, not a route or control qualification.

Current status:

| Family | Nominal truth objectives | Status |
| --- | ---: | --- |
| Hummingbird | 11 / 13 | nominal case pass — terminal contact pending |
| Skywalker X8 | 6 / 7 | nominal case pass — overall qualification pending |
| B747 | not rerun under strict evaluator | pending |
| X-15 | not rerun under strict evaluator | pending |

## Showcase packet layers

The public composite is the front page of a qualification packet, not the
qualification itself. Every family should eventually have three complementary
showcase kinds:

1. **Capability showcase** — one focused maneuver that proves a specific
   authority, response, or actuator behavior.
2. **Characteristic mission showcase** — the complete family-appropriate
   lifecycle, with ordered objectives, transitions, and terminal state.
3. **Envelope showcase** — interior, boundary, and declared-outside witnesses
   showing where the model is qualified, extended, or rejected.

The characteristic-mission packet should be organized into these pages:

* **Mission Summary** — full geometry, mission purpose, start and terminal
  contracts, objective-status strip, gate/waypoint geometry, and explicit
  failure cause.
* **Dynamics & Controls** — fidelity-specific states, commands versus achieved
  response, effectors, limits, saturation, and the exact physical nonclaims.
* **Numerical & Robustness** — convergence, batch/step parity, perturbations,
  failure-code distribution, and retained representative failures.
* **Cross-Fidelity** — semantic-mission mapping, event-time deltas, terminal
  deltas, and explained disagreement between 3DOF, named pseudo-6DOF, and
  rigid-body 6DOF.
* **Evidence & Claim Sheet** — provenance, qualified envelope, source class,
  exact claim, exact nonclaims, artifact hashes, and reproduction command.

Every Mission Summary board must include a short context block stating the
mission purpose and expected difficulty, a visible objective-status strip, and
a Claim/Nonclaim box. A controller transition is shown as a diagnostic event;
the truth evaluator's result remains the only evidence of physical objective
completion.

## Mission sequence view

In addition to plots, each packet should generate a sequence-diagram-style page
from the same `segment_timeline.json` and `events.json` sources:

```text
start contract
    -> stabilization
    -> objective 1 / truth result
    -> mode or guidance change
    -> objective 2 / truth result
    -> terminal contract
    -> post-terminal settle or classified failure
```

Each node displays the controller transition time and reason beside the
independent truth completion time and result. This makes a timeout, forced
advance, skipped gate, or terminal failure obvious without asking a reviewer
to infer it from a time-series plot.

## Packet contract

Each qualification packet must include:

* the exact claim and nonclaims;
* resolved mission and input tables;
* truth telemetry;
* controller transitions and reasons;
* per-objective truth result, closest error, tolerance, and margin;
* terminal-state results;
* envelope, numerical, and replay evidence;
* a reproduction command and hashes.

The renderer should show the declared objective geometry, actual trajectory,
transition reason, and truth pass/fail beside the trajectory. A final aggregate
score is never sufficient evidence of route completion.
