# Scored fidelity and mission-evidence plan

## Purpose

Produce trajectories that are both physically meaningful and reviewable. A
finite integration or a green pytest result is not enough: every trajectory
must have a declared vehicle, fidelity tier, objective set, units, tolerance
policy, termination policy, and machine-readable score.

The workflow is designed for B747, Skywalker X8, Hummingbird, X-15, point-mass
vehicles, and future vehicle families without adding a bespoke runner or
problem-file dialect.

## Fidelity ladder

Each family is exercised progressively at three tiers:

1. **True 3-DOF** — translational point-mass propagation with prescribed or
   solved aerodynamic angles, bank, thrust, atmosphere, wind, and the shared
   vehicle force evaluator.
2. **Pseudo-6-DOF bridge** — the same translational plant plus prescribed,
   scheduled, or lagged attitude/control reconstruction. It is an
   intermediate development model, not free rigid-body dynamics. Its evidence
   must include attitude commands, achieved attitude, attitude lag, rates, and
   force-direction consistency; overlapping altitude and speed curves alone
   prove nothing beyond reuse of the 3-DOF state.
3. **Full 6-DOF** — free rigid-body attitude and body rates, moments, inertia,
   actuator dynamics, control allocation, and all applicable mass and event
   dynamics.

The three tiers share one scenario contract. Only the dynamics tier may differ
for a reduction-parity experiment. A plotter must reject an overlay when the
initial state, environment, vehicle models, controls, events, duration, or
termination policy do not match.

## Scenario contract

Every generated run records hashes or stable identifiers for:

- vehicle description and family;
- canonical-SI initial state, mass, CG, and inertia;
- atmosphere, gravity, wind, aerodynamic tables, propulsion, and mass model;
- control command history, controller, actuator limits, and allocation;
- segment graph and transition events;
- fidelity tier: `3dof`, `pseudo_6dof`, or `6dof`;
- integrator, step size, output rate, duration, and termination policy.

The contract is generated from catalog metadata. Problem files are outputs of
that process and remain ordinary TAOS/TAORYX inputs.

## Objective model

Each scenario declares a list of typed objectives. An objective contains:

```yaml
id: waypoint-2
kind: waypoint
channel: north_m
target: 300.0
tolerance: 2.0
unit: m
comparison: absolute_error
weight: 1.0
phase: corner-2
severity: required
```

Supported objective families should include:

- state targets: altitude, position, speed, heading, alpha, beta, rates;
- bounded channels: dynamic pressure, Mach, heating proxy, load factor,
  actuator position/rate, table margin;
- event objectives: segment reached, separation occurred, ground contact,
  declared termination;
- trajectory objectives: waypoint arrival, racetrack completion, return to
  start, altitude transition, recovery after disturbance;
- invariants: continuity, mass balance, quaternion norm, force/moment closure,
  and step-size convergence.

Each measurement retains its physical unit and source channel. No objective
may be scored from a unitless display value.

### Trajectory resource rollup

Every controller-mission rollup also records resource and validity metrics,
not just terminal error:

- `table_margin_min_normalized`: worst distance to any queried table edge,
  dimensionless; `0` is the hard validity boundary.
- `table_margin_average_normalized`: time-sample average of the same margin.
- `control_saturation_fraction`: fraction of samples with any actuator
  saturation flag active.
- `control_saturation_average` and `control_saturation_max_abs`: aggregate
  saturation severity, dimensionless.
- `control_derivative_abs_average` and `control_derivative_abs_max`: absolute
  actuator-command derivative statistics, in the declared command unit per
  second, with per-control breakdowns retained.

These metrics are included in the quality score through advisory objectives.
They do not silently turn a diagnostic trajectory into a valid mission: hard
gates for table extrapolation, sustained saturation, continuity, and declared
mission completion remain separate. Rate limits are vehicle-family/controller
profile parameters and must not be interpreted as universal engineering
limits.

## Slack-aware scoring

The evaluator must preserve the raw result rather than returning only pass or
fail. For an absolute-error objective:

```text
error          = abs(actual - target)
normalized     = error / tolerance
slack          = tolerance - error
score          = max(0, 1 - normalized)
```

For a bounded objective, `error` is the amount outside the permitted band. For
an event objective, the result records whether the event occurred, when it
occurred, and its event audit. For a required invariant, any violation is
recorded with its maximum magnitude and time.

The result dictionary for each objective must contain at least:

```json
{
  "id": "waypoint-2",
  "status": "pass",
  "actual": 301.4,
  "target": 300.0,
  "error": 1.4,
  "tolerance": 2.0,
  "slack": 0.6,
  "normalized_error": 0.7,
  "unit": "m",
  "time_s": 42.0,
  "weight": 1.0,
  "severity": "required"
}
```

The final composite dictionary must contain:

```json
{
  "status": "pass",
  "score": 87.5,
  "required_objectives": 8,
  "required_passed": 8,
  "advisory_objectives": 3,
  "worst_required_normalized_error": 0.92,
  "objectives": [],
  "termination": {},
  "scenario_contract_sha256": "..."
}
```

The weighted score is informative, not a substitute for required gates:

```text
composite_score = 100 * sum(weight * score) / sum(weight)
```

A run cannot pass merely because strong objectives hide one failed required
objective. The report must also expose the worst normalized error and every
negative slack value. This lets reviewers distinguish a reasonable 2 m miss,
a marginal 300 m miss, and an unacceptable 40 km miss without changing the
underlying evidence.

Tolerance policy is part of the contract. Each tolerance must state its unit,
rationale, source or engineering basis, and whether it is a hard requirement,
soft objective, or diagnostic threshold. A tuning report should show score
changes when tolerances are tightened or relaxed; it must not silently widen a
goal to make a trajectory pass.

## Vehicle-family mission set

After plant and convention-firewall checks, each family gets a long mission
appropriate to its mechanics.

| Family | 3-DOF / bridge progression | Full 6-DOF evidence |
| --- | --- | --- |
| B747 | trim, altitude capture, heading corner, return | 120–300 s trim and recovery with local attitude/rate evidence |
| Skywalker X8 | powered trim, altitude step, rounded waypoint rectangle | 60–180 s recovery with elevon lag, beta, and envelope margins |
| Hummingbird | thrust-vector takeoff, hover, square, return | rotor allocation, motor lag, yaw step, wind recovery, landing event |
| X-15 | release, coast, descent corridor, ground event | source-prescribed release through bounded glide/descent; powered stages only when source data supports them |

Each route corner has a physically reasonable turn radius, speed, altitude
band, dwell time, and maximum cross-track error. A scenario is not successful
because it reaches one waypoint if it misses the next corner or terminates
below ground.

## Required tooling

Implement the workflow as generic utilities:

1. `scenario_contract` generation and equality checking;
2. goal/objective evaluation against canonical telemetry;
3. transition continuity and event audit;
4. unit-aware raw result and composite-dictionary emission;
5. family-filtered execution (`--family`, `--scenario`, `--tier`);
6. plots showing actual versus target, tolerance bands, event markers, and
   normalized error/slack;
7. evidence packaging with input/output hashes and the composite score.

The same harness must accept a new vehicle through catalog metadata, a source
problem/table binding, and a telemetry adapter. It must not require a new
vehicle-specific test framework.

## Gates

### Gate 1 — Contract integrity

All three tiers use matching physical inputs. No incomparable overlays are
generated.

### Gate 2 — Plant evidence

Source differential checks, units, frames, table margins, closure, and
convergence pass before controller objectives are evaluated.

### Gate 3 — Objective evidence

Every required objective has a raw unit-bearing result, tolerance, slack, and
time or event location. Continuity and termination rules pass.

### Gate 4 — Long mission

The vehicle completes its family-appropriate route with no unreported envelope
exit, saturation, below-ground propagation, or state discontinuity.

### Gate 5 — Claim review

The packet states exactly which tier and vehicle-family claims are supported.
Composite score is reported alongside, never instead of, the claim boundary.

## Milestones and exit criteria

Work proceeds in small, reviewable gates. A later milestone may not be marked
complete because an earlier result is merely finite or visually plausible.

| Milestone | Deliverable | Verification | Exit criterion |
| --- | --- | --- | --- |
| M0 — Contract freeze | Versioned contract schema and claim boundary | Schema validation tests and a four-family catalog audit | Every selected run identifies vehicle, tier, inputs, commands, events, duration, termination, and hashes |
| M1 — Objective scorer | Typed objective model and composite result dictionary | Unit tests for absolute, bounded, event, invariant, weighted, missing-data, and unit cases | Same telemetry produces deterministic scores; missing or unitless channels fail closed |
| M2 — Event and continuity audit | Segment transition audit with declared discontinuities | Positive/negative tests for continuous state, impulse, mass change, reset, ground contact, and envelope exit | No unannotated position/velocity/attitude/mass discontinuity can pass |
| M3 — Parity harness | Matched true 3-DOF, pseudo-6-DOF, and constrained 6-DOF contracts | Contract equality/refusal tests and initial-state parity reports | Incomparable overlays are rejected; matched cases emit unit-bearing parity deltas |
| M4 — Plant evidence | Source differential, trim, closure, margins, and convergence reports | Four-family firewall sequence | Each family has a declared green, diagnostic, or blocked status before mission scoring |
| M5a — Nominal long missions | One mechanics-appropriate plant mission per family and tier | Family-filtered integration runs and objective reports | Required objectives, termination, envelope, continuity, and convergence gates pass for the claimed tier |
| M5b — Controller mission closure | Family-specific closed-loop waypoint/recovery missions | Controller metrics, disturbance cases, and objective reports | Each claimed controller mission reaches its declared waypoints/altitudes, recovers from its declared disturbance, and remains inside plant and actuator limits |
| M6 — Evidence release | UUID-scoped packets, plots, manifests, rollup, and claim ledger | Clean rebuild, hash verification, focused tests, full test suite, and reviewer checklist | A reviewer can reproduce every score from packaged inputs without local paths or stale artifacts |

Each milestone must leave a machine-readable artifact under the evidence run
directory and a short human-readable report. The milestone status vocabulary is
`pass`, `diagnostic`, `blocked`, or `not_run`; `diagnostic` and `blocked` are
never promoted to pass by the rollup.

### Current implementation status

- **M0 — Contract freeze:** pass for the four vehicle-family packet catalog and
  the generic point-mass contract fixture in
  `verification/generic_point_mass_contract.yaml`.
- **M1 — Objective scorer:** implemented in `taoryx.objectives`, with YAML
  loading, unit-bearing results, composite scoring, and focused tests.
- **M2 — Event and continuity audit:** pass for the current packet path;
  declared runtime events are audited and channel-level margin telemetry is
  emitted. Future event-rich staging cases still require dedicated fixtures.
- **M3 — Parity harness:** pass for the four-family matched reduction packet;
  true 3-DOF versus pseudo-6-DOF contracts compare, while free 6-DOF
  mismatches are refused and reported.
- **M4 — Plant evidence:** pass for the current four-family packet after the
  X-15 bank-reversal case adopted its declared `dt=0.0125 s` convergence step;
  this remains source-bounded research-surrogate evidence, not global
  engineering validity.
- **M5a — Nominal long missions:** pass. The nominal four-family objective,
  runtime, continuity, and closure gates pass.
- **M5b — Controller mission closure:** pass for the declared controller-
  mission tranche. The packet executes and scores one native controller route
  or transition per family; these are source-bounded controller demonstrations,
  not flight qualification or historical-compatibility claims.
- **M6 — Evidence release:** the all-family packet is reproducible and
  hash-audited with no absolute archive paths; `tools/audit_fidelity_packet.py`
  independently recomputes each packaged objective composite.
  `tools/audit_fidelity_milestones.py` prevents a green objective score from
  hiding a failed closure or controller-mission gate. A clean source snapshot
  regenerated parity and controller evidence and reproduced all four plant
  scores plus all four controller-mission scores; the current release audit is
  green through M6.

### Final exit plan

The epic is complete only when all of the following are true:

1. M0–M3 pass for B747, Skywalker X8, Hummingbird, X-15, and a minimal generic
   point-mass fixture.
2. M5a passes for the nominal long plant mission of each family, and M5b
   passes for every controller mission included in the supported-claims ledger.
3. Each family has one source/convention firewall report, one matched parity
   packet, and one full-mission packet.
4. Every required objective has a canonical unit, measured value, tolerance,
   slack, normalized error, and time/event location.
5. Every packet records termination reason, envelope margins, continuity/event
   audit, convergence result, source/model hashes, and software revision.
6. The composite score is reproducible from the raw objective records and does
   not override any failed required objective.
7. Long missions use continuous integrated state histories; any allowed
   discontinuity has a declared event, policy, pre-state, post-state, and
   impulse or mass delta.
8. The final claim ledger distinguishes proven, provisional, diagnostic,
   blocked, and unsupported claims for each vehicle and fidelity tier.
9. A clean checkout can run the documented packet commands and reproduce the
   manifests and score schema without absolute local paths.

The executable release check is:

```bash
python tools/audit_fidelity_packet.py artifacts/verification/fidelity_ladder/<packet>.zip --json
python tools/audit_fidelity_milestones.py artifacts/verification/fidelity_ladder/<packet>.zip --json
```

The milestone audit is intentionally stricter than the weighted objective
score. A family with a failed independent closure gate blocks M4 and M5 even
when its nominal objective composite is green. M6 remains `diagnostic` until
the packet has been reproduced from a clean checkout; hash integrity alone is
not evidence of clean-checkout reproducibility. The epic exit status is
`complete` only when the audit reports M0--M6 as `pass` and the claim ledger
contains no unsupported green claim.

If a family cannot meet its plant or model-envelope gate, the exit plan does
not require inventing a tolerance or controller to force completion. It exits
with a claim-bounded `blocked` packet naming the failing milestone, evidence,
and next repair. This preserves useful progress without certifying an
unreasonable trajectory.

## Immediate execution order

1. Add event-rich staging/separation fixtures and verify channel-specific
   discontinuity policy for M2 extension coverage.
2. Extend M4 from the current source/convention packet to independent source
   differential and trim reports in every family release packet.
3. Treat any expansion beyond the declared M5b tranche—such as stronger
   disturbance recovery, true landing physics, or target arrival—as a new
   claim with its own objectives; do not promote the current route-transition
   evidence into those claims.
4. Optional follow-on: build family-filtered packets or a composite rollup for
   focused review. The all-family packet is the release authority; any scoped
   packet must pass the same packet and milestone auditors.
5. Tune only against declared plant truth and report any tolerance change as a
   reviewed metadata change.

The deliverable is not merely a set of trajectories. It is a reproducible,
unit-aware argument that each trajectory met the declared objectives by a
measured amount.
