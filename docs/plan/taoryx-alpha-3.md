# TAORYX Alpha 3: vehicle breadth and qualification expansion

**Status:** Planned after Alpha 2 closeout  
**Predecessor:** Alpha 2 closeout contract  
**Boundary:** Add and qualify new vehicle archetypes without weakening the
common runtime, family-package, control, or evidence contracts.

Alpha 2 must complete the [future-family interface stress test](future-family-interface-stress-test.md)
before this breadth work begins. That is a contract-closure gate, not a
requirement to implement the future families early.

## Why Alpha 3 exists

Alpha 2 should finish the reusable platform around the four established proof
families: B747, Skywalker X8, Hummingbird, and X-15, plus the bounded F-16 and
HL-20 source-grounded reference-anchor closeout. It should not also try to
finish every new aircraft, rotorcraft, spacecraft, surrogate, corpus, and fleet
idea now in the backlog.

Alpha 3 is the breadth and qualification release. Its purpose is to prove that
the Alpha 2 contracts extend cleanly to new families while preserving explicit
claim boundaries. A new family is not complete merely because it has a JSON
deck or can take one numerical step.

## Alpha 2 finish line

Alpha 2 is complete when all of the following are true:

1. The common scenario/evaluation substrate is the only scoring and evidence
   route for batch, interactive, and showcase runs.
2. The session API supports reset, step, run, pause/resume, interrupt,
   deterministic replay, and checkpoint/restart through one transition.
3. The onboarding/convention firewall accepts a new family through metadata,
   source records, tables, adapters, trim, closure, and bounded propagation,
   with actionable diagnostics.
4. The four established families have reproducible, family-appropriate
   flagship missions with source/evidence boundaries, event markers, achieved
   controls, envelope margins, closure, convergence, and objective scores.
5. Plant-truth trim and normalized gentle/standard/aggressive controller
   presets work through the common control contract; LQR remains a selectable
   baseline rather than a permanent architecture.
6. Checkpointed and uninterrupted runs agree within declared tolerances.
7. The four families reach at least M4 Composable for their supported claims;
   no global M5/M6 or real-aircraft validity claim is implied.
8. The F-16 S-119 and HL-20 Mod K reference anchors have deterministic
   collections, native source-plant replay, source check-case/trim evidence,
   and reproducible direct-control artifacts. Their actuator, controller,
   mission, RL, and reduced-fidelity layers remain separate follow-on work.

Alpha 2 explicitly does not require implementing the new helicopter, tiltrotor,
small-aircraft, spacecraft, or surrogate families. Their intake records and
source hashes may be present, but their runtime qualification belongs to Alpha
3.

## Alpha 3 workstreams

### A3-W1 — Family package and maturity expansion

Implement the family package layout and onboard, in order:

1. C172P-class light propeller aircraft.
2. R44-class helicopter.
3. UH-1H scheduled helicopter.
4. UH-60A scheduled helicopter.
5. XV-15-class tiltrotor.
6. V-22-class scaled tiltrotor surrogate.
7. Learjet 24-class local business jet.
8. Spacecraft reference family.
9. Public-data flight-dynamics surrogate roster.

Each package progresses independently through M0–M4. A weak package must not
inherit the maturity badge of a related family.

### A3-W2 — Hybrid mode transitions

Qualify a reusable transition contract for helicopter and tiltrotor modes:

- entry and exit guards;
- continuously scheduled force, lift-sharing, and mixer parameters;
- state, resource, and controller handoff;
- hysteresis and chatter prevention;
- actuator and nacelle/rate limits;
- abort/reversion;
- forward and reverse conversion;
- batch/step equivalence.

### A3-W3 — Source and model correlation

Add source differential checks and progressively stronger validation:

- R44 identified hover response;
- UH-1H source-point matrices, then nonlinear TM-73254 components;
- UH-60A source-point matrices, then GenHel/Airloads comparisons;
- XV-15 conversion and handling-quality references;
- C172 same-class performance and dynamic-response anchors;
- Learjet local modes and later Mach/configuration sources.

### A3-W4 — Breadth tooling

Add search/reachability, coherent parametric vehicles, trajectory corpora,
population/fleet execution, sensors, weather, and spacecraft qualification
only after the family package and evidence contracts are stable.

The sensor work must use the [EOM timing and committed-truth
contract](../architecture/eom-timing-contract.md). IMU, estimator, and
multi-rate sensor development cannot begin by interpolating published vehicle
states or exposing RK solver stages as truth.

### A3-W5 — Compatibility claim maintenance

Maintain the evidence-bounded TAOS 96.0 language/specification profile and its
machine-readable claim ledger. Historical runtime equivalence remains an
explicit nonclaim, not an Alpha 3 implementation target: it would require an
executable and complete historical table library, and reconstructed syntax or
modern numerical agreement is not a substitute.

## Alpha 3 exit gates

Alpha 3 can close only when:

- at least three new domain families reach M4 through the common onboarding
  route;
- one light fixed-wing, one conventional helicopter, and one tiltrotor have
  3DOF and pseudo-6DOF evidence with common scenario contracts;
- one source-grounded family reaches an independently reviewed source-dynamic
  comparison milestone;
- one hybrid transition family passes forward, reverse, abort, and replay
  tests;
- all promoted parameters retain source, derived, estimated, or unavailable
  labels;
- breadth work has not introduced bespoke runners or bypassed the common
  evaluator;
- every release claim links to a machine-readable maturity and evidence record.

## Deferred beyond Alpha 3

Global flight validity, certification, proprietary aircraft reconstruction,
historical TAOS runtime equivalence without an oracle, and unrestricted claims for
post-stall, ground-effect, engine, rotor, or thermal behavior remain outside
the release definition.
