# F-16 S-119 reference-family integration status

Status: in progress; the family is not yet racetrack-qualified.

This note records the reproducible path needed to bring the source-grounded
F-16 S-119 reference into the same powered-fixed-wing workflow used by the
X8 and B747.  It is an integration record, not a claim that a public F-16
flight-control or actuator database has been recovered.

## Current vertical slice

The following pieces are now present and tested:

1. The DAVE-ML source package and atmosphere binding load through a named
   `reference_f16_s119` family manifest.
2. A fixed-altitude source operating point is represented by an explicit
   trim state, trim effectors, mass, inertia, source coordinates, and units.
3. The runtime body-load plant evaluates source aerodynamic and propulsion
   loads and six body-state derivatives.
4. A centered finite-difference linearization is derived from that runtime
   plant.  A second perturbation size is retained as derivative evidence.
5. A local LQR is synthesized from the plant-derived state/effectors model.
6. The generic allocator maps a requested local wrench to elevator, aileron,
   rudder, and throttle with position, rate, lag, and availability limits.
7. The shared powered-fixed-wing racetrack resolves into six explicit phases
   and four independent truth-telemetry gates.
8. A local runner can execute a labelled direct-wrench comparison or the
   physically allocated surface path.  The latter logs actual effectors,
   requested/achieved wrench, residuals, limits, and saturation.
9. The independent objective evaluator consumes truth telemetry; a controller
   transition cannot create a mission pass.
10. The same route now runs through the point-mass and named pseudo-6DOF
    reductions, and a four-tier packet builder preserves all results side by
    side.
11. A reusable source-trim resolver now reproduces seven fixed-altitude,
    fixed-airspeed operating points at 0, 1.5, 3, 4.5, 6, 7.5, and 9 km.
    Each point has plant-derived derivative consistency evidence, a local
    rank-four physical-effector allocation witness, and a one-second
    unsaturated local plant-derived LQR trim-hold witness. The 1.5, 4.5, and
    7.5 km entries are explicit midpoint witnesses, not interpolated claims.
12. The self-contained packet now includes commanded-versus-actual actuator
    telemetry, source force/moment closure, truth event times, a terminal-gate
    snapshot with post-gate drift, control-coverage metrics, explicit resource
    nonclaims, and the supporting trim, maneuver, linearization, allocation,
    reduction, and robustness evidence files.
13. The packet now includes
    `reduction_mission_window_comparison.json`, which checks shared route
    references, truth-objective parity, event-time deltas, and measured
    trajectory disagreement against the physical-surface parent. This is
    semantic mission validation, not an assertion of rigid-body or actuator
    equivalence.

The direct-wrench result is therefore a comparison screen only.  The surface
result is the physically accountable path, but it remains development evidence
until the route gates and declared envelope pass.

The reduced route runner is also intentionally bounded and named.  It uses the
shared route reference for translational response, records source loads as
diagnostic observables, and does not turn a zero reduced residual into an
effector claim.

## Data contract by fidelity

### Point mass 3DOF

Required: mass or mass schedule, center-of-mass force model, propulsion or
energy/resource model, atmosphere/environment assumptions, and semantic
guidance limits.  It can prove path, energy, altitude, speed, resource, and
terminal geometry.  It cannot prove attitude, body rates, moments, or physical
surfaces.

### Named pseudo-6DOF / 3T+3K

In addition to the 3DOF contract, require a named attitude/rate response law,
state ordering, response time constants or identified response matrices, rate
and command limits, and explicit saturation semantics.  It can prove the
declared response law, not physical moment balance or individual surface
activity.

### Rigid-body 6DOF with direct wrench

Require mass properties, a force/moment plant, body frames, inertia, state and
wrench units, and an explicit direct-wrench injection boundary.  This tier is
useful for stabilizability and trajectory screening, but its requested moments
are not actuator proof.  Every result must carry `direct_wrench` provenance.

### Rigid-body 6DOF with physical effectors

Require all direct-wrench inputs to be replaced by physical effectors or a
declared actuator model: surface geometry or source control derivatives,
effector signs and frames, trim positions, travel/rate/lag limits, local
effectiveness, allocator residuals, achieved loads, and actuator telemetry.
The nonlinear plant must receive the actual effector state.  This is the
promotion target for the F-16, and the same rule applies to X8, B747, and
Hummingbird with family-specific effectors.

## Trim and linearization sequence

The F-16 operating-point workflow is:

```text
source package and atmosphere
    → canonical state/control ordering
    → source operating-point trim fixture
    → trim residual and frame/unit checks
    → local source-load effectiveness
    → plant-derived A/B derivatives
    → perturbation-size consistency check
    → scaled LQR synthesis
    → bounded physical allocation
    → nonlinear closed-loop witness
```

The canonical body-state order is
`[u, v, w, p, q, r]`; the physical effector order is
`[elevator, aileron, rudder, throttle]`.  A prior development defect allowed
the JSON insertion order `[p, q, r, u, v, w]` to drive the LQR scaling profile.
That contract is now corrected and tested explicitly.

The racetrack controller currently uses the sea-level point as its executable
mission fixture. The operating-point catalog now contains seven independently
retrimmed local witnesses, including three midpoint witnesses, but this is not
yet a trim solver over arbitrary
altitude, Mach, configuration, or fuel state. Promotion to a scheduled F-16
controller still requires gain synthesis at each point and continuous
schedule-transition tests.

Generate the reproducible operating-point evidence with:

```text
PYTHONPATH=src python3 tools/validate_f16_operating_points.py
```

The resulting `verification/f16_operating_points_evidence.json` is schedule-
readiness evidence, not a claim that the intermediate gains or transitions are
validated.

## Current racetrack result

The route is intentionally shared with the X8/B747 family template:

```text
outbound climb → outbound level
→ left turn / bank reversal
→ inbound descent → inbound level
→ right turn / return gate
```

The current source-realization runner is numerically stable through the full
nominal horizon and can produce a valid control-path artifact.  With the
F-16-specific 25 km turn radius and 6 degree bank schedule, the independent
surface run can now pass all four gates in development evidence.  The result
required two explicit runtime protections: a bounded state-based terminal
capture reference and a transition guard that holds the high-altitude contract
until truth position crosses the left-turn exit geometry.  The fixed
post-arc target and time-only descent handoff were not sufficient for the
source plant's phase lag.

This is a useful diagnostic result, not a qualification pass.  The 500 m
cross-track gate corridor and 35 m altitude tolerance remain explicit; they
have not been widened to make the route green.  The direct-wrench screen
remains comparison-only. The surface path now has a narrower T5-local
development promotion, but it is not a family or release qualification.

The current four-tier F-16 packet records the following nominal state:

| Fidelity | Required truth gates | Status | Honest claim |
| --- | ---: | --- | --- |
| point mass 3DOF | 4 / 4 | nominal case pass | bounded translational route and source-load diagnostics |
| named pseudo-6DOF | 4 / 4 | nominal case pass | bounded translation plus named attitude/rate response |
| 6DOF direct wrench | 1 / 4 | development pending | comparison screen only; no physical effector claim |
| 6DOF physical surfaces | 4 / 4 | T5 local development pass / family qualification pending | bounded effector allocation with truth-gate closure at one operating point |

The regenerated packet also records semantic timestep evidence. The 3DOF,
pseudo-6DOF, and surface-allocated packets preserve objective status with the
declared comparison step; the direct-wrench comparison remains pending because
its critical gate margin changes materially even though its failed mission
status is stable. The surface packet includes an R1 fixed initial-condition
matrix with 3/4 cases passing and a retained +2 degree initial-bank boundary
failure. These are development evidence, not release robustness.
Each run now also checks the source package's declared Mach, alpha, beta, and
altitude bounds from the runtime telemetry; an excursion makes the hard-gate
evaluation fail instead of remaining a diagnostic-only range.

Build the packet with:

```text
PYTHONPATH=src python3 tools/build_f16_racetrack_fidelity_packet.py \
  --output artifacts/showcases/f16-s119-racetrack-fidelity-ladder
```

Each tier contains its own claim, resolved case, telemetry, truth objective
report, board, schemas, provenance, actuator traces, supporting evidence, and
reproduction command.  The root
`comparison.json` does not collapse the tiers into one score.

The appropriate next work is:

1. Preserve the terminal gate as a truth objective and add a declared
   terminal-capture phase rather than clamping the guidance reference at the
   nominal geometric endpoint.
2. Add route-level speed, altitude, cross-track, heading, and ground-track
   phase-error telemetry to the packet.
3. Tune the nested outer attitude/speed/path commands against attainable local
   authority rather than changing the truth tolerances.
4. Validate surface allocation at the turn operating condition, not only at
   the sea-level trim point; in particular, record whether yaw-rate demand is
   being cancelled by the local LQR or lost to source aerodynamic damping.
5. Synthesize and validate local controllers at the additional operating
   points; the seven-node catalog is schedule-readiness evidence, but no
   continuous runtime schedule is implied.
6. Keep the reduced 3DOF and named pseudo-6DOF passes as valid lower-tier
   semantic mission evidence, but report disagreement instead of treating them
   as a rescue for a failed surface run. The packet's
   `reduction_mission_window_comparison.json` is the authoritative comparison.
7. Repair the direct-wrench route behavior or retain it explicitly as a
   screen-only comparison; it is not eligible for T4 promotion.
8. Connect the local runner to the provider batch/step API and add true
   batch-versus-step replay evidence; the current packet intentionally marks
   this contract as not run.

The current racetrack development controller uses a separately declared
outer roll-damping setting (`outer_roll_kd_s = 2.5`) in the runner.  It is a
development tuning parameter, not a source-derived F-16 control-law claim;
the next cleanup should move it into the family controller profile and run a
small reproducible tuning sweep before freezing the showcase acceptance.

## Promotion gates

The family should advance monotonically:

| Tier | Evidence required | F-16 status |
| --- | --- | --- |
| T0 structural | family, states, controls, units, sources | complete |
| T1 trimmed | source operating point and residual contract | complete locally |
| T2 linearized | runtime-derived A/B and derivative evidence | complete locally |
| T3 linearly controlled | LQR poles and local perturbation response | development pass |
| T4 physically allocated | bounded effectors and requested/achieved wrench | development path present |
| T5 nonlinear validated | route/maneuver passes at one operating point | development pass; family promotion pending |
| T6 envelope validated | multiple operating points and transitions | not started |

The seven-point catalog advances trim/linearization schedule readiness, but it
does not advance T5 or T6 by itself: no multi-point racetrack controller or
continuous gain schedule has yet been qualified.

The machine-readable `verification/f16_t5_local_promotion_evidence.json`
records the narrower T5-local development promotion. It is not a family or
release qualification badge.

No F-16 showcase should use `QUALIFIED` until the independent objective table
contains no failed, blocked, skipped, or saturated required phase and the
claim names the exact operating envelope.
