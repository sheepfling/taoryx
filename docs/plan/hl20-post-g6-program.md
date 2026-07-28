# HL-20 Post-G6 Completion Program

## Purpose

The HL-20 release/glide witness is qualified through HL20-G6 across the four
reduced fidelity tiers. This program defines the remaining work without
promoting the synthetic booster, fixed `CD`/`L/D` surrogate, logical surface
overlay, or synthetic California-to-Hawaii route into source-native or mission
performance claims.

The program is executed in order. A later milestone may use an earlier
milestone's artifacts, but it cannot silently widen an earlier claim boundary.

Current status: `HL20-P0` through `HL20-P6` are complete as source-bound,
fail-closed evidence tranches. None of them promotes a source-exact
trajectory, controller, route, or mission-performance claim. Extended vehicle
physics, controller qualification, and route-performance work are deferred to
a new contract.

## Milestones

### HL20-P0: Plan and claim reconciliation

Exit conditions:

- Existing R1-R4 and HL20-G0-G6 statuses distinguish `verified_witness` from
  `source_native_qualified` and `mission_qualified`.
- The source package, fixed mass binding, synthetic booster, route assumption,
  control assumption, and terminal policy are listed in one machine-readable
  contract.
- Every later milestone has an owner artifact, test command, and fail-closed
  claim boundary.
- The next open milestone at that point was source-native aerodynamic coupling,
  not another duplicate synthetic showcase.

### HL20-P1: Source-native aerodynamic coupling [complete: coupled, fail-closed]

Integrate the byte-pinned HL-20 DAVE-ML force/moment graph into the reachability
runner while preserving the four-tier ladder:

```text
3DOF source-derived loads
    -> pseudo-6DOF source-derived loads plus reduced attitude response
        -> native 6DOF source force/moment graph
            -> native 6DOF plus explicitly qualified surface inputs
```

Exit conditions:

- The runtime consumes the pinned source package and records package and graph
  hashes in every artifact.
- Mach, alpha, beta, altitude, dynamic-pressure, and surface bounds are
  enforced with actionable diagnostics.
- Source force/moment outputs are independently compared against DAVE-ML replay
  anchors before being used in reachability.
- Translation and rotation residuals remain within the contract tolerances.
- The reduced tiers use documented projections of the same source data rather
  than a second undocumented aerodynamic model.
- No source-exact trajectory claim is made.

The executable qualification command is
`python -m tools.dev qualify-hl20-source-reachability`. It emits replay-anchor
residuals, source-query counts, source hashes, and per-tier invalid/timeout
dispositions. A native tier that leaves the source envelope is recorded as
invalid; it is not extrapolated or promoted.

### HL20-P2: Controls, actuators, stabilization, and trim [complete: bounded with explicit trim dispositions]

Add the minimum control system required to make the source surfaces executable:

- explicit surface position, rate, and saturation limits;
- direct/open-loop surface commands for diagnostics;
- stabilization and glide-trim capture segments;
- source/control-direction probes;
- controller authority and actuator telemetry;
- trim generation for each declared operating point.

Exit conditions:

- Every command reports requested, achieved, saturated, and rate-limited values.
- A trim or stabilization failure identifies the violated bound and operating
  point instead of producing a plausible-looking trajectory.
- Direct surface response is verified independently from closed-loop response.
- Controller claims remain tier- and profile-specific.

Current evidence commands:

- `python -m tools.dev qualify-hl20-source-reachability` verifies the pinned
  coefficient replay anchor, independent seven-channel direction probe, and
  source validity diagnostics.
- `python -m tools.dev showcase-hl20-source-composites` writes source loads,
  requested/achieved/rate-limited actuator channels, and mission telemetry into
  the reviewer composites.

The current native velocity-aligned policy tracks the full scheduled attitude
and cancels the source moment disturbance through an explicit reduced trim
policy. The operating-point matrix is complete: Mach 0.3 is a near-tolerance
numerical residual and Mach 2.0 has no zero-surface pitch trim in the declared
alpha interval. Both are recorded with residuals, bounds, and operating-point
state; neither is silently treated as a stabilized trim.

### HL20-P3: Energy-managed entry and glide [complete: source-bound witness]

Implement the mission segments described by the family graph:

```text
release -> stabilization -> glide trim -> right bank -> left bank
        -> energy descent -> arrival energy corridor
```

Exit conditions:

- Segment transitions are event records with parent/child lineage and source or
  assumption provenance.
- The vehicle remains inside the declared source validity envelope or the run
  fails closed with a diagnostic.
- Specific energy, altitude/Mach, bank, crossrange, and terminal speed are
  recorded for every segment.
- Energy-management decisions are reproducible from the declared command and
  controller configuration.
- The route is still classified as a mission scenario until terminal goals pass.

The current source-bound witness declares and serializes
`hl20.energy_managed_entry_glide.v1`, with stabilization, glide trim, right
bank, left bank, and energy descent transitions. Segment telemetry includes
specific energy, altitude, speed, commanded bank, and native attitude tracking
error. The failed source trim anchors are explicitly dispositioned in the
source reachability report and family manifest, so P3 remains a bounded
integration witness rather than a source-exact mission result.

### HL20-P4: Executable California-to-Hawaii terminal contract [complete: gate exercised]

Promote the route from comparison-only to an executable target contract only
after P1-P3:

- target position and geodesic frame are explicit;
- impact/arrival radius is required where the scenario declares one;
- terminal speed bounds are explicit and unit checked;
- ground contact, target arrival, and horizon timeout are distinct outcomes;
- timed-out query IDs can be selected and rerun with a longer horizon;
- route success is never inferred from a horizon endpoint.

Exit conditions:

- The search artifact covers the declared route search space and records every
  successful, unsuccessful, invalid, and timed-out candidate.
- A successful candidate satisfies both spatial and speed terminal goals.
- A failed candidate records the first limiting criterion and terminal margins.
- The report explicitly states whether the result proves route capability or only
  bounded reachability under the declared assumptions.

The executable contract runner is
`python -m tools.dev qualify-hl20-terminal-contract`. It intentionally retains
the current failed radius/speed margins; that is evidence that the terminal
gate is active, not a route-capability result.

### HL20-P5: Robustness, realism, and evidence closure [complete: canonical matrix regenerated]

Expand beyond the nominal three-command witness:

- timestep and integration convergence;
- launch, trim, atmosphere, and crosswind perturbations;
- source-envelope boundary cases;
- booster thrust, burn, dry-mass, and separation variants;
- validated HL-20 mass-property variants;
- longer-horizon timeout reruns;
- composite flight, capability, search-coverage, controls, deployment, and
  terminal-margin boards.

Exit conditions:

- The robustness matrix is reproducible and classified, not cherry-picked.
- Every plot has a manifest pointing to the exact source artifact and hashes.
- The final report separates source-native evidence, synthetic assumptions,
  numerical verification, and mission-performance claims.
- The plan can be closed without unresolved stale statuses or undocumented
  generated evidence.

The source robustness runner is
`python -m tools.dev qualify-hl20-source-robustness`. It records timestep
comparison, invalid source queries, and longer-horizon reruns of timed-out
candidate IDs, plus named glider-mass, booster-thrust, booster-burn, booster
dry-mass, separation-impulse, crosswind, and inertia variants. The remaining
candidate outcomes are expected evidence: variants that leave the source
domain or fail to stabilize are classified invalid rather than omitted. The
70-second matrix, timeout rerun, source-boundary probes, fixed mass/inertia
binding, and source-bound showcase package are regenerated under the
canonical `artifacts/` paths.

### HL20-P6: Repository verification and closure [complete]

Close the program only after the implementation, contracts, and evidence all
agree on the same claim boundary.

Exit conditions:

- Canonical source reachability, terminal-contract, robustness, and showcase
  artifacts exist and contain source hashes, search-space parameters,
  successful/unsuccessful/invalid/timed-out dispositions, terminal margins,
  deployment telemetry, and plot manifests.
- The verification family, reference family, showcase contract, and this plan
  have no stale status that claims pending work already completed or promotes
  a failed/invalid witness to capability.
- The affected unit, end-to-end, lint, and type-check suites pass after the
  detached-body air-relative-velocity correction and central `tools.dev`
  package-path handling.
- The final report names deferred physics and follow-on work instead of
  leaving an implicit open gap.

Closure evidence:

- `artifacts/verification/hl20-source-reachability.json`
- `artifacts/verification/hl20-ca-hi-terminal-contract.json`
- `artifacts/verification/hl20-source-robustness.json`
- `artifacts/showcases/hl20_california_to_hawaii/source_bound/bundle-manifest.json`
- `python -m ruff check src tests tools scripts`
- `python -m tools.dev typecheck`
- `python -m pytest -m "not slow and not artifact and not simple_aero"`
- HL20 affected regression suite: 37 passed, 1 skipped

## Deferred scope

The following remain outside this program unless explicitly promoted into a new
contract: thermal protection, flexible-body/aeroelastic effects, landing gear
and runway dynamics, historical NASA trajectory reconstruction, and arbitrary
vehicle deployment beyond the already-qualified passive spent-cylinder case.

## Working commands

```text
python -m tools.dev qualify-hl20
python -m tools.dev qualify-hl20-source-reachability
python -m tools.dev qualify-hl20-terminal-contract
python -m tools.dev qualify-hl20-source-robustness
python -m tools.dev showcase-hl20-source-composites
python -m pytest tests/e2e/test_hl20_qualification.py -o addopts=''
python -m pytest tests/unit/test_hypersonic_lifting_body_family.py -o addopts=''
```

The first command remains the regression gate for the completed synthetic
integration. P1-P6 add source-bound reports and closure checks rather than
weakening HL20-G0-G6. A future Alpha 4 contract may promote controller,
physical mass-property, thermal, landing, or route-performance claims only with
new source and acceptance criteria.
