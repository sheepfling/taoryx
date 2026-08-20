# Mission composition and vehicle-onboarding automation

> **Priority:** This plan supplies the Runtime and Composition portions of the
> [Authoring → Runtime → Composition roadmap](authoring-runtime-composition-roadmap.md).
> Its next executable outcome is a fail-closed semantic-segment translator and
> `vehicle compose` → `vehicle run` path, first for the X8 and then for the
> shared airbreather template.  Registry visibility or adapter binding alone
> is not completion.

## Decision

Taoryx has three deliberately separate functional layers:

1. **Model Authoring** — parses, validates, and lowers a source model while
   preserving its meaning.
2. **Simulation Runtime** — evaluates a resolved model at a declared fidelity.
3. **Mission Composition** — combines a family definition, model assets,
   loadout, controls, resources, and evidence into an immutable vehicle
   realization, then converts a semantic mission
   intent into feasible geometry, objectives, controller references, and a
   truth-evaluated artifact pack.

Mission Composition must not be a hand-tuned collection of waypoints. It is the
bridge that lets a user run a vehicle coherently and lets a developer add a new
vehicle without rediscovering route length, turn radius, horizon, and objective
semantics by repeated simulation.

## Canonical layers

```text
family strategy + source data
    -> vehicle package + selected fidelity
    -> operating-point / capability profile
    -> semantic mission intent
    -> mission compiler and feasibility report
    -> resolved segment graph, objectives, and controller references
    -> controller / allocator / plant realization
    -> truth evaluation and showcase artifact
```

A family owns topology and the lowering rules.  A vehicle package owns source
data and calibrated capability values.  A fidelity realization owns the
control-path claim.  A mission template owns the objective sequence.  No layer
may silently supply facts that belong to another one.

## First reusable compiler: powered fixed-wing racetrack

The common airbreathing mission is:

```text
trim or airborne start
  -> isolated outbound climb
  -> high-altitude level dwell
  -> left turn
  -> isolated inbound descent
  -> low-altitude level dwell
  -> right turn / bank reversal
  -> oriented start-finish terminal gate
```

Its compiler receives a declared capability profile and a mission intent.  It
derives a conservative first route rather than accepting manually guessed
geometry:

\[
R_{\min} = \frac{V^2}{g\tan\phi_{\max}},
\]

\[
L_{\min} = V\max\left(
  \frac{\Delta h}{\dot h_{\mathrm{climb}}} + t_{\mathrm{level}},
  \frac{\Delta h}{\dot h_{\mathrm{descent}}} + t_{\mathrm{level}}
\right).
\]

The selected route uses a declared radius margin above \(R_{\min}\), a leg no
shorter than \(L_{\min}\), and a simulation horizon derived from the closed
course plus observation margin.  A requested speed, bank, or vertical rate
outside the capability profile is visible as a clipped intent; it is never
silently folded into a passing route.

This is a planning and preflight tool.  It does **not** certify the nonlinear
plant, controller, or physical effectors.  Existing checked-in racetrack
bindings remain the qualification baselines until generated routes are run and
truth-evaluated.

## Four fidelity realizations

Every compiled mission must bind to one explicit fidelity realization:

| Tier | What the route compiler supplies | What the realization must supply |
| --- | --- | --- |
| Point-mass 3DOF | speed, climb/descent, bank/lift-vector, route geometry, gates | achievable translational response and resource behavior |
| Named pseudo-6DOF | the 3DOF contract plus attitude/heading/flight-path references | declared response law, lags, limits, and achieved attitude/rate telemetry |
| Rigid-body 6DOF direct wrench | semantic route and body-motion references | force/moment path explicitly labeled `direct_wrench_screen` or bridge evidence |
| Rigid-body 6DOF physical effectors | semantic route and body-motion references | actual surfaces/thrusters/rotors, allocation, actuator limits, requested-versus-achieved wrench |

The compiler does not promote a lower tier.  Its artifact records the requested
fidelity and the evidence boundary supplied by the vehicle realization.

## Native-problem projection boundary

The structured vehicle package is intentionally richer than the native
`.prb` grammar.  Actuator-lag evidence, sensor configurations, allocation
matrices, provenance records, and other nested model facts belong in the
resolved vehicle/interface manifest unless the selected grammar has a declared
native representation for them.

Problem generation must therefore project only scalar, grammar-supported
runtime attributes.  It must never stringify an arbitrary YAML/Python mapping
or list into a `key=value` directive.  Every generated native problem is
parsed under its declared grammar profile as a regression gate.  This is a
Model Authoring syntax/integrity check, not evidence that structured actuator
metadata is active in the native plant.  A runtime capability is advertised
only when a named native binding consumes it.

## Onboarding workflow for an existing family

1. Select a physical family strategy; do not infer a topology from a vehicle
   name.
2. Declare data provenance, frames, units, state/control schema, resources,
   and supported fidelity tiers.
3. Establish an operating point: trim, hover, release, or orbit as applicable.
4. Produce a capability profile from source data, a trim/linearization result,
   or an explicitly labeled engineering estimate.
5. Compile the family mission and inspect clipping or infeasibility before
   simulation.
6. Bind guidance to the controller architecture appropriate to the selected
   fidelity.
7. Run independent truth objectives; a controller transition is diagnostic,
   not proof.
8. Promote only with the control/evidence tier actually demonstrated.

The expected developer input for a new powered-fixed-wing vehicle is therefore
data plus a capability profile and realization adapter — not a bespoke list of
waypoints and gains.

## New-family workflow

Creating a new physical family is a larger platform change.  It requires:

- a topology and resource model;
- a family strategy and four-tier lowering policy;
- a start/terminal contract and characteristic mission template;
- capability-estimation and feasibility rules;
- control intent, allocation, and actuator contracts;
- truth-objective evaluators and family-specific plot modules;
- synthetic and source-backed conformance witnesses.

New family creation should be deliberate.  A new named vehicle is normally an
existing-family onboarding task.

## Delivery sequence

1. Add a capability-scaled powered-fixed-wing compiler and profiles for X8,
   B747, A320, and F-16.
2. Compare generated geometry to existing hand-tuned bindings and surface
   mismatches as diagnostics.
3. Bind X8 at 3DOF and pseudo-6DOF first; preserve existing physical-surface
   evidence separately.
4. Validate B747, A320, and F-16 at their supported tiers using the same
   semantic mission and truth gates.
5. Generalize the compiler contract to gliders, rockets, rotorcraft,
   transition VTOL, orbital spacecraft, and ballistic bodies only after their
   family-specific capability variables and terminal semantics are defined.

## Exit criteria

- A new airbreathing vehicle can produce a reproducible first mission from a
  capability profile and selected fidelity.
- Route geometry and horizon are derived before simulation and explain why a
  request is clipped or infeasible.
- The same semantic mission template runs across X8, B747, A320, and F-16
  without copied route geometry.
- Objective semantics, controller path, evidence tier, and nonclaims remain
  explicit in every run artifact.
- Current source-grounded qualification baselines are not overwritten merely
  because a planner generated a plausible route.

## Candidate promotion contract

The compiler produces a candidate binding, never an automatic replacement.
The candidate has a SHA-256 fingerprint over the complete resolved route.  An
executable artifact must echo that fingerprint, candidate binding ID, and
fidelity, and independently show all required truth objectives, hard gates,
and numerical checks passing.  Only then is it *promotion eligible*; separate
family evidence and robustness gates still apply.

```bash
PYTHONPATH=src python3 tools/assess_powered_fixed_wing_mission_promotion.py \
  --profile x8-cruise --fidelity pseudo_6dof_kinematic_bridge
```

Without `--execution`, the result is deliberately
`candidate_pending_execution`.  This makes the next integration task clear:
teach the selected vehicle runner to emit the exact proposal fingerprint and
run its normal independent evaluator against the generated route.

## Execution-adapter rollout and current findings

The initial adapter rollout proves the distinction between *route generation*
and *vehicle execution* is workable:

| Vehicle | Candidate execution seam | Current result | Boundary |
| --- | --- | --- | --- |
| F-16 | `tools/validate_f16_racetrack.py --candidate-profile f16-subsonic` | pseudo-6DOF candidate passes all four independent gates and is promotion-eligible | named response bridge only; no physical-effector promotion |
| A320 | `tools/validate_a320_racetrack.py --candidate-profile a320-cruise` | pseudo-6DOF candidate passes all four independent gates and is promotion-eligible | OpenAP plus declared rotational surrogate; no manufacturer or actuator claim |
| X8 | disposable language-backed candidate materializer | pseudo-6DOF candidate passes all four independent gates and is promotion-eligible | named kinematic attitude bridge; no physical-elevon promotion |
| B747 | disposable language-backed candidate materializer | pseudo-6DOF candidate passes all four independent gates and is promotion-eligible | named kinematic attitude bridge; no physical-surface promotion |

The A320 candidate exposed a generic terminal-acquisition defect: after the
declared course it could continue along the final tangent through the simulation
margin without crossing the oriented finish gate.  The runner now explicitly
acquires that gate after nominal course completion and then holds the terminal
state.  This is route semantics, not controller tuning.

The language-backed candidate materializer now creates a temporary problem and
temporary racetrack catalog from a candidate, replaces the runtime route and
time-stop attributes, derives objective windows and the maximum step budget
from the resolved phase timeline, runs the normal packet builder, and records
the candidate fingerprint in the resulting evidence.  It never edits a
checked-in baseline `.prb`, catalog binding, or acceptance file during a
candidate run.  The B747 result also established that mission intent may be
fidelity-specific: its reduced lanes retain the 100–250 m trim corridor while
the direct-wrench lane retains its separately declared 500–650 m corridor.
