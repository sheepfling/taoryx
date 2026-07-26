# Reachability Envelope Result Format

The first executable Alpha 3 envelope writes
`trajectory.reachability-envelope/v1alpha1`. The artifact is a complete
description of one deterministic study, not only a list of successful terminal
points.

## Required Sections

```text
schema
study
search_space
success_spec
deployment
execution
summary
samples
```

`study` identifies the vehicle, fidelity, model equations, state fields, and
resolved vehicle parameters. Parameter names include their units where
practical, such as `thrust_n` and `burn_time_s`.

`search_space` records every realized outer-search axis, its units, exact
values, sampling method, candidate composition, candidate count, and ordering.
Each sample also repeats its search coordinates and decision vector, allowing a
consumer to reconstruct the candidate without relying on array position.

`success_spec` is the terminal acceptance rule. A future study may extend this
with path constraints, target motion, uncertainty, and margin definitions while
preserving the same section.

Impact studies use the same contract with explicit fields for target position,
maximum horizontal impact radius, and minimum/maximum impact speed. Impact
constraints require a ground-contact event; a candidate that reaches the
horizon first is not treated as an impact.

## Candidate Outcomes

Every entry in `samples` is retained, whether successful or unsuccessful. Each
sample contains:

```text
query_id
search_coordinates
decision_vector
success / feasible
classification
failure_reasons
limiting_factor
terminal_margins
path_metrics
termination
terminal_state
witness
trajectory (optional table)
```

The initial classifications are `feasible`, `infeasible`, and `invalid`.
`failure_reasons` is a list so multiple violated conditions can be preserved.
The artifact does not interpret an unsuccessful finite search as a proof of
physical impossibility.

The optional `trajectory` table has a `fields` array and row-major `rows`
array. This is intentionally easy to load into a plotting dataframe. It
contains time, position, velocity, speed, mass, and phase. Pseudo-6DOF records
attitude and attitude-rate channels; rigid-body 6-DOF records the attitude
quaternion, body rates, inertia, forces, moments, projected area, and
termination diagnostics.

`summary` provides counts by classification and failure reason, explicit lists
of successful and unsuccessful query IDs, and both feasible-only and all-
evaluated terminal bounds. This supports plotting the envelope while retaining
the evidence needed to explain holes and failures.

Deployment is optional in this artifact. A standalone reachability study may
have no spawned models at all; when deployment is composed into a study, child
histories and deployment markers are recorded only for the models that were
actually created. Aero-ballistic child fields are therefore specialization
data, not required reachability fields.

`deployment` is the top-level deployment summary. It records whether the
study was configured for deployment, whether deployment was enabled for the
run, the specialization kind, child model definitions, accepted event count,
spawned-child count, and child terminal classifications. This keeps the
deployment contract inspectable without requiring a consumer to scan every
candidate first.

Horizon termination is also tracked independently of feasibility through
`timed_out` on each sample and `summary.timed_out_query_ids`. Those candidates
can be focused into a longer study without rebuilding the search manually:

```bash
taoryx reachability rerun-timeouts envelope.json \
  --horizon-s 600 \
  --output timeout-rerun.json
```

The rerun artifact records its parent study and the fact that its candidate
set was the prior timeout subset.

## Standard Plots

The Matplotlib tranche consumes this artifact directly:

```bash
taoryx reachability plot envelope.json --output-dir plots/
taoryx reachability plot point-mass.json --compare pseudo-6dof.json --output-dir comparison/
```

The standard bundle contains flown trajectories, search coverage, terminal
capability, and, when comparison artifacts are supplied, fidelity progression.
The trajectory plot requires the optional trajectory tables; the other views
remain available from a compact summary artifact.

## X-15 Demonstration

The X-15 exercise runs the same reduced-order search grid at
`point_mass_3dof`, `pseudo_6dof`, and `rigid_body_6dof` fidelity:

```bash
taoryx reachability x15 --output-dir artifacts/x15-reachability
```

The command writes one result artifact per tier, a comparable plot bundle, and
`bundle-manifest.json`. The vehicle mass and reference area come from the
registry; thrust, burn time, drag, and lift-to-drag are declared surrogate
assumptions. Each artifact records the source X-15 cases and tables plus the
claim boundary that this is an X-15-scaled reachability surrogate, not a
native X-15 rigid-body batch provider. The rigid-body tier explicitly means a
reduced-order X-15 parent with a native rigid-body spawned spent-booster child;
it is an integration and ballistic-surrogate demonstration, not a historically
validated native X-15 aerodynamic deck.

The integration decisions and friction log are maintained in
`docs/plan/x15-reachability-integration-notebook.md`. The X-15 surrogate
records explicit `boost`, `coast`, and `glide` phases, a release mass drop, and
energy path metrics rather than treating the source release state as an
unexplained initial condition.

Selected pseudo-6DOF checkpoints can be sent through the native reference
runner:

```bash
taoryx reachability x15-native-replay \
  artifacts/x15-reachability/pseudo_6dof.json \
  --output-dir artifacts/x15-reachability/native-replay
```

This writes `replay-manifest.json`, generated native problem files, and native
`RunArtifact` telemetry for replayable points. Points outside the source
aerodynamic table domain are retained as blocked records rather than omitted.

## Compatibility Rules

- Consumers must use `schema` to select a parser.
- Unknown fields may be ignored by readers.
- Existing flat fields such as `fidelity`, `samples`, and `bounds` remain as
  compatibility conveniences; the nested sections are authoritative.
- Candidate order is deterministic and must not be used as the only candidate
  identity; use `query_id` and `search_coordinates`.
- Witness trajectories are model evidence for the declared fidelity and
  parameterization, not a claim of vehicle validation.
