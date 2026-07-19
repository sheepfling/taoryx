# Vehicle maneuver matrix: current evidence boundary

The machine-readable matrix is
`verification/vehicle_maneuver_matrix.yaml`; native problem-file bindings are
in `verification/maneuver_evidence.yaml`. Run the short evidence packet with:

```text
.venv/bin/python tools/run_maneuver_matrix.py \
  --status available,in_progress \
  --only b747/trim-hold-3dof,b747/trim-hold,b747/altitude-step,\
b747/heading-doublet,b747/descent-corridor,\
skywalker-x8/powered-trim-hold-3dof,skywalker-x8/powered-trim-hold,\
skywalker-x8/heading-doublet,skywalker-x8/descent-corridor,\
hummingbird/hover-3dof,hummingbird/hover,hummingbird/takeoff-hover-land,\
hummingbird/yaw-step,hummingbird/wind-gust,hummingbird/degraded-rotor,\
x15/powered-ascent,x15/release-coast,x15/terminal-waypoint \
  --output artifacts/vehicle-maneuver-matrix
```

The runner writes fingerprinted per-binding summaries, native run reports, and
standard plot bundles below the ignored
`artifacts/vehicle-maneuver-matrix/` directory. Later partial invocations reuse
only summaries whose problem, table, and binding fingerprints still match, so
the consolidated report cannot silently mix stale evidence with a new fixture.
It reports native point-mass output in both its historical ft/ft·s form and
canonical SI form.

To rebuild only the consolidated view after individual runs have completed,
use `--aggregate-only`. The report records both binding-level outcomes and a
row-level `passing_matrix_rows` count. A row passes when at least one of its
declared dimensions has a completed, limit-respecting binding; unsafe sibling
dimensions remain visible and are not erased.

## Current interpretation

- B747 3-DOF trim, B747 6-DOF trim, B747 heading response, X8 3-DOF
  powered-trim probe, X8 6-DOF powered response, X8 lateral response,
  Hummingbird 3-DOF hover, Hummingbird 6-DOF hover, Hummingbird short
  takeoff/waypoint, yaw, and wind probes reach their declared stop conditions.
- B747 descent is correctly classified unsafe when the trajectory leaves its
  local alpha envelope; completion of the integrator is not treated as a
  successful descent.
- Hummingbird degraded-rotor evidence is unsafe when the rigid-body safety
  guard terminates at Earth intersection.
- X-15 powered-ascent, release/coast, bank-energy, and terminal bindings now
  complete the declared 50-second native 6-DOF matrix prefix. The separate
  3-DOF point-mass reductions run their 180-second staging/coast schedule but
  are classified unsafe when they cross the declared ground boundary. Neither
  is full Hawaii-mission evidence.
- The paired B747, X8, and Hummingbird point-mass route reductions now use the
  shared `*runtime status route` contract and the same source tables where the
  reduction is defined. Their results remain reductions, not attitude-control
  validation.
- The B747 and Hummingbird rectangle/square cases have native bindings and
  generated long-run artifacts. The B747 point-mass rectangle completes its
  320-second horizon but slows to 14.58 m/s against a 20 m/s minimum. The X8
  point-mass rectangle completes 120 seconds but slows to 1.55 m/s against a
  2 m/s minimum; its 6-DOF counterpart completes 120 seconds but reaches 7.99°
  sideslip against a 5° envelope. These are completed-but-unsafe maneuver
  results, not default short-packet passes.
- The three former landing/go-around gaps now have native 6-DOF
  `non_contact_approach_checkpoint` bindings and pass their declared runtime
  and envelope checks. They do not model runway contact, ground effect, stall,
  landing gear, or certified thermal behavior.

The matrix status is therefore a research-surrogate execution inventory, not a
claim of TAOS 96 compatibility, flight qualification, or globally valid
vehicle models.

The current consolidated packet contains 43 binding summaries: 36 completed
within declared limits and 7 completed-but-unsafe. The unsafe cases are
preserved as useful diagnostics. All 24 matrix rows now have a completed,
limit-respecting binding; the three landing-related rows pass only as
explicitly non-contact checkpoints.

The current coverage audit is 24 of 24 rows fully dimension-bound and 24 of 24
rows passing at the row level, with zero actionable dimension gaps.
