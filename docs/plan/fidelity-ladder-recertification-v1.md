# Fidelity-ladder recertification plan

This plan separates two claims that were previously mixed in the same
overview image.

## A. Reduction-parity packet

For each family, create one scenario contract shared by the true 3-DOF,
pseudo-6-DOF bridge, and constrained rigid-body runs. The contract must hash or
identify the same:

- canonical-SI initial state, mass, and inertia;
- atmosphere, wind, source tables, propulsion, and mass-flow model;
- command history and event schedule;
- duration, integrator sampling, and termination policy.

Only the dynamics tier may differ. A plotter must refuse to call a comparison
parity when any of those fields differ. The current long showcase runs are
explicitly diagnostic because their contracts differ; their red warning is
intentional.

Required outputs per family:

1. initial-state round-trip audit;
2. source/table query audit;
3. true 3DOF versus pseudo-6-DOF bridge parity;
4. constrained 6DOF versus 3DOF parity over the declared window;
5. continuity/event audit;
6. dt, dt/2, and dt/4 comparison.

## B. Full-mission packet

The free rigid-body run is a separate experiment. It should not be expected to
overlay the reduction. It must instead show:

- source-anchored trim or equilibrium;
- independent force and moment closure;
- model-envelope margins and termination reason;
- actuator commands versus achieved values;
- convergence against a refined or adaptive reference;
- a long, benign maneuver appropriate to the family.

The minimum family missions are:

| Family | Mission evidence |
| --- | --- |
| B747 | 120 s trim, altitude step, heading step, recovery |
| X8 | powered trim, collective/differential doublets, 60 s recovery |
| Hummingbird | takeoff, hover, square translation, yaw step, return, landing |
| X-15 | release, unpowered glide, powered/coast segment, descent or ground event |

Each mission must stop at ground contact or another declared termination
condition. It must never continue below ground or silently splice independent
segments. Actual state channels must be plotted separately from commands and
references.

## Current evidence boundary

The current packet has useful long mission diagnostics and a valid 3DOF to
kinematic bridge check. It does not yet establish four-family reduction parity:
the current contract audit correctly reports 3DOF-to-6DOF mismatches in source
problem, table set, unit profile, and/or duration. No engineering-validity or
historical TAOS compatibility claim is made.

The next implementation tranche is to generate the four shared parity cases
from the vehicle catalog, then run the mission packet independently. Problem
files remain generated artifacts of catalog metadata; they are not hand-tuned
per-test replacements.

## C. Current native maneuver evidence

The native maneuver matrix now has a reproducible packet builder:

```text
python tools/build_maneuver_evidence_packet.py
```

The packet includes the matrix and binding catalogs, redacted run reports,
plots, termination events, and SHA-256 hashes. The current matrix contains 24
declared rows and treats four X-15 point-mass descent cases as
`completed_ground_terminated`; those are valid bounded terminal events, not
ordinary completed flight claims. Point-mass geodetic runs now share the rigid
body fail-closed ground-intersection guard. Rectangle point-mass reductions
also use the declared `rectangle-speed-mps` rather than slowing toward zero as
the remaining leg distance decreases.

## D. Family-scoped execution

The same packet builder can be used for a single vehicle while tuning or
debugging a new adapter:

```text
python tools/build_fidelity_ladder_packet.py --family skywalker_x8 --no-plots
```

Controller evidence can be narrowed independently by catalog id:

```text
python tools/build_fidelity_ladder_packet.py \
  --family hummingbird \
  --controller hummingbird-rate-damped-hover
```

The selected packet retains the same ladder schema, source-differential report,
closure contract, input copies, and hashes as the full packet. Vehicle ids in
`verification/controller_scenarios.yaml` bind controller cases to the common
vehicle catalog; adding a vehicle therefore requires catalog metadata and
problem/table bindings, not a new runner or bespoke evidence container.

Each executed controller entry also receives an `expectation_evaluation` in
the packet manifest. The evaluator applies the catalog's declared duration,
alpha/beta and body-rate limits, altitude/speed limits, final range or
altitude limits, guidance requirement, saturation requirement, and motor
shutdown requirement to the summarized raw telemetry. Every check records its
actual value, comparator, limit, and result. This makes a family packet useful
for a new vehicle without requiring reviewers to reconstruct pass/fail status
from plots or from a separate test implementation.

After the family runs are complete, create a review bundle without rerunning
them:

```text
python tools/build_fidelity_ladder_rollup.py
```

The rollup embeds the four family packet ZIPs and records each source packet's
run id and SHA-256. It is a packaging index, not a new validation claim.

The parity report now keeps the experiments separate:

- `parity_gate` is the long, metadata-generated true 3-DOF versus pseudo-6-DOF
  bridge
  reduction window;
- `rigid_window_max_difference` is the short constrained/free rigid-body
  comparison and is reported diagnostically rather than used to certify the
  reduction;
- convergence includes 3-DOF, bridge, and rigid-body step refinement.

The current reduction windows are 10 s for B747, 5 s for X8, 10 s for
Hummingbird, and 30 s for X-15. All four pass the translational reduction gate;
this does not promote the free rigid-body cases to full mission validation.

The controller catalog now also contains the X-15 source-trim to native ProNav
transition. Its current evidence is a 35-second bounded transition with ProNav
active, no actuator saturation, maximum sideslip about 5.02 degrees, and final
route error about 3.0 km. This is controller-transition evidence, not a
successful Hawaii arrival claim.
