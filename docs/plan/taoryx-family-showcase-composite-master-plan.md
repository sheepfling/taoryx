# TAORYX Family Showcase Composite Master Plan

**Status:** Alpha 2 extension / Alpha 3 execution plan  
**Schema target:** `taoryx.showcase/v1alpha1`  
**Proof unit:** a Family Flagship Qualification Pack, not a single image

## Decision

Taoryx needs one common showcase pipeline for every vehicle family. A
showcase is a semantic mission resolved through a vehicle binding and a named
fidelity realization. The visible composite is generated from the same
machine-readable run artifact as the metrics, event timeline, and replay
checks.

The four existing anchors—B747, Skywalker X8, Hummingbird, and X-15—are the
first migration set. New families must use the same path; a new vehicle may
declare a fidelity as unsupported or not qualified, but it must not receive
placeholder physics merely to fill a plot.

## Alpha 3 priority tranche: B747 and X-15

The current four-family composites remain useful integration baselines, but
they are not interchangeable family-qualification evidence. The next two
showcases should deliberately span opposite ends of the mission spectrum:

- **B747:** a slow, transport-scale energy-management and stabilized-arrival
  mission. Its visible proof is climb/cruise/descent geometry, true airspeed,
  Mach, specific energy, fuel, lift/drag/thrust, and a simultaneous arrival
  gate. It must not be reduced to a larger X8 route or an aerobatic bank demo.
- **X-15 / boost-glide:** a multi-phase mission storyboard. Its visible proof
  is the ordered carrier → release → boost → burnout/coast → entry → glide →
  terminal sequence, state evolution at each event, and parent/child object
  lineage when a spent stage is represented.

These two recipes are the first templates for the reusable showcase surface.
They are recorded in
[`verification/showcase_archetype_catalog.yaml`](../../verification/showcase_archetype_catalog.yaml)
and validated by `tools/audit_showcase_archetype_catalog.py`.

## Common showcase archetypes

Every family recipe composes a subset of the same five proof products:

| Archetype | Core question | B747 emphasis | X-15 emphasis |
|---|---|---|---|
| `mission_geometry` | What path did it fly? | altitude-vs-downrange and large-radius route | phase-colored boost/coast/entry/glide path |
| `mission_timeline` | What happened, and in what order? | climb, cruise, turns, descent, arrival | release, ignition, max-Q, burnout, separation, apogee, entry, handoff |
| `dynamics_and_resources` | How did the vehicle achieve it? | energy, fuel, throttle, lift/drag/thrust | Mach, dynamic pressure, energy, acceleration, mass |
| `envelope_and_qualification` | Was the result valid and how close were the limits? | alpha/lift/thrust/fuel/Mach/table margins and arrival residuals | Q/load/alpha/Mach/energy/terminal-corridor margins |
| `object_lineage` | Which objects were active after release or separation? | not required | carrier, X-15, spent booster, glide/terminal bodies |

The renderer must consume the same run artifact for all five products. Missing
physics is shown as `unavailable`; it is never synthesized to complete a
layout. Object lineage is an accepted-boundary record with stable object IDs,
parent IDs, spawn/separation events, active intervals, and terminal
dispositions—not a decorative icon layer.

## Alpha 2 extension: M0 foundation

Alpha 2 remains closed at `A2-CLOSEOUT-PASS`. This is a post-closeout extension
that adds the common showcase surface without reopening the validated release
claim.

### A2-S1 — Showcase contracts

Add typed contracts for:

- `FamilyShowcaseTemplate` — family-independent mission semantics;
- `VehicleShowcaseBinding` — data package, start state, limits, evidence grade,
  and supported segments;
- `FidelityShowcaseRealization` — state, command mapping, available physics,
  and nonclaims for 3DOF, named pseudo-6DOF, or rigid-body 6DOF;
- `ShowcaseRunArtifact` — immutable run identity, telemetry, controls,
  resources, events, evaluation, plots, and provenance;
- stable failure codes and family-local segment/objective timelines.

Exit: contracts reject duplicate segments, unsupported fidelity claims,
timeout-as-success terminal contracts, missing units on numeric metrics, and
unresolved required artifacts.

### A2-S2 — Standard evidence board

Add a reusable renderer that emits, from one telemetry source:

1. fleet index card;
2. 4K evidence board;
3. family-local objective/event timeline;
4. optional interactive report manifest.

The evidence board must include complete mission geometry, altitude/speed or
orbital state, energy/resources, AoA/bank/sideslip, FPA/heading, controls and
achieved effectors, envelope/closure/terminal evidence, and an explicit
claim/nonclaim footer. Missing channels are rendered as `unavailable`, never
as zeros.

### A2-S3 — Plot semantics and artifact audit

Freeze the metric dictionary and event vocabulary. Every displayed scalar must
be recomputable from included telemetry. Waypoint and mode markers belong only
to the owning family lane; cross-family dashboards may compare scores but may
not imply shared event times.

Exit: a synthetic reference run rebuilds all card/board numbers and the packet
audit rejects stale plots, missing source telemetry, local absolute paths,
unhashed files, and mismatched run identities.

## Alpha 3 milestones

### A3-M1 — Rebuild the four anchors

Create full family packs for:

- **B747 first:** airborne trim → climb to cruise → cruise stabilization →
  large-radius right turn → long cruise leg → second turn → managed descent →
  multidimensional stabilized-arrival gate;
- **X-15 second:** carrier/release → ignition and powered segment → burnout/coast
  and apogee → atmospheric entry → glide capture → energy management → terminal
  handoff, with object-lineage evidence for any detached booster;
- Hummingbird: spool → takeoff → hover → 3D waypoint box → yaw scan → gust
  recovery → return → precision landing/disarm;
- X8: stabilize → climb → right/left route legs → altitude and speed changes
  → return/recovery;

The existing Hummingbird and X8 nominal packets remain regression and
integration evidence until their family-specific mission contracts are
re-run under the same archetype renderer. A controller transition or scalar
score cannot promote an unevaluated objective.

Exit: each pack has a complete lifecycle, named events, achieved controls,
terminal semantics, convergence, replay, and family-appropriate claim limits.

### A3-M2 — Fixed-wing breadth

C172-class light GA, F-16 reference, cruise vehicle, and ISR drone use the
powered-fixed-wing templates. Their missions remain separate from the
transport and hypersonic claims.

### A3-M3 — High-energy and ballistic breadth

Add staged rocket, HL-20/glider, hypersonic corridor, ballistic body, and
tumbling-body templates. A tumbling claim requires rigid-body 6DOF; 3DOF and
pseudo-6DOF may only be labeled as center-of-mass or prescribed-orientation
reductions.

### A3-M4 — Vertical lift and transition modes

Add R44/UH-1H/UH-60A helicopter packs and XV-15/V-22-class tiltrotor packs.
The transition contract must cover entry/exit guards, lift sharing, mixer
schedules, rate limits, abort/reversion, state/resource continuity, and
forward/reverse conversion replay.

### A3-M5 — Autonomous and public-surrogate overlays

Add fixed-wing ISR, fleet/swarm, and Anduril public-data surrogate overlays.
Each surrogate must separate public facts, engineering assumptions,
uncertainties, sensitivity, and nonclaims.

### A3-M6 — Space and actuator-specific packs

Add orbital point mass, reaction wheels, magnetorquers, RCS free flyers, and
hybrid wheel/thruster spacecraft. Their plots and terminal contracts must be
domain-specific: orbital elements, magnetic authority, momentum, pulses,
keep-out, and resource use—not aircraft metrics copied into space.

### A3-M7 — Cross-fidelity and robustness

Run the same semantic mission at every advertised fidelity. Report terminal,
event-time, path, energy, resource, and envelope disagreement; do not demand
identical trajectories. Add fixed perturbation suites first, then archived
Monte Carlo seeds with failure taxonomy and retained witnesses.

### A3-M8 — Pickup-ready catalog

Provide one command to build, verify, and report the catalog. Every promoted
family has an immutable manifest, checksums, reproducibility command, evidence
badge, and exact claim/nonclaim language.

## Common evidence board contract

The standard board uses these sections:

| Section | Required evidence |
|---|---|
| Geometry | full 3D/path view, commanded route, start, events, waypoints, terminal inset |
| Mission/energy | altitude or orbital state, speed/Mach, range, energy, resources, segment timeline |
| Attitude/control | AoA, bank/local roll, sideslip; FPA and heading; commands, achieved controls, effectors, limits |
| Validity/terminal | table/state/resource margins, closure, convergence, batch/step parity, objective residuals |

The distinction between bank and roll is explicit: `bank_achieved` is the
navigation/aerodynamic lift-vector bank when available; `local_roll` is the
body attitude angle in the declared local frame. A plot may use local roll as
a fallback only when it labels that fallback.

Event markers are family-local. A route corner, nacelle transition, stage
separation, ProNav handoff, touchdown, or orbital maneuver is drawn on that
family's timeline and panels only. A fleet scoreboard may compare completion
status and scores, but it must not draw one vehicle's waypoint markers across
another vehicle's row.

## Release badges

`Nominal Showcase Complete` → `Flagship Mission Qualified` →
`Fixed-Matrix Qualified` → `Release-Robustness Qualified` →
`Multi-Fidelity Qualified` → `Pickup Ready`.

No image alone can advance a badge. Each badge links to the raw telemetry,
metric dictionary, source/provenance record, convergence, replay, and
robustness artifacts that justify it.

## Final definition of done

A family showcase is complete when a versioned vehicle and fidelity begin from
a documented family-appropriate start, execute a coherent mission with named
segments and events, exercise the controls available at that fidelity, remain
inside declared data/state/actuator/resource/numerical envelopes, reach an
explicit terminal set rather than timing out, and reproduce a self-contained
artifact pack containing telemetry, controls, effectors, resources,
provenance, evaluation, convergence, robustness, plots, and exact claim
boundaries.
