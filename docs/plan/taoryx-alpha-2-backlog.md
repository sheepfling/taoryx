# TAORYX Alpha 2 post-release backlog

**Status:** Active planning after the frozen Alpha 2 core release  
**Baseline:** `A2-RELEASE-PASS`  
**Machine-readable inventory:** [`verification/alpha2_post_release_backlog.yaml`](../../verification/alpha2_post_release_backlog.yaml)

## The new boundary

Alpha 2 core is complete. Its release proves the successor-side configuration,
provider lifecycle, control-authority, fidelity-composition, dual-launch, and
evidence-packet contracts for the checked-in proof families.

The repository has since accumulated several larger plans: interactive Lab
tasks, scored missions, generic vehicle onboarding, four-family flagship
flights, F-16/HL-20 reference anchors, reachability, parametric corpora, fleet
execution, spacecraft, sensors, and weather. These are not additional Alpha 2
release gates. They are ranked post-release work and must not be allowed to
silently widen the Alpha 2 claim.

## Release boundary

The post-release backlog is now split into two release lanes. Alpha 2 closes
the reusable platform and the four established proof families. Alpha 3 owns
new vehicle breadth and deeper source correlation.

### Alpha 2 finish line

Alpha 2 closes only after P0-1 through P0-5, P1-1, P1-2, and the bounded
source-anchor tranche P1-3 are complete:

- one common evaluator and evidence packet route;
- deterministic interactive stepping, replay, and checkpoint/restart;
- generic vehicle onboarding and convention firewall;
- future-family interface stress test covering components, resources, modes,
  allocation, channel semantics, and evidence metadata;
- reproducible B747, X8, Hummingbird, and X-15 flagship missions;
- normalized plant-truth controller presets and tuning;
- M4 Composable maturity for the four established proof families.
- F-16 S-119 and HL-20 Mod K source-grounded reference anchors that replay
  their pinned DAVE-ML plants and source regressions through the normal
  Taoryx path.

The finish line does not require implementing every vehicle now in intake. It
permits their source records and maturity entries to exist while keeping their
runtime claims in Alpha 3.

### Alpha 3 boundary

Alpha 3 begins with the C172, R44, UH-1H, UH-60A, XV-15, V-22-class, Learjet,
spacecraft, and public-surrogate pilots. It then adds source correlation,
hybrid transitions, search/reachability, trajectory corpora, fleet scale,
sensors/weather, and stronger reference anchors. See
[`taoryx-alpha-3.md`](taoryx-alpha-3.md) and the machine-readable
[`vehicle_maturity_registry.yaml`](../../verification/vehicle_maturity_registry.yaml).

## Ranking rule

Work is ordered by how much it reduces future rework:

| Priority | Meaning | Decision rule |
| --- | --- | --- |
| P0 | Alpha 2 platform closeout | Close generic execution, evaluation, onboarding, and replay seams. |
| P1 | Alpha 2 proof closeout | Prove the four established families and normalized controller tooling. |
| P2 | Alpha 3 domain pilots | Add new vehicle families only after the Alpha 2 closeout gates pass. |
| P3 | Alpha 3 scale and qualification | Qualify fleets, corpora, spacecraft breadth, sensors, and weather. |
| P4 | Deferred external dependency | Historical compatibility remains blocked until an oracle is acquired. |

## Recommended execution sequence

### P0 — Close the generic seams

1. **Common scenario and evaluation substrate**
   - Freeze scenario identity across fidelity tiers.
   - Score objectives with units, tolerance, slack, normalized error, and
     severity.
   - Audit continuity, events, table margins, saturation, command rates,
     closure, and convergence.
   - Keep requested controls, achieved actuator state, resource observables,
     and event predictions distinct in schemas and score inputs.
   - Reject incomparable plots and prevent weighted scores from hiding failed
     required gates.

2. **Interactive Lab and control session**
   - Finish the generic reset/step/run/pause/resume/interrupt surface.
   - Make action-to-applied-control provenance complete.
   - Support per-channel absolute and requested-rate inputs, while retaining
     hard actuator rate limits as a separate physical constraint.
   - Keep reward, termination, truncation, randomization, and evaluation in
     task metadata rather than vehicle physics.
   - Make batch and interactive paths use the same public transition.

3. **Generic model onboarding and convention firewall**
   - Make the integration record, source/evidence classes, table catalog,
     unit/frame adapters, trim, closure, and bounded propagation one prescribed
     path for every family.
   - A new vehicle should be diagnosable from missing metadata or a failing
     convention stage without reading a bespoke runner.

4. **Checkpoint/restart and reproducible run store**
   - Save accepted states, controls, events, diagnostics, model hashes, and
     numerical settings at legal boundaries.
   - Prove resumed trajectories against uninterrupted trajectories.

P0 is complete only when a new family can enter through metadata plus a normal
provider binding, run through the same session/evaluation path, and produce a
replayable evidence packet.

### P1 — Prove the current library

5. **Four-family flagship missions**

Run these in order because their mechanics and debugging cost increase:

| Order | Family | Proof mission |
| --- | --- | --- |
| 1 | Hummingbird | Takeoff, hover, 3D box, yaw, disturbance recovery, landing. |
| 2 | Skywalker X8 | Powered trim, altitude/speed changes, rounded course, return/recovery. |
| 3 | B747 | Airborne trim, climb, opposing turns, speed change, descent, stabilized arrival. |
| 4 | X-15 | Air release, source-supported powered/coast or glide segment, energy corridor, valid terminal event. |

Each mission needs a family-specific start and terminal contract, long enough
segments to expose mechanics, objective/event markers, command and actuator
telemetry, envelope margins, and a claim ledger. A finite trajectory or loose
terminal radius is not sufficient.

6. **Normalized controller presets and tuning**

Use plant-truth trim and linearization, mass/inertia/reference-geometry
scaling, control-direction probes, and explicit gentle/standard/aggressive
profiles. LQR is the first baseline, not a permanent architectural lock-in;
SciPy or historical solver adapters remain selectable and provenance-linked.

7. **F-16 and HL-20 DAVE-ML reference anchors**

Integrate their immutable source plants through the same catalog and firewall.
The Alpha 2 slice is deliberately narrower than full pickup readiness: it
requires pinned collections, native DAVE-ML replay, source check-case and
trim/hold evidence, direct-control plant artifacts, and reproducible hashes.
Add actuator, allocation, controller, mission, and reduction layers separately
after this anchor gate. Keep source-grounded reference claims distinct from
public-data and synthetic surrogate claims. A320 remains an intake record and
is not an Alpha 2 completion dependency because its exact source package is
still unavailable.

P1 is complete only when each claimed family mission is reproducible from a
resolved case and its lower-level plant, convention, and controller evidence.

### P2 — Alpha 3 domain pilots, search, and coherent breadth

8. **Spacecraft reference family**

Add the first orbital/attitude spacecraft provider through the same catalog,
session, checkpoint, telemetry, and evaluation contracts. Prove epochs,
frames, force-model provenance, resource accounting, wheel/thruster authority,
and the separation between orbital 3DOF, attitude-response pseudo-6DOF, and
rigid-body spacecraft 6DOF.

9. **Cessna-class light propeller-aircraft pilot**

Use the JSBSim C172X/R approximation as the operational baseline, UIUC C172
nonlinear/tabulated models as the independent cross-check, and NASA Cessna
177B performance and dynamic-response data as validation anchors. This family
adds low-speed, propeller, flap, gear, ground-contact, takeoff, landing, and
stall behavior that the current library does not exercise well. The current
research intake is recorded in
[`verification/small_aircraft_research_intake_v1.yaml`](../../verification/small_aircraft_research_intake_v1.yaml);
it is a candidate source package, not yet an opaque truth blob. First claims
are sub-stall and stall-onset pseudo-6DOF behavior; spins, deep stall,
authoritative propwash, and realistic gear dynamics remain excluded.

10. **Rotorcraft and VTOL transition pilot**

Add helicopter hover/forward-flight behavior and a generic V-22-like tiltrotor
transition profile. Treat nacelle/tilt scheduling, rotor allocation, mode
handoff, and achievable-wrench changes as first-class segment contracts. This
is a generic configuration pilot, not an exact production V-22 model. The
current close-out intake is recorded in
[`verification/rotorcraft_tiltrotor_research_intake_v1.yaml`](../../verification/rotorcraft_tiltrotor_research_intake_v1.yaml)
and defines the tranche explicitly:

- R44-class: source-identified hover anchor plus a labeled forward-flight
  schedule.
- XV-15-class: primary public conversion and control-mixing anchor.
- V-22-class: scaled transport surrogate using public envelope facts and
  sensitivity-bounded engineering estimates.

The rotorcraft tranche now also includes the candidate UH-1H/UH-60A scheduled
pseudo-6DOF package. UH-1H is anchored to NASA TM-73254 and CR-3144; UH-60A is
anchored to NASA TM-85890, with GenHel, real-time validation, Airloads, and
T700 engine references retained as later evidence gates. The package is
appropriate first for source-point interpolation, trim, pulse/step response,
guidance, and sensor studies. It is not yet a full nonlinear rotorcraft or
flight-test validation model.

The first close-out mission is R44 pad-to-pad. The next is XV-15 forward and
reverse conversion with abort/reversion. The V-22-class case follows only after
the shared transition contract is qualified.

11. **Spacecraft reference family**

Add the first orbital/attitude spacecraft provider through the same catalog,
session, checkpoint, telemetry, and evaluation contracts. Prove epochs,
frames, force-model provenance, resource accounting, wheel/thruster authority,
and the separation between orbital 3DOF, attitude-response pseudo-6DOF, and
rigid-body spacecraft 6DOF.

12. **Small-business-jet pilot**

Use Stengel’s generic business-jet models as the primary research anchor, the
UIUC Learjet 24 derivative model as an independent fixture, and the JSBSim
Global 5000 as an operational supplement. Keep low-alpha Mach-scheduled and
high-alpha low-subsonic profiles in separate validity envelopes; do not begin
with a Citation X reconstruction. The current Learjet 24-class intake is
listed beside the C172 record in the small-aircraft research ledger. Its
`source_linear` mode is for local source reproduction; its `bounded_proxy`
mode is an explicitly estimated continuity aid and cannot support stall,
certified performance, or global-flight claims.

13. **Anduril-inspired flight-dynamics surrogate pilot**

Implement the public-data surrogate library as a shared-equation domain pilot.
Use evidence grades, sensitivity bands, explicit unknowns, and common
controller/actuator contracts. These are generic public surrogates, not exact
Anduril vehicle models or classified-performance claims.

The next increment is the energy-coupled, mode-aware proxy tranche. It is not
complete when a vehicle can take a kinematic step. It is complete only when
the same resolved parameter pack can drive both a 3-DOF mission model and a
named pseudo-6-DOF response model, with resource state, actuator state, mode
ownership, and claim boundaries preserved in the run artifact. The data and
implementation contract is maintained in
[`verification/anduril_surrogate_parameters_v1.yaml`](../../verification/anduril_surrogate_parameters_v1.yaml)
and the contributor path is recorded in
[`anduril-surrogate-integration-notebook.md`](anduril-surrogate-integration-notebook.md).

Required tranche boundaries:

- Use one common state contract containing inertial position, body velocity,
  attitude quaternion, body rates, resource state, actuator states, and mode
  states. A point-mass reduction may omit attitude/rate integration only when
  the omission is declared in the fidelity profile.
- Implement selectable energy backends: battery-electric, fuel-burning, and
  series-hybrid. Battery depletion changes available power but not mass;
  fuel depletion changes mass and may change CG and inertia; a series hybrid
  must schedule generator and battery power rather than draining fuel in
  direct proportion to rotor power.
- Model flight modes as explicit state-machine transitions with entry
  conditions, blend variables, exit conditions, abort conditions, and
  resource reserves. Roadrunner, Omen, and Thunder must not be represented by
  a discontinuous hover/fixed-wing switch.
- Tag promoted numeric parameters individually as `P`, `D`, `E`, or `S`, and
  retain source references, units, uncertainty bands, and sensitivity policy.
  The parameter pack's compact fields are compatibility views; the parameter
  ledger is the review surface.
- Separate vehicle configuration from mission loading state. Public maximum
  payload, speed, range, and endurance values must not be combined into one
  impossible nominal configuration.
- Validate 3-DOF and pseudo-6-DOF against the same resolved vehicle instance.
  Differences must be reported as a fidelity comparison, not hidden by
  vehicle-specific tuning.

Recommended implementation order:

1. Shared resource and mass-property scheduler.
2. Battery and fuel proxy backends, including reserves and power derating.
3. Mode/state-machine contract and continuous transition blending.
4. Propulsion and aerodynamic force maps with speed/altitude dependence.
5. Allocator and actuator-lag overlays.
6. 3-DOF mission fixtures and resource monotonicity checks.
7. Pseudo-6-DOF attitude/rate fixtures and 3-DOF comparison runs.
8. Reachability, corpus generation, and sensitivity sweeps.

The first four calibration targets remain ALTIUS-600, Ghost-X, Roadrunner,
and Barracuda-250. Bolt is the smallest implementation smoke target. Omen
and ALTIUS retain selectable energy backends until public evidence identifies
their production arrangements. Thunder remains a series-hybrid concept proxy
and cannot advance beyond concept-level evidence without new public anchors.

14. **Guidance, tuning search, and reachability workbench**

Search first at 3DOF, refine with pseudo-6DOF, and validate candidates with
rigid-body 6DOF. Preserve seeds, checkpoints, controller identity, objective
reports, and the distinction between plant reachability and controller
reachability. Do not call “not found” impossible. The workbench must register
vehicle-specific profiles rather than emitting one maximum-range circle:

- Multirotors and hover-capable vehicles need stopping-distance, wind,
  battery-reserve, low-relative-speed, and moving-target products.
- Fixed-wing vehicles need velocity-relative bearing, turn-radius, left/right,
  stall/load-factor, terminal-heading, terminal-speed, and first-pass products.
- Rocket/glider systems need prelaunch, release-state, separation, boost,
  coast, glide, terminal-energy, and post-release retarget products.
- High-energy gliders need downrange/crossrange, specific-energy, heating,
  dynamic-pressure, bank-reversal, and fidelity-contraction products.
- Ballistic and passive bodies need terminal footprints and distributions,
  not controller-effective capture basins.
- Spacecraft need orbit visibility, attitude pointing, free-flyer capture,
  propellant, wheel momentum, eclipse, and line-of-sight products.

The full profile-to-family matrix is maintained in
[`verification/reachability_profile_catalog.yaml`](../../verification/reachability_profile_catalog.yaml)
and the detailed semantic contract is in
[`trajectory-reachability-workbench.md`](trajectory-reachability-workbench.md).

15. **Parametric and ML trajectory corpus**

Generate coherent realized vehicles from latent design vectors, not unrelated
random table cells. Produce paired fidelities, truth/observation editions,
identification fragments, and leakage-safe splits.

### P3 — Scale and domain qualification / Alpha 3 candidates

The reachability workbench and sensor/measurement orchestration are explicitly
Alpha 3 breadth candidates. They remain documented here because this file is
the post-Alpha-2 backlog, but neither is part of the Alpha 2 finish line.

16. **Population/fleet runtime** — Hummingbird reductions first; prove scalar/
    batched equivalence and deterministic population replay.
17. **Parametric spacecraft qualification** — qualify the source-anchored
    notional families (`6u_observer_rw.standard`, `6u_observer_rw.resilient`,
    `agile_imager_rw`, `spheres_like_rcs`, and `marco_like_hybrid`) with coherent wheel/thruster sizing, attainable-
    wrench checks, evidence labels, failure variants, and replayable flagship
    missions.
18. **Sensors, weather, and measurements** — truth staging, causal sampling,
    seeded noise/bias, wind providers, and estimator/AI truth isolation. The
    orchestration boundary is explicit: EOM/environment produces committed
    physical truth; ideal sensor projections produce observations; sensor
    error/electronics/timing layers produce timestamped packets; estimators,
    trackers, guidance, and control consume observation ports; and only latched
    commands feed the next EOM interval. This includes atomic
    `TruthPoint`/`TruthSegment` commits, stateful IMU delta-angle/delta-velocity
    adapters, focal-plane/seeker pipelines, multi-rate event alignment or
    committed dense output, latency and availability, reproducible stochastic
    streams, truth-leakage checks, and perfect-navigation versus
    sensor-closed-loop versus hardware/SWIL bindings. See
    [`sensor-measurement-orchestration.md`](sensor-measurement-orchestration.md)
    and backlog item `A2-POST-P3-3`.

These are shared infrastructure workstreams. They should not create separate
fleet, spacecraft, weather, or sensor runtimes.

### P4 — Historical compatibility

Historical compatibility is intentionally blocked. It starts only after a
historical executable, source tree, complete table library, or trusted output
corpus is acquired and hashed. Historical behavior must remain a separate
claim from manual fidelity and successor-side scientific interpretation.

## Explicitly removed from the near-term queue

- Remote model marketplaces and untrusted hot-loading.
- A universal autopilot across unrelated vehicle families.
- New vehicle-specific `.prb` dialects or copied runners.
- Runway takeoff/landing claims before low-speed, ground-contact, and gear
  models qualify.
- Stronger X-15 powered-flight claims where the source deck does not support
  the requested phase.
- ML corpus scale before coherent model realization and truth/observation
  separation are implemented.

## Exit map

| Exit | Required result |
| --- | --- |
| P0-ready | Generic session, scenario/evaluation, onboarding firewall, and checkpoint/replay path work for a new family. |
| P1-qualified | Hummingbird, X8, B747, and X-15 have family-appropriate claim-bounded flagship packets, with only supported fidelity tiers green. |
| P2-research-ready | Search/reachability and coherent parametric corpus outputs are reproducible and preserve plant/controller/source distinctions. |
| P3-domain-ready | Fleet, spacecraft, sensor, and weather additions pass their own family/domain gates without runtime forks. |
| P4-open | A historical oracle exists and a separately governed compatibility program can begin. |

The detailed machine-readable statuses and dependencies live in
`verification/alpha2_post_release_backlog.yaml`; this document is the review
surface for ordering and scope.

####
