# TAORYX Alpha 2 execution goal

**Status:** Complete — `A2-CLOSEOUT-PASS`  
**Release boundary:** Existing proof families only  
**Primary backlog:** [`taoryx-alpha-2-backlog.md`](taoryx-alpha-2-backlog.md)

## Goal statement

Complete Taoryx Alpha 2 as a reusable, evidence-bounded vehicle-family
platform for the established B747, Skywalker X8, Hummingbird, and X-15
families. A resolved case must run through one provider-neutral scenario,
control, evaluation, replay, and evidence path; use canonical units and frames;
retain immutable provenance; distinguish requested from achieved controls and
resources; and report validity, qualification, feasibility, outcome, closure,
convergence, and claim boundaries explicitly.

The four families must each have a reproducible, family-appropriate flagship
mission at the fidelities they actually support. Three-DOF, pseudo-6-DOF, and
rigid-body 6-DOF results are separate claims: unsupported or non-comparable
tiers remain diagnostic or blocked rather than being promoted by a composite
score.

## Exit gates

Alpha 2 is complete only when every gate below has checked-in implementation,
tests, machine-readable evidence, and a reproducible command:

1. **Common evaluation:** scenario contracts, objective results, event markers,
   requested/achieved controls, resources, closure, convergence, and neutral
   `TrajectoryEvaluation` reports are the single evidence route.
2. **Interactive and replay:** batch and stepwise execution share the public
   transition; pause/resume, interrupt, checkpoint, branch, and replay preserve
   state, controls, events, diagnostics, and provenance.
3. **Convention firewall:** a new family follows one onboarding sequence with
   actionable diagnostics for tables, units, frames, attitude, aerodynamic
   angles, margins, dimensional loads, mass properties, trim, propagation, and
   convergence.
4. **Bounded variants:** semantic modifiers, coupled derivations, hard versus
   qualified ranges, reject/project policy, resource checks, retrim flags, and
   immutable fingerprints are enforced before integration.
5. **Trim and controllers:** the four established families use the common
   bounded trim procedure and plant-scaled gentle/standard/aggressive control
   profiles. LQR is a baseline, not a permanent controller architecture.
6. **Flagship evidence:** B747, X8, Hummingbird, and X-15 each produce a clean
   UUID-scoped packet with raw telemetry, inputs, controls, events, objective
   results, table/envelope margins, closure, convergence, and a claim ledger.
7. **Release audit:** a clean-process replay regenerates the declared packet,
   all referenced files are hashed and resolvable, and the repository passes
   the required manual, equation-audit, check, and pytest commands.

## Explicit non-goals

Alpha 2 does not promise:

- historical TAOS 96.0 runtime compatibility without the original executable
  and complete table library;
- flight qualification, certification, or global physical validity;
- implementation or qualification of Cessna, Learjet, helicopter, tiltrotor,
  spacecraft, passive-body, F-16/HL-20, Anduril-surrogate, sensor, weather,
  fleet, or large corpus families;
- unrestricted raw table morphing, topology changes, geometry generation, or
  large-scale optimization;
- universal controller performance across all vehicle families or fidelities.

Those are Alpha 3 or later work. Alpha 2 may retain their interface probes,
research intake records, and maturity entries so that they stress the
contracts without blocking this release.

## Honest completion signal

The release signal is `A2-CLOSEOUT-PASS`, and it is valid only when the exit
gates above and the machine-readable required-item list in
`verification/alpha2_post_release_backlog.yaml` agree. A passing local plant,
bounded trajectory, or weighted score alone is not an Alpha 2 completion
signal.

## Closeout evidence

The final primary packet and clean-source replay were generated with
`tools/build_fidelity_ladder_packet.py`, audited with
`tools/audit_fidelity_packet.py`, and compared with
`tools/audit_fidelity_milestones.py`. The comparison passed M0--M6, including
matching four-family objective scores, controller-mission evaluations,
closure/convergence gates, hashes, and snapshot provenance. The generated
packets are release artifacts rather than checked-in source files.
