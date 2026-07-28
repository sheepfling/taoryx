# TAORYX Alpha 2 execution goal

**Status:** Core complete; showcase evidence closeout recorded — `A2-SHOWCASE-PACKS-PASS`
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

The established Alpha 2 release families retain their existing fidelity
claims. The final showcase closeout additionally packages the requested X8
physical racetrack, B747 transport evidence, Hummingbird pad-to-pad mission,
and synthetic X8-plus-boosters CA-HI case. Three-DOF, pseudo-6-DOF, and
rigid-body 6-DOF results are separate claims: unsupported or non-comparable
tiers remain diagnostic or blocked rather than being promoted by a composite
score.

## Showcase closeout extension

The canonical four-pack status is recorded in
[`verification/alpha2_showcase_catalog.yaml`](../../verification/alpha2_showcase_catalog.yaml)
and regenerated with:

```text
PYTHONPATH=.:src python3 tools/build_cahi_showcase_packet.py --output artifacts/showcases/alpha2
PYTHONPATH=.:src python3 tools/build_alpha2_showcase_catalog.py --output artifacts/showcases/alpha2/final-catalog-v1
```

`A2-SHOWCASE-PACKS-PASS` means that all four requested packs and composites
exist, are reproducible, and state their evidence boundaries. It does not mean
that every pack is family-qualified. The X8 and Hummingbird nominal objective
tables pass; the B747 packet remains a bounded multi-case evidence packet; and
the CA-HI endpoint is deliberately recorded as an independent nominal failure
until its endpoint contract is redesigned and passed.

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

## Honest completion signals

The core release signal is `A2-CLOSEOUT-PASS`, and it is valid only when the
exit gates above and the machine-readable required-item list in
`verification/alpha2_post_release_backlog.yaml` agree. A passing local plant,
bounded trajectory, or weighted score alone is not an Alpha 2 completion
signal. The showcase-extension signal is `A2-SHOWCASE-PACKS-PASS`; it requires
the four-pack catalog and composite, but preserves `qualification_pending` or
`evidence_only` statuses rather than upgrading them to family qualification.

## Closeout evidence

The final primary packet and clean-source replay were generated with
`tools/build_fidelity_ladder_packet.py`, audited with
`tools/audit_fidelity_packet.py`, and compared with
`tools/audit_fidelity_milestones.py`. The comparison passed M0--M6, including
matching four-family objective scores, controller-mission evaluations,
closure/convergence gates, hashes, and snapshot provenance. The generated
packets are release artifacts rather than checked-in source files.
