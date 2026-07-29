# A320 and NESC integration closeout

Status: Alpha 3 integration workstream

## Current position

The automatic pilot now discovers both collection manifests and the
source-grounded NESC family manifest using the same nine-stage vocabulary:

```text
intake → conventions → plant → effectivity → trim → operating_points
       → fidelity → controller → mission
```

It is diagnostic evidence, not a qualification badge. The current report is:

| Family | Current result | First remaining closeout gate |
| --- | --- | --- |
| NESC Scenario 17 | Source plant, conventions, trajectory checkpoint, staging/reduction/replay lineage, and mission graph pass. Trim/effectivity/controller are correctly not applicable. | Decide whether the deferred pseudo-6DOF tier remains unadvertised or receives an authoritative attitude/controls comparison. |
| A320 OpenAP 3DOF | Collection, conventions, plant, trim evidence, operating point, derived-exact fidelity, shared racetrack preflight, and all four independent truth gates pass. | Add broader operating-point witnesses; do not promote attitude/effectors from this lane. |
| A320 OpenAP + JSBSim pseudo-6DOF | Plant, trim, typed surrogate effector contract, shared racetrack preflight, and all four truth gates pass as development evidence. | Add independent nonlinear response/actuator evidence; retain surrogate and policy-overlay boundaries. |

## Closeout sequence

### 1. Stabilize the generic integration contract

- Keep `family.yaml` and `collection-manifest.json` as separate input forms,
  but normalize both into one immutable integration snapshot.
- Move applicability (`required`, `not_applicable`, `planned`) into the
  normalized snapshot rather than recovering it from prose in layer records.
- Normalize evidence references so a human description such as
  `272/272 oracle comparisons` cannot be mistaken for a file path.
- Add source-payload availability and external-hash status to every plant
  report.
- Add a packet writer for collection pilots, not only DAVE-ML reference
  families.
- Make generated reports record the hashes of the manifest, integration
  record, operational contract, operating-point entry, and runtime evidence.

Implemented in the first closeout slice:

- `IntegrationSnapshot` normalizes collection JSON and reference `family.yaml`
  inputs while retaining their manifest kind and family-specific applicability.
- `EvidenceReference` distinguishes local files, external package members, and
  inline status/prose; local declared hashes are checked against observed bytes.
- `write_vehicle_integration_packet` copies local family inputs and retains
  external dependency paths and hashes without pretending those packages are
  present.
- The pilot CLI can regenerate reports and packets with one command:
  `python tools/validate_vehicle_integration_pilots.py --family all
  --allow-blocked --json verification/vehicle_integration_pilots.json
  --packet-dir verification/integration-packets`.

### 2. Finish the A320 OpenAP 3DOF lane

- Add a reusable performance mission binding based on the common powered
  fixed-wing racetrack/airborne-arrival template.
- Declare the A320-specific speed, turn-radius, climb/descent, mass, and
  terminal-gate values in the binding; generate the initial time estimate from
  those values.
- Add an A320 trim recipe or explicitly bind the existing OpenAP oracle result
  as a derived-exact operating-point adapter. Do not create a guessed
  rigid-body trim file.
- Connect the independent truth evaluator to altitude, speed, route, energy,
  fuel, and terminal objectives.
- Generate a 3DOF showcase packet with the exact nonclaim that it has no
  attitude, moment, surface, or actuator proof.
- Add a negative mission test showing that a missing or impossible terminal
  gate produces a structured blocker rather than a passing scalar score.

Current implementation adds `qualification/racetrack-binding.yaml` for the
exact OpenAP lane and uses a declared 230.1542 m/s Mach-0.78 timing anchor.
`taoryx.trajectory.a320_racetrack.A320RacetrackRunner` now executes the shared
route at the declared reduced fidelity, and
`tools/validate_a320_racetrack.py` writes telemetry, truth-gate evidence, and
boards for both A320 lanes. The pilot consumes the resulting evidence rather
than treating a binding or controller transition as a mission pass.

Exit: the A320 3DOF lane is promotion-ready for its declared performance
mission, while remaining explicitly non-qualified for rigid-body dynamics.

### 3. Finish the A320 pseudo-6DOF lane

- Freeze the authority map: OpenAP remains authoritative for translational
  performance and propulsion; JSBSim remains a rotational surrogate; disabled
  contributions stay disabled.
- Convert the canonical control-mapping export into a typed effector contract:
  surfaces, engine response, signs, neutral, limits, rate, lag, and health.
- Expose requested versus achieved force/moment, actuator commands versus
  actual states, saturation, and allocation residual in runtime telemetry.
- Add a named pseudo-6DOF response/controller profile and a bounded trim
  recipe at the documented cruise point.
- Run isolated pitch, roll, yaw, speed, and coupled response probes. These
  prove the declared response law only; they do not prove Airbus moments.
- Reuse the A320 performance mission binding, but allow pseudo-6DOF-specific
  terminal attitude/rate requirements only where represented.
- Produce a surrogate-composite showcase and matched 3DOF comparison report.

Implemented for the current slice: `qualification/effector-contract.yaml`
declares throttle, aileron, elevator, and rudder bounds/rates plus the
semantic flight-path/bank commands. The contract states that coefficient
authority is JSBSim-derived but actuator realization is Taoryx policy; the
pseudo racetrack packet labels its path as `surrogate_policy_overlay`, not as
a physical allocator. The pseudo mission passes the same four truth gates as
the 3DOF lane, but the pilot remains development-gated because its
effectivity, controller, and fidelity are explicitly surrogate overlays.

Exit: pseudo-6DOF is development-qualified as a Taoryx surrogate at its
declared operating point and envelope, with no manufacturer or source-exact
6DOF claim.

### 4. Finish the NESC source/open-loop lane

- Preserve Scenario 17 as an open-loop, variable-mass source replay; do not
  invent equilibrium trim or actuator controls.
- Turn the existing staging lineage and checkpoint evidence into a reusable
  mission binding with explicit ignition, cutoff, staging, coast, and terminal
  events.
- Add resource/event continuity checks for propellant, dry-mass transition,
  stage identity, and terminal state.
- Generate a source-replay evidence packet and a 3DOF checkpoint comparison
  packet from the existing qualified reduction evidence.
- Decide the pseudo-6DOF disposition explicitly: keep it `deferred` and omit
  it from advertised supported tiers, or supply source/independent attitude
  evidence before promotion.
- Keep the synthetic passive deployment child in a separate lineage packet;
  it must never upgrade the NESC parent claim.

The pilot now includes the verified staging, reduction, replay, objectives,
and synthetic-deployment evidence in the NESC packet. It reports the five
source staging events, the bounded-reduction pass, and the explicit
`parent_qualification_unchanged_by_child` lineage flag.

Exit: NESC is complete as a source-grounded open-loop staged-vehicle
integration, with reductions and deployment children carrying independent
qualification labels.

## Pain-point ledger

| ID | Observed friction | Why it matters | Corrective action |
| --- | --- | --- | --- |
| P-01 | Two manifest shapes: typed `family.yaml` versus collection JSON. | Generic code either rejects A320 or assumes reference-only fields. | Normalize both into an immutable integration snapshot. |
| P-02 | Applicability is split between operational contracts and layer prose. | The tool cannot reliably know whether trim/controller/effectivity is required. | Make applicability a typed per-stage field in the normalized snapshot. |
| P-03 | Evidence values mix paths, prose, and package-member names. | Missing evidence can look present, or an external artifact can be treated as local. | Use typed `evidence_ref` records with `kind`, `path`, `sha256`, and availability. |
| P-04 | Collection runtime artifacts are often external to the repository. | A valid external package is incorrectly reported as missing, or a missing package is silently accepted. | Separate `declared`, `locally_present`, `hash_verified`, and `runtime_exercised`. |
| P-05 | A320 3DOF has verified oracle evidence but no reusable mission binding. | Plant evidence cannot flow into a showcase or route preflight. | Add a family binding without changing the OpenAP source claim. |
| P-06 | A320 pseudo-6DOF has a control overlay but no common effector contract. | The controller path cannot be audited for actual versus requested authority. | Generate a typed surrogate effector/actuator profile from the control mapping. |
| P-07 | NESC has no controls or trim by design, but generic stages naturally expect them. | Open-loop rockets appear incomplete instead of correctly not applicable. | Make not-applicable stage semantics first-class and test them. |
| P-08 | Fidelity status is family-level while profile evidence is tier-specific. | One deferred pseudo-6DOF profile can be hidden behind a passing parent. | Compute lowering eligibility per profile and report deferred tiers explicitly. |
| P-09 | Several registries repeat family IDs and paths. | New families require synchronized manual edits and stale hashes are common. | Generate secondary registries from one source integration index. |
| P-10 | Generated reports become stale after manifest merges. | CI failures appear far from the source change and tempt hand editing. | Add dependency hashes and a single regeneration command to the pilot. |
| P-11 | Mission integration is still hard-coded around aircraft racetracks. | NESC staging and A320 performance missions need different event/terminal contracts. | Generalize mission preflight around typed segments and event graphs. |
| P-12 | The current pilot stops at diagnosis and does not emit a collection packet. | Results cannot yet be handed off as self-contained evidence. | Reuse the packet manifest contract for collections and external references. |
| P-13 | A320 had strong static/trim evidence but no executable shared mission seam. | A mission binding could be mistaken for a flown mission. | Add a model-specific reduced runner, truth-gate packet, and pilot promotion hook. |
| P-14 | The pseudo-6DOF control mapping was an identity DAVE-ML export plus an authority map. | Bounds, rates, and policy/source boundaries were not inspectable as one contract. | Add a typed effector contract and validate it fail-closed. |
| P-15 | A kinematic terminal reference continued translating after its finish crossing. | The final plot could look like terminal drift even after the gate passed. | Detect the declared crossing and emit a bounded terminal hold interval. |
| P-16 | A320 performance timing and source envelope were not connected to a mission step. | Speed/altitude gate failures were discovered only after ad hoc runs. | Use the OpenAP trim/performance channels inside the shared racetrack runner. |

## Definition of done

This workstream is complete when:

1. One command discovers and normalizes A320 collection and NESC reference
   inputs.
2. The same stage schema reports passed, development, not-applicable, and
   blocked outcomes without family-specific branching in the renderer.
3. A320 3DOF has a reusable performance mission and independent terminal
   evaluation.
4. A320 pseudo-6DOF has a declared surrogate effector/controller path and
   requested-versus-achieved response evidence.
5. NESC has a source-replay staging mission packet and explicit reduction/
   deployment lineage.
6. Every result has current dependency hashes, exact claims, nonclaims, and a
   reproducible command.
7. Negative controls prove that missing frames, hashes, applicability, mission
   bindings, and fidelity evidence fail closed.

The final product is not “all three models pass every tier.” The honest result
is three different maturity outcomes produced by one automatic process.
