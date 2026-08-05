# Product 3 coverage audit

**Status:** Living evidence audit; Product 3 is not complete

**Purpose:** Record what the vehicle-and-mission composition product proves
today, what evidence supports that conclusion, and which gaps deliberately
remain source-owned. This audit prevents aggregate catalog counts from being
mistaken for a universal vehicle or qualification claim.

**Companion:** [Product 3 maturity plan](product-three-maturity.md)

## Verified snapshot

The following command was last exercised after the current local direct-wrench
screen and parity registrations:

```bash
taoryx vehicle maturity-report \
  --check-execution-witnesses \
  --execute-batch-witnesses \
  --execute-parity-witnesses
```

It returned `status: pass` with:

| Evidence | Verified count | Meaning |
| --- | ---: | --- |
| Family records | 9 | Public vehicle/family registry entries. |
| Resolved interfaces | 36 | One declared interface contract per registered family/fidelity combination. |
| Runnable endpoints | 30 | Exact batch or episode operations with checked-in composition witnesses. |
| Episode contracts | 11 | Opened episode endpoints preserve their selected interface and accepted-truth boundary. |
| Registered parity witnesses | 11 | Exact declared batch/episode pairs agree on one short committed-boundary trace. |
| Topology findings | 0 | Every currently exposed parameter, channel, observation binding, and objective field has a declared value-space contract. |

These counts are conformance coverage, not model-family qualification counts.

## Requirement audit

| Product requirement | Current evidence | Status | Deliberate boundary / next work |
| --- | --- | --- | --- |
| Explicit value-space topology for public parameters, actions, status, observations, and objectives | `taoryx vehicle topology-report`; versioned parameter, interface-channel, and objective catalogs | Verified for the current public catalog | New fields must be added to the catalogs before registry publication; physical source validity is separate. |
| Unified discovery and evidence-aware metadata | `catalog`, `describe`, `inspect`, `schema`, `endpoints`, `interface`, and authoring-kit commands | Verified for current declared entries | `planned` and `development` records are visible but not runnable. |
| Bounded variant resolution | Immutable variant resolver; runtime-bound A320 operating-mass and Hummingbird grounded-mass witnesses; per-family `variant_worklist` admission report | Framework verified; family breadth partial | Seven of nine families deliberately report `not_declared`: a pinned replay/state history or fixed source plant is not a safe public modifier until a source-owned runtime consumes every coupled derivation. |
| Semantic mission graphs and family capability preflight | Compiled immutable graph, exact family capability adapters, semantic-preflight handler report, and Hummingbird forced-timeout-to-touchdown execution witness | Verified for current registered translators; one non-success branch executed | Most native executors use template-owned linear graphs. Hummingbird alone executes one constrained timeout recovery; generic abort/resource/envelope branches remain future adapter work. |
| Uniform batch/episode execution | Execution-binding catalog, endpoint witnesses, accepted-truth episode contracts | Verified for every advertised runnable endpoint | Source replay and passive paths may legitimately remain batch-only; no unavailable episode is inferred. |
| Batch/episode equivalence | Explicit parity registry and 11 passing deterministic witnesses | Verified only for declared pairs | A parity result is not a mission, robustness, physical-effector, or cross-fidelity result. |
| Normalized evaluation and release artifacts | Typed `evaluation.json`, local-screen records, provenance, resource/action/graph evidence, release-catalog/reproduction checks | Verified structurally for current public batch outputs | Optional robustness, convergence, and cross-fidelity sidecars retain family-owned numerical semantics; their presence is not a release qualification badge. |
| Reusable family authoring | `authoring`, `authoring-template`, existing-family/new-topology intake scaffolds, `vehicle integration readiness/pipeline`, worklist/blocker reports | Structural workflow verified | Intake does not create source mappings, trim, controller, plant, or qualification evidence. The public pipeline exposes recorded source gates and blockers, but a new family still needs those source-owned inputs. |
| Runtime-owned source adapter reuse | `source_table_fixed_wing.py` owns the pinned X8/B747 plants, `source_table_multirotor.py` owns Hummingbird, and `source_f16.py` owns the F-16 first operating-point plant; developer evidence scripts and the Product 3 runtime registry use those same factories and operation probes | Verified for X8, B747, Hummingbird, and F-16 local direct-wrench/surface witnesses | This removes tool/runtime construction drift, but does not create a mission translator, resource model, gain schedule, or envelope qualification. |
| Fidelity and evidence boundaries | Execution modes, interface claim boundaries, direct-wrench screen registration, surface-allocation separation | Verified as a fail-closed catalog rule | Direct-wrench and pseudo-6DOF paths remain bridge evidence unless separately promoted through physical effectors and nonlinear validation. |

## Current source-owned promotion queue

1. HL-20 high-energy glide: the public 3DOF/pseudo semantic release,
   trim-capture, opposing-bank, and handoff plan now has pinned witnesses;
   bind its high-altitude runtime, trim/gravity state, crossrange, terminal
   evaluator, and source-bounded authority. The runnable subsonic local
   direct-wrench screen and booster/release replay are separate evidence only.
2. X-15 high-energy direct-wrench path: bind full phase state and terminal
   objectives to a scheduled controller. Its local recovery screen is not a
   flight mission.
3. A320 and NESC wrench tiers: add source-owned force/moment or gimbal plants,
   meaningful trim, and requested-versus-achieved authority evidence.
4. Reusable mission-branch execution: extend the Hummingbird's verified,
   timeout-to-touchdown committed-state recovery pattern to source-owned abort,
   resource, and envelope transitions only where each family supplies the
   required physical semantics.
5. Broader runtime-bound variants: add coupled source-backed mass, resource,
   propulsion, and geometry derivation only where the selected family runtime
   actually consumes it.

## Interpretation rule

Product 3 is progressing when it makes supported configurations easier to
discover, author, execute, and reproduce while returning a precise blocker for
everything else. It is not progressing if it reuses a nearby model, injects an
undeclared force/moment path, or converts structural metadata into a claim of
physical fidelity or mission qualification.
