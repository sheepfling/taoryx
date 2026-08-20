# Vehicle plug-in isolation

Vehicle packages are meant to make model work vertically testable: changing a
family's plant, controls, composition metadata, or API should require that
family's contract gate, not a re-verification of unrelated models.

## Ownership rule

The core package owns generic schemas, registry mergers, composition/compiler
contracts, and execution dispatch. A vehicle package owns the family-specific
parts that make one advertised endpoint real:

- model source assets and provenance;
- family, fidelity, control, and endpoint catalog fragments;
- native adapters, preflight handlers, execution factories, and controller
  campaigns;
- checked-in composition witnesses and parity evidence; and
- a focused provider when the family needs a dedicated user-facing surface.

The repository-root `verification/` catalogs remain the canonical editing
surface. Package-data extractors materialize non-overlapping family fragments.
At runtime the core merges those fragments by stable identity and rejects
duplicates. A compatibility aggregate may consume installed fragments, but it
must not become a hidden runtime dependency of the family that owns them.

Focused generic composition calls now carry an explicit or active
`PluginCatalog` to the catalog-resource resolver. A package that contributes a
`VehicleCatalogFragment` is then the sole eligible owner of its `data/` tree;
missing selected data is an error rather than permission to fall back to the
repository aggregate or a sibling wheel. F-16 introduced this typed
data-ownership seam, and every existing physical catalog owner now
registers the same metadata. Unscoped compatibility imports retain the
existing aggregate resolver for bootstrap and legacy direct-import paths.

Execution declarations, endpoint witnesses, and parity replay inputs use the
same selected resource scope. A package that declares a registered
batch/episode parity pair must ship its matching
`vehicle_execution_parity_witnesses.yaml` rows; a package with no registered
pair deliberately resolves to an empty witness catalog rather than borrowing
an aggregate trace. A missing replay row for a declared pair remains a focused
gate failure.

An optional workflow that adds an endpoint to an existing vehicle uses a
typed `VehicleCatalogOverlayFragment`, not the base family's
`VehicleCatalogFragment`. The overlay names its required base fragment(s), so
selected-resource lookup fails closed if a host chooses the overlay without
the vehicle. Composition overlays are append-only: they may add uniquely named
initialization, segment, mission, or variant rows, but cannot rewrite a base
vehicle declaration. X-15 staged reachability and HL-20 booster-release/
glide-energy work are such boundaries; their source fixtures remain with the
vehicle family while their optional composition rows, bindings, requests, and
witnesses are reachability-owned. Each family overlay has a distinct
reachability resource root, so an active X-15 overlay never parses HL-20 rows
or conversely.
An overlay whose base is absent remains inactive, so selecting that workbench
with a different vehicle family does not broaden or block the other family's
focused catalog.

## Focused package split

The initial focused package split is complete:

The metadata-only tooling regression keeps the split from drifting backward:
every declared physical family must retain exactly one non-aggregate catalog
owner, focused gate, asset extractor, package README, and wheel proof; moved
family modules must not reappear in either the host or compatibility package.

| Package | Scope | Isolation proof |
| --- | --- | --- |
| `taoryx-a320` | OpenAP point-mass and explicitly surrogate pseudo-6DOF A320 assets, reduced guidance controls, the package-owned native-coordinate LQI definition/advertisement, tuning, witnesses, and `taoryx.a320.mission-composition` | Core plus this plug-in can discover, plan, compile, batch-run, and open the reduced pseudo-6DOF A320 episode without importing `taoryx-reference-models`. Discovery retains the local LQI screen's static identity, named-coordinate bounds, cadence, and claim boundary without importing NumPy, SciPy, the surrogate plant, or numerical LQI runtime; its configuration resolves only when that exact batch-only endpoint is selected. The provider retains its selected catalog through batch/session execution, witness preflight/lowering, and parity replay. Its generated package-data gate rebuilds a temporary exact tree, retaining only the required hash-pinned corpus archive and attribution notice instead of the archive's exploded foreign vehicle artifacts. Its named-coordinate LQI screen is batch-only development evidence, not a physical A320 surface allocator. |
| `taoryx-f16` | F-16 S.119 source assets, 3DOF/pseudo-6DOF reductions, reduced guidance controls, a lazy pseudo-6DOF body-rate interface extension, bounded local physical-controller screens, tuning, witnesses, and `taoryx.f16.mission-composition` | Core plus this plug-in and DAVE-ML can discover, plan, compile, batch-run, and open a pseudo-6DOF F-16 episode without importing `taoryx-reference-models`. Discovery retains the stable family, trim, capability, preflight, execution, controller-screen, interface, and typed catalog-fragment identities without importing its source plant or runtime implementations; selected contracts resolve them lazily. A selected generic compiler resolves only the F-16 package data and fails closed on a missing F-16 asset. The common provider retains its selected catalog through batch/session execution, witness preflight/lowering, and parity replay; the F-16 wheel owns the rate-reference/readback addition while core validates the additive contract. Its generated package-data gate rebuilds a temporary exact tree and rejects stale or extra source/evidence assets. The physical screens remain separate local development evidence rather than route-level qualification. |
| `taoryx-hummingbird` | Hummingbird model/assets, pseudo-6DOF controls and lazy interface extension, pseudo and rotor-screen execution, tuning, witnesses, and `taoryx.hummingbird.mission-composition` | Core plus this plug-in can discover, plan, compile, batch-run, and open the pseudo-6DOF Hummingbird session without importing `taoryx-reference-models`. Discovery retains the static direct-wrench definition and its bounded public metadata without importing the source-hover configuration, numerical rotor tables, or runtime; selected contracts resolve them lazily. The provider retains its selected catalog through batch/session execution, witness preflight/lowering, and parity replay. Its generated package-data gate rebuilds a temporary exact tree and rejects stale or extra source/evidence assets. Its aggregate attitude/thrust, velocity/yaw, and live-waypoint public contract is package-owned; physical rotor/direct-wrench screens remain separate local evidence. |
| `taoryx-daveml` | shared DAVE-ML parser, semantic IR, evaluator, compatibility, collection, and replay modules, with the `daveml` model-format contribution | Core plus this plug-in can discover the selected model-format handler and load its implementation lazily without constructing a vehicle, parsing a source document, or importing a consumer family. Its focused source and wheel gates prove that `daveml` retains the package version/revision and that the handler resolves `taoryx.trajectory.daveml_import` from the installed wheel. A clean build staging hook prevents removed shared-namespace modules from surviving a reused local `build/` tree. |
| `taoryx-debug-models` | development-only analytical ballistic/waypoint and contract-probe providers, selectable low-fidelity controls, and three package-owned endpoint witnesses | Core plus this plug-in can discover, plan, batch-run, and open only the nonphysical fixtures without importing the compatibility aggregate. Its generated package-data gate rebuilds a temporary exact tree and rejects stale or extra endpoint fixtures. The ballistic coast is an explicit zero-action mode; waypoint and probe sessions expose typed selectable controls, runtime availability, and readback. Its wheel executes the three package-owned batch witnesses without promoting any model into a physical-vehicle, controller, or qualification claim. |
| `taoryx-simple-aero` | analytical fixed-L/D builder, `reference.point_mass`, package-owned `taoryx.simple-aero.mission-composition` provider, generated/direct-throttle session, and workflow-endpoint catalog/witness | Core plus this plug-in can discover, author, batch-run, and open the nonphysical point-mass workflow without the compatibility aggregate. Its generated package-data gate rebuilds a temporary exact tree, retaining its required analytical fixture/segment data while filtering the shared Alpha 2 catalog to `simple_aero` only. The default generated schedule has no caller actions; the selectable direct-throttle profile exposes a single normalized throttle command with requested/achieved feedback. Its endpoint catalog is filtered to the selected plug-in in a source checkout and loaded from package data in a wheel. |
| `taoryx-dual-launch` | synthetic dual-launch glider family metadata, source-problem lowering, normalized batch projection, `taoryx.dual-launch.mission-composition`, and its endpoint witness | Core plus this plug-in can discover, author, batch-run, and open the standard read-only replay session for both air-release and attached-booster point-mass forms without importing `taoryx-reference-models` or Simple Aero. Its generated package-data gate rebuilds a temporary exact tree containing only its family, maturity, endpoint, and witness fragments. It advertises source-generated bank/throttle as provider-managed telemetry, reports separation only as an event on its primary trajectory, and keeps live caller control, higher-fidelity, and independent-child routes explicitly blocked. |
| `taoryx-nesc` | NESC source-replay data, point/pseudo adapters, batch runtime, stage-separation parent contract, and `taoryx.nesc.mission-composition` | Core plus NESC and its DAVE-ML dependency can discover, plan, compile, batch-run, and open the standard read-only replay session for the two-stage source replay without importing `taoryx-reference-models`. Its generated package-data gate rebuilds a temporary exact tree, retaining the named passive-child composition as metadata without making the passive runtime an NESC dependency. Its deferred provider retains the selected catalog through the common batch/session API; the optional passive child remains an explicit two-package deployment proof rather than an aggregate dependency. |
| `taoryx-passive-bodies` | tumbling/released-body source fragments, package-owned direct-release truth interface, adapters/witnesses, and selected child propagation | Core plus this plug-in can discover, plan, compile, batch-run, and open the standard read-only replay session for `tumbling_body` without importing a parent vehicle or reachability plug-in. Discovery publishes the family-adapter, capability, semantic-preflight, interface-factory, and child-runtime identities without constructing their numerical stack, planner, or propagator; each implementation resolves only when its contract is used. Its generated package-data gate rebuilds a temporary exact tree and rejects stale direct-release assets. The provider retains its selected catalog through batch/session execution and preflight/lowering; its `no_external_action` contract means the replay has no controller or native live-plant claim. |
| `taoryx-x15` | X-15 source tables, source-local direct-wrench and source-surface screens, tuning, witnesses, and `taoryx.x15.mission-composition` | Core plus this plug-in can discover, plan, compile, batch-run, and open the local direct-wrench episode without importing the compatibility aggregate or reachability. Discovery retains the direct-wrench screens' identities, six-axis bounds, cadence, and claim boundaries without importing NumPy, SciPy, source tables, or numerical runtime; configuration factories resolve that implementation only after matching-screen selection. Its generated package-data gate rebuilds a temporary exact tree and rejects stale or unrelated source/evidence assets while retaining only source assets intentionally consumed by the optional reachability overlay. The focused provider retains the selected catalog and package version through that batch/session path, exposes only its four local endpoints; staged reachability rows, requests, bindings, and witnesses are absent until the reachability-owned typed overlay is selected with X-15. |
| `taoryx-hl20` | HL-20 Mod K source fixture, local direct-wrench and seven-surface controller screens, source trim evidence, tuning, witnesses, and `taoryx.hl20.mission-composition` | Core plus HL-20 and DAVE-ML can discover, plan, compile, batch-run, and open the local direct-wrench episode without importing the compatibility aggregate or reachability. Its generated package-data gate rebuilds a temporary exact tree and rejects stale or unrelated source/evidence assets while retaining the HL-20 fixture intentionally consumed by optional reachability overlays. The focused provider retains its selected catalog and package version through that batch/session path, exposes only its four local endpoints; booster-release and glide-energy rows, requests, bindings, and witnesses remain absent until the reachability-owned HL-20 overlay is selected. |
| `taoryx-source-table-fixed-wing` | Skywalker X8 and B747 source-table plants, route catalogs, focused composition providers, package-owned interactive route binding, local controller screens, campaigns, and witnesses | Core plus this plug-in can discover both focused providers, plan, compile, batch-run, and open the package-owned X8/B747 route episode without importing `taoryx-reference-models` or reading repository-root route data. Its generated package-data gate rebuilds an exact shared X8/B747 tree and rejects stale or third-family assets. Both providers retain their selected catalog through their lower-tier batch/session paths; their normal gates cover only point-mass and pseudo-6DOF guidance, while local physical screens remain separate batch-only development evidence. |
| `taoryx-reachability` | optional X-15/HL-20 reachability envelope, terminal, plotting, capability, preflight, and execution overlay | Core plus DAVE-ML, X-15, HL-20, and this plug-in can discover the workbench and stable overlay identities without importing source planners, preflights, or executors, and without installing `taoryx-reference-models`. Its X-15 staged and HL-20 booster-release/glide-energy routes each contribute an append-only typed catalog overlay that requires only its named vehicle base fragment and has its own package resource root; neither can be discovered as a stand-alone vehicle or parsed by the other family scope. Its exact package-data gate retains only the reachability profile, showcase, and overlay-owned composition/binding/witness assets; its clean staging hook removes legacy passive-body modules. Its selected installed-wheel proof builds the direct dependency wheels but exercises only this overlay's boundary, so an overlay change cannot rerun HL-20 trim or X-15 controller work. |
| `taoryx-cadac` | source-bound CADAC actor catalog, source-compatibility runtimes, native sensor projections, controller-analysis metadata, and canonical table conversion | Discovery contributes only the `cadac` provider identity; its Pydantic-heavy actor catalog materializes when a host explicitly selects the CADAC Composition API. Core plus this plug-in can then discover all CADAC actors without locating upstream input decks. A source-bound host can select one exact actor or package model so it materializes and registers no unrelated model schemas or fallback runtime. AIM5, ADS6 SRBM, standalone ADS6 SAM and AIRCRAFT3, AGM6, FALCON6, ADS6 engagement, and SRAAM6 cases bind only their exact model IDs, run through the catalog's batch API and standard core replay session where registered, retain their declared control and sensor boundaries, and cannot fall back to another actor. CRUISE5 is a selected pseudo-6DOF waypoint/line batch boundary: it publishes a provider-managed source-program control scheme and labeled command/response/guidance outputs while keeping its translation-only tier, native persistent controller session, and formal stability claim blocked. MAGSIX is a selected point-mass trajectory/spin batch boundary with a fixed source program; its pseudo-6DOF attitude phase validates only and cannot be relabeled as a runnable model. GHAME3 is a selected round-Earth point-mass batch boundary: its prescribed alpha/bank and hypersonic propulsion are telemetry from the source event schedule, not caller actions or 6-DoF rotational state. GHAME6 is a selected phase-aware multi-actor package: HYPER6, SAT3, and RADAR0 remain independent roots, caller-owned physical-surface/RCS commands retain requested/achieved evidence, and each RADAR0 source track has a separate standard raw `relative-state-track` event from committed geometry. Its batch-native contract preserves the T4-to-T3 phase boundary and blocks a fabricated `SensorBus`, live control session, and formal stability claim. ROCKET6G is a selected one-vehicle phase-aware launch boundary: caller-owned TVC/RCS controls publish requested/achieved output evidence, including the source RCS force-mode that explains when a thrust-vector input can produce force, and per-sample T3/T4 phase fidelity without fabricating a live control session or closed-loop stability claim. |
| `taoryx-reference-models` | legacy aggregate only | It owns no vehicle plant, focused workflow provider, source lowering, workflow endpoint witness, or generic host validation module. Its aggregate keeps legacy registry compatibility while Simple Aero and Dual Launch own their focused providers, endpoint specs, witnesses, and execution; core owns generic completion, maturity, parity, and integration reporting. The NESC parent may still request a passive child through the typed core binding. |

The global `taoryx.registry.mission-composition` provider remains for existing
catalogue consumers. New A320-, F-16-, HL-20-, Hummingbird-, X8-, B747-, X-15-,
or Dual-Launch-focused UI, agent, or composition work should use the corresponding focused provider so
its controls and capabilities are presented from the owning package.

The compatibility package also clears its own wheel staging tree before
assembly. That prevents a local rebuild from retaining a removed `taoryx.*`
module after an ownership extraction; the normal focused-wheel verifier builds
from a fresh source distribution as an independent backstop.

## Focused verification

For A320, F-16, HL-20, Hummingbird, X8, B747, X-15, NESC, or passive-body changes, use the applicable vehicle
gate as the normal inner loop:

```bash
python tools/dev.py check-vehicle a320_openap_3dof
python tools/dev.py check-vehicle f16_s119
python tools/dev.py check-vehicle hl20_mod_k
python tools/dev.py check-vehicle hummingbird
python tools/dev.py check-vehicle skywalker_x8
python tools/dev.py check-vehicle b747
python tools/dev.py check-vehicle x15
python tools/dev.py check-vehicle reference_nesc_two_stage_rocket
python tools/dev.py check-vehicle tumbling_body
```

The X-15 gate is intentionally a local direct-wrench bridge proof: one
batch/episode witness pair and their exact parity trace. Its reachability
overlay and source-surface screens are independent evidence lanes, so an API
or local-wrench change does not trigger their broader physical checks.

The HL-20 gate follows that same narrow local boundary: one direct-wrench
batch/episode witness pair, its exact parity trace, and a selected-catalog
provider/compiler/batch/session proof. Its source-release replay belongs to
the reachability owner, while seven-surface authority and LQI screens remain
separate physical-controller evidence.

CADAC actor work uses one exact source-bound vertical slice rather than an
actor-catalogue sweep:

```bash
python tools/dev.py test-vehicle cadac_aim5
```

That AIM5 route binds and materializes only `cadac.aim5.missile`; its embedded
target can appear in the selected model's result without becoming a separately
registered fallback model. ADS6 SRBM, ADS6 SAM, and AIRCRAFT3 follow the same
one-actor pattern under their corresponding `test-vehicle` gates. ADS6
engagement instead selects one exact source-owned package model: its SAM,
target, and RADAR0 roots remain result objects, not registered fallback models.
The selector recognizes that package identity even though it does not map to a
single actor manifest descriptor. AGM6 and SRAAM6 use the same exact-model
boundary while retaining their target/aircraft result roots; standalone FALCON6
uses its direct-surface aircraft model alone. `check-cadac` remains the
deliberate multi-actor integration and wheel checkpoint, not any actor's normal
inner loop.

`CadacSourceCaseBindings` validates that scope before constructing any bound
runtime. An omitted eager source deck therefore fails as a scope error rather
than parsing or validating unrelated source/Pydantic models during a focused
operation.

DAVE-ML is a shared model-format plug-in rather than a vehicle family. Its
narrow gate proves selected discovery and the lazy handler import without
retesting a consuming family:

```bash
python tools/dev.py check-daveml
```

Reachability is an optional overlay, not a vehicle-family regression lane. Its
gate checks its exact packaged assets, direct X-15/HL-20 dependency boundary,
and deferred registration without executing an X-15 or HL-20 study:

```bash
python tools/dev.py check-reachability
```

Simple Aero is a nonphysical workflow gate rather than a physical-family
check. Its normal inner loop and its independent wheel boundary are:

```bash
python tools/dev.py test-vehicle simple_aero
python tools/verify_plugin_wheels.py --plugin simple-aero --python .venv/bin/python
```

Dual Launch is likewise a nonphysical workflow gate. It proves only the two
source-generated point-mass launch forms, their explicit event-only separation
boundary, package-owned controls/readback, and packaged endpoint witness:

```bash
python tools/dev.py test-vehicle dual_launch_glider
python tools/verify_plugin_wheels.py --plugin dual-launch --python .venv/bin/python
```

Debug models have an equally narrow nonphysical workflow gate. It covers only
the analytical ballistic and waypoint fixtures plus the synthetic contract
probe—not the vehicle catalogue:

```bash
python tools/dev.py test-debug-models
python tools/verify_plugin_wheels.py --plugin debug-models --python .venv/bin/python
```

It uses synthetic test-only AIM5, CRUISE5, MAGSIX, GHAME3, GHAME6, ROCKET6G, ADS6 SRBM, standalone ADS6 SAM and AIRCRAFT3,
AGM6, FALCON6, ADS6 engagement, and SRAAM6 cases, never redistributed upstream decks.
The proof covers metadata-only discovery, explicit one-case bindings, normal
batch and persistent-session APIs where a source-owned session exists,
control-analysis readiness, and the native `relative-state-track` SensorBus
projection. The SAM slice instead proves its three caller-owned fixed-batch
control boundaries and requested/achieved evidence, then asserts that a
persistent session and `SensorBus` are blocked until the source-scheduled
package owns their hidden state. The AIRCRAFT3 slice proves the separate
source-program g-turn response, retains point-mass-only truth, and makes its
batch-only/no-sensor boundary explicit. The SRBM slice checks that its source
command and realized-acceleration outputs remain part of the normal batch and
session readback surfaces; AGM6 and SRAAM6 check source command to
physical-fin readback; FALCON6 checks caller-owned physical surfaces through
requested/achieved and position/rate-limit output evidence; the ADS6 package
slice checks the same boundary for a source-scheduled SAM/target/RADAR0
composition. The CRUISE5 slice proves selected-model construction, the source-owned
waypoint/response-law control scheme, its batch-only scope, and complete labeled
geodetic/guidance/control output surface without relabeling its blocked point-mass
projection as executable. The MAGSIX slice proves the selected point-mass source
trajectory/spin output boundary and rejects its validation-only pseudo-6DOF attitude
phase as a batch route. The GHAME3 slice proves selected source-bound Round3 trajectory,
event, force, aerodynamic, and propulsion readback while preserving the fixed source-program
and point-mass-only boundary. The GHAME6 slice proves selected package-level construction,
independent HYPER6/SAT3/RADAR0 roots, full fixed-batch control/readback, source-track versus
native relative-state event separation, and the blocked persistent-session/stability boundary.
The ROCKET6G slice proves selected phase-aware launch-vehicle
batch control, requested/achieved TVC/RCS evidence, stage/event readback, and the blocked
persistent-session/stability boundary. The catalog provider rewrites an inner actor result to the exact
catalog revision selected by the caller; an actor's implementation revision
remains visible in its model metadata rather than leaking into the provider
identity.

When package ownership, entry points, or package data change, prove the real
distribution boundary separately:

```bash
python tools/verify_plugin_wheels.py --plugin source-table-fixed-wing --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin hl20 --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin daveml --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin cadac --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin debug-models --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin simple-aero --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin dual-launch --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin reference-models --python .venv/bin/python
python tools/verify_plugin_wheels.py --plugin reachability --python .venv/bin/python
```

The selected wheel check builds core plus the named package and its direct
dependency wheels from fresh source distributions, installs only those wheels
in a temporary target, removes editable source roots, and exercises discovery,
providers, packaged data, and concrete adapters for the named boundary. A
dependency wheel is installed and discovered but does not run its own vertical
checks. Use `python tools/dev.py plugin-wheel-smoke` as the broader
package-boundary integration gate. Neither command is a reason to rerun
unrelated vehicle suites. A wheel boundary may compile a packaged composition
without executing it when the family’s focused gate already owns the numerical
witness; HL-20 uses that split so an installation check cannot silently repeat
its local direct-wrench or physical-controller evidence.

The NESC-to-passive integration has an intentionally separate source-free
proof because it needs two plug-ins and an explicit composition binding:

```bash
python tools/verify_plugin_wheels.py --plugin cross-plugin-deployment
```

It installs only the NESC, DAVE-ML, and passive-body wheels chosen by that
spec, then executes the NESC source-replay parent with its explicitly named passive child
both directly and through the common Mission Composition API using the NESC
provider selected from that same catalog. The parent can
still run without a child binding; a binding whose selected passive runtime is
unavailable fails closed rather than rediscovering the catalogue.

The package checks cover the selected package's resolved interface, endpoint
witnesses, registered batch/episode parity where applicable, and vertical
tests. They intentionally do not execute unrelated vehicle suites. CADAC's
wheel proof is metadata-only because source cases remain caller-owned; its
separate AIM5 gate supplies the explicit synthetic source binding. The
plug-in-boundary tests additionally prove that the A320, F-16, HL-20,
Hummingbird, NESC, passive-body, source-table fixed-wing, and Dual Launch source trees have
no reference-model import path.
Broad `check` and full pytest remain integration/release gates, appropriate
when shared core contracts or package
aggregation change.

The F-16 gate is intentionally narrower still: it validates only the
point-mass 3DOF and pseudo-6DOF interfaces, four reduced endpoint witnesses,
and two reduced parity traces under `taoryx.f16`. Its source-local
direct-wrench, allocated-surface LQR/LQI, and scheduled-controller evidence
remains a separate physical-controller suite, invoked only when that evidence
changes.

The A320 gate validates only the point-mass and pseudo-6DOF reduced interfaces,
their four normal batch/episode witnesses, and two parity traces under
`taoryx.a320`. Its native-coordinate LQI screen remains a separate batch-only
development-evidence suite, invoked only when that controller or screen
changes.

The Hummingbird gate follows the same boundary: it validates only the
pseudo-6DOF interface, its batch and episode witnesses, and its one parity
trace under `taoryx.hummingbird`. The source-local individual-rotor LQI,
translation, and direct-wrench screens stay in their physical evidence suite;
an interface or composition API change does not re-execute them.

The passive-body gate validates only its point-mass and pseudo-6DOF batch
interfaces and two direct-release witnesses under `taoryx.passive-bodies`.
It deliberately skips session and parity replay because neither endpoint is
advertised; absence is evidence, not a prompt to borrow another family route.
The NESC-to-passive release flow remains the separate explicit two-package
cross-plugin deployment proof.

Focused hosts may also request an explicit discovery scope. For example, the
Hummingbird vertical suite invokes
`discover_plugins(include_external=False, selected=("taoryx.hummingbird",))`.
Unselected plug-ins are not imported, registered, or diagnosed; this is a
performance boundary as well as a test-scope boundary. An aggregate host
omits `selected` and continues to discover every installed plug-in.

Discovery also registers callable endpoint seams rather than eagerly loading a
family's physical runtime. For example, Hummingbird discovery retains stable
family, provider, preflight, and endpoint identities but does not import its
numerical adapter, source rotor plant, physical screen, pseudo-6DOF episode,
or batch executor. Those components load only when a host selects the matching
adapter, provider API, preflight, episode, or batch endpoint. This keeps
Pydantic schema construction and numerical dependencies off the listing and
focused-discovery path without weakening the interface contract at use time.
Even constructing the focused Mission Composition provider registry only
indexes Hummingbird's advertised provider ID; it does not construct the
family's catalog or configuration schemas.

Source trim evidence follows the same ownership rule. A source-backed family
registers a lazy `trim_evidence_binding` under its source family ID; the core
aggregates the report but neither imports nor names the family's evaluator.
The caller should pass its selected plug-in catalog to
`solve_vehicle_trim_evidence`, which resolves only that contribution. This
keeps a trim request for one family from becoming a fallback import of another
family or the compatibility aggregate.

The developer tool exposes the same scope when a direct evidence run is
needed:

```bash
python tools/solve_vehicle_trim_evidence.py \
  --family reference_hl20_mod_k \
  --plugin taoryx.hl20
```

When a focused host executes a composition, it should pass the same catalog to
each public execution surface. The core carries that scope through nested
capability, preflight, lowering, factory, witness, and parity calls, so an
isolated host never falls back to a catalogue-wide rediscovery while opening
an episode, running a batch, or replaying a trace:

```python
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.plugins import discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

catalog = discover_plugins(
    include_external=False,
    selected=("taoryx.hummingbird",),
)
composition = compile_vehicle_composition(source_request, plugins=catalog)
preflight = preflight_vehicle_composition(composition, plugins=catalog)
batch = execute_vehicle_composition_batch(composition, output_dir, plugins=catalog)
episode = open_vehicle_composition_episode(composition, plugins=catalog)
```

A deferred focused Mission Composition provider binds the catalog that created
it to its concrete provider. The common runner and session manager then reuse
that catalog automatically for canonical vehicle batch and episode routes.
This makes the A320, F-16, Hummingbird, NESC, passive-body, X-15, HL-20, and
source-table fixed-wing providers' selected catalogs execution capabilities
rather than just discovery optimizations.

Family-specific public controls can use the typed
`vehicle_interface_extension` contribution. It is additive: a plug-in may
publish controls, authority profiles, and committed status/resource/diagnostic
readbacks for its own family, while the core validates duplicate IDs,
authority membership, value spaces, and execution availability. An extension
does not give a plug-in permission to mutate another family's contract or add
unbacked plant behavior.

Omitting `plugins` intentionally retains the legacy aggregate-discovery
behavior for broad catalogue hosts.

## Next split decisions

Do not split a vehicle merely to reduce a count. Promote a family into its own
package when it has a coherent source/evidence bundle, a distinct executable
dependency boundary, and an owner who can maintain the focused gate. Each
candidate must first satisfy the same acceptance criteria as A320, F-16, HL-20,
and Hummingbird:
family-owned package data, no aggregate runtime import, an explicit provider
or documented aggregate-only rationale, one vertical check command, and an
installed-wheel smoke path. No residual vehicle plant remains in the
compatibility aggregate; keep any future candidate in its current owning
package until it satisfies those criteria rather than creating a broad,
partially isolated plug-in tree.

### Current base-family coverage

The standard composition registry currently has one non-aggregate owner for
every declared family: A320, F-16, Hummingbird, NESC, passive tumbling body,
X8, B747, X-15, and HL-20. A metadata-only regression checks that each owner
also has an exact package-data extractor, a focused vertical test, a scoped
`check-vehicle` configuration, package documentation, and an installed-wheel
specification. It does not execute any vehicle trajectory.

A new standard vehicle family must enter that invariant in the same change:
register one `VehicleCatalogFragment`, add its focused gate and package-data
check, document the package boundary, and add an installed-wheel proof. An
optional cross-family capability must instead use an additive overlay with its
own resource root and named base dependency, as reachability does for X-15 and
HL-20.

### Completed shared extraction: source-table fixed wing

`taoryx-source-table-fixed-wing` is one shared distribution for both Skywalker X8 and B747,
not two vehicle-thin wheels. The pair shares the source-table plant machinery,
language-backed fixed-wing route runtime, common `powered_fixed_wing_racetrack_v1`
contract, table fixture, and associated catalog/evidence extraction. Separating
only one would leave the other coupled to the same implementation and would
not improve the vertical test boundary.

| Boundary | Implemented shape | Focused proof | Not part of the normal extraction loop |
| --- | --- | --- | --- |
| Package ownership | Shared source-table plant/route machinery plus both families' adapters, control metadata, source tables, witnesses, local screens, and route catalogs | The package has no runtime import of `taoryx-reference-models`; batch and episode routes use its packaged assets | A physics rewrite or a new controller design |
| UI and composition API | Two focused providers—one for X8 and one for B747—while the legacy aggregate consumes both fragments | Each provider advertises only its own model, tiers, controls, feedback, and endpoint metadata | A catalogue-wide provider regression |
| X8 vertical gate | `python tools/dev.py check-vehicle skywalker_x8` using one ordinary source-table racetrack realization | Catalog/preflight/selected execution and the X8 witness | Re-running every local physical LQR/LQI screen after an unrelated package move |
| B747 vertical gate | `python tools/dev.py check-vehicle b747` using only the point-mass and pseudo-6DOF racetrack lanes | B747 catalog/preflight, its four lower-tier batch/episode witnesses, parity traces, and a bounded selected-catalog batch/session proof | Re-running every B747 direct-wrench or local-controller screen after an unrelated package move |
| Distribution boundary | Selected `source-table-fixed-wing` wheel smoke | Core plus the wheel can run both focused providers, batch route, and package-owned episode without the compatibility aggregate | Full catalogue or reachability verification |

The local physical-controller screens still move with their owning family and
retain their own batch-only evidence. They are not silently dropped, but they
are not the default extraction gate: execute them when their controller,
source-table derivatives, or screen contract changes. `taoryx-reference-models`
is now a compatibility aggregate with no vehicle-plant implementation of its
own.
