# Running the test suite

The repository uses explicit pytest markers for cost categories and overlapping
test views. Views are intentionally not mutually exclusive: one test may
belong to both the equations and algorithms views.

To print the taxonomy from the portable task runner:

```bash
python -m tools.dev test-views
```

Use `pytest --markers` to inspect the registered marker descriptions directly.

## Static quality and CI

The `Core Quality` workflow runs on every push and pull request. It uses the
same portable task as local development, then runs the routing/quality contract
tests with an isolated deterministic pytest temporary root.

```bash
python -m tools.dev quality    # non-mutating Ruff fix preview, then Pyright
python -m tools.dev ruff-fix   # apply Ruff's safe fixes to that same surface
```

`quality` follows each declared `taoryx.plugins` entry point for Ruff, checks
the shared discovery/developer-route API with Pyright, and validates every
plug-in's structural developer route. A new plug-in registration is therefore
covered automatically without forcing its complete model implementation through
a catalogue-wide type scan. CI invokes Ruff with `--fix --diff`: it reports the
exact safe patch needed but never rewrites a contributor's branch. Run
`ruff-fix` locally to apply that patch. The broader legacy `python -m tools.dev
lint` and Mypy gates remain available for release work; the inherited parser
and model corpus has tracked type debt outside this actionable entry-point
surface.

## Choose the smallest useful tier

The repository has four development tiers. Start vehicle work with one
vertical Composition slice; the broad `test` task is a
regression gate, not the inner loop: it still selects more than two thousand
tests even though it excludes the explicitly marked `slow`, `artifact`, and
`simple_aero` categories.

```bash
python -m tools.dev test-vehicle f16_s119  # one runnable F-16 Composition path
python -m tools.dev test-f16               # convenience alias for the same slice
python -m tools.dev test-vehicle a320_openap_3dof
python -m tools.dev test-vehicle hummingbird
python -m tools.dev check-vehicle hummingbird  # scoped interface + witnesses + parity + vertical test
python -m tools.dev plugin-wheel-smoke  # isolated installed-wheel boundaries for every split plug-in
python tools/verify_plugin_wheels.py --plugin cross-plugin-deployment  # selected NESC + passive child handoff
python -m tools.dev test-vehicle x15
python -m tools.dev check-vehicle-maturity
python -m tools.dev test-vehicle simple_aero  # runnable fixed-L/D batch and persistent-session workflow
python -m tools.dev test-vehicle dual_launch_glider  # source-generated point-mass batch forms
python -m tools.dev test-debug-models  # isolated ballistic, waypoint, and contract-probe provider workflow
python -m tools.dev check-daveml  # selected DAVE-ML format handler and installed-wheel boundary
python -m tools.dev test-reachability  # exact reachability overlay data and deferred-registration boundary
python -m tools.dev check-reachability  # plus the direct-dependency installed-wheel boundary
python -m tools.dev test-cadac-discovery  # CADAC entry point without an actor sweep
python -m tools.dev test-vehicle cadac_aim5  # one selected source-bound CADAC actor
python -m tools.dev test-vehicle-catalogue  # campaign declarations must have vertical coverage
python -m tools.dev test-control-api-pilot  # four reduced-order control/API fixtures only
python -m tools.dev test-quick      # curated smoke/contracts; stop on first failure
python -m tools.dev test-changed     # changed tests, or test-quick when no mapping exists
python -m tools.dev test-parallel    # broad fast suite across workers, optional xdist
python -m tools.dev test-integration # end-to-end and package-vertical view
python -m tools.dev test-matrices    # full catalog, grid, and cross-product view
python -m tools.dev test             # broad local regression suite
python -m tools.dev check            # full handoff/release validation
```

For a focused edit, direct pytest selection is still the fastest option:

```bash
python -m pytest tests/unit/test_runtime_algorithms.py -q -x
python -m pytest --lf -q -x
# Validate native output-channel IDs, fidelity/mission applicability, and core
# versus telemetry classification without executing a long route.
python -m pytest tests/unit/test_native_output_contract.py -q
```

`test-control-api-pilot` is the rapid loop for selectable control schemes. It
covers the API stressor, analytical ballistic and waypoint sessions, Simple
Aero, generic session switching, flattened discovery metadata, and the
metadata-only reduced F-16 tier comparison. It intentionally does not execute
direct-wrench screens, surface/effector allocation, every CADAC source case,
or the broad vehicle catalogue. Run those separate vertical or release gates
only when their implementation changes or at a promotion checkpoint.

When changing a source executor's emitted fields or an advertised native
output map, run that structural contract followed by the exact affected
`vehicle_execution_witnesses.yaml` witness. Reserve the all-witness execution
run for a promotion checkpoint: full source routes can be intentionally much
more expensive than a local controller screen.

`test-changed` includes modified test files and uses imports from modified
`src/taoryx` modules to find related tests. For changes with no reliable
mapping—such as shared fixtures, project configuration, or documentation—it
falls back to `test-quick`. This is a feedback aid, not a release-quality
claim; run `test` or `check` before handoff as appropriate.

## Vehicle vertical slices

`test-vehicle <family>` is the focused loop for making a vehicle or workflow
solid. Each slice is explicit rather than a family-marker sweep: it checks the
plug-in advertisement and authoring plan, runs the documented batch endpoint,
exercises an interactive episode only when one is advertised, and runs any
registered controller campaign. It therefore proves the same user-facing
Composition route a model developer works through without pulling unrelated
vehicles or historical evidence into every edit.

The available physical-family slices are `f16_s119`, `a320_openap_3dof`,
`hummingbird`, `x15`, `hl20_mod_k`, `reference_nesc_two_stage_rocket`,
`tumbling_body`, `skywalker_x8`, and `b747`. `simple_aero` is the focused
package-owned fixed-L/D workflow slice: it proves its provider advertisement,
generated batch path, persistent generated-command/direct-throttle session,
and workflow endpoint data boundary, not a physical vehicle or actuator claim.
`dual_launch_glider` proves the source-generated point-mass batch path
for both launch forms and reports attached-booster separation as an event; it
does not claim an independently propagated released-glider history. Add the
next slice only after its documented composition has a real runnable endpoint;
the registry lives in `tools/dev.py` as
`VEHICLE_VERTICAL_TEST_PATHS`. This keeps the work vertical: establish one
model's data, controls, segment contract, execution, and tuning footing before
broadening the matrix. A narrow local controller screen—such as X-15's
direct-wrench route—must preserve that boundary rather than being labeled a
full flight mission.

`test-debug-models` is a separate non-vehicle plug-in gate for the analytical
ballistic and waypoint fixtures plus the synthetic contract probe. It proves
their package version, model/control advertisements, selected plug-in scope,
streamed action/readback paths, package-owned endpoint data, and concrete batch
results. It does not scan or execute physical vehicle families.

`check-daveml` is the corresponding shared-format gate. It selects only the
`daveml` model-format contribution, verifies its package revision, invokes the
lazy handler, and proves that the imported DAVE-ML implementation comes from an
installed core-plus-DAVE-ML wheel. It does not construct a consumer vehicle or
replay F-16, HL-20, or NESC source data.

`check-reachability` is the optional-overlay gate. It verifies the exact
reachability package data, direct X-15/HL-20 dependency declarations, deferred
capability/preflight/execution registration, and an installed-wheel boundary.
The X-15, HL-20, and DAVE-ML wheels are present only as installation and
discovery dependencies; their controller, trim, and study gates are not rerun.

`check-vehicle <family>` is the stronger plug-in-owned gate for a physical
vehicle family. It scopes the semantic-interface catalog report, endpoint
witness validation, registered batch/episode parity replay, and the executable
vertical pytest slice to that one family. It does not execute DAVEML, CADAC,
debug-provider, parser, or unrelated vehicle suites. Use it before handing a
vehicle change to catalogue integration; use `test-vehicle` while iterating on
the vehicle's plant or controller implementation.

`plugin-wheel-smoke` is the packaging boundary check for the currently split
plug-ins. It builds fresh source-distribution-derived wheels, installs core
plus the selected plug-ins into a temporary target, removes all editable source
roots from that child process, and then exercises each declared boundary:
providers, model-format lazy imports, packaged model data, and concrete
adapters where applicable. A selected package's dependency wheels are installed
and discovered, but their own vertical assertions are skipped unless explicitly
selected. Use it when changing package ownership, entry
points, package data, or deferred registration; it is not a catalogue-wide
mission regression run.

The validation layers are deliberately separate:

1. `test-vehicle <family>` — one vehicle's executable pytest slice.
2. `check-vehicle <family>` — one physical vehicle plug-in's full host-contract
   boundary.
3. `vehicle-catalogue` — cross-plug-in metadata reconciliation without broad
   mission execution.
4. `check` — repository integration/release validation, including unrelated
   packages and documentation.

Exact per-vehicle channels, authorities, units, lowering chains, and readbacks
belong in that vehicle's vertical tests. Catalogue tests check structural
invariants and uniqueness; they must not pin a global sum that changes whenever
an independent plug-in adds a legitimate channel.

For a catalogue-wide metadata change, run `python -m tools.dev
vehicle-catalogue`. It checks registered controller-campaign coverage, family
membership, semantic interfaces, and onboarding data for the runnable
catalogue without promoting planned fidelity tiers. It does not run every
vehicle or imply mission qualification. Keep this distinct from
`test-vehicle <family>`, which is the fast executable proof for one specific
vehicle.

`test-parallel` requires the development extra, which includes
`pytest-xdist`. It uses `--dist loadfile` so tests from one file stay on one
worker. Before relying on parallel execution, tests must write only to
`tmp_path`, the shared `artifact_dir` fixture, or another worker-safe output
root; fixed repository-root outputs can race.

Both broad runners print the 25 slowest tests. Use that report to maintain the
cost boundary: apply `slow` to an intentionally long simulation, a full
catalog/grid sweep, or a test that consistently exceeds roughly 20 seconds on
the reference development machine. Do not mark a test `slow` merely because
its `max_steps` is high when its normal stop condition makes it fast. Apply the
mark to the exact parameter or test when possible; use a module mark only when
the whole module has that cost. Parallel duration reports include worker and
CPU contention, so use an isolated or serial timing to decide a marker; use
the parallel runner to measure end-to-end wall-clock time.

| Marker | Meaning |
| --- | --- |
| `grammar` | Parser, lexer, EBNF, corpus, and language-validation view |
| `equations` | Equation catalog, implementation, provenance, and verification view |
| `algorithms` | Algorithm catalog, runtime bindings, and algorithm-verification view |
| `slow` | Long-running tests, including stress and historical runtime cases |
| `artifact` | Tests that intentionally write human-readable output under `artifacts/` |
| `simple_aero` | Simple Aero problem, segment, and trajectory corpus tests |
| `daveml` | DAVE-ML parser, evaluator, collection, and round-trip view |
| `integration` | Cross-component or end-to-end integration view; all `tests/e2e/` and `tests/families/` tests receive it |
| `matrix` | Full cross-product, grid, or catalog-sweep view; additive, not a cost label |
| `external_oracle` | Tests that invoke an optional external model interpreter |
| `runtime` | Tests requiring a historical or separately verified TAOS executable |
| `historical` | Exploratory or historically sourced runtime cases |
| `stress` | Intentionally large or long-running runtime cases |
| `segment` | Isolated segment contracts, maneuver objectives, and promotion gates |
| `plot` | Plotting and visualization-only checks; normally unit-level and fast |
| `negative_runtime` | Malformed inputs for executable-level rejection checks |
| `metamorphic` | Tests comparing multiple TAOS executions |
| `b747` | Boeing 747 family plant, trajectory, controller, and artifact tests |
| `x8` | Skywalker X8 family plant, trajectory, controller, and artifact tests |
| `hummingbird` | AscTec Hummingbird family plant, rotor, trajectory, and artifact tests |
| `x15` | X-15 family plant, glider, and artifact tests |

Collection assigns `integration` to end-to-end and package-vertical tests, and
assigns `matrix` to files named `test_*_matrix.py`. A broader cross-product
test may declare `matrix` explicitly. Both are semantic selection views; only
`slow`, `artifact`, and `simple_aero` alter the default fast selection.

## Inventory assertions in a plug-in host

Use an exact inventory count only when the artifact itself is a closed,
versioned corpus (for example, the historical equation registry). A test over
discoverable plug-ins should instead enumerate the current advertised
identities and prove that every one was exercised or projected. Package-local
tests may assert their required contribution IDs, but should not rely on a
total contribution count when an additive package change is valid.

Repository tests normally use `include_external=False`, so installing an
unrelated compatible wheel does not change the checked-in release scope. A
host or test that intentionally enables external discovery must treat an
additional valid provider, model, campaign, or endpoint as additive rather
than as a failed snapshot count.

## Planning documents are not unit-test contracts

Do not make a unit test assert a roadmap, backlog, milestone, release plan,
or the current completion state of a planning report. Those are deliberately
short-lived decision records. Test executable planners, validators, and public
CLI/API boundaries with stable inputs instead. A machine-readable file that is
consumed at runtime may have schema and behavior coverage, but its temporary
delivery ordering or project-status wording is not a permanent regression
contract.

The portable development runner provides the usual selections:

```bash
python -m tools.dev test             # fast tests: excludes slow/artifact/simple_aero
python -m tools.dev test-all         # every test category
python -m tools.dev test-integration # e2e and package-vertical integration view
python -m tools.dev test-matrices    # catalog/grid/cross-product matrix view
python -m tools.dev test-simple_aero     # only Simple Aero tests
python -m tools.dev test-artifacts   # only artifact-producing tests
python -m tools.dev test-slow        # only slow tests
python -m tools.dev test-grammar     # grammar/parser view
python -m tools.dev test-equations   # equation/provenance view
python -m tools.dev test-algorithms  # algorithm catalog/binding view
python -m tools.dev test-segments    # isolated segment contracts and gates
python -m tools.dev test-plots       # plotting-only view
python -m tools.dev test-views       # print all views and categories
python -m tools.dev test-b747        # only B747 tests, including slow/artifact cases
python -m tools.dev test-x8          # only Skywalker X8 tests, including slow/artifact cases
python -m tools.dev test-hummingbird # only Hummingbird tests, including slow/artifact cases
python -m tools.dev test-x15         # only X-15 tests, including slow/artifact cases
python -m tools.dev test-x15-catalog # fast X-15 catalog/source/table checks
python -m tools.dev test-x15-segments # X-15 catalog plus isolated segment gates
```

The equivalent direct pytest expressions are:

```bash
# This is also the configured default for bare `python -m pytest`.
python -m pytest -m "not slow and not artifact and not simple_aero"
python -m pytest -m ""
python -m pytest -m simple_aero
python -m pytest -m artifact
python -m pytest -m slow
python -m pytest -m grammar
python -m pytest -m equations
python -m pytest -m algorithms
python -m pytest -m segment
python -m pytest -m plot
python -m pytest -m b747
python -m pytest -m x8
python -m pytest -m hummingbird
python -m pytest -m x15
```

## Agent slices: choose the smallest useful view

Use the catalog and markers as a test router. Do not run the whole vehicle
family when the change is limited to one dynamics tier or one segment:

| Work focus | Command | Expected cost |
| --- | --- | --- |
| X-15 menu/source/table traceability | `python -m tools.dev test-x15-catalog` | fast unit checks |
| X-15 3-DOF segment | `python -m pytest tests/e2e/test_glider_family_validation.py -m 'x15 and segment and dof3' -k phugoid -o addopts=''` | one focused runtime |
| X-15 6-DOF segment | `python -m pytest tests/e2e/test_glider_family_validation.py -m 'x15 and segment and dof6' -k weave -o addopts=''` | one focused runtime |
| All isolated X-15 gates | `python -m tools.dev test-x15-segments` | slow, but excludes route/artifact-only tests |
| Grammar-only change | `python -m tools.dev test-grammar` | parser slice |
| Plot/visualization change | `python -m tools.dev test-plots` | fast visualization slice |
| Full vehicle family | `python -m tools.dev test-x15` | intentionally expensive |

The common controller-campaign catalog test validates registrations and their
portable advertisements without constructing every numerical plant.  The
cross-catalog runner proof intentionally executes every campaign and is marked
`slow`; opt into it only when changing the shared tuning runner or campaign
registration contract:

```bash
python -m pytest tests/unit/test_model_authoring.py::test_registered_reference_campaigns_execute_through_the_common_runner -m slow -o addopts=''
```

Likewise, the all-provider `model assess` matrix and its CLI proof are marked
`slow`. They are catalogue-release checks, not prerequisites for a focused
vehicle plug-in edit.

The `segment`, `dof3`, and `dof6` markers are additive views. `slow` is a cost
label, not a reason to select every slow test. `-o addopts=''` is required for
an explicit opt-in slice because the repository default excludes `slow`,
`artifact`, and `simple_aero`. The terms “view” and “shard” remain distinct: views
overlap, while a future CI shard must be a disjoint partition.

Vehicle-family markers are selective views, not disjoint CI shards. They are
applied to the family-specific modules and to mixed catalog parameters where
possible, so `-m b747` does not pull in X8, Hummingbird, or X-15 scenarios.
The explicit command-line marker replaces the fast default marker expression;
therefore family commands include their slow and artifact cases intentionally.

Artifact tests should request the shared `artifact_dir` fixture. It creates a
test-specific directory below `artifacts/`, which is ignored by Git. A
different output root can be selected with `--artifact-dir`, for example:

```bash
python -m pytest -m artifact --artifact-dir /tmp/taoryx-artifacts
```

Markers are intentionally additive. A test can be both `slow` and `simple_aero`,
or both `slow` and `artifact`, when both properties apply. The terms “view”
and “shard” are kept distinct: these views overlap; a future CI shard must be
a disjoint partition of the collected tests.
