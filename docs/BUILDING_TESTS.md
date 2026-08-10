# Running the test suite

The repository uses explicit pytest markers for cost categories and overlapping
test views. Views are intentionally not mutually exclusive: one test may
belong to both the equations and algorithms views.

To print the taxonomy from the portable task runner:

```bash
python -m tools.dev test-views
```

Use `pytest --markers` to inspect the registered marker descriptions directly.

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
python -m tools.dev test-vehicle x15
python -m tools.dev check-vehicle-maturity
python -m tools.dev test-vehicle simple_aero  # runnable fixed-L/D workflow, batch only
python -m tools.dev test-vehicle dual_launch_glider  # source-generated point-mass batch forms
python -m tools.dev test-vehicle-catalogue  # campaign declarations must have vertical coverage
python -m tools.dev test-quick      # curated smoke/contracts; stop on first failure
python -m tools.dev test-changed     # changed tests, or test-quick when no mapping exists
python -m tools.dev test-parallel    # broad fast suite across workers, optional xdist
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
fixed-L/D workflow slice: it proves the provider-generated batch path and its
explicitly blocked interactive boundary, not a physical vehicle or actuator
claim. `dual_launch_glider` proves the source-generated point-mass batch path
for both launch forms and reports attached-booster separation as an event; it
does not claim an independently propagated released-glider history. Add the
next slice only after its documented composition has a real runnable endpoint;
the registry lives in `tools/dev.py` as
`VEHICLE_VERTICAL_TEST_PATHS`. This keeps the work vertical: establish one
model's data, controls, segment contract, execution, and tuning footing before
broadening the matrix. A narrow local controller screen—such as X-15's
direct-wrench route—must preserve that boundary rather than being labeled a
full flight mission.

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

| Marker | Meaning |
| --- | --- |
| `grammar` | Parser, lexer, EBNF, corpus, and language-validation view |
| `equations` | Equation catalog, implementation, provenance, and verification view |
| `algorithms` | Algorithm catalog, runtime bindings, and algorithm-verification view |
| `slow` | Long-running tests, including stress and historical runtime cases |
| `artifact` | Tests that intentionally write human-readable output under `artifacts/` |
| `simple_aero` | Simple Aero problem, segment, and trajectory corpus tests |
| `segment` | Isolated segment contracts, maneuver objectives, and promotion gates |
| `plot` | Plotting and visualization-only checks; normally unit-level and fast |
| `b747` | Boeing 747 family plant, trajectory, controller, and artifact tests |
| `x8` | Skywalker X8 family plant, trajectory, controller, and artifact tests |
| `hummingbird` | AscTec Hummingbird family plant, rotor, trajectory, and artifact tests |
| `x15` | X-15 family plant, glider, and artifact tests |

The portable development runner provides the usual selections:

```bash
python -m tools.dev test             # fast tests: excludes slow/artifact/simple_aero
python -m tools.dev test-all         # every test category
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
