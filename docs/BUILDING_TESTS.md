# Running the test suite

The repository uses explicit pytest markers for cost categories and overlapping
test views. Views are intentionally not mutually exclusive: one test may
belong to both the equations and algorithms views.

To print the taxonomy from the portable task runner:

```bash
python tools/dev.py test-views
```

Use `pytest --markers` to inspect the registered marker descriptions directly.

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
python tools/dev.py test             # fast tests: excludes slow/artifact/simple_aero
python tools/dev.py test-all         # every test category
python tools/dev.py test-simple_aero     # only Simple Aero tests
python tools/dev.py test-artifacts   # only artifact-producing tests
python tools/dev.py test-slow        # only slow tests
python tools/dev.py test-grammar     # grammar/parser view
python tools/dev.py test-equations   # equation/provenance view
python tools/dev.py test-algorithms  # algorithm catalog/binding view
python tools/dev.py test-segments    # isolated segment contracts and gates
python tools/dev.py test-plots       # plotting-only view
python tools/dev.py test-views       # print all views and categories
python tools/dev.py test-b747        # only B747 tests, including slow/artifact cases
python tools/dev.py test-x8          # only Skywalker X8 tests, including slow/artifact cases
python tools/dev.py test-hummingbird # only Hummingbird tests, including slow/artifact cases
python tools/dev.py test-x15         # only X-15 tests, including slow/artifact cases
python tools/dev.py test-x15-catalog # fast X-15 catalog/source/table checks
python tools/dev.py test-x15-segments # X-15 catalog plus isolated segment gates
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
| X-15 menu/source/table traceability | `python tools/dev.py test-x15-catalog` | fast unit checks |
| X-15 3-DOF segment | `python -m pytest tests/e2e/test_glider_family_validation.py -m 'x15 and segment and dof3' -k phugoid -o addopts=''` | one focused runtime |
| X-15 6-DOF segment | `python -m pytest tests/e2e/test_glider_family_validation.py -m 'x15 and segment and dof6' -k weave -o addopts=''` | one focused runtime |
| All isolated X-15 gates | `python tools/dev.py test-x15-segments` | slow, but excludes route/artifact-only tests |
| Grammar-only change | `python tools/dev.py test-grammar` | parser slice |
| Plot/visualization change | `python tools/dev.py test-plots` | fast visualization slice |
| Full vehicle family | `python tools/dev.py test-x15` | intentionally expensive |

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
