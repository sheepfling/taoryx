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
| `spectre` | Spectre problem, segment, and trajectory corpus tests |
| `b747` | Boeing 747 family plant, trajectory, controller, and artifact tests |
| `x8` | Skywalker X8 family plant, trajectory, controller, and artifact tests |
| `hummingbird` | AscTec Hummingbird family plant, rotor, trajectory, and artifact tests |
| `x15` | X-15 family plant, glider, and artifact tests |

The portable development runner provides the usual selections:

```bash
python tools/dev.py test             # fast tests: excludes slow/artifact/spectre
python tools/dev.py test-all         # every test category
python tools/dev.py test-spectre     # only Spectre tests
python tools/dev.py test-artifacts   # only artifact-producing tests
python tools/dev.py test-slow        # only slow tests
python tools/dev.py test-grammar     # grammar/parser view
python tools/dev.py test-equations   # equation/provenance view
python tools/dev.py test-algorithms  # algorithm catalog/binding view
python tools/dev.py test-views       # print all views and categories
python tools/dev.py test-b747        # only B747 tests, including slow/artifact cases
python tools/dev.py test-x8          # only Skywalker X8 tests, including slow/artifact cases
python tools/dev.py test-hummingbird # only Hummingbird tests, including slow/artifact cases
python tools/dev.py test-x15         # only X-15 tests, including slow/artifact cases
```

The equivalent direct pytest expressions are:

```bash
# This is also the configured default for bare `python -m pytest`.
python -m pytest -m "not slow and not artifact and not spectre"
python -m pytest -m ""
python -m pytest -m spectre
python -m pytest -m artifact
python -m pytest -m slow
python -m pytest -m grammar
python -m pytest -m equations
python -m pytest -m algorithms
python -m pytest -m b747
python -m pytest -m x8
python -m pytest -m hummingbird
python -m pytest -m x15
```

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

Markers are intentionally additive. A test can be both `slow` and `spectre`,
or both `slow` and `artifact`, when both properties apply. The terms “view”
and “shard” are kept distinct: these views overlap; a future CI shard must be
a disjoint partition of the collected tests.
