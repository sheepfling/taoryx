# Runtime-ready Taoryx examples

These links expose the existing polished Taoryx showcases from the language
family index. Each showcase owns its source, `expected.yaml`, `plot.yaml`, and
runner script; generated products go to `artifacts/showcases/<id>/` using the
same normalized artifact, telemetry, and plot conventions.

Run one from the repository root, for example:

```bash
PYTHONPATH=src python examples/taoryx/full_examples/runtime/suborbital_ballistic_return/run_showcase.py
```

To regenerate every indexed dialect report, full Taoryx case, and runtime
showcase in one pass, use:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family all --execute --output artifacts/examples/all
```

Inspect the resulting `artifact.json` with:

```bash
taoryx artifact plot artifacts/showcases/suborbital_ballistic_return/run/artifact.json \
  --output-dir artifacts/showcases/suborbital_ballistic_return/plots
```

The source directories remain canonical under `examples/showcases/`; this
folder is a stable discovery path for new contributors.
