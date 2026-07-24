# Examples by language family

Start from one of the two language-family indexes below.

| Family | Purpose | Profile |
| --- | --- | --- |
| [`taos96/`](taos96/) | Historical TAOS 96.0 source and transcribed output | `taos96` |
| [`taoryx/`](taoryx/) | Taoryx syntax extensions and successor runtime examples | `taoryx` |

Each family separates small syntax fixtures from fuller examples. Runnable
examples write the same normalized bundle: `run.json`, `artifact.json`,
`telemetry.csv`, and `plots/`. Generated bundles belong under `artifacts/`, not
inside the source example directories.

For the complete junior workflow, regenerate both dialect reports and all
Taoryx full-example artifacts with one command:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family all --execute --output artifacts/examples/all
```

This parses both dialects and executes the TAOS96-compatible local-runtime
subset plus Taoryx. TAOS96 cases whose historical table decks are not bundled
are reported as unavailable; this local runtime is not a claim of bit-for-bit
compatibility with the original TAOS 96 executable.

The existing `chapter03/`, `chapter04/`, `mission_families/`, and `showcases/`
directories remain stable compatibility paths. The family indexes point to
those canonical files while giving new contributors one predictable starting
place.
