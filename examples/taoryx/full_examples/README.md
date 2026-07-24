# Taoryx full syntax examples

These cases combine multiple extension families so parser, semantic-AST, and
runtime changes can be tested against realistic inputs. The corpus runner
always emits a parser report, and its Taoryx execution pass emits a normalized
artifact, long-form telemetry CSV, and static plots for every case that reaches
the runtime.

For the complete corpus, produce parser reports and artifacts with:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family taoryx --execute --output artifacts/examples/taoryx
```

Use `--max-steps` to choose between a fast smoke run and a longer integration
budget. A runtime exit of `1` means the case produced artifacts but did not
reach a declared stop event within that budget; parser failures and runtime
exceptions remain exit `2`.
