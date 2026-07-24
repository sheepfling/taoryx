# Taoryx examples

This is the successor syntax and feature-extension corpus. Use the `taoryx`
profile explicitly:

The focused language reference is
[`docs/extensions/taoryx-language-reference.md`](../../docs/extensions/taoryx-language-reference.md).

```bash
taoryx-validate --profile taoryx \
  examples/taoryx/syntax_fragments/*.prb \
  examples/taoryx/syntax_fragments/*.tbl \
  examples/taoryx/full_examples/*.prb
```

To produce one machine-readable AST/diagnostic report for every Taoryx source
file:

```bash
PYTHONPATH=src python examples/validate_corpus.py \
  --family taoryx --output artifacts/examples/taoryx/validation
```

## Layout

- `syntax_fragments/` — one focused fixture per grammar extension family.
- `full_examples/` — larger combinations of the same features.
- `full_examples/runtime/` — discoverable links to runtime-ready showcases
  with expected outputs and plot specifications.
- `manifest.yaml` — stable IDs, source paths, profile, and output contract.
- `output_contract.yaml` — the common run/report/artifact/plot layout.

The full extension examples are runtime-and-artifact fixtures. A short smoke
run may end with exit code `1` when it reaches the step budget before a stop
event; that still leaves a normalized artifact and plots. Exit code `2` means
parsing or runtime failure.

For runtime-ready Taoryx showcases already in the repository, see
`examples/showcases/` and `examples/mission_families/`; their generated
artifacts follow the same output contract.
