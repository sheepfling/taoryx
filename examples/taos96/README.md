# Historical TAOS 96.0 examples

This index contains historical-language source and transcribed outputs. Parse
it with the `taos96` profile:

```bash
taoryx-validate --profile taos96 \
  examples/taos96/syntax_fragments/chapter03/*.tbl \
  examples/taos96/full_examples/chapter04/*.prb
```

To produce one machine-readable report for the whole historical corpus:

```bash
PYTHONPATH=src python examples/validate_corpus.py \
  --family taos96 --output artifacts/examples/taos96
```

The directories are links to the repository's canonical `examples/chapter03`
and `examples/chapter04` files, so source provenance and existing test paths
remain unchanged. Historical `.txt` printouts are transcriptions of the
manual; they are not regenerated runtime data.

## Layout

- `syntax_fragments/chapter03/` — table and syntax-focused historical inputs.
- `full_examples/chapter04/` — complete problem/table examples plus source
  output listings.
- `manifest.yaml` — machine-readable inventory and claim boundary.

The repository includes a local TAOS96-compatible runtime path. Examples with
the required table decks can be executed and plotted; examples whose original
table decks are absent are reported as unavailable. This is not a claim of
bit-for-bit compatibility with the historical TAOS executable.
