# TAOS manual snippet corpus v22

This directory is the tracked testing baseline promoted from the v22 intake
corpus. It contains 164 source-faithful Chapter 3/4 displays, generated
synthetic wrappers for fragment-level parser tests, the source-page manifest,
grammar-reference metadata, parser observations, and textual source audits.

The `snippets/` files are historical evidence. The `wrappers/` files are
synthetic test inputs and must not be treated as reconstructed TAOS source.
`manifest.json` is authoritative for source hashes, provenance, completeness,
parse mode, and expected outcome. Output listings and documentation/metasyntax
records remain in the corpus for provenance but are not sent to the TAOS
parsers.

The excerpt PDF from intake is intentionally not duplicated here; the
repository's authoritative manual/source-page workflow remains the source of
appearance evidence. The corpus source hash is retained in the manifests.

Run the baseline with:

```sh
python tools/dev.py grammar
```

The corpus check verifies every raw snippet hash, wrapper reference, lossless
round-trip, lexical comment/numeric example, every complete top-level document,
and that every wrapper can be parsed without an implementation exception.
Fragment diagnostics are retained as coverage evidence: a fragment may be
syntactically valid, intentionally negative, incomplete, or semantically
unsupported without being promoted to a complete-file fixture.
