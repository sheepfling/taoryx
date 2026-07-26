# NESC DAVE-ML Catalog v1.0

This directory is the canonical repository resource for the promoted NESC
DAVE-ML source catalog used by Alpha 3 round-trip, replay, and showcase
validation. It contains the normalized source documents, catalog metadata,
the qualified two-stage package, and its evidence release.

The original acquisition archive is identified by SHA-256 in `manifest.json`.
`promoted-checksums.sha256` covers every file retained in this repository
resource; `checksums.sha256` is the upstream release ledger preserved for
provenance comparison.

The normalized documents are source-preserving inputs for canonical Taoryx
reconstruction. Their provenance records retain the upstream origin and source
hash without making a temporary acquisition directory part of the runtime
contract.
