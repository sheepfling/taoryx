# Taoryx CADAC plug-in

`taoryx-cadac` is an optional, source-bound Taoryx plug-in for the public
CADAC aerospace vehicle examples. It contributes the CADAC Mission Composition
catalog and the CADAC table importer without copying any upstream input decks,
coefficient values, or generated trajectories into this repository.

The initial release is deliberately discovery-first. Installing it exposes the
phase-aware CADAC actor catalog through the standard `taoryx.plugins` entry
point, but does not scan an environment variable, parse a source archive, or
construct a vehicle while Taoryx is discovering plug-ins. A source case becomes
executable only when an exact, caller-owned case binding is supplied to the
corresponding CADAC runtime. There is no fallback to another vehicle or to a
synthetic deck.

## Install

From this source checkout:

```bash
python -m pip install -e . -e packages/taoryx-cadac
taoryx plugins check --profile cadac
taoryx plugins inspect taoryx.cadac
```

The package requires an independently installed `taoryx` host. It does not
download CADAC or assert a license for the upstream source material. Obtain a
local, authorized copy separately, for example from the
[`missiondesignsolutions/CADAC`](https://github.com/missiondesignsolutions/CADAC)
repository.

## Convert local CADAC decks

The included converter accepts `1DIM`, `2DIM`, and `3DIM` CADAC table decks and
writes deterministic, provider-neutral `taoryx.table.v1` resources plus a
hash-checked `taoryx.table_bundle.v1` manifest:

```bash
taoryx-cadac convert-tree /path/to/CADAC --output build/cadac-tables
taoryx-cadac validate-table build/cadac-tables
```

`convert-bundle` converts every physical deck referenced by one `input.asc`;
`convert-deck` converts a single table deck. The output records source paths
and SHA-256 digests so generated resources remain traceable to the exact local
source bytes. Do not commit generated bundles containing upstream coefficient
data unless their redistribution status has been independently confirmed.

## Claim boundary

The plug-in preserves source actor identity, source order, table direction,
interpolation policy, and provenance. Its compatibility runtimes are a Taoryx
reconstruction, not a claim of numerical parity with a compiled CADAC
executable. The CADAC integration documentation states each model's current
fidelity and execution boundary.

The source-compatibility code is under a scoped static-typing exception while
the imported NumPy-heavy prototype is typed incrementally. This does not
promote compiled-CADAC numerical parity.
