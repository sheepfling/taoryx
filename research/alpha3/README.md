# Alpha 3 research and resource registry

This directory is the source-controlled index for vehicle research that will
feed Alpha 3 family packages, source anchors, surrogates, corpora, and
qualification work. It separates research notes and machine-readable
configuration from raw external payloads.

The registry is not a claim that every listed vehicle is executable. Each
record declares its current evidence and integration state:

- `checked_in_metadata`: plans, source locks, family manifests, schemas, and
  verification contracts live in this repository.
- `external_hash_pinned`: the raw source or runtime payload is kept outside
  Git, with a required path, hash, provenance record, and reproduction command.
- `research_ready`: the evidence and intended model boundary are recorded, but
  a Taoryx family package is not yet qualified.
- `runtime_pending`: metadata exists, but a native provider/runtime binding or
  source replay gate remains.

## Raw-source policy

Raw DAVE-ML, `.txair`, public research archives, and other large or
license-sensitive payloads must not be copied into Git merely to make a
directory look complete. They remain external when the registry records:

1. source URI or repository;
2. exact local acquisition path or archive member;
3. SHA-256 and revision/date;
4. license and attribution requirements;
5. source role and intended model boundary;
6. the command that validates or ingests the payload.

The aerospace DAVE-ML source bundles used by Alpha 3 are promoted under
`resources/aerospace/daveml/` with immutable manifests and hash ledgers. A
temporary acquisition directory may be used during intake, but it is never a
runtime or provenance dependency.

## Contributor workflow

1. Start with [source-register.yaml](source-register.yaml).
2. Read the linked research plan and integration notebook.
3. Check the family maturity and source-lock records before adding parameters.
4. Keep source, derived, estimated, assumed, and unavailable values separate.
5. Run the family-specific intake/preflight command before writing a runtime
   binding.
6. Add the failure or missing-resource note to the relevant notebook.
7. Promote a family only when its registry status and machine-readable evidence
   agree.

The most developed workflow is the F-16/HL-20 DAVE-ML path:

```text
external source package
        ↓
source lock and package preflight
        ↓
loss-aware .txcollection
        ↓
native replay and source regressions
        ↓
separate actuator/controller/mission overlays
```

Alpha 3 broadens this same path to spacecraft, rotorcraft/tiltrotors,
public-data flight-dynamics surrogates, small aircraft, jets, fleets, and
trajectory corpora.
