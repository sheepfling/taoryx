# TAORYX documentation

This directory is organized by **audience and ownership**, rather than by the
order in which documents happened to be written. Start from the row that
matches the question you are trying to answer.

| Need | Canonical location | What belongs there |
| --- | --- | --- |
| Integrate a host, provider, UI, or control client | [API reference](api/README.md) | Stable schemas, provider protocols, value semantics, and wire artifacts |
| Add or maintain a TAORYX package | [Developer guides](developer/README.md) | Package authoring, discovery, focused validation, and model automation |
| Find one model/provider package | [Plug-in directory](plugins/README.md) | The package-owned README and any package-local design notes |
| Understand shared implementation design | [Architecture](architecture/README.md) | Runtime design, fidelity, algorithms, and internal boundaries |
| Run TAORYX as a user or operator | [Top-level guides](../README.md#what-to-read-next) | Installation, Mission Composition, runtime onboarding, and test/build guides |
| Review executable evidence or qualification | [Verification](verification/README.md) | Evidence ladders, maturity limits, and reproducible verification notes |
| Review historical reconstruction sources | [Manual](manual/README.md), [grammar](grammar/README.md), and [equations](equations/registry.md) | Source-linked reconstruction material |
| Read a proposal or work plan | [Plans](plan/) | Time-bounded design and execution records, not normative API reference |

## Placement rule

Put a document at the boundary that owns its facts:

- A reusable consumer/provider contract belongs under `docs/api/`.
- A shared TAORYX implementation design belongs under `docs/architecture/`.
- A contributor workflow belongs under `docs/developer/`.
- A model, source asset, family-specific control law, or package-specific
  witness belongs beside its distribution at `packages/<distribution>/docs/`.
- A cross-package evidence report belongs under `docs/verification/`.
- A plan remains under `docs/plan/` and must link to its normative API or
  architecture source rather than becoming one.

The old `docs/architecture/*-api.md`, loose CADAC pages, and other former
paths are short compatibility pointers. New links and new content must use the
canonical locations above.

## Fast starts

- External consumer/provider: [Trajectory contracts](api/trajectory-contracts.md)
  and [Vehicle Composition Advertisement](api/vehicle-composition-advertisement-api.md).
- TAORYX vehicle contributor: [Interface layers](developer/interface-layers.md),
  then [author a vehicle plug-in](developer/vehicle-plugin-authoring.md).
- One installed package: `python tools/dev.py plugin-focus <wheel-selector>`
  followed by its entry in the [plug-in directory](plugins/README.md).
