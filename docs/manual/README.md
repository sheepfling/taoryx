# TAOS manual notes

The canonical manual source lives under `manual/` in `manual/manual.tex`, the
chapter directories, and the shared LaTeX assets. This directory collects the
notes that help a reader understand the reconstruction, rebuild the PDF, and
trace the manual back into the equation and grammar layers.

If you only need the working commands:

```bash
python tools/dev.py manual
python tools/dev.py equation-audit
python tools/dev.py check
```

## What belongs here

- chapter-level source notes and transcription caveats
- open questions that still need historical confirmation
- cross-links from chapters to examples, equations, and fixtures
- handoff guidance for the reconstruction process
- manual-build and provenance notes that belong with the source edition

## What does not belong here

- new canonical manual text
- generated LaTeX or provenance outputs
- runtime implementation plans that belong under `docs/architecture/`
- parser and grammar workflow notes that belong under `docs/grammar/`

## Working rule

Keep historical observations separate from implementation decisions. Use
`Source notes` for what the original manual says, `Open questions` for text or
syntax that still needs confirmation, and `Python mapping` for the behavior
that `taoryx` is expected to provide.

## Useful references

- [Manual build and provenance workflow](../BUILDING.md)
- [Equation registry](../equations/registry.md)
- [Equation implementation bindings](../equations/implementation-bindings.md)
- [Grammar validation guide](../grammar/README.md)
