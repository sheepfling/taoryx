# TAOS manual notes

The canonical manual source lives under `manual/` in `manual/manual.tex`, the
chapter directories, and the shared LaTeX assets. This directory is only for
editorial notes, reconstruction guidance, and handoff context around that
source tree.

## What belongs here

- chapter-level source notes and transcription caveats
- open questions that still need historical confirmation
- cross-links from chapters to examples, equations, and fixtures
- handoff guidance for the reconstruction process

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
