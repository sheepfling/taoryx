# TAOS 1995 manual reconstruction

The canonical manual source now lives under `manual/` in `manual/manual.tex`,
the chapter directories, and the shared LaTeX assets. This directory holds
editorial and handoff notes about that source tree.

Each chapter should have:

1. a source/transcription note,
2. a readable Markdown draft,
3. links to the example files and equations it defines, and
4. tests or validation fixtures where the chapter specifies machine-readable
   behavior.

## Suggested chapter workflow

Keep historical observations separate from implementation decisions. Use
`Source notes` for what the original manual says, `Open questions` for text or
syntax that still needs confirmation, and `Python mapping` for the behavior
that `taoryx` is expected to provide.

The reconstructed source currently covers:

- introduction and execution model
- methods and equations of motion
- tables (`.tbl`)
- problems (`.prb`)
- equations, output variables, and numbering
- simulation/runtime boundaries
