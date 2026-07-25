# TAORYX extensions

This section documents capabilities that extend the manual-bounded TAOS
language or simulation model. Extension documentation must state its claim
boundary explicitly: useful TAORYX behavior is not automatically historical
TAOS behavior.

## Current extensions

- [Rigid-body 6-DOF](rigid-body-6dof.md): quaternion attitude, body rates,
  force/moment dynamics, mass, and thermal state.
- [Thermal entry](thermal-entry.md): heat-rate and heat-load limits used by
  entry guidance.
- [Terminal guidance](terminal-guidance.md): guidance-to-attitude handoff and
  terminal ProNav boundaries.
- [Problem-file guide](problem-file-guide.md): the documentation contract for
  every `.prb`/`.tbl` fixture.
- [Taoryx language extension reference](taoryx-language-reference.md): focused
  syntax, aliases, AST mappings, validation, and end-to-end artifact commands.
- [Transition truth](transition-truth.md): language boundary, immutable
  pre/post event snapshots, continuity rules, and artifact inspection.
- [Showcase catalog](showcase-catalog.md): polished, reproducible examples
  and their expected plots.
- [LQR](lqr.md): declarative linear-quadratic regulator configuration and
  validated continuous-time solver.
- [TAORYX extensions and verification guide](../latex/taoryx_extensions_and_verification.tex):
  buildable LaTeX documentation for the fidelity ladder, plant data contract,
  automatic tuning, trajectory scoring, and verification gates.
- [TAORYX language and mathematics reference](../latex/taoryx_language_reference.tex):
  manual-parallel coverage of successor math, problem grammar, and table syntax.

Build the composite PDF, which includes that guide plus this extension reference
set, with:

```bash
python tools/dev.py taoryx-extension-pdf
```

The normalized artifact is
`output/pdf/taoryx_extensions_composite.pdf`.

## Claim vocabulary

Every extension uses these labels:

| Label | Meaning |
| --- | --- |
| `manual-bounded` | Supported by the reconstructed 1995 manual only within its documented scope. |
| `taoryx-extension` | New successor functionality, deliberately outside the historical claim. |
| `scaffolded` | The source or contract exists, but the complete runtime oracle is not finished. |
| `verified` | The declared source, runtime behavior, and independent assertions pass. |

The absence of a historical executable and complete table library prevents the
project from claiming TAOS 96.0 behavioral compatibility for these extensions.
