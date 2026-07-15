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
- [Showcase catalog](showcase-catalog.md): polished, reproducible examples
  and their expected plots.

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
