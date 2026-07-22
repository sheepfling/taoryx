# Declarative segmentation epic

## Goal

Provide one generic, metadata-driven path from a vehicle description and
segment graph to native `.prb` inputs, resolved manifests, transition audits,
and reusable controller lifecycle behavior. Keep orchestration metadata out of
the TAOS 1995/96 language and prevent bespoke vehicle runners.

## Delivery slices

1. Catalog schema and graph validation — delivered in `taoryx.segmentation`.
2. Native problem compiler and manifests — delivered by
   `tools/compile_segments.py` (`lint`, `build`, and `run`).
3. Controller reset/transition hooks — delivered by
   `SegmentController`.
4. Layered unit tests for graph, source bounds, lifecycle, compiled runtime,
   and four-family compilation — delivered in `tests/unit/test_segmentation.py`.
5. Runtime transition application and state-continuity evidence — native
   `*when`/`goto`, `*reset`, and `*increment` paths are now exercised; richer
   per-channel runtime audit remains next.
6. Per-family true 3-DOF, pseudo-6-DOF bridge, and rigid-body 6-DOF catalogs —
   next; each tier must use a matched scenario contract before parity plots
   are generated. The bridge is an intermediate prescribed/lagged-attitude
   development tier, not a claim of free rotational dynamics.
7. Long-running mission evidence and claim review — after the plant and
   transition audits are green.

The feature is complete only when a new family can add a catalog entry and
source problem/template binding without adding a new runner or changing the
historical parser.
