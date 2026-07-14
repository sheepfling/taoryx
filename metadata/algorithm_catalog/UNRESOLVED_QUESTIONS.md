# Unresolved Questions and Historical Boundaries

The following items require an implementation decision, a cited primary source,
or access to historical TAOS code/output before exact behavior can be claimed.

## TAOS-ALG-ENV-005 — High-Altitude Atmosphere Interpolation

- **Question:** The manual does not specify extrapolation behavior above 1000 km.

## TAOS-ALG-GEO-001 — Sodano Inverse Ellipsoidal Geodesic

- **Question:** Document numerical behavior near antipodal configurations, which the manual does not discuss.

## TAOS-ALG-GEO-003 — Sodano Direct Ellipsoidal Geodesic

- **Note:** Preserve a traceable implementation decision for the source inconsistency in equation 2-239.

## TAOS-ALG-IIP-002 — Fehlberg Adaptive Runge-Kutta 4/5 Integration

- **Question:** The manual cites Fehlberg but does not print the exact embedded tableau; select and document the historical variant used.

## TAOS-ALG-OPT-002 — Han-Powell Recursive Quadratic Programming

- **Question:** The manual gives an overview but not the full vf02ad source algorithm; obtain or independently reimplement the referenced method.

## TAOS-ALG-TABLE-006 — Full-Table Storage Variable Semantics

- **Question:** The manual requires unique names but does not describe recovery from duplicate csto labels.

## TAOS-ALG-TABLE-008 — Skewed Tabulated-Data Evaluation

- **Question:** The manual describes data organization but not a unique interpolation order for all possible sparse outer grids; document the chosen recursive semantics.

## Successor data-model questions

These are not historical TAOS gaps. They are backlog items for the richer
vehicle-data layer described in `docs/architecture/vehicle-data-model.md`.

- **Question:** How should air-breathing engine decks represent installed thrust,
  fuel flow, engine state, and operating-envelope failures in a way that remains
  distinct from the legacy rocket thrust table?
- **Question:** Which control-effectors should be modeled as linear derivatives
  first, and which should be promoted directly to nonlinear increment tables or
  full configuration-specific aerodynamics?
- **Question:** What manifest format should aggregate geometry, mass properties,
  propulsion, and effectors while preserving provenance for synthetic versus
  measured data?
