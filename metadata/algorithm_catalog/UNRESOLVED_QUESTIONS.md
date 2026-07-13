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
