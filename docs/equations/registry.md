# Equation registry

The canonical equation catalog is loaded from
`metadata/equations_provenance.csv` and exposed by
`src/taoryx/equations/registry.py`.

The registry is keyed by the TAOS equation number, such as `1-1` or `4-8`,
and also exposes lookups by LaTeX label.

| ID | LaTeX label | Section | Notes |
| --- | --- | --- | --- |
| `1-1` | `eq:force-equation` | Trajectory Simulation | Force and moment equations share one source number |
| `1-2` | `eq:point-mass-force` | Trajectory Simulation | Point-mass force equation |

The source of truth remains the provenance CSV rather than this summary
document. Keep this page brief and update it only when the registry contract
changes.
