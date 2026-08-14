# Mass-model notes

The source-native schedule is coherent with the executable X-15 model, not with every published X-15 configuration.

- Empty vehicle: 14,560 lbm at structural station 345 in.
- Main oxidizer: 9,470 lbm at station 282.3 in.
- Main fuel: 8,236 lbm at station 408.3 in.
- RCS oxidizer: 12 lbm at station 140 in.
- Main propellants deplete independently at the source maximum flow rates.
- Burn ends when the first main tank reaches zero; the small residual in the other tank is retained.
- CG is a mass-weighted structural station.
- Variable inertia is derived by treating tanks as centerline point masses and shifting the published empty inertia with the parallel-axis theorem.
- Tank internal inertia, slosh, ullage migration, plumbing mass, pilot/experiment changes, and ventral-fin jettison are absent.

NASA launch and burnout figures are retained as independent anchors. They are not silently substituted into the JSBSim mass model.
