# TAORYX showcases

Showcases are polished demonstrations of the runtime and its extensions.
They are separate from the smaller parser and numerical fixtures under
`examples/mission_families/`.

Each showcase should contain:

- `README.md` — narrative and claim boundary;
- `showcase.yaml` — inputs, status, and provenance;
- `expected.yaml` — invariants and tolerances;
- `plot.yaml` — plot panels and telemetry channels;
- source `.prb`/`.tbl` files once the extension parser supports them;
- generated outputs only under ignored `artifacts/` directories.

Family profiles currently include staged launch/coast/entry, long-range
airbreathing flight, orbital insertion/coast/reentry, suborbital ballistic
return, and quadcopter/drone racetrack loiter. Scaffolded profiles explicitly
separate planned telemetry contracts from connected runtime capabilities.

The `3dof_target_intercept` fixture is the small standard-problem proof: it
uses only a `.prb` file and the ordinary runner to exercise a point-mass target
hit. The California-to-Hawaii fixture is the larger native 6-DOF proof and is
kept separately tagged as slow/artifact because it generates the full
telemetry and plot bundle.
