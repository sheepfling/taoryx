# Synthetic two-stage rocket family

This is a deliberately small vehicle-family composition used to exercise the
TAORYX runtime architecture. It is not a historical manual transcription and
its values are not an engineering vehicle dataset.

The case separates reusable data from mission wiring:

- `aero.tbl` contains configuration-specific aerodynamic tables;
- `propulsion.tbl` contains first- and second-stage thrust/mass-flow tables;
- `mission.prb` selects the active tables and defines segment transitions;
- `family.yaml` records provenance, scope, and intended verification claims.

The segment IDs use tens so that later generated cases can insert transitions
without renumbering the entire mission.

To validate the source files:

```text
taoryx-validate mission.prb aero.tbl propulsion.tbl
```

The expected behavior is structural: the first stage burns, separates with a
mass change, the upper stage burns, and a child trajectory inherits the
parent’s state at separation. No historical-runtime or vehicle-performance
claim is attached to this example.
