# Acquisition gaps

The complete NASA/NESC model tree and atmospheric participating-trajectory archive are bundled. Two aggregate official-site downloads were not retrievable through the acquisition path used for this release:

- Initial-condition workbook: `https://nescacademy.nasa.gov/src/flightsim/Scenarios/Initial_Conditions.xlsx`
- Orbital trajectory archive: `https://nescacademy.nasa.gov/src/flightsim/Datasets/Orbital_checkcases.zip`

These are not silently treated as local files. Each orbital case has its official scenario page in the scenario catalog, and every model needed to implement the orbital cases is bundled. The next acquisition pass should add the aggregate archive—or individual published CSV histories—without changing the model identities or catalog IDs.
