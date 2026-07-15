# Public hypersonic research dataset v1.0.0

This archive collects public-source aerodynamic, mass-property, and propulsion data for non-weapon hypersonic research simulations. It combines independent research configurations; **it is not one coherent vehicle database**.

## Contents

- NASA Langley winged-cone M>=6 longitudinal coefficient reconstruction.
- Direct X-33 hypersonic inviscid CFD coefficient tables.
- X-43A/HXLV separation mass properties and uncertainty data.
- X-43A flight-test envelope records and reported model discrepancies.
- Conceptual winged-cone scramjet map with fuel-flow limits.
- Pegasus/Orion manufacturer summary tables.
- Impulse-normalized Orion research thrust-curve surrogates.
- Source-envelope records for waverider and Mach-6 sideslip tests.

## Fidelity labels

- `DIRECT_TRANSCRIPTION`: copied from an explicit source table or statement.
- `EQUATION_RECONSTRUCTION`: generated from published equations.
- `FIGURE_RECONSTRUCTION`: generated from published plotted functions or breakpoints.
- `DERIVED_*`: unit conversion or arithmetic from source values.
- `PLOT_SHAPE_SURROGATE_IMPULSE_NORMALIZED`: non-certified curve constrained to exact source totals.
- `SOURCE_ENVELOPE_ONLY`: applicability metadata, not a pointwise curve table.

## Critical limitations

1. Do not splice the X-33, X-43A, waverider, and winged-cone rows into one vehicle without geometry-aware scaling.
2. The winged-cone table is longitudinal only; it has no beta, control-effectiveness, or dynamic-derivative deck.
3. X-33 rows are inviscid and omit base/aerospike rear-facing surfaces and viscous separation.
4. X-43A/HXLV mass properties apply at separation and use source-specific frames.
5. Orion time histories are research surrogates, not certified motor curves.
6. The scramjet model is a conceptual 1990s simulation deck, not a validated modern engine.
7. No terminal-guidance, targeting, or weapon-specific tuning is included.

For the reconstructed winged-cone body coefficients, x is forward, y right, z down. Keep aerodynamic moments about the published fixed reference and shift them to a moving CG at runtime.

Generated 2026-07-15.
