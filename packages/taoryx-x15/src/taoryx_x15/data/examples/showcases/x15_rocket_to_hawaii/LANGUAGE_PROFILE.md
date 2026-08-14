# X-15 showcase language profile

This showcase is intentionally a TAORYX file, not a claim that the historical
TAOS 96.0 grammar can express a rigid-body 6-DOF vehicle.

## Standard TAOS constructs used

The mission structure uses ordinary problem-language blocks:

- `*title`, `*atmos`, and `*earth`;
- `*trajectory`, `*initial`, and `*file`;
- `*segment`, `*integ`, `*prop`, `*aero`, `*fly`, `*reset`, and `*when`.

The altitude-triggered transitions and time fallbacks are deliberately
expressed with ordinary `*when` blocks. The aerodynamic data remains in `.tbl`
files rather than being embedded in Python.

## Explicit TAORYX extensions

The following are successor-language features and are kept visible in
`*runtime status` declarations:

- `*mode rigid-body-6dof`;
- vehicle inertia and reference geometry;
- route/target telemetry;
- artifact and telemetry configuration.

The generic controller equations are backend capabilities, not X-15-specific
problem-file directives. The X-15 identity is provided by the coefficient
tables, geometry, and mass data.

## Compatibility claim

Parsing this file with `GrammarProfile.TAOS96` must report the TAORYX extension
diagnostic. Parsing it with `GrammarProfile.TAORYX` is the supported claim. The
historical TAOS 96.0 runtime claim remains out of scope.
