# Taoryx aircraft-model tooling v0.10

v0.10 adds the exact NASA HL-20 Mod K lifting-body aerodynamic database and a
complete unpowered fixed-mass six-degree-of-freedom reference package.

## Added

- Exact byte-pinned `HL20_aero.dml` source:
  - revision `Mod K, 2010-12-01`;
  - Git blob `71e01b91e3a299a5b272c69f9a8296039aca59eb`;
  - SHA-256 `b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec`.
- Official source gate requiring 361 variables, 8 breakpoints, 241 functions,
  169 gridded tables, 6,247 values, zero parser issues, and 24/24 passing
  embedded shots.
- `Hl20AerodynamicAdapter` with SI/radian inputs, source-unit conversion,
  lift/drag-to-FRD conversion, MRC-to-CG moment transfer, dimensional SI
  output, and explicit clamp diagnostics.
- Separately pinned fixed vehicle binding:
  - mass 8,664.1 kg;
  - CG 0.555 chord fraction;
  - diagonal inertia 10,186.272 / 45,557.464 / 48,333.264 kg·m²;
  - no propulsion.
- `Hl20FixedSurfaceAircraftModel` implementing the canonical `AircraftModel`
  protocol with direct source-surface deflections.
- Deterministic steady-glide solver at 5,000 m and 220 m/s.
- Five-second frozen-atmosphere six-DOF trim-hold validation.
- Normalized alpha-limit, clean-aero, control-increment, and mass-property
  tables.
- Deterministic 16-artifact `.txair` builder and independent verifier that
  regenerates source acceptance, tables, trim, and trajectory evidence.
- CLI commands for source inspection/acquisition, acceptance, SI evaluation,
  table export, glide trim, glide validation, package build, and verification.
- Regression coverage for source-check execution with optional internal values
  disabled.

## Source-count clarification

The historical DAVE-ML examples listing states 168 tables and 6,240 values.
The exact pinned Mod K bytes contain 169 table definitions and 6,247 values due
to a seven-value Mach-dependent angle-of-attack limiter. The release preserves
both records and accepts against the exact bytes.

## Reference glide

```text
altitude:                         5,000 m
true airspeed:                    220 m/s
alpha:                            4.8535019926 deg
symmetric lower body flap:       10.0536518066 deg TED
flight-path angle:              -21.7735518167 deg
pitch attitude:                 -16.9200498241 deg
CL / CD:                          0.1664406611 / 0.0664824099
trim residuals:                  < 1e-6 SI
five-second hold samples:         501
```

## Compatibility

- Package version: `0.10.0`.
- Python: 3.12+.
- Manifest schema: `0.3.0`.
- All v0.9 commands and package readers remain available.


## Release verification

```text
pytest:                    117 passed
Ruff 0.15.22:              passed
Pyright 1.1.411 strict:    0 errors, 0 warnings
compileall:                passed
wheel build:               passed
clean-wheel smoke:         passed
.txair repeated build:     byte-identical
archive CRC checks:        passed
```

The final standalone `.txair` contains 16 declared model artifacts and has
SHA-256 `443e2ed905e310cc57014dc3bad8d3b24b6b5954b8423de9110d405d66154e63`.

## Boundary

This is an unpowered, fixed-mass, direct-surface baseline. It does not include
propulsion, actuator or control-law states, landing contact, flexible-body
behavior, or independent NASA trajectory agreement. The aerodynamic and
mass/inertia records remain separately attributed.
