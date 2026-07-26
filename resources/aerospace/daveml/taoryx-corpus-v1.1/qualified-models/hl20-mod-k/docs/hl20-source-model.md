# NASA HL-20 Mod K unpowered six-degree-of-freedom model

## Result

Version 0.10 converts the exact HL-20 Mod K aerodynamic source into a
self-contained, deterministic Taoryx package and binds a separately pinned
fixed mass/inertia record for unpowered rigid-body use:

```text
exact HL20_aero.dml bytes
    -> Git-blob and revision verification
    -> loss-preserving DAVE-ML parse
    -> 361-variable executable graph
    -> 169 regular tables / 6,247 values
    -> 24 embedded static-shot checks
    -> SI / forward-right-down aerodynamic adapter
    -> separately sourced fixed mass, CG, and inertia
    -> steady-glide trim and six-DOF trim-hold validation
    -> deterministic .txair package and independent reload
```

The aerodynamic source is authoritative for the coefficient graph. The fixed
mass/inertia binding is not declared by that DAVE-ML document; it is retained as
its own provenance record and can be replaced without altering the accepted
aerodynamic source.

## Pinned aerodynamic source

| Field | Value |
|---|---|
| Repository | `DyadLang/DyadDemos` |
| Commit | `9e10c296a173403bdfc993ac626759b6e6f77b2e` |
| Path | `HL20Demo/assets/shared/HL20_aero.dml` |
| Git blob | `71e01b91e3a299a5b272c69f9a8296039aca59eb` |
| SHA-256 | `b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec` |
| DAVE-ML version | `Mod K, 2010-12-01` |
| Exact size | 1,300,776 bytes / 22,456 lines |

The production gate computes the Git object identifier from the selected bytes
and fails closed on any byte or revision mismatch.

## Exact aerodynamic inventory

| Item | Exact pinned document |
|---|---:|
| Variables | 361 |
| Breakpoint definitions | 8 |
| Functions | 241 |
| Gridded table definitions | 169 |
| Table values | 6,247 |
| Ungridded tables | 0 |
| Parser issues | 0 |
| Embedded static shots | 24 |
| Passing static shots | 24 |

The historical DAVE-ML examples listing reports 168 tables and 6,240 values.
The exact pinned Mod K bytes contain one additional seven-value
Mach-dependent angle-of-attack limiter table. Both inventories are preserved;
acceptance uses the byte-pinned document counts.

## Aerodynamic coverage

The source includes subsonic and low-supersonic behavior through Mach 4,
Mach-dependent angle-of-attack limits, sideslip, body-rate damping, ground
effect, landing-gear increments, and direct deflections for:

- four upper/lower left/right body flaps;
- left and right wing flaps;
- rudder.

It returns lift, drag, side-force, roll, pitch, and yaw coefficients. The
canonical adapter converts lift and drag to FRD body coefficients:

```text
CX = CL sin(alpha) - CD cos(alpha)
CY = CY_source
CZ = -CL cos(alpha) - CD sin(alpha)
```

Reference geometry:

```text
S     = 286.45 ft² = 26.612075808 m²
cbar  = 28.24 ft   = 8.607552 m
span  = 13.89 ft   = 4.233672 m
X_MRC = 0.54 reference-chord fraction
```

For requested CG fraction `X_CG`:

```text
dx   = X_CG - X_MRC
Clcg = Clmrc
Cmcg = Cmmrc + dx * CZ
Cncg = Cnmrc - dx * (cbar/span) * CY
```

Dimensional forces and moments use `qbar = 0.5 rho V²`. Every source clamp is
retained as a structured event and changes the evaluation disposition from
`valid` to `clamped`.

## Fixed unpowered vehicle binding

| Quantity | Value |
|---|---:|
| Mass | 8,664.1 kg |
| CG | 0.555 reference-chord fraction |
| Ixx | 10,186.272 kg·m² |
| Iyy | 45,557.464 kg·m² |
| Izz | 48,333.264 kg·m² |
| Ixy, Ixz, Iyz | 0 kg·m² |
| Propulsion | none |

Provenance is recorded separately:

```text
DyadLang/DyadDemos
commit 9e10c296a173403bdfc993ac626759b6e6f77b2e
HL20Demo/assets/shared/hl20_spec.md
Git blob aa7854708d5cc81b96cda8561544c92e5cb35ac7
```

The companion specification states that these values were extracted from NASA
TM-107580, NASA TM-4302, and the HL-20 aerodynamic database. The runtime uses
direct surface deflections; it does not infer a mapping from generic stick or
pedal ratios.

## Reference steady glide

The release solves a symmetric, unpowered trim at 5,000 m and 220 m/s in the
implemented 1976 standard atmosphere:

| Quantity | Solved value |
|---|---:|
| Mach | 0.6863645076 |
| Angle of attack | 4.8535019926° |
| Symmetric lower body flap | 10.0536518066° trailing-edge down |
| Flight-path angle | −21.7735518167° |
| Pitch attitude | −16.9200498241° |
| CL | 0.1664406611 |
| CD | 0.0664824099 |
| Lift | 78,904.0946 N |
| Drag | 31,517.1445 N |
| Weight | 84,965.7963 N |

All three trim residuals are below `1e-6` in their respective SI units.

A five-second frozen-atmosphere six-DOF propagation uses 501 samples at a
0.01-second step. Speed, body velocity, attitude, and angular-rate drift remain
at numerical roundoff while the vehicle advances along the solved descending
flight path. This validates force/moment scaling, CG transfer, inertia,
quaternion conventions, and rigid-body integration together; it is not an
independent NASA trajectory comparison.

## Generated inspection tables

The package contains 45 normalized rows:

| Table | Rows |
|---|---:|
| Mach-dependent alpha limit | 7 |
| Clean aerodynamic samples | 28 |
| Control increments | 9 |
| Fixed mass properties | 1 |

The CSVs are inspection and interchange artifacts. Runtime evaluation continues
to use the complete DAVE-ML expression/table graph rather than those sampled
rows.

## Commands

```bash
tx-aircraft inspect-hl20 examples/daveml/hl20/HL20_aero.dml

tx-aircraft accept-hl20 \
  examples/daveml/hl20/HL20_aero.dml \
  --output build/hl20-acceptance.json

tx-aircraft export-hl20-tables \
  examples/daveml/hl20/HL20_aero.dml \
  --output-dir build/hl20-tables

tx-aircraft trim-hl20-glide \
  examples/daveml/hl20/HL20_aero.dml \
  --output build/hl20-glide-trim.json

tx-aircraft validate-hl20-glide \
  examples/daveml/hl20/HL20_aero.dml \
  --trajectory-csv build/hl20-glide-hold.csv \
  --output build/hl20-glide-hold.json

tx-aircraft build-hl20 \
  examples/daveml/hl20/HL20_aero.dml \
  build/hl20-mod-k-unpowered.txair

tx-aircraft verify-hl20 \
  build/hl20-mod-k-unpowered.txair \
  --output build/hl20-package-verification.json
```

## Exact completion boundary

The package is a complete **unpowered, fixed-mass, direct-surface rigid-body
baseline**. It intentionally does not claim:

- propulsion, reaction-control, or entry-energy management;
- actuator position/rate dynamics;
- SAS, guidance, or autopilot laws;
- fuel or payload-dependent mass properties;
- landing-gear contact or runway dynamics;
- flexible-body, thermal, damage, or aeroelastic effects;
- trajectory agreement with an independent NASA HL-20 simulation.

Those are additive fidelity layers. They should remain separately sourced and
versioned rather than being inferred from the aerodynamic database.
