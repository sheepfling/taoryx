# HL-20 official-source and vehicle acceptance procedure

## Gate order

1. Select the exact pinned `HL20_aero.dml` bytes.
2. Require Git blob `71e01b91e3a299a5b272c69f9a8296039aca59eb` and revision
   `Mod K, 2010-12-01`.
3. Parse with entity expansion and external DTD resolution disabled.
4. Require 361 variables, 8 breakpoints, 241 functions, 169 regular tables,
   6,247 table values, 24 static shots, and no parser issues.
5. Compile every supported regular table and MathML calculation.
6. Execute all 24 embedded shots with source units and tolerances.
7. Bind all 16 standardized inputs and 10 standardized outputs.
8. Run a no-internal-values regression path to ensure check execution does not
   depend on optional diagnostic dumps.
9. Evaluate the canonical SI/FRD reference vector and require no clamps.
10. Bind the separately pinned fixed mass, CG, and inertia record.
11. Generate alpha-limit, clean-aero, control-increment, and mass tables.
12. Solve the reference unpowered glide and require force/moment closure.
13. Propagate a five-second frozen-atmosphere six-DOF trim hold.
14. Build the `.txair`, verify every artifact hash, regenerate all evidence, and
   require byte-identical repeated builds.

## Required evidence

```text
models/aerodynamics.dml
runtime/aerodynamic-binding.json
runtime/vehicle-binding.json
validation/acceptance-spec.json
validation/acceptance.json
validation/check-report.json
validation/reference-evaluation.json
tables/alpha-limit.csv
tables/clean-aerodynamics.csv
tables/control-increments.csv
tables/mass-properties.csv
tables/summary.json
validation/reference-glide-trim.json
validation/glide-hold-summary.json
validation/glide-hold.csv
source/provenance.json
checksums.sha256
```

## Release-label rules

The aerodynamic subsystem may be called **accepted NASA HL-20 Mod K
aerodynamics** only when exact identity, structural inventory, runtime compile,
and all 24 source checks pass.

The complete package may be called **HL-20 Mod K unpowered fixed-mass 6-DOF
baseline** only when the separately sourced mass/inertia binding, glide trim,
trim-hold propagation, deterministic build, and independent package
regeneration also pass.

It must not be labeled a complete operational, entry-guidance, or landing model
without separately validated propulsion, controls, actuators, contact dynamics,
and trajectory evidence.
