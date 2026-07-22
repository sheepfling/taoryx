# Problem-file generation contract

TAORYX has two kinds of problem files:

1. **Source-preserved fixtures** — historical/manual examples and research
   inputs kept as received. These are not regenerated.
2. **Scenario products** — native `.prb` files assembled from tracked vehicle
   profiles, scenario metadata, and reusable templates. These must be
   reproducible.

The metadata source of truth is:

- `verification/vehicle_models.yaml` for vehicle identity, mass, inertia,
  reference geometry, frame conventions, envelopes, controls, and table
  bindings;
- `verification/vehicle_families.yaml` for family-level behavior: dynamics
  mode, frame contract, required fields, control roles, and problem template;
- `verification/problem_generation.yaml` for scenario identity, vehicle
  profile selection, initial conditions, controller settings, and phase
  parameters;
- `tools/problem_templates/` for reusable problem-file structure;
- the vehicle table catalogs for table provenance and units.
- `taoryx.vehicle_registry` for programmatic access to the same physical
  contract when a solver or analysis tool must emit temporary problem syntax.

Use:

```text
python tools/dev.py generate-problems
python tools/dev.py check-problems
python tools/dev.py audit-vehicles
# one-command validation of the complete vehicle workflow
python tools/dev.py vehicles
```

`check-problems` is part of the normal development check and fails if a
generated `.prb` differs from the metadata/template result.
`audit-vehicles` checks every scoped B747, X8, Hummingbird, and X-15 problem
header against `vehicle_models.yaml`, verifies the source bundle exists, and
checks that the public vehicle catalog binds the same source tables. It is
also part of `python tools/dev.py check`.
`vehicles` runs the registry, provenance, and generated-file checks together
and is the recommended command after changing a vehicle or scenario profile.

## Current migration boundary

The first migrated tranche is the four B747 diagnostics plus one canonical
generated plant problem for each standard vehicle family:

- open-loop descending response;
- controlled flight-path descent;
- integrated route and altitude transition;
- notional fuel/mass coupling.

The canonical generated plant outputs cover B747, Skywalker X8, Hummingbird,
and X-15. Their existing specialized scenario suites are still being migrated
incrementally; they must reference the same vehicle registry rather than copy
physical constants into each problem file.

## Adding a new vehicle family

1. Add a family contract to `vehicle_families.yaml`.
2. Add the family-required physical and convention fields to a vehicle entry
   in `vehicle_models.yaml`.
3. Add or select the family problem template.
4. Add table bindings and a canonical generated plant scenario.
5. Add the vehicle to `vehicle_catalog.yaml`.
6. Run `python tools/dev.py check-vehicles` and
   `python tools/dev.py check-problems` before adding controller maneuvers.

Controller gains belong to scenario metadata, not the vehicle description.
That keeps a B747, X8, or new vehicle’s physical contract stable while each
scenario declares its own controller tuning and acceptance corridor.

The remaining B747, X8, Hummingbird, and X-15 scenario files are currently
classified as source-preserved or hand-authored scenario products. Their
vehicle physical headers are still subject to the provenance audit, but their
scenario-specific initial conditions, controls, and route values remain
scenario data. They should be migrated family-by-family after their
acceptance contracts are stable. A file must not be labeled generated until
its metadata, template, table bindings, and deterministic regeneration check
are present.

Tests may continue to refer to stable generated output paths, but scenario
parameters and vehicle constants should move into the metadata catalog during
each migration. Temporary test variants may still be created in `tmp_path`;
they are diagnostic derivatives, not canonical problem files.
