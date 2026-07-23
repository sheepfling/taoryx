# Adding a vehicle to TAORYX

New vehicles enter through the existing catalogs. Do not create a bespoke
runner or copy a large `.prb` file with hard-coded physical constants. The
prescribed path is:

1. Preserve the source bundle under `tests/fixtures/` and record its source
   status and model path.
2. Add one SI vehicle entry to `verification/vehicle_models.yaml`, including
   `nominal_mass_kg` separately from `dry_mass_kg`. The nominal mass is the
   source-backed operating point used for controller scaling; it is not an
   inferred replacement for a mass schedule.
3. Select an existing family in `verification/vehicle_families.yaml`, or add
   a new family when the frame, dynamics, or control semantics are genuinely
   different.
4. Bind every verified `.tbl` file in `table_bindings`.
5. Add the same vehicle and source tables to `verification/vehicle_catalog.yaml`
   with one standard golden `.prb` path.
6. Add a metadata profile and scenario to
   `verification/problem_generation.yaml`, then generate the `.prb` products.
7. Add a bounded trim entry in `verification/trim_specs.yaml` and a linked
   controller design in `verification/controller_designs.yaml`.
8. Add signed control-direction probes in
   `verification/control_direction_contracts.yaml`.
9. Add the three LQR profile styles in
   `verification/lqr_scaling_profiles.yaml`.

The controller scale contract is derived automatically from the registry and
bound tables. It reports nominal mass, principal inertia, weight force,
weight moment, inertia/rate response moment, and actuator limits separately.
Do not copy gains or use one vehicle's moment scale for another vehicle.

## Diagnose before running a trajectory

Run the focused onboarding report first:

```bash
python tools/validate_vehicle_onboarding.py --vehicle new_vehicle
```

It reports the exact catalog path, diagnostic code, missing field/file, and a
repair hint. Validate the complete registry with:

```bash
python tools/dev.py onboard-vehicles
```

Warnings are explicit and non-blocking in the normal development workflow.
Use `--strict` when a release requires every source-only or provisional item
to be resolved:

```bash
python tools/validate_vehicle_onboarding.py --vehicle new_vehicle --strict --json artifacts/new_vehicle_onboarding.json
```

The validator checks metadata completeness, family mode/frame agreement, SI
geometry and inertia, table existence and parser ingestion, source provenance,
golden catalog discovery, generated-problem coverage, trim/controller hooks,
control-direction probes, and all three controller profiles. It does not
pretend that those hooks prove a trim or mission; the standard plant harness
still owns those claims.

## Advance through the common harness

Once onboarding is ready, use the shared sequence:

```bash
python tools/dev.py generate-problems
python tools/dev.py vehicles
python tools/dev.py control-directions
python tools/auto_tune_controllers.py --vehicle new_vehicle
python -m pytest -m '<family-marker>'
```

The final command is intentionally a family-specific pytest selection rather
than a hidden new runner. Add the family marker to `pyproject.toml` and use it
on the new tests. New vehicle tests should use the shared golden-plant
harness in `tests/e2e/support/golden_plants.py`, with the vehicle catalog
providing the `.prb`, `.tbl`, step policy, and source anchor.

## What each failure means

| Diagnostic | Meaning | Repair |
| --- | --- | --- |
| `unknown-family` | The vehicle semantics are not declared | Add/select a family before adding runtime branches |
| `missing-table-file` | Registry points at no verified table | Correct the fixture path or preserve the missing source |
| `table-ingestion-failed` | `.tbl` cannot enter the canonical parser | Fix syntax/axes and run table fixture tests |
| `missing-public-catalog-entry` | The common harness cannot discover the vehicle | Add one golden catalog entry |
| `missing-problem-generation-profile` | A problem is being hand-maintained | Add a metadata profile/template scenario |
| `missing-trim-spec` | Autotuning has no declared operating point | Add bounded state/control variables and residual names |
| `trim-source-only` | The trim exists but has no ready residual adapter | Implement the source-backed plant residual before promoting it |
| `missing-control-direction-contract` | Sign conventions are untested | Add signed probes and expected responses |
| `missing-lqr-profile` | Profile selection would fall through or cross vehicles | Add gentle/standard/aggressive variants for this vehicle |

The first success milestone is `ready` from the onboarding report. The next
milestones are `plant-golden`, controller-ready, and mission evidence from the
existing twelve-stage harness. A vehicle should not skip directly from
metadata registration to a waypoint claim.
