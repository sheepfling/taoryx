# HL-20 California-to-Hawaii comparison contract

This directory contains a commensurate route contract and a runnable
four-fidelity release witness for the source-grounded HL-20 Mod K unpowered
model. It is not a target-hit mission.

The contract uses the same synthetic California and Hawaii endpoints as the
existing route showcase, but compares only normalized entry and glide evidence:
phase timestamps, altitude/Mach, energy, range, crossrange, and termination
disposition. The HL-20 aerodynamic and fixed-mass bindings remain those of
`reference_hl20_mod_k`; the route, initial condition, and any arrival handling
will be separate Taoryx assumptions.

No controller, guidance, thermal-protection, landing, or historical trajectory
claim is made. The next implementation gate is an open-loop entry/glide runner
that fails closed outside the HL-20 source envelope and emits the shared
comparison channels.

Run the four-fidelity tranche from the repository root:

```text
python -m tools.dev showcase-hl20-composites
```

Render the source-coupled variant, which uses the pinned DAVE-ML graph and
records actuator and energy-management telemetry:

```text
python -m tools.dev showcase-hl20-source-composites
```

Qualify the generated bundle through the explicit HL-20 gates with:

```text
python -m tools.dev qualify-hl20
```

The machine-readable contract is `quality_gates.yaml`. The command writes
`verification/hl20_ca_hi_qualification.json` and reports HL20-G0 through
HL20-G6. Nominal parent and child flights must terminate by ground contact;
horizon timeouts remain classified as unsuccessful rather than being counted
as arrival capability.

The output is a deterministic `point_mass_3dof.json`, `pseudo_6dof.json`,
`rigid_body_6dof.json`, `rigid_body_6dof_surface_allocated.json`, and
`bundle-manifest.json` under
`artifacts/showcases/hl20_california_to_hawaii/low_fidelity`.

The same directory contains a `plots/` bundle with flown trajectories, search
coverage, terminal capability, and fidelity progression views plus its plot
manifest.

`composites/` contains the reviewer-facing four-tier evidence boards:

- `hl20-ca-hi-flight-composite.png` shows every tier's phase-colored parent
  path, boost/coast/glide timeline, available attitude or load/control channel,
  mass schedule, stage objectives, and terminal disposition;
- `hl20-ca-hi-envelope-comparison.png` shows the exact realized search grid,
  terminal capability, maximum altitude, terminal speed, objective status, and
  tier-to-tier terminal movement;
- `hl20-ca-hi-spent-booster-composite.png` follows the synthetic passive
  cylinder after release, including parent/child geometry, projected area,
  drag, rotation representation, and its terminal disposition.

`showcase-summary.json` records phase objectives, classifications, timeouts,
search space, candidate deltas, and the explicit claim boundary. The four CSV
files retain the per-tier candidate, parent telemetry, spent-cylinder
telemetry, and logical surface-allocation rows used by the boards. The adjacent
`showcase-manifest.json` hashes every input envelope, data artifact, and image.

The default surface-allocated file is a bounded synthetic logical seven-surface
overlay. The `source_bound` packet replaces that aerodynamic path with the
pinned DAVE-ML force/moment graph and first-order actuator realization, while
remaining explicitly open-loop and non-controller-qualified.
