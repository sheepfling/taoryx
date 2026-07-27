# HL-20 California-to-Hawaii comparison contract

This directory contains a commensurate route contract and the first runnable
low-fidelity release witness for the source-grounded HL-20 Mod K unpowered
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

Run the three-fidelity first tranche from the repository root:

```text
python tools/dev.py showcase-hl20-low-fidelity
```

The output is a deterministic `point_mass_3dof.json`, `pseudo_6dof.json`,
`rigid_body_6dof.json`, and `bundle-manifest.json` under
`artifacts/showcases/hl20_california_to_hawaii/low_fidelity`.
