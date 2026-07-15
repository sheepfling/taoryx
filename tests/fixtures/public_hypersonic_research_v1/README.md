# Public hypersonic research fixture

This fixture preserves the supplied public research dataset under `source/`
and stores generated TAORYX `.tbl` translations under `tables/`.

Included native tables:

- `langley_winged_cone_force.tbl`: 88-point Mach/alpha longitudinal `CX/CY/CZ`
  deck, with `CY=0` because the source has no sideslip data;
- `orion38_thrust_mdot.tbl`, `orion50xl_thrust_mdot.tbl`, and
  `orion50sxl_thrust_mdot.tbl`: time histories with SI thrust and mass-flow
  outputs.

The Langley deck is source-valid for Mach 6–25 and alpha -5°–20°. It is not a
complete 6-DOF aerodynamic model: it has no beta, control effectiveness, or
roll/yaw dynamic derivatives. The X-33 data is retained as source evidence but
is not selected for CA–HI because its sparse high-alpha envelope is a poor
match for that mission.

Regenerate the translations with:

```bash
python tools/import_public_hypersonic_tables.py
```
