# Taoryx extension grammar corpus

This focused corpus exercises Taoryx extension syntax, not historical TAOS
96.0 syntax. The fixtures are
deliberately small so parser and AST changes can be diagnosed by extension
family before the forms are promoted into larger examples.

Parse problem files with `profile=taoryx`. The table fixture exercises the
successor `aero_force`, `aero_moment`, and `inertia` table families.

| Fixture | Coverage |
| --- | --- |
| `01_dynamics_method.prb` | `*3dof`, `*sixdof`, and `*method` aliases |
| `02_mass_transition.prb` | `*mass`, `*ptmass`, and variable-mass method options |
| `03_aero_channels.prb` | reference lengths and derivative channels |
| `04_aero_tables.tbl` | force, moment, and inertia table families |
| `05_propulsion_rail.prb` | vacuum propulsion, rail angles, and `q_rail` |
| `06_deployment.prb` | deployed trajectory initialization |
| `07_surface_helpers.prb` | declarations and surface helper calls |
| `08_batch_cases.prb` | cases, symbolic optimization, and summaries |
| `09_environment.prb` | RCC atmosphere and file-backed wind |
| `10_parser_quirks.prb` | spacing, case, names, and non-1 start segments |

These are Taoryx grammar/AST fixtures. They do not claim complete TAOS 96 runtime
compatibility.
