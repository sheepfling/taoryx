# Data dictionary

| File | Purpose |
|---|---|
| `aero/langley_winged_cone_hypersonic_static.csv` | M>=6 CL, CD, Cm, CN, CX, CZ table |
| `aero/langley_winged_cone_hypersonic_components.csv` | Coefficient buildup |
| `aero/x33_longitudinal_cfd_summary.csv` | 21 direct X-33 CFD cases |
| `aero/x33_mach6_component_cfd.csv` | Body, canted-fin, and bodyflap contributions |
| `aero/x43a_flight_test_envelope.csv` | Maneuver-sequence starting conditions |
| `aero/uncertainty_models.csv` | Source uncertainty multipliers and observations |
| `mass/x43a_hxlv_nominal_mass_properties.csv` | Separation mass, CG, and inertias |
| `mass/x43a_hxlv_mass_uncertainty.csv` | Source ranges and frame warnings |
| `mass/pegasus_motor_summary_catalog.csv` | Standalone Orion motor summaries |
| `propulsion/winged_cone_scramjet_map.csv` | Mach/q/eta conceptual thrust map |
| `propulsion/orion*_thrust_curve_surrogate.csv` | Thrust, impulse, and mass depletion histories |
| `checks/validation_report.json` | Numerical consistency checks |

Column names carry units where practical. `sources.csv` maps every source ID to its publication and URL.
