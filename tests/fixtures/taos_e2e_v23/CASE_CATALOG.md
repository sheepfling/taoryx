# TAOS end-to-end case catalog

All paths are relative to the suite root. Runtime tiers describe intent, not completed execution.

## Positive cases

| ID | Tier | Title | Tables | Key contracts |
|---|---|---|---:|---|
| `p001_linear_ecfc_zero_force` | `analytic` | Analytic ECFC constant-velocity propagation | 0 | ecfc_initial_conditions, zero_gravity, zero_rotation, trajectory_file_output, fixed_step_integration |
| `p002_constant_thrust_table` | `analytic` | Analytic constant thrust with fixed mass | 2 | simple_tables, thrust_table, mass_flow_table, inertial_platform_ecfc, direct_euler_guidance |
| `p003_variable_mass_rocket` | `invariant` | Constant thrust with linearly decreasing mass | 2 | variable_mass, mass_flow_units, fuel_output, thrust_output |
| `p004_vacuum_ballistic_coarse` | `metamorphic` | Vacuum spherical-earth ballistic trajectory dt=2.0 | 0 | spherical_gravity, vacuum_atmosphere, runge_kutta, geodetic_initial_conditions |
| `p005_vacuum_ballistic_medium` | `metamorphic` | Vacuum spherical-earth ballistic trajectory dt=1.0 | 0 | spherical_gravity, vacuum_atmosphere, runge_kutta, geodetic_initial_conditions |
| `p006_vacuum_ballistic_fine` | `metamorphic` | Vacuum spherical-earth ballistic trajectory dt=0.25 | 0 | spherical_gravity, vacuum_atmosphere, runge_kutta, geodetic_initial_conditions |
| `p007_reentry_simple_table` | `metamorphic` | Ballistic reentry with simple aerodynamic table | 1 | standard_atmosphere, wgs84, simple_aero_table, impact_final_condition |
| `p008_reentry_full_table_equivalent` | `metamorphic` | Ballistic reentry with equivalent full aerodynamic table | 1 | full_table, tabulated_operation, simple_full_equivalence |
| `p009_piecewise_full_thrust` | `analytic` | Full-table if/goto piecewise thrust profile | 1 | full_table, if_then, goto, labels, piecewise_forcing |
| `p010_output_define_full_table` | `invariant` | User-defined output variable backed by full output table | 1 | output_table, trajectory_define, table_function, storage_variable, math_operations |
| `p011_skewed_aero_table` | `smoke` | Three-dimensional skewed aerodynamic table | 1 | skewed_table, three_dimensional_interpolation, direct_angle_guidance |
| `p012_user_atmosphere` | `analytic` | Complete user-defined atmosphere at a tabulated altitude | 0 | user_atmosphere, atmosphere_columns, exact_grid_point |
| `p013_site_atmosphere` | `analytic` | Site-measured pressure/density atmosphere | 0 | site_atmosphere, exponential_interpolation, derived_temperature |
| `p014_zero_wind` | `metamorphic` | Zero-wind airspeed baseline | 0 | zero_wind, airspeed_output |
| `p015_crosswind` | `metamorphic` | Altitude-table crosswind airspeed | 1 | wind_table, geodetic_wind, crosswind |
| `p016_rail_stage_branch` | `smoke` | Rail launch, staging, reset, increment, and branch trajectory | 2 | rail_launch, multiple_segments, reset, increment, trajectory_inheritance… |
| `p017_radar_relative` | `analytic` | Analytic two-vehicle relative and radar observations | 0 | multiple_vehicles, relative_outputs, radar_station, problem_level_file |
| `p018_geodetic_initialization` | `metamorphic` | Geodetic initialization representation | 0 | geodetic_initial_conditions, coordinate_transform |
| `p019_ecfc_initialization` | `metamorphic` | Equivalent ECFC initialization representation | 0 | ecfc_initial_conditions, coordinate_transform |
| `p020_survey_summarize` | `smoke` | Survey values with summary reduction | 0 | survey, summarize, parameter_placeholder, multiple_runs |
| `p021_search_linear_target` | `analytic` | One-dimensional search with analytic linear target | 0 | search, search_parameter, root_finding |
| `p022_optimize_linear_boundary` | `analytic` | Optimization of a linear objective to a parameter bound | 0 | optimization, optimization_parameter, bound_solution |
| `p023_units_format` | `analytic` | Metric unit conversion and output formatting | 0 | units_fmt, metric_conversion, fixed_format, scientific_format |
| `p024_guidance_table` | `invariant` | Time-based guidance table with circular interpolation | 0 | guidance_table, interp_2, integration_step_alignment, direct_attitude |
| `p025_level_mach_guidance` | `invariant` | Indirect level-flight and constant-Mach guidance | 4 | indirect_guidance, constant_mach, level_flight, free_alpha, free_power |
| `p026_predictive_intercept` | `metamorphic` | Two-vehicle intercept guidance | 2 | multi_vehicle, intercept, thrust_vectoring_via_body_attitude, relative_range |
| `p027_proportional_navigation` | `metamorphic` | Two-vehicle propnav guidance | 2 | multi_vehicle, propnav, thrust_vectoring_via_body_attitude, relative_range |
| `p028_tangent_iip_downrange` | `smoke` | Tangent plane, downrange/crossrange, and IIP outputs | 0 | tangent_plane, downrange_crossrange, initial_impact_point, iip_outputs |
| `p029_split_thrust_aggregation` | `metamorphic` | Two prop blocks whose forces sum | 0 | multiple_prop_blocks, force_aggregation |
| `p030_single_thrust_equivalent` | `metamorphic` | Single prop block equivalent to summed blocks | 0 | single_prop_block, force_aggregation_equivalence |
| `p031_multi_problem_document` | `smoke` | Two independent problems in one problem file | 0 | multiple_problems_per_file, sequential_execution |
| `p032_manual_ballistic_reentry` | `historical` | Manual Section 4.5.1 ballistic reentry | 1 | manual_fixture, survey, summarize, egs, reentry |
| `p033_manual_ballistic_rocket_synthetic` | `exploratory` | Manual ballistic-rocket.prb with synthetic stand-in tables | 13 | manual_fixture, synthetic_dependency_closure, full_application_smoke |
| `p034_manual_air_intercept_synthetic` | `exploratory` | Manual air-launched-intercept.prb with synthetic stand-in tables | 19 | manual_fixture, synthetic_dependency_closure, full_application_smoke |
| `p035_manual_ground_intercept_synthetic` | `exploratory` | Manual ground-launched-intercept.prb with synthetic stand-in tables | 4 | manual_fixture, synthetic_dependency_closure, full_application_smoke |
| `p036_constants_cg_user_variables` | `invariant` | User-defined constants, center-of-gravity table, and cg-dependent aerodynamics | 2 | constants_block, cg_table, user_defined_table_variable, cg_dependent_aerodynamics |
| `p037_wind_axis_aero_coefficients` | `invariant` | Complete lift, drag, and side-force coefficient set | 3 | cl_table, cd_table, cs_table, wind_axis_aerodynamics, complete_aero_set |
| `p038_body_axis_aero_coefficients` | `invariant` | Complete body-axis aerodynamic coefficient set | 3 | cx_table, cy_table, cz_table, body_axis_aerodynamics, complete_aero_set |
| `p039_table_driven_thrust_vector` | `invariant` | Table-driven thrust magnitude, mass flow, and both thrust-vector angles | 4 | tvec1_table, tvec2_table, thrust_vectoring, propulsion_table_set |
| `p040_wind_speed_heading_tables` | `metamorphic` | Wind speed, heading, and down-component tables | 3 | windv_table, windh_table, windd_table, wind_speed_heading_representation |
| `p041_wind_component_tables` | `metamorphic` | Equivalent east, north, and down wind-component tables | 3 | winde_table, windn_table, windd_table, wind_component_representation |
| `p042_ecic_initialization` | `metamorphic` | ECIC initialization equivalent to geodetic and ECFC at zero earth rotation | 0 | ecic_initial_conditions, coordinate_initialization_equivalence |
| `p043_wgs84_gravity_drop` | `invariant` | WGS-84 gravity-only vertical drop | 0 | wgs84_earth, gravity_model, ballistic_drop |
| `p044_split_aero_aggregation` | `metamorphic` | Two aerodynamic blocks aggregate to the total force | 0 | multiple_aero_blocks, aero_force_aggregation |
| `p045_single_aero_equivalent` | `metamorphic` | Single aerodynamic block equivalent to the summed split blocks | 0 | single_aero_block, aero_force_aggregation |
| `p046_case_insensitive_free_field` | `smoke` | Case-insensitive keywords, comments, commas, and scientific notation | 1 | case_insensitivity, comments, scientific_notation, free_field_delimiters |
| `p047_full_table_operation_gauntlet` | `exploratory` | Full-table accumulator, storage, flow-control, logarithmic, power, and trigonometric operations | 1 | all_table_math_operations, full_table_control_flow, output_table, storage_variable |
| `p048_problem_level_outputs_and_egs` | `smoke` | Problem-level define, column output, and EGS output across two trajectories | 0 | problem_define, problem_file_output, egs_output, multi_vehicle_output |
| `p049_large_table_many_segments_stress` | `stress` | Large table and twenty-segment execution stress case | 1 | large_table, many_segments, stress, segment_transition_chain |

## Negative cases

| ID | Tier | Title | Tables | Key contracts |
|---|---|---|---:|---|
| `n001_missing_end` | `none` | Missing *end | 0 | missing_end |
| `n002_bad_goto` | `none` | Goto undefined segment | 0 | bad_goto |
| `n003_duplicate_trajectory` | `none` | Duplicate trajectory number | 0 | duplicate_trajectory |
| `n004_duplicate_segment` | `none` | Duplicate segment number | 0 | duplicate_segment |
| `n005_missing_table_reference` | `none` | Unresolved table reference | 0 | missing_table |
| `n006_table_cardinality` | `none` | Simple table cardinality mismatch | 1 | table_cardinality |
| `n007_table_nonmonotonic` | `none` | Nonmonotonic independent variable | 1 | table_nonmonotonic |
| `n008_duplicate_table` | `none` | Duplicate table name | 2 | duplicate_table |
| `n009_inconsistent_fly_set` | `none` | Inconsistent body-attitude angle set | 0 | inconsistent_fly_angle_set |
| `n010_initial_mass_conflict` | `none` | Both mass and weight supplied | 0 | conflicting_initial_mass |
| `n011_invalid_units_dimension` | `none` | Dimensionally invalid unit conversion | 0 | incompatible_unit_dimension |
| `n012_user_atmos_duplicate_alt` | `none` | Duplicate user-atmosphere altitude | 0 | duplicate_atmos_altitude |
| `n013_first_segment_inertial_body` | `none` | Body-aligned inertial platform on first segment | 0 | inertial_body_first_segment |
| `n014_optimize_bad_parameter` | `none` | Optimization parameter index zero | 0 | nonsequential_optimize_parameters |

## Metamorphic groups

| ID | Cases | Comparison | Columns |
|---|---|---|---|
| `m001_dt_convergence` | `p004_vacuum_ballistic_coarse`, `p005_vacuum_ballistic_medium`, `p006_vacuum_ballistic_fine` | `convergence` | `alt` |
| `m002_simple_full_table_equivalence` | `p007_reentry_simple_table`, `p008_reentry_full_table_equivalent` | `allclose` | `time`, `alt`, `vel`, `mach`, `dynprs`, `ca` |
| `m003_wind_effect` | `p014_zero_wind`, `p015_crosswind` | `crosswind_increases_airspeed` | `vair`, `vel` |
| `m004_coordinate_initialization_equivalence` | `p018_geodetic_initialization`, `p019_ecfc_initialization` | `allclose` | `time`, `xecfc`, `yecfc`, `zecfc`, `xecfcdt`, `yecfcdt`, `zecfcdt` |
| `m005_prop_aggregation_equivalence` | `p029_split_thrust_aggregation`, `p030_single_thrust_equivalent` | `allclose` | `time`, `xecfc`, `xecfcdt`, `thrust` |
| `m006_guidance_intercept_methods` | `p026_predictive_intercept`, `p027_proportional_navigation` | `both_reduce_relative_range` | `relrng[2]` |
| `m007_wind_representation_equivalence` | `p040_wind_speed_heading_tables`, `p041_wind_component_tables` | `allclose` | `time`, `vel`, `vair`, `mach` |
| `m008_ecic_coordinate_initialization_equivalence` | `p018_geodetic_initialization`, `p019_ecfc_initialization`, `p042_ecic_initialization` | `allclose` | `time`, `xecfc`, `yecfc`, `zecfc`, `xecfcdt`, `yecfcdt`, `zecfcdt` |
| `m009_aerodynamic_aggregation_equivalence` | `p044_split_aero_aggregation`, `p045_single_aero_equivalent` | `allclose` | `time`, `alt`, `vel`, `xecfc`, `yecfc`, `zecfc` |
