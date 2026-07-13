# Documented TAOS language-surface coverage

This matrix checks whether complete positive integration inputs represent each documented Chapter 3 table type, Chapter 3 full-table operation, and Chapter 4 scoped data block.

- Scoped Chapter 4 blocks: **33/33**
- Chapter 3 table types: **19/19**
- Chapter 3 full-table operations: **28/28**

Representation coverage is not historical-runtime proof. Runtime tests remain gated by `TAOS_EXE`.

## Scoped Chapter 4 blocks

| Item | Covered | Cases |
|---|:---:|---|
| `segment:aero` | yes | p007_reentry_simple_table, p008_reentry_full_table_equivalent, p011_skewed_aero_table, p025_level_mach_guidance, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p044_split_aero_aggregation, p045_single_aero_equivalent |
| `segment:constants` | yes | p036_constants_cg_user_variables |
| `segment:cg` | yes | p034_manual_air_intercept_synthetic, p036_constants_cg_user_variables |
| `segment:fly` | yes | p002_constant_thrust_table, p003_variable_mass_rocket, p009_piecewise_full_thrust, p011_skewed_aero_table, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field |
| `segment:increment` | yes | p016_rail_stage_branch, p033_manual_ballistic_rocket_synthetic |
| `segment:inertial` | yes | p002_constant_thrust_table, p003_variable_mass_rocket, p009_piecewise_full_thrust, p026_predictive_intercept, p027_proportional_navigation, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p039_table_driven_thrust_vector, p046_case_insensitive_free_field |
| `segment:integ` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `segment:limits` | yes | p035_manual_ground_intercept_synthetic |
| `segment:prop` | yes | p002_constant_thrust_table, p003_variable_mass_rocket, p009_piecewise_full_thrust, p016_rail_stage_branch, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p039_table_driven_thrust_vector, p046_case_insensitive_free_field, p049_large_table_many_segments_stress |
| `segment:rail` | yes | p016_rail_stage_branch, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic |
| `segment:reset` | yes | p016_rail_stage_branch, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic |
| `segment:when` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `trajectory:define` | yes | p010_output_define_full_table, p034_manual_air_intercept_synthetic, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs |
| `trajectory:dwn/crs` | yes | p028_tangent_iip_downrange, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic |
| `trajectory:file` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `trajectory:iip` | yes | p028_tangent_iip_downrange |
| `trajectory:initial` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `trajectory:print` | yes | p001_linear_ecfc_zero_force, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p048_problem_level_outputs_and_egs |
| `trajectory:tangent` | yes | p028_tangent_iip_downrange |
| `problem:atmos` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `problem:define` | yes | p048_problem_level_outputs_and_egs |
| `problem:earth` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `problem:egs` | yes | p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p048_problem_level_outputs_and_egs |
| `problem:file` | yes | p017_radar_relative, p048_problem_level_outputs_and_egs |
| `problem:optimize` | yes | p022_optimize_linear_boundary, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic |
| `problem:print` | yes | p048_problem_level_outputs_and_egs |
| `problem:radar` | yes | p017_radar_relative |
| `problem:search` | yes | p021_search_linear_target |
| `problem:summarize` | yes | p020_survey_summarize, p032_manual_ballistic_reentry |
| `problem:survey` | yes | p020_survey_summarize, p032_manual_ballistic_reentry |
| `problem:title` | yes | p001_linear_ecfc_zero_force, p002_constant_thrust_table, p003_variable_mass_rocket, p004_vacuum_ballistic_coarse, p005_vacuum_ballistic_medium, p006_vacuum_ballistic_fine, p007_reentry_simple_table, p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p012_user_atmosphere, p013_site_atmosphere, p014_zero_wind, p015_crosswind, p016_rail_stage_branch, p017_radar_relative, p018_geodetic_initialization, p019_ecfc_initialization, p020_survey_summarize, p021_search_linear_target, p022_optimize_linear_boundary, p023_units_format, p024_guidance_table, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p028_tangent_iip_downrange, p029_split_thrust_aggregation, p030_single_thrust_equivalent, p031_multi_problem_document, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables, p037_wind_axis_aero_coefficients, p038_body_axis_aero_coefficients, p039_table_driven_thrust_vector, p040_wind_speed_heading_tables, p041_wind_component_tables, p042_ecic_initialization, p043_wgs84_gravity_drop, p044_split_aero_aggregation, p045_single_aero_equivalent, p046_case_insensitive_free_field, p047_full_table_operation_gauntlet, p048_problem_level_outputs_and_egs, p049_large_table_many_segments_stress |
| `problem:units/fmt` | yes | p023_units_format |
| `problem:wind` | yes | p015_crosswind, p040_wind_speed_heading_tables, p041_wind_component_tables |

## Chapter 3 table types

| Item | Covered | Cases |
|---|:---:|---|
| `ca` | yes | p007_reentry_simple_table, p008_reentry_full_table_equivalent, p011_skewed_aero_table, p032_manual_ballistic_reentry, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic, p036_constants_cg_user_variables |
| `cn` | yes | p034_manual_air_intercept_synthetic, p035_manual_ground_intercept_synthetic |
| `cl` | yes | p025_level_mach_guidance, p037_wind_axis_aero_coefficients |
| `cd` | yes | p025_level_mach_guidance, p037_wind_axis_aero_coefficients |
| `cs` | yes | p037_wind_axis_aero_coefficients |
| `cx` | yes | p038_body_axis_aero_coefficients |
| `cy` | yes | p038_body_axis_aero_coefficients |
| `cz` | yes | p038_body_axis_aero_coefficients |
| `thrust` | yes | p002_constant_thrust_table, p003_variable_mass_rocket, p009_piecewise_full_thrust, p016_rail_stage_branch, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p039_table_driven_thrust_vector, p046_case_insensitive_free_field, p049_large_table_many_segments_stress |
| `tvec1` | yes | p039_table_driven_thrust_vector |
| `tvec2` | yes | p039_table_driven_thrust_vector |
| `mdot` | yes | p002_constant_thrust_table, p003_variable_mass_rocket, p016_rail_stage_branch, p025_level_mach_guidance, p026_predictive_intercept, p027_proportional_navigation, p033_manual_ballistic_rocket_synthetic, p034_manual_air_intercept_synthetic, p039_table_driven_thrust_vector |
| `cg` | yes | p036_constants_cg_user_variables |
| `windv` | yes | p015_crosswind, p040_wind_speed_heading_tables |
| `windh` | yes | p040_wind_speed_heading_tables |
| `winde` | yes | p041_wind_component_tables |
| `windn` | yes | p041_wind_component_tables |
| `windd` | yes | p040_wind_speed_heading_tables, p041_wind_component_tables |
| `output` | yes | p010_output_define_full_table, p047_full_table_operation_gauntlet |

## Chapter 3 full-table operations

| Item | Covered | Cases |
|---|:---:|---|
| `add` | yes | p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p047_full_table_operation_gauntlet |
| `sub` | yes | p047_full_table_operation_gauntlet |
| `mult` | yes | p010_output_define_full_table, p047_full_table_operation_gauntlet |
| `div` | yes | p047_full_table_operation_gauntlet |
| `idiv` | yes | p047_full_table_operation_gauntlet |
| `exp` | yes | p047_full_table_operation_gauntlet |
| `iexp` | yes | p047_full_table_operation_gauntlet |
| `max` | yes | p047_full_table_operation_gauntlet |
| `min` | yes | p047_full_table_operation_gauntlet |
| `set` | yes | p047_full_table_operation_gauntlet |
| `abs` | yes | p047_full_table_operation_gauntlet |
| `neg` | yes | p047_full_table_operation_gauntlet |
| `sqr` | yes | p010_output_define_full_table, p047_full_table_operation_gauntlet |
| `sqrt` | yes | p010_output_define_full_table, p047_full_table_operation_gauntlet |
| `ln` | yes | p047_full_table_operation_gauntlet |
| `log` | yes | p047_full_table_operation_gauntlet |
| `e` | yes | p047_full_table_operation_gauntlet |
| `sin` | yes | p047_full_table_operation_gauntlet |
| `cos` | yes | p047_full_table_operation_gauntlet |
| `tan` | yes | p047_full_table_operation_gauntlet |
| `asin` | yes | p047_full_table_operation_gauntlet |
| `acos` | yes | p047_full_table_operation_gauntlet |
| `atan` | yes | p047_full_table_operation_gauntlet |
| `zero` | yes | p047_full_table_operation_gauntlet |
| `csto` | yes | p010_output_define_full_table, p047_full_table_operation_gauntlet |
| `if` | yes | p009_piecewise_full_thrust, p047_full_table_operation_gauntlet |
| `goto` | yes | p009_piecewise_full_thrust, p047_full_table_operation_gauntlet |
| `end` | yes | p008_reentry_full_table_equivalent, p009_piecewise_full_thrust, p010_output_define_full_table, p011_skewed_aero_table, p047_full_table_operation_gauntlet |
