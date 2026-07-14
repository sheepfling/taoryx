# Algorithm Implementation Bindings

This page records executable bindings from the algorithm catalog. A binding
means the named `taoryx` symbol exists and has focused tests; it does not imply
that every downstream algorithm depending on it is complete.

| Catalog ID | Manual equations | Executable binding | Verification |
| --- | --- | --- | --- |
| `TAOS-ALG-COORD-020` | 2-93 through 2-96 | `taoryx.linalg.transform_vector` | `tests/unit/test_contracts.py` |
| `TAOS-ALG-OPT-004` | 4-7 and 4-8 | `taoryx.numeric.finite_difference_jacobian` | `tests/unit/test_numeric.py` |
| `TAOS-ALG-SEARCH-001` | 2-298 and 2-299 | `taoryx.searches.newton_root` | `tests/unit/test_searches.py` |
| `TAOS-ALG-SEARCH-002` | 2-300 | `taoryx.searches.secant_bracketed_root` | `tests/unit/test_searches.py` |
| `TAOS-ALG-SEARCH-004` | 2-306 and 2-307 | `taoryx.searches.golden_section_minimize` | `tests/unit/test_searches.py` |
| `TAOS-ALG-COORD-001` | 2-1 through 2-5 | `taoryx.coordinates.ecic_coords`; derived inverse `ecfc_coords` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-002` | 2-6 | `taoryx.coordinates.geocentric_unit_vectors` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-003` | 2-7 | `taoryx.coordinates.geocentric_position_to_ecfc` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-004` | 2-8 and 2-9 | derived `taoryx.coordinates.ecfc_position_to_geocentric` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-005` | 2-10 through 2-14 | `taoryx.coordinates.ecfc_velocity_to_geocentric` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-006` | 2-15 through 2-17 | derived `taoryx.coordinates.geocentric_velocity_to_ecfc` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-007` | 2-18 | `taoryx.earth.resolve_ellipsoid_parameters` | `tests/unit/test_earth.py` |
| `TAOS-ALG-COORD-008` | 2-19 through 2-21 | `taoryx.coordinates.geodetic_unit_vectors` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-009` | 2-22 through 2-29 | `taoryx.earth.ellipsoidal_surface_geometry` | `tests/unit/test_earth.py` |
| `TAOS-ALG-COORD-010` | 2-30 through 2-33 | `taoryx.coordinates.geodetic_position_to_ecfc` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-011` | 2-34 through 2-41 | derived `taoryx.coordinates.ecfc_position_to_geodetic` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-012` | 2-42 through 2-46 | `taoryx.coordinates.ecfc_velocity_to_geodetic` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-013` | 2-47 through 2-49 | derived `taoryx.coordinates.geodetic_velocity_to_ecfc` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-014` | 2-50 through 2-53 | `taoryx.attitude.euler_angles_to_body_basis` | `tests/unit/test_attitude.py` |
| `TAOS-ALG-COORD-015` | 2-54 through 2-61 | derived `taoryx.attitude.body_basis_to_euler_angles` | `tests/unit/test_attitude.py` |
| `TAOS-ALG-COORD-016` | 2-62 through 2-70 | `taoryx.coordinates.velocity_frames` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-017` | 2-71 through 2-89 | `taoryx.attitude.wind_to_body_aerodynamic`; derived `body_basis_to_wind_aerodynamic` | `tests/unit/test_attitude.py` |
| `TAOS-ALG-COORD-018` | section 2.1.10 | `taoryx.coordinates.inertial_platform_coordinates`; derived `inertial_platform_to_ecic` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-COORD-019` | 2-90 through 2-92 | `taoryx.coordinates.tangent_plane_unit_vectors`, `tangent_plane_coordinates`; derived `tangent_plane_to_ecfc` | `tests/unit/test_coordinates.py` |
| `TAOS-ALG-DYN-001` | 2-97 through 2-106 | `taoryx.dynamics.earth_fixed_derivatives` | `tests/unit/test_dynamics.py` |
| `TAOS-ALG-DYN-002` | 2-107 through 2-113 | `taoryx.dynamics.assemble_state_derivatives` | `tests/unit/test_dynamics.py` |
| `TAOS-ALG-DYN-003` | 2-114 through 2-118 | `taoryx.integration.rk4_step`, `taoryx.integration.RK4Integrator` | `tests/unit/test_integration.py` |
| `TAOS-ALG-IIP-002` | Fehlberg adaptive Runge-Kutta | `taoryx.integration.rkf45_step` | `tests/unit/test_integration.py` |
| `TAOS-ALG-IIP-003` | IIP propagation to impact altitude | `taoryx.iip.initial_impact_point` | `tests/unit/test_iip.py` |
| `TAOS-ALG-OPT-001` | Chapter 4 optimization problem definition | `taoryx.optimization.build_optimization_problem` | `tests/unit/test_optimization.py` |
| `TAOS-ALG-OPT-002` | Chapter 4 Han-Powell/RQP optimization loop | `taoryx.optimization.han_powell_rqp` | `tests/unit/test_optimization.py` |
| `TAOS-ALG-OPT-003` | Chapter 4 path-constraint violation integral | `taoryx.optimization.path_violation_integral` | `tests/unit/test_optimization.py` |
| `TAOS-ALG-OPT-005` | Chapter 4 control-history redistribution | `taoryx.optimization.redistribute_control_history` | `tests/unit/test_optimization.py` |
| `TAOS-ALG-OUT-001` | Chapter 4 demand-driven output evaluation | `taoryx.outputs.build_output_evaluation_plan` | `tests/unit/test_outputs.py` |
| `TAOS-ALG-DYN-004` | 2-119 through 2-122 | `taoryx.dynamics.apply_rail_constraint` | `tests/unit/test_dynamics.py` |
| `TAOS-ALG-DYN-005` | 2-123 through 2-128 | `taoryx.state_rates.longitude_and_geocentric_latitude_rates` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-006` | 2-129 through 2-137 | `taoryx.state_rates.geodetic_latitude_and_altitude_rates` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-007` | 2-138 through 2-140 | `taoryx.state_rates.altitude_acceleration` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-008` | 2-141 through 2-144 | `taoryx.state_rates.dynamic_pressure_derivatives` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-009` | 2-145 through 2-147 | `taoryx.state_rates.mach_rate` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-010` | 2-148 through 2-152 | `taoryx.state_rates.ground_speed` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-011` | 2-153 through 2-171 | `taoryx.state_rates.flight_path_angle_rates` | `tests/unit/test_state_rates.py` |
| `TAOS-ALG-DYN-012` | 2-172 through 2-174 | `taoryx.dynamics.specific_load_factors` | `tests/unit/test_dynamics.py` |
| `TAOS-ALG-ENV-001` | 2-175 | `taoryx.dynamics.combine_acceleration_contributions` | `tests/unit/test_dynamics.py` |
| `TAOS-ALG-ENV-002` | section 2.3.1 | `taoryx.atmosphere.prepare_atmosphere_model` | `tests/unit/test_atmosphere.py` |
| `TAOS-ALG-ENV-003` | 2-176 through 2-184 | `taoryx.atmosphere.geopotential_altitude` | `tests/unit/test_atmosphere.py` |
| `TAOS-ALG-ENV-004` | 2-185 through 2-195 | `taoryx.atmosphere.atmosphere_properties` | `tests/unit/test_atmosphere.py` |
| `TAOS-ALG-ENV-005` | section 2.3.1 | `taoryx.atmosphere.high_altitude_atmosphere` | `tests/unit/test_atmosphere.py` |
| `TAOS-ALG-FORCE-001` | 2-196 through 2-197 | `taoryx.forces.force_from_axial_normal_coefficients` | `tests/unit/test_forces.py` |
| `TAOS-ALG-FORCE-002` | 2-198 | `taoryx.forces.force_from_wind_coefficients` | `tests/unit/test_forces.py` |
| `TAOS-ALG-FORCE-003` | 2-199 | `taoryx.forces.force_from_body_coefficients` | `tests/unit/test_forces.py` |
| `TAOS-ALG-FORCE-004` | 2-196 through 2-199 | `taoryx.forces.evaluate_aerodynamic_forces` | `tests/unit/test_forces.py` |
| `TAOS-ALG-FORCE-005` | 2-200 | `taoryx.forces.propulsive_force` | `tests/unit/test_forces.py` |
| `TAOS-ALG-TABLE-001` | section 3.3 through 3.4 | `taoryx.tables.prepare_table` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-002` | section 3.4 | `taoryx.tables.interpolate_nd` | `tests/unit/test_tables.py` |
| `TAOS-ALG-GRAV-001` | 2-201 through 2-205 | `taoryx.gravity.geopotential` | `tests/unit/test_gravity.py` |
| `TAOS-ALG-GRAV-002` | 2-206 through 2-209 | `taoryx.gravity.associated_legendre` | `tests/unit/test_gravity.py` |
| `TAOS-ALG-GRAV-003` | 2-210 through 2-211 | `taoryx.gravity.harmonic_normalization_factor` | `tests/unit/test_gravity.py` |
| `TAOS-ALG-GRAV-004` | 2-212 through 2-217 | `taoryx.gravity.gravity_acceleration_full` | `tests/unit/test_gravity.py` |
| `TAOS-ALG-GRAV-005` | 2-218 through 2-220 | `taoryx.gravity.gravity_acceleration_j2` | `tests/unit/test_gravity.py` |
| `TAOS-ALG-GEO-001` | 2-221 through 2-231 | `taoryx.geodesy.sodano_inverse` | `tests/unit/test_geodesy.py` |
| `TAOS-ALG-GEO-002` | 2-232 through 2-234 | `taoryx.geodesy.downrange_crossrange` | `tests/unit/test_geodesy.py` |
| `TAOS-ALG-GEO-003` | 2-235 through 2-245 | `taoryx.geodesy.sodano_direct` | `tests/unit/test_geodesy.py` |
| `TAOS-ALG-SEARCH-003` | 2-301 through 2-305 | `taoryx.searches.parabolic_root` | `tests/unit/test_searches.py` |
| `TAOS-ALG-SEARCH-005` | 2-308 | `taoryx.searches.parabolic_minimize` | `tests/unit/test_searches.py` |
| `TAOS-ALG-AERO-001` | 2-271 through 2-272 | `taoryx.aerodynamics.aerodynamic_performance_metrics` | `tests/unit/test_aerodynamics.py` |
| `TAOS-ALG-AERO-002` | section 2.4.5 | `taoryx.aerodynamics.maximum_lift_to_drag` | `tests/unit/test_aerodynamics.py` |
| `TAOS-ALG-GUID-003` | 2-273 through 2-276 | `taoryx.guidance.parabolic_guidance_correction` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-001` | Chapter 2.5.1 control-set classification | `taoryx.guidance.classify_guidance_rules` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-002` | Chapter 2.5.1 guidance loop | `taoryx.guidance.solve_guidance` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-004` | 2-277 through 2-280 | `taoryx.guidance.cubic_guidance_correction` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-005` | 2-281 through 2-283 | `taoryx.numeric.newton_system` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-006` | 2-284 through 2-286 | `taoryx.guidance.predictive_intercept` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-GUID-007` | 2-287 through 2-297 | `taoryx.guidance.proportional_navigation` | `tests/unit/test_radar.py` |
| `TAOS-ALG-GUID-008` | Chapter 2.5.3 range-insensitive axis | `taoryx.guidance.range_insensitive_axis` | `tests/unit/test_guidance.py`; `tests/unit/test_runtime_lowering.py` |
| `TAOS-ALG-GUID-009` | Chapter 2.5.4 flight-path limits | `taoryx.guidance.apply_flight_path_limits` | `tests/unit/test_guidance.py` |
| `TAOS-ALG-IIP-001` | 2-266 through 2-270 | `taoryx.iip.iip_derivatives` | `tests/unit/test_iip.py` |
| `TAOS-ALG-IIP-002` | Fehlberg adaptive Runge-Kutta | `taoryx.integration.rkf45_step` | `tests/unit/test_integration.py` |
| `TAOS-ALG-RADAR-001` | 2-246 through 2-261 | `taoryx.radar.radar_observations` | `tests/unit/test_radar.py` |
| `TAOS-ALG-REL-001` | 2-262 through 2-265 | `taoryx.radar.relative_vehicle_observations` | `tests/unit/test_radar.py` |
| `TAOS-ALG-TABLE-003` | Chapter 3 full-table operations | `taoryx.tables.evaluate_full_table` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-004` | Chapter 3 operand resolution | `taoryx.tables.resolve_table_operand` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-005` | Chapter 3 math-operation dispatch | `taoryx.tables.apply_table_operation` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-006` | Chapter 3 storage variables | `taoryx.tables.clear_and_store` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-007` | Chapter 3 if/goto control flow | `taoryx.tables.evaluate_full_table` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-008` | Chapter 3 skewed tabulated data | `taoryx.tables.interpolate_skewed` | `tests/unit/test_tables.py` |
| `TAOS-ALG-TABLE-009` | Chapter 3 multiple-table accumulation | `taoryx.tables.accumulate_table_values` | `tests/unit/test_tables.py` |
| `TAOS-ALG-EXEC-001` | Chapter 2.6 trajectory calculation workflow | `taoryx.runtime.engine.run_taos` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-002` | Chapter 2.6 runtime vehicle graph | `taoryx.runtime.runtime_model.build_runtime_problem` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-003` | Chapter 2.6 synchronized trajectory execution | `taoryx.runtime.engine.compute_trajectories` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-004` | Chapter 2.6 time-step boundary selection | `taoryx.runtime.engine.get_next_time_step` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-005` | Chapter 2.6 active vehicle integration | `taoryx.runtime.engine.integrate_active_vehicles` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-006` | Chapter 2.6 segment final-condition refinement | `taoryx.runtime.events.refine_segment_final_condition` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-007` | Chapter 2.6 search/restart dispatch | `taoryx.runtime.engine.dispatch_search_and_restart` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-008` | Chapter 2.6 dependent vehicle activation | `taoryx.runtime.engine.activate_dependent_vehicles` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-EXEC-009` | Chapter 2.6.1 derivative pipeline | `taoryx.runtime.engine.compute_derivatives` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-001` | Chapter 4 *Define Data Blocks | `taoryx.runtime.expressions.evaluate_definition_program` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-002` | Chapter 4 trajectory integral definitions | `taoryx.runtime.expressions.integral_variable_derivatives` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-003` | Chapter 4 *Initial Data Block | `taoryx.runtime.initialization.resolve_initial_state` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-004` | Chapter 4 *Increment and *Reset blocks | `taoryx.runtime.events.apply_state_discontinuity` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-005` | Chapter 4 *When Data Block | `taoryx.runtime.events.evaluate_when_conditions` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-006` | Chapter 4 *Fly Data Block | `taoryx.runtime.guidance.resolve_fly_rules` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-007` | Chapter 4 *Survey Data Block | `taoryx.runtime.surveys.generate_survey_cases` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-008` | Chapter 4 *Search Data Block | `taoryx.runtime.search_runtime.execute_search_loops` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-009` | Chapter 4 *Summarize Data Block | `taoryx.runtime.summaries.evaluate_summary` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-010` | Chapter 4 *Optimize Data Block | `taoryx.runtime.optimization_runtime.resolve_optimize_block` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-011` | Chapter 4 *Units/Fmt Data Block | `taoryx.runtime.units.resolve_units_and_formats` | `tests/unit/test_runtime_algorithms.py` |
| `TAOS-ALG-PRB-012` | Chapter 4 *Wind Data Block | `taoryx.runtime.environment_runtime.evaluate_wind` | `tests/unit/test_runtime_algorithms.py` |

The implementation follows the manual's passive transformation convention:
the basis vectors express the child frame in parent-frame components. Forward
transformation expands child components in the parent basis; inverse
transformation projects onto the orthonormal basis.
