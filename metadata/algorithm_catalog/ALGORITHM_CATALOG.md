# TAOS Algorithm Catalog, Version 1

This is the reviewed planning catalog for the 1995 TAOS manual. It decomposes
the manual into implementation-sized algorithms and is designed as a work queue
for a modern, traceable implementation rather than as a claim that the
historical TAOS 96.0 executable has been reproduced.

For the architecture-level explanation of the catalog boundaries, see
[`docs/architecture/algorithm-catalog.md`](../../docs/architecture/algorithm-catalog.md).

## Coverage

- **Algorithms:** 107
- **Dependency edges:** 152
- **Chapter 2 numbered equations mapped:** 315/315
- **All 326 numbered manual equations:** mapped to one or more algorithms or explicitly marked documentation-only in `equation_algorithm_map.csv`.
- **Runtime scope:** Chapter 2 computational methods, Chapter 3 table evaluation, and Chapter 4 execution semantics.

## Domain summary

| Domain | Algorithms |
|---|---:|
| `coordinates` | 20 |
| `dynamics` | 12 |
| `environment` | 5 |
| `execution` | 9 |
| `forces` | 5 |
| `geodesy` | 3 |
| `gravity` | 5 |
| `guidance` | 9 |
| `numerics` | 6 |
| `optimization` | 4 |
| `outputs` | 8 |
| `problem_runtime` | 12 |
| `table_runtime` | 9 |

## Implementation phases

| Phase | Algorithms |
|---|---:|
| `M0-foundation` | 9 |
| `M1-coordinate-foundation` | 19 |
| `M2-environment` | 12 |
| `M3-dynamics` | 18 |
| `M4-output-models` | 9 |
| `M5-guidance-search` | 9 |
| `M6-execution-engine` | 12 |
| `M7-table-and-problem-runtime` | 14 |
| `M8-optimization` | 5 |

## Catalog

The table below is intentionally compact. Full steps, assumptions, edge cases,
dependencies, implementation targets, and test plans are in `algorithms.yaml`.

| ID | Algorithm | Domain | Source | Equations | Target | Priority |
|---|---|---|---|---|---|---|
| TAOS-ALG-COORD-001 | ECFC to ECIC Coordinate Conversion | `coordinates` | §2.1.2; 2-7 | 2-1, 2-2, 2-3, 2-4, 2-5 | `taos_math.coordinates.ecic_coords` | P0 |
| TAOS-ALG-COORD-002 | Local Geocentric Horizon Unit Vectors | `coordinates` | §2.1.3; 2-8 | 2-6 | `taos_math.coordinates.geocentric_unit_vectors` | P0 |
| TAOS-ALG-COORD-003 | Geocentric Position to ECFC | `coordinates` | §2.1.4; 2-9 | 2-7 | `taos_math.coordinates.geocentric_position_to_ecfc` | P0 |
| TAOS-ALG-COORD-004 | ECFC Position to Geocentric Coordinates | `coordinates` | §2.1.4; 2-10 | 2-8, 2-9 | `taos_math.coordinates.ecfc_position_to_geocentric` | P0 |
| TAOS-ALG-COORD-005 | ECFC Velocity to Geocentric Flight-Path Coordinates | `coordinates` | §2.1.4; 2-11 | 2-10, 2-11, 2-12, 2-13, 2-14 | `taos_math.coordinates.ecfc_velocity_to_geocentric` | P0 |
| TAOS-ALG-COORD-006 | Geocentric Flight-Path Coordinates to ECFC Velocity | `coordinates` | §2.1.4; 2-11 | 2-15, 2-16, 2-17 | `taos_math.coordinates.geocentric_velocity_to_ecfc` | P0 |
| TAOS-ALG-COORD-007 | Ellipsoid Shape-Parameter Conversion | `coordinates` | §2.1.5; 2-12 | 2-18 | `taos_math.earth.resolve_ellipsoid_parameters` | P0 |
| TAOS-ALG-COORD-008 | Local Geodetic Horizon Unit Vectors | `coordinates` | §2.1.5; 2-13 | 2-19, 2-20, 2-21 | `taos_math.coordinates.geodetic_unit_vectors` | P0 |
| TAOS-ALG-COORD-009 | Ellipsoidal Surface Geometry | `coordinates` | §2.1.6; 2-14, 2-15 | 2-22, 2-23, 2-24, 2-25, 2-26, 2-27, 2-28, 2-29 | `taos_math.earth.ellipsoidal_surface_geometry` | P0 |
| TAOS-ALG-COORD-010 | Geodetic Position to ECFC | `coordinates` | §2.1.6; 2-16 | 2-30, 2-31, 2-32, 2-33 | `taos_math.coordinates.geodetic_position_to_ecfc` | P0 |
| TAOS-ALG-COORD-011 | ECFC Position to Geodetic Coordinates | `coordinates` | §2.1.6; 2-16, 2-17 | 2-34, 2-35, 2-36, 2-37, 2-38, 2-39, 2-40, 2-41 | `taos_math.coordinates.ecfc_position_to_geodetic` | P0 |
| TAOS-ALG-COORD-012 | ECFC Velocity to Geodetic Flight-Path Coordinates | `coordinates` | §2.1.6; 2-18 | 2-42, 2-43, 2-44, 2-45, 2-46 | `taos_math.coordinates.ecfc_velocity_to_geodetic` | P0 |
| TAOS-ALG-COORD-013 | Geodetic Flight-Path Coordinates to ECFC Velocity | `coordinates` | §2.1.6; 2-18 | 2-47, 2-48, 2-49 | `taos_math.coordinates.geodetic_velocity_to_ecfc` | P0 |
| TAOS-ALG-COORD-014 | Euler Angles to Body-Fixed Basis | `coordinates` | §2.1.7; 2-20 | 2-50, 2-51, 2-52, 2-53 | `taos_math.attitude.euler_angles_to_body_basis` | P0 |
| TAOS-ALG-COORD-015 | Body-Fixed Basis to Euler Angles | `coordinates` | §2.1.7; 2-21, 2-22 | 2-54, 2-55, 2-56, 2-57, 2-58, 2-59, 2-60, 2-61 | `taos_math.attitude.body_basis_to_euler_angles` | P0 |
| TAOS-ALG-COORD-016 | Wind-Relative and Earth-Relative Velocity Frames | `coordinates` | §2.1.8; 2-24, 2-25 | 2-62, 2-63, 2-64, 2-65, 2-66, 2-67, 2-68, 2-69, 2-70 | `taos_math.coordinates.velocity_frames` | P0 |
| TAOS-ALG-COORD-017 | Wind, Body, and Aerodynamic Angle Transformations | `coordinates` | §2.1.9; 2-26, 2-27, 2-29, 2-30 | 2-71, 2-72, 2-73, 2-74, 2-75, 2-76, 2-77, 2-78, 2-79, 2-80, 2-81, 2-82, 2-83, 2-84, 2-85, 2-86, 2-87, 2-88, 2-89 | `taos_math.attitude.resolve_aerodynamic_attitude` | P0 |
| TAOS-ALG-COORD-018 | Inertial Platform Coordinate Evaluation | `coordinates` | §2.1.10; 2-32 | — | `taos_math.coordinates.inertial_platform_coordinates` | P1 |
| TAOS-ALG-COORD-019 | Tangent-Plane Basis and Coordinates | `coordinates` | §2.1.11; 2-33 | 2-90, 2-91, 2-92 | `taos_math.coordinates.tangent_plane_coordinates` | P1 |
| TAOS-ALG-COORD-020 | Generic Unit-Vector Coordinate Transformation | `coordinates` | §2.1.12; 2-35 | 2-93, 2-94, 2-95, 2-96 | `taos_math.linalg.transform_vector` | P0 |
| TAOS-ALG-DYN-001 | Rotating-Earth Point-Mass Equations of Motion | `dynamics` | §2.2; 2-36, 2-37 | 2-97, 2-98, 2-99, 2-100, 2-101, 2-102, 2-103, 2-104, 2-105, 2-106 | `taos_math.dynamics.earth_fixed_derivatives` | P0 |
| TAOS-ALG-DYN-002 | State Vector and Derivative Assembly | `dynamics` | §2.2.1; 2-38, 2-39 | 2-107, 2-108, 2-109, 2-110, 2-111, 2-112, 2-113 | `taos_math.dynamics.assemble_state_derivatives` | P0 |
| TAOS-ALG-DYN-003 | Fixed-Step Fourth-Order Runge-Kutta Integration | `dynamics` | §2.2.1; 2-40 | 2-114, 2-115, 2-116, 2-117, 2-118 | `taos_math.integration.rk4_step` | P0 |
| TAOS-ALG-DYN-004 | Rail Launch and Sled Constraint Model | `dynamics` | §2.2.2; 2-41 | 2-119, 2-120, 2-121, 2-122 | `taos_math.dynamics.apply_rail_constraint` | P1 |
| TAOS-ALG-DYN-005 | Longitude and Geocentric Latitude Rates | `dynamics` | §2.2.3; 2-42, 2-43 | 2-123, 2-124, 2-125, 2-126, 2-127, 2-128 | `taos_math.state_rates.longitude_and_geocentric_latitude_rates` | P1 |
| TAOS-ALG-DYN-006 | Geodetic Latitude and Altitude Rates | `dynamics` | §2.2.3; 2-43 | 2-129, 2-130, 2-131, 2-132, 2-133, 2-134, 2-135, 2-136, 2-137 | `taos_math.state_rates.geodetic_latitude_and_altitude_rates` | P1 |
| TAOS-ALG-DYN-007 | Geodetic Altitude Acceleration | `dynamics` | §2.2.3; 2-44 | 2-138, 2-139, 2-140 | `taos_math.state_rates.altitude_acceleration` | P1 |
| TAOS-ALG-DYN-008 | Dynamic-Pressure Rate and Acceleration | `dynamics` | §2.2.3; 2-44 | 2-141, 2-142, 2-143, 2-144 | `taos_math.state_rates.dynamic_pressure_derivatives` | P2 |
| TAOS-ALG-DYN-009 | Mach-Number Rate | `dynamics` | §2.2.3; 2-45 | 2-145, 2-146, 2-147 | `taos_math.state_rates.mach_rate` | P2 |
| TAOS-ALG-DYN-010 | Ground Speed and Ground-Range Integration | `dynamics` | §2.2.4; 2-46 | 2-148, 2-149, 2-150, 2-151, 2-152 | `taos_math.state_rates.ground_speed` | P1 |
| TAOS-ALG-DYN-011 | Geocentric and Geodetic Flight-Path-Angle Rates | `dynamics` | §2.2.5; 2-47, 2-48, 2-49 | 2-153, 2-154, 2-155, 2-156, 2-157, 2-158, 2-159, 2-160, 2-161, 2-162, 2-163, 2-164, 2-165, 2-166, 2-167, 2-168, 2-169, 2-170, 2-171 | `taos_math.state_rates.flight_path_angle_rates` | P1 |
| TAOS-ALG-DYN-012 | Specific Load and Body-Axis Load Factors | `dynamics` | §2.2.6; 2-50 | 2-172, 2-173, 2-174 | `taos_math.dynamics.specific_load_factors` | P1 |
| TAOS-ALG-ENV-001 | Acceleration Contribution Decomposition | `environment` | §2.3; 2-51 | 2-175 | `taos_math.dynamics.combine_acceleration_contributions` | P1 |
| TAOS-ALG-ENV-002 | Atmosphere Model Setup and Layer Precomputation | `environment` | §2.3.1; 2-52, 2-53, 2-54, 2-55 | — | `taos_math.atmosphere.prepare_atmosphere_model` | P0 |
| TAOS-ALG-ENV-003 | Reference Gravity and Geopotential Altitude | `environment` | §2.3.1; 2-52, 2-53 | 2-176, 2-177, 2-178, 2-179, 2-180, 2-181, 2-182, 2-183, 2-184 | `taos_math.atmosphere.geopotential_altitude` | P0 |
| TAOS-ALG-ENV-004 | Layered Atmosphere Thermodynamic Properties | `environment` | §2.3.1; 2-54, 2-55 | 2-185, 2-186, 2-187, 2-188, 2-189, 2-190, 2-191, 2-192, 2-193, 2-194, 2-195 | `taos_math.atmosphere.atmosphere_properties` | P0 |
| TAOS-ALG-ENV-005 | High-Altitude Atmosphere Interpolation | `environment` | §2.3.1; 2-55 | — | `taos_math.atmosphere.high_altitude_atmosphere` | P1 |
| TAOS-ALG-FORCE-001 | Axial and Normal Aerodynamic Force Conversion | `forces` | §2.3.2; 2-56 | 2-196, 2-197 | `taos_math.aerodynamics.force_from_axial_normal_coefficients` | P0 |
| TAOS-ALG-FORCE-002 | Lift, Drag, and Side-Force Conversion | `forces` | §2.3.2; 2-57 | 2-198 | `taos_math.aerodynamics.force_from_wind_coefficients` | P0 |
| TAOS-ALG-FORCE-003 | Body-Axis Aerodynamic Force Conversion | `forces` | §2.3.2; 2-57 | 2-199 | `taos_math.aerodynamics.force_from_body_coefficients` | P0 |
| TAOS-ALG-FORCE-004 | Aerodynamic Table Evaluation and Force Accumulation | `forces` | §2.3.2; 2-56, 2-57, 2-58 | — | `taos_math.aerodynamics.evaluate_aerodynamic_forces` | P1 |
| TAOS-ALG-FORCE-005 | Propulsive Force Vector and Accumulation | `forces` | §2.3.3; 2-59 | 2-200 | `taos_math.propulsion.evaluate_propulsive_forces` | P1 |
| TAOS-ALG-GRAV-001 | Geopotential Spherical-Harmonic Expansion | `gravity` | §2.3.4; 2-60, 2-61 | 2-201, 2-202, 2-203, 2-204, 2-205 | `taos_math.gravity.geopotential` | P1 |
| TAOS-ALG-GRAV-002 | Associated Legendre Function Evaluation | `gravity` | §2.3.4; 2-61 | 2-206, 2-207, 2-208, 2-209 | `taos_math.gravity.associated_legendre` | P0 |
| TAOS-ALG-GRAV-003 | Spherical-Harmonic Normalization Conversion | `gravity` | §2.3.4; 2-61, 2-62 | 2-210, 2-211 | `taos_math.gravity.harmonic_normalization_factor` | P0 |
| TAOS-ALG-GRAV-004 | Degree-Four Spherical-Harmonic Gravity Acceleration | `gravity` | §2.3.4; 2-62 | 2-212, 2-213, 2-214, 2-215, 2-216, 2-217 | `taos_math.gravity.gravity_acceleration_full` | P1 |
| TAOS-ALG-GRAV-005 | J2-Only Gravity Acceleration | `gravity` | §2.3.4; 2-63 | 2-218, 2-219, 2-220 | `taos_math.gravity.gravity_acceleration_j2` | P0 |
| TAOS-ALG-OUT-001 | On-Demand Output-Variable Dispatch | `outputs` | §2.4; 2-64, 2-65 | — | `taos_runtime.outputs.build_output_evaluation_plan` | P2 |
| TAOS-ALG-GEO-001 | Sodano Inverse Ellipsoidal Geodesic | `geodesy` | §2.4.1; 2-67, 2-68 | 2-221, 2-222, 2-223, 2-224, 2-225, 2-226, 2-227, 2-228, 2-229, 2-230, 2-231 | `taos_math.geodesy.sodano_inverse` | P1 |
| TAOS-ALG-GEO-002 | Downrange and Crossrange Projection Search | `geodesy` | §2.4.1; 2-68, 2-69 | 2-232, 2-233, 2-234 | `taos_math.geodesy.downrange_crossrange` | P2 |
| TAOS-ALG-GEO-003 | Sodano Direct Ellipsoidal Geodesic | `geodesy` | §2.4.1; 2-69, 2-70 | 2-235, 2-236, 2-237, 2-238, 2-239, 2-240, 2-241, 2-242, 2-243, 2-244, 2-245 | `taos_math.geodesy.sodano_direct` | P1 |
| TAOS-ALG-RADAR-001 | Radar Observation Geometry and Derivatives | `outputs` | §2.4.2; 2-71, 2-72, 2-73 | 2-246, 2-247, 2-248, 2-249, 2-250, 2-251, 2-252, 2-253, 2-254, 2-255, 2-256, 2-257, 2-258, 2-259, 2-260, 2-261 | `taos_math.radar.radar_observations` | P1 |
| TAOS-ALG-REL-001 | Relative Vehicle Observation Geometry | `outputs` | §2.4.3; 2-74 | 2-262, 2-263, 2-264, 2-265 | `taos_math.relative_motion.relative_vehicle_observations` | P1 |
| TAOS-ALG-IIP-001 | Initial Impact Point Ballistic Derivatives | `outputs` | §2.4.4; 2-75 | 2-266, 2-267, 2-268, 2-269, 2-270 | `taos_math.iip.iip_derivatives` | P2 |
| TAOS-ALG-IIP-002 | Fehlberg Adaptive Runge-Kutta 4/5 Integration | `outputs` | §2.4.4; 2-75, 2-76 | — | `taos_math.integration.rkf45_step` | P1 |
| TAOS-ALG-IIP-003 | Initial Impact Point Propagation and Output | `outputs` | §2.4.4; 2-75, 2-76 | — | `taos_math.iip.initial_impact_point` | P2 |
| TAOS-ALG-AERO-001 | Ballistic Coefficient and Lift-to-Drag Ratio | `outputs` | §2.4.5; 2-77 | 2-271, 2-272 | `taos_math.aerodynamics.aerodynamic_performance_metrics` | P2 |
| TAOS-ALG-AERO-002 | Maximum Lift-to-Drag Angle Search | `outputs` | §2.4.5; 2-77 | — | `taos_math.aerodynamics.maximum_lift_to_drag` | P2 |
| TAOS-ALG-GUID-001 | Guidance Rule Classification and Control-Set Selection | `guidance` | §2.5; 2-78, 2-79, 2-80, 2-81 | — | `taos_math.guidance.classify_guidance_rules` | P1 |
| TAOS-ALG-GUID-002 | Iterative Guidance Loop | `guidance` | §2.5.1; 2-81, 2-82, 2-83, 2-84 | — | `taos_math.guidance.solve_guidance` | P1 |
| TAOS-ALG-GUID-003 | Parabolic Guidance Transition | `guidance` | §2.5.1; 2-82, 2-83 | 2-273, 2-274, 2-275, 2-276 | `taos_math.guidance.parabolic_guidance_correction` | P0 |
| TAOS-ALG-GUID-004 | Cubic Guidance Transition | `guidance` | §2.5.1; 2-83, 2-84 | 2-277, 2-278, 2-279, 2-280 | `taos_math.guidance.cubic_guidance_correction` | P0 |
| TAOS-ALG-GUID-005 | Multidimensional Newton-Raphson Control Solve | `guidance` | §2.5.1; 2-84 | 2-281, 2-282, 2-283 | `taos_math.numeric.newton_system` | P0 |
| TAOS-ALG-GUID-006 | Predictive Intercept Guidance | `guidance` | §2.5.2; 2-85 | 2-284, 2-285, 2-286 | `taos_math.guidance.predictive_intercept` | P2 |
| TAOS-ALG-GUID-007 | Proportional Navigation Guidance | `guidance` | §2.5.2; 2-87, 2-88 | 2-287, 2-288, 2-289, 2-290, 2-291, 2-292, 2-293, 2-294, 2-295, 2-296, 2-297 | `taos_math.guidance.proportional_navigation` | P2 |
| TAOS-ALG-GUID-008 | Range-Insensitive Axis Search | `guidance` | §2.5.3; 2-89 | — | `taos_math.guidance.range_insensitive_axis` | P3 |
| TAOS-ALG-GUID-009 | Flight-Path Limit Arbitration | `guidance` | §2.5.4; 2-90 | — | `taos_math.guidance.apply_flight_path_limits` | P2 |
| TAOS-ALG-SEARCH-001 | One-Dimensional Newton-Raphson Root Search | `numerics` | §2.6.2; 2-98 | 2-298, 2-299 | `taos_math.searches.newton_root` | P0 |
| TAOS-ALG-SEARCH-002 | Bracketed Secant Root Search | `numerics` | §2.6.2; 2-99 | 2-300 | `taos_math.searches.secant_bracketed_root` | P0 |
| TAOS-ALG-SEARCH-003 | Parabolic Root Search | `numerics` | §2.6.2; 2-100, 2-99 | 2-301, 2-302, 2-303, 2-304, 2-305 | `taos_math.searches.parabolic_root` | P1 |
| TAOS-ALG-SEARCH-004 | Golden-Section Minimization | `numerics` | §2.6.2; 2-101 | 2-306, 2-307 | `taos_math.searches.golden_section_minimize` | P0 |
| TAOS-ALG-SEARCH-005 | Parabolic Minimization | `numerics` | §2.6.2; 2-101 | 2-308 | `taos_math.searches.parabolic_minimize` | P1 |
| TAOS-ALG-OPT-001 | TAOS Nonlinear Programming Problem Construction | `optimization` | §2.6.3; 2-102 | 2-309, 2-310 | `taos_math.optimization.build_optimization_problem` | P2 |
| TAOS-ALG-OPT-002 | Han-Powell Recursive Quadratic Programming | `optimization` | §2.6.3; 2-102, 2-103 | 2-311, 2-312, 2-313 | `taos_math.optimization.han_powell_rqp` | P3 |
| TAOS-ALG-OPT-003 | Path-Integrated Optimization Constraint Variables | `optimization` | §2.6.3; 2-104 | 2-314, 2-315 | `taos_math.optimization.path_violation_integral` | P2 |
| TAOS-ALG-OPT-004 | Forward and Central Numerical Gradients | `numerics` | §2.6.3; 2-104, 2-105 | — | `taos_math.numeric.finite_difference_jacobian` | P0 |
| TAOS-ALG-OPT-005 | Trajectory-Shaping Time-Grid Redistribution | `optimization` | §2.6.3; 2-103, 2-104 | — | `taos_math.optimization.redistribute_control_history` | P3 |
| TAOS-ALG-EXEC-001 | Main Program Table/Problem/File Loop | `execution` | §2.6; 2-91, 2-92 | — | `taos_runtime.engine.run_taos` | P2 |
| TAOS-ALG-EXEC-002 | Vehicle, Segment, and Output Runtime Structure Construction | `execution` | §2.6; 2-92 | — | `taos_runtime.runtime_model.build_runtime_problem` | P1 |
| TAOS-ALG-EXEC-003 | Synchronized Multi-Vehicle Trajectory Loop | `execution` | §2.6; 2-92, 2-93, 2-94, 2-95 | — | `taos_runtime.engine.compute_trajectories` | P1 |
| TAOS-ALG-EXEC-004 | Common Next-Time-Step Selection | `execution` | §2.6; 2-93 | — | `taos_runtime.engine.get_next_time_step` | P1 |
| TAOS-ALG-EXEC-005 | Integrate All Active Vehicles One Step | `execution` | §2.6; 2-93, 2-94 | — | `taos_runtime.engine.integrate_active_vehicles` | P1 |
| TAOS-ALG-EXEC-006 | Segment Final-Condition Detection and Refinement | `execution` | §2.6; 2-94 | — | `taos_runtime.events.refine_segment_final_condition` | P1 |
| TAOS-ALG-EXEC-007 | Search and Optimization Endpoint Dispatch with Partial Restart | `execution` | §2.6; 2-94 | — | `taos_runtime.engine.dispatch_search_and_restart` | P2 |
| TAOS-ALG-EXEC-008 | Dependent Vehicle Activation | `execution` | §2.6; 2-94, 2-95 | — | `taos_runtime.engine.activate_dependent_vehicles` | P2 |
| TAOS-ALG-EXEC-009 | Complete Derivative Calculation Pipeline | `execution` | §2.6.1; 2-96, 2-97 | — | `taos_runtime.engine.compute_derivatives` | P1 |
| TAOS-ALG-TABLE-001 | Table Definition Validation and Storage Layout | `table_runtime` | §3.3-3.4; 3-6, 3-7, 3-8, 3-9, 3-10, 3-11, 3-12 | — | `taos_runtime.tables.prepare_table` | P0 |
| TAOS-ALG-TABLE-002 | N-Dimensional Linear Interpolation and Extrapolation | `table_runtime` | §3.4; 3-9, 3-10, 3-11, 3-12 | — | `taos_runtime.tables.interpolate_nd` | P0 |
| TAOS-ALG-TABLE-003 | Full-Table Accumulator Evaluation | `table_runtime` | §3.5; 3-13, 3-14, 3-15 | — | `taos_runtime.tables.evaluate_full_table` | P0 |
| TAOS-ALG-TABLE-004 | Full-Table Operand Resolution | `table_runtime` | §3.5.2-3.5.5; 3-16, 3-17, 3-18, 3-19 | — | `taos_runtime.tables.resolve_table_operand` | P0 |
| TAOS-ALG-TABLE-005 | Full-Table Math-Operation Dispatch | `table_runtime` | §3.5.1; 3-15 | — | `taos_runtime.tables.apply_table_operation` | P0 |
| TAOS-ALG-TABLE-006 | Full-Table Storage Variable Semantics | `table_runtime` | §3.5.4; 3-18 | — | `taos_runtime.tables.clear_and_store` | P1 |
| TAOS-ALG-TABLE-007 | Full-Table If/Label/Goto Control Flow | `table_runtime` | §3.5.6-3.5.7; 3-20, 3-21 | — | `taos_runtime.tables.execute_table_control_flow` | P1 |
| TAOS-ALG-TABLE-008 | Skewed Tabulated-Data Evaluation | `table_runtime` | §3.5.8; 3-22, 3-23, 3-24 | — | `taos_runtime.tables.interpolate_skewed` | P1 |
| TAOS-ALG-TABLE-009 | Multiple Table Reference Evaluation and Accumulation | `table_runtime` | §3.2; 3-5 | — | `taos_runtime.tables.evaluate_table_references` | P1 |
| TAOS-ALG-PRB-001 | User-Defined Expression Evaluation | `problem_runtime` | §4.3.1/4.4.2; 4-35, 4-36, 4-37, 4-38, 4-52, 4-53 | — | `taos_runtime.expressions.evaluate_definition_program` | P0 |
| TAOS-ALG-PRB-002 | User-Defined Integral Variable Runtime | `problem_runtime` | §4.3.1; 4-35, 4-36, 4-37, 4-38 | — | `taos_runtime.expressions.integral_variable_derivatives` | P1 |
| TAOS-ALG-PRB-003 | Trajectory Initial-Condition Resolution | `problem_runtime` | §4.3.5; 4-43, 4-44, 4-45 | — | `taos_runtime.initialization.resolve_initial_state` | P1 |
| TAOS-ALG-PRB-004 | Increment and Reset State Discontinuities | `problem_runtime` | §4.2.5/4.2.11; 4-20, 4-21, 4-22, 4-30 | — | `taos_runtime.events.apply_state_discontinuity` | P2 |
| TAOS-ALG-PRB-005 | *When Final-Condition Evaluation and Action | `problem_runtime` | §4.2.12; 4-31, 4-32 | — | `taos_runtime.events.evaluate_when_conditions` | P1 |
| TAOS-ALG-PRB-006 | *Fly Guidance Rule and Table Resolution | `problem_runtime` | §4.2.4; 4-13, 4-14, 4-15, 4-16, 4-17, 4-18, 4-19 | — | `taos_runtime.guidance.resolve_fly_rules` | P1 |
| TAOS-ALG-PRB-007 | Survey Value Generation and Cartesian Nesting | `problem_runtime` | §4.4.11; 4-83, 4-84 | — | `taos_runtime.surveys.generate_survey_cases` | P2 |
| TAOS-ALG-PRB-008 | Search Loop Construction, Nesting, and Execution | `problem_runtime` | §4.4.9; 4-76, 4-77, 4-78, 4-79 | — | `taos_runtime.search_runtime.execute_search_loops` | P2 |
| TAOS-ALG-PRB-009 | Survey Summary Variable Aggregation | `problem_runtime` | §4.4.10; 4-80, 4-81, 4-82 | — | `taos_runtime.summaries.evaluate_summary` | P2 |
| TAOS-ALG-PRB-010 | *Optimize Block Runtime Mapping | `problem_runtime` | §4.4.6; 4-62, 4-63, 4-64, 4-65, 4-66, 4-67, 4-68, 4-69, 4-70, 4-71 | — | `taos_runtime.optimization_runtime.resolve_optimize_block` | P2 |
| TAOS-ALG-PRB-011 | Units and Output-Format Resolution | `problem_runtime` | §4.4.13; 4-86, 4-87, 4-88 | — | `taos_runtime.units.resolve_units_and_formats` | P1 |
| TAOS-ALG-PRB-012 | Wind Input Resolution | `problem_runtime` | §4.4.14; 4-89, 4-90 | — | `taos_runtime.environment_runtime.evaluate_wind` | P1 |

## Reading an algorithm record

Each YAML record includes:

- exact manual sections, pages, equation numbers, figures, tables, and TeX provenance;
- the historical TAOS module/function name when the manual provides it;
- inputs, outputs, assumptions, procedural steps, and edge cases;
- dependencies on other catalog algorithms;
- a proposed modern package/module/function;
- unit, property, regression, and edge-case test ideas;
- implementation phase, priority, complexity, and unresolved questions.

## Important boundaries

- The catalog records algorithms described by the manual; it does not prove exact runtime equivalence.
- External algorithms such as Fehlberg RK4/5 and Han-Powell `vf02ad` require their cited primary sources or historical source code for exact replication.
- Several manual equations have documented editorial issues. The catalog points to the reconstructed source metadata rather than silently resolving them.
- Chapter 3 and 4 runtime records describe semantics beyond parsing; use the separate snippet corpus as grammar-level test input.
