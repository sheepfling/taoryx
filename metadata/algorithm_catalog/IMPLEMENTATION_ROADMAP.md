# TAOS Implementation Roadmap

This order is dependency-aware. Within a milestone, implement P0 items before P1,
then P2/P3. Mark an algorithm complete only after its proposed tests and provenance
links are committed.

## Milestone 0 prerequisites not represented as algorithms

Before the first catalog item, define:

- typed vectors, matrices, frames, angles, and state containers;
- a unit system and conversion registry;
- numeric tolerances and diagnostic types;
- immutable Earth/atmosphere model definitions;
- deterministic floating-point and angle-wrapping conventions.

## M0-foundation

- [x] **TAOS-ALG-COORD-020 — Generic Unit-Vector Coordinate Transformation**
  - Target: `taos_math.linalg.transform_vector`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-OPT-004 — Forward and Central Numerical Gradients**
  - Target: `taos_math.numeric.finite_difference_jacobian`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-SEARCH-001 — One-Dimensional Newton-Raphson Root Search**
  - Target: `taos_math.searches.newton_root`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-SEARCH-002 — Bracketed Secant Root Search**
  - Target: `taos_math.searches.secant_bracketed_root`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-SEARCH-004 — Golden-Section Minimization**
  - Target: `taos_math.searches.golden_section_minimize`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-TABLE-002 — N-Dimensional Linear Interpolation and Extrapolation**
  - Target: `taos_runtime.tables.interpolate_nd`
  - Priority / complexity: P0 / high
  - Dependencies: none
- [x] **TAOS-ALG-PRB-011 — Units and Output-Format Resolution**
  - Target: `taos_runtime.units.resolve_units_and_formats`
  - Priority / complexity: P1 / high
  - Dependencies: none
- [x] **TAOS-ALG-SEARCH-003 — Parabolic Root Search**
  - Target: `taos_math.searches.parabolic_root`
  - Priority / complexity: P1 / high
  - Dependencies: none
- [x] **TAOS-ALG-SEARCH-005 — Parabolic Minimization**
  - Target: `taos_math.searches.parabolic_minimize`
  - Priority / complexity: P1 / high
  - Dependencies: none

## M1-coordinate-foundation

- [x] **TAOS-ALG-COORD-001 — ECFC to ECIC Coordinate Conversion**
  - Target: `taos_math.coordinates.ecic_coords`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-COORD-002 — Local Geocentric Horizon Unit Vectors**
  - Target: `taos_math.coordinates.geocentric_unit_vectors`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-003 — Geocentric Position to ECFC**
  - Target: `taos_math.coordinates.geocentric_position_to_ecfc`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-004 — ECFC Position to Geocentric Coordinates**
  - Target: `taos_math.coordinates.ecfc_position_to_geocentric`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-005 — ECFC Velocity to Geocentric Flight-Path Coordinates**
  - Target: `taos_math.coordinates.ecfc_velocity_to_geocentric`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-002
- [x] **TAOS-ALG-COORD-006 — Geocentric Flight-Path Coordinates to ECFC Velocity**
  - Target: `taos_math.coordinates.geocentric_velocity_to_ecfc`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-002
- [x] **TAOS-ALG-COORD-007 — Ellipsoid Shape-Parameter Conversion**
  - Target: `taos_math.earth.resolve_ellipsoid_parameters`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-008 — Local Geodetic Horizon Unit Vectors**
  - Target: `taos_math.coordinates.geodetic_unit_vectors`
  - Priority / complexity: P0 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-009 — Ellipsoidal Surface Geometry**
  - Target: `taos_math.earth.ellipsoidal_surface_geometry`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-007
- [x] **TAOS-ALG-COORD-010 — Geodetic Position to ECFC**
  - Target: `taos_math.coordinates.geodetic_position_to_ecfc`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-009
- [x] **TAOS-ALG-COORD-011 — ECFC Position to Geodetic Coordinates**
  - Target: `taos_math.coordinates.ecfc_position_to_geodetic`
  - Priority / complexity: P0 / high
  - Dependencies: TAOS-ALG-COORD-007, TAOS-ALG-COORD-009
- [x] **TAOS-ALG-COORD-012 — ECFC Velocity to Geodetic Flight-Path Coordinates**
  - Target: `taos_math.coordinates.ecfc_velocity_to_geodetic`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-008
- [x] **TAOS-ALG-COORD-013 — Geodetic Flight-Path Coordinates to ECFC Velocity**
  - Target: `taos_math.coordinates.geodetic_velocity_to_ecfc`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-008
- [x] **TAOS-ALG-COORD-014 — Euler Angles to Body-Fixed Basis**
  - Target: `taos_math.attitude.euler_angles_to_body_basis`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-008
- [x] **TAOS-ALG-COORD-015 — Body-Fixed Basis to Euler Angles**
  - Target: `taos_math.attitude.body_basis_to_euler_angles`
  - Priority / complexity: P0 / high
  - Dependencies: TAOS-ALG-COORD-014
- [x] **TAOS-ALG-COORD-016 — Wind-Relative and Earth-Relative Velocity Frames**
  - Target: `taos_math.coordinates.velocity_frames`
  - Priority / complexity: P0 / high
  - Dependencies: TAOS-ALG-COORD-008
- [x] **TAOS-ALG-COORD-017 — Wind, Body, and Aerodynamic Angle Transformations**
  - Target: `taos_math.attitude.resolve_aerodynamic_attitude`
  - Priority / complexity: P0 / very_high
  - Dependencies: TAOS-ALG-COORD-016
- [x] **TAOS-ALG-COORD-018 — Inertial Platform Coordinate Evaluation**
  - Target: `taos_math.coordinates.inertial_platform_coordinates`
  - Priority / complexity: P1 / low
  - Dependencies: none
- [x] **TAOS-ALG-COORD-019 — Tangent-Plane Basis and Coordinates**
  - Target: `taos_math.coordinates.tangent_plane_coordinates`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-010

## M2-environment

- [x] **TAOS-ALG-ENV-002 — Atmosphere Model Setup and Layer Precomputation**
  - Target: `taos_math.atmosphere.prepare_atmosphere_model`
  - Priority / complexity: P0 / high
  - Dependencies: TAOS-ALG-ENV-003
- [x] **TAOS-ALG-ENV-003 — Reference Gravity and Geopotential Altitude**
  - Target: `taos_math.atmosphere.geopotential_altitude`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-ENV-004 — Layered Atmosphere Thermodynamic Properties**
  - Target: `taos_math.atmosphere.atmosphere_properties`
  - Priority / complexity: P0 / very_high
  - Dependencies: TAOS-ALG-ENV-002, TAOS-ALG-ENV-003
- [x] **TAOS-ALG-GRAV-002 — Associated Legendre Function Evaluation**
  - Target: `taos_math.gravity.associated_legendre`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-GRAV-003 — Spherical-Harmonic Normalization Conversion**
  - Target: `taos_math.gravity.harmonic_normalization_factor`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-GRAV-005 — J2-Only Gravity Acceleration**
  - Target: `taos_math.gravity.gravity_acceleration_j2`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-002
- [x] **TAOS-ALG-ENV-005 — High-Altitude Atmosphere Interpolation**
  - Target: `taos_math.atmosphere.high_altitude_atmosphere`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-TABLE-002
  - Resolve first: The manual does not specify extrapolation behavior above 1000 km.
- [x] **TAOS-ALG-GEO-001 — Sodano Inverse Ellipsoidal Geodesic**
  - Target: `taos_math.geodesy.sodano_inverse`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-007
  - Resolve first: Document numerical behavior near antipodal configurations, which the manual does not discuss.
- [x] **TAOS-ALG-GEO-003 — Sodano Direct Ellipsoidal Geodesic**
  - Target: `taos_math.geodesy.sodano_direct`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-007
- [x] **TAOS-ALG-GRAV-001 — Geopotential Spherical-Harmonic Expansion**
  - Target: `taos_math.gravity.geopotential`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-GRAV-002, TAOS-ALG-GRAV-003
- [x] **TAOS-ALG-GRAV-004 — Degree-Four Spherical-Harmonic Gravity Acceleration**
  - Target: `taos_math.gravity.gravity_acceleration_full`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-GRAV-001, TAOS-ALG-COORD-006
- [x] **TAOS-ALG-PRB-012 — Wind Input Resolution**
  - Target: `taos_runtime.environment_runtime.evaluate_wind`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-TABLE-009, TAOS-ALG-COORD-002, TAOS-ALG-COORD-008

## M3-dynamics

- [x] **TAOS-ALG-DYN-001 — Rotating-Earth Point-Mass Equations of Motion**
  - Target: `taos_math.dynamics.earth_fixed_derivatives`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-020
- [x] **TAOS-ALG-DYN-002 — State Vector and Derivative Assembly**
  - Target: `taos_math.dynamics.assemble_state_derivatives`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-DYN-001, TAOS-ALG-DYN-010
- [x] **TAOS-ALG-DYN-003 — Fixed-Step Fourth-Order Runge-Kutta Integration**
  - Target: `taos_math.integration.rk4_step`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-DYN-002
- [x] **TAOS-ALG-FORCE-001 — Axial and Normal Aerodynamic Force Conversion**
  - Target: `taos_math.aerodynamics.force_from_axial_normal_coefficients`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-COORD-017
- [x] **TAOS-ALG-FORCE-002 — Lift, Drag, and Side-Force Conversion**
  - Target: `taos_math.aerodynamics.force_from_wind_coefficients`
  - Priority / complexity: P0 / low
  - Dependencies: TAOS-ALG-COORD-017
- [x] **TAOS-ALG-FORCE-003 — Body-Axis Aerodynamic Force Conversion**
  - Target: `taos_math.aerodynamics.force_from_body_coefficients`
  - Priority / complexity: P0 / low
  - Dependencies: TAOS-ALG-COORD-014
- [x] **TAOS-ALG-DYN-004 — Rail Launch and Sled Constraint Model**
  - Target: `taos_math.dynamics.apply_rail_constraint`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-014, TAOS-ALG-DYN-001
- [x] **TAOS-ALG-DYN-005 — Longitude and Geocentric Latitude Rates**
  - Target: `taos_math.state_rates.longitude_and_geocentric_latitude_rates`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-002, TAOS-ALG-COORD-004
- [x] **TAOS-ALG-DYN-006 — Geodetic Latitude and Altitude Rates**
  - Target: `taos_math.state_rates.geodetic_latitude_and_altitude_rates`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-COORD-008, TAOS-ALG-COORD-009, TAOS-ALG-COORD-011
- [x] **TAOS-ALG-DYN-007 — Geodetic Altitude Acceleration**
  - Target: `taos_math.state_rates.altitude_acceleration`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-DYN-005, TAOS-ALG-DYN-006
- [x] **TAOS-ALG-DYN-010 — Ground Speed and Ground-Range Integration**
  - Target: `taos_math.state_rates.ground_speed`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-DYN-006, TAOS-ALG-COORD-012
- [x] **TAOS-ALG-DYN-011 — Geocentric and Geodetic Flight-Path-Angle Rates**
  - Target: `taos_math.state_rates.flight_path_angle_rates`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-016, TAOS-ALG-DYN-005, TAOS-ALG-DYN-006
- [x] **TAOS-ALG-DYN-012 — Specific Load and Body-Axis Load Factors**
  - Target: `taos_math.dynamics.specific_load_factors`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-020, TAOS-ALG-GRAV-004
- [x] **TAOS-ALG-ENV-001 — Acceleration Contribution Decomposition**
  - Target: `taos_math.dynamics.combine_acceleration_contributions`
  - Priority / complexity: P1 / low
  - Dependencies: TAOS-ALG-DYN-001, TAOS-ALG-FORCE-001, TAOS-ALG-FORCE-004, TAOS-ALG-GRAV-004
- [x] **TAOS-ALG-FORCE-004 — Aerodynamic Table Evaluation and Force Accumulation**
  - Target: `taos_math.aerodynamics.evaluate_aerodynamic_forces`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-FORCE-001, TAOS-ALG-FORCE-002, TAOS-ALG-FORCE-003, TAOS-ALG-TABLE-003
- [x] **TAOS-ALG-FORCE-005 — Propulsive Force Vector and Accumulation**
  - Target: `taos_math.propulsion.evaluate_propulsive_forces`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-020
- [x] **TAOS-ALG-DYN-008 — Dynamic-Pressure Rate and Acceleration**
  - Target: `taos_math.state_rates.dynamic_pressure_derivatives`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-DYN-006, TAOS-ALG-DYN-007, TAOS-ALG-ENV-004
- [x] **TAOS-ALG-DYN-009 — Mach-Number Rate**
  - Target: `taos_math.state_rates.mach_rate`
  - Priority / complexity: P2 / medium
  - Dependencies: TAOS-ALG-DYN-006, TAOS-ALG-ENV-004

## M4-output-models

- [x] **TAOS-ALG-IIP-002 — Fehlberg Adaptive Runge-Kutta 4/5 Integration**
  - Target: `taos_math.integration.rkf45_step`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-IIP-001
  - Resolve first: The manual cites Fehlberg but does not print the exact embedded tableau; select and document the historical variant used.
- [x] **TAOS-ALG-RADAR-001 — Radar Observation Geometry and Derivatives**
  - Target: `taos_math.radar.radar_observations`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-008, TAOS-ALG-COORD-010
- [x] **TAOS-ALG-REL-001 — Relative Vehicle Observation Geometry**
  - Target: `taos_math.relative_motion.relative_vehicle_observations`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-COORD-020
- [x] **TAOS-ALG-AERO-001 — Ballistic Coefficient and Lift-to-Drag Ratio**
  - Target: `taos_math.aerodynamics.aerodynamic_performance_metrics`
  - Priority / complexity: P2 / low
  - Dependencies: none
- [x] **TAOS-ALG-AERO-002 — Maximum Lift-to-Drag Angle Search**
  - Target: `taos_math.aerodynamics.maximum_lift_to_drag`
  - Priority / complexity: P2 / medium
  - Dependencies: TAOS-ALG-SEARCH-004, TAOS-ALG-FORCE-004, TAOS-ALG-AERO-001
- [x] **TAOS-ALG-GEO-002 — Downrange and Crossrange Projection Search**
  - Target: `taos_math.geodesy.downrange_crossrange`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-GEO-001, TAOS-ALG-GEO-003, TAOS-ALG-SEARCH-001
- [x] **TAOS-ALG-IIP-001 — Initial Impact Point Ballistic Derivatives**
  - Target: `taos_math.iip.iip_derivatives`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-DYN-001, TAOS-ALG-ENV-004, TAOS-ALG-GRAV-004
- [x] **TAOS-ALG-IIP-003 — Initial Impact Point Propagation and Output**
  - Target: `taos_math.iip.initial_impact_point`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-IIP-001, TAOS-ALG-IIP-002, TAOS-ALG-COORD-011, TAOS-ALG-GEO-001
- [x] **TAOS-ALG-OUT-001 — On-Demand Output-Variable Dispatch**
  - Target: `taos_runtime.outputs.build_output_evaluation_plan`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-EXEC-009

## M5-guidance-search

- [x] **TAOS-ALG-GUID-003 — Parabolic Guidance Transition**
  - Target: `taos_math.guidance.parabolic_guidance_correction`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-GUID-004 — Cubic Guidance Transition**
  - Target: `taos_math.guidance.cubic_guidance_correction`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-GUID-005 — Multidimensional Newton-Raphson Control Solve**
  - Target: `taos_math.numeric.newton_system`
  - Priority / complexity: P0 / high
  - Dependencies: TAOS-ALG-OPT-004
- [x] **TAOS-ALG-GUID-001 — Guidance Rule Classification and Control-Set Selection**
  - Target: `taos_math.guidance.classify_guidance_rules`
  - Priority / complexity: P1 / high
  - Dependencies: none
- [x] **TAOS-ALG-GUID-002 — Iterative Guidance Loop**
  - Target: `taos_math.guidance.solve_guidance`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-GUID-001, TAOS-ALG-GUID-005, TAOS-ALG-FORCE-004, TAOS-ALG-FORCE-005, TAOS-ALG-DYN-011
- [x] **TAOS-ALG-GUID-006 — Predictive Intercept Guidance**
  - Target: `taos_math.guidance.predictive_intercept`
  - Priority / complexity: P2 / medium
  - Dependencies: TAOS-ALG-COORD-012
- [x] **TAOS-ALG-GUID-007 — Proportional Navigation Guidance**
  - Target: `taos_math.guidance.proportional_navigation`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-DYN-011, TAOS-ALG-GUID-005
- [x] **TAOS-ALG-GUID-009 — Flight-Path Limit Arbitration**
  - Target: `taos_math.guidance.apply_flight_path_limits`
  - Priority / complexity: P2 / high
  - Dependencies: none
- [x] **TAOS-ALG-GUID-008 — Range-Insensitive Axis Search**
  - Target: `taos_math.guidance.range_insensitive_axis`
  - Priority / complexity: P3 / very_high
  - Dependencies: TAOS-ALG-IIP-003, TAOS-ALG-GUID-005

## Successor-data-model backlog

These items are research and digestion work rather than historical TAOS
algorithms. They should be tracked separately from the reviewed catalog until
their data shapes and provenance rules are stable.

- Air-breathing propulsion deck model
  - Define a shared propulsion-output interface that can represent rockets and
    air-breathing engines without collapsing them into the same table shape.
  - Specify installed thrust, fuel flow, engine-state dynamics, and operating
    envelope handling.
  - Preserve provenance for steady-state decks, generated decks, and measured
    data.
- Control-surface effectors
  - Define incremental coefficient decks for elevator, aileron, rudder, flaps,
    slats, spoilers, speed brakes, and fin mixers.
  - Support both linear derivatives near trim and nonlinear increment tables
    for large deflections or configuration changes.
  - Decide how to encode actuator limits, rate limits, and configuration
    transit times.
- Vehicle manifest bundling
  - Determine whether geometry, mass properties, propulsion, and effectors
    should be shipped as a single manifest or as a directory of linked files.
  - Keep synthetic reference data, generated decks, and measured datasets
    distinguishable at import time.

## M6-execution-engine

- [x] **TAOS-ALG-EXEC-002 — Vehicle, Segment, and Output Runtime Structure Construction**
  - Target: `taos_runtime.runtime_model.build_runtime_problem`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-PRB-003, TAOS-ALG-TABLE-001
- [x] **TAOS-ALG-EXEC-003 — Synchronized Multi-Vehicle Trajectory Loop**
  - Target: `taos_runtime.engine.compute_trajectories`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-EXEC-004, TAOS-ALG-EXEC-005, TAOS-ALG-EXEC-006, TAOS-ALG-EXEC-007, TAOS-ALG-EXEC-008
- [x] **TAOS-ALG-EXEC-004 — Common Next-Time-Step Selection**
  - Target: `taos_runtime.engine.get_next_time_step`
  - Priority / complexity: P1 / medium
  - Dependencies: none
- [x] **TAOS-ALG-EXEC-005 — Integrate All Active Vehicles One Step**
  - Target: `taos_runtime.engine.integrate_active_vehicles`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-DYN-003, TAOS-ALG-EXEC-009
- [x] **TAOS-ALG-EXEC-006 — Segment Final-Condition Detection and Refinement**
  - Target: `taos_runtime.events.refine_segment_final_condition`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-SEARCH-002, TAOS-ALG-EXEC-005, TAOS-ALG-PRB-005
- [x] **TAOS-ALG-EXEC-009 — Complete Derivative Calculation Pipeline**
  - Target: `taos_runtime.engine.compute_derivatives`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-011, TAOS-ALG-COORD-017, TAOS-ALG-DYN-002, TAOS-ALG-ENV-004, TAOS-ALG-FORCE-004, TAOS-ALG-FORCE-005, TAOS-ALG-GRAV-004, TAOS-ALG-GUID-002, TAOS-ALG-DYN-004
- [x] **TAOS-ALG-PRB-003 — Trajectory Initial-Condition Resolution**
  - Target: `taos_runtime.initialization.resolve_initial_state`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-COORD-001, TAOS-ALG-COORD-003, TAOS-ALG-COORD-006, TAOS-ALG-COORD-010, TAOS-ALG-COORD-013
- [x] **TAOS-ALG-PRB-005 — *When Final-Condition Evaluation and Action**
  - Target: `taos_runtime.events.evaluate_when_conditions`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-PRB-001
- [x] **TAOS-ALG-EXEC-001 — Main Program Table/Problem/File Loop**
  - Target: `taos_runtime.engine.run_taos`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-TABLE-001, TAOS-ALG-PRB-007, TAOS-ALG-EXEC-003
- [x] **TAOS-ALG-EXEC-007 — Search and Optimization Endpoint Dispatch with Partial Restart**
  - Target: `taos_runtime.engine.dispatch_search_and_restart`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-PRB-008, TAOS-ALG-OPT-001, TAOS-ALG-SEARCH-003
- [x] **TAOS-ALG-EXEC-008 — Dependent Vehicle Activation**
  - Target: `taos_runtime.engine.activate_dependent_vehicles`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-PRB-003, TAOS-ALG-PRB-004
- [x] **TAOS-ALG-PRB-004 — Increment and Reset State Discontinuities**
  - Target: `taos_runtime.events.apply_state_discontinuity`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-COORD-020, TAOS-ALG-PRB-001

## M7-table-and-problem-runtime

- [x] **TAOS-ALG-PRB-001 — User-Defined Expression Evaluation**
  - Target: `taos_runtime.expressions.evaluate_definition_program`
  - Priority / complexity: P0 / very_high
  - Dependencies: TAOS-ALG-TABLE-009
- [x] **TAOS-ALG-TABLE-001 — Table Definition Validation and Storage Layout**
  - Target: `taos_runtime.tables.prepare_table`
  - Priority / complexity: P0 / high
  - Dependencies: none
- [x] **TAOS-ALG-TABLE-003 — Full-Table Accumulator Evaluation**
  - Target: `taos_runtime.tables.evaluate_full_table`
  - Priority / complexity: P0 / very_high
  - Dependencies: TAOS-ALG-TABLE-004, TAOS-ALG-TABLE-005, TAOS-ALG-TABLE-007
- [x] **TAOS-ALG-TABLE-004 — Full-Table Operand Resolution**
  - Target: `taos_runtime.tables.resolve_table_operand`
  - Priority / complexity: P0 / medium
  - Dependencies: TAOS-ALG-TABLE-002, TAOS-ALG-TABLE-008
- [x] **TAOS-ALG-TABLE-005 — Full-Table Math-Operation Dispatch**
  - Target: `taos_runtime.tables.apply_table_operation`
  - Priority / complexity: P0 / medium
  - Dependencies: none
- [x] **TAOS-ALG-PRB-002 — User-Defined Integral Variable Runtime**
  - Target: `taos_runtime.expressions.integral_variable_derivatives`
  - Priority / complexity: P1 / high
  - Dependencies: TAOS-ALG-PRB-001, TAOS-ALG-DYN-002
- [x] **TAOS-ALG-PRB-006 — *Fly Guidance Rule and Table Resolution**
  - Target: `taos_runtime.guidance.resolve_fly_rules`
  - Priority / complexity: P1 / very_high
  - Dependencies: TAOS-ALG-GUID-001, TAOS-ALG-TABLE-009
- [x] **TAOS-ALG-TABLE-006 — Full-Table Storage Variable Semantics**
  - Target: `taos_runtime.tables.clear_and_store`
  - Priority / complexity: P1 / low
  - Dependencies: none
  - Duplicate storage names are rejected deterministically; the parser already diagnoses them before runtime.
- [x] **TAOS-ALG-TABLE-007 — Full-Table If/Label/Goto Control Flow**
  - Target: `taos_runtime.tables.execute_table_control_flow`
  - Priority / complexity: P1 / high
  - Dependencies: none
- [x] **TAOS-ALG-TABLE-008 — Skewed Tabulated-Data Evaluation**
  - Target: `taos_runtime.tables.interpolate_skewed`
  - Priority / complexity: P1 / very_high
  - Dependencies: none
  - Resolve first: The manual describes data organization but not a unique interpolation order for all possible sparse outer grids; document the chosen recursive semantics.
- [x] **TAOS-ALG-TABLE-009 — Multiple Table Reference Evaluation and Accumulation**
  - Target: `taos_runtime.tables.evaluate_table_references`
  - Priority / complexity: P1 / medium
  - Dependencies: TAOS-ALG-TABLE-003
- [x] **TAOS-ALG-PRB-007 — Survey Value Generation and Cartesian Nesting**
  - Target: `taos_runtime.surveys.generate_survey_cases`
  - Priority / complexity: P2 / medium
  - Dependencies: none
- [x] **TAOS-ALG-PRB-008 — Search Loop Construction, Nesting, and Execution**
  - Target: `taos_runtime.search_runtime.execute_search_loops`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-SEARCH-003
- [x] **TAOS-ALG-PRB-009 — Survey Summary Variable Aggregation**
  - Target: `taos_runtime.summaries.evaluate_summary`
  - Priority / complexity: P2 / high
  - Dependencies: TAOS-ALG-PRB-001, TAOS-ALG-PRB-007

## M8-optimization

- [x] **TAOS-ALG-OPT-001 — TAOS Nonlinear Programming Problem Construction**
  - Target: `taos_math.optimization.build_optimization_problem`
  - Priority / complexity: P2 / high
  - Dependencies: none
- [x] **TAOS-ALG-OPT-003 — Path-Integrated Optimization Constraint Variables**
  - Target: `taos_math.optimization.path_violation_integral`
  - Priority / complexity: P2 / medium
  - Dependencies: TAOS-ALG-DYN-002, TAOS-ALG-PRB-002
- [x] **TAOS-ALG-PRB-010 — *Optimize Block Runtime Mapping**
  - Target: `taos_runtime.optimization_runtime.resolve_optimize_block`
  - Priority / complexity: P2 / very_high
  - Dependencies: TAOS-ALG-PRB-001, TAOS-ALG-OPT-001
- [x] **TAOS-ALG-OPT-002 — Han-Powell Recursive Quadratic Programming**
  - Target: `taos_math.optimization.han_powell_rqp`
  - Priority / complexity: P3 / very_high
  - Dependencies: TAOS-ALG-OPT-001, TAOS-ALG-OPT-004
  - Resolve first: The manual gives an overview but not the full vf02ad source algorithm; obtain or independently reimplement the referenced method.
- [x] **TAOS-ALG-OPT-005 — Trajectory-Shaping Time-Grid Redistribution**
  - Target: `taos_math.optimization.redistribute_control_history`
  - Priority / complexity: P3 / high
  - Dependencies: TAOS-ALG-TABLE-002, TAOS-ALG-OPT-002
