"""Phase-aware GHAME6 atmospheric-to-exo vehicle, SAT3, and RADAR0 reconstruction."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from taoryx.sensor_api import MeasurementPacket, packet_to_record
from taoryx.sensor_plugins.relative_state import RelativeStateTrackerConfig, relative_state_track_from_geometry

from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .deck import CadacDeck
from .events import CadacEventApplication, CadacEventCursor
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .rocket6g import (
    AGRAV,
    DEG_PER_RAD,
    GM,
    RAD_PER_DEG,
    RGAS,
    WEII3,
    _cad_geo84_in,
    _cad_grav84_geocentric,
    _cad_in_geo84,
    _cad_tdi84,
    _cad_tgi84,
    _dcm_to_quaternion,
    _euler_from_dcm,
    _integrate_array,
    _integrate_scalar,
    _mat3tr,
    _polar_angles,
    _skew,
    _wrap_pi,
)
from .rocket6g_rcs import (
    Rocket6gRcsConfig,
    Rocket6gRcsRuntimeInput,
    Rocket6gRcsState,
    Rocket6gRcsStep,
    rocket6g_rcs_step,
)
from .source_environment import CADAC_SOURCE_EARTH_RADIUS_M, atmosphere76

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

_GHAME6_RADAR_SENSOR_ID = "ghame6-radar0-native-relative-state"
_GHAME6_RADAR_TARGET_ID = "ghame6-satellite-1"
_GHAME6_RADAR_SENSOR_FRAME_ID = "cadac.ghame6.radar0.eci"

_GHAME6_MODULES = (
    "kinematics",
    "environment",
    "aerodynamics",
    "propulsion",
    "gps",
    "startrack",
    "ins",
    "datalink",
    "seeker",
    "guidance",
    "control",
    "actuator",
    "rcs",
    "forces",
    "newton",
    "euler",
    "intercept",
)
_GHAME6_REQUIRED_MODULES = (
    "kinematics",
    "environment",
    "aerodynamics",
    "propulsion",
    "actuator",
    "rcs",
    "forces",
    "newton",
    "euler",
)
_GHAME6_AERO_TABLES = (
    "cd0_vs_alpha_mach",
    "cda_vs_alpha_mach",
    "cl0_vs_alpha_mach",
    "cla_vs_alpha_mach",
    "clde_vs_alpha_mach",
    "cyb_vs_alpha_mach",
    "cyda_vs_alpha_mach",
    "cydr_vs_alpha_mach",
    "cllb_vs_alpha_mach",
    "cllda_vs_alpha_mach",
    "clldr_vs_alpha_mach",
    "cllp_vs_alpha_mach",
    "cllr_vs_alpha_mach",
    "cm0_vs_alpha_mach",
    "cma_vs_alpha_mach",
    "cmde_vs_alpha_mach",
    "cmq_vs_alpha_mach",
    "clnb_vs_alpha_mach",
    "clnda_vs_alpha_mach",
    "clndr_vs_alpha_mach",
    "clnp_vs_alpha_mach",
    "clnr_vs_alpha_mach",
)
_GHAME6_PROP_TABLES = ("spi_vs_throttle_mach", "ca_vs_alpha_mach")

_NASA_ALTITUDE_KM = (
    0.0,
    11.019,
    20.063,
    32.162,
    47.35,
    51.413,
    71.802,
    86.0,
    91.0,
    94.0,
    97.0,
    100.0,
    103.0,
    106.0,
    108.0,
    110.0,
    112.0,
    115.0,
    120.0,
    125.0,
    130.0,
    135.0,
    140.0,
    145.0,
    150.0,
    155.0,
    160.0,
    165.0,
    170.0,
    180.0,
    190.0,
    210.0,
    230.0,
    265.0,
    300.0,
    350.0,
    400.0,
    450.0,
    500.0,
    550.0,
    600.0,
    650.0,
    700.0,
    750.0,
    800.0,
    850.0,
    900.0,
    950.0,
    1000.0,
)
_NASA_TEMPERATURE_K = (
    288.15,
    216.65,
    216.65,
    228.65,
    270.65,
    270.65,
    214.65,
    186.95,
    186.87,
    187.74,
    190.40,
    195.08,
    202.23,
    212.89,
    223.29,
    240.00,
    264.00,
    300.00,
    360.00,
    417.23,
    469.27,
    516.59,
    559.63,
    598.78,
    634.39,
    666.80,
    696.29,
    723.13,
    747.57,
    790.07,
    825.31,
    878.84,
    915.78,
    955.20,
    976.01,
    990.06,
    995.83,
    998.22,
    999.24,
    999.67,
    999.85,
    999.93,
    999.97,
    999.99,
    999.99,
    1000.0,
    1000.0,
    1000.0,
    1000.0,
)
_NASA_MOLECULAR_WEIGHT = (
    28.9644,
    28.9644,
    28.9644,
    28.9644,
    28.9644,
    28.9644,
    28.9644,
    28.9522,
    28.8890,
    28.7830,
    28.6200,
    28.3950,
    28.1040,
    27.7650,
    27.5210,
    27.2680,
    27.0200,
    26.6800,
    26.2050,
    25.8030,
    25.4360,
    25.0870,
    24.7490,
    24.4220,
    24.1030,
    23.7920,
    23.4880,
    23.1920,
    22.9020,
    22.3420,
    21.8090,
    20.8250,
    19.9520,
    18.6880,
    17.7260,
    16.7350,
    15.9840,
    15.2470,
    14.3300,
    13.0920,
    11.5050,
    9.7180,
    7.9980,
    6.5790,
    5.5430,
    4.8490,
    4.4040,
    4.1220,
    3.9400,
)
_NASA_PRESSURE_HPA = (
    1013.25,
    226.32,
    54.7487,
    8.68014,
    1.10905,
    0.66938,
    0.039564,
    3.7338e-03,
    1.5381e-03,
    9.0560e-04,
    5.3571e-04,
    3.2011e-04,
    1.9742e-04,
    1.2454e-04,
    9.3188e-05,
    7.1042e-05,
    5.5547e-05,
    4.0096e-05,
    2.5382e-05,
    1.7354e-05,
    1.2505e-05,
    9.3568e-06,
    7.2028e-06,
    5.6691e-06,
    4.5422e-06,
    3.6930e-06,
    3.0395e-06,
    2.5278e-06,
    2.1210e-06,
    1.5271e-06,
    1.1266e-06,
    6.4756e-07,
    3.9276e-07,
    1.7874e-07,
    8.7704e-08,
    3.4498e-08,
    1.4518e-08,
    6.4468e-09,
    3.0236e-09,
    1.5137e-09,
    8.2130e-10,
    4.8865e-10,
    3.1908e-10,
    2.2599e-10,
    1.7036e-10,
    1.3415e-10,
    1.0873e-10,
    8.9816e-11,
    7.5138e-11,
)


def ghame6_atmosphere(atmosphere_mode: int, altitude_m: float) -> tuple[float, float, float, float]:
    """Return the selected source atmosphere as density, pressure, temperature, and sound speed."""

    if not math.isfinite(altitude_m):
        raise ValueError("GHAME6 atmosphere altitude must be finite")
    ####
    if atmosphere_mode == 0:
        if altitude_m >= 84_852.0:
            temperature = 186.946
            return 0.0, 0.0, temperature, math.sqrt(1.4 * RGAS * temperature)
        ####
        density, pressure, temperature = atmosphere76(altitude_m)
        return density, pressure, temperature, math.sqrt(1.4 * RGAS * temperature)
    ####
    if atmosphere_mode != 100:
        raise ValueError(f"unsupported GHAME6 atmosphere mode {atmosphere_mode}")
    ####
    altitude_km = altitude_m / 1000.0
    if not 0.0 <= altitude_km <= 1000.0:
        raise ValueError("GHAME6 NASA atmosphere requires altitude within [0, 1000] km")
    ####
    lower = 0
    upper = len(_NASA_ALTITUDE_KM) - 1
    while upper - lower > 1:
        middle = (lower + upper) // 2
        if altitude_km > _NASA_ALTITUDE_KM[middle]:
            lower = middle
        else:
            upper = middle
        ####
    ####
    index = lower
    earth_radius_km = 6356.766
    nominal_gravity = 9.80665
    reference_molecular_weight = 28.9644
    universal_gas_constant = 8314.32
    if index < 7:
        lower_geopotential = earth_radius_km * _NASA_ALTITUDE_KM[index] / (earth_radius_km + _NASA_ALTITUDE_KM[index])
        upper_geopotential = earth_radius_km * _NASA_ALTITUDE_KM[index + 1] / (earth_radius_km + _NASA_ALTITUDE_KM[index + 1])
        geopotential = earth_radius_km * altitude_km / (earth_radius_km + altitude_km)
        gradient = (_NASA_TEMPERATURE_K[index + 1] - _NASA_TEMPERATURE_K[index]) / (upper_geopotential - lower_geopotential)
        if gradient != 0.0:
            pressure = (
                _NASA_PRESSURE_HPA[index]
                * (_NASA_TEMPERATURE_K[index] / (_NASA_TEMPERATURE_K[index] + gradient * (geopotential - lower_geopotential)))
                ** ((nominal_gravity * reference_molecular_weight) / (universal_gas_constant * gradient * 0.001))
                * 100.0
            )
        else:
            pressure = (
                _NASA_PRESSURE_HPA[index]
                * math.exp(
                    -(nominal_gravity * reference_molecular_weight * (geopotential * 1000.0 - lower_geopotential * 1000.0))
                    / (universal_gas_constant * _NASA_TEMPERATURE_K[index])
                )
                * 100.0
            )
        ####
        temperature = _NASA_TEMPERATURE_K[index] + gradient * (geopotential - lower_geopotential)
        molecular_weight = reference_molecular_weight
    else:
        if index == 7:
            temperature = _NASA_TEMPERATURE_K[8]
        elif 8 <= index < 15:
            temperature = 263.1905 - 76.3232 * math.sqrt(1.0 - ((altitude_km - 91.0) / 19.9429) ** 2)
        elif 15 <= index < 18:
            temperature = 240.0 + 12.0 * (altitude_km - 110.0)
        else:
            xi = (altitude_km - 120.0) * (earth_radius_km + 120.0) / (earth_radius_km + altitude_km)
            temperature = 1000.0 - 640.0 * math.exp(-0.01875 * xi)
        ####
        right_index = index - 1 if index == 47 else index
        molecular_weight_right = _quadratic_interpolate(
            altitude_km,
            _NASA_ALTITUDE_KM[right_index : right_index + 3],
            _NASA_MOLECULAR_WEIGHT[right_index : right_index + 3],
        )
        log_pressure_right = _quadratic_interpolate(
            altitude_km,
            _NASA_ALTITUDE_KM[right_index : right_index + 3],
            tuple(math.log(value) for value in _NASA_PRESSURE_HPA[right_index : right_index + 3]),
        )
        molecular_weight_left = molecular_weight_right
        log_pressure_left = log_pressure_right
        if index not in {7, 47}:
            left_index = right_index - 1
            molecular_weight_left = _quadratic_interpolate(
                altitude_km,
                _NASA_ALTITUDE_KM[left_index : left_index + 3],
                _NASA_MOLECULAR_WEIGHT[left_index : left_index + 3],
            )
            log_pressure_left = _quadratic_interpolate(
                altitude_km,
                _NASA_ALTITUDE_KM[left_index : left_index + 3],
                tuple(math.log(value) for value in _NASA_PRESSURE_HPA[left_index : left_index + 3]),
            )
        ####
        pressure = 100.0 * math.exp((log_pressure_right + log_pressure_left) / 2.0)
        molecular_weight = (molecular_weight_right + molecular_weight_left) / 2.0
    ####
    density = molecular_weight * pressure / (universal_gas_constant * temperature)
    speed_of_sound = math.sqrt(1.4 * pressure / density)
    return density, pressure, temperature, speed_of_sound


####


def _quadratic_interpolate(
    value: float,
    points: tuple[float, ...],
    samples: tuple[float, ...],
) -> float:
    if len(points) != 3 or len(samples) != 3:
        raise ValueError("quadratic interpolation requires exactly three points")
    ####
    result = 0.0
    for index in range(3):
        basis = 1.0
        for other in range(3):
            if other != index:
                basis *= (value - points[other]) / (points[index] - points[other])
            ####
        ####
        result += samples[index] * basis
    ####
    return result


####


class Ghame6SourceError(ValueError):
    """Raised when a CADAC case cannot lower to the bounded GHAME6 runtime."""


####


class Ghame6InitialState(CadacModel):
    """Source launch truth after optional automated orbital mission planning."""

    initialization_mode: int
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    geographic_speed_mps: float = Field(gt=0.0)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Ghame6SurfaceActuatorConfig(CadacModel):
    """Three physical atmospheric surfaces: left elevon, right elevon, and rudder."""

    mode: int
    position_limit_deg: float = Field(gt=0.0)
    rate_limit_deg_s: float = Field(gt=0.0)
    natural_frequency_rad_s: float = Field(gt=0.0)
    damping_ratio: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Ghame6SurfaceActuatorConfig":
        if self.mode not in {0, 2}:
            raise ValueError("GHAME6 surface actuator mode must be 0 or 2")
        ####
        return self

    ####


####


class Ghame6SatelliteConfig(CadacModel):
    """SAT3 orbital-element initialization and optional along-track thrust."""

    initialization_mode: int
    semi_major_axis_m: float = Field(gt=0.0)
    eccentricity: float = Field(ge=0.0, lt=1.0)
    inclination_deg: float
    longitude_ascending_node_deg: float
    argument_periapsis_deg: float
    true_anomaly_deg: float
    thrust_n: float = 0.0
    mass_kg: float = Field(default=100.0, gt=0.0)


####


class Ghame6RadarConfig(CadacModel):
    """RADAR0 fixed-Earth site and source measurement cadence."""

    enabled: bool
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    track_step_s: float = Field(ge=0.0)
    range_sigma_m: float = Field(default=0.0, ge=0.0)
    azimuth_sigma_rad: float = Field(default=0.0, ge=0.0)
    elevation_sigma_rad: float = Field(default=0.0, ge=0.0)
    velocity_sigma_mps: float = Field(default=0.0, ge=0.0)


####


class Ghame6SourceDefinition(CadacModel):
    """Prepared three-actor GHAME6 source case."""

    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    actor_order: tuple[str, ...]
    initial_state: Ghame6InitialState
    surface_actuator: Ghame6SurfaceActuatorConfig
    satellite: Ghame6SatelliteConfig
    radar: Ghame6RadarConfig
    initial_parameters: dict[str, int | float]
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = Field(min_length=5)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "rigid_body_6dof_surface_allocated"
    runtime_fidelity: str = "phase_reported_rigid_body_6dof"
    control_realization: str = "phase_transition_effector_to_direct_wrench"
    claim_boundary: str = (
        "WGS84 rigid-body truth, atmospheric left/right elevons and rudder, hypersonic and rocket propulsion, "
        "source event transitions, aggregate RCS, SAT3 truth, and RADAR0 tracking participate. Full arc/LTG/glideslope "
        "command generation, GPS/INS/star-tracker estimation, complete RF seeker/EKF physics, stochastic C-rand parity, "
        "discarded carrier propagation, and compiled-CADAC numerical parity remain outside this realization."
    )

    @model_validator(mode="after")
    def validate_decks(self) -> "Ghame6SourceDefinition":
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        missing_aero = tuple(name for name in _GHAME6_AERO_TABLES if name.casefold() not in aero_names)
        if missing_aero:
            raise ValueError("GHAME6 source definition is missing aerodynamic tables: " + ", ".join(missing_aero))
        ####
        prop_names = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing_prop = tuple(name for name in _GHAME6_PROP_TABLES if name.casefold() not in prop_names)
        if missing_prop:
            raise ValueError("GHAME6 source definition is missing propulsion tables: " + ", ".join(missing_prop))
        ####
        if self.actor_order != ("HYPER6", "SAT3", "RADAR0"):
            raise ValueError("GHAME6 source actor order must remain HYPER6, SAT3, RADAR0")
        ####
        return self

    ####


####


class Ghame6DirectCommand(CadacModel):
    """Explicit boundary replacing source guidance/control generation not yet reconstructed."""

    aileron_command_deg: float = 0.0
    elevator_command_deg: float = 0.0
    rudder_command_deg: float = 0.0
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    roll_command_deg: float | None = None
    pitch_command_deg: float | None = None
    yaw_command_deg: float | None = None
    alpha_command_deg: float = 0.0
    beta_command_deg: float = 0.0
    lateral_acceleration_command_g: float = 0.0
    normal_acceleration_command_g: float = 0.0
    boost_cutoff_time_s: float | None = Field(default=None, gt=0.0)
    terminal_lock_time_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_command(self) -> "Ghame6DirectCommand":
        values: tuple[float, ...] = (
            self.aileron_command_deg,
            self.elevator_command_deg,
            self.rudder_command_deg,
            *self.thrust_vector_unit_body,
            self.alpha_command_deg,
            self.beta_command_deg,
            self.lateral_acceleration_command_g,
            self.normal_acceleration_command_g,
            *(
                float(value)
                for value in (self.roll_command_deg, self.pitch_command_deg, self.yaw_command_deg, self.boost_cutoff_time_s, self.terminal_lock_time_s)
                if value is not None
            ),
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("GHAME6 direct commands must be finite")
        ####
        magnitude = math.sqrt(sum(value * value for value in self.thrust_vector_unit_body))
        if magnitude <= 0.0:
            raise ValueError("GHAME6 thrust-vector command must have positive magnitude")
        ####
        return self

    ####

    def normalized_thrust_vector(self) -> tuple[float, float, float]:
        magnitude = math.sqrt(sum(float(value) * float(value) for value in self.thrust_vector_unit_body))
        return (
            float(self.thrust_vector_unit_body[0]) / magnitude,
            float(self.thrust_vector_unit_body[1]) / magnitude,
            float(self.thrust_vector_unit_body[2]) / magnitude,
        )

    ####


####


class Ghame6SurfaceState(CadacModel):
    """Stored-derivative states for left elevon, right elevon, and rudder."""

    position_derivative_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_derivative_deg_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Ghame6SurfaceStep(CadacModel):
    """Requested and achieved physical-surface realization for one module pass."""

    active: bool
    requested_control_deg: tuple[float, float, float]
    requested_surfaces_deg: tuple[float, float, float]
    achieved_surfaces_deg: tuple[float, float, float]
    achieved_control_deg: tuple[float, float, float]
    position_limited: tuple[bool, bool, bool]
    rate_limited: tuple[bool, bool, bool]
    state: Ghame6SurfaceState


####


class Ghame6PropulsionState(CadacModel):
    """Participating source fuel, mass, and inertia state."""

    fuel_expended_derivative_kg_s: float = 0.0
    fuel_expended_kg: float = Field(default=0.0, ge=0.0)
    mass_kg: float = Field(gt=0.0)
    inertia_body_kgm2: tuple[float, float, float, float, float, float, float, float, float]
    throttle: float = 0.0
    specific_impulse_s: float = 0.0


####


class Ghame6AeroCoefficients(CadacModel):
    """Stability/body-axis GHAME6 force and moment coefficients."""

    reference_area_m2: float = Field(gt=0.0)
    drag_coefficient: float = 0.0
    lift_coefficient: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0
    cl: float = 0.0
    cm: float = 0.0
    cn: float = 0.0


####


class Ghame6BodyWrench(CadacModel):
    """Total non-gravitational body force and moment."""

    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Ghame6PhaseEvent(CadacModel):
    """One applied source event with before/after fidelity evidence."""

    event_index: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    source_line: int = Field(ge=1)
    watch_variable: str
    operator: str
    criterion: int | float
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]
    phase_before: str
    phase_after: str
    runtime_fidelity_before: str
    runtime_fidelity_after: str


####


class Ghame6RadarTrack(CadacModel):
    """One RADAR0 source track plus its native raw geometry projection.

    ``native_relative_state_packet`` is deliberately separate from the
    source-shaped polar/noise track fields.  It is the standard Taoryx raw
    measurement of the committed RADAR0/SAT3 geometry, not a replacement for
    CADAC's own radar corruption or track-file semantics.
    """

    time_s: float = Field(ge=0.0)
    measured_position_inertial_m: tuple[float, float, float]
    measured_velocity_inertial_mps: tuple[float, float, float]
    true_range_m: float = Field(ge=0.0)
    update_sequence: int = Field(ge=1)
    native_relative_state_packet: dict[str, Any]


####


class Ghame6HyperSample(CadacModel):
    """One accepted HYPER6 truth sample with phase-specific fidelity."""

    time_s: float = Field(ge=0.0)
    position_inertial_m: tuple[float, float, float]
    velocity_inertial_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_inertial_rad_s: tuple[float, float, float]
    body_rates_earth_rad_s: tuple[float, float, float]
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    geographic_speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    alpha_deg: float
    beta_deg: float
    mach: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    source_phase: str
    runtime_fidelity: str
    control_realization: str
    aerodynamic_mode: int
    propulsion_mode: int
    seeker_mode: int
    rcs_moment_mode: int
    rcs_force_mode: int
    mass_kg: float = Field(gt=0.0)
    remaining_fuel_kg: float
    inertia_diagonal_kgm2: tuple[float, float, float]
    inertia_xz_kgm2: float
    throttle: float
    thrust_n: float
    requested_control_deg: tuple[float, float, float]
    requested_surfaces_deg: tuple[float, float, float]
    achieved_surfaces_deg: tuple[float, float, float]
    achieved_control_deg: tuple[float, float, float]
    requested_rcs_attitude_deg: tuple[float, float, float]
    requested_rcs_incidence_deg: tuple[float, float]
    requested_rcs_acceleration_g: tuple[float, float]
    requested_thrust_vector_unit_body: tuple[float, float, float]
    rcs_force_body_n: tuple[float, float, float]
    rcs_moment_body_nm: tuple[float, float, float]
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Ghame6SatelliteSample(CadacModel):
    """One independently propagated SAT3 truth sample."""

    time_s: float = Field(ge=0.0)
    position_inertial_m: tuple[float, float, float]
    velocity_inertial_mps: tuple[float, float, float]
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    speed_mps: float = Field(ge=0.0)


####


class Ghame6RadarSiteSample(CadacModel):
    """One RADAR0 fixed-Earth site state expressed in inertial coordinates."""

    time_s: float = Field(ge=0.0)
    position_inertial_m: tuple[float, float, float]
    velocity_inertial_mps: tuple[float, float, float]
    longitude_deg: float
    latitude_deg: float
    altitude_m: float


####


class Ghame6RunResult(CadacModel):
    """Phase-aware GHAME6 HYPER6/SAT3/RADAR0 result."""

    schema_id: str = "taoryx.cadac.ghame6-phase-aware/v0alpha1"
    source_name: str = Field(min_length=1)
    initial_integration_step_s: float = Field(gt=0.0)
    final_integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    events: tuple[Ghame6PhaseEvent, ...]
    radar_tracks: tuple[Ghame6RadarTrack, ...]
    radar_update_count: int = Field(ge=0)
    samples: tuple[Ghame6HyperSample, ...] = Field(min_length=1)
    satellite_samples: tuple[Ghame6SatelliteSample, ...] = Field(min_length=1)
    radar_samples: tuple[Ghame6RadarSiteSample, ...] = Field(min_length=1)
    claim_boundary: str = (
        "HYPER6 remains one persistent truth object whose atmospheric physical-surface T4 phase transitions to aggregate-RCS T3 phases. "
        "SAT3 is an independent T1 root object; RADAR0 owns measurement events. No TVC realization is claimed for GHAME6."
    )


####


@dataclass(slots=True)
class _Ghame6HyperRuntime:
    source_values: dict[str, int | float]
    event_cursor: CadacEventCursor
    event_time_s: float
    dt_s: float
    position_inertial_m: FloatVector
    velocity_inertial_mps: FloatVector
    acceleration_inertial_mps2: FloatVector
    body_from_inertial: FloatMatrix
    body_from_inertial_derivative: FloatMatrix
    body_rates_inertial_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    longitude_rad: float
    latitude_rad: float
    altitude_m: float
    geodetic_from_inertial: FloatMatrix
    geocentric_from_inertial: FloatMatrix
    velocity_geodetic_mps: FloatVector
    geographic_speed_mps: float
    heading_deg: float
    flight_path_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    alpha_deg: float
    beta_deg: float
    body_rates_earth_rad_s: FloatVector
    density_kg_m3: float
    pressure_pa: float
    temperature_k: float
    speed_of_sound_mps: float
    mach: float
    dynamic_pressure_pa: float
    gravity_inertial_mps2: FloatVector
    propulsion: Ghame6PropulsionState
    remaining_fuel_kg: float
    thrust_n: float
    surface_state: Ghame6SurfaceState
    surface_step: Ghame6SurfaceStep
    rcs_state: Rocket6gRcsState
    rcs_step: Rocket6gRcsStep
    aero_coefficients: Ghame6AeroCoefficients
    wrench: Ghame6BodyWrench
    specific_force_body_mps2: FloatVector
    seeker_acquisition_epoch_s: float | None


####


@dataclass(slots=True)
class _Ghame6SatelliteRuntime:
    position_inertial_m: FloatVector
    velocity_inertial_mps: FloatVector
    acceleration_inertial_mps2: FloatVector


####


@dataclass(slots=True)
class _Ghame6RadarRuntime:
    next_track_time_s: float
    update_count: int
    latest_track: Ghame6RadarTrack | None


####


def load_ghame6_source_definition(path: str | Path) -> Ghame6SourceDefinition:
    """Parse, fingerprint, and lower one GHAME6 source case."""

    return lower_ghame6_source_bundle(load_cadac_source_bundle(path))


####


def lower_ghame6_source_bundle(bundle: CadacSourceBundle) -> Ghame6SourceDefinition:
    """Lower the exact HYPER6, SAT3, RADAR0 source composition."""

    actor_order = tuple(vehicle.model_name.upper() for vehicle in bundle.case.vehicles)
    if actor_order != ("HYPER6", "SAT3", "RADAR0"):
        raise Ghame6SourceError(f"GHAME6 lowering requires HYPER6, SAT3, RADAR0 source order; found {actor_order!r}")
    ####
    hyper = bundle.case.vehicle("HYPER6")
    satellite = bundle.case.vehicle("SAT3")
    radar = bundle.case.vehicle("RADAR0")
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    if "tvc" in module_order:
        raise Ghame6SourceError("GHAME6 source must not be promoted to a TVC realization; the shipped mission has no tvc module")
    ####
    unknown = tuple(name for name in module_order if name not in _GHAME6_MODULES)
    if unknown:
        raise Ghame6SourceError(f"GHAME6 reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _GHAME6_REQUIRED_MODULES if name not in module_order)
    if missing:
        raise Ghame6SourceError(f"GHAME6 reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    integration_step_s = timing.get("int_step")
    if integration_step_s is None:
        raise Ghame6SourceError("GHAME6 source case must declare TIMING int_step")
    ####
    values = _initial_runtime_values(hyper)
    atmosphere_mode = _int_value(values, "mair", 0)
    if atmosphere_mode not in {0, 100}:
        raise Ghame6SourceError("the first GHAME6 runtime supports US76/NASA-atmosphere no-wind modes 0 and 100; weather/wind/turbulence remain excluded")
    ####
    satellite_config = Ghame6SatelliteConfig(
        initialization_mode=_integer(satellite, "minit", 1),
        semi_major_axis_m=_number(satellite, "semi"),
        eccentricity=_number(satellite, "ecc"),
        inclination_deg=_number(satellite, "inclx"),
        longitude_ascending_node_deg=_number(satellite, "lon_anodex"),
        argument_periapsis_deg=_number(satellite, "arg_perix"),
        true_anomaly_deg=_number(satellite, "true_anomx"),
        thrust_n=_number(satellite, "sat_thrust", 0.0),
        mass_kg=_number(satellite, "sat_mass", 100.0),
    )
    initial_state = _lower_initial_state(hyper, values)
    radar_config = Ghame6RadarConfig(
        enabled=bool(_integer(radar, "radar_on", 0)),
        longitude_deg=_number(radar, "lonx"),
        latitude_deg=_number(radar, "latx"),
        altitude_m=_number(radar, "alt"),
        track_step_s=_number(radar, "track_step", 0.0),
        range_sigma_m=_number(radar, "dat_sigma", 0.0),
        azimuth_sigma_rad=_number(radar, "azat_sigma", 0.0),
        elevation_sigma_rad=_number(radar, "elat_sigma", 0.0),
        velocity_sigma_mps=_number(radar, "vel_sigma", 0.0),
    )
    return Ghame6SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=float(integration_step_s),
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        actor_order=actor_order,
        initial_state=initial_state,
        surface_actuator=Ghame6SurfaceActuatorConfig(
            mode=_int_value(values, "mact", 0),
            position_limit_deg=_float_value(values, "dlimx"),
            rate_limit_deg_s=_float_value(values, "ddlimx"),
            natural_frequency_rad_s=_float_value(values, "wnact"),
            damping_ratio=_float_value(values, "zetact"),
        ),
        satellite=satellite_config,
        radar=radar_config,
        initial_parameters=values,
        aerodynamic_deck=bundle.deck_for("HYPER6", CadacDeckKind.AERODYNAMIC),
        propulsion_deck=bundle.deck_for("HYPER6", CadacDeckKind.PROPULSION),
        events=hyper.events,
        source_artifacts=bundle.artifacts,
    )


####


def ghame6_surface_step(
    config: Ghame6SurfaceActuatorConfig,
    state: Ghame6SurfaceState,
    command: Ghame6DirectCommand,
    *,
    dt_s: float,
    mode_override: int | None = None,
    enabled: bool = True,
) -> Ghame6SurfaceStep:
    """Execute source mixing and three independent physical actuator states."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("GHAME6 surface dt_s must be positive and finite")
    ####
    mode = config.mode if mode_override is None else int(mode_override)
    if mode not in {0, 2}:
        raise ValueError("GHAME6 surface actuator mode must be 0 or 2")
    ####
    requested_control = np.asarray(
        (command.aileron_command_deg, command.elevator_command_deg, command.rudder_command_deg),
        dtype=np.float64,
    )
    requested_surfaces = np.asarray(
        (
            command.elevator_command_deg + command.aileron_command_deg,
            command.elevator_command_deg - command.aileron_command_deg,
            command.rudder_command_deg,
        ),
        dtype=np.float64,
    )
    position_flags = np.zeros(3, dtype=np.bool_)
    rate_flags = np.zeros(3, dtype=np.bool_)
    if mode == 0:
        achieved = np.clip(requested_surfaces, -config.position_limit_deg, config.position_limit_deg)
        position_flags = np.abs(requested_surfaces) > config.position_limit_deg
        next_state = state
    else:
        position = np.asarray(state.position_deg, dtype=np.float64)
        position_derivative = np.asarray(state.position_derivative_deg_s, dtype=np.float64)
        rate = np.asarray(state.rate_deg_s, dtype=np.float64)
        rate_derivative = np.asarray(state.rate_derivative_deg_s2, dtype=np.float64)
        for index in range(3):
            if abs(position[index]) > config.position_limit_deg:
                position_flags[index] = True
                position[index] = math.copysign(config.position_limit_deg, position[index])
                if position[index] * rate[index] > 0.0:
                    rate[index] = 0.0
                ####
            ####
            if abs(rate[index]) > config.rate_limit_deg_s:
                rate_flags[index] = True
                rate[index] = math.copysign(config.rate_limit_deg_s, rate[index])
            ####
            derivative_new = rate[index]
            position[index] = _integrate_scalar(position[index], derivative_new, position_derivative[index], dt_s)
            position_derivative[index] = derivative_new
            error = requested_surfaces[index] - position[index]
            rate_derivative_new = (
                config.natural_frequency_rad_s * config.natural_frequency_rad_s * error
                - 2.0 * config.damping_ratio * config.natural_frequency_rad_s * position_derivative[index]
            )
            rate[index] = _integrate_scalar(rate[index], rate_derivative_new, rate_derivative[index], dt_s)
            rate_derivative[index] = rate_derivative_new
            if rate_flags[index] and rate[index] * rate_derivative[index] > 0.0:
                rate_derivative[index] = 0.0
            ####
        ####
        achieved = position
        next_state = Ghame6SurfaceState(
            position_derivative_deg_s=_tuple3(position_derivative),
            position_deg=_tuple3(position),
            rate_derivative_deg_s2=_tuple3(rate_derivative),
            rate_deg_s=_tuple3(rate),
        )
    ####
    left, right, rudder = (float(value) for value in achieved)
    achieved_control = ((left - right) / 2.0, (left + right) / 2.0, rudder)
    return Ghame6SurfaceStep(
        active=enabled,
        requested_control_deg=_tuple3(requested_control),
        requested_surfaces_deg=_tuple3(requested_surfaces),
        achieved_surfaces_deg=_tuple3(achieved),
        achieved_control_deg=achieved_control,
        position_limited=_tuple3_bool(position_flags),
        rate_limited=_tuple3_bool(rate_flags),
        state=next_state,
    )


####


def ghame6_aerodynamic_coefficients(
    definition: Ghame6SourceDefinition,
    *,
    aerodynamic_mode: int,
    mach: float,
    alpha_deg: float,
    beta_deg: float,
    airspeed_mps: float,
    body_rates_earth_rad_s: tuple[float, float, float],
    achieved_control_deg: tuple[float, float, float],
) -> Ghame6AeroCoefficients:
    """Evaluate the source GHAME or transfer-vehicle aerodynamic closure."""

    if aerodynamic_mode == 0:
        return Ghame6AeroCoefficients(reference_area_m2=557.42)
    ####
    if aerodynamic_mode == 2:
        return Ghame6AeroCoefficients(reference_area_m2=7.0, cx=-0.4)
    ####
    if aerodynamic_mode != 1:
        raise ValueError(f"unsupported GHAME6 aerodynamic mode {aerodynamic_mode}")
    ####
    if any(not math.isfinite(value) for value in (mach, alpha_deg, beta_deg, airspeed_mps, *body_rates_earth_rad_s, *achieved_control_deg)):
        raise ValueError("GHAME6 aerodynamic inputs must be finite")
    ####
    deck = definition.aerodynamic_deck
    query = (alpha_deg, mach)
    aileron, elevator, rudder = achieved_control_deg
    p_rate, q_rate, r_rate = body_rates_earth_rad_s
    speed = max(abs(airspeed_mps), 1.0e-9)
    cd0 = deck.table("cd0_vs_alpha_mach").interpolate(query)
    cda = deck.table("cda_vs_alpha_mach").interpolate(query)
    drag = cd0 + cda * alpha_deg
    cl0 = deck.table("cl0_vs_alpha_mach").interpolate(query)
    cla = deck.table("cla_vs_alpha_mach").interpolate(query)
    clde = deck.table("clde_vs_alpha_mach").interpolate(query)
    lift = cl0 + cla * alpha_deg + clde * elevator
    cy = (
        deck.table("cyb_vs_alpha_mach").interpolate(query) * beta_deg
        + deck.table("cyda_vs_alpha_mach").interpolate(query) * aileron
        + deck.table("cydr_vs_alpha_mach").interpolate(query) * rudder
    )
    ref_span = 24.38
    ref_chord = 22.86
    cl_roll = (
        deck.table("cllb_vs_alpha_mach").interpolate(query) * beta_deg
        + deck.table("cllda_vs_alpha_mach").interpolate(query) * aileron
        + deck.table("clldr_vs_alpha_mach").interpolate(query) * rudder
        + deck.table("cllp_vs_alpha_mach").interpolate(query) * p_rate * ref_span / (2.0 * speed)
        + deck.table("cllr_vs_alpha_mach").interpolate(query) * r_rate * ref_span / (2.0 * speed)
    )
    cm = (
        deck.table("cm0_vs_alpha_mach").interpolate(query)
        + deck.table("cma_vs_alpha_mach").interpolate(query) * alpha_deg
        + deck.table("cmde_vs_alpha_mach").interpolate(query) * elevator
        + deck.table("cmq_vs_alpha_mach").interpolate(query) * q_rate * ref_chord / (2.0 * speed)
    )
    cn = (
        deck.table("clnb_vs_alpha_mach").interpolate(query) * beta_deg
        + deck.table("clnda_vs_alpha_mach").interpolate(query) * aileron
        + deck.table("clndr_vs_alpha_mach").interpolate(query) * rudder
        + deck.table("clnp_vs_alpha_mach").interpolate(query) * p_rate * ref_span / (2.0 * speed)
        + deck.table("clnr_vs_alpha_mach").interpolate(query) * r_rate * ref_span / (2.0 * speed)
    )
    alpha = alpha_deg * RAD_PER_DEG
    cx = -drag * math.cos(alpha) + lift * math.sin(alpha)
    cz = -drag * math.sin(alpha) - lift * math.cos(alpha)
    return Ghame6AeroCoefficients(
        reference_area_m2=557.42,
        drag_coefficient=drag,
        lift_coefficient=lift,
        cx=cx,
        cy=cy,
        cz=cz,
        cl=cl_roll,
        cm=cm,
        cn=cn,
    )


####


def ghame6_body_wrench(
    coefficients: Ghame6AeroCoefficients,
    *,
    dynamic_pressure_pa: float,
    thrust_n: float,
    rcs: Rocket6gRcsStep,
) -> Ghame6BodyWrench:
    """Assemble aerodynamic, centerline propulsion, and aggregate-RCS source wrench."""

    q_area = dynamic_pressure_pa * coefficients.reference_area_m2
    force = np.asarray(
        (q_area * coefficients.cx + thrust_n, q_area * coefficients.cy, q_area * coefficients.cz),
        dtype=np.float64,
    )
    moment = np.asarray(
        (
            q_area * 24.38 * coefficients.cl,
            q_area * 22.86 * coefficients.cm,
            q_area * 24.38 * coefficients.cn,
        ),
        dtype=np.float64,
    )
    force += np.asarray(rcs.force_body_n, dtype=np.float64)
    moment += np.asarray(rcs.moment_body_nm, dtype=np.float64)
    return Ghame6BodyWrench(force_body_n=_tuple3(force), moment_body_nm=_tuple3(moment))


####


def run_ghame6_phase_aware_mission(
    definition: Ghame6SourceDefinition,
    command: Ghame6DirectCommand | None = None,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
    random_seed: int = 12345,
) -> Ghame6RunResult:
    """Run the source-ordered HYPER6, SAT3, RADAR0 composition."""

    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("GHAME6 end_time_s must be positive and finite")
    ####
    cadence = max(definition.integration_step_s, 0.05) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("GHAME6 sample_step_s must be positive and finite")
    ####
    resolved_command = command or Ghame6DirectCommand()
    hyper = _initialize_hyper_runtime(definition)
    satellite = _initialize_satellite_runtime(definition.satellite)
    radar = _Ghame6RadarRuntime(next_track_time_s=0.0, update_count=0, latest_track=None)
    rng = np.random.default_rng(random_seed)
    samples: list[Ghame6HyperSample] = []
    satellite_samples: list[Ghame6SatelliteSample] = []
    radar_samples: list[Ghame6RadarSiteSample] = []
    events: list[Ghame6PhaseEvent] = []
    radar_tracks: list[Ghame6RadarTrack] = []
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0
    terminated_reason = "end_time"
    while sim_time <= requested_end + 0.5 * hyper.dt_s:
        steps += 1
        application = _evaluate_source_event(hyper, resolved_command, sim_time)
        if application is not None:
            events.append(application)
        ####
        dt_s = hyper.dt_s
        for actor_name in definition.actor_order:
            if actor_name == "HYPER6":
                _run_hyper_modules(definition, hyper, satellite, resolved_command, sim_time, dt_s)
            elif actor_name == "SAT3":
                _runtime_satellite(definition.satellite, satellite, dt_s)
            elif actor_name == "RADAR0":
                track = _runtime_radar(definition.radar, radar, satellite, sim_time, dt_s, rng)
                if track is not None and (sim_time + 0.5 * dt_s >= next_sample_time or not radar_tracks):
                    radar_tracks.append(track)
                ####
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(_hyper_sample(hyper, resolved_command, sim_time))
            satellite_samples.append(_satellite_sample(satellite, sim_time))
            radar_samples.append(_radar_site_sample(definition.radar, sim_time))
            if radar.latest_track is not None and (not radar_tracks or radar_tracks[-1].update_sequence != radar.latest_track.update_sequence):
                radar_tracks.append(radar.latest_track)
            ####
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        if not _runtime_is_finite(hyper, satellite):
            terminated_reason = "nonfinite_state"
            break
        ####
        sim_time += dt_s
        hyper.event_time_s += dt_s
    ####
    return Ghame6RunResult(
        source_name=definition.source_name,
        initial_integration_step_s=definition.integration_step_s,
        final_integration_step_s=hyper.dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        events=tuple(events),
        radar_tracks=tuple(radar_tracks),
        radar_update_count=radar.update_count,
        samples=tuple(samples),
        satellite_samples=tuple(satellite_samples),
        radar_samples=tuple(radar_samples),
    )


####


def _run_hyper_modules(
    definition: Ghame6SourceDefinition,
    runtime: _Ghame6HyperRuntime,
    satellite: _Ghame6SatelliteRuntime,
    command: Ghame6DirectCommand,
    sim_time_s: float,
    dt_s: float,
) -> None:
    for module in definition.module_order:
        if module == "kinematics":
            _runtime_kinematics(runtime, dt_s, sim_time_s)
        elif module == "environment":
            _runtime_environment(runtime, sim_time_s)
        elif module == "aerodynamics":
            runtime.aero_coefficients = ghame6_aerodynamic_coefficients(
                definition,
                aerodynamic_mode=_int_value(runtime.source_values, "maero", 0),
                mach=runtime.mach,
                alpha_deg=runtime.alpha_deg,
                beta_deg=runtime.beta_deg,
                airspeed_mps=runtime.geographic_speed_mps,
                body_rates_earth_rad_s=_tuple3(runtime.body_rates_earth_rad_s),
                achieved_control_deg=runtime.surface_step.achieved_control_deg,
            )
        elif module == "propulsion":
            _runtime_propulsion(definition, runtime, dt_s)
        elif module in {"gps", "startrack", "ins", "datalink"}:
            pass
        elif module == "seeker":
            _runtime_seeker(runtime, satellite, command, sim_time_s)
        elif module in {"guidance", "control"}:
            pass
        elif module == "actuator":
            runtime.surface_step = ghame6_surface_step(
                definition.surface_actuator,
                runtime.surface_state,
                command,
                dt_s=dt_s,
                mode_override=_int_value(runtime.source_values, "mact", 0),
                enabled=_int_value(runtime.source_values, "maero", 0) == 1,
            )
            runtime.surface_state = runtime.surface_step.state
        elif module == "rcs":
            _runtime_rcs(runtime, command)
        elif module == "forces":
            runtime.wrench = ghame6_body_wrench(
                runtime.aero_coefficients,
                dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                thrust_n=runtime.thrust_n,
                rcs=runtime.rcs_step,
            )
        elif module == "newton":
            _runtime_newton(runtime, dt_s, sim_time_s)
        elif module == "euler":
            _runtime_euler(runtime, dt_s)
        elif module == "intercept":
            pass
        ####
    ####


####


def _initialize_hyper_runtime(definition: Ghame6SourceDefinition) -> _Ghame6HyperRuntime:
    initial = definition.initial_state
    values = dict(definition.initial_parameters)
    lon = initial.longitude_deg * RAD_PER_DEG
    lat = initial.latitude_deg * RAD_PER_DEG
    position = _cad_in_geo84(lon, lat, initial.altitude_m, 0.0)
    tdi = _cad_tdi84(lon, lat, initial.altitude_m, 0.0)
    tgi = _cad_tgi84(lon, lat, initial.altitude_m, 0.0)
    body_from_geodetic = _mat3tr(initial.yaw_deg * RAD_PER_DEG, initial.pitch_deg * RAD_PER_DEG, initial.roll_deg * RAD_PER_DEG)
    body_from_inertial = body_from_geodetic @ tdi
    alpha = initial.alpha_deg * RAD_PER_DEG
    beta = initial.beta_deg * RAD_PER_DEG
    body_velocity = np.asarray(
        (
            math.cos(alpha) * math.cos(beta) * initial.geographic_speed_mps,
            math.sin(beta) * initial.geographic_speed_mps,
            math.sin(alpha) * math.cos(beta) * initial.geographic_speed_mps,
        ),
        dtype=np.float64,
    )
    velocity_geodetic = body_from_geodetic.T @ body_velocity
    omega_skew = _skew(np.asarray((0.0, 0.0, WEII3), dtype=np.float64))
    velocity_inertial = tdi.T @ velocity_geodetic + omega_skew @ position
    body_rates_earth = np.asarray(initial.body_rates_deg_s, dtype=np.float64) * RAD_PER_DEG
    body_rates_inertial = body_rates_earth + body_from_inertial @ np.asarray((0.0, 0.0, WEII3), dtype=np.float64)
    inertia = np.asarray(
        ((1.573e6, 0.0, 0.38e6), (0.0, 31.6e6, 0.0), (0.38e6, 0.0, 32.54e6)),
        dtype=np.float64,
    )
    propulsion = Ghame6PropulsionState(
        mass_kg=_float_value(values, "vmass0"),
        inertia_body_kgm2=_tuple9(inertia),
        throttle=_float_value(values, "throttle", 0.05),
    )
    zero_surface = Ghame6SurfaceStep(
        active=False,
        requested_control_deg=(0.0, 0.0, 0.0),
        requested_surfaces_deg=(0.0, 0.0, 0.0),
        achieved_surfaces_deg=(0.0, 0.0, 0.0),
        achieved_control_deg=(0.0, 0.0, 0.0),
        position_limited=(False, False, False),
        rate_limited=(False, False, False),
        state=Ghame6SurfaceState(),
    )
    zero_rcs = Rocket6gRcsStep(
        force_body_n=(0.0, 0.0, 0.0),
        moment_body_nm=(0.0, 0.0, 0.0),
        errors=(0.0, 0.0, 0.0, 0.0, 0.0),
        state=Rocket6gRcsState(),
    )
    heading, flight_path = _polar_angles(velocity_geodetic)
    return _Ghame6HyperRuntime(
        source_values=values,
        event_cursor=CadacEventCursor.from_events(definition.events),
        event_time_s=0.0,
        dt_s=definition.integration_step_s,
        position_inertial_m=position,
        velocity_inertial_mps=velocity_inertial,
        acceleration_inertial_mps2=np.zeros(3, dtype=np.float64),
        body_from_inertial=body_from_inertial,
        body_from_inertial_derivative=np.zeros((3, 3), dtype=np.float64),
        body_rates_inertial_rad_s=body_rates_inertial,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        longitude_rad=lon,
        latitude_rad=lat,
        altitude_m=initial.altitude_m,
        geodetic_from_inertial=tdi,
        geocentric_from_inertial=tgi,
        velocity_geodetic_mps=velocity_geodetic,
        geographic_speed_mps=initial.geographic_speed_mps,
        heading_deg=heading,
        flight_path_deg=flight_path,
        roll_deg=initial.roll_deg,
        pitch_deg=initial.pitch_deg,
        yaw_deg=initial.yaw_deg,
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        body_rates_earth_rad_s=body_rates_earth,
        density_kg_m3=0.0,
        pressure_pa=101_325.0,
        temperature_k=288.15,
        speed_of_sound_mps=340.0,
        mach=0.0,
        dynamic_pressure_pa=0.0,
        gravity_inertial_mps2=np.zeros(3, dtype=np.float64),
        propulsion=propulsion,
        remaining_fuel_kg=_float_value(values, "fmass0"),
        thrust_n=0.0,
        surface_state=Ghame6SurfaceState(),
        surface_step=zero_surface,
        rcs_state=Rocket6gRcsState(),
        rcs_step=zero_rcs,
        aero_coefficients=Ghame6AeroCoefficients(reference_area_m2=557.42),
        wrench=Ghame6BodyWrench(force_body_n=(0.0, 0.0, 0.0), moment_body_nm=(0.0, 0.0, 0.0)),
        specific_force_body_mps2=np.zeros(3, dtype=np.float64),
        seeker_acquisition_epoch_s=None,
    )


####


def _initialize_satellite_runtime(config: Ghame6SatelliteConfig) -> _Ghame6SatelliteRuntime:
    if config.initialization_mode != 1:
        raise Ghame6SourceError("the first GHAME6 SAT3 runtime supports source orbital-element initialization mode 1")
    ####
    position, velocity = _orbital_state(
        config.semi_major_axis_m,
        config.eccentricity,
        config.inclination_deg,
        config.longitude_ascending_node_deg,
        config.argument_periapsis_deg,
        config.true_anomaly_deg,
    )
    return _Ghame6SatelliteRuntime(
        position_inertial_m=position,
        velocity_inertial_mps=velocity,
        acceleration_inertial_mps2=np.zeros(3, dtype=np.float64),
    )


####


def _evaluate_source_event(
    runtime: _Ghame6HyperRuntime,
    command: Ghame6DirectCommand,
    sim_time_s: float,
) -> Ghame6PhaseEvent | None:
    values = runtime.source_values
    values["time"] = sim_time_s
    values["event_time"] = runtime.event_time_s
    values["alt"] = runtime.altitude_m
    values["fmassr"] = runtime.remaining_fuel_kg
    values["thrust"] = runtime.thrust_n
    if command.boost_cutoff_time_s is not None and sim_time_s >= command.boost_cutoff_time_s:
        values["beco_flag"] = 1
    ####
    if command.terminal_lock_time_s is not None and sim_time_s >= command.terminal_lock_time_s:
        values["mseek"] = 4
    ####
    phase_before = _source_phase(values)
    fidelity_before = _phase_fidelity(phase_before)
    application = runtime.event_cursor.evaluate_and_apply(values)
    if application is None:
        return None
    ####
    runtime.event_time_s = 0.0
    _apply_event_state_mutations(runtime, application)
    phase_after = _source_phase(values)
    return Ghame6PhaseEvent(
        event_index=application.event_index,
        time_s=max(0.0, sim_time_s),
        source_line=application.source_line,
        watch_variable=application.watch_variable,
        operator=application.operator.value,
        criterion=application.criterion,
        previous_values=application.previous_values,
        updated_values=application.updated_values,
        phase_before=phase_before,
        phase_after=phase_after,
        runtime_fidelity_before=fidelity_before,
        runtime_fidelity_after=_phase_fidelity(phase_after),
    )


####


def _apply_event_state_mutations(runtime: _Ghame6HyperRuntime, application: CadacEventApplication) -> None:
    values = runtime.source_values
    if any(name.casefold() == "fmasse" for name, _ in application.updated_values):
        runtime.propulsion = runtime.propulsion.model_copy(
            update={
                "fuel_expended_kg": _float_value(values, "fmasse", 0.0),
                "fuel_expended_derivative_kg_s": 0.0,
            }
        )
    ####
    if application.event_index == 2 and _int_value(values, "mguide", 0) == 5:
        values["mprop"] = 3
    ####
    next_step = _float_value(values, "int_step_new", runtime.dt_s)
    if next_step > 0.0:
        runtime.dt_s = next_step
    ####


####


def _runtime_kinematics(runtime: _Ghame6HyperRuntime, dt_s: float, sim_time_s: float) -> None:
    derivative_new = -_skew(runtime.body_rates_inertial_rad_s) @ runtime.body_from_inertial
    runtime.body_from_inertial = _integrate_array(
        runtime.body_from_inertial,
        derivative_new,
        runtime.body_from_inertial_derivative,
        dt_s,
    )
    runtime.body_from_inertial_derivative = derivative_new
    identity = np.eye(3, dtype=np.float64)
    error = identity - runtime.body_from_inertial @ runtime.body_from_inertial.T
    runtime.body_from_inertial = runtime.body_from_inertial + 0.5 * error @ runtime.body_from_inertial
    lon, lat, alt = _cad_geo84_in(runtime.position_inertial_m, sim_time_s)
    runtime.longitude_rad = lon
    runtime.latitude_rad = lat
    runtime.altitude_m = alt
    runtime.geodetic_from_inertial = _cad_tdi84(lon, lat, alt, sim_time_s)
    runtime.geocentric_from_inertial = _cad_tgi84(lon, lat, alt, sim_time_s)
    body_from_geodetic = runtime.body_from_inertial @ runtime.geodetic_from_inertial.T
    runtime.yaw_deg, runtime.pitch_deg, runtime.roll_deg = _euler_from_dcm(body_from_geodetic)
    earth_rotation_body = runtime.body_from_inertial @ np.asarray((0.0, 0.0, WEII3), dtype=np.float64)
    runtime.body_rates_earth_rad_s = runtime.body_rates_inertial_rad_s - earth_rotation_body
    air_body = body_from_geodetic @ runtime.velocity_geodetic_mps
    speed = max(float(np.linalg.norm(air_body)), 1.0e-12)
    runtime.alpha_deg = math.atan2(float(air_body[2]), float(air_body[0])) * DEG_PER_RAD
    beta_argument = min(1.0, max(-1.0, float(air_body[1]) / speed))
    runtime.beta_deg = math.asin(beta_argument) * DEG_PER_RAD


####


def _runtime_environment(runtime: _Ghame6HyperRuntime, sim_time_s: float) -> None:
    density, pressure, temperature, speed_of_sound = ghame6_atmosphere(
        _int_value(runtime.source_values, "mair", 0),
        runtime.altitude_m,
    )
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.speed_of_sound_mps = speed_of_sound
    runtime.geographic_speed_mps = float(np.linalg.norm(runtime.velocity_geodetic_mps))
    runtime.mach = abs(runtime.geographic_speed_mps / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * density * runtime.geographic_speed_mps * runtime.geographic_speed_mps
    runtime.gravity_inertial_mps2 = runtime.geocentric_from_inertial.T @ _cad_grav84_geocentric(runtime.position_inertial_m, sim_time_s)


####


def _runtime_propulsion(
    definition: Ghame6SourceDefinition,
    runtime: _Ghame6HyperRuntime,
    dt_s: float,
) -> None:
    values = runtime.source_values
    mode = _int_value(values, "mprop", 0)
    fuel_expended = runtime.propulsion.fuel_expended_kg
    derivative_previous = runtime.propulsion.fuel_expended_derivative_kg_s
    throttle = _float_value(values, "throttle", runtime.propulsion.throttle)
    isp = 0.0
    thrust = 0.0
    inertia_initial: FloatMatrix
    inertia_final: FloatMatrix
    if mode in {1, 2}:
        query = (throttle, runtime.mach)
        isp = definition.propulsion_deck.table("spi_vs_throttle_mach").interpolate(query)
        capture = definition.propulsion_deck.table("ca_vs_alpha_mach").interpolate((runtime.alpha_deg, runtime.mach))
        denom = 0.029 * isp * AGRAV * runtime.density_kg_m3 * runtime.geographic_speed_mps * capture * _float_value(values, "acowl")
        if mode == 2 and abs(denom) > 1.0e-12:
            required = (
                runtime.aero_coefficients.reference_area_m2
                * runtime.aero_coefficients.drag_coefficient
                * _float_value(values, "qhold")
                / max(math.cos(runtime.alpha_deg * RAD_PER_DEG), 1.0e-6)
            )
            requested = required / denom
            tq = max(_float_value(values, "tq", 1.0), 1.0e-9)
            gain = 2.0 * runtime.propulsion.mass_kg / max(runtime.density_kg_m3 * runtime.geographic_speed_mps * denom * tq, 1.0e-12)
            throttle = requested + gain * (_float_value(values, "qhold") - runtime.dynamic_pressure_pa)
            throttle = min(_float_value(values, "thrtl_max", 2.0), max(_float_value(values, "thrtl_idle", 0.05), throttle))
            isp = definition.propulsion_deck.table("spi_vs_throttle_mach").interpolate((throttle, runtime.mach))
        ####
        thrust = isp * 0.029 * throttle * AGRAV * runtime.density_kg_m3 * runtime.geographic_speed_mps * capture * _float_value(values, "acowl")
        inertia_initial = np.asarray(((1.573e6, 0.0, 0.38e6), (0.0, 31.6e6, 0.0), (0.38e6, 0.0, 32.54e6)), dtype=np.float64)
        inertia_final = np.asarray(((1.18e6, 0.0, 0.24e6), (0.0, 19.25e6, 0.0), (0.24e6, 0.0, 20.2e6)), dtype=np.float64)
    elif mode in {3, 4}:
        isp = _float_value(values, "isp_fuel", _float_value(values, "spi", 0.0))
        source_flow = _float_value(values, "fuel_flow_rate", 0.0)
        if mode == 3:
            burntime = _float_value(values, "burntime", _float_value(values, "burnout_epoch1", 0.0))
            if burntime > 0.0:
                source_flow = _float_value(values, "fmass0") / burntime
            ####
        ####
        thrust = isp * source_flow * AGRAV
        inertia_initial = np.diag(
            np.asarray(
                (
                    _float_value(values, "moi_roll_exo_0"),
                    _float_value(values, "moi_trans_exo_0"),
                    _float_value(values, "moi_trans_exo_0"),
                ),
                dtype=np.float64,
            )
        )
        inertia_final = np.diag(
            np.asarray(
                (
                    _float_value(values, "moi_roll_exo_1"),
                    _float_value(values, "moi_trans_exo_1"),
                    _float_value(values, "moi_trans_exo_1"),
                ),
                dtype=np.float64,
            )
        )
    elif mode == 0:
        runtime.thrust_n = 0.0
        runtime.propulsion = runtime.propulsion.model_copy(update={"fuel_expended_derivative_kg_s": 0.0, "throttle": throttle, "specific_impulse_s": 0.0})
        values["thrust"] = 0.0
        values["vmass"] = runtime.propulsion.mass_kg
        values["fmassr"] = runtime.remaining_fuel_kg
        return
    else:
        raise ValueError(f"unsupported GHAME6 propulsion mode {mode}")
    ####
    derivative_new = thrust / max(isp * AGRAV, 1.0e-12)
    fuel_expended = _integrate_scalar(fuel_expended, derivative_new, derivative_previous, dt_s)
    initial_mass = _float_value(values, "vmass0")
    initial_fuel = max(_float_value(values, "fmass0"), 1.0e-9)
    ratio = fuel_expended / initial_fuel
    mass = max(initial_mass - fuel_expended, 1.0e-9)
    inertia = inertia_initial + (inertia_final - inertia_initial) * ratio
    remaining = initial_fuel - fuel_expended
    if remaining <= 0.0:
        values["mprop"] = 0
        if mode == 3:
            values["beco_flag"] = 1
        ####
        thrust = 0.0
    ####
    runtime.propulsion = Ghame6PropulsionState(
        fuel_expended_derivative_kg_s=derivative_new,
        fuel_expended_kg=max(fuel_expended, 0.0),
        mass_kg=mass,
        inertia_body_kgm2=_tuple9(inertia),
        throttle=throttle,
        specific_impulse_s=isp,
    )
    runtime.remaining_fuel_kg = remaining
    runtime.thrust_n = thrust
    values["throttle"] = throttle
    values["fmasse"] = fuel_expended
    values["fmassr"] = remaining
    values["vmass"] = mass
    values["thrust"] = thrust


####


def _runtime_seeker(
    runtime: _Ghame6HyperRuntime,
    satellite: _Ghame6SatelliteRuntime,
    command: Ghame6DirectCommand,
    sim_time_s: float,
) -> None:
    values = runtime.source_values
    if command.terminal_lock_time_s is not None and sim_time_s >= command.terminal_lock_time_s:
        values["mseek"] = 4
        return
    ####
    mode = _int_value(values, "mseek", 0)
    if mode == 0:
        return
    ####
    relative_inertial = satellite.position_inertial_m - runtime.position_inertial_m
    distance = float(np.linalg.norm(relative_inertial))
    values["dbt"] = distance
    if mode == 2 and distance < _float_value(values, "racq", 0.0):
        values["mseek"] = 3
        runtime.seeker_acquisition_epoch_s = sim_time_s
        mode = 3
    ####
    if mode == 3:
        body_relative = runtime.body_from_inertial @ relative_inertial
        azimuth = math.atan2(float(body_relative[1]), float(body_relative[0]))
        elevation = math.atan2(-float(body_relative[2]), math.hypot(float(body_relative[0]), float(body_relative[1])))
        fov = _float_value(values, "fovlimx", 85.0) * RAD_PER_DEG
        epoch = runtime.seeker_acquisition_epoch_s if runtime.seeker_acquisition_epoch_s is not None else sim_time_s
        if abs(azimuth) <= fov and abs(elevation) <= fov and sim_time_s - epoch > _float_value(values, "dtimac", 0.0):
            values["mseek"] = 4
        ####
    ####
    if distance < _float_value(values, "dblind", 0.0):
        values["mseek"] = 5
    ####


####


def _runtime_rcs(runtime: _Ghame6HyperRuntime, command: Ghame6DirectCommand) -> None:
    values = runtime.source_values
    mass = runtime.propulsion.mass_kg
    config = Rocket6gRcsConfig(
        moment_mode=_int_value(values, "mrcs_moment", 0),
        force_mode=_int_value(values, "mrcs_force", 0),
        dead_zone=_float_value(values, "dead_zone", 0.0),
        hysteresis=_float_value(values, "hysteresis", 0.0),
        time_slope_s=_float_value(values, "rcs_tau", 0.0),
        roll_moment_limit_nm=_float_value(values, "roll_mom_max", 0.0),
        pitch_moment_limit_nm=_float_value(values, "pitch_mom_max", 0.0),
        yaw_moment_limit_nm=_float_value(values, "yaw_mom_max", 0.0),
        proportional_damping=_float_value(values, "rcs_zeta", 0.0),
        proportional_frequency_rad_s=_float_value(values, "rcs_freq", 0.0),
        acceleration_gain_n_per_mps2=mass,
        side_force_limit_n=_float_value(values, "side_force_max", 0.0),
        roll_command_deg=_float_value(values, "phibdcomx", 0.0) if command.roll_command_deg is None else command.roll_command_deg,
        pitch_command_deg=_float_value(values, "thtbdcomx", 0.0) if command.pitch_command_deg is None else command.pitch_command_deg,
        yaw_command_deg=_float_value(values, "psibdcomx", 0.0) if command.yaw_command_deg is None else command.yaw_command_deg,
    )
    runtime.rcs_step = rocket6g_rcs_step(
        config,
        Rocket6gRcsRuntimeInput(
            inertia_diagonal_kgm2=_inertia_diagonal(runtime.propulsion.inertia_body_kgm2),
            body_rates_rad_s=_tuple3(runtime.body_rates_earth_rad_s),
            geodetic_angles_deg=(runtime.roll_deg, runtime.pitch_deg, runtime.yaw_deg),
            thrust_vector_unit_body=command.normalized_thrust_vector(),
            incidence_deg=(runtime.alpha_deg, runtime.beta_deg),
            incidence_commands_deg=(command.alpha_command_deg, command.beta_command_deg),
            specific_force_body_mps2=(0.0, 0.0, 0.0),
            acceleration_commands_g=(command.lateral_acceleration_command_g, command.normal_acceleration_command_g),
        ),
        runtime.rcs_state,
    )
    runtime.rcs_state = runtime.rcs_step.state


####


def _runtime_newton(runtime: _Ghame6HyperRuntime, dt_s: float, sim_time_s: float) -> None:
    force_body = np.asarray(runtime.wrench.force_body_n, dtype=np.float64)
    runtime.specific_force_body_mps2 = force_body / runtime.propulsion.mass_kg
    acceleration_new = runtime.body_from_inertial.T @ runtime.specific_force_body_mps2 + runtime.gravity_inertial_mps2
    old_velocity = runtime.velocity_inertial_mps.copy()
    next_velocity = _integrate_array(runtime.velocity_inertial_mps, acceleration_new, runtime.acceleration_inertial_mps2, dt_s)
    runtime.position_inertial_m = _integrate_array(runtime.position_inertial_m, next_velocity, old_velocity, dt_s)
    runtime.acceleration_inertial_mps2 = acceleration_new
    runtime.velocity_inertial_mps = next_velocity
    lon, lat, alt = _cad_geo84_in(runtime.position_inertial_m, sim_time_s)
    runtime.longitude_rad = lon
    runtime.latitude_rad = lat
    runtime.altitude_m = alt
    runtime.geodetic_from_inertial = _cad_tdi84(lon, lat, alt, sim_time_s)
    runtime.geocentric_from_inertial = _cad_tgi84(lon, lat, alt, sim_time_s)
    omega_skew = _skew(np.asarray((0.0, 0.0, WEII3), dtype=np.float64))
    runtime.velocity_geodetic_mps = runtime.geodetic_from_inertial @ (runtime.velocity_inertial_mps - omega_skew @ runtime.position_inertial_m)
    runtime.geographic_speed_mps = float(np.linalg.norm(runtime.velocity_geodetic_mps))
    runtime.heading_deg, runtime.flight_path_deg = _polar_angles(runtime.velocity_geodetic_mps)


####


def _runtime_euler(runtime: _Ghame6HyperRuntime, dt_s: float) -> None:
    inertia = np.asarray(runtime.propulsion.inertia_body_kgm2, dtype=np.float64).reshape(3, 3)
    omega = runtime.body_rates_inertial_rad_s
    moment = np.asarray(runtime.wrench.moment_body_nm, dtype=np.float64)
    derivative_new = np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega))
    runtime.body_rates_inertial_rad_s = _integrate_array(omega, derivative_new, runtime.body_rate_derivative_rad_s2, dt_s)
    runtime.body_rate_derivative_rad_s2 = derivative_new


####


def _runtime_satellite(
    config: Ghame6SatelliteConfig,
    runtime: _Ghame6SatelliteRuntime,
    dt_s: float,
) -> None:
    radius = max(float(np.linalg.norm(runtime.position_inertial_m)), 1.0)
    gravity = -GM * runtime.position_inertial_m / (radius * radius * radius)
    velocity_unit = runtime.velocity_inertial_mps / max(float(np.linalg.norm(runtime.velocity_inertial_mps)), 1.0e-12)
    thrust_acceleration = velocity_unit * (config.thrust_n / config.mass_kg)
    acceleration_new = gravity + thrust_acceleration
    old_velocity = runtime.velocity_inertial_mps.copy()
    next_velocity = _integrate_array(runtime.velocity_inertial_mps, acceleration_new, runtime.acceleration_inertial_mps2, dt_s)
    runtime.position_inertial_m = _integrate_array(runtime.position_inertial_m, next_velocity, old_velocity, dt_s)
    runtime.velocity_inertial_mps = next_velocity
    runtime.acceleration_inertial_mps2 = acceleration_new


####


def _runtime_radar(
    config: Ghame6RadarConfig,
    runtime: _Ghame6RadarRuntime,
    satellite: _Ghame6SatelliteRuntime,
    sim_time_s: float,
    dt_s: float,
    rng: np.random.Generator,
) -> Ghame6RadarTrack | None:
    if not config.enabled or sim_time_s + 0.5 * dt_s < runtime.next_track_time_s:
        return None
    ####
    site_position, site_velocity = _ground0_inertial_state(config, sim_time_s)
    line = site_position - satellite.position_inertial_m
    true_range = float(np.linalg.norm(line))
    azimuth = math.atan2(float(line[1]), float(line[0]))
    elevation = math.atan2(-float(line[2]), math.hypot(float(line[0]), float(line[1])))
    measured_range = true_range + float(rng.normal(0.0, config.range_sigma_m))
    measured_azimuth = azimuth + float(rng.normal(0.0, config.azimuth_sigma_rad))
    measured_elevation = elevation + float(rng.normal(0.0, config.elevation_sigma_rad))
    measured_relative = np.asarray(
        (
            measured_range * math.cos(measured_elevation) * math.cos(measured_azimuth),
            measured_range * math.cos(measured_elevation) * math.sin(measured_azimuth),
            -measured_range * math.sin(measured_elevation),
        ),
        dtype=np.float64,
    )
    measured_position = site_position - measured_relative
    measured_velocity = satellite.velocity_inertial_mps + rng.normal(0.0, config.velocity_sigma_mps, size=3)
    runtime.update_count += 1
    native_packet = _native_radar_relative_state_packet(
        sim_time_s=sim_time_s,
        site_position_inertial_m=site_position,
        site_velocity_inertial_mps=site_velocity,
        satellite=satellite,
        update_sequence=runtime.update_count,
    )
    track = Ghame6RadarTrack(
        time_s=max(0.0, sim_time_s),
        measured_position_inertial_m=_tuple3(measured_position),
        measured_velocity_inertial_mps=_tuple3(np.asarray(measured_velocity, dtype=np.float64)),
        true_range_m=true_range,
        update_sequence=runtime.update_count,
        native_relative_state_packet=native_packet,
    )
    runtime.latest_track = track
    runtime.next_track_time_s = sim_time_s + (config.track_step_s if config.track_step_s > 0.0 else dt_s)
    return track


####


def _native_radar_relative_state_packet(
    *,
    sim_time_s: float,
    site_position_inertial_m: FloatVector,
    site_velocity_inertial_mps: FloatVector,
    satellite: _Ghame6SatelliteRuntime,
    update_sequence: int,
) -> dict[str, Any]:
    """Project one committed RADAR0/SAT3 boundary through the native sensor API.

    GHAME6's source RADAR0 track is a separately preserved noisy polar
    reconstruction.  This zero-noise raw packet gives standard consumers the
    corresponding committed geometry without claiming a persistent
    ``SensorBus`` or rewriting the source radar's scheduling/filter state.
    RADAR0 has no modeled gimbal/body attitude in this source package, so its
    sensor frame is explicitly aligned with the published inertial frame.
    """

    sample_time_s = max(0.0, sim_time_s)
    payload_or_reason = relative_state_track_from_geometry(
        target_id=_GHAME6_RADAR_TARGET_ID,
        host_position_world_m=site_position_inertial_m,
        host_velocity_world_mps=site_velocity_inertial_mps,
        orientation_world_from_body=np.eye(3, dtype=np.float64),
        target_position_world_m=satellite.position_inertial_m,
        target_velocity_world_mps=satellite.velocity_inertial_mps,
        host_body_rate_rad_s=None,
        config=RelativeStateTrackerConfig(target_id=_GHAME6_RADAR_TARGET_ID),
    )
    if isinstance(payload_or_reason, str):
        packet = MeasurementPacket(
            sampled_at_s=sample_time_s,
            available_at_s=sample_time_s,
            interval_start_s=None,
            payload=None,
            valid=False,
            sensor_id=_GHAME6_RADAR_SENSOR_ID,
            port="track",
            sequence=update_sequence - 1,
            schema_id="taoryx.tracking.relative-state/v1",
            invalid_reason=payload_or_reason,
        )
    else:
        packet = MeasurementPacket(
            sampled_at_s=sample_time_s,
            available_at_s=sample_time_s,
            interval_start_s=None,
            payload=replace(payload_or_reason, frame_id=_GHAME6_RADAR_SENSOR_FRAME_ID),
            sensor_id=_GHAME6_RADAR_SENSOR_ID,
            port="track",
            sequence=update_sequence - 1,
            schema_id="taoryx.tracking.relative-state/v1",
        )
    ####
    return packet_to_record(
        packet,
        provenance={
            "provider": "relative-state-track",
            "execution": "source-ordered-batch-boundary",
            "source_actor": "RADAR0",
            "target_actor": "SAT3",
            "source_update_sequence": update_sequence,
            "claim_boundary": (
                "Raw committed RADAR0/SAT3 geometry only. The accompanying radar_track_update retains CADAC's "
                "source polar convention, deterministic noise sequence, cadence, and track-file behavior; no persistent SensorBus is claimed."
            ),
        },
    )


####


def _hyper_sample(
    runtime: _Ghame6HyperRuntime,
    command: Ghame6DirectCommand,
    sim_time_s: float,
) -> Ghame6HyperSample:
    phase = _source_phase(runtime.source_values)
    inertia_diagonal = _inertia_diagonal(runtime.propulsion.inertia_body_kgm2)
    inertia_matrix = np.asarray(runtime.propulsion.inertia_body_kgm2, dtype=np.float64).reshape(3, 3)
    return Ghame6HyperSample(
        time_s=max(0.0, sim_time_s),
        position_inertial_m=_tuple3(runtime.position_inertial_m),
        velocity_inertial_mps=_tuple3(runtime.velocity_inertial_mps),
        quaternion_wxyz=_dcm_to_quaternion(runtime.body_from_inertial),
        body_rates_inertial_rad_s=_tuple3(runtime.body_rates_inertial_rad_s),
        body_rates_earth_rad_s=_tuple3(runtime.body_rates_earth_rad_s),
        longitude_deg=runtime.longitude_rad * DEG_PER_RAD,
        latitude_deg=runtime.latitude_rad * DEG_PER_RAD,
        altitude_m=runtime.altitude_m,
        geographic_speed_mps=max(runtime.geographic_speed_mps, 0.0),
        heading_deg=runtime.heading_deg,
        flight_path_deg=runtime.flight_path_deg,
        roll_deg=runtime.roll_deg,
        pitch_deg=runtime.pitch_deg,
        yaw_deg=runtime.yaw_deg,
        alpha_deg=runtime.alpha_deg,
        beta_deg=runtime.beta_deg,
        mach=max(runtime.mach, 0.0),
        dynamic_pressure_pa=max(runtime.dynamic_pressure_pa, 0.0),
        source_phase=phase,
        runtime_fidelity=_phase_fidelity(phase),
        control_realization=_phase_realization(phase),
        aerodynamic_mode=_int_value(runtime.source_values, "maero", 0),
        propulsion_mode=_int_value(runtime.source_values, "mprop", 0),
        seeker_mode=_int_value(runtime.source_values, "mseek", 0),
        rcs_moment_mode=_int_value(runtime.source_values, "mrcs_moment", 0),
        rcs_force_mode=_int_value(runtime.source_values, "mrcs_force", 0),
        mass_kg=runtime.propulsion.mass_kg,
        remaining_fuel_kg=runtime.remaining_fuel_kg,
        inertia_diagonal_kgm2=inertia_diagonal,
        inertia_xz_kgm2=float(inertia_matrix[0, 2]),
        throttle=runtime.propulsion.throttle,
        thrust_n=runtime.thrust_n,
        requested_control_deg=runtime.surface_step.requested_control_deg,
        requested_surfaces_deg=runtime.surface_step.requested_surfaces_deg,
        achieved_surfaces_deg=runtime.surface_step.achieved_surfaces_deg,
        achieved_control_deg=runtime.surface_step.achieved_control_deg,
        requested_rcs_attitude_deg=(
            _float_value(runtime.source_values, "phibdcomx", 0.0) if command.roll_command_deg is None else command.roll_command_deg,
            _float_value(runtime.source_values, "thtbdcomx", 0.0) if command.pitch_command_deg is None else command.pitch_command_deg,
            _float_value(runtime.source_values, "psibdcomx", 0.0) if command.yaw_command_deg is None else command.yaw_command_deg,
        ),
        requested_rcs_incidence_deg=(command.alpha_command_deg, command.beta_command_deg),
        requested_rcs_acceleration_g=(
            command.lateral_acceleration_command_g,
            command.normal_acceleration_command_g,
        ),
        requested_thrust_vector_unit_body=command.normalized_thrust_vector(),
        rcs_force_body_n=runtime.rcs_step.force_body_n,
        rcs_moment_body_nm=runtime.rcs_step.moment_body_nm,
        force_body_n=runtime.wrench.force_body_n,
        moment_body_nm=runtime.wrench.moment_body_nm,
    )


####


def _satellite_sample(runtime: _Ghame6SatelliteRuntime, sim_time_s: float) -> Ghame6SatelliteSample:
    lon, lat, alt = _cad_geo84_in(runtime.position_inertial_m, sim_time_s)
    return Ghame6SatelliteSample(
        time_s=max(0.0, sim_time_s),
        position_inertial_m=_tuple3(runtime.position_inertial_m),
        velocity_inertial_mps=_tuple3(runtime.velocity_inertial_mps),
        longitude_deg=lon * DEG_PER_RAD,
        latitude_deg=lat * DEG_PER_RAD,
        altitude_m=alt,
        speed_mps=float(np.linalg.norm(runtime.velocity_inertial_mps)),
    )


####


def _ground0_inertial_state(
    config: Ghame6RadarConfig,
    sim_time_s: float,
) -> tuple[FloatVector, FloatVector]:
    radius = CADAC_SOURCE_EARTH_RADIUS_M + config.altitude_m
    latitude = config.latitude_deg * RAD_PER_DEG
    celestial_longitude = config.longitude_deg * RAD_PER_DEG + WEII3 * sim_time_s
    position = np.asarray(
        (
            radius * math.cos(latitude) * math.cos(celestial_longitude),
            radius * math.cos(latitude) * math.sin(celestial_longitude),
            radius * math.sin(latitude),
        ),
        dtype=np.float64,
    )
    velocity = np.cross(np.asarray((0.0, 0.0, WEII3), dtype=np.float64), position)
    return position, velocity


####


def _radar_site_sample(config: Ghame6RadarConfig, sim_time_s: float) -> Ghame6RadarSiteSample:
    position, velocity = _ground0_inertial_state(config, sim_time_s)
    return Ghame6RadarSiteSample(
        time_s=max(0.0, sim_time_s),
        position_inertial_m=_tuple3(position),
        velocity_inertial_mps=_tuple3(velocity),
        longitude_deg=config.longitude_deg,
        latitude_deg=config.latitude_deg,
        altitude_m=config.altitude_m,
    )


####


def _lower_initial_state(
    vehicle: CadacVehicleBlock,
    values: dict[str, int | float],
) -> Ghame6InitialState:
    mode = _int_value(values, "minit", 0)
    if mode == 0:
        longitude = _number(vehicle, "lonx")
        latitude = _number(vehicle, "latx")
        yaw = _number(vehicle, "psibdx", _number(vehicle, "psivdx", 0.0))
    elif mode == 1:
        position_orbit, velocity_orbit = _orbital_state(
            _float_value(values, "sat_semi"),
            _float_value(values, "sat_ecc"),
            _float_value(values, "sat_inclx"),
            _float_value(values, "sat_lon_anodex"),
            _float_value(values, "sat_arg_perix"),
            _float_value(values, "sat_true_anomx") + _float_value(values, "ranglex_l_t"),
        )
        lon_rad, lat_rad, _ = _cad_geo84_in(position_orbit, 0.0)
        tdi = _cad_tdi84(lon_rad, lat_rad, 0.0, 0.0)
        relative_velocity = tdi @ (velocity_orbit - np.cross(np.asarray((0.0, 0.0, WEII3), dtype=np.float64), position_orbit))
        heading, _ = _polar_angles(relative_velocity)
        if _int_value(values, "headon_flag", 0):
            heading = _wrap_pi(heading * RAD_PER_DEG - math.pi) * DEG_PER_RAD
        ####
        longitude = lon_rad * DEG_PER_RAD
        latitude = lat_rad * DEG_PER_RAD
        yaw = heading
    else:
        raise Ghame6SourceError(f"unsupported GHAME6 initialization mode {mode}")
    ####
    return Ghame6InitialState(
        initialization_mode=mode,
        longitude_deg=longitude,
        latitude_deg=latitude,
        altitude_m=_number(vehicle, "alt"),
        geographic_speed_mps=_number(vehicle, "dvbe"),
        roll_deg=_number(vehicle, "phibdx", 0.0),
        pitch_deg=_number(vehicle, "thtbdx", 0.0),
        yaw_deg=yaw,
        alpha_deg=_number(vehicle, "alpha0x", 0.0),
        beta_deg=_number(vehicle, "beta0x", 0.0),
        body_rates_deg_s=(
            _number(vehicle, "ppx", 0.0),
            _number(vehicle, "qqx", 0.0),
            _number(vehicle, "rrx", 0.0),
        ),
    )


####


def _initial_runtime_values(vehicle: CadacVehicleBlock) -> dict[str, int | float]:
    values: dict[str, int | float] = {}
    for assignment in vehicle.assignments:
        if isinstance(assignment.value, (int, float)):
            values[assignment.name] = assignment.value
        ####
    ####
    for event in vehicle.events:
        if event.condition.variable not in values:
            values[event.condition.variable] = 0
        ####
        for assignment in event.assignments:
            if assignment.name not in values and isinstance(assignment.value, (int, float)):
                values[assignment.name] = 0 if isinstance(assignment.value, int) else 0.0
            ####
        ####
    ####
    defaults: dict[str, int | float] = {
        "time": 0.0,
        "event_time": 0.0,
        "int_step_new": 0.0,
        "thrust": 0.0,
        "fmasse": 0.0,
        "fmassr": _number(vehicle, "fmass0", 0.0),
        "vmass": _number(vehicle, "vmass0"),
        "beco_flag": 0,
        "mseek": 0,
        "mrcs_moment": 0,
        "mrcs_force": 0,
        "rcs_zeta": 0.0,
        "rcs_freq": 0.0,
        "side_force_max": 0.0,
        "dead_zone": 0.0,
        "hysteresis": 0.0,
        "rcs_tau": 0.0,
        "phibdcomx": 0.0,
        "thtbdcomx": 0.0,
        "psibdcomx": 0.0,
        "isp_fuel": 0.0,
        "fuel_flow_rate": 0.0,
        "moi_roll_exo_0": 1.0,
        "moi_roll_exo_1": 1.0,
        "moi_trans_exo_0": 1.0,
        "moi_trans_exo_1": 1.0,
        "burntime": 0.0,
        "burnout_epoch1": 0.0,
    }
    for name, value in defaults.items():
        values.setdefault(name, value)
    ####
    if _float_value(values, "int_step_new", 0.0) <= 0.0:
        values["int_step_new"] = 0.0
    ####
    # CADAC variable storage declares these mission scalars as real-valued even
    # when an input literal is written without a decimal point. Keeping every
    # runtime scalar as float prevents later event mutations such as 0.05 from
    # inheriting and truncating through the lexical type of an earlier 0.
    return {name: float(value) for name, value in values.items()}


####


def _source_phase(values: dict[str, int | float]) -> str:
    if _int_value(values, "maero", 0) == 1:
        return "atmospheric_surfaces"
    ####
    if _int_value(values, "mseek", 0) >= 4 and _int_value(values, "mrcs_force", 0) > 0:
        return "interceptor_terminal_rcs"
    ####
    if _int_value(values, "mguide", 0) == 8:
        return "interceptor_glideslope_rcs"
    ####
    if _int_value(values, "mguide", 0) == 5:
        return "transfer_vector_rcs"
    ####
    if _int_value(values, "mrcs_moment", 0) > 0 or _int_value(values, "mrcs_force", 0) > 0:
        return "transfer_angle_rcs"
    ####
    return "uncontrolled_rigid_body"


####


def _phase_fidelity(phase: str) -> str:
    if phase == "atmospheric_surfaces":
        return "rigid_body_6dof_surface_allocated"
    ####
    return "rigid_body_6dof_direct_wrench"


####


def _phase_realization(phase: str) -> str:
    if phase == "atmospheric_surfaces":
        return "physical_three_surface"
    ####
    if phase == "uncontrolled_rigid_body":
        return "uncontrolled"
    ####
    return "axis_aggregate_rcs"


####


def _orbital_state(
    semi_major_axis_m: float,
    eccentricity: float,
    inclination_deg: float,
    longitude_ascending_node_deg: float,
    argument_periapsis_deg: float,
    true_anomaly_deg: float,
) -> tuple[FloatVector, FloatVector]:
    p_value = semi_major_axis_m * (1.0 - eccentricity * eccentricity)
    if p_value <= 0.0:
        raise ValueError("GHAME6 orbital semi-latus rectum must be positive")
    ####
    anomaly = true_anomaly_deg * RAD_PER_DEG
    radius = p_value / (1.0 + eccentricity * math.cos(anomaly))
    position_perifocal = np.asarray((radius * math.cos(anomaly), radius * math.sin(anomaly), 0.0), dtype=np.float64)
    scale = math.sqrt(GM / p_value)
    velocity_perifocal = np.asarray((-scale * math.sin(anomaly), scale * (eccentricity + math.cos(anomaly)), 0.0), dtype=np.float64)
    inclination = inclination_deg * RAD_PER_DEG
    node = longitude_ascending_node_deg * RAD_PER_DEG
    periapsis = argument_periapsis_deg * RAD_PER_DEG
    c_node, s_node = math.cos(node), math.sin(node)
    c_peri, s_peri = math.cos(periapsis), math.sin(periapsis)
    c_inc, s_inc = math.cos(inclination), math.sin(inclination)
    transform = np.asarray(
        (
            (c_node * c_peri - s_node * s_peri * c_inc, -c_node * s_peri - s_node * c_peri * c_inc, s_node * s_inc),
            (s_node * c_peri + c_node * s_peri * c_inc, -s_node * s_peri + c_node * c_peri * c_inc, -c_node * s_inc),
            (s_peri * s_inc, c_peri * s_inc, c_inc),
        ),
        dtype=np.float64,
    )
    return transform @ position_perifocal, transform @ velocity_perifocal


####


def _runtime_is_finite(
    hyper: _Ghame6HyperRuntime,
    satellite: _Ghame6SatelliteRuntime,
) -> bool:
    arrays = (
        hyper.position_inertial_m,
        hyper.velocity_inertial_mps,
        hyper.body_from_inertial,
        hyper.body_rates_inertial_rad_s,
        satellite.position_inertial_m,
        satellite.velocity_inertial_mps,
    )
    scalars = (
        hyper.altitude_m,
        hyper.geographic_speed_mps,
        hyper.mach,
        hyper.dynamic_pressure_pa,
        hyper.propulsion.mass_kg,
        hyper.thrust_n,
    )
    return all(bool(np.isfinite(array).all()) for array in arrays) and all(math.isfinite(value) for value in scalars)


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Ghame6SourceError(f"GHAME6 vehicle {vehicle.model_name!r} is missing source parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Ghame6SourceError(f"GHAME6 source parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not float(value).is_integer():
        raise Ghame6SourceError(f"GHAME6 source parameter {name!r} must be integral")
    ####
    return int(value)


####


def _float_value(values: dict[str, int | float], name: str, default: float | None = None) -> float:
    try:
        value = values[name]
    except KeyError:
        if default is None:
            raise Ghame6SourceError(f"GHAME6 runtime is missing scalar {name!r}") from None
        ####
        return float(default)
    ####
    return float(value)


####


def _int_value(values: dict[str, int | float], name: str, default: int | None = None) -> int:
    try:
        value = values[name]
    except KeyError:
        if default is None:
            raise Ghame6SourceError(f"GHAME6 runtime is missing integer {name!r}") from None
        ####
        return int(default)
    ####
    return int(value)


####


def _tuple3(values: FloatVector) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


####


def _tuple3_bool(values: NDArray[np.bool_]) -> tuple[bool, bool, bool]:
    return (bool(values[0]), bool(values[1]), bool(values[2]))


####


def _tuple9(values: FloatMatrix) -> tuple[float, float, float, float, float, float, float, float, float]:
    flat = np.asarray(values, dtype=np.float64).reshape(9)
    return (
        float(flat[0]),
        float(flat[1]),
        float(flat[2]),
        float(flat[3]),
        float(flat[4]),
        float(flat[5]),
        float(flat[6]),
        float(flat[7]),
        float(flat[8]),
    )


####


def _inertia_diagonal(
    values: tuple[float, float, float, float, float, float, float, float, float],
) -> tuple[float, float, float]:
    return (float(values[0]), float(values[4]), float(values[8]))


####


__all__ = [
    "Ghame6AeroCoefficients",
    "Ghame6BodyWrench",
    "Ghame6DirectCommand",
    "Ghame6HyperSample",
    "Ghame6InitialState",
    "Ghame6PhaseEvent",
    "Ghame6RadarConfig",
    "Ghame6RadarSiteSample",
    "Ghame6RadarTrack",
    "Ghame6RunResult",
    "Ghame6SatelliteConfig",
    "Ghame6SatelliteSample",
    "Ghame6SourceDefinition",
    "Ghame6SourceError",
    "Ghame6SurfaceActuatorConfig",
    "Ghame6SurfaceState",
    "Ghame6SurfaceStep",
    "ghame6_aerodynamic_coefficients",
    "ghame6_atmosphere",
    "ghame6_body_wrench",
    "ghame6_surface_step",
    "load_ghame6_source_definition",
    "lower_ghame6_source_bundle",
    "run_ghame6_phase_aware_mission",
]
