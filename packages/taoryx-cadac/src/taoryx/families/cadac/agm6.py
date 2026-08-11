"""Source-grounded AGM6 three-actor air-to-ground engagement runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .aim5 import mat2tr
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .deck import CadacDeck
from .events import CadacEventApplication, CadacEventCursor, CadacRuntimeScalar
from .input_ast import (
    CadacDeckKind,
    CadacEventBlock,
    CadacModel,
    CadacModuleStage,
    CadacStochasticKind,
    CadacVehicleBlock,
)
from .sensor_adapter import cadac_local_ned_relative_state_track, cadac_local_ned_sensor_context
from .source_environment import atmosphere76, cadac_source_inverse_square_gravity_mps2
from .sraam6 import (
    Sraam6ActuatorConfig,
    Sraam6ActuatorStep,
    Sraam6ControlCommand,
    Sraam6FinActuatorState,
    Sraam6FinSet,
    sraam6_actuator_step,
)

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

Agm6ActuatorConfig = Sraam6ActuatorConfig
Agm6ActuatorStep = Sraam6ActuatorStep
Agm6ControlCommand = Sraam6ControlCommand
Agm6FinActuatorState = Sraam6FinActuatorState
Agm6FinSet = Sraam6FinSet

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_AGRAV = 9.80675445
_R_AIR = 287.053
_SMALL = 1.0e-10

_AGM6_MODULES = (
    "environment",
    "kinematics",
    "aerodynamics",
    "propulsion",
    "forces",
    "ins",
    "datalink",
    "sensor",
    "guidance",
    "control",
    "actuator",
    "euler",
    "newton",
    "intercept",
)
_AGM6_AERO_TABLES = (
    "ca0_vs_mach",
    "caa_vs_mach",
    "cad_vs_mach",
    "cyp_vs_mach_alpha",
    "cndq_vs_mach",
    "cn0_vs_mach_alpha",
    "cnp_vs_mach_alpha",
    "cllap_vs_mach",
    "cllp_vs_mach",
    "clldp_vs_mach",
    "clm0_vs_mach_alpha",
    "clmp_vs_mach_alpha",
    "clmq_vs_mach",
    "clmdq_vs_mach",
    "clnp_vs_mach_alpha",
)
_AGM6_WEATHER_TABLES = ("density", "pressure", "temperature", "speed", "direction")


class Agm6SourceError(ValueError):
    """Source-bundle incompatibility with the AGM6 reconstruction."""


####


class Agm6InitialState(CadacModel):
    """Source initial truth state for one AGM6 ``MISSILE6`` actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    launch_delay_s: float = Field(default=0.0, ge=0.0)


####


class Agm6Airframe(CadacModel):
    """Fixed AGM6 source geometry and launch mass properties."""

    reference_length_m: float = 0.5
    reference_area_m2: float = 0.196
    launch_mass_kg: float = Field(gt=0.0)
    launch_roll_inertia_kg_m2: float = Field(gt=0.0)
    launch_pitch_inertia_kg_m2: float = Field(gt=0.0)


####


class Agm6AeroLimits(CadacModel):
    """Source aerodynamic and stopping limits."""

    alpha_limit_deg: float = Field(gt=0.0)
    structural_acceleration_limit_g: float = Field(gt=0.0)
    minimum_mach: float = 0.4
    minimum_dynamic_pressure_pa: float = 10_000.0
    minimum_load_capacity_g: float = 0.5
    maximum_total_incidence_rad: float = 1.0
    maximum_quaternion_error: float = 1.0e-4


####


class Agm6PropulsionConfig(CadacModel):
    """Constant-throttle source rocket motor and continuous fuel state."""

    initial_mode: int
    nozzle_exit_area_m2: float = Field(ge=0.0)
    specific_impulse_s: float = Field(gt=0.0)
    sea_level_thrust_n: float = Field(ge=0.0)
    throttle: float = Field(ge=0.0)
    initial_fuel_mass_kg: float = Field(ge=0.0)
    sea_level_pressure_pa: float = Field(default=101_325.0, gt=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Agm6PropulsionConfig":
        if self.initial_mode not in {0, 1}:
            raise ValueError("AGM6 source propulsion mode must be 0 or 1")
        ####
        return self

    ####


####


class Agm6ControlConfig(CadacModel):
    """Source roll, rate, and acceleration controller parameters."""

    initial_mode: int
    acceleration_natural_frequency_rad_s: float = Field(gt=0.0)
    acceleration_damping_ratio: float = Field(ge=0.0)
    acceleration_real_pole_rad_s: float = Field(gt=0.0)
    structural_limit_g: float = Field(gt=0.0)
    pitch_command_limit_deg: float = Field(gt=0.0)
    yaw_command_limit_deg: float = Field(gt=0.0)
    roll_command_limit_deg: float = Field(gt=0.0)
    commanded_roll_deg: float
    roll_natural_frequency_rad_s: float = Field(gt=0.0)
    roll_damping_ratio: float = Field(ge=0.0)
    acceleration_feedforward_gain_s2_m: float = 0.0
    rate_loop_damping_ratio: float = Field(ge=0.0)
    pitch_rate_command_deg_s: float = 0.0
    yaw_rate_command_deg_s: float = 0.0

    @model_validator(mode="after")
    def validate_mode(self) -> "Agm6ControlConfig":
        if self.initial_mode not in {0, 1, 2, 3}:
            raise ValueError("AGM6 source control mode must be 0, 1, 2, or 3")
        ####
        return self

    ####


####


class Agm6SensorConfig(CadacModel):
    """Source IIR sensor state machine and partial dynamic-filter parameters."""

    initial_mode: int
    dynamic_mode: int
    blind_range_m: float = Field(ge=0.0)
    acquisition_range_m: float = Field(gt=0.0)
    acquisition_time_s: float = Field(ge=0.0)
    filter_gain_per_s: float = Field(ge=0.0)
    filter_damping_ratio: float = Field(ge=0.0)
    filter_natural_frequency_rad_s: float = Field(gt=0.0)
    yaw_half_fov_rad: float = Field(gt=0.0)
    pitch_half_fov_rad: float = Field(gt=0.0)
    target_number: int = Field(ge=1)
    maximum_pitch_gimbal_rad: float = math.pi / 2.0
    maximum_pitch_gimbal_rate_rad_s: float = 10.0
    maximum_roll_gimbal_rate_rad_s: float = 14.0
    maximum_tracking_error_rad: float = 1.0

    @model_validator(mode="after")
    def validate_modes(self) -> "Agm6SensorConfig":
        if self.initial_mode not in {0, 2, 3, 4, 5}:
            raise ValueError("AGM6 sensor mode must be 0, 2, 3, 4, or 5")
        ####
        if self.dynamic_mode not in {0, 1}:
            raise ValueError("AGM6 sensor dynamic mode must be 0 or 1")
        ####
        return self

    ####


####


class Agm6GuidanceConfig(CadacModel):
    """Source midcourse and terminal guidance inputs."""

    initial_mode: int
    navigation_gain: float = Field(ge=0.0)
    gravity_bias_g: float = 0.0
    line_gain_per_s: float = 0.0
    nonlinear_gain_factor: float = 0.0
    distance_decrement_m: float = 1.0
    vertical_line_of_attack_deg: float = 0.0

    @model_validator(mode="after")
    def validate_mode(self) -> "Agm6GuidanceConfig":
        midcourse = self.initial_mode // 10
        terminal = self.initial_mode % 10
        if midcourse not in {0, 2, 3, 4} or terminal not in {0, 6}:
            raise ValueError("AGM6 guidance mode must encode midcourse 0/2/3/4 and terminal 0/6")
        ####
        return self

    ####


####


class Agm6EnvironmentConfig(CadacModel):
    """Atmosphere, wind, and source-shaped turbulence inputs."""

    mode: int
    constant_wind_speed_mps: float = Field(default=0.0, ge=0.0)
    wind_direction_deg: float = 0.0
    vertical_wind_mps: float = 0.0
    wind_time_constant_s: float = Field(default=0.1, gt=0.0)
    turbulence_length_m: float = Field(default=100.0, gt=0.0)
    turbulence_sigma_mps: float = Field(default=0.0, ge=0.0)
    rayleigh_wind_mode_mps: float | None = Field(default=None, ge=0.0)

    @property
    def atmosphere_mode(self) -> int:
        return self.mode // 100

    ####

    @property
    def turbulence_mode(self) -> int:
        return (self.mode % 100) // 10

    ####

    @property
    def wind_mode(self) -> int:
        return self.mode % 10

    ####

    @model_validator(mode="after")
    def validate_modes(self) -> "Agm6EnvironmentConfig":
        if self.atmosphere_mode not in {0, 2}:
            raise ValueError("AGM6 atmosphere mode must be 0 or 2")
        ####
        if self.turbulence_mode not in {0, 1}:
            raise ValueError("AGM6 turbulence mode must be 0 or 1")
        ####
        if self.wind_mode not in {0, 1, 2}:
            raise ValueError("AGM6 wind mode must be 0, 1, or 2")
        ####
        return self

    ####


####


class Agm6GroundTargetConfig(CadacModel):
    """Moving TARGET3 point-mass source actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    longitudinal_acceleration_g: float = 0.0
    lateral_acceleration_g: float = 0.0
    launch_delay_s: float = Field(default=0.0, ge=0.0)


####


class Agm6AircraftConfig(CadacModel):
    """Tracking AIRCRAFT3 point-mass source actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    aircraft_option: int = 0
    guidance_gain: float = 0.0
    turn_g: float = 0.0
    bank_time_constant_s: float = Field(default=0.1, ge=0.0)
    bank_limit_deg: float = Field(default=120.0, gt=0.0)
    normal_load_time_constant_s: float = Field(default=0.1, ge=0.0)
    alpha_limit_deg: float = Field(default=40.0, gt=0.0)
    lift_slope_per_deg: float = Field(default=0.0523, gt=0.0)
    wing_loading_n_m2: float = Field(default=3247.0, gt=0.0)
    longitudinal_acceleration_g: float = 0.0
    launch_delay_s: float = Field(default=0.0, ge=0.0)

    @model_validator(mode="after")
    def validate_option(self) -> "Agm6AircraftConfig":
        if self.aircraft_option not in {0, 1, 2}:
            raise ValueError("AGM6 AIRCRAFT3 option must be 0, 1, or 2")
        ####
        return self

    ####


####


class Agm6TrackingConfig(CadacModel):
    """AIRCRAFT3 target-track measurement cadence and noise."""

    track_step_s: float = Field(gt=0.0)
    range_sigma_m: float = Field(ge=0.0)
    azimuth_sigma_rad: float = Field(ge=0.0)
    elevation_sigma_rad: float = Field(ge=0.0)
    velocity_sigma_mps: float = Field(ge=0.0)


####


class Agm6SourceDefinition(CadacModel):
    """Prepared AGM6 three-actor case retaining source schedule and artifacts."""

    schema_id: str = "taoryx.cadac.agm6-source/v0alpha1"
    source_name: str = Field(min_length=1)
    source_model: Literal["MISSILE6"] = "MISSILE6"
    target_source_model: Literal["TARGET3"] = "TARGET3"
    aircraft_source_model: Literal["AIRCRAFT3"] = "AIRCRAFT3"
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    monte_carlo_seed: int = 0
    module_order: tuple[str, ...]
    vehicle_order: tuple[Literal["MISSILE6", "TARGET3", "AIRCRAFT3"], ...]
    initial_state: Agm6InitialState
    airframe: Agm6Airframe
    aerodynamics: Agm6AeroLimits
    propulsion: Agm6PropulsionConfig
    actuator: Agm6ActuatorConfig
    control: Agm6ControlConfig
    sensor: Agm6SensorConfig
    guidance: Agm6GuidanceConfig
    environment: Agm6EnvironmentConfig
    target: Agm6GroundTargetConfig
    aircraft: Agm6AircraftConfig
    tracking: Agm6TrackingConfig
    ins_mode_requested: int = 0
    stop_on_termination: bool = False
    target_plane_yaw_deg: float = 0.0
    target_plane_pitch_deg: float = 0.0
    target_sphere_radius_m: float = Field(default=100.0, gt=0.0)
    aerodynamic_deck: CadacDeck
    weather_deck: CadacDeck
    missile_events: tuple[CadacEventBlock, ...] = ()
    target_events: tuple[CadacEventBlock, ...] = ()
    aircraft_events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "rigid_body_6dof_surface_allocated"
    runtime_fidelity: str = "rigid_body_6dof"
    control_realization: str = "effector_allocated"

    @model_validator(mode="after")
    def validate_definition(self) -> "Agm6SourceDefinition":
        if self.vehicle_order != ("MISSILE6", "TARGET3", "AIRCRAFT3"):
            raise ValueError("AGM6 runtime requires source order MISSILE6, TARGET3, AIRCRAFT3")
        ####
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        weather_names = {table.name.casefold() for table in self.weather_deck.tables}
        missing_aero = tuple(name for name in _AGM6_AERO_TABLES if name.casefold() not in aero_names)
        missing_weather = (
            tuple(name for name in _AGM6_WEATHER_TABLES if name.casefold() not in weather_names)
            if self.environment.atmosphere_mode == 2 or self.environment.wind_mode == 2
            else ()
        )
        if missing_aero or missing_weather:
            raise ValueError("AGM6 source definition is missing required tables: " + ", ".join((*missing_aero, *missing_weather)))
        ####
        if self.sensor.target_number != 1:
            raise ValueError("the first AGM6 runtime supports exactly one TARGET3 assignment")
        ####
        return self

    ####


####


class Agm6PropulsionStep(CadacModel):
    """Continuous source rocket state at one propulsion module call."""

    mode: int
    thrust_n: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    fuel_expended_kg: float = Field(ge=0.0)
    fuel_remaining_kg: float = Field(ge=0.0)
    fuel_flow_kg_s: float = Field(ge=0.0)


####


class Agm6AeroCoefficients(CadacModel):
    """Body-axis coefficients and controller derivatives."""

    axial: float
    side: float
    normal: float
    roll_moment: float
    pitch_moment: float
    yaw_moment: float
    max_acceleration_g: float = Field(ge=0.0)
    normal_alpha_derivative_mps2: float
    normal_control_derivative_mps2: float
    pitch_alpha_derivative_rad_s2: float
    pitch_rate_derivative_per_s: float
    pitch_control_derivative_rad_s2: float
    roll_rate_derivative_per_s: float
    roll_control_derivative_rad_s2: float


####


class Agm6BodyWrench(CadacModel):
    """Total AGM6 body-axis force and moment."""

    force_n: tuple[float, float, float]
    moment_nm: tuple[float, float, float]


####


class Agm6Intercept(CadacModel):
    """Target-plane crossing result."""

    time_s: float = Field(ge=0.0)
    miss_distance_m: float = Field(ge=0.0)
    miss_vector_target_plane_m: tuple[float, float, float]
    target_range_m: float = Field(ge=0.0)


####


class Agm6EventTrace(CadacModel):
    """One accepted source event before an actor module pass."""

    time_s: float = Field(ge=0.0)
    actor: Literal["MISSILE6", "TARGET3", "AIRCRAFT3"]
    event_index: int = Field(ge=0)
    source_line: int = Field(ge=1)
    watch_variable: str = Field(min_length=1)
    operator: str = Field(min_length=1, max_length=1)
    criterion: int | float
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]


####


class Agm6TrackSample(CadacModel):
    """One aircraft-produced target track available to the missile datalink."""

    time_s: float = Field(ge=0.0)
    update_sequence: int = Field(ge=0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]


####


class Agm6Sample(CadacModel):
    """Accepted missile truth and physical-effector telemetry."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    airspeed_mps: float = Field(ge=0.0)
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    altitude_m: float
    wind_ned_mps: tuple[float, float, float]
    turbulence_ned_mps: tuple[float, float, float]
    mach: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    alpha_deg: float
    beta_deg: float
    total_alpha_deg: float
    aerodynamic_roll_deg: float
    requested_control_deg: tuple[float, float, float]
    requested_fins_deg: tuple[float, float, float, float]
    achieved_fins_deg: tuple[float, float, float, float]
    achieved_control_deg: tuple[float, float, float]
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    mass_kg: float = Field(gt=0.0)
    fuel_remaining_kg: float = Field(ge=0.0)
    roll_inertia_kg_m2: float = Field(gt=0.0)
    pitch_inertia_kg_m2: float = Field(gt=0.0)
    thrust_n: float = Field(ge=0.0)
    propulsion_mode: int
    sensor_mode: int
    guidance_mode: int
    autopilot_mode: int
    datalink_update_mode: int
    datalink_track_sequence: int = Field(ge=0)
    target_range_m: float = Field(ge=0.0)
    closing_speed_mps: float
    sensor_pointing_pitch_rad: float
    sensor_pointing_yaw_rad: float
    sensor_los_rate_pitch_rad_s: float
    sensor_los_rate_yaw_rad_s: float
    normal_command_g: float
    lateral_command_g: float
    normal_acceleration_g: float
    lateral_acceleration_g: float
    ins_mode_requested: int
    ins_model_effective: Literal["truth_aligned"] = "truth_aligned"


####


class Agm6PointMassSample(CadacModel):
    """Independent TARGET3 or AIRCRAFT3 root-object sample."""

    time_s: float = Field(ge=0.0)
    actor: Literal["TARGET3", "AIRCRAFT3"]
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    altitude_m: float
    bank_deg: float = 0.0
    normal_load_g: float = 1.0
    alive: bool


####


class Agm6RunResult(CadacModel):
    """Deterministic source-ordered AGM6/TARGET3/AIRCRAFT3 result."""

    schema_id: str = "taoryx.cadac.agm6-run/v0alpha1"
    source_name: str
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: Literal["intercept", "ground_impact", "source_stop", "nonfinite_state", "end_time"]
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    samples: tuple[Agm6Sample, ...] = Field(min_length=1)
    target_samples: tuple[Agm6PointMassSample, ...] = Field(min_length=1)
    aircraft_samples: tuple[Agm6PointMassSample, ...] = Field(min_length=1)
    track_samples: tuple[Agm6TrackSample, ...] = Field(min_length=1)
    intercept: Agm6Intercept | None = None
    event_trace: tuple[Agm6EventTrace, ...] = ()
    claim_boundary: str = (
        "The flat-Earth rigid-body missile, source aerodynamic tables, four physical fins, continuous rocket fuel state, "
        "moving TARGET3, AIRCRAFT3 track production, datalink update/extrapolation, proportional-navigation guidance, "
        "and controller modes execute in source order. Requested real-INS mode is currently projected through a truth-aligned "
        "navigation solution. The IIR sensor preserves acquisition/lock/blind-range and source-shaped LOS-filter dynamics, "
        "but complete focal-plane corruption, aimpoint modulation, gimbal-head geometry, and C-rand stochastic parity remain outside the claim."
    )


####


@dataclass(slots=True)
class _SensorAxisState:
    rate_rad_s: float = 0.0
    rate_derivative_rad_s2: float = 0.0
    acceleration_rad_s2: float = 0.0
    acceleration_derivative_rad_s3: float = 0.0


####


@dataclass(slots=True)
class _PointMassRuntime:
    position_ned_m: FloatVector
    position_derivative_ned_mps: FloatVector
    velocity_ned_mps: FloatVector
    acceleration_ned_mps2: FloatVector
    speed_mps: float
    heading_rad: float
    flight_path_rad: float
    vehicle_to_local: FloatMatrix
    velocity_to_local: FloatMatrix
    gravity_mps2: float = _AGRAV
    density_kg_m3: float = 1.225
    speed_of_sound_mps: float = 340.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    altitude_m: float = 0.0
    bank_rad: float = 0.0
    bank_derivative_rad_s: float = 0.0
    normal_load_g: float = 1.0
    normal_load_derivative_g_s: float = 0.0
    commanded_acceleration_local_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    specific_force_vehicle_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    longitudinal_acceleration_g: float = 0.0
    lateral_acceleration_g: float = 0.0
    actor_option: int = 0
    turn_g: float = 0.0
    guidance_gain: float = 0.0


####


@dataclass(slots=True)
class _TrackPacket:
    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    update_sequence: int
    time_s: float


####


@dataclass(slots=True)
class _ActorPacket:
    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    heading_deg: float
    flight_path_deg: float
    alive: bool


####


@dataclass(slots=True)
class _AircraftPacket(_ActorPacket):
    track: _TrackPacket


####


@dataclass(slots=True)
class _TrackingRuntime:
    next_update_time_s: float = 0.0
    update_sequence: int = 0
    track: _TrackPacket = field(
        default_factory=lambda: _TrackPacket(
            position_ned_m=np.zeros(3, dtype=np.float64),
            velocity_ned_mps=np.zeros(3, dtype=np.float64),
            update_sequence=0,
            time_s=0.0,
        )
    )


####


@dataclass(slots=True)
class _MissileRuntime:
    position_ned_m: FloatVector
    position_derivative_ned_mps: FloatVector
    velocity_body_mps: FloatVector
    velocity_body_derivative_mps2: FloatVector
    velocity_ned_mps: FloatVector
    quaternion_wxyz: FloatVector
    quaternion_derivative: FloatVector
    body_rates_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    fin_state: Agm6FinActuatorState
    requested_control: Agm6ControlCommand
    actuator_step: Agm6ActuatorStep
    propulsion: Agm6PropulsionStep
    coefficients: Agm6AeroCoefficients
    wrench: Agm6BodyWrench
    control_mode: int
    guidance_mode: int
    datalink_update_mode: int
    propulsion_mode: int
    sensor_mode: int
    ins_mode_requested: int
    navigation_gain: float
    gravity_bias_g: float
    event_time_s: float = 0.0
    event_epoch_s: float = 0.0
    launch_epoch_s: float = 0.0
    launch_time_s: float = 0.0
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    total_alpha_deg: float = 0.0
    aerodynamic_roll_rad: float = 0.0
    altitude_m: float = 0.0
    gravity_mps2: float = _AGRAV
    density_kg_m3: float = 1.225
    pressure_pa: float = 101_325.0
    temperature_k: float = 288.15
    speed_of_sound_mps: float = 340.0
    airspeed_mps: float = 0.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    wind_state_ned_mps: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    wind_derivative_ned_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    wind_ned_mps: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    turbulence_ned_mps: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    turbulence_state: float = 0.0
    turbulence_derivative: float = 0.0
    turbulence_state_2: float = 0.0
    turbulence_derivative_2: float = 0.0
    resolved_constant_wind_speed_mps: float = 0.0
    normal_command_g: float = 0.0
    lateral_command_g: float = 0.0
    normal_acceleration_g: float = 0.0
    lateral_acceleration_g: float = 0.0
    yaw_feedforward_derivative: float = 0.0
    yaw_feedforward_state: float = 0.0
    pitch_feedforward_derivative: float = 0.0
    pitch_feedforward_state: float = 0.0
    target_position_stored_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    target_velocity_stored_ned_mps: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    target_update_epoch_s: float = 0.0
    datalink_track_sequence: int = 0
    datalink_target_position_norm_m: float = 0.0
    target_range_m: float = math.inf
    closing_speed_mps: float = 0.0
    unit_los_local: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    unit_los_body: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    true_los_rate_body_rad_s: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    sensor_pointing_pitch_rad: float = 0.0
    sensor_pointing_yaw_rad: float = 0.0
    sensor_pointing_pitch_derivative_rad_s: float = 0.0
    sensor_pointing_yaw_derivative_rad_s: float = 0.0
    sensor_acquisition_epoch_s: float = 0.0
    sensor_initialized: bool = False
    sensor_pitch: _SensorAxisState = field(default_factory=_SensorAxisState)
    sensor_yaw: _SensorAxisState = field(default_factory=_SensorAxisState)
    previous_target_plane_position_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    previous_time_s: float = 0.0
    source_stop: bool = False


####


class Agm6ScenarioSession:
    """Persistent source-order AGM6, TARGET3, and AIRCRAFT3 execution state."""

    def __init__(self, definition: Agm6SourceDefinition) -> None:
        self.definition = definition
        self.reset()
    ####

    def reset(self) -> None:
        """Reconstruct source actors, event cursors, stochastic state, and buses."""

        self.rng = np.random.default_rng(self.definition.monte_carlo_seed)
        self.missile = _initialize_missile(self.definition, self.definition.initial_state, self.rng)
        self.target = _initialize_point_mass_target(self.definition.target)
        self.aircraft = _initialize_aircraft(self.definition.aircraft)
        self.target_packet = _point_packet(self.target, alive=True)
        self.tracking = _TrackingRuntime()
        self.aircraft_packet = _aircraft_packet(self.aircraft, self.tracking, alive=True)
        self.missile_events = CadacEventCursor.from_events(self.definition.missile_events)
        self.target_events = CadacEventCursor.from_events(self.definition.target_events)
        self.aircraft_events = CadacEventCursor.from_events(self.definition.aircraft_events)
        self.target_event_epoch_s = 0.0
        self.aircraft_event_epoch_s = 0.0
        self.intercept: Agm6Intercept | None = None
        self.missile_alive = True
        self.target_alive = True
        self.aircraft_alive = True
        self.sim_time_s = 0.0
        self.executed_steps = 0
        self.terminated_reason: Literal[
            "intercept",
            "ground_impact",
            "source_stop",
            "nonfinite_state",
            "end_time",
        ] | None = None
    ####

    @property
    def completed(self) -> bool:
        """Return whether the source loop reached one terminal boundary."""

        return self.terminated_reason is not None
    ####

    def sample(self) -> Agm6Sample:
        """Return current committed missile state and source telemetry."""

        return _missile_sample(self.sim_time_s, self.missile, self.definition)
    ####

    def target_sample(self) -> Agm6PointMassSample:
        """Return current committed TARGET3 state."""

        return _point_sample(self.sim_time_s, self.target, actor="TARGET3", alive=self.target_alive)
    ####

    def aircraft_sample(self) -> Agm6PointMassSample:
        """Return current committed AIRCRAFT3 state."""

        return _point_sample(self.sim_time_s, self.aircraft, actor="AIRCRAFT3", alive=self.aircraft_alive)
    ####

    def native_sensor_context(self):
        """Project committed missile/TARGET3 geometry into Taoryx sensor context."""

        return cadac_local_ned_sensor_context(
            time_s=self.sim_time_s,
            host_position_ned_m=self.missile.position_ned_m,
            host_velocity_ned_mps=self.missile.velocity_ned_mps,
            target_id="agm6-target",
            target_position_ned_m=self.target.position_ned_m,
            target_velocity_ned_mps=self.target.velocity_ned_mps,
            body_from_local=_dcm_body_from_local(self.missile.quaternion_wxyz),
        )
    ####

    def advance(self, duration_s: float) -> tuple[Agm6EventTrace, ...]:
        """Advance across an integral number of exact source timesteps."""

        steps = _agm6_session_step_count(duration_s, self.definition.integration_step_s)
        traces: list[Agm6EventTrace] = []
        for _ in range(steps):
            if self.completed:
                break
            ####
            traces.extend(self._advance_one())
        ####
        return tuple(traces)
    ####

    def _advance_one(self) -> tuple[Agm6EventTrace, ...]:
        """Commit one source vehicle-major scheduler pass without external feedback."""

        sim_time = self.sim_time_s
        dt_s = self.definition.integration_step_s
        traces: list[Agm6EventTrace] = []
        self.executed_steps += 1

        if self.missile_alive and sim_time + _SMALL >= self.missile.launch_epoch_s:
            self.missile.launch_time_s = sim_time - self.missile.launch_epoch_s
            self.missile.event_time_s = sim_time - self.missile.event_epoch_s
            application = _apply_missile_event(self.missile, self.missile_events, sim_time)
            if application is not None:
                traces.append(_event_trace(application, sim_time, "MISSILE6"))
            ####
            for module in self.definition.module_order:
                if module == "environment":
                    _missile_environment(self.missile, self.definition, dt_s, self.rng)
                elif module == "kinematics":
                    _missile_kinematics(self.missile, self.definition, dt_s)
                elif module == "aerodynamics":
                    self.missile.coefficients = agm6_aerodynamic_coefficients(self.definition, self.missile)
                    _update_source_stop(self.missile, self.definition)
                elif module == "propulsion":
                    self.missile.propulsion = agm6_propulsion_step(
                        self.definition,
                        self.missile.propulsion,
                        pressure_pa=self.missile.pressure_pa,
                        dt_s=dt_s,
                        mode=self.missile.propulsion_mode,
                    )
                    self.missile.propulsion_mode = self.missile.propulsion.mode
                elif module == "forces":
                    self.missile.wrench = agm6_body_wrench(self.definition, self.missile)
                elif module == "ins":
                    _missile_ins(self.missile)
                elif module == "datalink":
                    _missile_datalink(self.missile, self.aircraft_packet)
                elif module == "sensor":
                    _missile_sensor(self.missile, self.definition.sensor, self.target_packet, sim_time, dt_s)
                elif module == "guidance":
                    _missile_guidance(self.missile, self.definition.guidance, self.target_packet, self.aircraft_packet)
                elif module == "control":
                    self.missile.requested_control = _missile_control(self.missile, self.definition.control, dt_s)
                elif module == "actuator":
                    self.missile.actuator_step = agm6_actuator_step(
                        self.definition.actuator,
                        self.missile.fin_state,
                        self.missile.requested_control,
                        dt_s,
                    )
                    self.missile.fin_state = self.missile.actuator_step.state
                elif module == "euler":
                    _missile_euler(self.missile, self.definition, dt_s)
                elif module == "newton":
                    _missile_newton(self.missile, dt_s)
                elif module == "intercept":
                    self.intercept = _missile_intercept(self.missile, self.definition, self.target_packet, sim_time, dt_s)
                    if self.intercept is not None:
                        self.missile_alive = False
                        self.terminated_reason = "intercept"
                    elif self.missile.altitude_m <= 0.0:
                        self.missile_alive = False
                        self.terminated_reason = "ground_impact"
                    elif self.missile.source_stop and self.definition.stop_on_termination:
                        self.missile_alive = False
                        self.terminated_reason = "source_stop"
                    ####
                ####
            ####
        ####

        if self.target_alive and sim_time + _SMALL >= self.definition.target.launch_delay_s:
            target_time = sim_time - self.definition.target.launch_delay_s
            application, self.target_event_epoch_s = _apply_point_event(
                self.target,
                self.target_events,
                actor="TARGET3",
                time_s=target_time,
                event_epoch_s=self.target_event_epoch_s,
            )
            if application is not None:
                traces.append(_event_trace(application, sim_time, "TARGET3"))
            ####
            for module in self.definition.module_order:
                if module == "environment":
                    _point_environment(self.target)
                elif module == "forces":
                    _ground_target_forces(self.target)
                elif module == "newton":
                    _point_newton(self.target, dt_s)
                ####
            ####
            self.target_packet = _point_packet(self.target, alive=self.target_alive)
        ####

        if self.aircraft_alive and sim_time + _SMALL >= self.definition.aircraft.launch_delay_s:
            aircraft_time = sim_time - self.definition.aircraft.launch_delay_s
            application, self.aircraft_event_epoch_s = _apply_point_event(
                self.aircraft,
                self.aircraft_events,
                actor="AIRCRAFT3",
                time_s=aircraft_time,
                event_epoch_s=self.aircraft_event_epoch_s,
            )
            if application is not None:
                traces.append(_event_trace(application, sim_time, "AIRCRAFT3"))
            ####
            for module in self.definition.module_order:
                if module == "environment":
                    _point_environment(self.aircraft)
                elif module == "forces":
                    _aircraft_forces(self.aircraft)
                elif module == "sensor":
                    _aircraft_sensor_track(
                        self.aircraft,
                        self.target_packet,
                        self.tracking,
                        self.definition.tracking,
                        self.rng,
                        sim_time,
                    )
                elif module == "guidance":
                    _aircraft_guidance(self.aircraft, self.target_packet)
                elif module == "control":
                    _aircraft_control(self.aircraft, self.definition.aircraft, dt_s)
                elif module == "newton":
                    _point_newton(self.aircraft, dt_s)
                ####
            ####
            self.aircraft_packet = _aircraft_packet(self.aircraft, self.tracking, alive=self.aircraft_alive)
        ####

        if not _missile_is_finite(self.missile) or not _point_is_finite(self.target) or not _point_is_finite(self.aircraft):
            self.terminated_reason = "nonfinite_state"
            return tuple(traces)
        ####
        if self.terminated_reason is not None:
            return tuple(traces)
        ####
        self.sim_time_s += dt_s
        if self.sim_time_s + 0.5 * dt_s >= self.definition.end_time_s:
            self.terminated_reason = "end_time"
        ####
        return tuple(traces)
    ####


####


def _agm6_session_step_count(duration_s: float, source_step_s: float) -> int:
    """Validate an external hold against the exact AGM6 source timestep."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("AGM6 session duration_s must be positive and finite")
    ####
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"AGM6 session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    ####
    return steps


####


def lower_agm6_source_bundle(bundle: CadacSourceBundle) -> Agm6SourceDefinition:
    """Lower one default-shape AGM6 three-actor case into typed inputs."""

    missiles = bundle.case.vehicles_named("MISSILE6")
    targets = bundle.case.vehicles_named("TARGET3")
    aircraft = bundle.case.vehicles_named("AIRCRAFT3")
    if len(missiles) != 1 or len(targets) != 1 or len(aircraft) != 1 or len(bundle.case.vehicles) != 3:
        raise Agm6SourceError(
            "AGM6 lowering requires one MISSILE6, one TARGET3, and one AIRCRAFT3; "
            f"found {len(missiles)}, {len(targets)}, {len(aircraft)} in {len(bundle.case.vehicles)} actors"
        )
    ####
    vehicle_order = tuple(vehicle.model_name.upper() for vehicle in bundle.case.vehicles)
    if vehicle_order != ("MISSILE6", "TARGET3", "AIRCRAFT3"):
        raise Agm6SourceError("AGM6 source-compatible bus lag requires MISSILE6, TARGET3, AIRCRAFT3 order")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _AGM6_MODULES)
    if unknown:
        raise Agm6SourceError(f"AGM6 reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _AGM6_MODULES if name not in module_order)
    if missing:
        raise Agm6SourceError(f"AGM6 reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Agm6SourceError("AGM6 source case must declare TIMING int_step") from error
    ####
    missile = missiles[0]
    target = targets[0]
    tracking_aircraft = aircraft[0]
    weather_bindings = tuple(binding for binding in bundle.decks_for("MISSILE6") if binding.kind is CadacDeckKind.GENERIC)
    if len(weather_bindings) != 1:
        raise Agm6SourceError(f"AGM6 source case requires one WEATHER_DECK; found {len(weather_bindings)}")
    ####
    environment = Agm6EnvironmentConfig(
        mode=_integer(missile, "mair", 0),
        constant_wind_speed_mps=_number(missile, "dvae", 0.0),
        wind_direction_deg=_number(missile, "psiwdx", 0.0),
        vertical_wind_mps=_number(missile, "vaed3", 0.0),
        wind_time_constant_s=_number(missile, "twind", 0.1),
        turbulence_length_m=_number(missile, "turb_length", 100.0),
        turbulence_sigma_mps=_number(missile, "turb_sigma", 0.0),
        rayleigh_wind_mode_mps=_stochastic_parameter(
            missile,
            "dvae",
            CadacStochasticKind.RAYLEIGH,
            parameter_index=0,
        ),
    )
    return Agm6SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        monte_carlo_seed=bundle.case.monte_carlo.seed if bundle.case.monte_carlo is not None else 0,
        module_order=module_order,
        vehicle_order=("MISSILE6", "TARGET3", "AIRCRAFT3"),
        initial_state=Agm6InitialState(
            position_ned_m=(
                _number(missile, "sbel1"),
                _number(missile, "sbel2"),
                _number(missile, "sbel3"),
            ),
            speed_mps=_number(missile, "dvbe"),
            yaw_deg=_number(missile, "psiblx"),
            pitch_deg=_number(missile, "thtblx"),
            roll_deg=_number(missile, "phiblx"),
            alpha_deg=_number(missile, "alpha0x", 0.0),
            beta_deg=_number(missile, "beta0x", 0.0),
            body_rates_deg_s=(
                _number(missile, "ppx", 0.0),
                _number(missile, "qqx", 0.0),
                _number(missile, "rrx", 0.0),
            ),
            launch_delay_s=_number(missile, "launch_delay", 0.0),
        ),
        airframe=Agm6Airframe(
            launch_mass_kg=_number(missile, "vmass0"),
            launch_roll_inertia_kg_m2=_number(missile, "ai11"),
            launch_pitch_inertia_kg_m2=_number(missile, "ai33"),
        ),
        aerodynamics=Agm6AeroLimits(
            alpha_limit_deg=_number(missile, "alplimx"),
            structural_acceleration_limit_g=_number(missile, "alimit"),
            minimum_mach=_number(missile, "trmach", 0.4),
            minimum_dynamic_pressure_pa=_number(missile, "trdynm", 10_000.0),
            minimum_load_capacity_g=_number(missile, "trload", 0.5),
            maximum_total_incidence_rad=_number(missile, "tralp", 1.0),
            maximum_quaternion_error=_number(missile, "trortho", 1.0e-4),
        ),
        propulsion=Agm6PropulsionConfig(
            initial_mode=_integer(missile, "mprop", 0),
            nozzle_exit_area_m2=_number(missile, "aexit", 0.0),
            specific_impulse_s=_number(missile, "spi"),
            sea_level_thrust_n=_number(missile, "thrsl"),
            throttle=_number(missile, "throtl", 1.0),
            initial_fuel_mass_kg=_number(missile, "fmass0"),
        ),
        actuator=Agm6ActuatorConfig(
            mode=_integer(missile, "mact", 0),
            position_limit_deg=_number(missile, "dlimx"),
            rate_limit_deg_s=_number(missile, "ddlimx"),
            natural_frequency_rad_s=_number(missile, "wnact"),
            damping_ratio=_number(missile, "zetact"),
        ),
        control=Agm6ControlConfig(
            initial_mode=_integer(missile, "maut", 0),
            acceleration_natural_frequency_rad_s=_number(missile, "wacl", 2.0),
            acceleration_damping_ratio=_number(missile, "zacl", 0.7),
            acceleration_real_pole_rad_s=_number(missile, "pacl", 10.0),
            structural_limit_g=_number(missile, "alimit"),
            pitch_command_limit_deg=_number(missile, "dqlimx"),
            yaw_command_limit_deg=_number(missile, "drlimx"),
            roll_command_limit_deg=_number(missile, "dplimx"),
            commanded_roll_deg=_number(missile, "phicomx", 0.0),
            roll_natural_frequency_rad_s=_number(missile, "wrcl"),
            roll_damping_ratio=_number(missile, "zrcl"),
            acceleration_feedforward_gain_s2_m=_number(missile, "gainp", 0.0),
            rate_loop_damping_ratio=_number(missile, "zetlagr"),
            pitch_rate_command_deg_s=_number(missile, "qqcomx", 0.0),
            yaw_rate_command_deg_s=_number(missile, "rrcomx", 0.0),
        ),
        sensor=Agm6SensorConfig(
            initial_mode=_integer(missile, "mseek", 0),
            dynamic_mode=_integer(missile, "skr_dyn", 0),
            blind_range_m=_number(missile, "dblind", 0.0),
            acquisition_range_m=_number(missile, "racq", 1.0e9),
            acquisition_time_s=_number(missile, "dtimac", 0.0),
            filter_gain_per_s=_number(missile, "gk", 0.0),
            filter_damping_ratio=_number(missile, "zetak", 0.0),
            filter_natural_frequency_rad_s=_number(missile, "wnk", 1.0),
            yaw_half_fov_rad=_number(missile, "fovyaw", math.pi),
            pitch_half_fov_rad=_number(missile, "fovpitch", math.pi),
            target_number=_integer(missile, "tgt_num", 1),
            maximum_pitch_gimbal_rad=_number(missile, "trtht", math.pi / 2.0),
            maximum_pitch_gimbal_rate_rad_s=_number(missile, "trthtd", 10.0),
            maximum_roll_gimbal_rate_rad_s=_number(missile, "trphid", 14.0),
            maximum_tracking_error_rad=_number(missile, "trate", 1.0),
        ),
        guidance=Agm6GuidanceConfig(
            initial_mode=_integer(missile, "mguid", 0),
            navigation_gain=_number(missile, "gnav", 0.0),
            gravity_bias_g=_number(missile, "grav_bias", 0.0),
            line_gain_per_s=_number(missile, "line_gain", 0.0),
            nonlinear_gain_factor=_number(missile, "nl_gain_fact", 0.0),
            distance_decrement_m=_number(missile, "decrement", 1.0),
            vertical_line_of_attack_deg=_number(missile, "thtflx", 0.0),
        ),
        environment=environment,
        target=Agm6GroundTargetConfig(
            position_ned_m=(
                _number(target, "sael1"),
                _number(target, "sael2"),
                _number(target, "sael3"),
            ),
            speed_mps=_number(target, "dvae"),
            heading_deg=_number(target, "psivlx"),
            flight_path_deg=_number(target, "thtvlx", 0.0),
            longitudinal_acceleration_g=_number(target, "acc_longx", 0.0),
            lateral_acceleration_g=_number(target, "acc_latx", 0.0),
            launch_delay_s=_number(target, "launch_delay", 0.0),
        ),
        aircraft=Agm6AircraftConfig(
            position_ned_m=(
                _number(tracking_aircraft, "sael1"),
                _number(tracking_aircraft, "sael2"),
                _number(tracking_aircraft, "sael3"),
            ),
            speed_mps=_number(tracking_aircraft, "dvae"),
            heading_deg=_number(tracking_aircraft, "psivlx"),
            flight_path_deg=_number(tracking_aircraft, "thtvlx"),
            aircraft_option=_integer(tracking_aircraft, "acft_option", 0),
            guidance_gain=_number(tracking_aircraft, "guid_gain", 0.0),
            turn_g=_number(tracking_aircraft, "gturn", 0.0),
            bank_time_constant_s=_number(tracking_aircraft, "tphi", 0.1),
            bank_limit_deg=_number(tracking_aircraft, "philimx", 120.0),
            normal_load_time_constant_s=_number(tracking_aircraft, "tanx", 0.1),
            alpha_limit_deg=_number(tracking_aircraft, "alplimx", 40.0),
            lift_slope_per_deg=_number(tracking_aircraft, "clalpha", 0.0523),
            wing_loading_n_m2=_number(tracking_aircraft, "wingloading", 3247.0),
            longitudinal_acceleration_g=_number(tracking_aircraft, "acc_longx", 0.0),
            launch_delay_s=_number(tracking_aircraft, "launch_delay", 0.0),
        ),
        tracking=Agm6TrackingConfig(
            track_step_s=_number(tracking_aircraft, "track_step", 1.0),
            range_sigma_m=_number(tracking_aircraft, "dat_sigma", 0.0),
            azimuth_sigma_rad=_number(tracking_aircraft, "azat_sigma", 0.0),
            elevation_sigma_rad=_number(tracking_aircraft, "elat_sigma", 0.0),
            velocity_sigma_mps=_number(tracking_aircraft, "vel_sigma", 0.0),
        ),
        ins_mode_requested=_integer(missile, "mins", 0),
        stop_on_termination=bool(_integer(missile, "stop", 0)),
        target_plane_yaw_deg=_number(missile, "psiplx", 0.0),
        target_plane_pitch_deg=_number(missile, "thtplx", 0.0),
        # AGM6's source intercept implementation uses a literal 100 m sphere.
        # ``critmax`` is documented and plotted, but it does not participate in
        # the target-plane crossing condition in the pinned source revision.
        target_sphere_radius_m=100.0,
        aerodynamic_deck=bundle.deck_for("MISSILE6", CadacDeckKind.AERODYNAMIC),
        weather_deck=weather_bindings[0].deck,
        missile_events=missile.events,
        target_events=target.events,
        aircraft_events=tracking_aircraft.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_agm6_source_definition(path: str | Path) -> Agm6SourceDefinition:
    """Parse, fingerprint, and lower one AGM6 source case."""

    return lower_agm6_source_bundle(load_cadac_source_bundle(path))


####


def agm6_initial_quaternion(initial: Agm6InitialState) -> tuple[float, float, float, float]:
    """Return the source scalar-first quaternion from yaw, pitch, and roll."""

    psi = initial.yaw_deg * _RAD_PER_DEG
    theta = initial.pitch_deg * _RAD_PER_DEG
    phi = initial.roll_deg * _RAD_PER_DEG
    spsi, cpsi = math.sin(psi / 2.0), math.cos(psi / 2.0)
    stheta, ctheta = math.sin(theta / 2.0), math.cos(theta / 2.0)
    sphi, cphi = math.sin(phi / 2.0), math.cos(phi / 2.0)
    return (
        cpsi * ctheta * cphi + spsi * stheta * sphi,
        cpsi * ctheta * sphi - spsi * stheta * cphi,
        cpsi * stheta * cphi + spsi * ctheta * sphi,
        -cpsi * stheta * sphi + spsi * ctheta * cphi,
    )


####


def agm6_actuator_step(
    config: Agm6ActuatorConfig,
    state: Agm6FinActuatorState,
    command: Agm6ControlCommand,
    dt_s: float,
) -> Agm6ActuatorStep:
    """Advance AGM6's source-identical four-fin actuator path."""

    return sraam6_actuator_step(config, state, command, dt_s)


####


def agm6_propulsion_step(
    definition: Agm6SourceDefinition,
    previous: Agm6PropulsionStep,
    *,
    pressure_pa: float,
    dt_s: float,
    mode: int,
) -> Agm6PropulsionStep:
    """Advance source fuel expenditure and pressure-corrected thrust."""

    config = definition.propulsion
    fuel_flow_new = 0.0
    thrust = 0.0
    resolved_mode = mode
    if resolved_mode == 1:
        fuel_flow_new = config.sea_level_thrust_n * config.throttle / (config.specific_impulse_s * 9.81)
        thrust = config.sea_level_thrust_n * config.throttle + (config.sea_level_pressure_pa - pressure_pa) * config.nozzle_exit_area_m2
    ####
    fuel_expended = _integrate_scalar(
        previous.fuel_expended_kg,
        fuel_flow_new,
        previous.fuel_flow_kg_s,
        dt_s,
    )
    fuel_expended = max(0.0, fuel_expended)
    remaining = max(0.0, config.initial_fuel_mass_kg - fuel_expended)
    mass = max(1.0e-6, definition.airframe.launch_mass_kg - fuel_expended)
    if fuel_expended >= config.initial_fuel_mass_kg:
        resolved_mode = 0
        thrust = 0.0
        fuel_flow_new = 0.0
    ####
    return Agm6PropulsionStep(
        mode=resolved_mode,
        thrust_n=max(0.0, thrust),
        mass_kg=mass,
        fuel_expended_kg=fuel_expended,
        fuel_remaining_kg=remaining,
        fuel_flow_kg_s=fuel_flow_new,
    )


####


def agm6_aerodynamic_coefficients(
    definition: Agm6SourceDefinition,
    runtime: _MissileRuntime,
) -> Agm6AeroCoefficients:
    """Evaluate AGM6 aeroballistic tables and body-axis coefficient closure."""

    deck = definition.aerodynamic_deck
    mach = runtime.mach
    alpha_total = runtime.total_alpha_deg
    phi = runtime.aerodynamic_roll_rad
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    sin4phi = math.sin(4.0 * phi)
    sin2phi_squared = math.sin(2.0 * phi) ** 2
    speed = max(1.0e-9, runtime.airspeed_mps)
    roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s = runtime.body_rates_rad_s * _DEG_PER_RAD
    achieved = runtime.actuator_step.achieved_control
    pitch_control_aero = achieved.pitch_deg * cphi - achieved.yaw_deg * sphi
    yaw_control_aero = achieved.pitch_deg * sphi + achieved.yaw_deg * cphi
    pitch_rate_aero_deg_s = pitch_rate_deg_s * cphi - yaw_rate_deg_s * sphi
    yaw_rate_aero_deg_s = pitch_rate_deg_s * sphi + yaw_rate_deg_s * cphi

    ca0 = deck.table("ca0_vs_mach").interpolate((mach,))
    caa = deck.table("caa_vs_mach").interpolate((mach,))
    cad = deck.table("cad_vs_mach").interpolate((mach,))
    effective_control = (abs(pitch_control_aero) + abs(yaw_control_aero)) / 2.0
    axial = ca0 + caa * alpha_total + cad * effective_control * effective_control

    cyp = deck.table("cyp_vs_mach_alpha").interpolate((mach, alpha_total))
    control_normal = deck.table("cndq_vs_mach").interpolate((mach,))
    side_aero = cyp * sin4phi + control_normal * yaw_control_aero
    cn0 = deck.table("cn0_vs_mach_alpha").interpolate((mach, alpha_total))
    cnp = deck.table("cnp_vs_mach_alpha").interpolate((mach, alpha_total))
    normal_aero = cn0 + cnp * sin2phi_squared + control_normal * pitch_control_aero

    cllap = deck.table("cllap_vs_mach").interpolate((mach,))
    cllp = deck.table("cllp_vs_mach").interpolate((mach,))
    clldp = deck.table("clldp_vs_mach").interpolate((mach,))
    roll_moment = (
        cllap * alpha_total * alpha_total * sin4phi
        + cllp * roll_rate_deg_s * definition.airframe.reference_length_m / (2.0 * speed)
        + clldp * achieved.roll_deg
    )

    clm0 = deck.table("clm0_vs_mach_alpha").interpolate((mach, alpha_total))
    clmp = deck.table("clmp_vs_mach_alpha").interpolate((mach, alpha_total))
    clmq = deck.table("clmq_vs_mach").interpolate((mach,))
    clmdq = deck.table("clmdq_vs_mach").interpolate((mach,))
    pitch_moment = (
        clm0 + clmp * sin2phi_squared + clmq * pitch_rate_aero_deg_s * definition.airframe.reference_length_m / (2.0 * speed) + clmdq * pitch_control_aero
    )
    clnp = deck.table("clnp_vs_mach_alpha").interpolate((mach, alpha_total))
    yaw_moment = clnp * sin4phi + clmq * yaw_rate_aero_deg_s * definition.airframe.reference_length_m / (2.0 * speed) + clmdq * yaw_control_aero
    side = side_aero * cphi - normal_aero * sphi
    normal = side_aero * sphi + normal_aero * cphi
    pitch_body = pitch_moment * cphi + yaw_moment * sphi
    yaw_body = yaw_moment * cphi - pitch_moment * sphi

    normal_alpha_derivative = runtime.coefficients.normal_alpha_derivative_mps2
    normal_control_derivative = runtime.coefficients.normal_control_derivative_mps2
    pitch_alpha_derivative = runtime.coefficients.pitch_alpha_derivative_rad_s2
    pitch_rate_derivative = runtime.coefficients.pitch_rate_derivative_per_s
    pitch_control_derivative = runtime.coefficients.pitch_control_derivative_rad_s2
    roll_rate_derivative = runtime.coefficients.roll_rate_derivative_per_s
    roll_control_derivative = runtime.coefficients.roll_control_derivative_rad_s2
    if alpha_total < definition.aerodynamics.alpha_limit_deg - 3.0:
        alpha_plus = max(3.0, alpha_total + 3.0)
        alpha_minus = max(0.0, alpha_total - 3.0)
        span = max(1.0e-9, alpha_plus - alpha_minus)
        normal_plus = deck.table("cn0_vs_mach_alpha").interpolate((mach, alpha_plus))
        normal_minus = deck.table("cn0_vs_mach_alpha").interpolate((mach, alpha_minus))
        pitch_plus = deck.table("clm0_vs_mach_alpha").interpolate((mach, alpha_plus))
        pitch_minus = deck.table("clm0_vs_mach_alpha").interpolate((mach, alpha_minus))
        normal_alpha_per_rad = (normal_plus - normal_minus) / span * _DEG_PER_RAD
        pitch_alpha_per_rad = (pitch_plus - pitch_minus) / span * _DEG_PER_RAD
        normal_control_per_rad = control_normal * _DEG_PER_RAD
        pitch_control_per_rad = clmdq * _DEG_PER_RAD
        pitch_rate_per_rad = clmq * _DEG_PER_RAD
        roll_rate_per_rad = cllp * _DEG_PER_RAD
        roll_control_per_rad = clldp * _DEG_PER_RAD
        q_area_over_mass = runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2 / runtime.propulsion.mass_kg
        normal_alpha_derivative = q_area_over_mass * normal_alpha_per_rad
        normal_control_derivative = q_area_over_mass * normal_control_per_rad
        pitch_scale = (
            runtime.dynamic_pressure_pa
            * definition.airframe.reference_area_m2
            * definition.airframe.reference_length_m
            / definition.airframe.launch_pitch_inertia_kg_m2
        )
        pitch_alpha_derivative = pitch_scale * pitch_alpha_per_rad
        pitch_rate_derivative = pitch_scale * definition.airframe.reference_length_m / (2.0 * speed) * pitch_rate_per_rad
        pitch_control_derivative = pitch_scale * pitch_control_per_rad
        roll_scale = (
            runtime.dynamic_pressure_pa
            * definition.airframe.reference_area_m2
            * definition.airframe.reference_length_m
            / definition.airframe.launch_roll_inertia_kg_m2
        )
        roll_rate_derivative = roll_scale * definition.airframe.reference_length_m / (2.0 * speed) * roll_rate_per_rad
        roll_control_derivative = roll_scale * roll_control_per_rad
    ####
    max_normal = deck.table("cn0_vs_mach_alpha").interpolate((mach, definition.aerodynamics.alpha_limit_deg))
    weight = runtime.propulsion.mass_kg * _AGRAV
    max_g = max_normal * runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2 / weight
    max_g = max(0.0, min(max_g, definition.aerodynamics.structural_acceleration_limit_g))
    return Agm6AeroCoefficients(
        axial=axial,
        side=side,
        normal=normal,
        roll_moment=roll_moment,
        pitch_moment=pitch_body,
        yaw_moment=yaw_body,
        max_acceleration_g=max_g,
        normal_alpha_derivative_mps2=normal_alpha_derivative,
        normal_control_derivative_mps2=normal_control_derivative,
        pitch_alpha_derivative_rad_s2=pitch_alpha_derivative,
        pitch_rate_derivative_per_s=pitch_rate_derivative,
        pitch_control_derivative_rad_s2=pitch_control_derivative,
        roll_rate_derivative_per_s=roll_rate_derivative,
        roll_control_derivative_rad_s2=roll_control_derivative,
    )


####


def agm6_body_wrench(definition: Agm6SourceDefinition, runtime: _MissileRuntime) -> Agm6BodyWrench:
    """Close aerodynamic coefficients and motor thrust into the body wrench."""

    q_area = runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2
    coefficient = runtime.coefficients
    force = (
        -q_area * coefficient.axial + runtime.propulsion.thrust_n,
        q_area * coefficient.side,
        -q_area * coefficient.normal,
    )
    moment_scale = q_area * definition.airframe.reference_length_m
    moment = (
        moment_scale * coefficient.roll_moment,
        moment_scale * coefficient.pitch_moment,
        moment_scale * coefficient.yaw_moment,
    )
    return Agm6BodyWrench(force_n=force, moment_nm=moment)


####


def run_agm6_source_compatibility(
    definition: Agm6SourceDefinition,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
    initial_state: Agm6InitialState | None = None,
    target_config: Agm6GroundTargetConfig | None = None,
    aircraft_config: Agm6AircraftConfig | None = None,
    random_seed: int | None = None,
) -> Agm6RunResult:
    """Execute the AGM6 missile, moving target, and tracking aircraft in source order."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    cadence = sample_step_s if sample_step_s is not None else definition.plot_step_s or max(dt_s, 0.02)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("AGM6 end_time_s must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("AGM6 sample_step_s must be positive and finite")
    ####

    seed = definition.monte_carlo_seed if random_seed is None else int(random_seed)
    rng = np.random.default_rng(seed)
    target_definition = target_config or definition.target
    aircraft_definition = aircraft_config or definition.aircraft
    missile = _initialize_missile(definition, initial_state or definition.initial_state, rng)
    target = _initialize_point_mass_target(target_definition)
    aircraft = _initialize_aircraft(aircraft_definition)

    target_packet = _point_packet(target, alive=True)
    tracking = _TrackingRuntime()
    aircraft_packet = _aircraft_packet(aircraft, tracking, alive=True)
    missile_events = CadacEventCursor.from_events(definition.missile_events)
    target_events = CadacEventCursor.from_events(definition.target_events)
    aircraft_events = CadacEventCursor.from_events(definition.aircraft_events)
    target_event_epoch_s = 0.0
    aircraft_event_epoch_s = 0.0

    samples: list[Agm6Sample] = []
    target_samples: list[Agm6PointMassSample] = []
    aircraft_samples: list[Agm6PointMassSample] = []
    track_samples: list[Agm6TrackSample] = [_track_sample(tracking.track)]
    event_trace: list[Agm6EventTrace] = []
    intercept: Agm6Intercept | None = None
    missile_alive = True
    target_alive = True
    aircraft_alive = True
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0
    terminated_reason: Literal[
        "intercept",
        "ground_impact",
        "source_stop",
        "nonfinite_state",
        "end_time",
    ] = "end_time"

    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1

        if missile_alive and sim_time + _SMALL >= missile.launch_epoch_s:
            missile.launch_time_s = sim_time - missile.launch_epoch_s
            missile.event_time_s = sim_time - missile.event_epoch_s
            application = _apply_missile_event(missile, missile_events, sim_time)
            if application is not None:
                event_trace.append(_event_trace(application, sim_time, "MISSILE6"))
            ####

            for module in definition.module_order:
                if module == "environment":
                    _missile_environment(missile, definition, dt_s, rng)
                elif module == "kinematics":
                    _missile_kinematics(missile, definition, dt_s)
                elif module == "aerodynamics":
                    missile.coefficients = agm6_aerodynamic_coefficients(definition, missile)
                    _update_source_stop(missile, definition)
                elif module == "propulsion":
                    missile.propulsion = agm6_propulsion_step(
                        definition,
                        missile.propulsion,
                        pressure_pa=missile.pressure_pa,
                        dt_s=dt_s,
                        mode=missile.propulsion_mode,
                    )
                    missile.propulsion_mode = missile.propulsion.mode
                elif module == "forces":
                    missile.wrench = agm6_body_wrench(definition, missile)
                elif module == "ins":
                    _missile_ins(missile)
                elif module == "datalink":
                    _missile_datalink(missile, aircraft_packet)
                elif module == "sensor":
                    _missile_sensor(missile, definition.sensor, target_packet, sim_time, dt_s)
                elif module == "guidance":
                    _missile_guidance(
                        missile,
                        definition.guidance,
                        target_packet,
                        aircraft_packet,
                    )
                elif module == "control":
                    missile.requested_control = _missile_control(missile, definition.control, dt_s)
                elif module == "actuator":
                    missile.actuator_step = agm6_actuator_step(
                        definition.actuator,
                        missile.fin_state,
                        missile.requested_control,
                        dt_s,
                    )
                    missile.fin_state = missile.actuator_step.state
                elif module == "euler":
                    _missile_euler(missile, definition, dt_s)
                elif module == "newton":
                    _missile_newton(missile, dt_s)
                elif module == "intercept":
                    intercept = _missile_intercept(
                        missile,
                        definition,
                        target_packet,
                        sim_time,
                        dt_s,
                    )
                    if intercept is not None:
                        missile_alive = False
                        terminated_reason = "intercept"
                    elif missile.altitude_m <= 0.0:
                        missile_alive = False
                        terminated_reason = "ground_impact"
                    elif missile.source_stop and definition.stop_on_termination:
                        missile_alive = False
                        terminated_reason = "source_stop"
                    ####
                ####
            ####
        ####

        if target_alive and sim_time + _SMALL >= target_definition.launch_delay_s:
            target_time = sim_time - target_definition.launch_delay_s
            application, target_event_epoch_s = _apply_point_event(
                target,
                target_events,
                actor="TARGET3",
                time_s=target_time,
                event_epoch_s=target_event_epoch_s,
            )
            if application is not None:
                event_trace.append(_event_trace(application, sim_time, "TARGET3"))
            ####
            for module in definition.module_order:
                if module == "environment":
                    _point_environment(target)
                elif module == "forces":
                    _ground_target_forces(target)
                elif module == "newton":
                    _point_newton(target, dt_s)
                ####
            ####
            target_packet = _point_packet(target, alive=target_alive)
        ####

        previous_track_sequence = tracking.update_sequence
        if aircraft_alive and sim_time + _SMALL >= aircraft_definition.launch_delay_s:
            aircraft_time = sim_time - aircraft_definition.launch_delay_s
            application, aircraft_event_epoch_s = _apply_point_event(
                aircraft,
                aircraft_events,
                actor="AIRCRAFT3",
                time_s=aircraft_time,
                event_epoch_s=aircraft_event_epoch_s,
            )
            if application is not None:
                event_trace.append(_event_trace(application, sim_time, "AIRCRAFT3"))
            ####
            for module in definition.module_order:
                if module == "environment":
                    _point_environment(aircraft)
                elif module == "forces":
                    _aircraft_forces(aircraft)
                elif module == "sensor":
                    _aircraft_sensor_track(
                        aircraft,
                        target_packet,
                        tracking,
                        definition.tracking,
                        rng,
                        sim_time,
                    )
                elif module == "guidance":
                    _aircraft_guidance(aircraft, target_packet)
                elif module == "control":
                    _aircraft_control(aircraft, aircraft_definition, dt_s)
                elif module == "newton":
                    _point_newton(aircraft, dt_s)
                ####
            ####
            aircraft_packet = _aircraft_packet(aircraft, tracking, alive=aircraft_alive)
        ####
        if tracking.update_sequence != previous_track_sequence:
            track_samples.append(_track_sample(tracking.track))
        ####

        should_sample = sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end or not missile_alive
        if should_sample:
            samples.append(_missile_sample(sim_time, missile, definition))
            target_samples.append(_point_sample(sim_time, target, actor="TARGET3", alive=target_alive))
            aircraft_samples.append(_point_sample(sim_time, aircraft, actor="AIRCRAFT3", alive=aircraft_alive))
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####

        if not _missile_is_finite(missile) or not _point_is_finite(target) or not _point_is_finite(aircraft):
            terminated_reason = "nonfinite_state"
            if not samples or samples[-1].time_s != sim_time:
                samples.append(_missile_sample(sim_time, missile, definition))
                target_samples.append(_point_sample(sim_time, target, actor="TARGET3", alive=target_alive))
                aircraft_samples.append(_point_sample(sim_time, aircraft, actor="AIRCRAFT3", alive=aircraft_alive))
            ####
            break
        ####
        if not missile_alive:
            break
        ####
        sim_time += dt_s
    ####

    return Agm6RunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
        target_samples=tuple(target_samples),
        aircraft_samples=tuple(aircraft_samples),
        track_samples=tuple(track_samples),
        intercept=intercept,
        event_trace=tuple(event_trace),
    )


####


def _initialize_missile(
    definition: Agm6SourceDefinition,
    initial: Agm6InitialState,
    rng: np.random.Generator,
) -> _MissileRuntime:
    quaternion = np.asarray(agm6_initial_quaternion(initial), dtype=np.float64)
    transform = _dcm_body_from_local(quaternion)
    alpha = initial.alpha_deg * _RAD_PER_DEG
    beta = initial.beta_deg * _RAD_PER_DEG
    body_velocity = np.asarray(
        (
            math.cos(alpha) * math.cos(beta) * initial.speed_mps,
            math.sin(beta) * initial.speed_mps,
            math.sin(alpha) * math.cos(beta) * initial.speed_mps,
        ),
        dtype=np.float64,
    )
    local_velocity = transform.T @ body_velocity
    zero_control = Agm6ControlCommand()
    zero_actuator = Agm6ActuatorStep(
        requested_control=zero_control,
        requested_fins=Agm6FinSet(),
        achieved_fins=Agm6FinSet(),
        achieved_control=zero_control,
        state=Agm6FinActuatorState(),
        position_limited=(False, False, False, False),
        rate_limited=(False, False, False, False),
    )
    zero_coefficients = Agm6AeroCoefficients(
        axial=0.0,
        side=0.0,
        normal=0.0,
        roll_moment=0.0,
        pitch_moment=0.0,
        yaw_moment=0.0,
        max_acceleration_g=0.0,
        normal_alpha_derivative_mps2=0.0,
        normal_control_derivative_mps2=0.0,
        pitch_alpha_derivative_rad_s2=0.0,
        pitch_rate_derivative_per_s=0.0,
        pitch_control_derivative_rad_s2=0.0,
        roll_rate_derivative_per_s=0.0,
        roll_control_derivative_rad_s2=0.0,
    )
    propulsion = Agm6PropulsionStep(
        mode=definition.propulsion.initial_mode,
        thrust_n=0.0,
        mass_kg=definition.airframe.launch_mass_kg,
        fuel_expended_kg=0.0,
        fuel_remaining_kg=definition.propulsion.initial_fuel_mass_kg,
        fuel_flow_kg_s=0.0,
    )
    wind_speed = definition.environment.constant_wind_speed_mps
    if definition.environment.rayleigh_wind_mode_mps is not None:
        wind_speed = float(rng.rayleigh(definition.environment.rayleigh_wind_mode_mps))
    ####
    position = np.asarray(initial.position_ned_m, dtype=np.float64)
    return _MissileRuntime(
        position_ned_m=position,
        position_derivative_ned_mps=np.zeros(3, dtype=np.float64),
        velocity_body_mps=body_velocity,
        velocity_body_derivative_mps2=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=local_velocity,
        quaternion_wxyz=quaternion,
        quaternion_derivative=np.zeros(4, dtype=np.float64),
        body_rates_rad_s=np.asarray(initial.body_rates_deg_s, dtype=np.float64) * _RAD_PER_DEG,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        fin_state=Agm6FinActuatorState(),
        requested_control=zero_control,
        actuator_step=zero_actuator,
        propulsion=propulsion,
        coefficients=zero_coefficients,
        wrench=Agm6BodyWrench(force_n=(0.0, 0.0, 0.0), moment_nm=(0.0, 0.0, 0.0)),
        control_mode=definition.control.initial_mode,
        guidance_mode=definition.guidance.initial_mode,
        datalink_update_mode=0,
        propulsion_mode=definition.propulsion.initial_mode,
        sensor_mode=definition.sensor.initial_mode,
        ins_mode_requested=definition.ins_mode_requested,
        navigation_gain=definition.guidance.navigation_gain,
        gravity_bias_g=definition.guidance.gravity_bias_g,
        launch_epoch_s=initial.launch_delay_s,
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        altitude_m=-float(position[2]),
        airspeed_mps=initial.speed_mps,
        resolved_constant_wind_speed_mps=wind_speed,
    )


####


def _initialize_point_mass_target(config: Agm6GroundTargetConfig) -> _PointMassRuntime:
    return _initialize_point_mass(
        config.position_ned_m,
        config.speed_mps,
        config.heading_deg,
        config.flight_path_deg,
        longitudinal_acceleration_g=config.longitudinal_acceleration_g,
        lateral_acceleration_g=config.lateral_acceleration_g,
    )


####


def _initialize_aircraft(config: Agm6AircraftConfig) -> _PointMassRuntime:
    return _initialize_point_mass(
        config.position_ned_m,
        config.speed_mps,
        config.heading_deg,
        config.flight_path_deg,
        longitudinal_acceleration_g=config.longitudinal_acceleration_g,
        actor_option=config.aircraft_option,
        turn_g=config.turn_g,
        guidance_gain=config.guidance_gain,
    )


####


def _initialize_point_mass(
    position_ned_m: tuple[float, float, float],
    speed_mps: float,
    heading_deg: float,
    flight_path_deg: float,
    *,
    longitudinal_acceleration_g: float,
    lateral_acceleration_g: float = 0.0,
    actor_option: int = 0,
    turn_g: float = 0.0,
    guidance_gain: float = 0.0,
) -> _PointMassRuntime:
    heading = heading_deg * _RAD_PER_DEG
    flight_path = flight_path_deg * _RAD_PER_DEG
    velocity_to_local = mat2tr(heading, flight_path)
    velocity_ned = velocity_to_local.T @ np.asarray((speed_mps, 0.0, 0.0), dtype=np.float64)
    return _PointMassRuntime(
        position_ned_m=np.asarray(position_ned_m, dtype=np.float64),
        position_derivative_ned_mps=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=velocity_ned,
        acceleration_ned_mps2=np.zeros(3, dtype=np.float64),
        speed_mps=speed_mps,
        heading_rad=heading,
        flight_path_rad=flight_path,
        vehicle_to_local=velocity_to_local.copy(),
        velocity_to_local=velocity_to_local,
        longitudinal_acceleration_g=longitudinal_acceleration_g,
        lateral_acceleration_g=lateral_acceleration_g,
        actor_option=actor_option,
        turn_g=turn_g,
        guidance_gain=guidance_gain,
    )


####


def _missile_environment(
    runtime: _MissileRuntime,
    definition: Agm6SourceDefinition,
    dt_s: float,
    rng: np.random.Generator,
) -> None:
    config = definition.environment
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    if config.atmosphere_mode == 0:
        density, pressure, temperature = atmosphere76(runtime.altitude_m)
    else:
        density = definition.weather_deck.table("density").interpolate((runtime.altitude_m,))
        pressure = definition.weather_deck.table("pressure").interpolate((runtime.altitude_m,))
        temperature = definition.weather_deck.table("temperature").interpolate((runtime.altitude_m,)) + 273.16
    ####
    runtime.density_kg_m3 = max(0.0, density)
    runtime.pressure_pa = max(0.0, pressure)
    runtime.temperature_k = max(1.0, temperature)
    runtime.speed_of_sound_mps = math.sqrt(1.4 * _R_AIR * runtime.temperature_k)

    raw_wind = np.zeros(3, dtype=np.float64)
    if config.wind_mode == 1:
        wind_speed = runtime.resolved_constant_wind_speed_mps
        direction = config.wind_direction_deg * _RAD_PER_DEG
        raw_wind[:] = (
            -wind_speed * math.cos(direction),
            -wind_speed * math.sin(direction),
            config.vertical_wind_mps,
        )
    elif config.wind_mode == 2:
        wind_speed = definition.weather_deck.table("speed").interpolate((runtime.altitude_m,))
        direction = definition.weather_deck.table("direction").interpolate((runtime.altitude_m,)) * _RAD_PER_DEG
        raw_wind[:] = (
            -wind_speed * math.cos(direction),
            -wind_speed * math.sin(direction),
            config.vertical_wind_mps,
        )
    ####
    derivative_new = (raw_wind - runtime.wind_state_ned_mps) / config.wind_time_constant_s
    runtime.wind_state_ned_mps = _integrate_vector(
        runtime.wind_state_ned_mps,
        derivative_new,
        runtime.wind_derivative_ned_mps2,
        dt_s,
    )
    runtime.wind_derivative_ned_mps2 = derivative_new

    runtime.turbulence_ned_mps.fill(0.0)
    preliminary_air_velocity = runtime.velocity_ned_mps - runtime.wind_state_ned_mps
    preliminary_airspeed = max(_SMALL, float(np.linalg.norm(preliminary_air_velocity)))
    if config.turbulence_mode == 1 and config.turbulence_sigma_mps > 0.0:
        white_noise = float(rng.normal(0.0, 1.0 / math.sqrt(dt_s)))
        velocity_over_length = max(_SMALL, preliminary_airspeed / config.turbulence_length_m)
        first_derivative_new = runtime.turbulence_state_2
        runtime.turbulence_state = _integrate_scalar(
            runtime.turbulence_state,
            first_derivative_new,
            runtime.turbulence_derivative,
            dt_s,
        )
        runtime.turbulence_derivative = first_derivative_new
        second_derivative_new = (
            -(velocity_over_length**2) * runtime.turbulence_state
            - 2.0 * velocity_over_length * runtime.turbulence_state_2
            + (velocity_over_length**2) * white_noise
        )
        runtime.turbulence_state_2 = _integrate_scalar(
            runtime.turbulence_state_2,
            second_derivative_new,
            runtime.turbulence_derivative_2,
            dt_s,
        )
        runtime.turbulence_derivative_2 = second_derivative_new
        tau = (
            config.turbulence_sigma_mps
            * math.sqrt(1.0 / (velocity_over_length * math.pi))
            * (runtime.turbulence_state + math.sqrt(3.0) * runtime.turbulence_state_2 / velocity_over_length)
        )
        alpha = runtime.total_alpha_deg * _RAD_PER_DEG
        phi = runtime.aerodynamic_roll_rad
        turbulence_body = np.asarray(
            (
                -tau * math.sin(alpha),
                tau * math.sin(phi) * math.cos(alpha),
                tau * math.cos(phi) * math.cos(alpha),
            ),
            dtype=np.float64,
        )
        runtime.turbulence_ned_mps = _dcm_body_from_local(runtime.quaternion_wxyz).T @ turbulence_body
    ####
    runtime.wind_ned_mps = runtime.wind_state_ned_mps + runtime.turbulence_ned_mps
    relative_air_velocity = runtime.velocity_ned_mps - runtime.wind_ned_mps
    runtime.airspeed_mps = float(np.linalg.norm(relative_air_velocity))
    runtime.mach = abs(runtime.airspeed_mps / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * runtime.density_kg_m3 * runtime.airspeed_mps**2
    if runtime.guidance_mode and (
        runtime.mach <= definition.aerodynamics.minimum_mach or runtime.dynamic_pressure_pa <= definition.aerodynamics.minimum_dynamic_pressure_pa
    ):
        runtime.source_stop = True
    ####


####


def _missile_kinematics(
    runtime: _MissileRuntime,
    definition: Agm6SourceDefinition,
    dt_s: float,
) -> None:
    q0, q1, q2, q3 = (float(value) for value in runtime.quaternion_wxyz)
    roll_rate, pitch_rate, yaw_rate = (float(value) for value in runtime.body_rates_rad_s)
    metric = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3
    error = 1.0 - metric
    correction = 50.0
    derivative_new = np.asarray(
        (
            0.5 * (-roll_rate * q1 - pitch_rate * q2 - yaw_rate * q3) + correction * error * q0,
            0.5 * (roll_rate * q0 + yaw_rate * q2 - pitch_rate * q3) + correction * error * q1,
            0.5 * (pitch_rate * q0 - yaw_rate * q1 + roll_rate * q3) + correction * error * q2,
            0.5 * (yaw_rate * q0 + pitch_rate * q1 - roll_rate * q2) + correction * error * q3,
        ),
        dtype=np.float64,
    )
    runtime.quaternion_wxyz = _integrate_vector(
        runtime.quaternion_wxyz,
        derivative_new,
        runtime.quaternion_derivative,
        dt_s,
    )
    runtime.quaternion_derivative = derivative_new
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    runtime.velocity_body_mps = transform @ runtime.velocity_ned_mps
    air_velocity_body = transform @ (runtime.velocity_ned_mps - runtime.wind_ned_mps)
    airspeed = max(_SMALL, float(np.linalg.norm(air_velocity_body)))
    first, second, third = (float(value) for value in air_velocity_body)
    runtime.alpha_deg = math.atan2(third, first) * _DEG_PER_RAD
    runtime.beta_deg = math.asin(max(-1.0, min(1.0, second / airspeed))) * _DEG_PER_RAD
    total_alpha = math.acos(max(-1.0, min(1.0, first / airspeed)))
    if abs(second) < _SMALL and abs(third) < _SMALL:
        aerodynamic_roll = 0.0
    elif abs(second) < _SMALL:
        aerodynamic_roll = 0.0 if third > 0.0 else math.pi
    else:
        aerodynamic_roll = math.atan2(second, third)
    ####
    runtime.total_alpha_deg = total_alpha * _DEG_PER_RAD
    runtime.aerodynamic_roll_rad = aerodynamic_roll
    if abs(error) > definition.aerodynamics.maximum_quaternion_error:
        runtime.source_stop = True
    ####
    if total_alpha > definition.aerodynamics.maximum_total_incidence_rad:
        runtime.source_stop = True
    ####


####


def _missile_ins(runtime: _MissileRuntime) -> None:
    """Expose truth-aligned computed navigation while retaining requested INS mode telemetry."""

    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    runtime.velocity_body_mps = transform @ runtime.velocity_ned_mps


####


def _point_environment(runtime: _PointMassRuntime) -> None:
    altitude = -float(runtime.position_ned_m[2])
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(altitude)
    density, _, _ = atmosphere76(altitude)
    runtime.density_kg_m3 = density
    runtime.speed_mps = float(np.linalg.norm(runtime.velocity_ned_mps))
    runtime.dynamic_pressure_pa = 0.5 * density * runtime.speed_mps**2


####


def _ground_target_forces(runtime: _PointMassRuntime) -> None:
    runtime.specific_force_vehicle_mps2 = np.asarray(
        (
            runtime.longitudinal_acceleration_g * runtime.gravity_mps2,
            runtime.lateral_acceleration_g * runtime.gravity_mps2,
            -runtime.gravity_mps2,
        ),
        dtype=np.float64,
    )


####


def _aircraft_guidance(runtime: _PointMassRuntime, target: _ActorPacket) -> None:
    if runtime.actor_option == 0:
        runtime.commanded_acceleration_local_mps2 = np.asarray(
            (0.0, 0.0, -runtime.gravity_mps2),
            dtype=np.float64,
        )
    elif runtime.actor_option == 1:
        command_velocity = np.asarray(
            (0.0, runtime.turn_g * runtime.gravity_mps2, -runtime.gravity_mps2),
            dtype=np.float64,
        )
        runtime.commanded_acceleration_local_mps2 = runtime.velocity_to_local.T @ command_velocity
    else:
        displacement = runtime.position_ned_m - target.position_ned_m
        distance = float(np.linalg.norm(displacement))
        target_speed = float(np.linalg.norm(target.velocity_ned_mps))
        own_speed = max(_SMALL, runtime.speed_mps)
        if distance <= _SMALL or target_speed <= _SMALL:
            command = np.asarray((0.0, 0.0, -runtime.gravity_mps2), dtype=np.float64)
        else:
            own_unit = runtime.velocity_ned_mps / own_speed
            target_unit = target.velocity_ned_mps / target_speed
            gain = runtime.guidance_gain * float(np.linalg.norm(np.cross(runtime.velocity_ned_mps, target.velocity_ned_mps))) / distance
            error = np.cross(own_unit, target_unit)
            command = np.cross(error, own_unit) * gain + np.asarray(
                (0.0, 0.0, -runtime.gravity_mps2),
                dtype=np.float64,
            )
        ####
        runtime.commanded_acceleration_local_mps2 = command
    ####


####


def _aircraft_control(
    runtime: _PointMassRuntime,
    config: Agm6AircraftConfig,
    dt_s: float,
) -> None:
    command_velocity = runtime.velocity_to_local @ runtime.commanded_acceleration_local_mps2
    lateral = float(command_velocity[1])
    vertical = float(command_velocity[2])
    bank_command = 0.0 if abs(lateral) < _SMALL and abs(vertical) < _SMALL else math.atan2(lateral, -vertical)
    if config.bank_time_constant_s > 0.0:
        derivative_new = (bank_command - runtime.bank_rad) / config.bank_time_constant_s
        runtime.bank_rad = _integrate_scalar(
            runtime.bank_rad,
            derivative_new,
            runtime.bank_derivative_rad_s,
            dt_s,
        )
        runtime.bank_derivative_rad_s = derivative_new
    else:
        runtime.bank_rad = bank_command
    ####
    runtime.bank_rad = _clip(runtime.bank_rad * _DEG_PER_RAD, config.bank_limit_deg) * _RAD_PER_DEG

    normal_command = math.hypot(lateral, vertical) / max(runtime.gravity_mps2, _SMALL)
    if config.normal_load_time_constant_s > 0.0:
        derivative_new = (normal_command - runtime.normal_load_g) / config.normal_load_time_constant_s
        runtime.normal_load_g = _integrate_scalar(
            runtime.normal_load_g,
            derivative_new,
            runtime.normal_load_derivative_g_s,
            dt_s,
        )
        runtime.normal_load_derivative_g_s = derivative_new
    else:
        runtime.normal_load_g = normal_command
    ####
    if runtime.actor_option > 0:
        limit = runtime.dynamic_pressure_pa * config.lift_slope_per_deg * config.alpha_limit_deg / max(config.wing_loading_n_m2, _SMALL)
        runtime.normal_load_g = _clip(runtime.normal_load_g, max(0.0, limit))
    ####


####


def _aircraft_forces(runtime: _PointMassRuntime) -> None:
    runtime.specific_force_vehicle_mps2 = np.asarray(
        (
            runtime.longitudinal_acceleration_g * runtime.gravity_mps2,
            0.0,
            -runtime.normal_load_g * runtime.gravity_mps2,
        ),
        dtype=np.float64,
    )


####


def _point_newton(runtime: _PointMassRuntime, dt_s: float) -> None:
    gravity_local = np.asarray((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    acceleration_new = runtime.vehicle_to_local.T @ runtime.specific_force_vehicle_mps2 + gravity_local
    velocity_new = _integrate_vector(
        runtime.velocity_ned_mps,
        acceleration_new,
        runtime.acceleration_ned_mps2,
        dt_s,
    )
    runtime.position_ned_m = _integrate_vector(
        runtime.position_ned_m,
        velocity_new,
        runtime.position_derivative_ned_mps,
        dt_s,
    )
    runtime.position_derivative_ned_mps = velocity_new
    runtime.acceleration_ned_mps2 = acceleration_new
    runtime.velocity_ned_mps = velocity_new
    runtime.speed_mps = float(np.linalg.norm(velocity_new))
    if runtime.speed_mps > _SMALL:
        north, east, down = (float(value) for value in velocity_new)
        runtime.heading_rad = math.atan2(east, north) if north != 0.0 or east != 0.0 else 0.0
        runtime.flight_path_rad = math.atan2(-down, math.hypot(north, east))
    ####
    runtime.velocity_to_local = mat2tr(runtime.heading_rad, runtime.flight_path_rad)
    cosine = math.cos(runtime.bank_rad)
    sine = math.sin(runtime.bank_rad)
    aircraft_from_velocity = np.asarray(
        (
            (1.0, 0.0, 0.0),
            (0.0, cosine, sine),
            (0.0, -sine, cosine),
        ),
        dtype=np.float64,
    )
    runtime.vehicle_to_local = aircraft_from_velocity @ runtime.velocity_to_local


####


def _point_packet(runtime: _PointMassRuntime, *, alive: bool) -> _ActorPacket:
    return _ActorPacket(
        position_ned_m=runtime.position_ned_m.copy(),
        velocity_ned_mps=runtime.velocity_ned_mps.copy(),
        heading_deg=runtime.heading_rad * _DEG_PER_RAD,
        flight_path_deg=runtime.flight_path_rad * _DEG_PER_RAD,
        alive=alive,
    )


####


def _aircraft_packet(
    runtime: _PointMassRuntime,
    tracking: _TrackingRuntime,
    *,
    alive: bool,
) -> _AircraftPacket:
    return _AircraftPacket(
        position_ned_m=runtime.position_ned_m.copy(),
        velocity_ned_mps=runtime.velocity_ned_mps.copy(),
        heading_deg=runtime.heading_rad * _DEG_PER_RAD,
        flight_path_deg=runtime.flight_path_rad * _DEG_PER_RAD,
        alive=alive,
        track=_TrackPacket(
            position_ned_m=tracking.track.position_ned_m.copy(),
            velocity_ned_mps=tracking.track.velocity_ned_mps.copy(),
            update_sequence=tracking.track.update_sequence,
            time_s=tracking.track.time_s,
        ),
    )


####


def _aircraft_sensor_track(
    aircraft: _PointMassRuntime,
    target: _ActorPacket,
    tracking: _TrackingRuntime,
    config: Agm6TrackingConfig,
    rng: np.random.Generator,
    sim_time: float,
) -> None:
    if sim_time + _SMALL < tracking.next_update_time_s:
        return
    ####
    carrier_minus_target = aircraft.position_ned_m - target.position_ned_m
    distance, azimuth, elevation = _polar_from_cartesian(carrier_minus_target)
    measured_distance = max(0.0, distance + float(rng.normal(0.0, config.range_sigma_m)))
    measured_azimuth = azimuth + float(rng.normal(0.0, config.azimuth_sigma_rad))
    measured_elevation = elevation + float(rng.normal(0.0, config.elevation_sigma_rad))
    measured_carrier_minus_target = _cart_from_polar(
        measured_distance,
        measured_azimuth,
        measured_elevation,
    )
    measured_position = aircraft.position_ned_m - measured_carrier_minus_target
    measured_velocity = target.velocity_ned_mps + rng.normal(
        0.0,
        config.velocity_sigma_mps,
        size=3,
    )
    tracking.update_sequence += 1
    tracking.track = _TrackPacket(
        position_ned_m=np.asarray(measured_position, dtype=np.float64),
        velocity_ned_mps=np.asarray(measured_velocity, dtype=np.float64),
        update_sequence=tracking.update_sequence,
        time_s=sim_time,
    )
    tracking.next_update_time_s = sim_time + config.track_step_s


####


def _missile_datalink(
    runtime: _MissileRuntime,
    aircraft: _AircraftPacket,
) -> None:
    runtime.datalink_update_mode = 0
    track = aircraft.track
    runtime.datalink_track_sequence = max(
        runtime.datalink_track_sequence,
        track.update_sequence,
    )
    target_position_norm_m = float(np.linalg.norm(track.position_ned_m))
    if abs(runtime.datalink_target_position_norm_m - target_position_norm_m) > _SMALL:
        runtime.datalink_update_mode = 3
        runtime.datalink_target_position_norm_m = target_position_norm_m
        runtime.target_position_stored_ned_m = track.position_ned_m.copy()
        runtime.target_velocity_stored_ned_mps = track.velocity_ned_mps.copy()
    ####


####


def _missile_sensor(
    runtime: _MissileRuntime,
    config: Agm6SensorConfig,
    target: _ActorPacket,
    time_s: float,
    dt_s: float,
) -> None:
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    sensor = cadac_local_ned_relative_state_track(
        time_s=time_s,
        host_position_ned_m=runtime.position_ned_m,
        host_velocity_ned_mps=runtime.velocity_ned_mps,
        target_id="agm6-target",
        target_position_ned_m=target.position_ned_m,
        target_velocity_ned_mps=target.velocity_ned_mps,
        body_from_local=transform,
    )
    if isinstance(sensor, str):
        runtime.target_range_m = 0.0
        return
    ####
    distance = sensor.range_m
    runtime.target_range_m = distance
    unit_body = np.asarray(sensor.unit_los_sensor, dtype=np.float64)
    unit_local = transform.T @ unit_body
    runtime.closing_speed_mps = sensor.closing_speed_mps
    los_rate_body = np.asarray(sensor.line_of_sight_rate_sensor_rad_s, dtype=np.float64)
    _, yaw_true, pitch_true = _polar_from_cartesian(unit_body)
    runtime.unit_los_local = unit_local
    runtime.unit_los_body = unit_body
    runtime.true_los_rate_body_rad_s = los_rate_body

    if runtime.sensor_mode == 2 and distance < config.acquisition_range_m:
        runtime.sensor_mode = 3
        runtime.sensor_initialized = False
    ####
    if runtime.sensor_mode == 3:
        if not runtime.sensor_initialized:
            runtime.sensor_pointing_pitch_rad = pitch_true
            runtime.sensor_pointing_yaw_rad = yaw_true
            runtime.sensor_pitch = _SensorAxisState(rate_rad_s=float(los_rate_body[1]))
            runtime.sensor_yaw = _SensorAxisState(rate_rad_s=float(los_rate_body[2]))
            runtime.sensor_acquisition_epoch_s = time_s
            runtime.sensor_initialized = True
        ####
        if config.dynamic_mode == 1:
            pitch_error, yaw_error = _update_dynamic_sensor(
                runtime,
                config,
                pitch_true,
                yaw_true,
                dt_s,
            )
        else:
            runtime.sensor_pointing_pitch_rad = pitch_true
            runtime.sensor_pointing_yaw_rad = yaw_true
            runtime.sensor_pitch.rate_rad_s = float(los_rate_body[1])
            runtime.sensor_yaw.rate_rad_s = float(los_rate_body[2])
            pitch_error = 0.0
            yaw_error = 0.0
        ####
        if time_s - runtime.sensor_acquisition_epoch_s > config.acquisition_time_s:
            if abs(yaw_error) <= config.yaw_half_fov_rad and abs(pitch_error) <= config.pitch_half_fov_rad:
                runtime.sensor_mode = 4
            else:
                runtime.source_stop = True
            ####
        ####
    elif runtime.sensor_mode == 4:
        if config.dynamic_mode == 1:
            pitch_error, yaw_error = _update_dynamic_sensor(
                runtime,
                config,
                pitch_true,
                yaw_true,
                dt_s,
            )
        else:
            runtime.sensor_pointing_pitch_rad = pitch_true
            runtime.sensor_pointing_yaw_rad = yaw_true
            runtime.sensor_pitch.rate_rad_s = float(los_rate_body[1])
            runtime.sensor_yaw.rate_rad_s = float(los_rate_body[2])
            pitch_error = 0.0
            yaw_error = 0.0
        ####
        if abs(runtime.sensor_pointing_pitch_rad) > config.maximum_pitch_gimbal_rad:
            runtime.source_stop = True
        ####
        if abs(runtime.sensor_pitch.rate_rad_s) > config.maximum_pitch_gimbal_rate_rad_s:
            runtime.source_stop = True
        ####
        if math.hypot(pitch_error, yaw_error) > config.maximum_tracking_error_rad:
            runtime.source_stop = True
        ####
        if distance < config.blind_range_m:
            runtime.sensor_mode = 5
        ####
    ####


####


def _update_dynamic_sensor(
    runtime: _MissileRuntime,
    config: Agm6SensorConfig,
    pitch_true_rad: float,
    yaw_true_rad: float,
    dt_s: float,
) -> tuple[float, float]:
    pitch_error = _wrap_pi(pitch_true_rad - runtime.sensor_pointing_pitch_rad)
    yaw_error = _wrap_pi(yaw_true_rad - runtime.sensor_pointing_yaw_rad)
    runtime.sensor_pitch = _update_sensor_axis(
        runtime.sensor_pitch,
        pitch_error,
        config,
        dt_s,
    )
    runtime.sensor_yaw = _update_sensor_axis(
        runtime.sensor_yaw,
        yaw_error,
        config,
        dt_s,
    )
    pitch_rate_new = runtime.sensor_pitch.rate_rad_s
    yaw_rate_new = runtime.sensor_yaw.rate_rad_s
    runtime.sensor_pointing_pitch_rad = _integrate_scalar(
        runtime.sensor_pointing_pitch_rad,
        pitch_rate_new,
        runtime.sensor_pointing_pitch_derivative_rad_s,
        dt_s,
    )
    runtime.sensor_pointing_yaw_rad = _integrate_scalar(
        runtime.sensor_pointing_yaw_rad,
        yaw_rate_new,
        runtime.sensor_pointing_yaw_derivative_rad_s,
        dt_s,
    )
    runtime.sensor_pointing_pitch_derivative_rad_s = pitch_rate_new
    runtime.sensor_pointing_yaw_derivative_rad_s = yaw_rate_new
    return (pitch_error, yaw_error)


####


def _update_sensor_axis(
    state: _SensorAxisState,
    error_rad: float,
    config: Agm6SensorConfig,
    dt_s: float,
) -> _SensorAxisState:
    acceleration_derivative_new = (
        config.filter_natural_frequency_rad_s**2 * (config.filter_gain_per_s * error_rad - state.rate_rad_s)
        - 2.0 * config.filter_damping_ratio * config.filter_natural_frequency_rad_s * state.acceleration_rad_s2
    )
    acceleration = _integrate_scalar(
        state.acceleration_rad_s2,
        acceleration_derivative_new,
        state.acceleration_derivative_rad_s3,
        dt_s,
    )
    rate_derivative_new = acceleration
    rate = _integrate_scalar(
        state.rate_rad_s,
        rate_derivative_new,
        state.rate_derivative_rad_s2,
        dt_s,
    )
    return _SensorAxisState(
        rate_rad_s=rate,
        rate_derivative_rad_s2=rate_derivative_new,
        acceleration_rad_s2=acceleration,
        acceleration_derivative_rad_s3=acceleration_derivative_new,
    )


####


def _missile_guidance(
    runtime: _MissileRuntime,
    config: Agm6GuidanceConfig,
    target: _ActorPacket,
    aircraft: _AircraftPacket,
) -> None:
    midcourse = runtime.guidance_mode // 10
    terminal = runtime.guidance_mode % 10
    runtime.normal_command_g = 0.0
    runtime.lateral_command_g = 0.0
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    if terminal == 6:
        _terminal_guidance(runtime, transform)
    elif midcourse == 2:
        _line_guidance(runtime, config, aircraft)
    elif midcourse == 3:
        if runtime.datalink_update_mode == 3:
            runtime.target_update_epoch_s = runtime.launch_time_s
        ####
        extrapolated = runtime.target_position_stored_ned_m + runtime.target_velocity_stored_ned_mps * (runtime.launch_time_s - runtime.target_update_epoch_s)
        _proportional_navigation(
            runtime,
            transform,
            target_position_ned_m=extrapolated,
            target_velocity_ned_mps=runtime.target_velocity_stored_ned_mps,
            gravity_bias_g=runtime.gravity_bias_g,
        )
    elif midcourse == 4:
        _proportional_navigation(
            runtime,
            transform,
            target_position_ned_m=target.position_ned_m,
            target_velocity_ned_mps=target.velocity_ned_mps,
            gravity_bias_g=runtime.gravity_bias_g,
        )
    ####
    maximum = runtime.coefficients.max_acceleration_g
    magnitude = math.hypot(runtime.lateral_command_g, runtime.normal_command_g)
    if maximum > 0.0 and magnitude > maximum:
        scale = maximum / magnitude
        runtime.lateral_command_g *= scale
        runtime.normal_command_g *= scale
    ####


####


def _proportional_navigation(
    runtime: _MissileRuntime,
    transform_body_from_local: FloatMatrix,
    *,
    target_position_ned_m: FloatVector,
    target_velocity_ned_mps: FloatVector,
    gravity_bias_g: float,
) -> None:
    relative = target_position_ned_m - runtime.position_ned_m
    distance = float(np.linalg.norm(relative))
    if distance <= _SMALL:
        return
    ####
    unit_local = relative / distance
    relative_velocity = target_velocity_ned_mps - runtime.velocity_ned_mps
    closing = abs(float(unit_local @ relative_velocity))
    los_rate_local = np.cross(unit_local, relative_velocity) / distance
    gravity_compensation = np.asarray(
        (0.0, 0.0, gravity_bias_g * runtime.gravity_mps2),
        dtype=np.float64,
    )
    acceleration_local = np.cross(los_rate_local, unit_local) * runtime.navigation_gain * closing - gravity_compensation
    acceleration_body_g = transform_body_from_local @ acceleration_local / _AGRAV
    runtime.normal_command_g = -float(acceleration_body_g[2])
    runtime.lateral_command_g = float(acceleration_body_g[1])


####


def _line_guidance(
    runtime: _MissileRuntime,
    config: Agm6GuidanceConfig,
    aircraft: _AircraftPacket,
) -> None:
    target = runtime.target_position_stored_ned_m
    missile_to_target = target - runtime.position_ned_m
    range_to_target, los_heading, los_pitch = _polar_from_cartesian(missile_to_target)
    if range_to_target <= _SMALL:
        return
    ####
    aircraft_to_target = target - aircraft.position_ned_m
    _, loa_heading, loa_pitch_online = _polar_from_cartesian(aircraft_to_target)
    loa_pitch = loa_pitch_online if config.vertical_line_of_attack_deg == 0.0 else config.vertical_line_of_attack_deg * _RAD_PER_DEG
    loa_transform = mat2tr(loa_heading, loa_pitch)
    los_transform = mat2tr(los_heading, los_pitch)
    north, east, down = (float(value) for value in runtime.velocity_ned_mps)
    heading = math.atan2(east, north) if north != 0.0 or east != 0.0 else 0.0
    flight_path = math.atan2(-down, math.hypot(north, east))
    velocity_transform = mat2tr(heading, flight_path)
    body_from_velocity = _dcm_body_from_local(runtime.quaternion_wxyz) @ velocity_transform.T
    velocity_los = los_transform @ runtime.velocity_ned_mps
    velocity_loa = loa_transform @ runtime.velocity_ned_mps
    nonlinear_gain = config.nonlinear_gain_factor * (1.0 - math.exp(-range_to_target / max(config.distance_decrement_m, _SMALL)))
    acceleration_velocity_g = np.asarray(
        (
            runtime.gravity_mps2 * math.sin(flight_path) / _AGRAV,
            config.line_gain_per_s * (-float(velocity_los[1]) + nonlinear_gain * float(velocity_loa[1])) / _AGRAV,
            config.line_gain_per_s
            * (-float(velocity_los[2]) + nonlinear_gain * float(velocity_loa[2]) - runtime.gravity_mps2 * math.cos(flight_path))
            / _AGRAV,
        ),
        dtype=np.float64,
    )
    acceleration_body_g = body_from_velocity @ acceleration_velocity_g
    runtime.lateral_command_g = float(acceleration_body_g[1])
    runtime.normal_command_g = -float(acceleration_body_g[2])


####


def _terminal_guidance(runtime: _MissileRuntime, transform_body_from_local: FloatMatrix) -> None:
    closing = max(0.0, runtime.closing_speed_mps)
    yaw_angle = runtime.sensor_pointing_yaw_rad
    pitch_angle = runtime.sensor_pointing_pitch_rad
    pitch_los_rate = runtime.sensor_pitch.rate_rad_s
    yaw_los_rate = runtime.sensor_yaw.rate_rad_s
    longitudinal_specific_force = runtime.wrench.force_n[0] / runtime.propulsion.mass_kg
    lateral_compensation = longitudinal_specific_force * math.tan(yaw_angle) / _AGRAV
    pitch_compensation = longitudinal_specific_force * math.tan(pitch_angle) / max(_SMALL, math.cos(yaw_angle) * _AGRAV)
    gravity_body = transform_body_from_local @ np.asarray((0.0, 0.0, 1.0), dtype=np.float64)
    gain = runtime.navigation_gain * closing
    lateral_pn = gain * yaw_los_rate / max(_SMALL, math.cos(yaw_angle) * _AGRAV)
    normal_pn = gain * (yaw_los_rate * math.tan(pitch_angle) * math.tan(yaw_angle) + pitch_los_rate / max(_SMALL, math.cos(pitch_angle))) / _AGRAV
    runtime.lateral_command_g = lateral_pn + lateral_compensation - float(gravity_body[1])
    runtime.normal_command_g = normal_pn + pitch_compensation + float(gravity_body[2])


####


def _missile_control(
    runtime: _MissileRuntime,
    config: Agm6ControlConfig,
    dt_s: float,
) -> Agm6ControlCommand:
    if runtime.control_mode == 0:
        return Agm6ControlCommand()
    ####
    roll = _control_roll(runtime, config)
    pitch = 0.0
    yaw = 0.0
    if runtime.control_mode == 2:
        pitch, yaw = _control_rate(runtime, config)
    elif runtime.control_mode == 3:
        pitch, yaw = _control_acceleration(runtime, config, dt_s)
    ####
    return Agm6ControlCommand(
        roll_deg=_clip(roll, config.roll_command_limit_deg),
        pitch_deg=_clip(pitch, config.pitch_command_limit_deg),
        yaw_deg=_clip(yaw, config.yaw_command_limit_deg),
    )


####


def _control_roll(runtime: _MissileRuntime, config: Agm6ControlConfig) -> float:
    dlp = runtime.coefficients.roll_rate_derivative_per_s
    dld = _nonzero(runtime.coefficients.roll_control_derivative_rad_s2)
    gkp = (2.0 * config.roll_damping_ratio * config.roll_natural_frequency_rad_s + dlp) / dld
    gkphi = config.roll_natural_frequency_rad_s**2 / dld
    roll_angle_deg = _euler_from_dcm(_dcm_body_from_local(runtime.quaternion_wxyz))[2] * _DEG_PER_RAD
    error = gkphi * (config.commanded_roll_deg - roll_angle_deg) * _RAD_PER_DEG
    return (error - gkp * float(runtime.body_rates_rad_s[0])) * _DEG_PER_RAD


####


def _control_rate(runtime: _MissileRuntime, config: Agm6ControlConfig) -> tuple[float, float]:
    speed = max(_SMALL, runtime.airspeed_mps)
    dna = runtime.coefficients.normal_alpha_derivative_mps2
    dnd = runtime.coefficients.normal_control_derivative_mps2
    dma = runtime.coefficients.pitch_alpha_derivative_rad_s2
    dmq = runtime.coefficients.pitch_rate_derivative_per_s
    dmd = _nonzero(runtime.coefficients.pitch_control_derivative_rad_s2)
    zrate = dna / speed - dma * dnd / (speed * dmd)
    aa = dna / speed - dmq
    bb = -dma - dmq * dna / speed
    damping = config.rate_loop_damping_ratio
    dum1 = aa - 2.0 * damping * damping * zrate
    dum2 = aa * aa - 4.0 * damping * damping * bb
    radix = max(0.0, dum1 * dum1 - dum2)
    gain = (-dum1 + math.sqrt(radix)) / (-dmd)
    pitch = gain * float(runtime.body_rates_rad_s[1]) * _DEG_PER_RAD - config.pitch_rate_command_deg_s
    yaw = gain * float(runtime.body_rates_rad_s[2]) * _DEG_PER_RAD - config.yaw_rate_command_deg_s
    return (pitch, yaw)


####


def _control_acceleration(
    runtime: _MissileRuntime,
    config: Agm6ControlConfig,
    dt_s: float,
) -> tuple[float, float]:
    lateral = runtime.lateral_command_g
    normal = runtime.normal_command_g
    magnitude = math.hypot(lateral, normal)
    if magnitude > config.structural_limit_g:
        scale = config.structural_limit_g / magnitude
        lateral *= scale
        normal *= scale
    ####
    wacl = config.acceleration_natural_frequency_rad_s
    zacl = config.acceleration_damping_ratio
    pacl = config.acceleration_real_pole_rad_s
    dna = _nonzero(runtime.coefficients.normal_alpha_derivative_mps2)
    dma = runtime.coefficients.pitch_alpha_derivative_rad_s2
    dmq = runtime.coefficients.pitch_rate_derivative_per_s
    dmd = _nonzero(runtime.coefficients.pitch_control_derivative_rad_s2)
    speed = max(_SMALL, runtime.airspeed_mps)
    gainfb3 = wacl * wacl * pacl / (dna * dmd)
    gainfb2 = (2.0 * zacl * wacl + pacl + dmq - dna / speed) / dmd
    gainfb1 = (wacl * wacl + 2.0 * zacl * wacl * pacl + dma + dmq * dna / speed - gainfb2 * dna * dmd / speed) / (
        dna * dmd
    ) - config.acceleration_feedforward_gain_s2_m

    force_body = np.asarray(runtime.wrench.force_n, dtype=np.float64) / runtime.propulsion.mass_kg
    pitch_force = float(force_body[2])
    pitch_derivative_new = _AGRAV * normal + pitch_force
    runtime.pitch_feedforward_state = _integrate_scalar(
        runtime.pitch_feedforward_state,
        pitch_derivative_new,
        runtime.pitch_feedforward_derivative,
        dt_s,
    )
    runtime.pitch_feedforward_derivative = pitch_derivative_new
    pitch = (
        -gainfb1 * (-pitch_force)
        - gainfb2 * float(runtime.body_rates_rad_s[1])
        + gainfb3 * runtime.pitch_feedforward_state
        + config.acceleration_feedforward_gain_s2_m * pitch_derivative_new
    ) * _DEG_PER_RAD

    yaw_force = float(force_body[1])
    yaw_derivative_new = _AGRAV * lateral - yaw_force
    runtime.yaw_feedforward_state = _integrate_scalar(
        runtime.yaw_feedforward_state,
        yaw_derivative_new,
        runtime.yaw_feedforward_derivative,
        dt_s,
    )
    runtime.yaw_feedforward_derivative = yaw_derivative_new
    yaw = (
        -gainfb1 * yaw_force
        - gainfb2 * float(runtime.body_rates_rad_s[2])
        + gainfb3 * runtime.yaw_feedforward_state
        + config.acceleration_feedforward_gain_s2_m * yaw_derivative_new
    ) * _DEG_PER_RAD
    return (pitch, yaw)


####


def _missile_euler(
    runtime: _MissileRuntime,
    definition: Agm6SourceDefinition,
    dt_s: float,
) -> None:
    moment = np.asarray(runtime.wrench.moment_nm, dtype=np.float64)
    roll_inertia = definition.airframe.launch_roll_inertia_kg_m2
    transverse_inertia = definition.airframe.launch_pitch_inertia_kg_m2
    roll_rate, pitch_rate, yaw_rate = (float(value) for value in runtime.body_rates_rad_s)
    derivative_new = np.asarray(
        (
            float(moment[0]) / roll_inertia,
            ((transverse_inertia - roll_inertia) * roll_rate * yaw_rate + float(moment[1])) / transverse_inertia,
            (-(transverse_inertia - roll_inertia) * roll_rate * pitch_rate + float(moment[2])) / transverse_inertia,
        ),
        dtype=np.float64,
    )
    runtime.body_rates_rad_s = _integrate_vector(
        runtime.body_rates_rad_s,
        derivative_new,
        runtime.body_rate_derivative_rad_s2,
        dt_s,
    )
    runtime.body_rate_derivative_rad_s2 = derivative_new


####


def _missile_newton(runtime: _MissileRuntime, dt_s: float) -> None:
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    gravity_local = np.asarray((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    specific_force_body = np.asarray(runtime.wrench.force_n, dtype=np.float64) / runtime.propulsion.mass_kg
    tangent_acceleration = np.cross(runtime.body_rates_rad_s, runtime.velocity_body_mps)
    derivative_new = specific_force_body - tangent_acceleration + transform @ gravity_local
    runtime.velocity_body_mps = _integrate_vector(
        runtime.velocity_body_mps,
        derivative_new,
        runtime.velocity_body_derivative_mps2,
        dt_s,
    )
    runtime.velocity_body_derivative_mps2 = derivative_new
    local_velocity = transform.T @ runtime.velocity_body_mps
    runtime.position_ned_m = _integrate_vector(
        runtime.position_ned_m,
        local_velocity,
        runtime.position_derivative_ned_mps,
        dt_s,
    )
    runtime.position_derivative_ned_mps = local_velocity
    runtime.velocity_ned_mps = local_velocity
    runtime.altitude_m = -float(runtime.position_ned_m[2])
    runtime.normal_acceleration_g = -float(specific_force_body[2]) / max(runtime.gravity_mps2, _SMALL)
    runtime.lateral_acceleration_g = float(specific_force_body[1]) / max(runtime.gravity_mps2, _SMALL)


####


def _update_source_stop(runtime: _MissileRuntime, definition: Agm6SourceDefinition) -> None:
    if runtime.coefficients.max_acceleration_g < definition.aerodynamics.minimum_load_capacity_g:
        runtime.source_stop = True
    ####
    if runtime.total_alpha_deg * _RAD_PER_DEG > definition.aerodynamics.maximum_total_incidence_rad:
        runtime.source_stop = True
    ####


####


def _missile_intercept(
    runtime: _MissileRuntime,
    definition: Agm6SourceDefinition,
    target: _ActorPacket,
    sim_time: float,
    dt_s: float,
) -> Agm6Intercept | None:
    midcourse = runtime.guidance_mode // 10
    terminal = runtime.guidance_mode % 10
    missile_wrt_target = runtime.position_ned_m - target.position_ned_m
    distance = float(np.linalg.norm(missile_wrt_target))
    runtime.target_range_m = max(0.0, distance)
    if midcourse != 4 and terminal != 6:
        return None
    ####
    if distance >= definition.target_sphere_radius_m:
        return None
    ####
    target_plane = mat2tr(
        definition.target_plane_yaw_deg * _RAD_PER_DEG,
        definition.target_plane_pitch_deg * _RAD_PER_DEG,
    )
    current_plane = target_plane @ missile_wrt_target
    previous_plane = runtime.previous_target_plane_position_m
    result: Agm6Intercept | None = None
    if current_plane[2] > 0.0:
        delta = current_plane - previous_plane
        denominator = float(delta[2])
        if abs(denominator) > _SMALL:
            fraction = -float(previous_plane[2]) / denominator
            miss_vector = previous_plane + fraction * delta
            hit_time = runtime.previous_time_s + fraction * dt_s
            result = Agm6Intercept(
                time_s=max(0.0, hit_time),
                miss_distance_m=float(np.linalg.norm(miss_vector)),
                miss_vector_target_plane_m=_tuple3(miss_vector),
                target_range_m=max(0.0, distance),
            )
        ####
    ####
    runtime.previous_target_plane_position_m = current_plane.copy()
    runtime.previous_time_s = sim_time
    return result


####


def _apply_missile_event(
    runtime: _MissileRuntime,
    cursor: CadacEventCursor,
    sim_time: float,
) -> CadacEventApplication | None:
    values: dict[str, CadacRuntimeScalar] = {
        "time": sim_time,
        "event_time": sim_time - runtime.event_epoch_s,
        "mnav": runtime.datalink_update_mode,
        "mprop": runtime.propulsion_mode,
        "mseek": runtime.sensor_mode,
        "mguid": runtime.guidance_mode,
        "maut": runtime.control_mode,
        "grav_bias": runtime.gravity_bias_g,
        "gnav": runtime.navigation_gain,
        "thrust": runtime.propulsion.thrust_n,
    }
    previous_sensor_mode = runtime.sensor_mode
    application = cursor.evaluate_and_apply(values)
    if application is None:
        return None
    ####
    runtime.datalink_update_mode = int(values["mnav"])
    runtime.propulsion_mode = int(values["mprop"])
    runtime.sensor_mode = int(values["mseek"])
    runtime.guidance_mode = int(values["mguid"])
    runtime.control_mode = int(values["maut"])
    runtime.gravity_bias_g = float(values["grav_bias"])
    runtime.navigation_gain = float(values["gnav"])
    if runtime.sensor_mode != previous_sensor_mode:
        runtime.sensor_initialized = False
    ####
    runtime.event_epoch_s = sim_time
    runtime.event_time_s = 0.0
    return application


####


def _apply_point_event(
    runtime: _PointMassRuntime,
    cursor: CadacEventCursor,
    *,
    actor: Literal["TARGET3", "AIRCRAFT3"],
    time_s: float,
    event_epoch_s: float,
) -> tuple[CadacEventApplication | None, float]:
    values: dict[str, CadacRuntimeScalar] = {
        "time": time_s,
        "event_time": time_s - event_epoch_s,
        "acc_longx": runtime.longitudinal_acceleration_g,
        "acc_latx": runtime.lateral_acceleration_g,
        "acft_option": runtime.actor_option,
        "gturn": runtime.turn_g,
        "guid_gain": runtime.guidance_gain,
    }
    application = cursor.evaluate_and_apply(values)
    if application is None:
        return (None, event_epoch_s)
    ####
    runtime.longitudinal_acceleration_g = float(values["acc_longx"])
    runtime.lateral_acceleration_g = float(values["acc_latx"])
    runtime.actor_option = int(values["acft_option"])
    runtime.turn_g = float(values["gturn"])
    runtime.guidance_gain = float(values["guid_gain"])
    del actor
    return (application, time_s)


####


def _event_trace(
    application: CadacEventApplication,
    time_s: float,
    actor: Literal["MISSILE6", "TARGET3", "AIRCRAFT3"],
) -> Agm6EventTrace:
    return Agm6EventTrace(
        time_s=time_s,
        actor=actor,
        event_index=application.event_index,
        source_line=application.source_line,
        watch_variable=application.watch_variable,
        operator=application.operator.value,
        criterion=application.criterion,
        previous_values=application.previous_values,
        updated_values=application.updated_values,
    )


####


def _missile_sample(
    time_s: float,
    runtime: _MissileRuntime,
    definition: Agm6SourceDefinition,
) -> Agm6Sample:
    return Agm6Sample(
        time_s=time_s,
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        speed_mps=float(np.linalg.norm(runtime.velocity_ned_mps)),
        airspeed_mps=runtime.airspeed_mps,
        quaternion_wxyz=_tuple4(runtime.quaternion_wxyz),
        body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
        altitude_m=runtime.altitude_m,
        wind_ned_mps=_tuple3(runtime.wind_ned_mps),
        turbulence_ned_mps=_tuple3(runtime.turbulence_ned_mps),
        mach=max(0.0, runtime.mach),
        dynamic_pressure_pa=max(0.0, runtime.dynamic_pressure_pa),
        alpha_deg=runtime.alpha_deg,
        beta_deg=runtime.beta_deg,
        total_alpha_deg=runtime.total_alpha_deg,
        aerodynamic_roll_deg=runtime.aerodynamic_roll_rad * _DEG_PER_RAD,
        requested_control_deg=runtime.requested_control.vector(),
        requested_fins_deg=runtime.actuator_step.requested_fins.vector(),
        achieved_fins_deg=runtime.actuator_step.achieved_fins.vector(),
        achieved_control_deg=runtime.actuator_step.achieved_control.vector(),
        force_body_n=runtime.wrench.force_n,
        moment_body_nm=runtime.wrench.moment_nm,
        mass_kg=runtime.propulsion.mass_kg,
        fuel_remaining_kg=runtime.propulsion.fuel_remaining_kg,
        roll_inertia_kg_m2=definition.airframe.launch_roll_inertia_kg_m2,
        pitch_inertia_kg_m2=definition.airframe.launch_pitch_inertia_kg_m2,
        thrust_n=runtime.propulsion.thrust_n,
        propulsion_mode=runtime.propulsion_mode,
        sensor_mode=runtime.sensor_mode,
        guidance_mode=runtime.guidance_mode,
        autopilot_mode=runtime.control_mode,
        datalink_update_mode=runtime.datalink_update_mode,
        datalink_track_sequence=runtime.datalink_track_sequence,
        target_range_m=0.0 if not math.isfinite(runtime.target_range_m) else runtime.target_range_m,
        closing_speed_mps=runtime.closing_speed_mps,
        sensor_pointing_pitch_rad=runtime.sensor_pointing_pitch_rad,
        sensor_pointing_yaw_rad=runtime.sensor_pointing_yaw_rad,
        sensor_los_rate_pitch_rad_s=runtime.sensor_pitch.rate_rad_s,
        sensor_los_rate_yaw_rad_s=runtime.sensor_yaw.rate_rad_s,
        normal_command_g=runtime.normal_command_g,
        lateral_command_g=runtime.lateral_command_g,
        normal_acceleration_g=runtime.normal_acceleration_g,
        lateral_acceleration_g=runtime.lateral_acceleration_g,
        ins_mode_requested=runtime.ins_mode_requested,
    )


####


def _point_sample(
    time_s: float,
    runtime: _PointMassRuntime,
    *,
    actor: Literal["TARGET3", "AIRCRAFT3"],
    alive: bool,
) -> Agm6PointMassSample:
    return Agm6PointMassSample(
        time_s=time_s,
        actor=actor,
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        speed_mps=runtime.speed_mps,
        heading_deg=runtime.heading_rad * _DEG_PER_RAD,
        flight_path_deg=runtime.flight_path_rad * _DEG_PER_RAD,
        altitude_m=-float(runtime.position_ned_m[2]),
        bank_deg=runtime.bank_rad * _DEG_PER_RAD,
        normal_load_g=runtime.normal_load_g,
        alive=alive,
    )


####


def _track_sample(track: _TrackPacket) -> Agm6TrackSample:
    return Agm6TrackSample(
        time_s=max(0.0, track.time_s),
        update_sequence=track.update_sequence,
        position_ned_m=_tuple3(track.position_ned_m),
        velocity_ned_mps=_tuple3(track.velocity_ned_mps),
    )


####


def _missile_is_finite(runtime: _MissileRuntime) -> bool:
    arrays = (
        runtime.position_ned_m,
        runtime.velocity_ned_mps,
        runtime.velocity_body_mps,
        runtime.quaternion_wxyz,
        runtime.body_rates_rad_s,
        runtime.wind_ned_mps,
    )
    scalars = (
        runtime.altitude_m,
        runtime.mach,
        runtime.dynamic_pressure_pa,
        runtime.propulsion.mass_kg,
        runtime.propulsion.thrust_n,
    )
    return all(np.all(np.isfinite(array)) for array in arrays) and all(math.isfinite(value) for value in scalars)


####


def _point_is_finite(runtime: _PointMassRuntime) -> bool:
    return np.all(np.isfinite(runtime.position_ned_m)) and np.all(np.isfinite(runtime.velocity_ned_mps)) and math.isfinite(runtime.speed_mps)


####


def _dcm_body_from_local(quaternion_wxyz: FloatVector) -> FloatMatrix:
    q0, q1, q2, q3 = (float(value) for value in quaternion_wxyz)
    return np.asarray(
        (
            (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2.0 * (q1 * q2 + q0 * q3), 2.0 * (q1 * q3 - q0 * q2)),
            (2.0 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2.0 * (q2 * q3 + q0 * q1)),
            (2.0 * (q1 * q3 + q0 * q2), 2.0 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3),
        ),
        dtype=np.float64,
    )


####


def _euler_from_dcm(transform: FloatMatrix) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -float(transform[0, 2]))))
    cosine_pitch = max(_SMALL, math.cos(pitch))
    yaw = math.acos(max(-1.0, min(1.0, float(transform[0, 0]) / cosine_pitch))) * _sign(float(transform[0, 1]))
    roll = math.acos(max(-1.0, min(1.0, float(transform[2, 2]) / cosine_pitch))) * _sign(float(transform[1, 2]))
    return (yaw, pitch, roll)


####


def _integrate_scalar(
    state: float,
    derivative_new: float,
    derivative_previous: float,
    dt_s: float,
) -> float:
    return state + 0.5 * (derivative_new + derivative_previous) * dt_s


####


def _integrate_vector(
    state: FloatVector,
    derivative_new: FloatVector,
    derivative_previous: FloatVector,
    dt_s: float,
) -> FloatVector:
    return state + 0.5 * (derivative_new + derivative_previous) * dt_s


####


def _cart_from_polar(magnitude: float, azimuth: float, elevation: float) -> FloatVector:
    return np.asarray(
        (
            magnitude * math.cos(elevation) * math.cos(azimuth),
            magnitude * math.cos(elevation) * math.sin(azimuth),
            -magnitude * math.sin(elevation),
        ),
        dtype=np.float64,
    )


####


def _polar_from_cartesian(vector: FloatVector) -> tuple[float, float, float]:
    north, east, down = (float(value) for value in vector)
    magnitude = math.sqrt(north * north + east * east + down * down)
    azimuth = math.atan2(east, north)
    horizontal = math.hypot(north, east)
    if horizontal > 0.0:
        elevation = math.atan2(-down, horizontal)
    elif down > 0.0:
        elevation = -math.pi / 2.0
    elif down < 0.0:
        elevation = math.pi / 2.0
    else:
        elevation = 0.0
    ####
    return (magnitude, azimuth, elevation)


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    assignments = tuple(assignment for assignment in vehicle.assignments if assignment.name.casefold() == name.casefold())
    if len(assignments) == 1:
        value = assignments[0].value
        if not isinstance(value, (int, float)):
            raise Agm6SourceError(f"AGM6 scalar {name!r} must be numeric")
        ####
        return float(value)
    ####
    if len(assignments) > 1:
        raise Agm6SourceError(f"AGM6 scalar {name!r} is assigned more than once before events")
    ####
    if default is None:
        raise Agm6SourceError(f"AGM6 source actor {vehicle.model_name!r} is missing scalar {name!r}")
    ####
    return float(default)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not float(value).is_integer():
        raise Agm6SourceError(f"AGM6 integer {name!r} must have an integral value")
    ####
    return int(value)


####


def _stochastic_parameter(
    vehicle: CadacVehicleBlock,
    name: str,
    kind: CadacStochasticKind,
    *,
    parameter_index: int,
) -> float | None:
    matches = tuple(assignment for assignment in vehicle.stochastic_assignments if assignment.name.casefold() == name.casefold() and assignment.kind is kind)
    if not matches:
        return None
    ####
    if len(matches) != 1:
        raise Agm6SourceError(f"AGM6 stochastic scalar {name!r} is assigned more than once")
    ####
    parameters = matches[0].parameters
    try:
        return float(parameters[parameter_index])
    except IndexError as error:
        raise Agm6SourceError(f"AGM6 stochastic scalar {name!r} is missing parameter {parameter_index}") from error
    ####


####


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


####


def _clip(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    ####
    return max(-limit, min(limit, value))


####


def _nonzero(value: float) -> float:
    if abs(value) >= _SMALL:
        return value
    ####
    return _SMALL if value >= 0.0 else -_SMALL


####


def _sign(value: float) -> int:
    return -1 if value < 0.0 else 1


####


def _tuple3(values: FloatVector | tuple[float, float, float]) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


####


def _tuple4(
    values: FloatVector | tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


####


__all__ = [
    "Agm6ActuatorConfig",
    "Agm6ActuatorStep",
    "Agm6AeroCoefficients",
    "Agm6AeroLimits",
    "Agm6AircraftConfig",
    "Agm6Airframe",
    "Agm6BodyWrench",
    "Agm6ControlCommand",
    "Agm6ControlConfig",
    "Agm6EnvironmentConfig",
    "Agm6EventTrace",
    "Agm6FinActuatorState",
    "Agm6FinSet",
    "Agm6GroundTargetConfig",
    "Agm6GuidanceConfig",
    "Agm6InitialState",
    "Agm6Intercept",
    "Agm6PointMassSample",
    "Agm6PropulsionConfig",
    "Agm6PropulsionStep",
    "Agm6RunResult",
    "Agm6ScenarioSession",
    "Agm6Sample",
    "Agm6SensorConfig",
    "Agm6SourceDefinition",
    "Agm6SourceError",
    "Agm6TrackSample",
    "Agm6TrackingConfig",
    "agm6_actuator_step",
    "agm6_aerodynamic_coefficients",
    "agm6_body_wrench",
    "agm6_initial_quaternion",
    "agm6_propulsion_step",
    "load_agm6_source_definition",
    "lower_agm6_source_bundle",
    "run_agm6_source_compatibility",
]
