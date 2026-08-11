"""Source-grounded SRAAM6 physical-surface missile and TARGET3 engagement runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from taoryx.sensor_api import SensorContext

from .aim5 import (
    Aim5TargetConfig,
    _AircraftState,
    _initialize_target,
    _packet_for_target,
    _target_control,
    _target_forces,
    _target_guidance,
    mat2tr,
)
from .aim5 import (
    _environment as _target_environment,
)
from .aim5 import (
    _newton as _target_newton,
)
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .deck import CadacDeck
from .events import CadacEventApplication, CadacEventCursor
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .sensor_adapter import cadac_local_ned_relative_state_track, cadac_local_ned_sensor_context
from .source_environment import atmosphere76, cadac_source_inverse_square_gravity_mps2

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_AGRAV = 9.80675445
_R_AIR = 287.053
_SMALL = 1.0e-10

_SRAAM6_MODULES = (
    "environment",
    "kinematics",
    "aerodynamics",
    "propulsion",
    "seeker",
    "guidance",
    "control",
    "actuator",
    "tvc",
    "forces",
    "euler",
    "newton",
    "intercept",
)
_SRAAM6_AERO_TABLES = (
    "ca0_vs_mach",
    "caa_vs_mach",
    "cad_vs_mach",
    "caoff_vs_mach",
    "clmq_vs_mach",
    "cn0_vs_mach_alpha",
    "cnp_vs_mach_alpha",
    "clm0_vs_mach_alpha",
    "clmp_vs_mach_alpha",
    "cyp_vs_mach_alpha",
    "cndq_vs_mach_alpha",
    "clmdq_vs_mach_alpha",
    "clnp_vs_mach_alpha",
    "cllap_vs_mach_alpha",
    "cllp_vs_mach_alpha",
    "clldp_vs_mach_alpha",
)
_SRAAM6_PROP_TABLES = (
    "mass_vs_time",
    "thrust_vs_time",
    "moipitch_vs_time",
    "moiroll_vs_time",
    "cg_vs_time",
)


class Sraam6SourceError(ValueError):
    """Source-bundle incompatibility with the SRAAM6 reconstruction."""


####


class Sraam6InitialState(CadacModel):
    """Source initial truth state for one SRAAM6 ``MISSILE6`` actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Sraam6Airframe(CadacModel):
    """Fixed source geometry and launch mass-property reference values."""

    reference_length_m: float = 0.1524
    reference_area_m2: float = 0.01824
    reference_cg_m: float = 1.536
    launch_mass_kg: float = 92.0
    launch_roll_inertia_kg_m2: float = 0.308
    launch_pitch_inertia_kg_m2: float = 59.80

    @model_validator(mode="after")
    def validate_positive(self) -> "Sraam6Airframe":
        values = (
            self.reference_length_m,
            self.reference_area_m2,
            self.launch_mass_kg,
            self.launch_roll_inertia_kg_m2,
            self.launch_pitch_inertia_kg_m2,
        )
        if any(value <= 0.0 or not math.isfinite(value) for value in values):
            raise ValueError("SRAAM6 source airframe values must be positive and finite")
        ####
        return self

    ####


####


class Sraam6AeroLimits(CadacModel):
    """Source aerodynamic and stopping limits."""

    alpha_limit_deg: float = Field(gt=0.0)
    structural_acceleration_limit_g: float = Field(gt=0.0)
    minimum_closing_speed_mps: float = 0.0
    minimum_mach: float = 0.8
    minimum_dynamic_pressure_pa: float = 10_000.0
    minimum_load_capacity_g: float = 3.0
    maximum_total_incidence_rad: float = 1.0


####


class Sraam6PropulsionConfig(CadacModel):
    """Single-pulse source rocket motor configuration."""

    mode: int
    nozzle_exit_area_m2: float = Field(ge=0.0)
    burnout_time_s: float = Field(default=2.69, gt=0.0)
    sea_level_pressure_pa: float = Field(default=101_325.0, gt=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Sraam6PropulsionConfig":
        if self.mode not in {0, 1}:
            raise ValueError("SRAAM6 source propulsion mode must be 0 or 1")
        ####
        return self

    ####


####


class Sraam6ActuatorConfig(CadacModel):
    """Independent four-fin actuator configuration."""

    mode: int
    position_limit_deg: float = Field(gt=0.0)
    rate_limit_deg_s: float = Field(gt=0.0)
    natural_frequency_rad_s: float = Field(gt=0.0)
    damping_ratio: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Sraam6ActuatorConfig":
        if self.mode not in {0, 2}:
            raise ValueError("SRAAM6 source actuator mode must be 0 or 2")
        ####
        return self

    ####


####


class Sraam6ControlConfig(CadacModel):
    """Source roll/rate/acceleration controller parameters."""

    initial_mode: int
    acceleration_natural_frequency_rad_s: float
    acceleration_damping_ratio: float
    acceleration_real_pole_rad_s: float
    structural_limit_g: float = Field(gt=0.0)
    pitch_command_limit_deg: float = Field(gt=0.0)
    yaw_command_limit_deg: float = Field(gt=0.0)
    roll_command_limit_deg: float = Field(gt=0.0)
    commanded_roll_deg: float
    roll_natural_frequency_rad_s: float = Field(gt=0.0)
    roll_damping_ratio: float = Field(ge=0.0)
    acceleration_feedforward_gain_s2_m: float = 0.0
    rate_loop_damping_ratio: float = Field(ge=0.0)
    rate_command_limit_deg_s: float = Field(default=1.0e9, gt=0.0)
    acceleration_frequency_factor: float = 0.0
    acceleration_damping_factor: float = 0.0

    @model_validator(mode="after")
    def validate_mode(self) -> "Sraam6ControlConfig":
        if self.initial_mode not in {0, 1, 2, 3}:
            raise ValueError("SRAAM6 source control mode must be 0, 1, 2, or 3")
        ####
        return self

    ####


####


class Sraam6SeekerConfig(CadacModel):
    """Source seeker mode machine and LOS-rate-filter parameters."""

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
    shooter_number: int = Field(ge=0)

    @model_validator(mode="after")
    def validate_modes(self) -> "Sraam6SeekerConfig":
        if self.initial_mode not in {0, 2, 3, 4, 5}:
            raise ValueError("SRAAM6 seeker mode must be 0, 2, 3, 4, or 5")
        ####
        if self.dynamic_mode not in {0, 1}:
            raise ValueError("SRAAM6 seeker dynamic mode must be 0 or 1")
        ####
        return self

    ####


####


class Sraam6GuidanceConfig(CadacModel):
    """Source proportional-navigation inputs."""

    initial_mode: int
    navigation_update_mode: int
    navigation_gain: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_modes(self) -> "Sraam6GuidanceConfig":
        if self.initial_mode not in {0, 3, 6}:
            raise ValueError("SRAAM6 guidance mode must be 0, 3, or 6")
        ####
        if self.navigation_update_mode not in {0, 3}:
            raise ValueError("SRAAM6 navigation-update mode must be 0 or 3")
        ####
        return self

    ####


####


class Sraam6SourceDefinition(CadacModel):
    """Prepared SRAAM6/TARGET3 case retaining actor and module order."""

    schema_id: str = "taoryx.cadac.sraam6-source/v0alpha1"
    source_name: str = Field(min_length=1)
    source_model: Literal["MISSILE6"] = "MISSILE6"
    target_source_model: Literal["TARGET3"] = "TARGET3"
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    vehicle_order: tuple[Literal["MISSILE6", "TARGET3"], Literal["MISSILE6", "TARGET3"]]
    initial_state: Sraam6InitialState
    airframe: Sraam6Airframe = Field(default_factory=Sraam6Airframe)
    aerodynamics: Sraam6AeroLimits
    propulsion: Sraam6PropulsionConfig
    actuator: Sraam6ActuatorConfig
    control: Sraam6ControlConfig
    seeker: Sraam6SeekerConfig
    guidance: Sraam6GuidanceConfig
    target: Aim5TargetConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    missile_events: tuple[CadacEventBlock, ...] = ()
    target_events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "rigid_body_6dof_surface_allocated"
    runtime_fidelity: str = "rigid_body_6dof"
    control_realization: str = "effector_allocated"

    @model_validator(mode="after")
    def validate_definition(self) -> "Sraam6SourceDefinition":
        if self.vehicle_order != ("MISSILE6", "TARGET3"):
            raise ValueError("SRAAM6 compatibility runtime requires source vehicle order MISSILE6 then TARGET3")
        ####
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop_names = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing_aero = tuple(name for name in _SRAAM6_AERO_TABLES if name.casefold() not in aero_names)
        missing_prop = tuple(name for name in _SRAAM6_PROP_TABLES if name.casefold() not in prop_names)
        if missing_aero or missing_prop:
            raise ValueError("SRAAM6 source definition is missing required tables: " + ", ".join((*missing_aero, *missing_prop)))
        ####
        if self.seeker.target_number != 1:
            raise ValueError("the first SRAAM6 plug-in runtime supports exactly one TARGET3 assignment")
        ####
        return self

    ####


####


class Sraam6ControlCommand(CadacModel):
    """Requested roll, pitch, and yaw aerodynamic control coordinates."""

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def vector(self) -> tuple[float, float, float]:
        return (self.roll_deg, self.pitch_deg, self.yaw_deg)

    ####


####


class Sraam6FinSet(CadacModel):
    """Requested or achieved physical fin deflections in source fin order."""

    fin1_deg: float = 0.0
    fin2_deg: float = 0.0
    fin3_deg: float = 0.0
    fin4_deg: float = 0.0

    def vector(self) -> tuple[float, float, float, float]:
        return (self.fin1_deg, self.fin2_deg, self.fin3_deg, self.fin4_deg)

    ####


####


class Sraam6FinActuatorState(CadacModel):
    """Stored-derivative state for four independent physical fins."""

    position_derivative_deg_s: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    position_deg: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    rate_derivative_deg_s2: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    rate_deg_s: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


####


class Sraam6ActuatorStep(CadacModel):
    """One visible command-mixing and physical-fin actuator transition."""

    requested_control: Sraam6ControlCommand
    requested_fins: Sraam6FinSet
    achieved_fins: Sraam6FinSet
    achieved_control: Sraam6ControlCommand
    state: Sraam6FinActuatorState
    position_limited: tuple[bool, bool, bool, bool]
    rate_limited: tuple[bool, bool, bool, bool]


####


class Sraam6PropulsionStep(CadacModel):
    """Source-deck mass properties and thrust at one accepted module call."""

    mode: int
    thrust_n: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    center_of_gravity_m: float
    roll_inertia_kg_m2: float = Field(gt=0.0)
    pitch_inertia_kg_m2: float = Field(gt=0.0)


####


class Sraam6AeroCoefficients(CadacModel):
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


class Sraam6BodyWrench(CadacModel):
    """Total source aerodynamic and propulsive body wrench."""

    force_n: tuple[float, float, float]
    moment_nm: tuple[float, float, float]


####


class Sraam6Intercept(CadacModel):
    """Closest-approach terminal result."""

    time_s: float = Field(ge=0.0)
    miss_distance_m: float = Field(ge=0.0)
    miss_vector_ned_m: tuple[float, float, float]
    differential_speed_mps: float = Field(ge=0.0)


####


class Sraam6EventTrace(CadacModel):
    """One source event accepted before the actor module pass."""

    time_s: float = Field(ge=0.0)
    actor: Literal["MISSILE6", "TARGET3"]
    event_index: int = Field(ge=0)
    source_line: int = Field(ge=1)
    watch_variable: str = Field(min_length=1)
    operator: str = Field(min_length=1, max_length=1)
    criterion: int | float
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]


####


class Sraam6Sample(CadacModel):
    """Accepted missile truth and physical-control telemetry sample."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    altitude_m: float
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
    center_of_gravity_m: float
    roll_inertia_kg_m2: float = Field(gt=0.0)
    pitch_inertia_kg_m2: float = Field(gt=0.0)
    thrust_n: float = Field(ge=0.0)
    propulsion_mode: int
    seeker_mode: int
    guidance_mode: int
    autopilot_mode: int
    target_range_m: float = Field(ge=0.0)
    closing_speed_mps: float
    seeker_pointing_pitch_rad: float
    seeker_pointing_yaw_rad: float
    seeker_los_rate_pitch_rad_s: float
    seeker_los_rate_yaw_rad_s: float
    normal_command_g: float
    lateral_command_g: float
    normal_acceleration_g: float
    lateral_acceleration_g: float


####


class Sraam6TargetSample(CadacModel):
    """Independent TARGET3 root-object truth sample."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    altitude_m: float
    bank_deg: float
    normal_load_g: float
    alive: bool


####


class Sraam6RunResult(CadacModel):
    """Deterministic source-ordered SRAAM6/TARGET3 engagement result."""

    schema_id: str = "taoryx.cadac.sraam6-run/v0alpha1"
    source_name: str
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: Literal["intercept", "ground_impact", "nonfinite_state", "end_time"]
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    samples: tuple[Sraam6Sample, ...] = Field(min_length=1)
    target_samples: tuple[Sraam6TargetSample, ...] = Field(min_length=1)
    intercept: Sraam6Intercept | None = None
    event_trace: tuple[Sraam6EventTrace, ...] = ()
    claim_boundary: str = (
        "The rigid-body, source aerodynamic tables, four-fin physical actuators, source propulsion, target, "
        "proportional-navigation guidance, and controller mode transitions execute in source order. The dynamic "
        "seeker retains the source acquisition/lock/blind-range state machine and a second-order LOS-rate filter, "
        "but the complete optical error, aimpoint, and gimbal-head geometry remains outside this reconstruction."
    )


####


@dataclass(slots=True)
class _SeekerAxisState:
    rate_rad_s: float = 0.0
    rate_derivative_rad_s2: float = 0.0
    acceleration_rad_s2: float = 0.0
    acceleration_derivative_rad_s3: float = 0.0


####


@dataclass(slots=True)
class _Sraam6Runtime:
    position_ned_m: FloatVector
    position_derivative_ned_mps: FloatVector
    velocity_body_mps: FloatVector
    velocity_body_derivative_mps2: FloatVector
    velocity_ned_mps: FloatVector
    quaternion_wxyz: FloatVector
    quaternion_derivative: FloatVector
    body_rates_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    fin_state: Sraam6FinActuatorState
    requested_control: Sraam6ControlCommand
    actuator_step: Sraam6ActuatorStep
    propulsion: Sraam6PropulsionStep
    coefficients: Sraam6AeroCoefficients
    wrench: Sraam6BodyWrench
    control_mode: int
    guidance_mode: int
    navigation_update_mode: int
    propulsion_mode: int
    seeker_mode: int
    event_time_s: float = 0.0
    event_epoch_s: float = 0.0
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    total_alpha_deg: float = 0.0
    aerodynamic_roll_rad: float = 0.0
    altitude_m: float = 0.0
    gravity_mps2: float = 0.0
    density_kg_m3: float = 0.0
    pressure_pa: float = 0.0
    temperature_k: float = 0.0
    speed_of_sound_mps: float = 0.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    normal_command_g: float = 0.0
    lateral_command_g: float = 0.0
    normal_acceleration_g: float = 0.0
    lateral_acceleration_g: float = 0.0
    roll_feedforward_state: float = 0.0
    yaw_feedforward_derivative: float = 0.0
    yaw_feedforward_state: float = 0.0
    pitch_feedforward_derivative: float = 0.0
    pitch_feedforward_state: float = 0.0
    target_position_stored_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    target_velocity_stored_ned_mps: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    target_update_epoch_s: float = 0.0
    target_range_m: float = math.inf
    closing_speed_mps: float = 0.0
    unit_los_local: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    unit_los_body: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    true_los_rate_body_rad_s: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    seeker_pointing_pitch_rad: float = 0.0
    seeker_pointing_yaw_rad: float = 0.0
    seeker_pointing_pitch_derivative_rad_s: float = 0.0
    seeker_pointing_yaw_derivative_rad_s: float = 0.0
    seeker_acquisition_epoch_s: float = 0.0
    seeker_initialized: bool = False
    seeker_pitch: _SeekerAxisState = field(default_factory=_SeekerAxisState)
    seeker_yaw: _SeekerAxisState = field(default_factory=_SeekerAxisState)
    previous_relative_position_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    previous_missile_position_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    previous_target_position_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    previous_time_s: float = 0.0
    entered_intercept_sphere: bool = False


####


class Sraam6ScenarioSession:
    """Persistent source-ordered SRAAM6/TARGET3 engagement state.

    The session owns the same missile, target, event, seeker, and source-bus
    state that the batch runner advances.  Holds are deliberately restricted
    to exact source-step multiples: this preserves the source actor ordering
    and its intentional previous-target-pass seeker observation.
    """

    def __init__(
        self,
        definition: Sraam6SourceDefinition,
        *,
        initial_state: Sraam6InitialState | None = None,
        target_config: Aim5TargetConfig | None = None,
    ) -> None:
        self.definition = definition
        self._initial_state = initial_state or definition.initial_state
        self._target_config = target_config or definition.target
        self.reset()
        ####

    def reset(self) -> None:
        """Reconstruct all source-owned actor, event, and seeker state."""

        self.runtime = _initialize_runtime(self.definition, self._initial_state)
        self.target = _initialize_target(self._target_config)
        self.target_bus = _packet_for_target(self.target, alive=True)
        self.missile_events = CadacEventCursor.from_events(self.definition.missile_events)
        self.target_events = CadacEventCursor.from_events(self.definition.target_events)
        self.event_trace: list[Sraam6EventTrace] = []
        self.intercept: Sraam6Intercept | None = None
        self.missile_alive = True
        self.target_alive = True
        self.target_event_epoch_s = 0.0
        self.sim_time_s = 0.0
        self.executed_steps = 0
        self.terminated_reason: Literal["intercept", "ground_impact", "nonfinite_state", "end_time"] | None = None
        ####

    @property
    def completed(self) -> bool:
        return self.terminated_reason is not None
        ####

    @property
    def sample(self) -> Sraam6Sample:
        """Return the latest committed missile state."""

        return _sample(self.sim_time_s, self.runtime)
        ####

    @property
    def target_sample(self) -> Sraam6TargetSample:
        """Return the latest committed target state."""

        return _target_sample(self.sim_time_s, self.target, alive=self.target_alive)
        ####

    def native_sensor_context(self) -> SensorContext:
        """Return raw committed target geometry through the native sensor API."""

        return cadac_local_ned_sensor_context(
            time_s=self.sim_time_s,
            host_position_ned_m=self.runtime.position_ned_m,
            host_velocity_ned_mps=self.runtime.velocity_ned_mps,
            target_id="sraam6-target",
            target_position_ned_m=self.target.flat.position_ned_m,
            target_velocity_ned_mps=self.target.flat.velocity_ned_mps,
            body_from_local=_dcm_body_from_local(self.runtime.quaternion_wxyz),
            host_body_rate_rad_s=self.runtime.body_rates_rad_s,
        )
        ####

    def advance(self, duration_s: float) -> tuple[Sraam6EventTrace, ...]:
        """Advance whole source passes and return event traces committed by this hold."""

        steps = _session_step_count(duration_s, self.definition.integration_step_s, "SRAAM6")
        start = len(self.event_trace)
        for _ in range(steps):
            if self.completed:
                break
            ####
            self._advance_one()
        ####
        return tuple(self.event_trace[start:])
        ####

    def _advance_one(self) -> None:
        sim_time_s = self.sim_time_s
        self.executed_steps += 1
        if self.missile_alive:
            application = _apply_missile_event(self.runtime, self.missile_events, sim_time_s)
            if application is not None:
                self.event_trace.append(_event_trace(application, sim_time_s, "MISSILE6"))
            ####
            self.runtime.event_time_s = sim_time_s - self.runtime.event_epoch_s
            for module in self.definition.module_order:
                if module == "environment":
                    _missile_environment(self.runtime)
                elif module == "kinematics":
                    _missile_kinematics(self.runtime, self.definition.integration_step_s)
                elif module == "aerodynamics":
                    self.runtime.coefficients = sraam6_aerodynamic_coefficients(self.definition, self.runtime)
                elif module == "propulsion":
                    self.runtime.propulsion = sraam6_propulsion_step(
                        self.definition,
                        time_s=sim_time_s,
                        pressure_pa=self.runtime.pressure_pa,
                        mode=self.runtime.propulsion_mode,
                    )
                    self.runtime.propulsion_mode = self.runtime.propulsion.mode
                elif module == "seeker":
                    _missile_seeker(self.runtime, self.definition.seeker, self.target_bus, sim_time_s, self.definition.integration_step_s)
                elif module == "guidance":
                    _missile_guidance(self.runtime, self.definition.guidance, self.target_bus, sim_time_s)
                elif module == "control":
                    self.runtime.requested_control = _missile_control(self.runtime, self.definition.control, self.definition.integration_step_s)
                elif module == "actuator":
                    self.runtime.actuator_step = sraam6_actuator_step(
                        self.definition.actuator,
                        self.runtime.fin_state,
                        self.runtime.requested_control,
                        self.definition.integration_step_s,
                    )
                    self.runtime.fin_state = self.runtime.actuator_step.state
                elif module == "tvc":
                    pass
                elif module == "forces":
                    self.runtime.wrench = sraam6_body_wrench(self.definition, self.runtime)
                elif module == "euler":
                    _missile_euler(self.runtime, self.definition.integration_step_s)
                elif module == "newton":
                    _missile_newton(self.runtime, self.definition.integration_step_s)
                elif module == "intercept":
                    self.intercept = _missile_intercept(self.runtime, self.target_bus, sim_time_s, self.definition.integration_step_s)
                    if self.intercept is not None:
                        self.missile_alive = False
                        self.target_alive = False
                        self.terminated_reason = "intercept"
                    elif self.runtime.altitude_m <= 0.0:
                        self.missile_alive = False
                        self.terminated_reason = "ground_impact"
                    ####
                ####
            ####
        ####
        if self.target_alive:
            application, self.target_event_epoch_s = _apply_target_event(
                self.target,
                self.target_events,
                sim_time=sim_time_s,
                event_epoch_s=self.target_event_epoch_s,
            )
            if application is not None:
                self.event_trace.append(_event_trace(application, sim_time_s, "TARGET3"))
            ####
            for module in self.definition.module_order:
                if module == "environment":
                    _target_environment(self.target.flat)
                elif module == "guidance":
                    _target_guidance(self.target)
                elif module == "control":
                    _target_control(self.target, self.definition.integration_step_s)
                elif module == "forces":
                    _target_forces(self.target)
                elif module == "newton":
                    _target_newton(self.target.flat, self.definition.integration_step_s)
                ####
            ####
            self.target.flat.time_s = sim_time_s
            self.target_bus = _packet_for_target(self.target, alive=True)
        ####
        self.sim_time_s = min(self.definition.end_time_s, sim_time_s + self.definition.integration_step_s)
        if not _runtime_is_finite(self.runtime) or not _target_is_finite(self.target):
            self.terminated_reason = "nonfinite_state"
        elif self.terminated_reason is None and self.sim_time_s >= self.definition.end_time_s - 1.0e-12:
            self.terminated_reason = "end_time"
        ####

    ####


def _session_step_count(duration_s: float, source_step_s: float, model_name: str) -> int:
    """Validate an externally held duration without synthesising source substeps."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError(f"{model_name} session duration_s must be positive and finite")
    ####
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"{model_name} session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    ####
    return steps
    ####


def lower_sraam6_source_bundle(bundle: CadacSourceBundle) -> Sraam6SourceDefinition:
    """Lower one default-shape SRAAM6/TARGET3 source case into typed inputs."""

    missiles = bundle.case.vehicles_named("MISSILE6")
    targets = bundle.case.vehicles_named("TARGET3")
    if len(missiles) != 1 or len(targets) != 1 or len(bundle.case.vehicles) != 2:
        raise Sraam6SourceError(
            "SRAAM6 source lowering requires exactly one MISSILE6 and one TARGET3; "
            f"found {len(missiles)} missile(s), {len(targets)} target(s), {len(bundle.case.vehicles)} total"
        )
    ####
    vehicle_order = tuple(vehicle.model_name.upper() for vehicle in bundle.case.vehicles)
    if vehicle_order != ("MISSILE6", "TARGET3"):
        raise Sraam6SourceError("SRAAM6 source-compatible bus lag requires MISSILE6 before TARGET3")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _SRAAM6_MODULES)
    if unknown:
        raise Sraam6SourceError(f"SRAAM6 reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _SRAAM6_MODULES if name not in module_order)
    if missing:
        raise Sraam6SourceError(f"SRAAM6 reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Sraam6SourceError("SRAAM6 source case must declare TIMING int_step") from error
    ####
    missile = missiles[0]
    target = targets[0]
    mtvc = _integer(missile, "mtvc", 0)
    if mtvc != 0:
        raise Sraam6SourceError("the first runnable SRAAM6 realization is the standard four-fin source path; optional TVC remains validate-only")
    ####
    return Sraam6SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        vehicle_order=("MISSILE6", "TARGET3"),
        initial_state=Sraam6InitialState(
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
        ),
        aerodynamics=Sraam6AeroLimits(
            alpha_limit_deg=_number(missile, "alplimx"),
            structural_acceleration_limit_g=_number(missile, "alimit", 50.0),
            minimum_closing_speed_mps=_number(missile, "trcvel", 0.0),
            minimum_mach=_number(missile, "trmach", 0.8),
            minimum_dynamic_pressure_pa=_number(missile, "trdynm", 10_000.0),
            minimum_load_capacity_g=_number(missile, "trload", 3.0),
            maximum_total_incidence_rad=_number(missile, "tralp", 1.0),
        ),
        propulsion=Sraam6PropulsionConfig(
            mode=_integer(missile, "mprop", 0),
            nozzle_exit_area_m2=_number(missile, "aexit", 0.0),
        ),
        actuator=Sraam6ActuatorConfig(
            mode=_integer(missile, "mact", 0),
            position_limit_deg=_number(missile, "dlimx"),
            rate_limit_deg_s=_number(missile, "ddlimx"),
            natural_frequency_rad_s=_number(missile, "wnact"),
            damping_ratio=_number(missile, "zetact"),
        ),
        control=Sraam6ControlConfig(
            initial_mode=_integer(missile, "maut", 0),
            acceleration_natural_frequency_rad_s=_number(missile, "wacl", 0.0),
            acceleration_damping_ratio=_number(missile, "zacl", 0.0),
            acceleration_real_pole_rad_s=_number(missile, "pacl", 0.0),
            structural_limit_g=_number(missile, "alimit"),
            pitch_command_limit_deg=_number(missile, "dqlimx"),
            yaw_command_limit_deg=_number(missile, "drlimx"),
            roll_command_limit_deg=_number(missile, "dplimx"),
            commanded_roll_deg=_number(missile, "phicomx", 0.0),
            roll_natural_frequency_rad_s=_number(missile, "wrcl"),
            roll_damping_ratio=_number(missile, "zrcl"),
            acceleration_feedforward_gain_s2_m=_number(missile, "gainp", 0.0),
            rate_loop_damping_ratio=_number(missile, "zetlagr"),
            rate_command_limit_deg_s=_number(missile, "ratelimx", 1.0e9),
            acceleration_frequency_factor=_number(missile, "factwacl", 0.0),
            acceleration_damping_factor=_number(missile, "factzacl", 0.0),
        ),
        seeker=Sraam6SeekerConfig(
            initial_mode=_integer(missile, "mseek", 0),
            dynamic_mode=_integer(missile, "ms1dyn", 0),
            blind_range_m=_number(missile, "dblind", 0.0),
            acquisition_range_m=_number(missile, "racq", 1.0e9),
            acquisition_time_s=_number(missile, "dtimac", 0.0),
            filter_gain_per_s=_number(missile, "gk", 0.0),
            filter_damping_ratio=_number(missile, "zetak", 0.0),
            filter_natural_frequency_rad_s=_number(missile, "wnk", 1.0),
            yaw_half_fov_rad=_number(missile, "fovyaw", math.pi),
            pitch_half_fov_rad=_number(missile, "fovpitch", math.pi),
            target_number=_integer(missile, "tgt_num", 1),
            shooter_number=_integer(missile, "sht_num", 0),
        ),
        guidance=Sraam6GuidanceConfig(
            initial_mode=_integer(missile, "mguid", 0),
            navigation_update_mode=_integer(missile, "mnav", 0),
            navigation_gain=_number(missile, "gnav", 0.0),
        ),
        target=Aim5TargetConfig(
            position_ned_m=(
                _number(target, "sael1"),
                _number(target, "sael2"),
                _number(target, "sael3"),
            ),
            speed_mps=_number(target, "dvae"),
            heading_deg=_number(target, "psialx"),
            flight_path_deg=_number(target, "thtalx"),
            aircraft_option=_integer(target, "tgt_option", 0),
            guidance_gain=_number(target, "guid_gain", 0.0),
            turn_g=_number(target, "gturn", 0.0),
            bank_time_constant_s=_number(target, "tphi", 0.2),
            bank_limit_deg=_number(target, "philimx", 120.0),
            normal_load_time_constant_s=_number(target, "tanx", 0.1),
            alpha_limit_deg=_number(target, "alplimx", 40.0),
            lift_slope_per_deg=_number(target, "clalpha", 0.0523),
            wing_loading_n_m2=_number(target, "wingloading", 3247.0),
            longitudinal_acceleration_g=_number(target, "acc_longx", 0.0),
        ),
        aerodynamic_deck=bundle.deck_for("MISSILE6", CadacDeckKind.AERODYNAMIC),
        propulsion_deck=bundle.deck_for("MISSILE6", CadacDeckKind.PROPULSION),
        missile_events=missile.events,
        target_events=target.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_sraam6_source_definition(path: str | Path) -> Sraam6SourceDefinition:
    """Parse, fingerprint, and lower one SRAAM6 source case."""

    return lower_sraam6_source_bundle(load_cadac_source_bundle(path))


####


def sraam6_initial_quaternion(initial: Sraam6InitialState) -> tuple[float, float, float, float]:
    """Return the source scalar-first quaternion from yaw/pitch/roll."""

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


def sraam6_mix_fin_commands(command: Sraam6ControlCommand) -> Sraam6FinSet:
    """Map roll/pitch/yaw commands into the source four-fin cruciform mixer."""

    roll, pitch, yaw = command.vector()
    return Sraam6FinSet(
        fin1_deg=-roll + pitch - yaw,
        fin2_deg=-roll + pitch + yaw,
        fin3_deg=roll + pitch - yaw,
        fin4_deg=roll + pitch + yaw,
    )


####


def sraam6_unmix_fin_positions(fins: Sraam6FinSet) -> Sraam6ControlCommand:
    """Recover achieved roll/pitch/yaw coordinates from physical fin positions."""

    fin1, fin2, fin3, fin4 = fins.vector()
    return Sraam6ControlCommand(
        roll_deg=(-fin1 - fin2 + fin3 + fin4) / 4.0,
        pitch_deg=(fin1 + fin2 + fin3 + fin4) / 4.0,
        yaw_deg=(-fin1 + fin2 - fin3 + fin4) / 4.0,
    )


####


def sraam6_actuator_step(
    config: Sraam6ActuatorConfig,
    state: Sraam6FinActuatorState,
    command: Sraam6ControlCommand,
    dt_s: float,
) -> Sraam6ActuatorStep:
    """Advance independent physical fins using the exact source saturation order."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("SRAAM6 actuator dt_s must be positive and finite")
    ####
    requested_fins = sraam6_mix_fin_commands(command)
    requested = np.asarray(requested_fins.vector(), dtype=np.float64)
    if config.mode == 0:
        achieved = np.clip(requested, -config.position_limit_deg, config.position_limit_deg)
        achieved_fins = _fin_set(achieved)
        return Sraam6ActuatorStep(
            requested_control=command,
            requested_fins=requested_fins,
            achieved_fins=achieved_fins,
            achieved_control=sraam6_unmix_fin_positions(achieved_fins),
            state=state,
            position_limited=tuple(bool(abs(value) > config.position_limit_deg) for value in requested),
            rate_limited=(False, False, False, False),
        )
    ####
    position_derivative = np.asarray(state.position_derivative_deg_s, dtype=np.float64).copy()
    position = np.asarray(state.position_deg, dtype=np.float64).copy()
    rate_derivative = np.asarray(state.rate_derivative_deg_s2, dtype=np.float64).copy()
    rate = np.asarray(state.rate_deg_s, dtype=np.float64).copy()
    position_limited = [False, False, False, False]
    rate_limited = [False, False, False, False]
    wn = config.natural_frequency_rad_s
    damping = config.damping_ratio
    for index in range(4):
        if abs(position[index]) > config.position_limit_deg:
            position_limited[index] = True
            position[index] = math.copysign(config.position_limit_deg, position[index])
            if position[index] * rate[index] > 0.0:
                rate[index] = 0.0
            ####
        ####
        was_rate_limited = abs(rate[index]) > config.rate_limit_deg_s
        if was_rate_limited:
            rate_limited[index] = True
            rate[index] = math.copysign(config.rate_limit_deg_s, rate[index])
        ####
        derivative_new = rate[index]
        position[index] = _integrate_scalar(position[index], derivative_new, position_derivative[index], dt_s)
        position_derivative[index] = derivative_new
        error = requested[index] - position[index]
        acceleration_new = wn * wn * error - 2.0 * damping * wn * position_derivative[index]
        rate[index] = _integrate_scalar(rate[index], acceleration_new, rate_derivative[index], dt_s)
        rate_derivative[index] = acceleration_new
        if was_rate_limited and rate[index] * rate_derivative[index] > 0.0:
            rate_derivative[index] = 0.0
        ####
    ####
    achieved_fins = _fin_set(position)
    return Sraam6ActuatorStep(
        requested_control=command,
        requested_fins=requested_fins,
        achieved_fins=achieved_fins,
        achieved_control=sraam6_unmix_fin_positions(achieved_fins),
        state=Sraam6FinActuatorState(
            position_derivative_deg_s=_tuple4(position_derivative),
            position_deg=_tuple4(position),
            rate_derivative_deg_s2=_tuple4(rate_derivative),
            rate_deg_s=_tuple4(rate),
        ),
        position_limited=tuple(position_limited),
        rate_limited=tuple(rate_limited),
    )


####


def sraam6_propulsion_step(
    definition: Sraam6SourceDefinition,
    *,
    time_s: float,
    pressure_pa: float,
    mode: int,
) -> Sraam6PropulsionStep:
    """Evaluate source-deck thrust and time-varying mass properties."""

    query_time = max(0.0, time_s)
    deck = definition.propulsion_deck
    mass = deck.table("mass_vs_time").interpolate((query_time,))
    center_of_gravity = deck.table("cg_vs_time").interpolate((query_time,))
    pitch_inertia = deck.table("moipitch_vs_time").interpolate((query_time,))
    roll_inertia = deck.table("moiroll_vs_time").interpolate((query_time,))
    active_mode = mode
    thrust = 0.0
    if active_mode == 1 and time_s <= definition.propulsion.burnout_time_s:
        thrust_sl = deck.table("thrust_vs_time").interpolate((query_time,))
        thrust = thrust_sl + (definition.propulsion.sea_level_pressure_pa - pressure_pa) * definition.propulsion.nozzle_exit_area_m2
        thrust = max(0.0, thrust)
    elif time_s > definition.propulsion.burnout_time_s:
        active_mode = 0
    ####
    return Sraam6PropulsionStep(
        mode=active_mode,
        thrust_n=thrust,
        mass_kg=mass,
        center_of_gravity_m=center_of_gravity,
        roll_inertia_kg_m2=roll_inertia,
        pitch_inertia_kg_m2=pitch_inertia,
    )


####


def sraam6_aerodynamic_coefficients(
    definition: Sraam6SourceDefinition,
    runtime: _Sraam6Runtime,
) -> Sraam6AeroCoefficients:
    """Evaluate the source aeroballistic tables and body-axis coefficient closure."""

    deck = definition.aerodynamic_deck
    mach = runtime.mach
    alpha_total = runtime.total_alpha_deg
    phi = runtime.aerodynamic_roll_rad
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    sin4phi = math.sin(4.0 * phi)
    sin2phi_squared = math.sin(2.0 * phi) ** 2
    speed = max(1.0e-9, float(np.linalg.norm(runtime.velocity_ned_mps)))
    roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s = runtime.body_rates_rad_s * _DEG_PER_RAD
    achieved = runtime.actuator_step.achieved_control
    roll_control = achieved.roll_deg
    pitch_control = achieved.pitch_deg
    yaw_control = achieved.yaw_deg
    pitch_control_aero = pitch_control * cphi - yaw_control * sphi
    yaw_control_aero = pitch_control * sphi + yaw_control * cphi
    pitch_rate_aero_deg_s = pitch_rate_deg_s * cphi - yaw_rate_deg_s * sphi
    yaw_rate_aero_deg_s = pitch_rate_deg_s * sphi + yaw_rate_deg_s * cphi

    ca0 = deck.table("ca0_vs_mach").interpolate((mach,))
    caa = deck.table("caa_vs_mach").interpolate((mach,))
    cad = deck.table("cad_vs_mach").interpolate((mach,))
    caoff = deck.table("caoff_vs_mach").interpolate((mach,))
    effective_control = (abs(pitch_control_aero) + abs(yaw_control_aero)) / 2.0
    axial = ca0 + caa * alpha_total + cad * effective_control * effective_control
    if runtime.propulsion_mode == 0:
        axial += caoff
    ####

    cyp = deck.table("cyp_vs_mach_alpha").interpolate((mach, alpha_total))
    control_normal = deck.table("cndq_vs_mach_alpha").interpolate((mach, alpha_total))
    side_aero = cyp * sin4phi + control_normal * yaw_control_aero
    cn0 = deck.table("cn0_vs_mach_alpha").interpolate((mach, alpha_total))
    cnp = deck.table("cnp_vs_mach_alpha").interpolate((mach, alpha_total))
    normal_aero = cn0 + cnp * sin2phi_squared + control_normal * pitch_control_aero

    cllap = deck.table("cllap_vs_mach_alpha").interpolate((mach, alpha_total))
    cllp = deck.table("cllp_vs_mach_alpha").interpolate((mach, alpha_total))
    clldp = deck.table("clldp_vs_mach_alpha").interpolate((mach, alpha_total))
    roll_moment = (
        cllap * alpha_total * alpha_total * sin4phi + cllp * roll_rate_deg_s * definition.airframe.reference_length_m / (2.0 * speed) + clldp * roll_control
    )

    clm0 = deck.table("clm0_vs_mach_alpha").interpolate((mach, alpha_total))
    clmp = deck.table("clmp_vs_mach_alpha").interpolate((mach, alpha_total))
    clmq = deck.table("clmq_vs_mach").interpolate((mach,))
    clmdq = deck.table("clmdq_vs_mach_alpha").interpolate((mach, alpha_total))
    pitch_reference = (
        clm0 + clmp * sin2phi_squared + clmq * pitch_rate_aero_deg_s * definition.airframe.reference_length_m / (2.0 * speed) + clmdq * pitch_control_aero
    )
    pitch_aero = (
        pitch_reference - normal_aero * (definition.airframe.reference_cg_m - runtime.propulsion.center_of_gravity_m) / definition.airframe.reference_length_m
    )

    clnp = deck.table("clnp_vs_mach_alpha").interpolate((mach, alpha_total))
    yaw_reference = clnp * sin4phi + clmq * yaw_rate_aero_deg_s * definition.airframe.reference_length_m / (2.0 * speed) + clmdq * yaw_control_aero
    yaw_aero = (
        yaw_reference - side_aero * (definition.airframe.reference_cg_m - runtime.propulsion.center_of_gravity_m) / definition.airframe.reference_length_m
    )

    side = side_aero * cphi - normal_aero * sphi
    normal = side_aero * sphi + normal_aero * cphi
    pitch_moment = pitch_aero * cphi + yaw_aero * sphi
    yaw_moment = -pitch_aero * sphi + yaw_aero * cphi

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
        pitch_alpha_per_rad = (pitch_plus - pitch_minus) / span * _DEG_PER_RAD - normal_alpha_per_rad * (
            definition.airframe.reference_cg_m - runtime.propulsion.center_of_gravity_m
        ) / definition.airframe.reference_length_m
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
            / runtime.propulsion.pitch_inertia_kg_m2
        )
        pitch_alpha_derivative = pitch_scale * pitch_alpha_per_rad
        pitch_rate_derivative = pitch_scale * definition.airframe.reference_length_m / (2.0 * speed) * pitch_rate_per_rad
        pitch_control_derivative = pitch_scale * pitch_control_per_rad
        roll_scale = (
            runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2 * definition.airframe.reference_length_m / runtime.propulsion.roll_inertia_kg_m2
        )
        roll_rate_derivative = roll_scale * definition.airframe.reference_length_m / (2.0 * speed) * roll_rate_per_rad
        roll_control_derivative = roll_scale * roll_control_per_rad
    ####

    max_normal = deck.table("cn0_vs_mach_alpha").interpolate((mach, definition.aerodynamics.alpha_limit_deg))
    weight = runtime.propulsion.mass_kg * _AGRAV
    max_g = max_normal * runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2 / weight
    max_g = max(0.0, min(max_g, definition.aerodynamics.structural_acceleration_limit_g))
    return Sraam6AeroCoefficients(
        axial=axial,
        side=side,
        normal=normal,
        roll_moment=roll_moment,
        pitch_moment=pitch_moment,
        yaw_moment=yaw_moment,
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


def sraam6_body_wrench(
    definition: Sraam6SourceDefinition,
    runtime: _Sraam6Runtime,
) -> Sraam6BodyWrench:
    """Close aerodynamic coefficients and axial motor thrust into the body wrench."""

    q_area = runtime.dynamic_pressure_pa * definition.airframe.reference_area_m2
    coefficients = runtime.coefficients
    force = (
        -q_area * coefficients.axial + runtime.propulsion.thrust_n,
        q_area * coefficients.side,
        -q_area * coefficients.normal,
    )
    moment_scale = q_area * definition.airframe.reference_length_m
    moment = (
        moment_scale * coefficients.roll_moment,
        moment_scale * coefficients.pitch_moment,
        moment_scale * coefficients.yaw_moment,
    )
    return Sraam6BodyWrench(force_n=force, moment_nm=moment)


####


def run_sraam6_source_compatibility(
    definition: Sraam6SourceDefinition,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
    initial_state: Sraam6InitialState | None = None,
    target_config: Aim5TargetConfig | None = None,
) -> Sraam6RunResult:
    """Execute the standard four-fin SRAAM6 engagement in source actor/module order."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    cadence = sample_step_s if sample_step_s is not None else definition.plot_step_s or max(dt_s, 0.02)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("SRAAM6 end_time_s must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("SRAAM6 sample_step_s must be positive and finite")
    ####
    runtime = _initialize_runtime(definition, initial_state or definition.initial_state)
    target = _initialize_target(target_config or definition.target)
    target_bus = _packet_for_target(target, alive=True)
    missile_events = CadacEventCursor.from_events(definition.missile_events)
    target_events = CadacEventCursor.from_events(definition.target_events)
    samples: list[Sraam6Sample] = []
    target_samples: list[Sraam6TargetSample] = []
    event_trace: list[Sraam6EventTrace] = []
    intercept: Sraam6Intercept | None = None
    missile_alive = True
    target_alive = True
    target_event_epoch_s = 0.0
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0
    terminated_reason: Literal["intercept", "ground_impact", "nonfinite_state", "end_time"] = "end_time"
    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1
        if missile_alive:
            application = _apply_missile_event(runtime, missile_events, sim_time)
            if application is not None:
                event_trace.append(_event_trace(application, sim_time, "MISSILE6"))
            ####
            runtime.event_time_s = sim_time - runtime.event_epoch_s
            for module in definition.module_order:
                if module == "environment":
                    _missile_environment(runtime)
                elif module == "kinematics":
                    _missile_kinematics(runtime, dt_s)
                elif module == "aerodynamics":
                    runtime.coefficients = sraam6_aerodynamic_coefficients(definition, runtime)
                elif module == "propulsion":
                    runtime.propulsion = sraam6_propulsion_step(
                        definition,
                        time_s=sim_time,
                        pressure_pa=runtime.pressure_pa,
                        mode=runtime.propulsion_mode,
                    )
                    runtime.propulsion_mode = runtime.propulsion.mode
                elif module == "seeker":
                    _missile_seeker(runtime, definition.seeker, target_bus, sim_time, dt_s)
                elif module == "guidance":
                    _missile_guidance(runtime, definition.guidance, target_bus, sim_time)
                elif module == "control":
                    runtime.requested_control = _missile_control(runtime, definition.control, dt_s)
                elif module == "actuator":
                    runtime.actuator_step = sraam6_actuator_step(
                        definition.actuator,
                        runtime.fin_state,
                        runtime.requested_control,
                        dt_s,
                    )
                    runtime.fin_state = runtime.actuator_step.state
                elif module == "tvc":
                    # The standard source case has mtvc=0; optional TVC is a separate validate-only phase.
                    pass
                elif module == "forces":
                    runtime.wrench = sraam6_body_wrench(definition, runtime)
                elif module == "euler":
                    _missile_euler(runtime, dt_s)
                elif module == "newton":
                    _missile_newton(runtime, dt_s)
                elif module == "intercept":
                    intercept = _missile_intercept(runtime, target_bus, sim_time, dt_s)
                    if intercept is not None:
                        missile_alive = False
                        target_alive = False
                        terminated_reason = "intercept"
                    elif runtime.altitude_m <= 0.0:
                        missile_alive = False
                        terminated_reason = "ground_impact"
                    ####
                ####
            ####
        ####
        if target_alive:
            target_application, target_event_epoch_s = _apply_target_event(
                target,
                target_events,
                sim_time=sim_time,
                event_epoch_s=target_event_epoch_s,
            )
            if target_application is not None:
                event_trace.append(_event_trace(target_application, sim_time, "TARGET3"))
            ####
            for module in definition.module_order:
                if module == "environment":
                    _target_environment(target.flat)
                elif module == "guidance":
                    _target_guidance(target)
                elif module == "control":
                    _target_control(target, dt_s)
                elif module == "forces":
                    _target_forces(target)
                elif module == "newton":
                    _target_newton(target.flat, dt_s)
                ####
            ####
            target.flat.time_s = sim_time
            target_bus = _packet_for_target(target, alive=target_alive)
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end or not missile_alive:
            samples.append(_sample(sim_time, runtime))
            target_samples.append(_target_sample(sim_time, target, alive=target_alive))
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        if not _runtime_is_finite(runtime) or not _target_is_finite(target):
            terminated_reason = "nonfinite_state"
            if samples[-1].time_s != sim_time:
                samples.append(_sample(sim_time, runtime))
                target_samples.append(_target_sample(sim_time, target, alive=target_alive))
            ####
            break
        ####
        if not missile_alive:
            break
        ####
        sim_time += dt_s
    ####
    return Sraam6RunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
        target_samples=tuple(target_samples),
        intercept=intercept,
        event_trace=tuple(event_trace),
    )


####


def _initialize_runtime(definition: Sraam6SourceDefinition, initial: Sraam6InitialState) -> _Sraam6Runtime:
    quaternion = np.asarray(sraam6_initial_quaternion(initial), dtype=np.float64)
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
    propulsion = Sraam6PropulsionStep(
        mode=definition.propulsion.mode,
        thrust_n=0.0,
        mass_kg=definition.airframe.launch_mass_kg,
        center_of_gravity_m=definition.airframe.reference_cg_m,
        roll_inertia_kg_m2=definition.airframe.launch_roll_inertia_kg_m2,
        pitch_inertia_kg_m2=definition.airframe.launch_pitch_inertia_kg_m2,
    )
    zero_control = Sraam6ControlCommand()
    zero_actuator = Sraam6ActuatorStep(
        requested_control=zero_control,
        requested_fins=Sraam6FinSet(),
        achieved_fins=Sraam6FinSet(),
        achieved_control=zero_control,
        state=Sraam6FinActuatorState(),
        position_limited=(False, False, False, False),
        rate_limited=(False, False, False, False),
    )
    zero_coefficients = Sraam6AeroCoefficients(
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
    position = np.asarray(initial.position_ned_m, dtype=np.float64)
    return _Sraam6Runtime(
        position_ned_m=position,
        position_derivative_ned_mps=np.zeros(3, dtype=np.float64),
        velocity_body_mps=body_velocity,
        velocity_body_derivative_mps2=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=local_velocity,
        quaternion_wxyz=quaternion,
        quaternion_derivative=np.zeros(4, dtype=np.float64),
        body_rates_rad_s=np.asarray(initial.body_rates_deg_s, dtype=np.float64) * _RAD_PER_DEG,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        fin_state=Sraam6FinActuatorState(),
        requested_control=zero_control,
        actuator_step=zero_actuator,
        propulsion=propulsion,
        coefficients=zero_coefficients,
        wrench=Sraam6BodyWrench(force_n=(0.0, 0.0, 0.0), moment_nm=(0.0, 0.0, 0.0)),
        control_mode=definition.control.initial_mode,
        guidance_mode=definition.guidance.initial_mode,
        navigation_update_mode=definition.guidance.navigation_update_mode,
        propulsion_mode=definition.propulsion.mode,
        seeker_mode=definition.seeker.initial_mode,
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        altitude_m=-float(position[2]),
        previous_missile_position_ned_m=position.copy(),
    )


####


def _missile_environment(runtime: _Sraam6Runtime) -> None:
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    density, pressure, temperature = atmosphere76(runtime.altitude_m)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.speed_of_sound_mps = math.sqrt(1.4 * _R_AIR * temperature)
    speed = float(np.linalg.norm(runtime.velocity_ned_mps))
    runtime.mach = abs(speed / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * density * speed * speed


####


def _missile_kinematics(runtime: _Sraam6Runtime, dt_s: float) -> None:
    q0, q1, q2, q3 = runtime.quaternion_wxyz
    p_rate, q_rate, r_rate = runtime.body_rates_rad_s
    error = 1.0 - float(runtime.quaternion_wxyz @ runtime.quaternion_wxyz)
    correction = 50.0
    derivative_new = np.asarray(
        (
            0.5 * (-p_rate * q1 - q_rate * q2 - r_rate * q3) + correction * error * q0,
            0.5 * (p_rate * q0 + r_rate * q2 - q_rate * q3) + correction * error * q1,
            0.5 * (q_rate * q0 - r_rate * q1 + p_rate * q3) + correction * error * q2,
            0.5 * (r_rate * q0 + q_rate * q1 - p_rate * q2) + correction * error * q3,
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
    body_velocity = runtime.velocity_body_mps
    speed = float(np.linalg.norm(body_velocity))
    if speed <= _SMALL:
        runtime.alpha_deg = 0.0
        runtime.beta_deg = 0.0
        runtime.total_alpha_deg = 0.0
        runtime.aerodynamic_roll_rad = 0.0
        runtime.velocity_ned_mps = transform.T @ body_velocity
        return
    ####
    body_x, body_y, body_z = (float(value) for value in body_velocity)
    alpha = math.atan2(body_z, body_x)
    beta = math.asin(min(1.0, max(-1.0, body_y / speed)))
    total_argument = min(1.0, max(-1.0, body_x / speed))
    total_alpha = math.acos(total_argument)
    if abs(body_y) < _SMALL:
        aerodynamic_roll = 0.0 if body_z >= 0.0 else math.pi
    else:
        aerodynamic_roll = math.atan2(body_y, body_z)
    ####
    runtime.alpha_deg = alpha * _DEG_PER_RAD
    runtime.beta_deg = beta * _DEG_PER_RAD
    runtime.total_alpha_deg = total_alpha * _DEG_PER_RAD
    runtime.aerodynamic_roll_rad = aerodynamic_roll
    runtime.velocity_ned_mps = transform.T @ body_velocity


####


def _missile_seeker(
    runtime: _Sraam6Runtime,
    config: Sraam6SeekerConfig,
    target_bus: object,
    time_s: float,
    dt_s: float,
) -> None:
    target_position = np.asarray(getattr(target_bus, "position_ned_m"), dtype=np.float64)
    target_velocity = np.asarray(getattr(target_bus, "velocity_ned_mps"), dtype=np.float64)
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    sensor = cadac_local_ned_relative_state_track(
        time_s=time_s,
        host_position_ned_m=runtime.position_ned_m,
        host_velocity_ned_mps=runtime.velocity_ned_mps,
        target_id="sraam6-target",
        target_position_ned_m=target_position,
        target_velocity_ned_mps=target_velocity,
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
    closing_signed = sensor.closing_speed_mps
    los_rate_body = np.asarray(sensor.line_of_sight_rate_sensor_rad_s, dtype=np.float64)
    body_x, body_y, body_z = (float(value) for value in unit_body)
    pitch_true = math.atan2(-body_z, math.hypot(body_x, body_y))
    yaw_true = math.atan2(body_y, body_x)
    pointing_transform = mat2tr(yaw_true, pitch_true)
    los_rate_pointing = pointing_transform @ los_rate_body
    runtime.closing_speed_mps = closing_signed
    runtime.unit_los_local = unit_local
    runtime.unit_los_body = unit_body
    runtime.true_los_rate_body_rad_s = los_rate_body

    if runtime.seeker_mode == 2 and distance < config.acquisition_range_m:
        runtime.seeker_mode = 3
        runtime.seeker_initialized = False
    ####
    if runtime.seeker_mode == 3:
        if not runtime.seeker_initialized:
            runtime.seeker_pointing_pitch_rad = pitch_true
            runtime.seeker_pointing_yaw_rad = yaw_true
            runtime.seeker_pointing_pitch_derivative_rad_s = 0.0
            runtime.seeker_pointing_yaw_derivative_rad_s = 0.0
            runtime.seeker_pitch = _SeekerAxisState()
            runtime.seeker_yaw = _SeekerAxisState()
            runtime.seeker_acquisition_epoch_s = time_s
            runtime.seeker_initialized = True
        ####
        if config.dynamic_mode == 1:
            pitch_error, yaw_error = _update_dynamic_seeker(
                runtime,
                config,
                pitch_true_rad=pitch_true,
                yaw_true_rad=yaw_true,
                dt_s=dt_s,
            )
        else:
            runtime.seeker_pointing_pitch_rad = pitch_true
            runtime.seeker_pointing_yaw_rad = yaw_true
            runtime.seeker_pitch.rate_rad_s = float(los_rate_pointing[1])
            runtime.seeker_yaw.rate_rad_s = float(los_rate_pointing[2])
            pitch_error = 0.0
            yaw_error = 0.0
        ####
        if abs(yaw_error) <= config.yaw_half_fov_rad and abs(pitch_error) <= config.pitch_half_fov_rad:
            if time_s - runtime.seeker_acquisition_epoch_s > config.acquisition_time_s:
                runtime.seeker_mode = 4
            ####
        else:
            runtime.seeker_acquisition_epoch_s = time_s
        ####
    elif runtime.seeker_mode == 4:
        runtime.guidance_mode = 6
        if config.dynamic_mode == 1:
            _update_dynamic_seeker(
                runtime,
                config,
                pitch_true_rad=pitch_true,
                yaw_true_rad=yaw_true,
                dt_s=dt_s,
            )
        else:
            runtime.seeker_pointing_pitch_rad = pitch_true
            runtime.seeker_pointing_yaw_rad = yaw_true
            runtime.seeker_pitch.rate_rad_s = float(los_rate_pointing[1])
            runtime.seeker_yaw.rate_rad_s = float(los_rate_pointing[2])
        ####
        if distance <= config.blind_range_m:
            runtime.seeker_mode = 5
        ####
    ####


####


def _update_dynamic_seeker(
    runtime: _Sraam6Runtime,
    config: Sraam6SeekerConfig,
    *,
    pitch_true_rad: float,
    yaw_true_rad: float,
    dt_s: float,
) -> tuple[float, float]:
    """Advance the source filter/look-angle states using simplified pointing errors.

    The source optical-error, aimpoint, and gimbal-head construction is not yet
    present.  The true LOS-to-current-pointing displacement is therefore the
    filter error while the source stored-derivative state equations are retained.
    """

    pitch_error = _wrap_pi(pitch_true_rad - runtime.seeker_pointing_pitch_rad)
    yaw_error = _wrap_pi(yaw_true_rad - runtime.seeker_pointing_yaw_rad)
    _update_seeker_axis(runtime.seeker_pitch, pitch_error, config, dt_s)
    _update_seeker_axis(runtime.seeker_yaw, yaw_error, config, dt_s)

    pointing_transform = mat2tr(
        runtime.seeker_pointing_yaw_rad,
        runtime.seeker_pointing_pitch_rad,
    )
    body_rate_pointing = pointing_transform @ runtime.body_rates_rad_s
    pitch_derivative_new = runtime.seeker_pitch.rate_rad_s - float(body_rate_pointing[1])
    runtime.seeker_pointing_pitch_rad = _wrap_pi(
        _integrate_scalar(
            runtime.seeker_pointing_pitch_rad,
            pitch_derivative_new,
            runtime.seeker_pointing_pitch_derivative_rad_s,
            dt_s,
        )
    )
    runtime.seeker_pointing_pitch_derivative_rad_s = pitch_derivative_new
    yaw_derivative_new = runtime.seeker_yaw.rate_rad_s - float(body_rate_pointing[2])
    runtime.seeker_pointing_yaw_rad = _wrap_pi(
        _integrate_scalar(
            runtime.seeker_pointing_yaw_rad,
            yaw_derivative_new,
            runtime.seeker_pointing_yaw_derivative_rad_s,
            dt_s,
        )
    )
    runtime.seeker_pointing_yaw_derivative_rad_s = yaw_derivative_new
    return (pitch_error, yaw_error)


####


def _update_seeker_axis(
    state: _SeekerAxisState,
    pointing_error_rad: float,
    config: Sraam6SeekerConfig,
    dt_s: float,
) -> None:
    wn = config.filter_natural_frequency_rad_s
    damping = config.filter_damping_ratio
    rate_derivative_new = state.acceleration_rad_s2
    state.rate_rad_s = _integrate_scalar(
        state.rate_rad_s,
        rate_derivative_new,
        state.rate_derivative_rad_s2,
        dt_s,
    )
    state.rate_derivative_rad_s2 = rate_derivative_new
    acceleration_derivative_new = (
        config.filter_gain_per_s * wn * wn * pointing_error_rad - 2.0 * damping * wn * state.rate_derivative_rad_s2 - wn * wn * state.rate_rad_s
    )
    state.acceleration_rad_s2 = _integrate_scalar(
        state.acceleration_rad_s2,
        acceleration_derivative_new,
        state.acceleration_derivative_rad_s3,
        dt_s,
    )
    state.acceleration_derivative_rad_s3 = acceleration_derivative_new


####


def _missile_guidance(
    runtime: _Sraam6Runtime,
    config: Sraam6GuidanceConfig,
    target_bus: object,
    time_s: float,
) -> None:
    target_position = np.asarray(getattr(target_bus, "position_ned_m"), dtype=np.float64)
    target_velocity = np.asarray(getattr(target_bus, "velocity_ned_mps"), dtype=np.float64)
    if runtime.navigation_update_mode == 3:
        runtime.navigation_update_mode = 0
        runtime.target_update_epoch_s = time_s
        runtime.target_position_stored_ned_m = target_position.copy()
        runtime.target_velocity_stored_ned_mps = target_velocity.copy()
    ####
    if runtime.guidance_mode == 0:
        runtime.normal_command_g = 0.0
        runtime.lateral_command_g = 0.0
        return
    ####
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    if runtime.guidance_mode == 3:
        extrapolated = runtime.target_position_stored_ned_m + runtime.target_velocity_stored_ned_mps * (time_s - runtime.target_update_epoch_s)
        relative = extrapolated - runtime.position_ned_m
        distance = float(np.linalg.norm(relative))
        if distance <= _SMALL:
            runtime.normal_command_g = 0.0
            runtime.lateral_command_g = 0.0
            return
        ####
        unit_local = relative / distance
        relative_velocity = runtime.target_velocity_stored_ned_mps - runtime.velocity_ned_mps
        closing = abs(float(unit_local @ relative_velocity))
        los_rate_local = np.cross(unit_local, relative_velocity) / distance
        acceleration_body = transform @ np.cross(los_rate_local, unit_local) * config.navigation_gain * closing
        runtime.normal_command_g = -float(acceleration_body[2]) / max(runtime.gravity_mps2, _SMALL)
        runtime.lateral_command_g = float(acceleration_body[1]) / max(runtime.gravity_mps2, _SMALL)
    elif runtime.guidance_mode == 6:
        closing = max(0.0, runtime.closing_speed_mps)
        yaw_angle = runtime.seeker_pointing_yaw_rad
        pitch_angle = runtime.seeker_pointing_pitch_rad
        pitch_los_rate = runtime.seeker_pitch.rate_rad_s
        yaw_los_rate = runtime.seeker_yaw.rate_rad_s
        longitudinal_specific_force = runtime.wrench.force_n[0] / runtime.propulsion.mass_kg
        lateral_compensation = longitudinal_specific_force * math.tan(yaw_angle) / max(runtime.gravity_mps2, _SMALL)
        pitch_compensation = longitudinal_specific_force * math.tan(pitch_angle) / max(_SMALL, math.cos(yaw_angle) * runtime.gravity_mps2)
        gravity_body = transform @ np.asarray((0.0, 0.0, 1.0), dtype=np.float64)
        gain = config.navigation_gain * closing
        lateral_pn = gain * yaw_los_rate / max(_SMALL, math.cos(yaw_angle) * runtime.gravity_mps2)
        normal_pn = (
            gain
            * (yaw_los_rate * math.tan(pitch_angle) * math.tan(yaw_angle) + pitch_los_rate / max(_SMALL, math.cos(pitch_angle)))
            / max(runtime.gravity_mps2, _SMALL)
        )
        lateral = lateral_pn + lateral_compensation - float(gravity_body[1])
        normal = normal_pn + pitch_compensation + float(gravity_body[2])
        maximum = runtime.coefficients.max_acceleration_g
        magnitude = math.hypot(lateral, normal)
        if maximum > 0.0 and magnitude > maximum:
            scale = maximum / magnitude
            lateral *= scale
            normal *= scale
        ####
        runtime.lateral_command_g = lateral
        runtime.normal_command_g = normal
    ####


####


def _missile_control(
    runtime: _Sraam6Runtime,
    config: Sraam6ControlConfig,
    dt_s: float,
) -> Sraam6ControlCommand:
    if runtime.control_mode == 0:
        return Sraam6ControlCommand()
    ####
    roll = _control_roll(runtime, config)
    pitch = 0.0
    yaw = 0.0
    if runtime.control_mode == 2:
        pitch, yaw = _control_rate(runtime, config)
    elif runtime.control_mode == 3:
        pitch, yaw = _control_acceleration(runtime, config, dt_s)
    ####
    return Sraam6ControlCommand(
        roll_deg=_clip(roll, config.roll_command_limit_deg),
        pitch_deg=_clip(pitch, config.pitch_command_limit_deg),
        yaw_deg=_clip(yaw, config.yaw_command_limit_deg),
    )


####


def _control_roll(runtime: _Sraam6Runtime, config: Sraam6ControlConfig) -> float:
    dlp = runtime.coefficients.roll_rate_derivative_per_s
    dld = _nonzero(runtime.coefficients.roll_control_derivative_rad_s2)
    gkp = (2.0 * config.roll_damping_ratio * config.roll_natural_frequency_rad_s + dlp) / dld
    gkphi = config.roll_natural_frequency_rad_s**2 / dld
    roll_angle_deg = _euler_from_dcm(_dcm_body_from_local(runtime.quaternion_wxyz))[2] * _DEG_PER_RAD
    error = gkphi * (config.commanded_roll_deg - roll_angle_deg) * _RAD_PER_DEG
    return (error - gkp * float(runtime.body_rates_rad_s[0])) * _DEG_PER_RAD


####


def _control_rate(runtime: _Sraam6Runtime, config: Sraam6ControlConfig) -> tuple[float, float]:
    speed = max(_SMALL, float(np.linalg.norm(runtime.velocity_ned_mps)))
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
    pitch_rate = _clip(float(runtime.body_rates_rad_s[1]) * _DEG_PER_RAD, config.rate_command_limit_deg_s)
    yaw_rate = _clip(float(runtime.body_rates_rad_s[2]) * _DEG_PER_RAD, config.rate_command_limit_deg_s)
    return (gain * pitch_rate, gain * yaw_rate)


####


def _control_acceleration(
    runtime: _Sraam6Runtime,
    config: Sraam6ControlConfig,
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
    dynamic_pressure_root = math.sqrt(max(0.0, runtime.dynamic_pressure_pa))
    wacl = (0.013 * dynamic_pressure_root + 7.1) * (1.0 + config.acceleration_frequency_factor)
    zacl = (0.559e-3 * dynamic_pressure_root + 0.232) * (1.0 + config.acceleration_damping_factor)
    pacl = 14.0
    dna = _nonzero(runtime.coefficients.normal_alpha_derivative_mps2)
    dma = runtime.coefficients.pitch_alpha_derivative_rad_s2
    dmq = runtime.coefficients.pitch_rate_derivative_per_s
    dmd = _nonzero(runtime.coefficients.pitch_control_derivative_rad_s2)
    speed = max(_SMALL, float(np.linalg.norm(runtime.velocity_ned_mps)))
    gainfb3 = wacl * wacl * pacl / (dna * dmd)
    gainfb2 = (2.0 * zacl * wacl + pacl + dmq - dna / speed) / dmd
    gainfb1 = (wacl * wacl + 2.0 * zacl * wacl * pacl + dma + dmq * dna / speed - gainfb2 * dmd * dna / speed) / (
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
    pitch = (-gainfb1 * (-pitch_force) - gainfb2 * float(runtime.body_rates_rad_s[1]) + gainfb3 * runtime.pitch_feedforward_state) * _DEG_PER_RAD
    yaw_force = float(force_body[1])
    yaw_derivative_new = _AGRAV * lateral - yaw_force
    runtime.yaw_feedforward_state = _integrate_scalar(
        runtime.yaw_feedforward_state,
        yaw_derivative_new,
        runtime.yaw_feedforward_derivative,
        dt_s,
    )
    runtime.yaw_feedforward_derivative = yaw_derivative_new
    yaw = (-gainfb1 * yaw_force - gainfb2 * float(runtime.body_rates_rad_s[2]) + gainfb3 * runtime.yaw_feedforward_state) * _DEG_PER_RAD
    return (pitch, yaw)


####


def _missile_euler(runtime: _Sraam6Runtime, dt_s: float) -> None:
    p_rate, q_rate, r_rate = (float(value) for value in runtime.body_rates_rad_s)
    roll_inertia = runtime.propulsion.roll_inertia_kg_m2
    pitch_inertia = runtime.propulsion.pitch_inertia_kg_m2
    moment_roll, moment_pitch, moment_yaw = runtime.wrench.moment_nm
    derivative_new = np.asarray(
        (
            moment_roll / roll_inertia,
            ((pitch_inertia - roll_inertia) * p_rate * r_rate + moment_pitch) / pitch_inertia,
            (-(pitch_inertia - roll_inertia) * p_rate * q_rate + moment_yaw) / pitch_inertia,
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


def _missile_newton(runtime: _Sraam6Runtime, dt_s: float) -> None:
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    force_body = np.asarray(runtime.wrench.force_n, dtype=np.float64)
    specific_force = force_body / runtime.propulsion.mass_kg
    tangent = np.cross(runtime.body_rates_rad_s, runtime.velocity_body_mps)
    gravity_local = np.asarray((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    derivative_new = specific_force - tangent + transform @ gravity_local
    runtime.velocity_body_mps = _integrate_vector(
        runtime.velocity_body_mps,
        derivative_new,
        runtime.velocity_body_derivative_mps2,
        dt_s,
    )
    runtime.velocity_body_derivative_mps2 = derivative_new
    runtime.velocity_ned_mps = transform.T @ runtime.velocity_body_mps
    position_derivative_new = runtime.velocity_ned_mps
    runtime.position_ned_m = _integrate_vector(
        runtime.position_ned_m,
        position_derivative_new,
        runtime.position_derivative_ned_mps,
        dt_s,
    )
    runtime.position_derivative_ned_mps = position_derivative_new
    runtime.altitude_m = -float(runtime.position_ned_m[2])
    runtime.lateral_acceleration_g = float(specific_force[1]) / max(runtime.gravity_mps2, _SMALL)
    runtime.normal_acceleration_g = -float(specific_force[2]) / max(runtime.gravity_mps2, _SMALL)


####


def _missile_intercept(
    runtime: _Sraam6Runtime,
    target_bus: object,
    time_s: float,
    dt_s: float,
) -> Sraam6Intercept | None:
    target_position = np.asarray(getattr(target_bus, "position_ned_m"), dtype=np.float64)
    target_velocity = np.asarray(getattr(target_bus, "velocity_ned_mps"), dtype=np.float64)
    relative = target_position - runtime.position_ned_m
    distance = float(np.linalg.norm(relative))
    relative_velocity = target_velocity - runtime.velocity_ned_mps
    if distance < 100.0:
        runtime.entered_intercept_sphere = True
    ####
    if runtime.entered_intercept_sphere and runtime.previous_time_s < time_s:
        previous_relative = runtime.previous_target_position_ned_m - runtime.previous_missile_position_ned_m
        previous_distance = float(np.linalg.norm(previous_relative))
        if distance >= previous_distance and previous_distance < 100.0:
            delta_relative = relative - previous_relative
            denominator = float(delta_relative @ delta_relative)
            fraction = 0.0
            if denominator > _SMALL:
                fraction = min(1.0, max(0.0, -float(previous_relative @ delta_relative) / denominator))
            ####
            miss_vector = previous_relative + fraction * delta_relative
            hit_time = runtime.previous_time_s + fraction * dt_s
            return Sraam6Intercept(
                time_s=max(0.0, hit_time),
                miss_distance_m=float(np.linalg.norm(miss_vector)),
                miss_vector_ned_m=_tuple3(miss_vector),
                differential_speed_mps=float(np.linalg.norm(relative_velocity)),
            )
        ####
    ####
    runtime.previous_relative_position_ned_m = relative.copy()
    runtime.previous_missile_position_ned_m = runtime.position_ned_m.copy()
    runtime.previous_target_position_ned_m = target_position.copy()
    runtime.previous_time_s = time_s
    return None


####


def _apply_missile_event(
    runtime: _Sraam6Runtime,
    cursor: CadacEventCursor,
    sim_time: float,
) -> CadacEventApplication | None:
    values: dict[str, int | float] = {
        "time": sim_time,
        "event_time": sim_time - runtime.event_epoch_s,
        "maut": runtime.control_mode,
        "mguid": runtime.guidance_mode,
        "mnav": runtime.navigation_update_mode,
        "mseek": runtime.seeker_mode,
        "mprop": runtime.propulsion_mode,
        "thrust": runtime.propulsion.thrust_n,
    }
    try:
        application = cursor.evaluate_and_apply(values)
    except (KeyError, TypeError) as error:
        raise Sraam6SourceError(f"MISSILE6 event cannot be bound to the executable runtime: {error}") from error
    ####
    if application is None:
        return None
    ####
    mutable = {"maut", "mguid", "mnav", "mseek", "mprop"}
    unsupported = tuple(name for name, _ in application.updated_values if name.casefold() not in mutable)
    if unsupported:
        raise Sraam6SourceError(f"MISSILE6 event assignments are readable but not mutable in this executable slice: {unsupported!r}")
    ####
    previous_seeker_mode = runtime.seeker_mode
    runtime.control_mode = int(values["maut"])
    runtime.guidance_mode = int(values["mguid"])
    runtime.navigation_update_mode = int(values["mnav"])
    runtime.seeker_mode = int(values["mseek"])
    runtime.propulsion_mode = int(values["mprop"])
    if runtime.seeker_mode != previous_seeker_mode:
        runtime.seeker_initialized = False
    ####
    runtime.event_epoch_s = sim_time
    runtime.event_time_s = 0.0
    return application


####


def _apply_target_event(
    target: _AircraftState,
    cursor: CadacEventCursor,
    *,
    sim_time: float,
    event_epoch_s: float,
) -> tuple[CadacEventApplication | None, float]:
    values: dict[str, int | float] = {
        "time": sim_time,
        "event_time": sim_time - event_epoch_s,
        "tgt_option": target.config.aircraft_option,
        "guid_gain": target.config.guidance_gain,
        "gturn": target.config.turn_g,
    }
    try:
        application = cursor.evaluate_and_apply(values)
    except (KeyError, TypeError) as error:
        raise Sraam6SourceError(f"TARGET3 event cannot be bound to the executable runtime: {error}") from error
    ####
    if application is None:
        return (None, event_epoch_s)
    ####
    mutable = {"tgt_option", "guid_gain", "gturn"}
    unsupported = tuple(name for name, _ in application.updated_values if name.casefold() not in mutable)
    if unsupported:
        raise Sraam6SourceError(f"TARGET3 event assignments are readable but not mutable in this executable slice: {unsupported!r}")
    ####
    target.config = target.config.model_copy(
        update={
            "aircraft_option": int(values["tgt_option"]),
            "turn_g": float(values["gturn"]),
            "guidance_gain": float(values["guid_gain"]),
        }
    )
    return (application, sim_time)


####


def _event_trace(
    application: CadacEventApplication,
    time_s: float,
    actor: Literal["MISSILE6", "TARGET3"],
) -> Sraam6EventTrace:
    return Sraam6EventTrace(
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


def _sample(time_s: float, runtime: _Sraam6Runtime) -> Sraam6Sample:
    return Sraam6Sample(
        time_s=max(0.0, time_s),
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        speed_mps=float(np.linalg.norm(runtime.velocity_ned_mps)),
        quaternion_wxyz=_tuple4(runtime.quaternion_wxyz),
        body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
        altitude_m=runtime.altitude_m,
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
        center_of_gravity_m=runtime.propulsion.center_of_gravity_m,
        roll_inertia_kg_m2=runtime.propulsion.roll_inertia_kg_m2,
        pitch_inertia_kg_m2=runtime.propulsion.pitch_inertia_kg_m2,
        thrust_n=max(0.0, runtime.propulsion.thrust_n),
        propulsion_mode=runtime.propulsion_mode,
        seeker_mode=runtime.seeker_mode,
        guidance_mode=runtime.guidance_mode,
        autopilot_mode=runtime.control_mode,
        target_range_m=max(0.0, runtime.target_range_m),
        closing_speed_mps=runtime.closing_speed_mps,
        seeker_pointing_pitch_rad=runtime.seeker_pointing_pitch_rad,
        seeker_pointing_yaw_rad=runtime.seeker_pointing_yaw_rad,
        seeker_los_rate_pitch_rad_s=runtime.seeker_pitch.rate_rad_s,
        seeker_los_rate_yaw_rad_s=runtime.seeker_yaw.rate_rad_s,
        normal_command_g=runtime.normal_command_g,
        lateral_command_g=runtime.lateral_command_g,
        normal_acceleration_g=runtime.normal_acceleration_g,
        lateral_acceleration_g=runtime.lateral_acceleration_g,
    )


####


def _target_sample(time_s: float, target: _AircraftState, *, alive: bool) -> Sraam6TargetSample:
    return Sraam6TargetSample(
        time_s=max(0.0, time_s),
        position_ned_m=_tuple3(target.flat.position_ned_m),
        velocity_ned_mps=_tuple3(target.flat.velocity_ned_mps),
        speed_mps=target.flat.speed_mps,
        heading_deg=target.flat.heading_rad * _DEG_PER_RAD,
        flight_path_deg=target.flat.flight_path_rad * _DEG_PER_RAD,
        altitude_m=target.flat.altitude_m,
        bank_deg=target.flat.bank_rad * _DEG_PER_RAD,
        normal_load_g=target.normal_load_g,
        alive=alive,
    )


####


def _runtime_is_finite(runtime: _Sraam6Runtime) -> bool:
    arrays = (
        runtime.position_ned_m,
        runtime.velocity_body_mps,
        runtime.velocity_ned_mps,
        runtime.quaternion_wxyz,
        runtime.body_rates_rad_s,
    )
    scalars = (
        runtime.altitude_m,
        runtime.mach,
        runtime.dynamic_pressure_pa,
        runtime.propulsion.mass_kg,
        runtime.propulsion.thrust_n,
    )
    return all(bool(np.all(np.isfinite(array))) for array in arrays) and all(math.isfinite(value) for value in scalars)


####


def _target_is_finite(target: _AircraftState) -> bool:
    return bool(np.all(np.isfinite(target.flat.position_ned_m)) and np.all(np.isfinite(target.flat.velocity_ned_mps)) and math.isfinite(target.flat.speed_mps))


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
    pitch = math.asin(min(1.0, max(-1.0, -float(transform[0, 2]))))
    cosine_pitch = max(_SMALL, math.cos(pitch))
    yaw = math.atan2(float(transform[0, 1]) / cosine_pitch, float(transform[0, 0]) / cosine_pitch)
    roll = math.atan2(float(transform[1, 2]) / cosine_pitch, float(transform[2, 2]) / cosine_pitch)
    return (yaw, pitch, roll)


####


def _integrate_scalar(state: float, derivative_new: float, derivative_previous: float, dt_s: float) -> float:
    return state + (derivative_new + derivative_previous) * dt_s / 2.0


####


def _integrate_vector(
    state: FloatVector,
    derivative_new: FloatVector,
    derivative_previous: FloatVector,
    dt_s: float,
) -> FloatVector:
    return state + (derivative_new + derivative_previous) * dt_s / 2.0


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Sraam6SourceError(f"{vehicle.model_name} is missing required parameter {name!r}") from None
        ####
        return default
    ####
    if not isinstance(value, (int, float)):
        raise Sraam6SourceError(f"{vehicle.model_name} parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Sraam6SourceError(f"{vehicle.model_name} parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, float(default) if default is not None else None)
    if not value.is_integer():
        raise Sraam6SourceError(f"{vehicle.model_name} parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


####


def _clip(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))


####


def _nonzero(value: float) -> float:
    if abs(value) >= _SMALL:
        return value
    ####
    return math.copysign(_SMALL, value if value != 0.0 else 1.0)


####


def _tuple3(values: NDArray[np.float64] | tuple[float, float, float]) -> tuple[float, float, float]:
    array = np.asarray(values, dtype=np.float64)
    return (float(array[0]), float(array[1]), float(array[2]))


####


def _tuple4(
    values: NDArray[np.float64] | tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    array = np.asarray(values, dtype=np.float64)
    return (float(array[0]), float(array[1]), float(array[2]), float(array[3]))


####


def _fin_set(values: NDArray[np.float64]) -> Sraam6FinSet:
    return Sraam6FinSet(
        fin1_deg=float(values[0]),
        fin2_deg=float(values[1]),
        fin3_deg=float(values[2]),
        fin4_deg=float(values[3]),
    )


####


__all__ = [
    "Sraam6ActuatorConfig",
    "Sraam6ActuatorStep",
    "Sraam6AeroCoefficients",
    "Sraam6AeroLimits",
    "Sraam6Airframe",
    "Sraam6BodyWrench",
    "Sraam6ControlCommand",
    "Sraam6ControlConfig",
    "Sraam6EventTrace",
    "Sraam6FinActuatorState",
    "Sraam6FinSet",
    "Sraam6GuidanceConfig",
    "Sraam6InitialState",
    "Sraam6Intercept",
    "Sraam6PropulsionConfig",
    "Sraam6PropulsionStep",
    "Sraam6RunResult",
    "Sraam6Sample",
    "Sraam6ScenarioSession",
    "Sraam6SeekerConfig",
    "Sraam6SourceDefinition",
    "Sraam6SourceError",
    "Sraam6TargetSample",
    "load_sraam6_source_definition",
    "lower_sraam6_source_bundle",
    "run_sraam6_source_compatibility",
    "sraam6_actuator_step",
    "sraam6_aerodynamic_coefficients",
    "sraam6_body_wrench",
    "sraam6_initial_quaternion",
    "sraam6_mix_fin_commands",
    "sraam6_propulsion_step",
    "sraam6_unmix_fin_positions",
]
