"""Source-shaped ADS6 SAM INS, seeker, guidance, and autopilot chain."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal, TypeAlias, cast

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .ads6_sam import (
    Ads6SamControlCommand,
    Ads6SamDirectCommand,
    Ads6SamPhase,
    Ads6SamPlantObservation,
    Ads6SamSourceDefinition,
)
from .bundle import CadacSourceBundle
from .compatibility import cadac_stored_derivative_step
from .events import CadacEventApplication, CadacEventCursor, CadacRuntimeScalar
from .input_ast import CadacEventBlock, CadacModel, CadacVehicleBlock
from .sensor_adapter import cadac_local_ned_relative_state_track

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]
Ads6SamTargetKind = Literal["aircraft", "srbm"]
Ads6SamSensorKind = Literal["off", "rf", "ir"]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_AGRAV = 9.80675445
_SMALL = 1.0e-9
_SOURCE_CONTROLLER_MODULES = ("ins", "sensor", "guidance", "control")


class Ads6SamControllerSourceError(ValueError):
    """Source actor cannot be lowered into the ADS6 controller chain."""


####


class Ads6SamInsConfig(CadacModel):
    """Requested source INS mode and current reconstruction boundary."""

    mode: int = Field(default=0, ge=0, le=3)
    implementation: Literal["truth_aligned"] = "truth_aligned"


####


class Ads6SamSensorConfig(CadacModel):
    """Common RF/IR seeker modes and deterministic participating parameters."""

    initial_mode: int = Field(default=0, ge=0, le=29)
    dynamic_mode: int = Field(default=0, ge=0, le=1)
    target_kind: Ads6SamTargetKind = "aircraft"
    blind_range_m: float = Field(default=0.0, ge=0.0)
    rf_acquisition_range_m: float = Field(default=0.0, ge=0.0)
    rf_acquisition_time_s: float = Field(default=0.0, ge=0.0)
    rf_field_of_regard_deg: float = Field(default=180.0, ge=0.0, le=180.0)
    rf_field_of_view_deg: float = Field(default=180.0, ge=0.0, le=180.0)
    rf_tracking_gain_per_s: float = Field(default=0.0, ge=0.0)
    ir_acquisition_range_m: float = Field(default=0.0, ge=0.0)
    ir_acquisition_time_s: float = Field(default=0.0, ge=0.0)
    ir_yaw_half_fov_rad: float = Field(default=math.pi, ge=0.0)
    ir_pitch_half_fov_rad: float = Field(default=math.pi, ge=0.0)
    ir_filter_gain_per_s: float = Field(default=0.0, ge=0.0)
    ir_filter_natural_frequency_rad_s: float = Field(default=0.0, ge=0.0)
    ir_filter_damping_ratio: float = Field(default=0.0, ge=0.0)
    maximum_pitch_gimbal_rad: float = Field(default=math.pi, ge=0.0)
    maximum_pitch_rate_rad_s: float = Field(default=math.inf, ge=0.0)
    maximum_roll_rate_rad_s: float = Field(default=math.inf, ge=0.0)
    maximum_tracking_rate_rad_s: float = Field(default=math.inf, ge=0.0)

    @model_validator(mode="after")
    def validate_mode_encoding(self) -> "Ads6SamSensorConfig":
        sensor_type = self.initial_mode // 10
        sensor_mode = self.initial_mode % 10
        if self.initial_mode != 0 and sensor_type not in {1, 2}:
            raise ValueError("ADS6 SAM sensor type must be RF=1 or IR=2")
        ####
        if self.initial_mode != 0 and sensor_mode not in {1, 2, 3, 4, 5}:
            raise ValueError("ADS6 SAM sensor mode must be 1 through 5")
        ####
        return self

    ####


####


class Ads6SamGuidanceConfig(CadacModel):
    """Source line-guidance and terminal proportional-navigation parameters."""

    initial_mode: int = Field(default=0, ge=0, le=99)
    gravity_bias_g: float = 1.0
    navigation_gain: float = Field(default=0.0, ge=0.0)
    navigation_gain_state: float = Field(default=0.0, ge=0.0)
    navigation_gain_time_constant_s: float = Field(default=0.0, ge=0.0)
    line_gain_per_s: float = Field(default=0.0, ge=0.0)
    nonlinear_gain_factor: float = Field(default=0.0, ge=0.0)
    distance_decrement_m: float = Field(default=1.0, gt=0.0)
    vertical_line_of_attack_deg: float = 0.0


####


class Ads6SamAutopilotConfig(CadacModel):
    """Source rate/acceleration controller parameters and adaptive biases."""

    initial_mode: int = Field(default=0, ge=0, le=4)
    structural_limit_g: float = Field(default=50.0, gt=0.0)
    roll_command_limit_deg: float = Field(default=28.0, gt=0.0)
    pitch_command_limit_deg: float = Field(default=28.0, gt=0.0)
    yaw_command_limit_deg: float = Field(default=28.0, gt=0.0)
    commanded_roll_deg: float = 0.0
    roll_damping_ratio: float = Field(default=0.9, ge=0.0)
    roll_rate_time_constant_s: float = Field(default=0.1, gt=0.0)
    roll_bandwidth_factor: float = Field(default=0.0, gt=-1.0)
    rate_loop_damping_ratio: float = Field(default=1.2, ge=0.0)
    acceleration_feedforward_gain_s2_m: float = 0.0
    acceleration_frequency_bias: float = Field(default=0.0, gt=-1.0)
    acceleration_real_pole_bias: float = Field(default=0.0, gt=-1.0)
    acceleration_damping_bias: float = Field(default=0.0, gt=-1.0)
    pitch_test_command_g: float = 0.0
    yaw_test_command_g: float = 0.0
    pitch_rate_command_deg_s: float = 0.0
    yaw_rate_command_deg_s: float = 0.0


####


class Ads6SamControllerDefinition(CadacModel):
    """Immutable source controller definition paired with one SAM plant actor."""

    source_name: str = Field(min_length=1)
    source_role: str = Field(min_length=1)
    module_order: tuple[str, ...]
    integration_step_s: float = Field(gt=0.0)
    initial_position_ned_m: tuple[float, float, float]
    selected_phase: Ads6SamPhase
    fin_actuator_mode: int = Field(default=0, ge=0, le=2)
    tvc_mode: int = Field(default=0, ge=0, le=3)
    rcs_moment_mode: int = Field(default=0, ge=0, le=29)
    rcs_force_mode: int = Field(default=0, ge=0, le=2)
    ins: Ads6SamInsConfig
    sensor: Ads6SamSensorConfig
    guidance: Ads6SamGuidanceConfig
    autopilot: Ads6SamAutopilotConfig
    events: tuple[CadacEventBlock, ...] = ()
    claim_boundary: str = (
        "Source-shaped ADS6 truth-aligned INS, deterministic RF/IR seeker kinematics and participating gimbal/filter "
        "states, radar-IP line guidance, terminal proportional navigation, and adaptive rate/acceleration autopilot. "
        "RF glint/thermal-noise power modeling, complete IR focal-plane/aimpoint corruption, exact source stochastic "
        "sequences, and compiled-CADAC parity remain outside this controller boundary."
    )

    @model_validator(mode="after")
    def validate_module_support(self) -> "Ads6SamControllerDefinition":
        missing = tuple(name for name in _SOURCE_CONTROLLER_MODULES if name not in self.module_order)
        if missing:
            raise ValueError(f"ADS6 source controller requires modules {missing!r}")
        ####
        if self.fin_actuator_mode not in {0, 2}:
            raise ValueError("ADS6 controller fin-actuator mode must be 0 or 2")
        ####
        return self

    ####


####


class Ads6SamControllerContext(CadacModel):
    """One source-order target packet and current RADAR0 intercept-point uplink."""

    missile_index: int = Field(ge=1, le=3)
    target_actor_id: str = Field(min_length=1)
    target_kind: Ads6SamTargetKind
    target_position_ned_m: tuple[float, float, float]
    target_velocity_ned_mps: tuple[float, float, float]
    target_packet_epoch_s: float = Field(ge=0.0)
    intercept_point_ned_m: tuple[float, float, float]
    radar_packet_epoch_s: float = Field(ge=0.0)


####


class Ads6SamAeroDerivatives(CadacModel):
    """Source finite-difference and dimensional derivatives used by the autopilot."""

    normal_alpha_mps2: float
    normal_control_mps2: float
    pitch_alpha_rad_s2: float
    pitch_rate_per_s: float
    pitch_control_rad_s2: float
    roll_rate_per_s: float
    roll_control_rad_s2: float
    side_beta_mps2: float
    yaw_beta_rad_s2: float
    yaw_rate_per_s: float
    yaw_control_rad_s2: float
    pitch_real_root_1_rad_s: float
    pitch_real_root_2_rad_s: float
    pitch_natural_frequency_rad_s: float
    yaw_real_root_1_rad_s: float
    yaw_real_root_2_rad_s: float
    yaw_natural_frequency_rad_s: float


####


class Ads6SamControllerEventTrace(CadacModel):
    """One sequential source event applied before a SAM module pass."""

    time_s: float = Field(ge=0.0)
    missile_time_s: float = Field(ge=0.0)
    event_index: int = Field(ge=0)
    source_line: int = Field(gt=0)
    watch_variable: str = Field(min_length=1)
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]


####


class Ads6SamControllerSample(CadacModel):
    """Current source-controller modes, measurements, commands, and evidence flags."""

    time_s: float = Field(ge=0.0)
    missile_time_s: float = Field(ge=0.0)
    control_mode: int
    guidance_mode: int
    sensor_mode: int
    sensor_kind: Ads6SamSensorKind
    target_actor_id: str | None
    target_packet_epoch_s: float | None = Field(default=None, ge=0.0)
    target_range_m: float = Field(ge=0.0)
    closing_speed_mps: float
    pointing_pitch_yaw_rad: tuple[float, float]
    los_rates_pitch_yaw_rad_s: tuple[float, float]
    tracking_error_pitch_yaw_rad: tuple[float, float]
    normal_lateral_command_g: tuple[float, float]
    requested_control_deg: tuple[float, float, float]
    radar_intercept_point_ned_m: tuple[float, float, float]
    source_event_count: int = Field(ge=0)
    ins_implementation: Literal["truth_aligned"] = "truth_aligned"
    rf_measurement_boundary: Literal["deterministic_no_glint_or_thermal_noise"] = "deterministic_no_glint_or_thermal_noise"
    ir_measurement_boundary: Literal["deterministic_no_focal_plane_or_aimpoint_corruption"] = "deterministic_no_focal_plane_or_aimpoint_corruption"


####


@dataclass(slots=True)
class _SensorAxisState:
    rate_rad_s: float = 0.0
    rate_derivative_rad_s2: float = 0.0
    acceleration_rad_s2: float = 0.0
    acceleration_derivative_rad_s3: float = 0.0


####


@dataclass(slots=True)
class Ads6SamSourceController:
    """Persistent source controller interleaved with an :class:`Ads6SamPlantStepper`."""

    definition: Ads6SamControllerDefinition
    plant_definition: Ads6SamSourceDefinition
    event_cursor: CadacEventCursor = field(init=False)
    control_mode: int = field(init=False)
    guidance_mode: int = field(init=False)
    sensor_mode: int = field(init=False)
    fin_actuator_mode: int = field(init=False)
    tvc_mode: int = field(init=False)
    rcs_moment_mode: int = field(init=False)
    rcs_force_mode: int = field(init=False)
    line_gain_per_s: float = field(init=False)
    nonlinear_gain_factor: float = field(init=False)
    distance_decrement_m: float = field(init=False)
    vertical_line_of_attack_deg: float = field(init=False)
    navigation_gain: float = field(init=False)
    navigation_gain_state: float = field(init=False)
    navigation_gain_derivative: float = 0.0
    navigation_gain_time_constant_s: float = field(init=False)
    acceleration_frequency_bias: float = field(init=False)
    acceleration_real_pole_bias: float = field(init=False)
    acceleration_damping_bias: float = field(init=False)
    pitch_test_command_g: float = field(init=False)
    yaw_test_command_g: float = field(init=False)
    event_time_s: float = 0.0
    _epoch_count: int = 0
    _sim_time_s: float = 0.0
    _missile_time_s: float = 0.0
    _context: Ads6SamControllerContext | None = None
    _target_range_m: float = math.inf
    _closing_speed_mps: float = 0.0
    _unit_los_local: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _unit_los_body: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _los_rate_body_rad_s: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _pointing_pitch_rad: float = 0.0
    _pointing_yaw_rad: float = 0.0
    _pointing_pitch_derivative_rad_s: float = 0.0
    _pointing_yaw_derivative_rad_s: float = 0.0
    _ir_pitch: _SensorAxisState = field(default_factory=_SensorAxisState)
    _ir_yaw: _SensorAxisState = field(default_factory=_SensorAxisState)
    _rf_pitch_gimbal_rad: float = 0.0
    _rf_yaw_gimbal_rad: float = 0.0
    _rf_pitch_gimbal_derivative_rad_s: float = 0.0
    _rf_yaw_gimbal_derivative_rad_s: float = 0.0
    _tracking_pitch_error_rad: float = 0.0
    _tracking_yaw_error_rad: float = 0.0
    _sensor_acquisition_epoch_s: float = 0.0
    _sensor_initialized: bool = False
    _normal_command_g: float = 0.0
    _lateral_command_g: float = 0.0
    _requested_control: Ads6SamControlCommand = field(default_factory=Ads6SamControlCommand)
    _pitch_feedforward_state: float = 0.0
    _pitch_feedforward_derivative: float = 0.0
    _yaw_feedforward_state: float = 0.0
    _yaw_feedforward_derivative: float = 0.0
    _derivatives: Ads6SamAeroDerivatives | None = None
    _event_traces: list[Ads6SamControllerEventTrace] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.event_cursor = CadacEventCursor.from_events(self.definition.events)
        self.control_mode = self.definition.autopilot.initial_mode
        self.guidance_mode = self.definition.guidance.initial_mode
        self.sensor_mode = self.definition.sensor.initial_mode
        self.fin_actuator_mode = self.definition.fin_actuator_mode
        self.tvc_mode = self.definition.tvc_mode
        self.rcs_moment_mode = self.definition.rcs_moment_mode
        self.rcs_force_mode = self.definition.rcs_force_mode
        self.line_gain_per_s = self.definition.guidance.line_gain_per_s
        self.nonlinear_gain_factor = self.definition.guidance.nonlinear_gain_factor
        self.distance_decrement_m = self.definition.guidance.distance_decrement_m
        self.vertical_line_of_attack_deg = self.definition.guidance.vertical_line_of_attack_deg
        self.navigation_gain = self.definition.guidance.navigation_gain
        self.navigation_gain_state = self.definition.guidance.navigation_gain_state
        self.navigation_gain_time_constant_s = self.definition.guidance.navigation_gain_time_constant_s
        self.acceleration_frequency_bias = self.definition.autopilot.acceleration_frequency_bias
        self.acceleration_real_pole_bias = self.definition.autopilot.acceleration_real_pole_bias
        self.acceleration_damping_bias = self.definition.autopilot.acceleration_damping_bias
        self.pitch_test_command_g = self.definition.autopilot.pitch_test_command_g
        self.yaw_test_command_g = self.definition.autopilot.yaw_test_command_g

    ####

    def begin_epoch(
        self,
        missile_time_s: float,
        sim_time_s: float,
        observation: Ads6SamPlantObservation,
        context: object,
    ) -> None:
        """Apply the next source event before the module loop."""

        resolved = _controller_context(context)
        if self._epoch_count > 0:
            self.event_time_s += self.definition.integration_step_s
        ####
        self._epoch_count += 1
        self._sim_time_s = sim_time_s
        self._missile_time_s = max(0.0, missile_time_s)
        self._context = resolved
        values = self._event_values()
        application = self.event_cursor.evaluate_and_apply(values)
        if application is not None:
            self._apply_event_values(values)
            self.event_time_s = 0.0
            self._event_traces.append(_event_trace(sim_time_s, self._missile_time_s, application))
        ####
        del observation

    ####

    def execute_module(
        self,
        module_name: str,
        observation: Ads6SamPlantObservation,
        context: object,
    ) -> Ads6SamDirectCommand | None:
        """Execute one source GNC module at its exact plant-module boundary."""

        resolved = _controller_context(context)
        self._context = resolved
        if module_name == "ins":
            self._derivatives = ads6_sam_aerodynamic_derivatives(
                self.plant_definition,
                observation,
            )
            return None
        ####
        if module_name == "sensor":
            self._sensor(observation, resolved)
            return None
        ####
        if module_name == "guidance":
            self._guidance(observation, resolved)
            return None
        ####
        if module_name == "control":
            self._requested_control = self._control(observation)
            return Ads6SamDirectCommand(
                phase=self.definition.selected_phase,
                control=self._requested_control,
                tvc_mode=self.tvc_mode if self.definition.selected_phase == "tvc_control" else None,
                rcs_moment_mode=(self.rcs_moment_mode if self.definition.selected_phase == "aggregate_rcs" else None),
                rcs_force_mode=(self.rcs_force_mode if self.definition.selected_phase == "aggregate_rcs" else None),
                acceleration_commands_g=(self._lateral_command_g, self._normal_command_g),
                thrust_vector_unit_body=_tuple3(self._unit_los_body, fallback=(1.0, 0.0, 0.0)),
            )
        ####
        raise ValueError(f"unsupported ADS6 controller module {module_name!r}")

    ####

    def sample(self) -> Ads6SamControllerSample:
        """Project current controller state for package telemetry and parity."""

        context = self._context
        sensor_kind = _sensor_kind(self.sensor_mode)
        return Ads6SamControllerSample(
            time_s=max(0.0, self._sim_time_s),
            missile_time_s=max(0.0, self._missile_time_s),
            control_mode=self.control_mode,
            guidance_mode=self.guidance_mode,
            sensor_mode=self.sensor_mode,
            sensor_kind=sensor_kind,
            target_actor_id=None if context is None else context.target_actor_id,
            target_packet_epoch_s=None if context is None else context.target_packet_epoch_s,
            target_range_m=max(0.0, self._target_range_m if math.isfinite(self._target_range_m) else 0.0),
            closing_speed_mps=self._closing_speed_mps,
            pointing_pitch_yaw_rad=(self._pointing_pitch_rad, self._pointing_yaw_rad),
            los_rates_pitch_yaw_rad_s=(
                float(self._los_rate_body_rad_s[1]),
                float(self._los_rate_body_rad_s[2]),
            ),
            tracking_error_pitch_yaw_rad=(
                self._tracking_pitch_error_rad,
                self._tracking_yaw_error_rad,
            ),
            normal_lateral_command_g=(self._normal_command_g, self._lateral_command_g),
            requested_control_deg=self._requested_control.vector(),
            radar_intercept_point_ned_m=((0.0, 0.0, 0.0) if context is None else context.intercept_point_ned_m),
            source_event_count=len(self._event_traces),
        )

    ####

    @property
    def event_traces(self) -> tuple[Ads6SamControllerEventTrace, ...]:
        return tuple(self._event_traces)

    ####

    def _sensor(
        self,
        observation: Ads6SamPlantObservation,
        context: Ads6SamControllerContext,
    ) -> None:
        sensor_type = self.sensor_mode // 10
        sensor_mode = self.sensor_mode % 10
        if self.sensor_mode == 0 or sensor_type not in {1, 2}:
            return
        ####
        missile_position = np.asarray(observation.position_ned_m, dtype=np.float64)
        missile_velocity = np.asarray(observation.velocity_ned_mps, dtype=np.float64)
        target_position = np.asarray(context.target_position_ned_m, dtype=np.float64)
        target_velocity = np.asarray(context.target_velocity_ned_mps, dtype=np.float64)
        body_from_local = _dcm_body_from_local(observation.quaternion_wxyz)
        sensor = cadac_local_ned_relative_state_track(
            time_s=self._sim_time_s,
            host_position_ned_m=missile_position,
            host_velocity_ned_mps=missile_velocity,
            target_id="ads6-sam-target",
            target_position_ned_m=target_position,
            target_velocity_ned_mps=target_velocity,
            body_from_local=body_from_local,
        )
        if isinstance(sensor, str):
            self._target_range_m = 0.0
            return
        ####
        distance = sensor.range_m
        self._target_range_m = distance
        missile_to_target = body_from_local.T @ np.asarray(sensor.relative_position_sensor_m, dtype=np.float64)
        relative_velocity = body_from_local.T @ np.asarray(sensor.relative_velocity_sensor_mps, dtype=np.float64)
        self._closing_speed_mps = sensor.closing_speed_mps
        unit_local = body_from_local.T @ np.asarray(sensor.unit_los_sensor, dtype=np.float64)
        unit_body = np.asarray(sensor.unit_los_sensor, dtype=np.float64)
        los_rate_body = np.asarray(sensor.line_of_sight_rate_sensor_rad_s, dtype=np.float64)
        _, yaw_true, pitch_true = _polar_from_cartesian(unit_body)
        self._unit_los_local = unit_local
        self._unit_los_body = unit_body
        self._los_rate_body_rad_s = los_rate_body
        acquisition_range = self.definition.sensor.rf_acquisition_range_m if sensor_type == 1 else self.definition.sensor.ir_acquisition_range_m
        acquisition_time = self.definition.sensor.rf_acquisition_time_s if sensor_type == 1 else self.definition.sensor.ir_acquisition_time_s
        if sensor_mode == 2 and distance < acquisition_range:
            sensor_mode = 3
            self._sensor_initialized = False
        ####
        if sensor_mode == 3 and not self._sensor_initialized:
            self._pointing_pitch_rad = pitch_true
            self._pointing_yaw_rad = yaw_true
            self._ir_pitch = _SensorAxisState(rate_rad_s=float(los_rate_body[1]))
            self._ir_yaw = _SensorAxisState(rate_rad_s=float(los_rate_body[2]))
            self._rf_pitch_gimbal_rad = pitch_true
            self._rf_yaw_gimbal_rad = yaw_true
            self._sensor_acquisition_epoch_s = self._sim_time_s
            self._sensor_initialized = True
        ####
        if sensor_mode in {3, 4}:
            if sensor_type == 1:
                if self.definition.sensor.dynamic_mode == 0:
                    self._pointing_pitch_rad = pitch_true
                    self._pointing_yaw_rad = yaw_true
                    self._tracking_pitch_error_rad = 0.0
                    self._tracking_yaw_error_rad = 0.0
                else:
                    self._rf_sensor(observation, missile_to_target, relative_velocity)
                ####
            else:
                self._ir_sensor(observation, pitch_true, yaw_true)
            ####
        ####
        if sensor_mode == 3 and self._sim_time_s - self._sensor_acquisition_epoch_s > acquisition_time:
            if sensor_type == 1:
                if self.definition.sensor.dynamic_mode == 0:
                    sensor_mode = 4
                else:
                    aspect_yaw_deg = abs(yaw_true * _DEG_PER_RAD)
                    aspect_pitch_deg = abs(pitch_true * _DEG_PER_RAD)
                    if max(aspect_yaw_deg, aspect_pitch_deg) <= self.definition.sensor.rf_field_of_regard_deg:
                        sensor_mode = 4
                    ####
                ####
            elif (
                abs(self._tracking_yaw_error_rad) <= self.definition.sensor.ir_yaw_half_fov_rad
                and abs(self._tracking_pitch_error_rad) <= self.definition.sensor.ir_pitch_half_fov_rad
            ):
                sensor_mode = 4
            ####
        ####
        if sensor_mode == 4 and distance < self.definition.sensor.blind_range_m:
            sensor_mode = 5
        ####
        if sensor_type == 2 and sensor_mode == 4 and self.definition.sensor.dynamic_mode == 1:
            tracking_error = math.hypot(
                self._tracking_pitch_error_rad,
                self._tracking_yaw_error_rad,
            )
            break_lock = (
                abs(self._pointing_pitch_rad) > self.definition.sensor.maximum_pitch_gimbal_rad
                or abs(self._pointing_pitch_derivative_rad_s) > self.definition.sensor.maximum_pitch_rate_rad_s
                or abs(self._pointing_yaw_derivative_rad_s) > self.definition.sensor.maximum_roll_rate_rad_s
                or tracking_error > self.definition.sensor.maximum_tracking_rate_rad_s
            )
            if break_lock:
                sensor_mode = 2
                self.guidance_mode = 50
                self._sensor_initialized = False
            ####
        ####
        self.sensor_mode = 10 * sensor_type + sensor_mode

    ####

    def _rf_sensor(
        self,
        observation: Ads6SamPlantObservation,
        missile_to_target_local: FloatVector,
        relative_velocity_local: FloatVector,
    ) -> None:
        body_from_local = _dcm_body_from_local(observation.quaternion_wxyz)
        yaw = self._rf_yaw_gimbal_rad
        pitch = self._rf_pitch_gimbal_rad
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        cpitch, spitch = math.cos(pitch), math.sin(pitch)
        gimbal_from_body = np.asarray(
            (
                (cyaw * cpitch, syaw, -cyaw * spitch),
                (-syaw * cpitch, cyaw, syaw * spitch),
                (spitch, 0.0, cpitch),
            ),
            dtype=np.float64,
        )
        relative_gimbal = gimbal_from_body @ body_from_local @ missile_to_target_local
        x, y, z = (float(value) for value in relative_gimbal)
        self._tracking_pitch_error_rad = math.atan2(-z, x)
        self._tracking_yaw_error_rad = math.atan2(y, x)
        gain = self.definition.sensor.rf_tracking_gain_per_s
        yaw_los_rate_gimbal = gain * self._tracking_yaw_error_rad
        pitch_los_rate_gimbal = gain * self._tracking_pitch_error_rad
        los_rate_body = gimbal_from_body.T @ np.asarray(
            (0.0, pitch_los_rate_gimbal, yaw_los_rate_gimbal),
            dtype=np.float64,
        )
        self._los_rate_body_rad_s = los_rate_body
        body_rates = gimbal_from_body @ np.asarray(observation.body_rates_rad_s, dtype=np.float64)
        yaw_rate_new = yaw_los_rate_gimbal - float(body_rates[2])
        pitch_rate_new = (pitch_los_rate_gimbal - float(body_rates[1])) / _nonzero(cyaw)
        self._rf_yaw_gimbal_rad = _integrate_scalar(
            self._rf_yaw_gimbal_rad,
            yaw_rate_new,
            self._rf_yaw_gimbal_derivative_rad_s,
            self.definition.integration_step_s,
        )
        self._rf_pitch_gimbal_rad = _integrate_scalar(
            self._rf_pitch_gimbal_rad,
            pitch_rate_new,
            self._rf_pitch_gimbal_derivative_rad_s,
            self.definition.integration_step_s,
        )
        self._rf_yaw_gimbal_derivative_rad_s = yaw_rate_new
        self._rf_pitch_gimbal_derivative_rad_s = pitch_rate_new
        self._pointing_yaw_rad = self._rf_yaw_gimbal_rad
        self._pointing_pitch_rad = self._rf_pitch_gimbal_rad
        distance = max(_SMALL, float(np.linalg.norm(missile_to_target_local)))
        unit_local = missile_to_target_local / distance
        self._closing_speed_mps = -float(unit_local @ relative_velocity_local)

    ####

    def _ir_sensor(
        self,
        observation: Ads6SamPlantObservation,
        pitch_true_rad: float,
        yaw_true_rad: float,
    ) -> None:
        if self.definition.sensor.dynamic_mode == 0:
            self._pointing_pitch_rad = pitch_true_rad
            self._pointing_yaw_rad = yaw_true_rad
            self._tracking_pitch_error_rad = 0.0
            self._tracking_yaw_error_rad = 0.0
            return
        ####
        pitch_error = _wrap_pi(pitch_true_rad - self._pointing_pitch_rad)
        yaw_error = _wrap_pi(yaw_true_rad - self._pointing_yaw_rad)
        self._tracking_pitch_error_rad = pitch_error
        self._tracking_yaw_error_rad = yaw_error
        self._ir_pitch = _update_ir_axis(
            self._ir_pitch,
            pitch_error,
            self.definition.sensor,
            self.definition.integration_step_s,
        )
        self._ir_yaw = _update_ir_axis(
            self._ir_yaw,
            yaw_error,
            self.definition.sensor,
            self.definition.integration_step_s,
        )
        pointing_from_body = _mat2tr(
            self._pointing_yaw_rad,
            self._pointing_pitch_rad,
        )
        body_rate_pointing = pointing_from_body @ np.asarray(
            observation.body_rates_rad_s,
            dtype=np.float64,
        )
        pitch_rate_new = self._ir_pitch.rate_rad_s - float(body_rate_pointing[1])
        yaw_rate_new = self._ir_yaw.rate_rad_s - float(body_rate_pointing[2])
        self._pointing_pitch_rad = _integrate_scalar(
            self._pointing_pitch_rad,
            pitch_rate_new,
            self._pointing_pitch_derivative_rad_s,
            self.definition.integration_step_s,
        )
        self._pointing_yaw_rad = _integrate_scalar(
            self._pointing_yaw_rad,
            yaw_rate_new,
            self._pointing_yaw_derivative_rad_s,
            self.definition.integration_step_s,
        )
        self._pointing_pitch_derivative_rad_s = pitch_rate_new
        self._pointing_yaw_derivative_rad_s = yaw_rate_new
        self._los_rate_body_rad_s[1] = self._ir_pitch.rate_rad_s
        self._los_rate_body_rad_s[2] = self._ir_yaw.rate_rad_s

    ####

    def _guidance(
        self,
        observation: Ads6SamPlantObservation,
        context: Ads6SamControllerContext,
    ) -> None:
        midcourse = self.guidance_mode // 10
        terminal = self.guidance_mode % 10
        self._normal_command_g = 0.0
        self._lateral_command_g = 0.0
        if midcourse == 2:
            self._line_guidance(observation, context)
        elif midcourse == 3:
            self._midcourse_pronav(observation, context)
        ####
        if terminal == 6:
            self._terminal_ir_guidance(observation, context)
        elif terminal == 7:
            self._terminal_rf_guidance(observation)
        ####
        maximum = observation.coefficients.maximum_load_g
        magnitude = math.hypot(self._lateral_command_g, self._normal_command_g)
        if magnitude > maximum:
            if maximum <= 0.0:
                self._lateral_command_g = 0.0
                self._normal_command_g = 0.0
            else:
                scale = maximum / magnitude
                self._lateral_command_g *= scale
                self._normal_command_g *= scale
            ####
        ####

    ####

    def _line_guidance(
        self,
        observation: Ads6SamPlantObservation,
        context: Ads6SamControllerContext,
    ) -> None:
        intercept = np.asarray(context.intercept_point_ned_m, dtype=np.float64)
        current = np.asarray(observation.position_ned_m, dtype=np.float64)
        initial = np.asarray(self.definition.initial_position_ned_m, dtype=np.float64)
        to_intercept = intercept - current
        initial_to_intercept = intercept - initial
        distance, los_heading, los_pitch = _polar_from_cartesian(to_intercept)
        _, line_heading, _ = _polar_from_cartesian(initial_to_intercept)
        if distance <= _SMALL:
            return
        ####
        line_pitch = self.vertical_line_of_attack_deg * _RAD_PER_DEG
        line_transform = _mat2tr(line_heading, line_pitch)
        los_transform = _mat2tr(los_heading, los_pitch)
        velocity = np.asarray(observation.velocity_ned_mps, dtype=np.float64)
        velocity_los = los_transform @ velocity
        velocity_line = line_transform @ velocity
        north, east, down = (float(value) for value in velocity)
        heading = math.atan2(east, north) if north != 0.0 or east != 0.0 else 0.0
        flight_path = math.atan2(-down, math.hypot(north, east))
        velocity_transform = _mat2tr(heading, flight_path)
        body_from_velocity = _dcm_body_from_local(observation.quaternion_wxyz) @ velocity_transform.T
        nonlinear_gain = self.nonlinear_gain_factor * (1.0 - math.exp(-distance / max(self.distance_decrement_m, _SMALL)))
        acceleration_velocity = np.asarray(
            (
                observation.gravity_mps2 * math.sin(flight_path),
                self.line_gain_per_s * (-float(velocity_los[1]) + nonlinear_gain * float(velocity_line[1])),
                self.line_gain_per_s * (-float(velocity_los[2]) + nonlinear_gain * float(velocity_line[2])) - observation.gravity_mps2 * math.cos(flight_path),
            ),
            dtype=np.float64,
        )
        acceleration_body_g = body_from_velocity @ acceleration_velocity / _AGRAV
        self._lateral_command_g = float(acceleration_body_g[1])
        self._normal_command_g = -float(acceleration_body_g[2])

    ####

    def _midcourse_pronav(
        self,
        observation: Ads6SamPlantObservation,
        context: Ads6SamControllerContext,
    ) -> None:
        relative = np.asarray(context.intercept_point_ned_m, dtype=np.float64) - np.asarray(
            observation.position_ned_m,
            dtype=np.float64,
        )
        distance = float(np.linalg.norm(relative))
        if distance <= _SMALL:
            return
        ####
        unit_local = relative / distance
        velocity = np.asarray(observation.velocity_ned_mps, dtype=np.float64)
        closing = abs(float(unit_local @ velocity))
        los_rate_local = np.cross(unit_local, velocity) / distance
        acceleration_local = np.cross(los_rate_local, unit_local) * self.navigation_gain * closing
        acceleration_body_g = _dcm_body_from_local(observation.quaternion_wxyz) @ acceleration_local / _AGRAV
        self._lateral_command_g = float(acceleration_body_g[1])
        self._normal_command_g = -float(acceleration_body_g[2])

    ####

    def _terminal_ir_guidance(
        self,
        observation: Ads6SamPlantObservation,
        context: Ads6SamControllerContext,
    ) -> None:
        target_position = np.asarray(context.target_position_ned_m, dtype=np.float64)
        target_velocity = np.asarray(context.target_velocity_ned_mps, dtype=np.float64)
        missile_position = np.asarray(observation.position_ned_m, dtype=np.float64)
        missile_velocity = np.asarray(observation.velocity_ned_mps, dtype=np.float64)
        target_to_missile = missile_position - target_position
        distance = max(_SMALL, float(np.linalg.norm(target_to_missile)))
        closing_signed = float(target_to_missile @ (missile_velocity - target_velocity)) / distance
        gain = self._delayed_navigation_gain()
        compensated_gain = -gain * closing_signed
        pitch_rate = float(self._los_rate_body_rad_s[1])
        yaw_rate = float(self._los_rate_body_rad_s[2])
        yaw = self._pointing_yaw_rad
        pitch = self._pointing_pitch_rad
        longitudinal_specific_force = float(observation.specific_force_body_mps2[0])
        lateral_compensation = longitudinal_specific_force * math.tan(yaw) / _AGRAV
        pitch_compensation = longitudinal_specific_force * math.tan(pitch) / (_nonzero(math.cos(yaw)) * _AGRAV)
        gravity_body = _dcm_body_from_local(observation.quaternion_wxyz) @ np.asarray(
            (0.0, 0.0, self.definition.guidance.gravity_bias_g),
            dtype=np.float64,
        )
        lateral_pn = compensated_gain * yaw_rate / (_nonzero(math.cos(yaw)) * _AGRAV)
        normal_pn = compensated_gain * (yaw_rate * math.tan(pitch) * math.tan(yaw) + pitch_rate / _nonzero(math.cos(pitch))) / _AGRAV
        self._lateral_command_g = lateral_pn + lateral_compensation - float(gravity_body[1])
        self._normal_command_g = normal_pn + pitch_compensation + float(gravity_body[2])

    ####

    def _terminal_rf_guidance(self, observation: Ads6SamPlantObservation) -> None:
        gain = self._delayed_navigation_gain()
        range_rate = -self._closing_speed_mps
        compensated_gain = -gain * range_rate
        yaw = self._rf_yaw_gimbal_rad
        pitch = self._rf_pitch_gimbal_rad
        pitch_rate = float(self._los_rate_body_rad_s[1])
        yaw_rate = float(self._los_rate_body_rad_s[2])
        gravity_body = _dcm_body_from_local(observation.quaternion_wxyz) @ np.asarray(
            (0.0, 0.0, self.definition.guidance.gravity_bias_g),
            dtype=np.float64,
        )
        lateral_pn = compensated_gain * yaw_rate / (_nonzero(math.cos(yaw)) * _AGRAV)
        normal_pn = compensated_gain * (yaw_rate * math.tan(pitch) * math.tan(yaw) + pitch_rate / _nonzero(math.cos(pitch))) / _AGRAV
        self._lateral_command_g = lateral_pn - float(gravity_body[1])
        self._normal_command_g = normal_pn + float(gravity_body[2])

    ####

    def _delayed_navigation_gain(self) -> float:
        if self.navigation_gain_time_constant_s > 0.0:
            derivative_new = (self.navigation_gain - self.navigation_gain_state) / self.navigation_gain_time_constant_s
            self.navigation_gain_state = _integrate_scalar(
                self.navigation_gain_state,
                derivative_new,
                self.navigation_gain_derivative,
                self.definition.integration_step_s,
            )
            self.navigation_gain_derivative = derivative_new
        else:
            self.navigation_gain_state = self.navigation_gain
        ####
        return self.navigation_gain_state

    ####

    def _control(self, observation: Ads6SamPlantObservation) -> Ads6SamControlCommand:
        if self.control_mode == 0:
            return Ads6SamControlCommand()
        ####
        derivatives = self._derivatives or ads6_sam_aerodynamic_derivatives(
            self.plant_definition,
            observation,
        )
        roll = self._roll_control(observation, derivatives)
        pitch = 0.0
        yaw = 0.0
        if self.control_mode == 2:
            pitch, yaw = self._rate_control(observation, derivatives)
        elif self.control_mode in {3, 4}:
            pitch, yaw = self._acceleration_control(observation, derivatives)
        ####
        return Ads6SamControlCommand(
            roll_deg=_clip(roll, self.definition.autopilot.roll_command_limit_deg),
            pitch_deg=_clip(pitch, self.definition.autopilot.pitch_command_limit_deg),
            yaw_deg=_clip(yaw, self.definition.autopilot.yaw_command_limit_deg),
        )

    ####

    def _roll_control(
        self,
        observation: Ads6SamPlantObservation,
        derivatives: Ads6SamAeroDerivatives,
    ) -> float:
        dlp = derivatives.roll_rate_per_s
        dld = _nonzero(derivatives.roll_control_rad_s2)
        bandwidth = -0.8 * dlp * (1.0 + self.definition.autopilot.roll_bandwidth_factor)
        gkp = (2.0 * self.definition.autopilot.roll_damping_ratio * bandwidth + dlp) / dld
        gkphi = bandwidth * bandwidth / dld
        roll_rate = float(observation.body_rates_rad_s[0])
        roll_angle_deg = observation.body_angles_deg[2]
        pitch_angle_deg = observation.body_angles_deg[1]
        if abs(pitch_angle_deg) > 88.0:
            gain = (1.0 / self.definition.autopilot.roll_rate_time_constant_s + dlp) / dld
            return gain * roll_rate * _DEG_PER_RAD
        ####
        error = gkphi * (self.definition.autopilot.commanded_roll_deg - roll_angle_deg) * _RAD_PER_DEG
        return (error - gkp * roll_rate) * _DEG_PER_RAD

    ####

    def _rate_control(
        self,
        observation: Ads6SamPlantObservation,
        derivatives: Ads6SamAeroDerivatives,
    ) -> tuple[float, float]:
        speed = max(_SMALL, float(np.linalg.norm(observation.velocity_ned_mps)))
        dna = derivatives.normal_alpha_mps2
        dnd = derivatives.normal_control_mps2
        dma = derivatives.pitch_alpha_rad_s2
        dmq = derivatives.pitch_rate_per_s
        dmd = _nonzero(derivatives.pitch_control_rad_s2)
        zrate = dna / speed - dma * dnd / (speed * dmd)
        aa = dna / speed - dmq
        bb = -dma - dmq * dna / speed
        damping = self.definition.autopilot.rate_loop_damping_ratio
        dum1 = aa - 2.0 * damping * damping * zrate
        dum2 = aa * aa - 4.0 * damping * damping * bb
        radix = max(0.0, dum1 * dum1 - dum2)
        gain = (-dum1 + math.sqrt(radix)) / dmd
        pitch = gain * float(observation.body_rates_rad_s[1]) * _DEG_PER_RAD
        yaw = gain * float(observation.body_rates_rad_s[2]) * _DEG_PER_RAD
        return (pitch, yaw)

    ####

    def _acceleration_control(
        self,
        observation: Ads6SamPlantObservation,
        derivatives: Ads6SamAeroDerivatives,
    ) -> tuple[float, float]:
        normal = self._normal_command_g + self.pitch_test_command_g
        lateral = self._lateral_command_g + self.yaw_test_command_g
        magnitude = math.hypot(lateral, normal)
        if magnitude > self.definition.autopilot.structural_limit_g:
            scale = self.definition.autopilot.structural_limit_g / magnitude
            lateral *= scale
            normal *= scale
        ####
        zacl = 0.7 * (1.0 + self.acceleration_damping_bias)
        wacl_pitch = abs(derivatives.pitch_real_root_1_rad_s) * (1.0 + self.acceleration_frequency_bias)
        pacl_pitch = (abs(derivatives.pitch_real_root_2_rad_s) + 35.0) * (1.0 + self.acceleration_real_pole_bias)
        pitch = self._acceleration_axis(
            command_g=normal,
            measured_specific_force_mps2=float(observation.specific_force_body_mps2[2]),
            body_rate_rad_s=float(observation.body_rates_rad_s[1]),
            force_slope=derivatives.normal_alpha_mps2,
            moment_slope=derivatives.pitch_alpha_rad_s2,
            moment_rate=derivatives.pitch_rate_per_s,
            moment_control=derivatives.pitch_control_rad_s2,
            natural_frequency=wacl_pitch,
            real_pole=pacl_pitch,
            damping=zacl,
            yaw_axis=False,
            speed_mps=max(_SMALL, float(np.linalg.norm(observation.velocity_ned_mps))),
        )
        # CADAC computes one adaptive closed-loop pole pair from the pitch roots,
        # then reuses that pair when forming the yaw-axis feedback gains.
        yaw = self._acceleration_axis(
            command_g=lateral,
            measured_specific_force_mps2=float(observation.specific_force_body_mps2[1]),
            body_rate_rad_s=float(observation.body_rates_rad_s[2]),
            force_slope=derivatives.side_beta_mps2,
            moment_slope=derivatives.yaw_beta_rad_s2,
            moment_rate=derivatives.yaw_rate_per_s,
            moment_control=derivatives.yaw_control_rad_s2,
            natural_frequency=wacl_pitch,
            real_pole=pacl_pitch,
            damping=zacl,
            yaw_axis=True,
            speed_mps=max(_SMALL, float(np.linalg.norm(observation.velocity_ned_mps))),
        )
        return pitch, yaw

    ####

    def _acceleration_axis(
        self,
        *,
        command_g: float,
        measured_specific_force_mps2: float,
        body_rate_rad_s: float,
        force_slope: float,
        moment_slope: float,
        moment_rate: float,
        moment_control: float,
        natural_frequency: float,
        real_pole: float,
        damping: float,
        yaw_axis: bool,
        speed_mps: float,
    ) -> float:
        gainp = self.definition.autopilot.acceleration_feedforward_gain_s2_m
        control = _nonzero(moment_control)
        if yaw_axis:
            slope = _nonzero(force_slope)
            gainfb3 = -(natural_frequency**2) * real_pole / (slope * control)
            gainfb2 = (2.0 * damping * natural_frequency + real_pole + moment_rate + slope / speed_mps) / control
            gainfb1 = (
                -(natural_frequency**2)
                - 2.0 * damping * natural_frequency * real_pole
                + moment_slope
                + moment_rate * slope / speed_mps
                - gainfb2 * slope * control / speed_mps
            ) / (slope * control) - gainp
            derivative_new = _AGRAV * command_g - measured_specific_force_mps2
            self._yaw_feedforward_state = _integrate_scalar(
                self._yaw_feedforward_state,
                derivative_new,
                self._yaw_feedforward_derivative,
                self.definition.integration_step_s,
            )
            self._yaw_feedforward_derivative = derivative_new
            output = -gainfb1 * measured_specific_force_mps2 - gainfb2 * body_rate_rad_s + gainfb3 * self._yaw_feedforward_state + gainp * derivative_new
            return output * _DEG_PER_RAD
        ####
        slope = _nonzero(force_slope)
        gainfb3 = natural_frequency**2 * real_pole / (slope * control)
        gainfb2 = (2.0 * damping * natural_frequency + real_pole + moment_rate - slope / speed_mps) / control
        gainfb1 = (
            natural_frequency**2
            + 2.0 * damping * natural_frequency * real_pole
            + moment_slope
            + moment_rate * slope / speed_mps
            - gainfb2 * slope * control / speed_mps
        ) / (slope * control) - gainp
        derivative_new = _AGRAV * command_g + measured_specific_force_mps2
        self._pitch_feedforward_state = _integrate_scalar(
            self._pitch_feedforward_state,
            derivative_new,
            self._pitch_feedforward_derivative,
            self.definition.integration_step_s,
        )
        self._pitch_feedforward_derivative = derivative_new
        output = -gainfb1 * (-measured_specific_force_mps2) - gainfb2 * body_rate_rad_s + gainfb3 * self._pitch_feedforward_state + gainp * derivative_new
        return output * _DEG_PER_RAD

    ####

    def _event_values(self) -> dict[str, CadacRuntimeScalar]:
        return {
            "time": self._sim_time_s,
            "msl_time": self._missile_time_s,
            "event_time": self.event_time_s,
            "mseek": self.sensor_mode,
            "mguide": self.guidance_mode,
            "maut": self.control_mode,
            "mact": self.fin_actuator_mode,
            "mtvc": self.tvc_mode,
            "mrcs_moment": self.rcs_moment_mode,
            "mrcs_force": self.rcs_force_mode,
            "line_gain": self.line_gain_per_s,
            "nl_gain_fact": self.nonlinear_gain_factor,
            "decrement": self.distance_decrement_m,
            "thtflx": self.vertical_line_of_attack_deg,
            "gnav": self.navigation_gain,
            "gn": self.navigation_gain_state,
            "tgnav": self.navigation_gain_time_constant_s,
            "wacl_bias": self.acceleration_frequency_bias,
            "pacl_bias": self.acceleration_real_pole_bias,
            "zacl_bias": self.acceleration_damping_bias,
            "ancomx_test": self.pitch_test_command_g,
            "alcomx_test": self.yaw_test_command_g,
        }

    ####

    def _apply_event_values(self, values: dict[str, CadacRuntimeScalar]) -> None:
        previous_sensor_mode = self.sensor_mode
        self.sensor_mode = int(values["mseek"])
        self.guidance_mode = int(values["mguide"])
        self.control_mode = int(values["maut"])
        self.fin_actuator_mode = int(values["mact"])
        self.tvc_mode = int(values["mtvc"])
        self.rcs_moment_mode = int(values["mrcs_moment"])
        self.rcs_force_mode = int(values["mrcs_force"])
        self.line_gain_per_s = float(values["line_gain"])
        self.nonlinear_gain_factor = float(values["nl_gain_fact"])
        self.distance_decrement_m = max(_SMALL, float(values["decrement"]))
        self.vertical_line_of_attack_deg = float(values["thtflx"])
        self.navigation_gain = float(values["gnav"])
        self.navigation_gain_state = float(values["gn"])
        self.navigation_gain_time_constant_s = max(0.0, float(values["tgnav"]))
        self.acceleration_frequency_bias = float(values["wacl_bias"])
        self.acceleration_real_pole_bias = float(values["pacl_bias"])
        self.acceleration_damping_bias = float(values["zacl_bias"])
        self.pitch_test_command_g = float(values["ancomx_test"])
        self.yaw_test_command_g = float(values["alcomx_test"])
        if self.fin_actuator_mode not in {0, 2}:
            raise ValueError("source event selected unsupported ADS6 fin actuator mode")
        ####
        if self.sensor_mode != previous_sensor_mode:
            self._sensor_initialized = False
        ####

    ####


####


def lower_ads6_sam_controller_actor(
    bundle: CadacSourceBundle,
    vehicle: CadacVehicleBlock,
    plant: Ads6SamSourceDefinition,
    *,
    selected_phase: Ads6SamPhase,
) -> Ads6SamControllerDefinition:
    """Lower source GNC values for one ADS6 ``MISSILE6`` package actor."""

    if vehicle.model_name.casefold() != "missile6":
        raise Ads6SamControllerSourceError(f"expected MISSILE6 actor, received {vehicle.model_name!r}")
    ####
    target_flag = _integer(vehicle, "mtarget", 2)
    if target_flag not in {1, 2}:
        raise Ads6SamControllerSourceError("ADS6 SAM mtarget must be 1=SRBM or 2=aircraft")
    ####
    return Ads6SamControllerDefinition(
        source_name=bundle.case.source_name,
        source_role=vehicle.role,
        module_order=plant.module_order,
        integration_step_s=plant.integration_step_s,
        initial_position_ned_m=plant.initial_state.position_ned_m,
        selected_phase=selected_phase,
        fin_actuator_mode=plant.fin_actuator.mode,
        tvc_mode=plant.tvc.mode,
        rcs_moment_mode=plant.rcs.moment_mode,
        rcs_force_mode=plant.rcs.force_mode,
        ins=Ads6SamInsConfig(mode=_integer(vehicle, "mins", 0)),
        sensor=Ads6SamSensorConfig(
            initial_mode=_integer(vehicle, "mseek", 0),
            dynamic_mode=_integer(vehicle, "skr_dyn", 0),
            target_kind="srbm" if target_flag == 1 else "aircraft",
            blind_range_m=_number(vehicle, "dblind", 0.0),
            rf_acquisition_range_m=_number(vehicle, "racq_rf", 0.0),
            rf_acquisition_time_s=_number(vehicle, "dtimac_rf", 0.0),
            rf_field_of_regard_deg=_number(vehicle, "forlim_rfx", 180.0),
            rf_field_of_view_deg=_number(vehicle, "fovlim_rfx", 180.0),
            rf_tracking_gain_per_s=_number(vehicle, "gain_rf", 0.0),
            ir_acquisition_range_m=_number(vehicle, "racq_ir", 0.0),
            ir_acquisition_time_s=_number(vehicle, "dtimac_ir", 0.0),
            ir_yaw_half_fov_rad=_number(vehicle, "fovyaw_ir", math.pi),
            ir_pitch_half_fov_rad=_number(vehicle, "fovpitch_ir", math.pi),
            ir_filter_gain_per_s=_number(vehicle, "gk", 0.0),
            ir_filter_natural_frequency_rad_s=_number(vehicle, "wnk", 0.0),
            ir_filter_damping_ratio=_number(vehicle, "zetak", 0.0),
            maximum_pitch_gimbal_rad=_number(vehicle, "trtht", math.pi),
            maximum_pitch_rate_rad_s=_number(vehicle, "trthtd", math.inf),
            maximum_roll_rate_rad_s=_number(vehicle, "trphid", math.inf),
            maximum_tracking_rate_rad_s=_number(vehicle, "trate", math.inf),
        ),
        guidance=Ads6SamGuidanceConfig(
            initial_mode=_integer(vehicle, "mguide", 0),
            gravity_bias_g=_number(vehicle, "grav_bias", 1.0),
            navigation_gain=_number(vehicle, "gnav", 0.0),
            navigation_gain_state=_number(vehicle, "gn", 0.0),
            navigation_gain_time_constant_s=_number(vehicle, "tgnav", 0.0),
            line_gain_per_s=_number(vehicle, "line_gain", 0.0),
            nonlinear_gain_factor=_number(vehicle, "nl_gain_fact", 0.0),
            distance_decrement_m=max(_SMALL, _number(vehicle, "decrement", 1.0)),
            vertical_line_of_attack_deg=_number(vehicle, "thtflx", 0.0),
        ),
        autopilot=Ads6SamAutopilotConfig(
            initial_mode=_integer(vehicle, "maut", 0),
            structural_limit_g=_number(vehicle, "alimitx", plant.aerodynamics.structural_limit_g),
            roll_command_limit_deg=_number(vehicle, "dplimx", plant.fin_actuator.position_limit_deg),
            pitch_command_limit_deg=_number(vehicle, "dqlimx", plant.fin_actuator.position_limit_deg),
            yaw_command_limit_deg=_number(vehicle, "drlimx", plant.fin_actuator.position_limit_deg),
            commanded_roll_deg=_number(vehicle, "phicomx", 0.0),
            roll_damping_ratio=_number(vehicle, "zrcl", 0.9),
            roll_rate_time_constant_s=max(_SMALL, _number(vehicle, "tp", 0.1)),
            roll_bandwidth_factor=_number(vehicle, "factwrcl", 0.0),
            rate_loop_damping_ratio=_number(vehicle, "zetlagr", 1.2),
            acceleration_feedforward_gain_s2_m=_number(vehicle, "gainp", 0.0),
            acceleration_frequency_bias=_number(vehicle, "wacl_bias", 0.0),
            acceleration_real_pole_bias=_number(vehicle, "pacl_bias", 0.0),
            acceleration_damping_bias=_number(vehicle, "zacl_bias", 0.0),
            pitch_test_command_g=_number(vehicle, "ancomx_test", 0.0),
            yaw_test_command_g=_number(vehicle, "alcomx_test", 0.0),
            pitch_rate_command_deg_s=_number(vehicle, "qqcomx", 0.0),
            yaw_rate_command_deg_s=_number(vehicle, "rrcomx", 0.0),
        ),
        events=vehicle.events,
    )


####


def ads6_sam_aerodynamic_derivatives(
    definition: Ads6SamSourceDefinition,
    observation: Ads6SamPlantObservation,
) -> Ads6SamAeroDerivatives:
    """Project the current source-pass aerodynamic derivative ledger.

    The physical plant evaluates the source finite-difference derivatives during
    its ``aerodynamics`` module.  The controller consumes that exact ledger at
    the subsequent ``ins`` boundary rather than repeating table queries with a
    potentially different state snapshot.
    """

    del definition
    coefficients = observation.coefficients
    return Ads6SamAeroDerivatives(
        normal_alpha_mps2=coefficients.normal_alpha_derivative_mps2,
        normal_control_mps2=coefficients.normal_control_derivative_mps2,
        pitch_alpha_rad_s2=coefficients.pitch_alpha_derivative_rad_s2,
        pitch_rate_per_s=coefficients.pitch_rate_derivative_per_s,
        pitch_control_rad_s2=coefficients.pitch_control_derivative_rad_s2,
        roll_rate_per_s=coefficients.roll_rate_derivative_per_s,
        roll_control_rad_s2=coefficients.roll_control_derivative_rad_s2,
        side_beta_mps2=coefficients.side_beta_derivative_mps2,
        yaw_beta_rad_s2=coefficients.yaw_beta_derivative_rad_s2,
        yaw_rate_per_s=coefficients.yaw_rate_derivative_per_s,
        yaw_control_rad_s2=coefficients.yaw_control_derivative_rad_s2,
        pitch_real_root_1_rad_s=coefficients.pitch_real_root_1_rad_s,
        pitch_real_root_2_rad_s=coefficients.pitch_real_root_2_rad_s,
        pitch_natural_frequency_rad_s=coefficients.pitch_natural_frequency_rad_s,
        yaw_real_root_1_rad_s=coefficients.yaw_real_root_1_rad_s,
        yaw_real_root_2_rad_s=coefficients.yaw_real_root_2_rad_s,
        yaw_natural_frequency_rad_s=coefficients.yaw_natural_frequency_rad_s,
    )


####


def _update_ir_axis(
    state: _SensorAxisState,
    error_rad: float,
    config: Ads6SamSensorConfig,
    dt_s: float,
) -> _SensorAxisState:
    rate_derivative_new = state.acceleration_rad_s2
    rate = _integrate_scalar(
        state.rate_rad_s,
        rate_derivative_new,
        state.rate_derivative_rad_s2,
        dt_s,
    )
    derivative_new = (
        config.ir_filter_gain_per_s * config.ir_filter_natural_frequency_rad_s**2 * error_rad
        - 2.0 * config.ir_filter_damping_ratio * config.ir_filter_natural_frequency_rad_s * rate_derivative_new
        - config.ir_filter_natural_frequency_rad_s**2 * rate
    )
    acceleration = _integrate_scalar(
        state.acceleration_rad_s2,
        derivative_new,
        state.acceleration_derivative_rad_s3,
        dt_s,
    )
    return _SensorAxisState(
        rate_rad_s=rate,
        rate_derivative_rad_s2=rate_derivative_new,
        acceleration_rad_s2=acceleration,
        acceleration_derivative_rad_s3=derivative_new,
    )


####


def _controller_context(context: object) -> Ads6SamControllerContext:
    if not isinstance(context, Ads6SamControllerContext):
        raise TypeError("ADS6 source controller received an incompatible context")
    ####
    return cast(Ads6SamControllerContext, context)


####


def _event_trace(
    time_s: float,
    missile_time_s: float,
    application: CadacEventApplication,
) -> Ads6SamControllerEventTrace:
    return Ads6SamControllerEventTrace(
        time_s=time_s,
        missile_time_s=missile_time_s,
        event_index=application.event_index,
        source_line=application.source_line,
        watch_variable=application.watch_variable,
        previous_values=application.previous_values,
        updated_values=application.updated_values,
    )


####


def _sensor_kind(mode: int) -> Ads6SamSensorKind:
    sensor_type = mode // 10
    if sensor_type == 1:
        return "rf"
    ####
    if sensor_type == 2:
        return "ir"
    ####
    return "off"


####


def _integrate_scalar(state: float, derivative_new: float, derivative_previous: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((state,), (derivative_new,), (derivative_previous,), dt_s)[0]


####


def _dcm_body_from_local(quaternion_wxyz: tuple[float, float, float, float]) -> FloatMatrix:
    q0, q1, q2, q3 = quaternion_wxyz
    return np.asarray(
        (
            (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2.0 * (q1 * q2 + q0 * q3), 2.0 * (q1 * q3 - q0 * q2)),
            (2.0 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2.0 * (q2 * q3 + q0 * q1)),
            (2.0 * (q1 * q3 + q0 * q2), 2.0 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3),
        ),
        dtype=np.float64,
    )


####


def _mat2tr(heading_rad: float, flight_path_rad: float) -> FloatMatrix:
    cpsi, spsi = math.cos(heading_rad), math.sin(heading_rad)
    ctheta, stheta = math.cos(flight_path_rad), math.sin(flight_path_rad)
    return np.asarray(
        (
            (ctheta * cpsi, ctheta * spsi, -stheta),
            (-spsi, cpsi, 0.0),
            (stheta * cpsi, stheta * spsi, ctheta),
        ),
        dtype=np.float64,
    )


####


def _polar_from_cartesian(vector: FloatVector) -> tuple[float, float, float]:
    magnitude = float(np.linalg.norm(vector))
    if magnitude <= _SMALL:
        return 0.0, 0.0, 0.0
    ####
    north, east, down = (float(value) for value in vector)
    azimuth = math.atan2(east, north)
    elevation = math.atan2(-down, math.hypot(north, east))
    return magnitude, azimuth, elevation


####


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


####


def _nonzero(value: float) -> float:
    if abs(value) >= _SMALL:
        return value
    ####
    return _SMALL if value >= 0.0 else -_SMALL


####


def _clip(value: float, limit: float) -> float:
    if abs(value) <= limit:
        return value
    ####
    return math.copysign(limit, value)


####


def _tuple3(
    values: FloatVector,
    *,
    fallback: tuple[float, float, float] | None = None,
) -> tuple[float, float, float]:
    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)) or float(np.linalg.norm(array)) <= _SMALL:
        if fallback is None:
            return tuple(float(value) for value in array)
        ####
        return fallback
    ####
    return tuple(float(value) for value in array)


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Ads6SamControllerSourceError(f"{vehicle.model_name} {vehicle.role!r} is missing required parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Ads6SamControllerSourceError(f"ADS6 controller parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Ads6SamControllerSourceError(f"ADS6 controller parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    integer = int(value)
    if float(integer) != value:
        raise Ads6SamControllerSourceError(f"ADS6 controller parameter {name!r} must be integral")
    ####
    return integer


####


__all__ = [
    "Ads6SamAeroDerivatives",
    "Ads6SamAutopilotConfig",
    "Ads6SamControllerContext",
    "Ads6SamControllerDefinition",
    "Ads6SamControllerEventTrace",
    "Ads6SamControllerSample",
    "Ads6SamControllerSourceError",
    "Ads6SamGuidanceConfig",
    "Ads6SamInsConfig",
    "Ads6SamSensorConfig",
    "Ads6SamSourceController",
    "ads6_sam_aerodynamic_derivatives",
    "lower_ads6_sam_controller_actor",
]
