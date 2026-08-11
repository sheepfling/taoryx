"""Source-compatible ROCKET6G aggregate reaction-control-system wrench model."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .input_ast import CadacInputCase, CadacModel, CadacVehicleBlock
from .input_parser import parse_cadac_input_file

AGRAV = 9.80675445
RAD_TO_DEG = 57.2957795130823


class Rocket6gRcsSourceError(ValueError):
    """Raised when one ROCKET6G source case cannot lower to the RCS contract."""


####


class Rocket6gRcsConfig(CadacModel):
    """Source RCS configuration that maps attitude/acceleration errors to aggregate wrench."""

    moment_mode: int
    force_mode: int = 0
    dead_zone: float = Field(ge=0.0)
    hysteresis: float = Field(ge=0.0)
    time_slope_s: float = Field(ge=0.0)
    roll_moment_limit_nm: float = Field(ge=0.0)
    pitch_moment_limit_nm: float = Field(ge=0.0)
    yaw_moment_limit_nm: float = Field(ge=0.0)
    proportional_damping: float = Field(ge=0.0)
    proportional_frequency_rad_s: float = Field(ge=0.0)
    acceleration_gain_n_per_mps2: float = Field(ge=0.0)
    side_force_limit_n: float = Field(ge=0.0)
    roll_command_deg: float = 0.0
    pitch_command_deg: float = 0.0
    yaw_command_deg: float = 0.0

    @property
    def moment_type(self) -> int:
        return self.moment_mode // 10

    ####

    @property
    def control_mode(self) -> int:
        return self.moment_mode % 10

    ####

    @model_validator(mode="after")
    def validate_modes(self) -> "Rocket6gRcsConfig":
        if self.moment_type not in {0, 1, 2}:
            raise ValueError("ROCKET6G RCS moment type must be 0, 1, or 2")
        ####
        if self.control_mode not in {0, 1, 2, 3}:
            raise ValueError("ROCKET6G RCS control mode must be 0, 1, 2, or 3")
        ####
        if self.force_mode not in {0, 1, 2}:
            raise ValueError("ROCKET6G RCS force mode must be 0, 1, or 2")
        ####
        if self.moment_type == 1 and (self.proportional_damping <= 0.0 or self.proportional_frequency_rad_s <= 0.0):
            raise ValueError("proportional ROCKET6G RCS moments require positive damping and natural frequency")
        ####
        return self

    ####


####


class Rocket6gRcsSourceDefinition(CadacModel):
    """One source-lowered RCS subsystem bound to a ROCKET6G HYPER6 case."""

    source_name: str
    source_line: int = Field(ge=1)
    source_model: str = "HYPER6"
    taoryx_tier: str = "rigid_body_6dof_direct_wrench"
    control_realization: str = "direct_wrench"
    config: Rocket6gRcsConfig
    claim_boundary: str = (
        "Aggregate CADAC RCS force/moment realization only. Individual thruster geometry/allocation, full vehicle dynamics, "
        "staging, navigation, propulsion, TVC, and trajectory propagation are outside this subsystem claim."
    )


####


class Rocket6gRcsRuntimeInput(CadacModel):
    """Current source signals consumed by one RCS module execution."""

    inertia_diagonal_kgm2: tuple[float, float, float]
    body_rates_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    geodetic_angles_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    incidence_deg: tuple[float, float] = (0.0, 0.0)
    incidence_commands_deg: tuple[float, float] = (0.0, 0.0)
    specific_force_body_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration_commands_g: tuple[float, float] = (0.0, 0.0)

    @model_validator(mode="after")
    def validate_finite_input(self) -> "Rocket6gRcsRuntimeInput":
        groups = (
            self.inertia_diagonal_kgm2,
            self.body_rates_rad_s,
            self.geodetic_angles_deg,
            self.thrust_vector_unit_body,
            self.incidence_deg,
            self.incidence_commands_deg,
            self.specific_force_body_mps2,
            self.acceleration_commands_g,
        )
        if any(not math.isfinite(float(value)) for group in groups for value in group):
            raise ValueError("ROCKET6G RCS runtime input must contain finite values")
        ####
        if any(value <= 0.0 for value in self.inertia_diagonal_kgm2):
            raise ValueError("ROCKET6G RCS inertia diagonal must be positive")
        ####
        return self

    ####


####


class Rocket6gRcsState(CadacModel):
    """Saved Schmitt-trigger state carried between source RCS executions."""

    roll_saved_error: float = 0.0
    pitch_saved_error: float = 0.0
    yaw_saved_error: float = 0.0
    right_saved_error: float = 0.0
    down_saved_error: float = 0.0
    roll_output: int = 0
    pitch_output: int = 0
    yaw_output: int = 0
    right_output: int = 0
    down_output: int = 0
    roll_switch_count: int = Field(default=0, ge=0)
    pitch_switch_count: int = Field(default=0, ge=0)
    yaw_switch_count: int = Field(default=0, ge=0)
    right_switch_count: int = Field(default=0, ge=0)
    down_switch_count: int = Field(default=0, ge=0)

    @model_validator(mode="after")
    def validate_trigger_outputs(self) -> "Rocket6gRcsState":
        if any(
            value not in {-1, 0, 1}
            for value in (
                self.roll_output,
                self.pitch_output,
                self.yaw_output,
                self.right_output,
                self.down_output,
            )
        ):
            raise ValueError("ROCKET6G RCS Schmitt outputs must be -1, 0, or 1")
        ####
        return self

    ####


####


class Rocket6gRcsStep(CadacModel):
    """Aggregate body force/moment and saved trigger state from one source RCS pass."""

    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    errors: tuple[float, float, float, float, float]
    state: Rocket6gRcsState


####


def load_rocket6g_rcs_source_definition(path: str | Path) -> Rocket6gRcsSourceDefinition:
    """Parse one ROCKET6G input case and lower the HYPER6 aggregate RCS configuration."""

    case = parse_cadac_input_file(path)
    return lower_rocket6g_rcs_source_case(case)


####


def lower_rocket6g_rcs_source_case(case: CadacInputCase) -> Rocket6gRcsSourceDefinition:
    """Lower only the source RCS parameter surface without claiming a full ROCKET6G runtime."""

    vehicles = case.vehicles_named("HYPER6")
    if len(vehicles) != 1:
        raise Rocket6gRcsSourceError(f"ROCKET6G RCS lowering requires exactly one HYPER6 vehicle; found {len(vehicles)}")
    ####
    if not any(module.name.casefold() == "rcs" for module in case.modules):
        raise Rocket6gRcsSourceError("ROCKET6G source case does not declare the rcs module")
    ####
    vehicle = vehicles[0]
    config = Rocket6gRcsConfig(
        moment_mode=_integer(vehicle, "mrcs_moment", 0),
        force_mode=_integer(vehicle, "mrcs_force", 0),
        dead_zone=_number(vehicle, "dead_zone", 0.0),
        hysteresis=_number(vehicle, "hysteresis", 0.0),
        time_slope_s=_number(vehicle, "rcs_tau", 0.0),
        roll_moment_limit_nm=_number(vehicle, "roll_mom_max", 0.0),
        pitch_moment_limit_nm=_number(vehicle, "pitch_mom_max", 0.0),
        yaw_moment_limit_nm=_number(vehicle, "yaw_mom_max", 0.0),
        proportional_damping=_number(vehicle, "rcs_zeta", 0.0),
        proportional_frequency_rad_s=_number(vehicle, "rcs_freq", 0.0),
        acceleration_gain_n_per_mps2=_number(vehicle, "acc_gain", 0.0),
        side_force_limit_n=_number(vehicle, "side_force_max", 0.0),
        roll_command_deg=_number(vehicle, "phibdcomx", 0.0),
        pitch_command_deg=_number(vehicle, "thtbdcomx", 0.0),
        yaw_command_deg=_number(vehicle, "psibdcomx", 0.0),
    )
    return Rocket6gRcsSourceDefinition(
        source_name=case.source_name,
        source_line=vehicle.source_line,
        config=config,
    )


####


def rocket6g_rcs_step(
    config: Rocket6gRcsConfig,
    runtime: Rocket6gRcsRuntimeInput,
    state: Rocket6gRcsState | None = None,
) -> Rocket6gRcsStep:
    """Execute the CADAC aggregate RCS module for one source epoch."""

    previous = state or Rocket6gRcsState()
    force = [0.0, 0.0, 0.0]
    moment = [0.0, 0.0, 0.0]
    p_rate, q_rate, r_rate = runtime.body_rates_rad_s
    roll_deg, pitch_deg, yaw_deg = runtime.geodetic_angles_deg
    alpha_deg, beta_deg = runtime.incidence_deg
    alpha_command_deg, beta_command_deg = runtime.incidence_commands_deg
    ay_command_g, az_command_g = runtime.acceleration_commands_g
    vector = runtime.thrust_vector_unit_body
    inertia = runtime.inertia_diagonal_kgm2

    e_roll = 0.0
    e_pitch = 0.0
    e_yaw = 0.0
    e_right = 0.0
    e_down = 0.0

    roll_output = previous.roll_output
    pitch_output = previous.pitch_output
    yaw_output = previous.yaw_output
    right_output = previous.right_output
    down_output = previous.down_output
    roll_count = previous.roll_switch_count
    pitch_count = previous.pitch_switch_count
    yaw_count = previous.yaw_switch_count
    right_count = previous.right_switch_count
    down_count = previous.down_switch_count
    roll_saved = previous.roll_saved_error
    pitch_saved = previous.pitch_saved_error
    yaw_saved = previous.yaw_saved_error
    right_saved = previous.right_saved_error
    down_saved = previous.down_saved_error

    if config.moment_type == 1:
        roll_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * inertia[0]
        pitch_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * inertia[1]
        yaw_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * inertia[2]
        position_gain = config.proportional_frequency_rad_s / (2.0 * config.proportional_damping)
        e_roll = roll_gain * (position_gain * (config.roll_command_deg - roll_deg) - p_rate)
        moment[0] = rocket6g_rcs_proportional(e_roll, config.roll_moment_limit_nm)
        if config.control_mode == 1:
            e_pitch = pitch_gain * (position_gain * (config.pitch_command_deg - pitch_deg) - q_rate)
            e_yaw = yaw_gain * (position_gain * (config.yaw_command_deg - yaw_deg) - r_rate)
        elif config.control_mode == 2:
            e_pitch = pitch_gain * (position_gain * (-vector[2]) * RAD_TO_DEG - q_rate)
            e_yaw = yaw_gain * (position_gain * vector[1] * RAD_TO_DEG - r_rate)
        ####
        moment[1] = rocket6g_rcs_proportional(e_pitch, config.pitch_moment_limit_nm)
        moment[2] = rocket6g_rcs_proportional(e_yaw, config.yaw_moment_limit_nm)
    ####

    if config.force_mode == 1:
        e_right = config.acceleration_gain_n_per_mps2 * (ay_command_g * AGRAV - runtime.specific_force_body_mps2[1])
        e_down = config.acceleration_gain_n_per_mps2 * (az_command_g * AGRAV - runtime.specific_force_body_mps2[2])
        force[1] = rocket6g_rcs_proportional(e_right, config.side_force_limit_n)
        force[2] = rocket6g_rcs_proportional(e_down, config.side_force_limit_n)
    ####

    if config.moment_type == 2:
        e_roll = config.roll_command_deg - (config.time_slope_s * p_rate + roll_deg)
        new_roll_output = rocket6g_rcs_schmitt(e_roll, roll_saved, config.dead_zone, config.hysteresis)
        roll_count += int(new_roll_output != roll_output)
        roll_output = new_roll_output
        roll_saved = e_roll
        if config.control_mode == 1:
            e_pitch = config.pitch_command_deg - (config.time_slope_s * q_rate + pitch_deg)
            e_yaw = config.yaw_command_deg - (config.time_slope_s * r_rate + yaw_deg)
        ####
        if config.control_mode == 3:
            e_pitch = alpha_command_deg - (config.time_slope_s * q_rate + alpha_deg)
            e_yaw = -beta_command_deg - (config.time_slope_s * r_rate - beta_deg)
        elif config.control_mode == 2:
            e_pitch = -config.time_slope_s * q_rate - vector[2] * RAD_TO_DEG
            e_yaw = -config.time_slope_s * r_rate + vector[1] * RAD_TO_DEG
        ####
        new_pitch_output = rocket6g_rcs_schmitt(e_pitch, pitch_saved, config.dead_zone, config.hysteresis)
        pitch_count += int(new_pitch_output != pitch_output)
        pitch_output = new_pitch_output
        pitch_saved = e_pitch
        new_yaw_output = rocket6g_rcs_schmitt(e_yaw, yaw_saved, config.dead_zone, config.hysteresis)
        yaw_count += int(new_yaw_output != yaw_output)
        yaw_output = new_yaw_output
        yaw_saved = e_yaw
        moment = [
            roll_output * config.roll_moment_limit_nm,
            pitch_output * config.pitch_moment_limit_nm,
            yaw_output * config.yaw_moment_limit_nm,
        ]
    ####

    if config.force_mode == 2:
        e_right = config.acceleration_gain_n_per_mps2 * (ay_command_g * AGRAV - runtime.specific_force_body_mps2[1])
        new_right_output = rocket6g_rcs_schmitt(e_right, right_saved, config.dead_zone, config.hysteresis)
        right_count += int(new_right_output != right_output)
        right_output = new_right_output
        right_saved = e_right
        e_down = config.acceleration_gain_n_per_mps2 * (az_command_g * AGRAV - runtime.specific_force_body_mps2[2])
        new_down_output = rocket6g_rcs_schmitt(e_down, down_saved, config.dead_zone, config.hysteresis)
        down_count += int(new_down_output != down_output)
        down_output = new_down_output
        down_saved = e_down
        force = [0.0, right_output * config.side_force_limit_n, down_output * config.side_force_limit_n]
    ####

    next_state = Rocket6gRcsState(
        roll_saved_error=roll_saved,
        pitch_saved_error=pitch_saved,
        yaw_saved_error=yaw_saved,
        right_saved_error=right_saved,
        down_saved_error=down_saved,
        roll_output=roll_output,
        pitch_output=pitch_output,
        yaw_output=yaw_output,
        right_output=right_output,
        down_output=down_output,
        roll_switch_count=roll_count,
        pitch_switch_count=pitch_count,
        yaw_switch_count=yaw_count,
        right_switch_count=right_count,
        down_switch_count=down_count,
    )
    return Rocket6gRcsStep(
        force_body_n=tuple(force),
        moment_body_nm=tuple(moment),
        errors=(e_roll, e_pitch, e_yaw, e_right, e_down),
        state=next_state,
    )


####


def rocket6g_rcs_proportional(value: float, limit: float) -> float:
    """Apply the source symmetric magnitude limiter used by proportional RCS channels."""

    if limit < 0.0 or not math.isfinite(value) or not math.isfinite(limit):
        raise ValueError("ROCKET6G RCS proportional input and limit must be finite with a nonnegative limit")
    ####
    if abs(value) > limit:
        return math.copysign(limit, value)
    ####
    return value


####


def rocket6g_rcs_schmitt(input_new: float, previous_input: float, dead_zone: float, hysteresis: float) -> int:
    """Replicate the CADAC Schmitt relay, including its previous-input comparison semantics."""

    values = (input_new, previous_input, dead_zone, hysteresis)
    if any(not math.isfinite(value) for value in values) or dead_zone < 0.0 or hysteresis < 0.0:
        raise ValueError("ROCKET6G RCS Schmitt inputs must be finite and dead-zone/hysteresis nonnegative")
    ####
    trend = -1 if input_new - previous_input < 0.0 else 1
    side = -1 if previous_input < 0.0 else 1
    trigger = (dead_zone * side + hysteresis * trend) / 2.0
    if previous_input >= trigger and side == 1:
        return 1
    ####
    if previous_input <= trigger and side == -1:
        return -1
    ####
    return 0


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Rocket6gRcsSourceError(f"ROCKET6G RCS source vehicle is missing parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Rocket6gRcsSourceError(f"ROCKET6G RCS parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise Rocket6gRcsSourceError(f"ROCKET6G RCS parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


__all__ = [
    "Rocket6gRcsConfig",
    "Rocket6gRcsRuntimeInput",
    "Rocket6gRcsSourceDefinition",
    "Rocket6gRcsSourceError",
    "Rocket6gRcsState",
    "Rocket6gRcsStep",
    "load_rocket6g_rcs_source_definition",
    "lower_rocket6g_rcs_source_case",
    "rocket6g_rcs_proportional",
    "rocket6g_rcs_schmitt",
    "rocket6g_rcs_step",
]
