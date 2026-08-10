"""One source of truth for native telemetry normalization and advertisement."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Literal

from .configuration_contract import (
    ConfigurationPeriodicity,
    TrajectoryOutputChannelMetadata,
    TrajectoryReferenceFrameMetadata,
    ValuePresentationMetadata,
)

ChannelClass = Literal["core_state", "telemetry"]


@dataclass(frozen=True, slots=True)
class NativeOutputBinding:
    """Canonical channel and exact source path for one native output family."""

    id: str
    label: str
    description: str
    channel_class: ChannelClass
    quantity: str
    unit: str | None
    frame: str | None
    source_paths: tuple[str, ...]
    fidelities: tuple[str, ...]
    mission_templates: tuple[str, ...] = ()
    telemetry_group: str | None = None
    interpolation: Literal["linear", "step", "periodic", "slerp", "event"] = "linear"


def _b(
    identifier: str,
    label: str,
    channel_class: ChannelClass,
    quantity: str,
    unit: str | None,
    frame: str | None,
    source_paths: tuple[str, ...],
    fidelities: tuple[str, ...],
    *,
    group: str | None = None,
    missions: tuple[str, ...] = (),
    interpolation: Literal["linear", "step", "periodic", "slerp", "event"] = "linear",
) -> NativeOutputBinding:
    return NativeOutputBinding(
        id=identifier,
        label=label,
        description=f"Normalized {label.casefold()} emitted by the exact native execution binding.",
        channel_class=channel_class,
        quantity=quantity,
        unit=unit,
        frame=frame,
        source_paths=source_paths,
        fidelities=fidelities,
        mission_templates=missions,
        # Core state is intentionally not a telemetry group.  Some source maps
        # retain ``group="state"`` for authoring readability, but the public
        # configuration contract reserves groups for optional telemetry only.
        telemetry_group=group if channel_class == "telemetry" else None,
        interpolation=interpolation,
    )
    ####


_REDUCED = ("point_mass_3dof", "pseudo_6dof")
_DIRECT = ("rigid_body_6dof_direct_wrench",)
_SURFACE = ("rigid_body_6dof_surface_allocated",)
_FIXED_WING_RACETRACK = (*_REDUCED, *_DIRECT)
_RACETRACK = ("powered_fixed_wing_racetrack_v1",)
_X8_LOCAL_PHYSICAL_SCREEN = (
    "x8_local_physical_surface_lqr_screen_v1",
    "x8_local_physical_surface_lqi_screen_v1",
    "x8_local_physical_surface_lqi_long_recovery_screen_v1",
)
_B747_LOCAL_PHYSICAL_SCREEN = (
    "b747_condition3_local_physical_surface_lqr_screen_v1",
    "b747_condition3_local_physical_surface_lqi_screen_v1",
)
_A320_LOCAL_NATIVE_LQI_SCREEN = ("a320_local_native_coordinate_lqi_screen_v1",)
_F16_LOCAL_PHYSICAL_CONTROL_SCREEN = ("f16_local_physical_control_screen_v1",)
_F16_LOCAL_PHYSICAL_LQI_SCREEN = ("f16_local_physical_surface_lqi_screen_v1",)
_F16_LOCAL_PHYSICAL_LQR_SCHEDULE_INTERIOR_SCREEN = (
    "f16_local_physical_surface_lqr_schedule_interior_screen_v1",
)
_F16_LOCAL_PHYSICAL_LQI_SCHEDULE_INTERIOR_SCREEN = (
    "f16_local_physical_surface_lqi_schedule_interior_screen_v1",
)
_F16_LOCAL_PHYSICAL_LQR_SCHEDULE_TRANSITION_SCREEN = (
    "f16_local_physical_surface_lqr_schedule_transition_screen_v1",
)
_F16_LOCAL_PHYSICAL_SCREENS = (
    *_F16_LOCAL_PHYSICAL_CONTROL_SCREEN,
    *_F16_LOCAL_PHYSICAL_LQI_SCREEN,
    *_F16_LOCAL_PHYSICAL_LQR_SCHEDULE_INTERIOR_SCREEN,
    *_F16_LOCAL_PHYSICAL_LQI_SCHEDULE_INTERIOR_SCREEN,
    *_F16_LOCAL_PHYSICAL_LQR_SCHEDULE_TRANSITION_SCREEN,
)
_HUMMINGBIRD_LOCAL_DIRECT_SCREEN = ("hummingbird_local_direct_wrench_screen_v1",)
_HUMMINGBIRD_LOCAL_PHYSICAL_SCREEN = (
    "hummingbird_local_individual_rotor_lqi_screen_v1",
    "hummingbird_local_horizontal_translation_lqi_screen_v1",
)
_HUMMINGBIRD_LOCAL_VERTICAL_SCREEN = ("hummingbird_local_vertical_translation_lqi_screen_v1",)
_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS = (
    *_HUMMINGBIRD_LOCAL_PHYSICAL_SCREEN,
    *_HUMMINGBIRD_LOCAL_VERTICAL_SCREEN,
)
_X15_LOCAL_DIRECT_WRENCH_SCREENS = (
    "x15_local_direct_wrench_screen_v1",
    "x15_local_direct_wrench_lqi_screen_v1",
)
_X15_SOURCE_SURFACE_AUTHORITY_SCREEN = ("x15_source_surface_authority_screen_v1",)
_X15_SOURCE_SURFACE_LQI_SCREEN = ("x15_source_surface_attitude_rate_lqi_screen_v1",)
_X15_SOURCE_SURFACE_SCREENS = (
    *_X15_SOURCE_SURFACE_AUTHORITY_SCREEN,
    *_X15_SOURCE_SURFACE_LQI_SCREEN,
)
_HL20_LOCAL_DIRECT_WRENCH_SCREENS = (
    "hl20_local_direct_wrench_screen_v1",
    "hl20_local_direct_wrench_lqi_screen_v1",
)
_HL20_SOURCE_SURFACE_AUTHORITY_SCREEN = ("hl20_source_surface_pitch_authority_screen_v1",)
_HL20_SOURCE_SURFACE_LQI_SCREEN = ("hl20_source_surface_attitude_rate_lqi_screen_v1",)
_HL20_SOURCE_SURFACE_SCREENS = (
    *_HL20_SOURCE_SURFACE_AUTHORITY_SCREEN,
    *_HL20_SOURCE_SURFACE_LQI_SCREEN,
)

_GEODETIC_CORE = (
    _b("position.geodetic.altitude", "Geodetic Altitude", "core_state", "length", "m", "geodetic", ("altitude_m",), _FIXED_WING_RACETRACK),
    _b("position.geodetic.latitude", "Geodetic Latitude", "core_state", "angle", "deg", "geodetic", ("latitude_deg",), _FIXED_WING_RACETRACK),
    _b("position.geodetic.longitude", "Geodetic Longitude", "core_state", "angle", "deg", "geodetic", ("longitude_deg",), _FIXED_WING_RACETRACK, interpolation="periodic"),
    _b("velocity.speed", "Speed", "core_state", "speed", "m/s", None, ("speed_m_s",), _FIXED_WING_RACETRACK),
)


def _local_core(fidelities: tuple[str, ...], *, frame: str = "NED", missions: tuple[str, ...] = ()) -> tuple[NativeOutputBinding, ...]:
    return (
        _b("position.local.x", "Local X Position", "core_state", "length", "m", frame, ("north_m", "position_m[0]"), fidelities, missions=missions),
        _b("position.local.y", "Local Y Position", "core_state", "length", "m", frame, ("east_m", "position_m[1]"), fidelities, missions=missions),
        _b("position.local.z", "Local Z Position", "core_state", "length", "m", frame, ("altitude_m", "position_m[2]"), fidelities, missions=missions),
        _b("velocity.speed", "Speed", "core_state", "speed", "m/s", None, ("speed_m_s", "velocity_m_s|norm"), fidelities, missions=missions),
    )
    ####


def _local_vector_core(fidelities: tuple[str, ...], *, missions: tuple[str, ...] = ()) -> tuple[NativeOutputBinding, ...]:
    return (
        _b("position.local.x", "Local X Position", "core_state", "length", "m", "local_cartesian", ("position_m[0]",), fidelities, missions=missions),
        _b("position.local.y", "Local Y Position", "core_state", "length", "m", "local_cartesian", ("position_m[1]",), fidelities, missions=missions),
        _b("position.local.z", "Local Z Position", "core_state", "length", "m", "local_cartesian", ("position_m[2]",), fidelities, missions=missions),
        _b("velocity.local.x", "Local X Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[0]",), fidelities, missions=missions),
        _b("velocity.local.y", "Local Y Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[1]",), fidelities, missions=missions),
        _b("velocity.local.z", "Local Z Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[2]",), fidelities, missions=missions),
    )
    ####


def _direct_wrench_core(
    *missions: str,
    source_surface_lqi_missions: tuple[str, ...] = (),
) -> tuple[NativeOutputBinding, ...]:
    """Bind direct-wrench state, optionally sharing exact body-rate channels with surface LQI."""

    return tuple(
        _b(
            identifier,
            label,
            "core_state",
            quantity,
            unit,
            "body",
            ((path, path.removeprefix("state.")) if identifier.startswith("angular_rate.") and source_surface_lqi_missions else (path,)),
            (*_DIRECT, *(_SURFACE if identifier.startswith("angular_rate.") and source_surface_lqi_missions else ())),
            missions=(*missions, *(source_surface_lqi_missions if identifier.startswith("angular_rate.") else ())),
        )
        for identifier, label, quantity, unit, path in (
            ("velocity.body.x", "Body X Velocity", "speed", "m/s", "state.u_m_s"),
            ("velocity.body.y", "Body Y Velocity", "speed", "m/s", "state.v_m_s"),
            ("velocity.body.z", "Body Z Velocity", "speed", "m/s", "state.w_m_s"),
            ("angular_rate.body.x", "Body Roll Rate", "angular_rate", "rad/s", "state.p_rad_s"),
            ("angular_rate.body.y", "Body Pitch Rate", "angular_rate", "rad/s", "state.q_rad_s"),
            ("angular_rate.body.z", "Body Yaw Rate", "angular_rate", "rad/s", "state.r_rad_s"),
        )
    )
    ####


def _source_surface_attitude_error_lqi_core(*missions: str) -> tuple[NativeOutputBinding, ...]:
    """Expose the shared propagated LQI attitude-error state."""

    return tuple(
        _b(identifier, label, "core_state", quantity, unit, "body", (path,), _SURFACE, missions=missions)
        for identifier, label, quantity, unit, path in (
            ("attitude.local.roll_error", "Local Roll Error", "angle", "rad", "roll_error_rad"),
            ("attitude.local.pitch_error", "Local Pitch Error", "angle", "rad", "pitch_error_rad"),
            ("attitude.local.yaw_error", "Local Yaw Error", "angle", "rad", "yaw_error_rad"),
        )
    )
    ####


def _direct_wrench_telemetry(*missions: str) -> tuple[NativeOutputBinding, ...]:
    axes = ("x", "y", "z")
    result: list[NativeOutputBinding] = []
    for state in ("requested", "achieved"):
        source_state = f"{state}_wrench"
        for vector, quantity, unit, suffix in (("force", "force", "N", "n"), ("moment", "moment", "N*m", "nm")):
            for axis in axes:
                result.append(
                    _b(
                        f"control.wrench.{state}.{vector}.{axis}",
                        f"{state.title()} {vector.title()} {axis.upper()}",
                        "telemetry",
                        quantity,
                        unit,
                        "body",
                        (f"wrench.{source_state}.{vector}_{axis}_{suffix}",),
                        _DIRECT,
                        group="controls",
                        missions=missions,
                    )
                )
    for vector, quantity, unit, suffix in (("force", "force", "N", "n"), ("moment", "moment", "N*m", "nm")):
        for axis in axes:
            result.append(
                _b(
                    f"control.wrench.residual.{vector}.{axis}",
                    f"Residual {vector.title()} {axis.upper()}",
                    "telemetry",
                    quantity,
                    unit,
                    "body",
                    (f"wrench.residual_wrench.{vector}_{axis}_{suffix}",),
                    _DIRECT,
                    group="diagnostics",
                    missions=missions,
                )
            )
    result.extend(
        (
            _b("control.wrench.residual_norm", "Wrench Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("wrench.residual_norm",), _DIRECT, group="diagnostics", missions=missions),
            _b("control.feedback_norm", "Feedback Error Norm", "telemetry", "mixed_state_norm", None, None, ("feedback_norm",), _DIRECT, group="diagnostics", missions=missions),
        )
    )
    return tuple(result)
    ####


def _hummingbird_wrench_telemetry() -> tuple[NativeOutputBinding, ...]:
    """Bind direct-wrench and individual-rotor screen telemetry without duplicate public IDs."""

    axes = ("x", "y", "z")
    result: list[NativeOutputBinding] = []
    for state in ("requested", "achieved"):
        source_state = f"{state}_wrench"
        for vector, quantity, unit, suffix in (("force", "force", "N", "n"), ("moment", "moment", "N*m", "nm")):
            for axis in axes:
                physical_moment = vector == "moment"
                physical_vertical_force = vector == "force"
                result.append(
                    _b(
                        f"control.wrench.{state}.{vector}.{axis}",
                        f"{state.title()} {vector.title()} {axis.upper()}",
                        "telemetry",
                        quantity,
                        unit,
                        "body",
                        (
                            f"wrench.{source_state}.{vector}_{axis}_{suffix}",
                            *((f"{state}_{vector}_{axis}_{suffix}",) if physical_moment or physical_vertical_force else ()),
                        ),
                        (*_DIRECT, *(_SURFACE if physical_moment or physical_vertical_force else ())),
                        group="controls",
                        missions=(
                            *_HUMMINGBIRD_LOCAL_DIRECT_SCREEN,
                            *(_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS if physical_moment else _HUMMINGBIRD_LOCAL_VERTICAL_SCREEN if physical_vertical_force else ()),
                        ),
                    )
                )
    for vector, quantity, unit, suffix in (("force", "force", "N", "n"),):
        for axis in axes:
            result.append(
                _b(
                    f"control.wrench.residual.{vector}.{axis}",
                    f"Residual {vector.title()} {axis.upper()}",
                    "telemetry",
                    quantity,
                    unit,
                    "body",
                    (f"wrench.residual_wrench.{vector}_{axis}_{suffix}", f"residual_{vector}_{axis}_{suffix}"),
                    (*_DIRECT, *_SURFACE),
                    group="diagnostics",
                    missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_VERTICAL_SCREEN),
                )
            )
    result.extend(
        (
            _b("control.wrench.residual_norm", "Wrench Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("wrench.residual_norm",), _DIRECT, group="diagnostics", missions=_HUMMINGBIRD_LOCAL_DIRECT_SCREEN),
            _b("control.feedback_norm", "Feedback Error Norm", "telemetry", "mixed_state_norm", None, None, ("feedback_norm",), _DIRECT, group="diagnostics", missions=_HUMMINGBIRD_LOCAL_DIRECT_SCREEN),
        )
    )
    return tuple(result)
    ####


_MODEL_BINDINGS: dict[str, tuple[NativeOutputBinding, ...]] = {
    "skywalker_x8": (
        *_GEODETIC_CORE,
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle",), _REDUCED, group="controls"),
        _b("actuator.elevon.collective", "Collective Elevon", "telemetry", "angle", "deg", "body", ("collective-elevon-deg",), _REDUCED, group="actuators"),
        _b("actuator.elevon.differential", "Differential Elevon", "telemetry", "angle", "deg", "body", ("differential-elevon-deg",), _REDUCED, group="actuators"),
        _b("attitude.local.roll_error", "Local Roll Error", "core_state", "angle", "rad", "body", ("roll_error_rad",), _SURFACE, group="state", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("attitude.local.pitch_error", "Local Pitch Error", "core_state", "angle", "rad", "body", ("pitch_error_rad",), _SURFACE, group="state", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("angular_rate.body.x", "Body Roll Rate", "core_state", "angular_rate", "rad/s", "body", ("p_rad_s",), _SURFACE, group="state", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("angular_rate.body.y", "Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("q_rad_s",), _SURFACE, group="state", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("mass.total", "Source-Trim Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _SURFACE, group="resources", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.requested.moment.x", "Requested Roll Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_x_nm",), _SURFACE, group="controls", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.requested.moment.y", "Requested Pitch Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_y_nm",), _SURFACE, group="controls", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.achieved.moment.x", "Achieved Roll Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_x_nm",), _SURFACE, group="controls", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.achieved.moment.y", "Achieved Pitch Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_y_nm",), _SURFACE, group="controls", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.x", "Residual Roll Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_x_nm",), _SURFACE, group="diagnostics", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.y", "Residual Pitch Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_y_nm",), _SURFACE, group="diagnostics", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.z", "Residual Yaw Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_z_nm",), _SURFACE, group="diagnostics", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("diagnostics.allocation_residual_norm", "Allocation Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("allocation_controlled_residual_norm",), _SURFACE, group="diagnostics", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("diagnostics.saturation_count", "Saturation Count", "telemetry", "count", None, None, ("saturation_count",), _SURFACE, group="diagnostics", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("actuator.elevon.collective.actual", "Actual Collective Elevon", "telemetry", "angle", "deg", None, ("collective_elevon_deg",), _SURFACE, group="actuators", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("actuator.elevon.differential.actual", "Actual Differential Elevon", "telemetry", "angle", "deg", None, ("differential_elevon_deg",), _SURFACE, group="actuators", missions=_X8_LOCAL_PHYSICAL_SCREEN),
        _b("control.throttle.actual", "Actual Throttle", "telemetry", "dimensionless", "1", None, ("throttle_fraction",), _SURFACE, group="actuators", missions=_X8_LOCAL_PHYSICAL_SCREEN),
    ),
    "b747": (
        *_GEODETIC_CORE,
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle",), _REDUCED, group="controls"),
        _b("actuator.elevator", "Elevator Position", "telemetry", "angle", "deg", "body", ("elevator-deg",), _REDUCED, group="actuators"),
        _b("attitude.local.roll_error", "Local Roll Error", "core_state", "angle", "rad", "body", ("roll_error_rad",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("attitude.local.pitch_error", "Local Pitch Error", "core_state", "angle", "rad", "body", ("pitch_error_rad",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("attitude.local.yaw_error", "Local Yaw Error", "core_state", "angle", "rad", "body", ("yaw_error_rad",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("angular_rate.body.x", "Body Roll Rate", "core_state", "angular_rate", "rad/s", "body", ("p_rad_s",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("angular_rate.body.y", "Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("q_rad_s",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("angular_rate.body.z", "Body Yaw Rate", "core_state", "angular_rate", "rad/s", "body", ("r_rad_s",), _SURFACE, group="state", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("mass.total", "Condition-3 Source Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _SURFACE, group="resources", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.requested.moment.x", "Requested Roll Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_x_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.requested.moment.y", "Requested Pitch Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_y_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.requested.moment.z", "Requested Yaw Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_z_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.achieved.moment.x", "Achieved Roll Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_x_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.achieved.moment.y", "Achieved Pitch Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_y_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.achieved.moment.z", "Achieved Yaw Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_z_nm",), _SURFACE, group="controls", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.x", "Residual Roll Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_x_nm",), _SURFACE, group="diagnostics", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.y", "Residual Pitch Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_y_nm",), _SURFACE, group="diagnostics", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.wrench.residual.moment.z", "Residual Yaw Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_z_nm",), _SURFACE, group="diagnostics", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("diagnostics.allocation_residual_norm", "Allocation Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("allocation_controlled_residual_norm",), _SURFACE, group="diagnostics", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("diagnostics.saturation_count", "Saturation Count", "telemetry", "count", None, None, ("saturation_count",), _SURFACE, group="diagnostics", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("actuator.elevator.actual", "Actual Elevator", "telemetry", "angle", "deg", None, ("elevator_deg",), _SURFACE, group="actuators", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("actuator.aileron.actual", "Actual Aileron", "telemetry", "angle", "deg", None, ("aileron_deg",), _SURFACE, group="actuators", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("actuator.rudder.actual", "Actual Rudder", "telemetry", "angle", "deg", None, ("rudder_deg",), _SURFACE, group="actuators", missions=_B747_LOCAL_PHYSICAL_SCREEN),
        _b("control.throttle.actual", "Actual Throttle", "telemetry", "dimensionless", "1", None, ("throttle_fraction",), _SURFACE, group="actuators", missions=_B747_LOCAL_PHYSICAL_SCREEN),
    ),
    "a320_openap_3dof": (
        *_local_core(_REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources"),
        _b("mass.fuel_flow", "Fuel Flow", "telemetry", "mass_rate", "kg/s", None, ("fuel_flow_kg_s",), _REDUCED, group="resources"),
        _b("aerodynamics.dynamic_pressure", "Dynamic Pressure", "telemetry", "pressure", "Pa", None, ("dynamic_pressure_pa",), _REDUCED, group="aerodynamics"),
        _b("propulsion.thrust", "Thrust", "telemetry", "force", "N", "body", ("thrust_n",), _REDUCED, group="propulsion"),
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle_ratio",), _REDUCED, group="controls"),
        _b("attitude.local.roll_error", "Local Roll Error", "core_state", "angle", "rad", "body", ("state_error.bank_angle_rad",), ("pseudo_6dof",), missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("angular_rate.body.x", "Body Roll Rate", "core_state", "angular_rate", "rad/s", "body", ("body_rate_rad_s[0]",), ("pseudo_6dof",), missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("angular_rate.body.y", "Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("body_rate_rad_s[1]",), ("pseudo_6dof",), missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("angular_rate.body.z", "Body Yaw Rate", "core_state", "angular_rate", "rad/s", "body", ("body_rate_rad_s[2]",), ("pseudo_6dof",), missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("aerodynamics.angle_of_attack", "Angle of Attack", "telemetry", "angle", "rad", "body", ("state.alpha_rad",), ("pseudo_6dof",), group="aerodynamics", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("aerodynamics.sideslip", "Sideslip", "telemetry", "angle", "rad", "body", ("state.beta_rad",), ("pseudo_6dof",), group="aerodynamics", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.aileron.requested", "Requested Native Aileron Coordinate", "telemetry", "angle", "rad", "body", ("requested_controls.aileron_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.aileron.applied", "Applied Native Aileron Coordinate", "telemetry", "angle", "rad", "body", ("applied_controls.aileron_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.elevator.requested", "Requested Native Elevator Coordinate", "telemetry", "angle", "rad", "body", ("requested_controls.elevator_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.elevator.applied", "Applied Native Elevator Coordinate", "telemetry", "angle", "rad", "body", ("applied_controls.elevator_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.rudder.requested", "Requested Native Rudder Coordinate", "telemetry", "angle", "rad", "body", ("requested_controls.rudder_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.native.rudder.applied", "Applied Native Rudder Coordinate", "telemetry", "angle", "rad", "body", ("applied_controls.rudder_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("control.lqi.integral_error.roll", "LQI Roll Integral Error", "telemetry", "angle_time", "rad*s", "body", ("lqi_integral_error.bank_angle_rad",), ("pseudo_6dof",), group="controls", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("diagnostics.feedback_error_norm", "Native Coordinate Feedback Error Norm", "telemetry", "mixed_state_norm", None, None, ("feedback_error_norm",), ("pseudo_6dof",), group="diagnostics", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
        _b("diagnostics.saturation_count", "Native Control Saturation Count", "telemetry", "count", None, None, ("saturation_count",), ("pseudo_6dof",), group="diagnostics", missions=_A320_LOCAL_NATIVE_LQI_SCREEN),
    ),
    "f16_s119": (
        *_local_core(_REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), (*_REDUCED, *_DIRECT, *_SURFACE), group="resources"),
        _b("aerodynamics.dynamic_pressure", "Dynamic Pressure", "telemetry", "pressure", "Pa", None, ("dynamic_pressure_pa",), _REDUCED, group="aerodynamics"),
        _b("aerodynamics.angle_of_attack", "Angle of Attack", "telemetry", "angle", "deg", "body", ("aero_alpha_deg",), _REDUCED, group="aerodynamics"),
        _b("aerodynamics.sideslip", "Sideslip", "telemetry", "angle", "deg", "body", ("aero_beta_deg",), _REDUCED, group="aerodynamics"),
        _b("actuator.elevator", "Elevator Position", "telemetry", "angle", "deg", "body", ("elevator_deg",), _REDUCED, group="actuators"),
        _b("actuator.aileron", "Aileron Position", "telemetry", "angle", "deg", "body", ("aileron_deg",), _REDUCED, group="actuators"),
        _b("actuator.rudder", "Rudder Position", "telemetry", "angle", "deg", "body", ("rudder_deg",), _REDUCED, group="actuators"),
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle_fraction",), _REDUCED, group="controls"),
        _b("velocity.body.x", "Body X Velocity", "core_state", "speed", "m/s", "body", ("u_m_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("velocity.body.y", "Body Y Velocity", "core_state", "speed", "m/s", "body", ("v_m_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("velocity.body.z", "Body Z Velocity", "core_state", "speed", "m/s", "body", ("w_m_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("angular_rate.body.x", "Body Roll Rate", "core_state", "angular_rate", "rad/s", "body", ("p_rad_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("angular_rate.body.y", "Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("q_rad_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("angular_rate.body.z", "Body Yaw Rate", "core_state", "angular_rate", "rad/s", "body", ("r_rad_s",), (*_DIRECT, *_SURFACE), group="state", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.requested.force.x", "Requested Axial Force", "telemetry", "force", "N", "body", ("requested_force_x_n",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.requested.moment.x", "Requested Roll Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_x_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.requested.moment.y", "Requested Pitch Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_y_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.requested.moment.z", "Requested Yaw Moment", "telemetry", "moment", "N*m", "body", ("requested_moment_z_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.achieved.force.x", "Achieved Axial Force", "telemetry", "force", "N", "body", ("achieved_force_x_n",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.achieved.moment.x", "Achieved Roll Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_x_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.achieved.moment.y", "Achieved Pitch Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_y_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.achieved.moment.z", "Achieved Yaw Moment", "telemetry", "moment", "N*m", "body", ("achieved_moment_z_nm",), (*_DIRECT, *_SURFACE), group="controls", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.residual.force.x", "Residual Axial Force", "telemetry", "force", "N", "body", ("residual_force_x_n",), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.residual.moment.x", "Residual Roll Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_x_nm",), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.residual.moment.y", "Residual Pitch Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_y_nm",), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.wrench.residual.moment.z", "Residual Yaw Moment", "telemetry", "moment", "N*m", "body", ("residual_moment_z_nm",), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("diagnostics.allocation_residual_norm", "Allocation Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("allocation_residual_norm", "allocation_controlled_residual_norm"), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("diagnostics.saturation_count", "Saturation Count", "telemetry", "count", None, None, ("saturation_count",), (*_DIRECT, *_SURFACE), group="diagnostics", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("actuator.elevator.actual", "Actual Elevator", "telemetry", "angle", "deg", None, ("elevator_deg",), _SURFACE, group="actuators", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("actuator.aileron.actual", "Actual Aileron", "telemetry", "angle", "deg", None, ("aileron_deg",), _SURFACE, group="actuators", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("actuator.rudder.actual", "Actual Rudder", "telemetry", "angle", "deg", None, ("rudder_deg",), _SURFACE, group="actuators", missions=_F16_LOCAL_PHYSICAL_SCREENS),
        _b("control.throttle.actual", "Actual Throttle", "telemetry", "dimensionless", "1", None, ("throttle_fraction",), _SURFACE, group="actuators", missions=_F16_LOCAL_PHYSICAL_SCREENS),
    ),
    "hummingbird": (
        _b("position.local.x", "Local X Position", "core_state", "length", "m", "local_cartesian", ("position_m[0]", "position_local_m[0]"), ("pseudo_6dof", *_SURFACE)),
        _b("position.local.y", "Local Y Position", "core_state", "length", "m", "local_cartesian", ("position_m[1]", "position_local_m[1]"), ("pseudo_6dof", *_SURFACE)),
        _b("position.local.z", "Local Z Position", "core_state", "length", "m", "local_cartesian", ("position_m[2]", "position_local_m[2]"), ("pseudo_6dof", *_SURFACE)),
        _b("velocity.local.x", "Local X Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[0]", "velocity_local_m_s[0]"), ("pseudo_6dof", *_SURFACE)),
        _b("velocity.local.y", "Local Y Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[1]", "velocity_local_m_s[1]"), ("pseudo_6dof", *_SURFACE)),
        _b("velocity.local.z", "Local Z Velocity", "core_state", "speed", "m/s", "local_cartesian", ("velocity_m_s[2]", "velocity_local_m_s[2]"), ("pseudo_6dof", *_SURFACE)),
        _b("velocity.body.x", "Body X Velocity", "core_state", "speed", "m/s", "body", ("state.u_m_s", "u_m_s"), (*_DIRECT, *_SURFACE), missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("velocity.body.y", "Body Y Velocity", "core_state", "speed", "m/s", "body", ("state.v_m_s", "v_m_s"), (*_DIRECT, *_SURFACE), missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("velocity.body.z", "Body Z Velocity", "core_state", "speed", "m/s", "body", ("state.w_m_s", "w_m_s"), (*_DIRECT, *_SURFACE), missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("angular_rate.body.y", "Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("state.q_rad_s", "q_rad_s"), (*_DIRECT, *_SURFACE), missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("angular_rate.body.z", "Body Yaw Rate", "core_state", "angular_rate", "rad/s", "body", ("state.r_rad_s", "r_rad_s"), (*_DIRECT, *_SURFACE), missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("resources.battery_fraction", "Battery Fraction", "telemetry", "dimensionless", "1", None, ("battery_fraction",), ("pseudo_6dof",), group="resources"),
        _b("mass.total", "Source-Hover Mass", "telemetry", "mass", "kg", None, ("mass_kg",), (*_DIRECT, *_SURFACE), group="resources", missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("control.thrust.commanded_ratio", "Commanded Thrust Ratio", "telemetry", "dimensionless", "1", None, ("commanded_thrust_ratio",), ("pseudo_6dof",), group="controls"),
        _b("propulsion.thrust.realized", "Realized Thrust", "telemetry", "force", "N", "body", ("achieved_thrust_n",), ("pseudo_6dof",), group="propulsion"),
        _b("attitude.commanded.roll", "Commanded Roll", "telemetry", "angle", "rad", "body", ("commanded_attitude_rad[0]",), ("pseudo_6dof",), group="guidance"),
        _b("attitude.realized.roll", "Realized Roll", "telemetry", "angle", "rad", "body", ("achieved_attitude_rad[0]",), ("pseudo_6dof",), group="controls"),
        _b("attitude.local.roll_error", "Local Roll Error", "core_state", "angle", "rad", "body", ("roll_error_rad",), _SURFACE, missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("attitude.local.pitch_error", "Local Pitch Error", "core_state", "angle", "rad", "body", ("pitch_error_rad",), _SURFACE, missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("attitude.local.yaw_error", "Local Yaw Error", "core_state", "angle", "rad", "body", ("yaw_error_rad",), _SURFACE, missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("angular_rate.body.x", "Body Roll Rate", "core_state", "angular_rate", "rad/s", "body", ("state.p_rad_s", "p_rad_s", "body_rate_rad_s[0]"), ("pseudo_6dof", *_DIRECT, *_SURFACE), group="state"),
        _b("control.wrench.residual.moment.x", "Residual Roll Moment", "telemetry", "moment", "N*m", "body", ("wrench.residual_wrench.moment_x_nm", "residual_moment_x_nm"), (*_DIRECT, *_SURFACE), group="diagnostics", missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("control.wrench.residual.moment.y", "Residual Pitch Moment", "telemetry", "moment", "N*m", "body", ("wrench.residual_wrench.moment_y_nm", "residual_moment_y_nm"), (*_DIRECT, *_SURFACE), group="diagnostics", missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("control.wrench.residual.moment.z", "Residual Yaw Moment", "telemetry", "moment", "N*m", "body", ("wrench.residual_wrench.moment_z_nm", "residual_moment_z_nm"), (*_DIRECT, *_SURFACE), group="diagnostics", missions=(*_HUMMINGBIRD_LOCAL_DIRECT_SCREEN, *_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)),
        _b("diagnostics.allocation_residual_norm", "Allocation Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("allocation_residual_norm",), _SURFACE, group="diagnostics", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("diagnostics.saturation_count", "Saturation Count", "telemetry", "count", None, None, ("saturation_count",), _SURFACE, group="diagnostics", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("control.lqi.integral_error.roll", "LQI Roll Integral Error", "telemetry", "angle_time", "rad*s", "body", ("integral_roll_error_rad_s",), _SURFACE, group="controls", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("control.lqi.integral_error.pitch", "LQI Pitch Integral Error", "telemetry", "angle_time", "rad*s", "body", ("integral_pitch_error_rad_s",), _SURFACE, group="controls", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("control.lqi.integral_error.yaw", "LQI Yaw Integral Error", "telemetry", "angle_time", "rad*s", "body", ("integral_yaw_error_rad_s",), _SURFACE, group="controls", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS),
        _b("control.lqi.integral_error.vertical_speed", "LQI Vertical-Speed Integral Error", "telemetry", "speed_time", "m/s*s", "body", ("integral_vertical_speed_m_s_s",), _SURFACE, group="controls", missions=_HUMMINGBIRD_LOCAL_VERTICAL_SCREEN),
        *(
            _b(f"actuator.rotor.{index}.speed.actual", f"Actual Rotor {index} Speed", "telemetry", "angular_rate", "rad/s", "body", (f"rotor_{index}_speed_rad_s",), _SURFACE, group="actuators", missions=_HUMMINGBIRD_LOCAL_ALL_PHYSICAL_SCREENS)
            for index in range(1, 5)
        ),
        *_hummingbird_wrench_telemetry(),
    ),
    "reference_nesc_two_stage_rocket": (
        _b("position.eci.x", "ECI X Position", "core_state", "length", "m", "ECI", ("position_eci_x_m",), _REDUCED),
        _b("position.eci.y", "ECI Y Position", "core_state", "length", "m", "ECI", ("position_eci_y_m",), _REDUCED),
        _b("position.eci.z", "ECI Z Position", "core_state", "length", "m", "ECI", ("position_eci_z_m",), _REDUCED),
        _b("velocity.eci.x", "ECI X Velocity", "core_state", "speed", "m/s", "ECI", ("velocity_eci_mps[0]",), _REDUCED),
        _b("velocity.eci.y", "ECI Y Velocity", "core_state", "speed", "m/s", "ECI", ("velocity_eci_mps[1]",), _REDUCED),
        _b("velocity.eci.z", "ECI Z Velocity", "core_state", "speed", "m/s", "ECI", ("velocity_eci_mps[2]",), _REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources"),
        _b("diagnostics.position_error", "Position Replay Error", "telemetry", "length", "m", None, ("position_error_m",), _REDUCED, group="diagnostics"),
        _b("diagnostics.velocity_error", "Velocity Replay Error", "telemetry", "speed", "m/s", None, ("velocity_error_mps",), _REDUCED, group="diagnostics"),
    ),
    "x15": (
        *_local_vector_core(_REDUCED, missions=("x15_staged_booster_reachability_v1",)),
        *_direct_wrench_core(
            *_X15_LOCAL_DIRECT_WRENCH_SCREENS,
            source_surface_lqi_missions=_X15_SOURCE_SURFACE_LQI_SCREEN,
        ),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), (*_REDUCED, *_DIRECT, *_SURFACE), group="resources", missions=("x15_staged_booster_reachability_v1", *_X15_LOCAL_DIRECT_WRENCH_SCREENS, *_X15_SOURCE_SURFACE_SCREENS)),
        _b("aerodynamics.drag_force", "Drag Force", "telemetry", "force", "N", "local_cartesian", ("drag_force_n",), _REDUCED, group="aerodynamics", missions=("x15_staged_booster_reachability_v1",)),
        _b("guidance.bank.command", "Bank Command", "telemetry", "angle", "deg", "local_cartesian", ("mission_bank_command_deg",), _REDUCED, group="guidance", missions=("x15_staged_booster_reachability_v1",)),
        *_direct_wrench_telemetry(*_X15_LOCAL_DIRECT_WRENCH_SCREENS),
        _b("control.lqi.integral_error.u", "LQI Body-Speed Integral Error", "telemetry", "speed_time", "m", "body", ("integral_error.u_m_s",), _DIRECT, group="controls", missions=("x15_local_direct_wrench_lqi_screen_v1",)),
        *_source_surface_attitude_error_lqi_core(*_X15_SOURCE_SURFACE_LQI_SCREEN),
        *(
            _b(
                f"source_fixture.velocity.body.{axis}",
                f"Source Fixture Body {axis.upper()} Velocity",
                "core_state",
                "speed",
                "m/s",
                "body",
                (f"{axis}_m_s", f"body_velocity_m_s[{index}]"),
                _SURFACE,
                group="dynamics",
                missions=_X15_SOURCE_SURFACE_SCREENS,
            )
            for index, axis in enumerate(("u", "v", "w"))
        ),
        _b("source_fixture.mach", "Source Fixture Mach", "telemetry", "dimensionless", "dimensionless", "body", ("source_mach",), _SURFACE, group="aerodynamics", missions=_X15_SOURCE_SURFACE_SCREENS),
        _b("source_fixture.alpha", "Source Fixture Angle of Attack", "telemetry", "angle", "deg", "body", ("source_alpha_deg",), _SURFACE, group="aerodynamics", missions=_X15_SOURCE_SURFACE_SCREENS),
        _b("source_fixture.beta", "Source Fixture Sideslip", "telemetry", "angle", "deg", "body", ("source_beta_deg",), _SURFACE, group="aerodynamics", missions=_X15_SOURCE_SURFACE_SCREENS),
        *(
            _b(f"control.moment.{axis}.requested", f"Requested {axis.upper()} Moment", "telemetry", "moment", "N*m", "body", (f"requested_moment_{axis}_nm",), _SURFACE, group="controls", missions=_X15_SOURCE_SURFACE_SCREENS)
            for axis in ("x", "y", "z")
        ),
        *(
            _b(f"control.moment.{axis}.achieved", f"Achieved {axis.upper()} Moment", "telemetry", "moment", "N*m", "body", (f"achieved_moment_{axis}_nm",), _SURFACE, group="controls", missions=_X15_SOURCE_SURFACE_SCREENS)
            for axis in ("x", "y", "z")
        ),
        *(
            _b(f"control.moment.{axis}.residual", f"Residual {axis.upper()} Moment", "telemetry", "moment", "N*m", "body", (f"residual_moment_{axis}_nm",), _SURFACE, group="diagnostics", missions=_X15_SOURCE_SURFACE_SCREENS)
            for axis in ("x", "y", "z")
        ),
        _b("control.allocation.residual_norm", "Controlled Allocation Residual", "telemetry", "moment", "N*m", "body", ("allocation_controlled_residual_norm",), _SURFACE, group="diagnostics", missions=_X15_SOURCE_SURFACE_SCREENS),
        _b("control.allocation.status", "Source-Surface Allocation Status", "telemetry", "enum", None, None, ("allocation_status",), _SURFACE, group="diagnostics", missions=_X15_SOURCE_SURFACE_SCREENS, interpolation="step"),
        _b("control.allocation.saturation_count", "Allocation Saturation Count", "telemetry", "count", None, None, ("saturation_count",), _SURFACE, group="diagnostics", missions=_X15_SOURCE_SURFACE_SCREENS),
        _b("control.source_effectiveness_rank", "Source Effectiveness Rank", "telemetry", "count", None, None, ("source_effectiveness_rank",), _SURFACE, group="diagnostics", missions=_X15_SOURCE_SURFACE_SCREENS, interpolation="step"),
        *(
            _b(f"actuator.surface.{name}.actual", f"Actual {name.replace('_', ' ').title()}", "telemetry", "angle", "deg", "body", (f"surface_{name}_deg",), _SURFACE, group="actuators", missions=_X15_SOURCE_SURFACE_SCREENS)
            for name in ("symmetric_stabilator", "differential_stabilator", "rudder")
        ),
    ),
    "hl20_mod_k": (
        *_local_vector_core(_REDUCED, missions=("hl20_source_booster_release_replay_v1",)),
        *_direct_wrench_core(
            *_HL20_LOCAL_DIRECT_WRENCH_SCREENS,
            source_surface_lqi_missions=_HL20_SOURCE_SURFACE_LQI_SCREEN,
        ),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), (*_REDUCED, *_DIRECT, *_SURFACE), group="resources", missions=("hl20_source_booster_release_replay_v1", *_HL20_LOCAL_DIRECT_WRENCH_SCREENS, *_HL20_SOURCE_SURFACE_SCREENS)),
        *_direct_wrench_telemetry(*_HL20_LOCAL_DIRECT_WRENCH_SCREENS),
        _b("control.lqi.integral_error.u", "LQI Body-Speed Integral Error", "telemetry", "speed_time", "m", "body", ("integral_error.u_m_s",), _DIRECT, group="controls", missions=("hl20_local_direct_wrench_lqi_screen_v1",)),
        *_source_surface_attitude_error_lqi_core(*_HL20_SOURCE_SURFACE_LQI_SCREEN),
        _b("source_fixture.velocity.body.x", "Source Fixture Body X Velocity", "core_state", "speed", "m/s", "body", ("u_m_s", "body_velocity_m_s[0]"), _SURFACE, group="dynamics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("source_fixture.velocity.body.y", "Source Fixture Body Y Velocity", "core_state", "speed", "m/s", "body", ("v_m_s", "body_velocity_m_s[1]"), _SURFACE, group="dynamics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("source_fixture.velocity.body.z", "Source Fixture Body Z Velocity", "core_state", "speed", "m/s", "body", ("w_m_s", "body_velocity_m_s[2]"), _SURFACE, group="dynamics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("source_fixture.body_rate.pitch", "Source Fixture Body Pitch Rate", "core_state", "angular_rate", "rad/s", "body", ("q_rad_s",), _SURFACE, group="dynamics", missions=_HL20_SOURCE_SURFACE_AUTHORITY_SCREEN),
        _b("aerodynamics.pitch_coefficient", "Source Pitch Coefficient", "telemetry", "dimensionless", "dimensionless", "body", ("source_pitch_coefficient",), _SURFACE, group="aerodynamics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.pitch_moment.requested", "Requested Pitch Moment", "telemetry", "moment", "N*m", "body", ("requested_pitch_moment_nm",), _SURFACE, group="controls", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.pitch_moment.achieved", "Achieved Pitch Moment", "telemetry", "moment", "N*m", "body", ("achieved_pitch_moment_nm",), _SURFACE, group="controls", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.pitch_moment.residual", "Pitch Moment Residual", "telemetry", "moment", "N*m", "body", ("pitch_moment_residual_nm",), _SURFACE, group="controls", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.allocation.residual_norm", "Controlled Allocation Residual", "telemetry", "moment", "N*m", "body", ("allocation_controlled_residual_norm",), _SURFACE, group="diagnostics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.allocation.status", "Source-Surface Allocation Status", "telemetry", "enum", None, None, ("allocation_status",), _SURFACE, group="diagnostics", missions=_HL20_SOURCE_SURFACE_SCREENS, interpolation="step"),
        _b("control.allocation.saturation_count", "Allocation Saturation Count", "telemetry", "count", None, None, ("saturation_count",), _SURFACE, group="diagnostics", missions=_HL20_SOURCE_SURFACE_SCREENS),
        _b("control.source_effectiveness_rank", "Source Effectiveness Rank", "telemetry", "count", None, None, ("source_effectiveness_rank",), _SURFACE, group="diagnostics", missions=_HL20_SOURCE_SURFACE_SCREENS, interpolation="step"),
        *(
            _b(f"actuator.surface.{name}.actual", f"Actual {name.replace('_', ' ').title()}", "telemetry", "angle", "deg", "body", (f"surface_{name}_deg",), _SURFACE, group="actuators", missions=_HL20_SOURCE_SURFACE_SCREENS)
            for name in (
                "upper_left_body_flap",
                "lower_left_body_flap",
                "upper_right_body_flap",
                "lower_right_body_flap",
                "left_wing_flap",
                "right_wing_flap",
                "rudder",
            )
        ),
    ),
    "tumbling_body": (
        *_local_vector_core(_REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources"),
        _b("aerodynamics.drag_force", "Drag Force", "telemetry", "force", "N", "local_cartesian", ("drag_force_n",), _REDUCED, group="aerodynamics"),
        _b("aerodynamics.projected_area", "Projected Area", "telemetry", "area", "m^2", "body", ("projected_area_m2",), _REDUCED, group="aerodynamics"),
        _b("angular_rate.norm", "Angular-Rate Norm", "telemetry", "angular_rate", "rad/s", "body", ("angular_rate_norm_rad_s",), _REDUCED, group="dynamics"),
    ),
}


def native_output_bindings(model_id: str) -> tuple[NativeOutputBinding, ...]:
    """Return exact normalized channels for one canonical native family."""

    return _MODEL_BINDINGS.get(model_id, ())
    ####


def native_output_channel_metadata(model_id: str) -> tuple[TrajectoryOutputChannelMetadata, ...]:
    """Build provider advertisement records from the executable extraction map."""

    return tuple(
        TrajectoryOutputChannelMetadata(
            id=item.id,
            label=item.label,
            description=item.description,
            quantity=item.quantity,
            canonical_unit=item.unit,
            display_unit=item.unit,
            data_type="string" if item.quantity == "enum" else "float64",
            sampling_semantics="discrete_sample" if item.quantity == "enum" else "continuous_sample",
            frame=item.frame,
            interpolation=item.interpolation,
            periodicity=(
                ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0)
                if item.interpolation == "periodic"
                else None
            ),
            availability="guaranteed" if item.channel_class == "core_state" else "guaranteed",
            compatible_fidelities=item.fidelities,
            compatible_mission_templates=item.mission_templates,
            operations=("batch",),
            presentation=ValuePresentationMetadata(group=item.telemetry_group or "core_state"),
            source_refs=("src/taoryx/trajectory/native_output_contract.py",),
            provenance="exact native truth-telemetry extraction binding",
            claim_boundary="Availability is exact for the compatible mission/fidelity batch binding only.",
        )
        for item in native_output_bindings(model_id)
    )
    ####


def native_output_reference_frames(model_id: str) -> tuple[TrajectoryReferenceFrameMetadata, ...]:
    """Return every frame named by the native channel map."""

    frame_ids = tuple(dict.fromkeys(item.frame for item in native_output_bindings(model_id) if item.frame is not None))
    definitions = {
        "geodetic": TrajectoryReferenceFrameMetadata(
            id="geodetic",
            name="Geodetic Coordinates",
            description="Latitude, longitude, and altitude relative to the configured Earth ellipsoid.",
            frame_kind="geodetic",
            axes=("latitude", "longitude", "altitude"),
            handedness="not_applicable",
            origin="Earth ellipsoid",
            orientation="geodetic latitude/longitude with positive-up altitude",
            provenance="native truth telemetry",
        ),
        "NED": TrajectoryReferenceFrameMetadata(
            id="NED",
            name="Local North-East-Down",
            description="Mission-local north/east/down tangent frame; altitude channels remain explicitly positive up.",
            frame_kind="local_tangent",
            axes=("north", "east", "down"),
            handedness="right",
            origin="mission local datum",
            orientation="north-east-down",
            provenance="native truth telemetry",
        ),
        "local_cartesian": TrajectoryReferenceFrameMetadata(
            id="local_cartesian",
            name="Local Cartesian",
            description="Provider-declared local Cartesian trajectory frame.",
            frame_kind="local_tangent",
            axes=("x", "y", "z"),
            handedness="right",
            origin="mission local datum",
            orientation="declared by the native mission adapter",
            provenance="native truth telemetry",
        ),
        "ECI": TrajectoryReferenceFrameMetadata(
            id="ECI",
            name="Earth-Centered Inertial",
            description="Earth-centered inertial Cartesian frame used by the NESC source replay.",
            frame_kind="inertial",
            axes=("x", "y", "z"),
            handedness="right",
            origin="Earth center",
            orientation="source-defined inertial axes",
            provenance="NESC source replay",
        ),
        "body": TrajectoryReferenceFrameMetadata(
            id="body",
            name="Body Frame",
            description="Vehicle-fixed forward-right-down body frame.",
            frame_kind="body",
            axes=("forward", "right", "down"),
            handedness="right",
            origin="vehicle reference point",
            orientation="forward-right-down",
            provenance="native vehicle adapter",
        ),
    }
    return tuple(definitions[item] for item in frame_ids)
    ####


def extract_native_channel(row: dict[str, object], binding: NativeOutputBinding) -> float | str:
    """Read one output value with the exact advertised representation."""

    if binding.quantity == "enum":
        for path in binding.source_paths:
            try:
                value = _path_value(row, path)
            except (KeyError, IndexError, TypeError, ValueError):
                continue
            if isinstance(value, str) and value:
                return value
        raise ValueError(f"native truth row does not contain text source data for advertised enum channel {binding.id!r}")

    for path in binding.source_paths:
        try:
            value = _path_value(row, path)
        except (KeyError, IndexError, TypeError, ValueError):
            continue
        if isinstance(value, str):
            try:
                value = float(value)
            except ValueError:
                continue
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
            continue
        return float(value)
    raise ValueError(f"native truth row does not contain finite source data for advertised channel {binding.id!r}")
    ####


def _path_value(row: dict[str, object], path: str) -> object:
    if path.endswith("|norm"):
        norm_source = _path_value(row, path.removesuffix("|norm"))
        vector = _vector(norm_source)
        return math.sqrt(sum(item * item for item in vector))
    index: int | None = None
    if path.endswith("]") and "[" in path:
        path, index_text = path.rsplit("[", maxsplit=1)
        index = int(index_text[:-1])
    value: object = row
    for name in path.split("."):
        if not isinstance(value, dict):
            raise TypeError(path)
        value = value[name]
    if index is not None:
        value = _vector(value)[index]
    return value
    ####


def _vector(value: object) -> tuple[float, ...]:
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, list | tuple):
        raise TypeError("expected vector")
    result = tuple(float(item) for item in value)
    if not result or not all(math.isfinite(item) for item in result):
        raise ValueError("invalid vector")
    return result
    ####


__all__ = [
    "NativeOutputBinding",
    "extract_native_channel",
    "native_output_bindings",
    "native_output_channel_metadata",
    "native_output_reference_frames",
]
