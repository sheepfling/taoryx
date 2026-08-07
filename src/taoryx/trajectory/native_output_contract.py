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
        telemetry_group=group,
        interpolation=interpolation,
    )
    ####


_REDUCED = ("point_mass_3dof", "pseudo_6dof")
_DIRECT = ("rigid_body_6dof_direct_wrench",)
_RACETRACK = ("powered_fixed_wing_racetrack_v1",)

_GEODETIC_CORE = (
    _b("position.geodetic.altitude", "Geodetic Altitude", "core_state", "length", "m", "geodetic", ("altitude_m",), _REDUCED),
    _b("position.geodetic.latitude", "Geodetic Latitude", "core_state", "angle", "deg", "geodetic", ("latitude_deg",), _REDUCED),
    _b("position.geodetic.longitude", "Geodetic Longitude", "core_state", "angle", "deg", "geodetic", ("longitude_deg",), _REDUCED, interpolation="periodic"),
    _b("velocity.speed", "Speed", "core_state", "speed", "m/s", None, ("speed_m_s",), _REDUCED),
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


def _direct_wrench_core(mission: str) -> tuple[NativeOutputBinding, ...]:
    return tuple(
        _b(identifier, label, "core_state", quantity, unit, "body", (path,), _DIRECT, missions=(mission,))
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


def _direct_wrench_telemetry(mission: str) -> tuple[NativeOutputBinding, ...]:
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
                        missions=(mission,),
                    )
                )
    result.extend(
        (
            _b("control.wrench.residual_norm", "Wrench Residual Norm", "telemetry", "mixed_wrench_norm", None, None, ("wrench.residual_norm",), _DIRECT, group="diagnostics", missions=(mission,)),
            _b("control.feedback_norm", "Feedback Error Norm", "telemetry", "mixed_state_norm", None, None, ("feedback_norm",), _DIRECT, group="diagnostics", missions=(mission,)),
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
    ),
    "b747": (
        *_GEODETIC_CORE,
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle",), _REDUCED, group="controls"),
        _b("actuator.elevator", "Elevator Position", "telemetry", "angle", "deg", "body", ("elevator-deg",), _REDUCED, group="actuators"),
    ),
    "a320_openap_3dof": (
        *_local_core(_REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources"),
        _b("mass.fuel_flow", "Fuel Flow", "telemetry", "mass_rate", "kg/s", None, ("fuel_flow_kg_s",), _REDUCED, group="resources"),
        _b("aerodynamics.dynamic_pressure", "Dynamic Pressure", "telemetry", "pressure", "Pa", None, ("dynamic_pressure_pa",), _REDUCED, group="aerodynamics"),
        _b("propulsion.thrust", "Thrust", "telemetry", "force", "N", "body", ("thrust_n",), _REDUCED, group="propulsion"),
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle_ratio",), _REDUCED, group="controls"),
    ),
    "f16_s119": (
        *_local_core(_REDUCED),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources"),
        _b("aerodynamics.dynamic_pressure", "Dynamic Pressure", "telemetry", "pressure", "Pa", None, ("dynamic_pressure_pa",), _REDUCED, group="aerodynamics"),
        _b("aerodynamics.angle_of_attack", "Angle of Attack", "telemetry", "angle", "deg", "body", ("aero_alpha_deg",), _REDUCED, group="aerodynamics"),
        _b("aerodynamics.sideslip", "Sideslip", "telemetry", "angle", "deg", "body", ("aero_beta_deg",), _REDUCED, group="aerodynamics"),
        _b("actuator.elevator", "Elevator Position", "telemetry", "angle", "deg", "body", ("elevator_deg",), _REDUCED, group="actuators"),
        _b("actuator.aileron", "Aileron Position", "telemetry", "angle", "deg", "body", ("aileron_deg",), _REDUCED, group="actuators"),
        _b("actuator.rudder", "Rudder Position", "telemetry", "angle", "deg", "body", ("rudder_deg",), _REDUCED, group="actuators"),
        _b("control.throttle.realized", "Realized Throttle", "telemetry", "dimensionless", "1", None, ("throttle_fraction",), _REDUCED, group="controls"),
    ),
    "hummingbird": (
        *_local_vector_core(("pseudo_6dof",)),
        _b("resources.battery_fraction", "Battery Fraction", "telemetry", "dimensionless", "1", None, ("battery_fraction",), ("pseudo_6dof",), group="resources"),
        _b("control.thrust.commanded_ratio", "Commanded Thrust Ratio", "telemetry", "dimensionless", "1", None, ("commanded_thrust_ratio",), ("pseudo_6dof",), group="controls"),
        _b("propulsion.thrust.realized", "Realized Thrust", "telemetry", "force", "N", "body", ("achieved_thrust_n",), ("pseudo_6dof",), group="propulsion"),
        _b("attitude.commanded.roll", "Commanded Roll", "telemetry", "angle", "rad", "body", ("commanded_attitude_rad[0]",), ("pseudo_6dof",), group="guidance"),
        _b("attitude.realized.roll", "Realized Roll", "telemetry", "angle", "rad", "body", ("achieved_attitude_rad[0]",), ("pseudo_6dof",), group="controls"),
        _b("angular_rate.body.x", "Body Roll Rate", "telemetry", "angular_rate", "rad/s", "body", ("body_rate_rad_s[0]",), ("pseudo_6dof",), group="controls"),
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
        *_direct_wrench_core("x15_local_direct_wrench_screen_v1"),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources", missions=("x15_staged_booster_reachability_v1",)),
        _b("aerodynamics.drag_force", "Drag Force", "telemetry", "force", "N", "local_cartesian", ("drag_force_n",), _REDUCED, group="aerodynamics", missions=("x15_staged_booster_reachability_v1",)),
        _b("guidance.bank.command", "Bank Command", "telemetry", "angle", "deg", "local_cartesian", ("mission_bank_command_deg",), _REDUCED, group="guidance", missions=("x15_staged_booster_reachability_v1",)),
        *_direct_wrench_telemetry("x15_local_direct_wrench_screen_v1"),
    ),
    "hl20_mod_k": (
        *_local_vector_core(_REDUCED, missions=("hl20_source_booster_release_replay_v1",)),
        *_direct_wrench_core("hl20_local_direct_wrench_screen_v1"),
        _b("mass.total", "Total Mass", "telemetry", "mass", "kg", None, ("mass_kg",), _REDUCED, group="resources", missions=("hl20_source_booster_release_replay_v1",)),
        *_direct_wrench_telemetry("hl20_local_direct_wrench_screen_v1"),
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


def extract_native_channel(row: dict[str, object], binding: NativeOutputBinding) -> float:
    """Read one finite numeric channel from an accepted native truth row."""

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
