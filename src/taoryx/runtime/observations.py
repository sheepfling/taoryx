"""Fast standard and tiered runtime observation contracts."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import cast

from taoryx.contracts import Vector3
from taoryx.modes import Quaternion
from taoryx.trajectory.standard_output import StandardEcefState, standard_ecef_state_from_values

from .common import RuntimeVehicle


@dataclass(frozen=True, slots=True)
class StandardRuntimeOutput:
    """Stable hot-path snapshot shared by controllers, UIs, and telemetry.

    The historical nullable convenience fields remain available, while the
    required ``standard_ecef`` sidecar gives every runtime observation the
    common kinematic minimum even when the underlying vehicle only exposes a
    partial native state.
    """

    time: float
    position_ecfc: Vector3 | None
    velocity_ecfc: Vector3 | None
    acceleration_ecfc: Vector3 | None
    attitude_quaternion: Quaternion | None
    omega_body: Vector3 | None
    mass: float | None
    segment: int
    standard_ecef: StandardEcefState


@dataclass(frozen=True, slots=True)
class RuntimeObservation:
    """Three-tier observation: standard, curated status, and optional deep data."""

    standard: StandardRuntimeOutput
    status: Mapping[str, object] = field(default_factory=dict)
    deep: Mapping[str, object] | None = None

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-compatible observation view."""

        standard = self.standard
        return {
            "standard": {
                "time": standard.time,
                "position_ecfc": _vector_payload(standard.position_ecfc),
                "velocity_ecfc": _vector_payload(standard.velocity_ecfc),
                "acceleration_ecfc": _vector_payload(standard.acceleration_ecfc),
                "attitude_quaternion": _quaternion_payload(standard.attitude_quaternion),
                "omega_body": _vector_payload(standard.omega_body),
                "mass": standard.mass,
                "segment": standard.segment,
                "standard_ecef": standard.standard_ecef.model_dump(mode="json", by_alias=True),
            },
            "status": dict(self.status),
            "deep": None if self.deep is None else dict(self.deep),
        }
    ####


def observe_vehicle(
    vehicle: RuntimeVehicle,
    *,
    status_names: Sequence[str] = (),
    include_deep: bool = False,
) -> RuntimeObservation:
    """Build a tiered observation without forcing callers through raw state names."""

    state = vehicle.state
    named = dict(state.named)
    named.update(vehicle.parameters)
    # Interactive control commands are live inputs even before the next
    # integration sample; expose their achieved values through status output.
    named.update(vehicle.control_values)
    for name, value in zip(state.value_names, state.values, strict=False):
        named.setdefault(name, value)
    position = _vector_from_aliases(named, (("x_ecfc", "x"), ("y_ecfc", "y"), ("z_ecfc", "z")))
    velocity = _vector_from_aliases(named, (("xdot_ecfc", "xdot", "xdt"), ("ydot_ecfc", "ydot", "ydt"), ("zdot_ecfc", "zdot", "zdt")))
    acceleration = _acceleration(vehicle)
    attitude = vehicle.kinematic_state.attitude if vehicle.kinematic_state is not None else None
    omega = vehicle.body_rate_provider(state) if vehicle.body_rate_provider is not None else None
    standard = StandardRuntimeOutput(
        time=state.time,
        position_ecfc=position,
        velocity_ecfc=velocity,
        acceleration_ecfc=acceleration,
        attitude_quaternion=attitude,
        omega_body=omega,
        mass=_number(named.get("mass", named.get("wt"))),
        segment=vehicle.segment_number,
        standard_ecef=standard_ecef_state_from_values(
            state.time,
            _standard_ecef_values(
                position=position,
                velocity=velocity,
                acceleration=acceleration,
                attitude=attitude,
                angular_velocity=omega,
            ),
        ),
    )
    status = {name: named[name.casefold()] for name in status_names if name.casefold() in named}
    deep = dict(named) if include_deep else None
    return RuntimeObservation(standard, status, deep)
####


def _vector_from_names(named: Mapping[str, float], names: tuple[str, str, str]) -> Vector3 | None:
    values = [named.get(name) for name in names]
    if not all(isinstance(value, (int, float)) for value in values):
        return None
    numeric_values = cast(tuple[float | int, ...], tuple(values))
    return Vector3(*(float(value) for value in numeric_values))
####


def _vector_from_aliases(named: Mapping[str, float], names: tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]) -> Vector3 | None:
    values = [next((named.get(alias) for alias in aliases if alias in named), None) for aliases in names]
    if not all(isinstance(value, (int, float)) for value in values):
        return None
    numeric_values = cast(tuple[float | int, ...], tuple(values))
    return Vector3(*(float(value) for value in numeric_values))
####


def _acceleration(vehicle: RuntimeVehicle) -> Vector3 | None:
    if vehicle.derivative is None:
        return _vector_from_names(vehicle.state.named, ("xddot_ecfc", "yddot_ecfc", "zddot_ecfc"))
    try:
        rates = tuple(float(value) for value in vehicle.derivative(vehicle.state))
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
    if len(rates) < 6:
        return None
    return Vector3(*rates[3:6])
####


def _standard_ecef_values(
    *,
    position: Vector3 | None,
    velocity: Vector3 | None,
    acceleration: Vector3 | None,
    attitude: Quaternion | None,
    angular_velocity: Vector3 | None,
) -> dict[str, tuple[float, float, float] | tuple[float, float, float, float]]:
    """Map the runtime's typed native values into the shared projector."""

    values: dict[str, tuple[float, float, float] | tuple[float, float, float, float]] = {}
    if position is not None:
        values["position_ecfc_m"] = (position.x, position.y, position.z)
    if velocity is not None:
        values["velocity_ecfc_mps"] = (velocity.x, velocity.y, velocity.z)
    if acceleration is not None:
        values["acceleration_ecfc_mps2"] = (acceleration.x, acceleration.y, acceleration.z)
    if attitude is not None:
        values["ecef_from_body_wxyz"] = (attitude.w, attitude.x, attitude.y, attitude.z)
    if angular_velocity is not None:
        values["angular_velocity_body_radps"] = (angular_velocity.x, angular_velocity.y, angular_velocity.z)
    return values
    ####


def _number(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) else None
####


def _vector_payload(value: Vector3 | None) -> dict[str, float] | None:
    return None if value is None else {"x": value.x, "y": value.y, "z": value.z}
####


def _quaternion_payload(value: Quaternion | None) -> dict[str, float] | None:
    return None if value is None else {"w": value.w, "x": value.x, "y": value.y, "z": value.z}
####
