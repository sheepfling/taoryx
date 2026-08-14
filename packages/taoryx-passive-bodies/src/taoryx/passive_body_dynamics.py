"""Standalone reduced propagation for reusable passive released bodies.

This module intentionally owns only the released body's local atmospheric
propagation.  It does not know which vehicle emitted the body, does not import
any parent plug-in, and does not infer a separation impulse.  A caller must
provide an accepted release state and its explicitly declared transfer policy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import cast

from .contracts import Frame, FrameVector3, Vector3
from .modes import Quaternion
from .rigid_body import RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment
from .vehicle import DetachedBodyDefinition, DetachedBodyShape, TumblingPolicy

VectorTuple = tuple[float, float, float]


class ReachabilityFidelity(StrEnum):
    """Released-body fidelity labels retained for compatible passive witnesses."""

    POINT_MASS_3DOF = "point_mass_3dof"
    PSEUDO_6DOF = "pseudo_6dof"
    RIGID_BODY_6DOF = "rigid_body_6dof"
    ####


class EnvelopeTermination(StrEnum):
    """Terminal classification for an independently propagated passive body."""

    HORIZON = "horizon"
    GROUND_CONTACT = "ground_contact"
    INVALID = "invalid"
    ####


@dataclass(frozen=True, slots=True)
class RocketGlideVehicle:
    """Local atmospheric environment retained under the legacy plan name.

    The historical name is preserved so the direct-release translator remains
    source-compatible.  Only environment and initial-release fields are used;
    propulsion and lift fields are provenance placeholders for the supplied
    passive witness and never create parent-vehicle dynamics.
    """

    vehicle_id: str = "taoryx-passive-direct-release-v1"
    dry_mass_kg: float = 1.0
    propellant_mass_kg: float = 0.0
    thrust_n: float = 0.0
    burn_time_s: float = 0.0
    reference_area_m2: float = 1.0
    drag_coefficient: float = 0.25
    lift_to_drag: float = 1.0
    initial_speed_m_s: float = 0.0
    initial_altitude_m: float = 0.0
    gravity_m_s2: float = 9.80665
    sea_level_density_kg_m3: float = 1.225
    density_scale_height_m: float = 8_500.0
    aerodynamic_model_id: str = "passive_drag_surrogate_v1"
    actuator_profile_id: str = "none"
    mission_profile_id: str = "passive_direct_release_v1"
    configuration_variant_id: str = "nominal"
    mass_property_profile_id: str = "generic_passive_body_v1"
    wind_velocity_m_s: Vector3 = Vector3(0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        for name in (
            "dry_mass_kg",
            "reference_area_m2",
            "drag_coefficient",
            "gravity_m_s2",
            "sea_level_density_kg_m3",
            "density_scale_height_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"passive environment {name} must be positive and finite")
        for name in ("propellant_mass_kg", "thrust_n", "burn_time_s", "initial_speed_m_s", "initial_altitude_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"passive environment {name} must be non-negative and finite")
        if self.propellant_mass_kg != 0.0 or self.thrust_n != 0.0 or self.burn_time_s != 0.0:
            raise ValueError("a passive released-body environment cannot carry propulsion")
        if not self.vehicle_id.strip() or not self.aerodynamic_model_id.strip():
            raise ValueError("passive environment identifiers must be non-empty")
        if not all(
            math.isfinite(value)
            for value in (
                self.wind_velocity_m_s.x,
                self.wind_velocity_m_s.y,
                self.wind_velocity_m_s.z,
            )
        ):
            raise ValueError("passive environment wind must be finite")
        ####

    ####


@dataclass(frozen=True, slots=True)
class LaunchCommand:
    """Initial local release direction for the standalone direct witness."""

    azimuth_rad: float
    elevation_rad: float
    bank_rad: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.azimuth_rad, self.elevation_rad, self.bank_rad)):
            raise ValueError("passive release direction must be finite")
        ####

    @property
    def launch_direction(self) -> VectorTuple:
        """Return an orthonormal local-tangent launch direction."""

        horizontal = math.cos(self.elevation_rad)
        return (
            horizontal * math.cos(self.azimuth_rad),
            horizontal * math.sin(self.azimuth_rad),
            math.sin(self.elevation_rad),
        )
        ####

    ####


@dataclass(frozen=True, slots=True)
class PointMass3DofState:
    """Local Cartesian translational state for an orientation-averaged body."""

    time_s: float
    position_m: VectorTuple
    velocity_m_s: VectorTuple
    mass_kg: float
    phase: str = "ballistic"

    @property
    def speed_m_s(self) -> float:
        return _norm(self.velocity_m_s)
        ####

    ####


@dataclass(frozen=True, slots=True)
class RigidBody6DofReachabilityState:
    """Passive wrapper around the shared rigid-body state integrator."""

    native: RigidBody6DofState
    phase: str = "ballistic"

    @property
    def time_s(self) -> float:
        return self.native.time
        ####

    @property
    def position_m(self) -> VectorTuple:
        vector = self.native.position.vector
        return (vector.x, vector.y, vector.z)
        ####

    @property
    def velocity_m_s(self) -> VectorTuple:
        vector = self.native.velocity.vector
        return (vector.x, vector.y, vector.z)
        ####

    @property
    def mass_kg(self) -> float:
        return self.native.mass
        ####

    @property
    def speed_m_s(self) -> float:
        return _norm(self.velocity_m_s)
        ####

    ####


PassiveState = PointMass3DofState | RigidBody6DofReachabilityState


@dataclass(frozen=True, slots=True)
class DetachedBodyTrajectory:
    """Independent child trajectory and its standardized passive telemetry."""

    body_id: str
    shape: str
    parent_event_id: str
    deployment_time_s: float
    fidelity: ReachabilityFidelity
    states: tuple[PassiveState, ...]
    termination: EnvelopeTermination
    telemetry: tuple[dict[str, object], ...] = ()
    requested_fidelity: ReachabilityFidelity | None = None

    @property
    def classification(self) -> str:
        return {
            EnvelopeTermination.GROUND_CONTACT: "impact",
            EnvelopeTermination.HORIZON: "timeout",
            EnvelopeTermination.INVALID: "invalid",
        }[self.termination]
        ####

    @property
    def terminal(self) -> PassiveState:
        return self.states[-1]
        ####

    ####


@dataclass(frozen=True, slots=True)
class _Derivative:
    """One translational derivative for the explicit point-mass reduction."""

    position_m_s: VectorTuple
    velocity_m_s2: VectorTuple
    ####


def simulate_passive_body_release(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    command: LaunchCommand,
    *,
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    step_size_s: float = 0.25,
    horizon_s: float = 120.0,
) -> DetachedBodyTrajectory:
    """Propagate one direct atmospheric release without parent coupling."""

    direction = command.launch_direction
    return simulate_passive_body_from_release_state(
        vehicle,
        body,
        position_m=(0.0, 0.0, vehicle.initial_altitude_m),
        velocity_m_s=_scale(direction, vehicle.initial_speed_m_s),
        release_time_s=0.0,
        fidelity=fidelity,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        parent_event_id="atmospheric-release",
        initial_attitude=_euler_quaternion((command.bank_rad, -command.elevation_rad, command.azimuth_rad)),
    )
    ####


def simulate_passive_body_from_release_state(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    *,
    position_m: VectorTuple,
    velocity_m_s: VectorTuple,
    release_time_s: float,
    fidelity: ReachabilityFidelity,
    step_size_s: float,
    horizon_s: float,
    parent_event_id: str,
    initial_attitude: Quaternion | None = None,
) -> DetachedBodyTrajectory:
    """Propagate one body from a parent-supplied accepted local state.

    The position and velocity use a parent-declared local tangent Cartesian
    frame.  This routine deliberately does not transform geodetic or ECI
    state; a parent-specific bridge must make that transformation explicit in
    its release request provenance.
    """

    if not math.isfinite(release_time_s):
        raise ValueError("passive release time must be finite")
    if step_size_s <= 0.0 or horizon_s <= 0.0 or not math.isfinite(step_size_s) or not math.isfinite(horizon_s):
        raise ValueError("passive propagation step_size_s and horizon_s must be positive and finite")
    if not parent_event_id.strip():
        raise ValueError("passive propagation requires a parent event identifier")
    _require_finite_vector("passive release position", position_m)
    _require_finite_vector("passive release velocity", velocity_m_s)
    realized = _realized_fidelity(fidelity, body)
    state = _initial_state(
        body,
        position_m=position_m,
        velocity_m_s=velocity_m_s,
        time_s=release_time_s,
        fidelity=realized,
        initial_attitude=initial_attitude or Quaternion.identity(),
    )
    states: list[PassiveState] = [state]
    termination = EnvelopeTermination.HORIZON
    horizon_end_s = release_time_s + horizon_s
    while state.time_s < horizon_end_s - 1.0e-12:
        step = min(step_size_s, horizon_end_s - state.time_s)
        try:
            state = _rk4_step(vehicle, body, state, step)
        except (ArithmeticError, ValueError):
            termination = EnvelopeTermination.INVALID
            break
        states.append(state)
        if state.time_s > release_time_s and state.position_m[2] <= 0.0:
            termination = EnvelopeTermination.GROUND_CONTACT
            break
        if not _finite_state(state):
            termination = EnvelopeTermination.INVALID
            break
    telemetry = [_body_telemetry(vehicle, body, item, requested_fidelity=fidelity, parent_event_id=parent_event_id) for item in states]
    if telemetry:
        telemetry[-1]["termination"] = termination.value
    return DetachedBodyTrajectory(
        body_id=body.body_id,
        shape=body.shape.value,
        parent_event_id=parent_event_id,
        deployment_time_s=release_time_s,
        fidelity=realized,
        states=tuple(states),
        termination=termination,
        telemetry=tuple(telemetry),
        requested_fidelity=fidelity,
    )
    ####


def _initial_state(
    body: DetachedBodyDefinition,
    *,
    position_m: VectorTuple,
    velocity_m_s: VectorTuple,
    time_s: float,
    fidelity: ReachabilityFidelity,
    initial_attitude: Quaternion,
) -> PassiveState:
    if fidelity is ReachabilityFidelity.RIGID_BODY_6DOF:
        rate = Vector3(0.0, 0.0, 0.0) if body.tumbling_policy is TumblingPolicy.FIXED_ATTITUDE else body.initial_angular_rate_body_rad_s
        native = RigidBody6DofState(
            time_s,
            FrameVector3(Vector3(*position_m), Frame.ECIC),
            FrameVector3(Vector3(*velocity_m_s), Frame.ECIC),
            initial_attitude.normalized(),
            rate,
            body.mass_kg,
            0.0,
        )
        return RigidBody6DofReachabilityState(native)
    return PointMass3DofState(time_s, position_m, velocity_m_s, body.mass_kg)
    ####


def _realized_fidelity(fidelity: ReachabilityFidelity, body: DetachedBodyDefinition) -> ReachabilityFidelity:
    if fidelity is ReachabilityFidelity.PSEUDO_6DOF and body.tumbling_policy is not TumblingPolicy.FIXED_ATTITUDE:
        return ReachabilityFidelity.RIGID_BODY_6DOF
    if fidelity is ReachabilityFidelity.PSEUDO_6DOF:
        return ReachabilityFidelity.PSEUDO_6DOF
    return fidelity
    ####


def _rk4_step(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    state: PassiveState,
    step_size_s: float,
) -> PassiveState:
    if isinstance(state, RigidBody6DofReachabilityState):
        return _rigid_rk4_step(vehicle, body, state, step_size_s)
    first = _point_derivative(vehicle, body, state)
    second = _point_derivative(vehicle, body, _state_with_delta(state, first, 0.5 * step_size_s))
    third = _point_derivative(vehicle, body, _state_with_delta(state, second, 0.5 * step_size_s))
    fourth = _point_derivative(vehicle, body, _state_with_delta(state, third, step_size_s))
    position_rate = cast(
        VectorTuple,
        tuple(
            (first.position_m_s[index] + 2.0 * second.position_m_s[index] + 2.0 * third.position_m_s[index] + fourth.position_m_s[index]) / 6.0
            for index in range(3)
        ),
    )
    velocity_rate = cast(
        VectorTuple,
        tuple(
            (first.velocity_m_s2[index] + 2.0 * second.velocity_m_s2[index] + 2.0 * third.velocity_m_s2[index] + fourth.velocity_m_s2[index]) / 6.0
            for index in range(3)
        ),
    )
    return PointMass3DofState(
        state.time_s + step_size_s,
        _add(state.position_m, _scale(position_rate, step_size_s)),
        _add(state.velocity_m_s, _scale(velocity_rate, step_size_s)),
        state.mass_kg,
    )
    ####


def _point_derivative(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    state: PointMass3DofState,
) -> _Derivative:
    air_velocity = _subtract(state.velocity_m_s, _vector_tuple(vehicle.wind_velocity_m_s))
    speed = _norm(air_velocity)
    density = _atmosphere(vehicle, state.position_m[2])
    area = _average_projected_area(body)
    drag_magnitude = 0.5 * density * speed * speed * area * vehicle.drag_coefficient
    drag = _scale(_unit(air_velocity), -drag_magnitude)
    acceleration = _add(_scale(drag, 1.0 / state.mass_kg), (0.0, 0.0, -vehicle.gravity_m_s2))
    return _Derivative(state.velocity_m_s, acceleration)
    ####


def _state_with_delta(state: PointMass3DofState, derivative: _Derivative, dt_s: float) -> PointMass3DofState:
    return PointMass3DofState(
        state.time_s + dt_s,
        _add(state.position_m, _scale(derivative.position_m_s, dt_s)),
        _add(state.velocity_m_s, _scale(derivative.velocity_m_s2, dt_s)),
        state.mass_kg,
    )
    ####


def _rigid_rk4_step(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    state: RigidBody6DofReachabilityState,
    step_size_s: float,
) -> RigidBody6DofReachabilityState:
    if step_size_s > 0.05:
        current = state
        remaining = step_size_s
        while remaining > 1.0e-12:
            substep = min(0.05, remaining)
            current = _rigid_rk4_step(vehicle, body, current, substep)
            remaining -= substep
        return current
    model = _rigid_body_model(vehicle, body)
    values = state.native.to_values()

    def derivative(time_s: float, current: tuple[float, ...]) -> tuple[float, ...]:
        return model.derivative(RigidBody6DofState.from_values(time_s, current))

    first = derivative(state.time_s, values)
    second_values = tuple(value + 0.5 * step_size_s * slope for value, slope in zip(values, first, strict=True))
    second = derivative(state.time_s + 0.5 * step_size_s, second_values)
    third_values = tuple(value + 0.5 * step_size_s * slope for value, slope in zip(values, second, strict=True))
    third = derivative(state.time_s + 0.5 * step_size_s, third_values)
    fourth_values = tuple(value + step_size_s * slope for value, slope in zip(values, third, strict=True))
    fourth = derivative(state.time_s + step_size_s, fourth_values)
    integrated = tuple(
        value + step_size_s * (first_value + 2.0 * second_value + 2.0 * third_value + fourth_value) / 6.0
        for value, first_value, second_value, third_value, fourth_value in zip(values, first, second, third, fourth, strict=True)
    )
    return RigidBody6DofReachabilityState(RigidBody6DofState.from_values(state.time_s + step_size_s, integrated))
    ####


def _rigid_body_model(vehicle: RocketGlideVehicle, body: DetachedBodyDefinition) -> RigidBody6DofModel:
    inertia = _inertia(body)

    def force_moment(native: RigidBody6DofState) -> RigidBodyForceMoment:
        air_velocity_ecic = native.velocity.vector - vehicle.wind_velocity_m_s
        air_velocity_body = native.attitude.conjugate().rotate(air_velocity_ecic)
        speed = air_velocity_body.norm()
        density = _atmosphere(vehicle, native.position.vector.z)
        area = _projected_area(body, native.attitude, (air_velocity_ecic.x, air_velocity_ecic.y, air_velocity_ecic.z))
        dynamic_pressure = 0.5 * density * speed * speed
        drag_body = air_velocity_body.scaled(-dynamic_pressure * area * vehicle.drag_coefficient / max(speed, 1.0e-12))
        lever = body.center_of_pressure_m - body.center_of_mass_m
        aerodynamic_moment = lever.cross(drag_body)
        damping = Vector3(
            inertia.x * native.body_rate.x / 5.0,
            inertia.y * native.body_rate.y / 5.0,
            inertia.z * native.body_rate.z / 5.0,
        )
        moment = aerodynamic_moment - damping if body.tumbling_policy is not TumblingPolicy.FIXED_ATTITUDE else Vector3(0.0, 0.0, 0.0)
        return RigidBodyForceMoment(drag_body, moment, aero_force_body=drag_body, aero_moment_body=moment)

    return RigidBody6DofModel(
        inertia=inertia,
        force_moment=force_moment,
        gravity=lambda _state: Vector3(0.0, 0.0, -vehicle.gravity_m_s2),
        dry_mass=body.mass_kg,
    )
    ####


def _inertia(body: DetachedBodyDefinition) -> Vector3:
    if body.inertia_kg_m2 is not None:
        return body.inertia_kg_m2
    dimensions = body.dimensions_m
    mass = body.mass_kg
    if body.shape is DetachedBodyShape.SPHERE:
        value = 0.4 * mass * dimensions[0] ** 2
        return Vector3(value, value, value)
    if body.shape is DetachedBodyShape.CYLINDER:
        radius, length = dimensions
        return Vector3(0.5 * mass * radius**2, mass * (3.0 * radius**2 + length**2) / 12.0, mass * (3.0 * radius**2 + length**2) / 12.0)
    if body.shape is DetachedBodyShape.CONE:
        radius, height = dimensions
        return Vector3(0.3 * mass * radius**2, mass * (0.15 * radius**2 + 0.6 * height**2), mass * (0.15 * radius**2 + 0.6 * height**2))
    if body.shape is DetachedBodyShape.TRIAXIAL_ELLIPSOID:
        axis_x, axis_y, axis_z = dimensions
        return Vector3(mass * (axis_y**2 + axis_z**2) / 5.0, mass * (axis_x**2 + axis_z**2) / 5.0, mass * (axis_x**2 + axis_y**2) / 5.0)
    axial, transverse = dimensions
    return Vector3(mass * transverse**2 / 5.0, mass * (axial**2 + transverse**2) / 5.0, mass * (axial**2 + transverse**2) / 5.0)
    ####


def _body_telemetry(
    vehicle: RocketGlideVehicle,
    body: DetachedBodyDefinition,
    state: PassiveState,
    *,
    requested_fidelity: ReachabilityFidelity,
    parent_event_id: str,
) -> dict[str, object]:
    air_velocity = _subtract(state.velocity_m_s, _vector_tuple(vehicle.wind_velocity_m_s))
    speed = _norm(air_velocity)
    density = _atmosphere(vehicle, state.position_m[2])
    attitude = state.native.attitude if isinstance(state, RigidBody6DofReachabilityState) else None
    area = _projected_area(body, attitude, air_velocity)
    average_area = _average_projected_area(body)
    area_policy = "orientation_averaged_projected_area" if isinstance(state, PointMass3DofState) else "native_rigid_body_reuse_instantaneous_projected_area"
    payload: dict[str, object] = {
        "time_s": state.time_s,
        "position_m": list(state.position_m),
        "velocity_m_s": list(state.velocity_m_s),
        "air_relative_velocity_m_s": list(air_velocity),
        "air_relative_speed_m_s": speed,
        "mass_kg": state.mass_kg,
        "phase": state.phase,
        "projected_area_m2": area,
        "projected_area_reference_average_m2": average_area,
        "projected_area_ratio_to_reference_average": area / max(average_area, 1.0e-12),
        "projected_area_policy": area_policy,
        "requested_fidelity": requested_fidelity.value,
        "realized_state_fidelity": (
            ReachabilityFidelity.POINT_MASS_3DOF.value if isinstance(state, PointMass3DofState) else ReachabilityFidelity.RIGID_BODY_6DOF.value
        ),
        "angular_rate_norm_rad_s": 0.0,
        "drag_force_n": 0.5 * density * speed * speed * area * vehicle.drag_coefficient,
        "event_id": parent_event_id,
        "termination": None,
        "configuration_variant_id": vehicle.configuration_variant_id,
        "mass_property_profile_id": vehicle.mass_property_profile_id,
        "wind_velocity_m_s": list(_vector_tuple(vehicle.wind_velocity_m_s)),
    }
    if isinstance(state, RigidBody6DofReachabilityState):
        quaternion = state.native.attitude
        rate = state.native.body_rate
        payload["attitude_quaternion"] = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        payload["attitude_rate_rad_s"] = [rate.x, rate.y, rate.z]
        payload["angular_rate_norm_rad_s"] = rate.norm()
    return payload
    ####


def _projected_area(body: DetachedBodyDefinition, attitude: Quaternion | None, velocity_m_s: VectorTuple) -> float:
    if body.shape is DetachedBodyShape.SPHERE:
        return body.reference_area_m2
    if attitude is None:
        return _average_projected_area(body) if body.tumbling_policy is not TumblingPolicy.FIXED_ATTITUDE else body.reference_area_m2
    axis = attitude.rotate(Vector3(1.0, 0.0, 0.0))
    cosine = abs(_dot(_unit(velocity_m_s), (axis.x, axis.y, axis.z)))
    sine = math.sqrt(max(0.0, 1.0 - cosine * cosine))
    return _area_for_axis_alignment(body, cosine, sine, velocity_m_s)
    ####


def _average_projected_area(body: DetachedBodyDefinition) -> float:
    if body.shape is DetachedBodyShape.SPHERE:
        return body.reference_area_m2
    samples = 128
    return (
        sum(
            _area_for_axis_alignment(
                body, math.cos((index + 0.5) * math.pi / (2.0 * samples)), math.sin((index + 0.5) * math.pi / (2.0 * samples)), (1.0, 0.0, 0.0)
            )
            for index in range(samples)
        )
        / samples
    )
    ####


def _area_for_axis_alignment(
    body: DetachedBodyDefinition,
    cosine: float,
    sine: float,
    velocity_m_s: VectorTuple,
) -> float:
    dimensions = body.dimensions_m
    if body.shape is DetachedBodyShape.CYLINDER:
        radius, length = dimensions
        return math.pi * radius**2 * cosine + 2.0 * radius * length * sine
    if body.shape is DetachedBodyShape.SPHEROID:
        axial, transverse = dimensions
        return math.pi * transverse * math.sqrt(transverse**2 * cosine**2 + axial**2 * sine**2)
    if body.shape is DetachedBodyShape.CONE:
        radius, height = dimensions
        if cosine <= 1.0e-12:
            return radius * height
        delta = height * sine / (radius * cosine)
        if delta <= 1.0:
            return math.pi * radius**2 * cosine
        return radius**2 * cosine * (math.pi + math.sqrt(delta**2 - 1.0) - math.acos(1.0 / delta))
    if body.shape is DetachedBodyShape.TRIAXIAL_ELLIPSOID:
        axis_x, axis_y, axis_z = dimensions
        direction = _unit(velocity_m_s)
        return math.pi * axis_x * axis_y * axis_z * math.sqrt((direction[0] / axis_x) ** 2 + (direction[1] / axis_y) ** 2 + (direction[2] / axis_z) ** 2)
    return body.reference_area_m2
    ####


def _atmosphere(vehicle: RocketGlideVehicle, altitude_m: float) -> float:
    return vehicle.sea_level_density_kg_m3 * math.exp(-max(0.0, altitude_m) / vehicle.density_scale_height_m)
    ####


def _euler_quaternion(attitude_rad: VectorTuple) -> Quaternion:
    roll, pitch, yaw = (0.5 * value for value in attitude_rad)
    return Quaternion(
        math.cos(yaw) * math.cos(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(pitch) * math.sin(roll),
        math.cos(yaw) * math.cos(pitch) * math.sin(roll) - math.sin(yaw) * math.sin(pitch) * math.cos(roll),
        math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.cos(pitch) * math.sin(roll),
        math.sin(yaw) * math.cos(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(pitch) * math.cos(roll),
    ).normalized()
    ####


def _finite_state(state: PassiveState) -> bool:
    return all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg))
    ####


def _require_finite_vector(label: str, value: VectorTuple) -> None:
    if not all(math.isfinite(component) for component in value):
        raise ValueError(f"{label} must contain finite components")
    ####


def _vector_tuple(value: Vector3) -> VectorTuple:
    return (value.x, value.y, value.z)
    ####


def _norm(value: VectorTuple) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _unit(value: VectorTuple) -> VectorTuple:
    magnitude = _norm(value)
    if magnitude <= 1.0e-12:
        return (1.0, 0.0, 0.0)
    return _scale(value, 1.0 / magnitude)
    ####


def _dot(left: VectorTuple, right: VectorTuple) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
    ####


def _add(left: VectorTuple, right: VectorTuple) -> VectorTuple:
    return tuple(a + b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


def _subtract(left: VectorTuple, right: VectorTuple) -> VectorTuple:
    return tuple(a - b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


def _scale(value: VectorTuple, factor: float) -> VectorTuple:
    return tuple(component * factor for component in value)  # type: ignore[return-value]
    ####


__all__ = [
    "DetachedBodyTrajectory",
    "EnvelopeTermination",
    "LaunchCommand",
    "ReachabilityFidelity",
    "RocketGlideVehicle",
    "simulate_passive_body_from_release_state",
    "simulate_passive_body_release",
]
