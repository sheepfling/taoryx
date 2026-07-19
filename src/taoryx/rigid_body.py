"""Small, explicit rigid-body 6-DOF extension model for TAORYX.

This module is intentionally separate from the manual-bounded TAOS point-mass
kernel.  It provides the state and force/moment contracts needed to build
synthetic 6-DOF scenarios without claiming historical TAOS compatibility.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from .contracts import Frame, FrameVector3, Vector3
from .modes import Quaternion

RIGID_BODY_STATE_NAMES = (
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "qw",
    "qx",
    "qy",
    "qz",
    "wx",
    "wy",
    "wz",
    "mass",
    "propellant_mass",
    "heat_load",
    "peak_heat_rate",
)


@dataclass(frozen=True, slots=True)
class RigidBody6DofState:
    """ECIC translation plus body attitude, body rates, mass, and heating."""

    time: float
    position: FrameVector3
    velocity: FrameVector3
    attitude: Quaternion
    body_rate: Vector3
    mass: float
    propellant_mass: float
    heat_load: float = 0.0
    peak_heat_rate: float = 0.0

    def __post_init__(self) -> None:
        if self.position.frame is not Frame.ECIC or self.velocity.frame is not Frame.ECIC:
            raise ValueError("rigid-body position and velocity must use ECIC")
        if not math.isfinite(self.time):
            raise ValueError("rigid-body time must be finite")
        if not math.isfinite(self.mass) or self.mass <= 0.0:
            raise ValueError("rigid-body mass must be positive and finite")
        if not math.isfinite(self.propellant_mass) or self.propellant_mass < 0.0 or self.propellant_mass > self.mass:
            raise ValueError("rigid-body propellant mass must be finite and within total mass")
        if not all(math.isfinite(value) and value >= 0.0 for value in (self.heat_load, self.peak_heat_rate)):
            raise ValueError("rigid-body thermal state must be finite and nonnegative")
        ####

    def to_values(self) -> tuple[float, ...]:
        """Pack the state in the stable runtime integration order."""

        return (
            self.position.vector.x,
            self.position.vector.y,
            self.position.vector.z,
            self.velocity.vector.x,
            self.velocity.vector.y,
            self.velocity.vector.z,
            self.attitude.w,
            self.attitude.x,
            self.attitude.y,
            self.attitude.z,
            self.body_rate.x,
            self.body_rate.y,
            self.body_rate.z,
            self.mass,
            self.propellant_mass,
            self.heat_load,
            self.peak_heat_rate,
        )
        ####

    @classmethod
    def from_values(cls, time: float, values: Sequence[float]) -> RigidBody6DofState:
        """Unpack a runtime vector and normalize the integrated quaternion."""

        if len(values) != len(RIGID_BODY_STATE_NAMES):
            raise ValueError(f"rigid-body state requires {len(RIGID_BODY_STATE_NAMES)} values")
        return cls(
            time,
            FrameVector3(Vector3(*values[0:3]), Frame.ECIC),
            FrameVector3(Vector3(*values[3:6]), Frame.ECIC),
            Quaternion(*values[6:10]).normalized(),
            Vector3(*values[10:13]),
            values[13],
            values[14],
            values[15],
            values[16],
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class RigidBodyForceMoment:
    """Body-frame force and moment plus mass-flow and heating observables."""

    force_body: Vector3
    moment_body: Vector3
    propellant_mass_rate: float = 0.0
    heat_rate: float = 0.0
    aero_force_body: Vector3 | None = None
    propulsion_force_body: Vector3 | None = None
    aero_moment_body: Vector3 | None = None
    propulsion_moment_body: Vector3 | None = None

    def __post_init__(self) -> None:
        if self.propellant_mass_rate < 0.0 or not math.isfinite(self.propellant_mass_rate):
            raise ValueError("propellant mass rate must be finite and nonnegative")
        if self.heat_rate < 0.0 or not math.isfinite(self.heat_rate):
            raise ValueError("heat rate must be finite and nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ThermalLimits:
    """Hard limits used by an entry controller or event policy."""

    maximum_heat_rate: float
    maximum_heat_load: float
    maximum_dynamic_pressure: float | None = None

    def __post_init__(self) -> None:
        if self.maximum_heat_rate <= 0.0 or self.maximum_heat_load <= 0.0:
            raise ValueError("thermal limits must be positive")
        if self.maximum_dynamic_pressure is not None and self.maximum_dynamic_pressure <= 0.0:
            raise ValueError("dynamic-pressure limit must be positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ThermalAssessment:
    """Current margins for heat-rate and integrated heat-load limits."""

    heat_rate_margin: float
    heat_load_margin: float
    safe: bool


ForceMomentProvider = Callable[[RigidBody6DofState], RigidBodyForceMoment]
GravityProvider = Callable[[RigidBody6DofState], Vector3]


@dataclass(frozen=True, slots=True)
class RigidBody6DofModel:
    """Evaluate a diagonal-inertia rigid-body state derivative.

    Forces and moments are supplied in body coordinates. Gravity is supplied
    in ECIC coordinates. This keeps the model usable for stage, coast, entry,
    and terminal-guidance phases while leaving vehicle-specific aerodynamics
    and propulsion outside the integrator.
    """

    inertia: Vector3
    force_moment: ForceMomentProvider
    gravity: GravityProvider = lambda state: Vector3(0.0, 0.0, 0.0)
    dry_mass: float | None = None

    def __post_init__(self) -> None:
        if min(self.inertia.x, self.inertia.y, self.inertia.z) <= 0.0:
            raise ValueError("all principal moments of inertia must be positive")
        if self.dry_mass is not None and self.dry_mass <= 0.0:
            raise ValueError("dry mass must be positive")
        ####

    def _load(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Resolve a load and stop propellant flow at dry-mass depletion."""

        load = self.force_moment(state)
        if self.dry_mass is None or (
            state.propellant_mass > 1.0e-8
            and state.mass > self.dry_mass + 1.0e-8
        ):
            return load
        return RigidBodyForceMoment(
            load.force_body,
            load.moment_body,
            0.0,
            load.heat_rate,
            load.aero_force_body,
            load.propulsion_force_body,
            load.aero_moment_body,
            load.propulsion_moment_body,
        )
        ####

    def derivative(self, state: RigidBody6DofState) -> tuple[float, ...]:
        """Return the full derivative in ``RIGID_BODY_STATE_NAMES`` order."""

        load = self._load(state)
        gravity = self.gravity(state)
        force_ecic = state.attitude.rotate(load.force_body) + gravity.scaled(state.mass)
        acceleration = force_ecic.scaled(1.0 / state.mass)
        angular_momentum = Vector3(
            self.inertia.x * state.body_rate.x,
            self.inertia.y * state.body_rate.y,
            self.inertia.z * state.body_rate.z,
        )
        angular_acceleration = Vector3(
            (load.moment_body.x - state.body_rate.cross(angular_momentum).x) / self.inertia.x,
            (load.moment_body.y - state.body_rate.cross(angular_momentum).y) / self.inertia.y,
            (load.moment_body.z - state.body_rate.cross(angular_momentum).z) / self.inertia.z,
        )
        quaternion_rate = state.attitude.derivative(state.body_rate)
        return (
            state.velocity.vector.x,
            state.velocity.vector.y,
            state.velocity.vector.z,
            acceleration.x,
            acceleration.y,
            acceleration.z,
            quaternion_rate.w * 0.5,
            quaternion_rate.x * 0.5,
            quaternion_rate.y * 0.5,
            quaternion_rate.z * 0.5,
            angular_acceleration.x,
            angular_acceleration.y,
            angular_acceleration.z,
            -load.propellant_mass_rate,
            -load.propellant_mass_rate,
            load.heat_rate,
            max(0.0, load.heat_rate - state.peak_heat_rate),
        )
        ####

    def observables(self, state: RigidBody6DofState) -> dict[str, float]:
        """Return force, moment, acceleration, heating, and attitude channels.

        These values are derived from the same force/moment evaluation used by
        :meth:`derivative`, so runtime telemetry and the integrated state do
        not require a second physics implementation.
        """

        load = self._load(state)
        gravity = self.gravity(state)
        force_ecic = state.attitude.rotate(load.force_body)
        total_force_ecic = force_ecic + gravity.scaled(state.mass)
        acceleration = total_force_ecic.scaled(1.0 / state.mass)
        gravity_force_body = state.attitude.conjugate().rotate(gravity.scaled(state.mass))
        total_force_body = load.force_body + gravity_force_body
        acceleration_body = state.attitude.conjugate().rotate(acceleration)
        # ``acceleration_body`` is the inertial acceleration resolved in body
        # axes, not the time derivative of the body velocity components.  The
        # latter would require the additional omega-cross-velocity term.  The
        # Newton equation in this observable contract is therefore the direct
        # body-resolved inertial acceleration balance:
        #
        #     m R_BI a_I - (F_aero,B + F_prop,B + F_gravity,B) = 0.
        #
        # Keeping this distinct prevents rotating-frame transport terms from
        # being counted twice in closure telemetry.
        force_residual = acceleration_body.scaled(state.mass) - total_force_body
        attitude = state.attitude.normalized()
        roll = math.atan2(2.0 * (attitude.w * attitude.x + attitude.y * attitude.z), 1.0 - 2.0 * (attitude.x * attitude.x + attitude.y * attitude.y))
        pitch_argument = 2.0 * (attitude.w * attitude.y - attitude.z * attitude.x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(2.0 * (attitude.w * attitude.z + attitude.x * attitude.y), 1.0 - 2.0 * (attitude.y * attitude.y + attitude.z * attitude.z))
        aero_force = load.aero_force_body or Vector3(0.0, 0.0, 0.0)
        propulsion_force = load.propulsion_force_body or (load.force_body - aero_force)
        aero_moment = load.aero_moment_body or Vector3(0.0, 0.0, 0.0)
        propulsion_moment = load.propulsion_moment_body or (load.moment_body - aero_moment)
        angular_momentum = Vector3(self.inertia.x * state.body_rate.x, self.inertia.y * state.body_rate.y, self.inertia.z * state.body_rate.z)
        angular_acceleration = Vector3(
            (load.moment_body.x - state.body_rate.cross(angular_momentum).x) / self.inertia.x,
            (load.moment_body.y - state.body_rate.cross(angular_momentum).y) / self.inertia.y,
            (load.moment_body.z - state.body_rate.cross(angular_momentum).z) / self.inertia.z,
        )
        moment_residual = Vector3(
            self.inertia.x * angular_acceleration.x,
            self.inertia.y * angular_acceleration.y,
            self.inertia.z * angular_acceleration.z,
        ) + state.body_rate.cross(angular_momentum) - load.moment_body
        force_scale = max(state.mass * gravity.norm(), total_force_body.norm(), 1.0e-12)
        moment_scale = max(load.moment_body.norm(), 1.0)
        return {
            "force_body_x_n": load.force_body.x,
            "force_body_y_n": load.force_body.y,
            "force_body_z_n": load.force_body.z,
            "moment_body_x_nm": load.moment_body.x,
            "moment_body_y_nm": load.moment_body.y,
            "moment_body_z_nm": load.moment_body.z,
            "force_ecic_x_n": force_ecic.x,
            "force_ecic_y_n": force_ecic.y,
            "force_ecic_z_n": force_ecic.z,
            "total_force_ecic_n": total_force_ecic.norm(),
            "aero_force_body_x_n": aero_force.x,
            "aero_force_body_y_n": aero_force.y,
            "aero_force_body_z_n": aero_force.z,
            "propulsion_force_body_x_n": propulsion_force.x,
            "propulsion_force_body_y_n": propulsion_force.y,
            "propulsion_force_body_z_n": propulsion_force.z,
            "gravity_force_body_x_n": gravity_force_body.x,
            "gravity_force_body_y_n": gravity_force_body.y,
            "gravity_force_body_z_n": gravity_force_body.z,
            "total_force_body_x_n": total_force_body.x,
            "total_force_body_y_n": total_force_body.y,
            "total_force_body_z_n": total_force_body.z,
            "aero_moment_body_x_nm": aero_moment.x,
            "aero_moment_body_y_nm": aero_moment.y,
            "aero_moment_body_z_nm": aero_moment.z,
            "propulsion_moment_body_x_nm": propulsion_moment.x,
            "propulsion_moment_body_y_nm": propulsion_moment.y,
            "propulsion_moment_body_z_nm": propulsion_moment.z,
            "total_moment_body_x_nm": load.moment_body.x,
            "total_moment_body_y_nm": load.moment_body.y,
            "total_moment_body_z_nm": load.moment_body.z,
            "translation_equation_residual_n": force_residual.norm(),
            "translation_equation_residual_normalized": force_residual.norm() / force_scale,
            "rotation_equation_residual_nm": moment_residual.norm(),
            "rotation_equation_residual_normalized": moment_residual.norm() / moment_scale,
            "acceleration_ecic_x_m_s2": acceleration.x,
            "acceleration_ecic_y_m_s2": acceleration.y,
            "acceleration_ecic_z_m_s2": acceleration.z,
            "propellant_mass_rate_kg_s": load.propellant_mass_rate,
            "heat_rate_w_m2": load.heat_rate,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
        }
        ####

    def runtime_derivative(self) -> Callable[[object], tuple[float, ...]]:
        """Return a callback compatible with ``RuntimeVehicle.derivative``."""

        def evaluate(runtime_state: object) -> tuple[float, ...]:
            time = float(getattr(runtime_state, "time"))
            values = getattr(runtime_state, "values")
            return self.derivative(RigidBody6DofState.from_values(time, values))
        ####

        return evaluate
        ####
    ####


def assess_thermal_limits(state: RigidBody6DofState, limits: ThermalLimits, heat_rate: float) -> ThermalAssessment:
    """Return explicit thermal margins for guidance and event logic."""

    rate_margin = limits.maximum_heat_rate - heat_rate
    load_margin = limits.maximum_heat_load - state.heat_load
    return ThermalAssessment(rate_margin, load_margin, rate_margin >= 0.0 and load_margin >= 0.0)
####
