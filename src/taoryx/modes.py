"""Explicit taoryx dynamics modes and kinematic attitude propagation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from .contracts import Frame, FrameVector3, Vector3


class FidelitySetupError(ValueError):
    """Actionable equation-tier setup failure."""

    def __init__(self, code: str, message: str, action: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        self.action = action
        location = f" field={field!r}" if field is not None else ""
        super().__init__(f"[fidelity:{code}]{location} {message} Fix: {action}")


class DynamicsMode(StrEnum):
    """Supported and planned trajectory dynamics modes."""

    POINT_MASS = "point-mass"
    KINEMATIC_6DOF = "kinematic-6dof"
    RIGID_BODY_6DOF = "rigid-body-6dof"
####


@dataclass(frozen=True, slots=True)
class Quaternion:
    """Unit quaternion mapping body vectors into the selected reference frame."""

    w: float
    x: float
    y: float
    z: float

    @classmethod
    def identity(cls) -> Quaternion:
        return cls(1.0, 0.0, 0.0, 0.0)
        ####

    def normalized(self) -> Quaternion:
        magnitude = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if magnitude <= 0.0 or not math.isfinite(magnitude):
            raise ValueError("attitude quaternion must have a finite nonzero norm")
        return Quaternion(self.w / magnitude, self.x / magnitude, self.y / magnitude, self.z / magnitude)
        ####

    def multiply(self, other: Quaternion) -> Quaternion:
        return Quaternion(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )
        ####

    def conjugate(self) -> Quaternion:
        """Return the inverse rotation for this unit quaternion."""

        return Quaternion(self.w, -self.x, -self.y, -self.z)
        ####

    def rotate(self, vector: Vector3) -> Vector3:
        """Map a body-frame vector into the state's reference coordinates."""

        pure = Quaternion(0.0, vector.x, vector.y, vector.z)
        rotated = self.normalized().multiply(pure).multiply(self.normalized().conjugate())
        return Vector3(rotated.x, rotated.y, rotated.z)
        ####

    def derivative(self, body_rate: Vector3) -> Quaternion:
        """Return the quaternion derivative for a body-frame angular rate."""

        return self.multiply(Quaternion(0.0, body_rate.x, body_rate.y, body_rate.z))
        ####

    def integrate_body_rate(self, body_rate: Vector3, step_size: float) -> Quaternion:
        """Propagate attitude using a controller-supplied body rate."""

        if not math.isfinite(step_size) or step_size <= 0.0:
            raise ValueError("attitude step size must be positive and finite")
        derivative = self.derivative(body_rate)
        return Quaternion(
            self.w + 0.5 * step_size * derivative.w,
            self.x + 0.5 * step_size * derivative.x,
            self.y + 0.5 * step_size * derivative.y,
            self.z + 0.5 * step_size * derivative.z,
        ).normalized()
        ####
####


@dataclass(frozen=True, slots=True)
class Kinematic6DofState:
    """Translational state plus attitude for controller-driven 6-DOF motion.

    Angular rates are commands supplied by the attitude controller, not
    moment-derived state derivatives.  This is deliberately distinct from a
    rigid-body 6-DOF model.
    """

    time: float
    position: FrameVector3
    velocity: FrameVector3
    attitude: Quaternion = Quaternion.identity()

    def __post_init__(self) -> None:
        if self.position.frame is not Frame.ECFC or self.velocity.frame is not Frame.ECFC:
            raise FidelitySetupError(
                "kinematic-frame-mismatch",
                "kinematic 6-DOF position and velocity must use ECFC",
                "convert the local state to the declared ECFC frame before constructing the sidecar",
                field="position/velocity",
            )
        if not math.isfinite(self.time):
            raise FidelitySetupError(
                "kinematic-time-nonfinite",
                "kinematic 6-DOF time must be finite",
                "provide a finite integration time in seconds",
                field="time",
            )
        ####

    def with_attitude_rate(self, body_rate: Vector3, step_size: float) -> Kinematic6DofState:
        """Advance only the attitude using the supplied controller rate."""

        return Kinematic6DofState(self.time + step_size, self.position, self.velocity, self.attitude.integrate_body_rate(body_rate, step_size))
        ####

    def with_translation(self, position: FrameVector3, velocity: FrameVector3, time: float) -> Kinematic6DofState:
        """Replace translation after a force integration step while preserving attitude."""

        return Kinematic6DofState(time, position, velocity, self.attitude)
        ####

    def advance(self, position: FrameVector3, velocity: FrameVector3, body_rate: Vector3, step_size: float) -> Kinematic6DofState:
        """Advance force-integrated translation and controller-driven attitude together."""

        return Kinematic6DofState(
            self.time + step_size,
            position,
            velocity,
            self.attitude.integrate_body_rate(body_rate, step_size),
        )
        ####
####
