"""Small external-control laws for TAORYX extension scenarios."""

from __future__ import annotations

import math
from dataclasses import dataclass

from taoryx.contracts import Vector3


@dataclass(frozen=True, slots=True)
class TargetState:
    """Target position and velocity in the declared integration frame."""

    position: Vector3
    velocity: Vector3
    frame: str = "ecic"

    def __post_init__(self) -> None:
        if self.frame.casefold() not in {"ecic", "ecfc"}:
            raise ValueError("target frame must be ecic or ecfc")
        ####
    ####


@dataclass(frozen=True, slots=True)
class GuidanceDemand:
    """Canonical acceleration demand passed from guidance to allocation."""

    acceleration: Vector3
    target: TargetState
    navigation_gain: float | None = None
    saturated: bool = False
    source: str = ""

    def __post_init__(self) -> None:
        if self.navigation_gain is not None and self.navigation_gain <= 0.0:
            raise ValueError("navigation gain must be positive when supplied")
        ####
    ####


@dataclass(frozen=True, slots=True)
class AttitudeCommand:
    """Bounded attitude/rate command consumed by the rigid-body controller."""

    angle_of_attack_radians: float
    bank_radians: float
    body_rate: Vector3 = Vector3(0.0, 0.0, 0.0)
    saturated: bool = False


@dataclass(frozen=True, slots=True)
class ControlOutput:
    """Achieved body load and residual for one control evaluation."""

    force_body: Vector3
    moment_body: Vector3
    demanded_acceleration: Vector3
    achieved_acceleration: Vector3
    residual_acceleration: Vector3
    saturated: bool = False


@dataclass(frozen=True, slots=True)
class PursuitCommand:
    """Wind-compensated air-velocity and throttle request."""

    air_velocity: Vector3
    ground_velocity: Vector3
    throttle: float
    heading_radians: float
    range_to_target: float
####


@dataclass(frozen=True, slots=True)
class GlideCommand:
    """Wind-compensated velocity command with a bounded flight-path angle."""

    air_velocity: Vector3
    ground_velocity: Vector3
    flight_path_angle_radians: float
    range_to_target: float
####


@dataclass(frozen=True, slots=True)
class AlphaBankCommand:
    """Bounded alpha/bank allocation for a velocity-frame acceleration demand."""

    angle_of_attack_radians: float
    bank_radians: float
    demanded_acceleration: Vector3
    achieved_acceleration: Vector3
    residual_acceleration: Vector3
    saturated: bool


def allocate_alpha_bank(
    demanded_acceleration: Vector3,
    *,
    lift_acceleration_per_radian: float,
    maximum_angle_of_attack_radians: float,
    maximum_bank_radians: float = math.pi,
) -> AlphaBankCommand:
    """Allocate a velocity-frame demand into bounded alpha and bank.

    The first axis is axial, the second lateral, and the third normal. This
    is an allocator contract, not an aerodynamic model: the lift slope is an
    explicit resolved input and the resulting command must still pass through
    aerodynamic coefficient tables and actuator dynamics.
    """

    if lift_acceleration_per_radian <= 0.0 or maximum_angle_of_attack_radians < 0.0:
        raise ValueError("lift slope must be positive and alpha limit nonnegative")
    if not 0.0 < maximum_bank_radians <= math.pi:
        raise ValueError("bank limit must be positive and no greater than pi")
    transverse = math.hypot(demanded_acceleration.y, demanded_acceleration.z)
    raw_alpha = transverse / lift_acceleration_per_radian
    alpha = min(maximum_angle_of_attack_radians, raw_alpha)
    lift = lift_acceleration_per_radian * alpha
    raw_bank = math.atan2(demanded_acceleration.y, demanded_acceleration.z) if transverse > 0.0 else 0.0
    bank = max(-maximum_bank_radians, min(maximum_bank_radians, raw_bank))
    achieved = Vector3(
        demanded_acceleration.x,
        lift * math.sin(bank),
        lift * math.cos(bank),
    )
    residual = demanded_acceleration - achieved
    saturated = not math.isclose(alpha, raw_alpha, rel_tol=0.0, abs_tol=1.0e-12) or not math.isclose(bank, raw_bank, rel_tol=0.0, abs_tol=1.0e-12)
    return AlphaBankCommand(alpha, bank, demanded_acceleration, achieved, residual, saturated)
    ####
####


@dataclass(frozen=True, slots=True)
class WindCompensatedPursuit:
    """Pure pursuit law for a point-mass, externally steered vehicle.

    The commanded ground velocity aims at the predicted target position and
    the wind vector is removed before producing the air-relative command. This
    is an extension control law, not a claim about historical TAOS guidance.
    """

    airspeed: float
    prediction_time: float = 0.0
    throttle_reference: float | None = None
    capture_gain: float = 1.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.airspeed) or self.airspeed <= 0.0:
            raise ValueError("pursuit airspeed must be positive and finite")
        if not math.isfinite(self.prediction_time) or self.prediction_time < 0.0:
            raise ValueError("pursuit prediction time must be finite and nonnegative")
        if self.throttle_reference is not None and (not math.isfinite(self.throttle_reference) or self.throttle_reference <= 0.0):
            raise ValueError("pursuit throttle reference must be positive and finite")
        if not math.isfinite(self.capture_gain) or self.capture_gain <= 0.0:
            raise ValueError("pursuit capture gain must be positive and finite")
        ####
    ####

    def command(
        self,
        position: Vector3,
        target_position: Vector3,
        target_velocity: Vector3,
        wind: Vector3 = Vector3(0.0, 0.0, 0.0),
    ) -> PursuitCommand:
        predicted = target_position + target_velocity.scaled(self.prediction_time)
        line_of_sight = predicted - position
        range_to_target = line_of_sight.norm()
        if range_to_target <= 1e-12:
            ground_velocity = target_velocity
        else:
            closing_speed = min(self.airspeed, self.capture_gain * range_to_target)
            ground_velocity = target_velocity + line_of_sight.scaled(closing_speed / range_to_target)
        air_velocity = ground_velocity - wind
        throttle = air_velocity.norm() / (self.throttle_reference or self.airspeed)
        heading = math.atan2(ground_velocity.y, ground_velocity.x)
        return PursuitCommand(air_velocity, ground_velocity, throttle, heading, range_to_target)
    ####
####


@dataclass(frozen=True, slots=True)
class HypersonicGlideTerminal:
    """TAORYX extension law for bounded-angle terminal atmospheric glide."""

    airspeed: float
    maximum_flight_path_angle_radians: float = math.radians(25.0)
    capture_gain: float = 1.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.airspeed) or self.airspeed <= 0.0:
            raise ValueError("glide airspeed must be positive and finite")
        if not 0.0 < self.maximum_flight_path_angle_radians < math.pi / 2.0:
            raise ValueError("glide flight-path angle must lie between zero and ninety degrees")
        if not math.isfinite(self.capture_gain) or self.capture_gain <= 0.0:
            raise ValueError("glide capture gain must be positive and finite")
        ####
    ####

    def command(
        self,
        position: Vector3,
        target_position: Vector3,
        target_velocity: Vector3 = Vector3(0.0, 0.0, 0.0),
        wind: Vector3 = Vector3(0.0, 0.0, 0.0),
    ) -> GlideCommand:
        relative = target_position - position
        range_to_target = relative.norm()
        horizontal_range = math.hypot(relative.x, relative.y)
        if horizontal_range <= 1e-12:
            horizontal_direction = Vector3(1.0, 0.0, 0.0)
        else:
            horizontal_direction = Vector3(relative.x / horizontal_range, relative.y / horizontal_range, 0.0)
        angle = math.atan2(relative.z, max(horizontal_range, 1e-12))
        angle = max(-self.maximum_flight_path_angle_radians, min(self.maximum_flight_path_angle_radians, angle))
        closing_speed = min(self.airspeed, self.capture_gain * range_to_target)
        horizontal_speed = closing_speed * math.cos(angle)
        ground_velocity = target_velocity + Vector3(
            horizontal_direction.x * horizontal_speed,
            horizontal_direction.y * horizontal_speed,
            closing_speed * math.sin(angle),
        )
        return GlideCommand(ground_velocity - wind, ground_velocity, angle, range_to_target)
    ####
####
