"""Reusable kinematic racetrack reference and tracking commands."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from taoryx.contracts import Vector3


class RacetrackPhase(StrEnum):
    STRAIGHT_LOW = "straight-low"
    TURN_HIGH = "turn-high"
    STRAIGHT_HIGH = "straight-high"
    TURN_LOW = "turn-low"
####


@dataclass(frozen=True, slots=True)
class RacetrackSpec:
    """Planar racetrack dimensions and nominal ground speed."""

    straight_length: float
    turn_radius: float
    speed: float

    def __post_init__(self) -> None:
        if min(self.straight_length, self.turn_radius, self.speed) <= 0.0:
            raise ValueError("racetrack dimensions and speed must be positive")
        ####
    ####

    @property
    def perimeter(self) -> float:
        return 2.0 * self.straight_length + 2.0 * math.pi * self.turn_radius
    ####

    @property
    def period(self) -> float:
        return self.perimeter / self.speed
    ####
####


@dataclass(frozen=True, slots=True)
class RacetrackReference:
    position: Vector3
    velocity: Vector3
    phase: RacetrackPhase
    lap: int
####


@dataclass(frozen=True, slots=True)
class RacetrackCommand:
    """Air-relative velocity command after environmental wind compensation."""

    air_velocity: Vector3
    ground_velocity: Vector3
    phase: RacetrackPhase
    lap: int
    cross_track_error: float
####


@dataclass(frozen=True, slots=True)
class RacetrackController:
    """Generate a bounded-error velocity command around a planar racetrack."""

    spec: RacetrackSpec
    position_gain: float = 0.4

    def __post_init__(self) -> None:
        if not math.isfinite(self.position_gain) or self.position_gain <= 0.0:
            raise ValueError("racetrack position gain must be positive and finite")
        ####
    ####

    def reference(self, time: float) -> RacetrackReference:
        if not math.isfinite(time) or time < 0.0:
            raise ValueError("racetrack time must be finite and nonnegative")
        distance = (time * self.spec.speed) % self.spec.perimeter
        lap = math.floor(time / self.spec.period)
        half_length = self.spec.straight_length / 2.0
        radius = self.spec.turn_radius
        if distance < self.spec.straight_length:
            return RacetrackReference(Vector3(-half_length + distance, -radius, 0.0), Vector3(self.spec.speed, 0.0, 0.0), RacetrackPhase.STRAIGHT_LOW, lap)
        distance -= self.spec.straight_length
        arc_length = math.pi * radius
        if distance < arc_length:
            angle = -math.pi / 2.0 + distance / radius
            return RacetrackReference(
                Vector3(half_length + radius * math.cos(angle), radius * math.sin(angle), 0.0),
                Vector3(-self.spec.speed * math.sin(angle), self.spec.speed * math.cos(angle), 0.0),
                RacetrackPhase.TURN_HIGH,
                lap,
            )
        distance -= arc_length
        if distance < self.spec.straight_length:
            return RacetrackReference(Vector3(half_length - distance, radius, 0.0), Vector3(-self.spec.speed, 0.0, 0.0), RacetrackPhase.STRAIGHT_HIGH, lap)
        distance -= self.spec.straight_length
        angle = math.pi / 2.0 + distance / radius
        return RacetrackReference(
            Vector3(-half_length + radius * math.cos(angle), radius * math.sin(angle), 0.0),
            Vector3(-self.spec.speed * math.sin(angle), self.spec.speed * math.cos(angle), 0.0),
            RacetrackPhase.TURN_LOW,
            lap,
        )
    ####

    def command(self, time: float, position: Vector3, wind: Vector3 = Vector3(0.0, 0.0, 0.0)) -> RacetrackCommand:
        target = self.reference(time)
        error = target.position - position
        correction = error.scaled(self.position_gain)
        ground_velocity = target.velocity + correction
        cross_track_error = math.hypot(error.x, error.y)
        return RacetrackCommand(ground_velocity - wind, ground_velocity, target.phase, target.lap, cross_track_error)
    ####
####
