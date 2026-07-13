"""Frame-aware vector and basis contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from .units import Unit


class Frame(StrEnum):
    """Coordinate frames named by the TAOS manual and its planned runtime."""

    ECFC = "ecfc"
    ECIC = "ecic"
    GEOCENTRIC_HORIZON = "geocentric_horizon"
    GEODETIC_HORIZON = "geodetic_horizon"
    BODY = "body"
    WIND = "wind"
    TANGENT_PLANE = "tangent_plane"
    INERTIAL_PLATFORM = "inertial_platform"
    VELOCITY_EARTH = "velocity_earth"
####


@dataclass(frozen=True, slots=True)
class Vector3:
    """Immutable Cartesian vector with named components."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("vector components must be finite")
        ####
    ####

    def __add__(self, other: Vector3) -> Vector3:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    ####

    def __sub__(self, other: Vector3) -> Vector3:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    ####

    def scaled(self, factor: float) -> Vector3:
        return Vector3(self.x * factor, self.y * factor, self.z * factor)
    ####

    def dot(self, other: Vector3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z
    ####

    def cross(self, other: Vector3) -> Vector3:
        return Vector3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
    ####

    def norm(self) -> float:
        return math.sqrt(self.dot(self))
    ####


@dataclass(frozen=True, slots=True)
class FrameVector3:
    """A vector whose frame is part of its type-level runtime contract."""

    vector: Vector3
    frame: Frame

    def add(self, other: FrameVector3) -> FrameVector3:
        if self.frame is not other.frame:
            raise ValueError(f"cannot add vectors in {self.frame} and {other.frame}")
        return FrameVector3(self.vector + other.vector, self.frame)
    ####


@dataclass(frozen=True, slots=True)
class FrameQuantityVector3:
    """A frame-aware Cartesian vector whose components share one unit."""

    vector: Vector3
    frame: Frame
    unit: Unit

    def to(self, unit: Unit) -> FrameQuantityVector3:
        if self.unit.dimension != unit.dimension:
            raise ValueError(f"cannot convert {self.unit.dimension} to {unit.dimension}")
        scale = self.unit.scale_to_si / unit.scale_to_si
        return FrameQuantityVector3(self.vector.scaled(scale), self.frame, unit)
    ####


@dataclass(frozen=True, slots=True)
class Basis3:
    """Three child-frame basis vectors expressed in one parent frame."""

    first: Vector3
    second: Vector3
    third: Vector3
    parent_frame: Frame
    child_frame: Frame

    def is_orthonormal(self, *, tolerance: float = 1e-12) -> bool:
        vectors = (self.first, self.second, self.third)
        return all(math.isclose(vector.norm(), 1.0, abs_tol=tolerance, rel_tol=0.0) for vector in vectors) and all(
            math.isclose(left.dot(right), 0.0, abs_tol=tolerance, rel_tol=0.0)
            for index, left in enumerate(vectors)
            for right in vectors[index + 1 :]
        )
    ####
####
