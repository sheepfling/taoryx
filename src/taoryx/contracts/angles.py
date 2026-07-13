"""Explicit angle types and deterministic wrapping conventions."""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(radians: float, *, lower: float = -math.pi) -> float:
    """Wrap an angle into ``[lower, lower + 2*pi)``."""

    if not math.isfinite(radians) or not math.isfinite(lower):
        raise ValueError("angle values must be finite")
    return (radians - lower) % math.tau + lower
####


@dataclass(frozen=True, slots=True)
class Angle:
    """An angle stored in radians."""

    radians: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.radians):
            raise ValueError("angle must be finite")
        ####
    ####

    def wrapped(self) -> Angle:
        """Return the canonical wrapped representation in ``[-pi, pi)``."""

        return type(self)(wrap_angle(self.radians))
    ####


@dataclass(frozen=True, slots=True)
class Longitude(Angle):
    """East-positive longitude in radians."""

    def wrapped(self) -> Longitude:
        """Return longitude wrapped independently into ``[-pi, pi)``."""

        return Longitude(wrap_angle(self.radians))
####


@dataclass(frozen=True, slots=True)
class Latitude(Angle):
    """Latitude in radians, constrained to the closed physical range."""

    def __post_init__(self) -> None:
        Angle.__post_init__(self)
        if not -math.pi / 2 <= self.radians <= math.pi / 2:
            raise ValueError("latitude must be within [-pi/2, pi/2]")
        ####
    ####

    def wrapped(self) -> Latitude:
        """Reject independent wrapping, which can change the represented direction."""

        raise TypeError("latitude requires coupled longitude/latitude normalization")
    ####


@dataclass(frozen=True, slots=True)
class Heading(Angle):
    """Local horizontal heading angle in radians."""
####


@dataclass(frozen=True, slots=True)
class FlightPathAngle(Angle):
    """Flight-path angle in radians."""
####


def normalize_longitude_latitude(longitude_radians: float, latitude_radians: float) -> tuple[Longitude, Latitude]:
    """Normalize a spherical direction without separating longitude and latitude.

    The pair is projected to a unit Cartesian direction and converted back, so
    crossing a pole changes longitude by pi while reflecting latitude. At a
    pole the longitude is conventionally selected as zero because it is
    geometrically undefined.
    """

    if not math.isfinite(longitude_radians) or not math.isfinite(latitude_radians):
        raise ValueError("spherical angles must be finite")
    cosine_latitude = math.cos(latitude_radians)
    x = cosine_latitude * math.cos(longitude_radians)
    y = cosine_latitude * math.sin(longitude_radians)
    z = math.sin(latitude_radians)
    latitude = math.asin(max(-1.0, min(1.0, z)))
    horizontal = math.hypot(x, y)
    longitude = 0.0 if horizontal <= 1e-15 else math.atan2(y, x)
    return Longitude(longitude), Latitude(latitude)
####
