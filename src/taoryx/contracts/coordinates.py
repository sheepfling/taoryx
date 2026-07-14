"""Coordinate records that make TAOS component order explicit."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TypeAlias

from .angles import Latitude, Longitude
from .units import Quantity

LongitudeLatitudeAltitude: TypeAlias = tuple[Longitude, Latitude, Quantity]
LatitudeLongitudeAltitude: TypeAlias = tuple[Latitude, Longitude, Quantity]
RadiusLongitudeLatitude: TypeAlias = tuple[Quantity, Longitude, Latitude]
####


class CoordinateOrder(StrEnum):
    """Named serialization orders; never infer order from a bare tuple."""

    LONGITUDE_LATITUDE_ALTITUDE = "longitude,latitude,altitude"
    LATITUDE_LONGITUDE_ALTITUDE = "latitude,longitude,altitude"
    RADIUS_LONGITUDE_LATITUDE = "radius,longitude,latitude"
####


@dataclass(frozen=True, slots=True)
class GeodeticCoordinates:
    """Geodetic coordinates with explicit longitude/latitude/altitude fields."""

    longitude: Longitude
    latitude: Latitude
    altitude: Quantity

    def __post_init__(self) -> None:
        if self.altitude.unit.dimension != "length":
            raise ValueError("geodetic altitude must have length units")
        ####

    def as_tuple(self, order: CoordinateOrder) -> LongitudeLatitudeAltitude | LatitudeLongitudeAltitude:
        """Serialize only after the caller chooses a named coordinate order."""

        if order is CoordinateOrder.LONGITUDE_LATITUDE_ALTITUDE:
            return self.longitude, self.latitude, self.altitude
        if order is CoordinateOrder.LATITUDE_LONGITUDE_ALTITUDE:
            return self.latitude, self.longitude, self.altitude
        raise ValueError(f"order {order} is not valid for geodetic coordinates")
    ####

    def as_longitude_latitude_altitude(self) -> LongitudeLatitudeAltitude:
        """Return the documented longitude/latitude/altitude ordering."""

        return self.longitude, self.latitude, self.altitude
    ####

    def as_latitude_longitude_altitude(self) -> LatitudeLongitudeAltitude:
        """Return the alternate latitude/longitude/altitude ordering explicitly."""

        return self.latitude, self.longitude, self.altitude
    ####


@dataclass(frozen=True, slots=True)
class GeocentricCoordinates:
    """Spherical coordinates in the documented radius/longitude/latitude order."""

    radius: Quantity
    longitude: Longitude
    latitude: Latitude

    def __post_init__(self) -> None:
        if self.radius.unit.dimension != "length":
            raise ValueError("geocentric radius must have length units")
        ####

    def as_tuple(self, order: CoordinateOrder) -> RadiusLongitudeLatitude:
        if order is not CoordinateOrder.RADIUS_LONGITUDE_LATITUDE:
            raise ValueError(f"order {order} is not valid for geocentric coordinates")
        return self.radius, self.longitude, self.latitude
    ####

    def as_radius_longitude_latitude(self) -> RadiusLongitudeLatitude:
        """Return the canonical radius/longitude/latitude ordering explicitly."""

        return self.radius, self.longitude, self.latitude
    ####
####
