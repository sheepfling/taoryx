"""Coordinate records that make TAOS component order explicit."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from .angles import Latitude, Longitude
from .units import Quantity


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

    def as_tuple(self, order: CoordinateOrder) -> tuple[object, object, object]:
        """Serialize only after the caller chooses a named coordinate order."""

        if order is CoordinateOrder.LONGITUDE_LATITUDE_ALTITUDE:
            return self.longitude, self.latitude, self.altitude
        if order is CoordinateOrder.LATITUDE_LONGITUDE_ALTITUDE:
            return self.latitude, self.longitude, self.altitude
        raise ValueError(f"order {order} is not valid for geodetic coordinates")
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

    def as_tuple(self, order: CoordinateOrder) -> tuple[object, object, object]:
        if order is not CoordinateOrder.RADIUS_LONGITUDE_LATITUDE:
            raise ValueError(f"order {order} is not valid for geocentric coordinates")
        return self.radius, self.longitude, self.latitude
    ####
####
