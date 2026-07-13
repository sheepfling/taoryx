from __future__ import annotations

import math

from taoryx.contracts import Angle, GeodeticCoordinates, Latitude, Longitude, Quantity, Unit
from taoryx.earth import resolve_ellipsoid_parameters
from taoryx.geodesy import sodano_direct, sodano_inverse


def test_sodano_direct_and_inverse_are_consistent_on_an_ellipsoid() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    start = GeodeticCoordinates(Longitude(0.2), Latitude(0.4), Quantity(0.0, Unit.KILOMETER))
    direct = sodano_direct(start, Quantity(500.0, Unit.KILOMETER), Angle(math.radians(35.0)), parameters)
    inverse = sodano_inverse(start, direct.destination, parameters)

    assert 0.0 < inverse.distance.value < 500.0
    assert math.isfinite(direct.destination.latitude.radians)
    assert math.isfinite(inverse.forward_azimuth.radians)
    assert math.isfinite(inverse.backward_azimuth.radians)
####
