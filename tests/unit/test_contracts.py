from __future__ import annotations

import math

import pytest

from taoryx.contracts import (
    AtmosphereLayer,
    AtmosphereModel,
    Basis3,
    CoordinateOrder,
    EarthModel,
    Frame,
    FrameVector3,
    GeocentricCoordinates,
    GeodeticCoordinates,
    Latitude,
    Longitude,
    NumericalTolerances,
    Quantity,
    Unit,
    Vector3,
    normalize_longitude_latitude,
    wrap_angle,
)
from taoryx.linalg import transform_vector


def test_frame_vectors_and_bases_keep_frame_semantics_explicit() -> None:
    left = FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC)
    right = FrameVector3(Vector3(4.0, 5.0, 6.0), Frame.ECFC)

    assert left.add(right).vector == Vector3(5.0, 7.0, 9.0)
    with pytest.raises(ValueError, match="cannot add"):
        left.add(FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC))

    basis = Basis3(
        Vector3(0.0, 1.0, 0.0),
        Vector3(-1.0, 0.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        parent_frame=Frame.ECFC,
        child_frame=Frame.BODY,
    )
    body_vector = FrameVector3(Vector3(3.0, -4.0, 5.0), Frame.BODY)
    parent_vector = transform_vector(body_vector, basis, to_frame=Frame.ECFC)
    assert parent_vector == FrameVector3(Vector3(4.0, 3.0, 5.0), Frame.ECFC)
    assert transform_vector(parent_vector, basis, to_frame=Frame.BODY) == body_vector
####


def test_coordinate_records_require_named_order_and_units() -> None:
    coordinates = GeodeticCoordinates(
        longitude=Longitude(1.0),
        latitude=Latitude(0.5),
        altitude=Quantity(2.0, Unit.KILOMETER),
    )

    assert coordinates.as_tuple(CoordinateOrder.LONGITUDE_LATITUDE_ALTITUDE)[0] == Longitude(1.0)
    assert coordinates.as_tuple(CoordinateOrder.LATITUDE_LONGITUDE_ALTITUDE)[0] == Latitude(0.5)
    spherical = GeocentricCoordinates(Quantity(6378.137, Unit.KILOMETER), Longitude(1.0), Latitude(0.5))
    assert spherical.as_tuple(CoordinateOrder.RADIUS_LONGITUDE_LATITUDE)[0].unit is Unit.KILOMETER
    with pytest.raises(ValueError, match="not valid"):
        coordinates.as_tuple(CoordinateOrder.RADIUS_LONGITUDE_LATITUDE)
    with pytest.raises(ValueError, match="length units"):
        GeodeticCoordinates(Longitude(0.0), Latitude(0.0), Quantity(1.0, Unit.SECOND))
####


def test_units_convert_only_within_a_dimension() -> None:
    assert Quantity(2.0, Unit.KILOMETER).to(Unit.METER).value == 2000.0
    assert math.isclose(Quantity(180.0, Unit.DEGREE).to(Unit.RADIAN).value, math.pi)
    with pytest.raises(ValueError, match="cannot convert"):
        Quantity(1.0, Unit.METER).to(Unit.SECOND)
    with pytest.raises(ValueError, match="finite"):
        Quantity(float("inf"), Unit.METER)
####


def test_earth_and_atmosphere_models_are_immutable_and_ordered() -> None:
    earth = EarthModel(
        equatorial_radius=Quantity(6378.137, Unit.KILOMETER),
        flattening=1.0 / 298.257223563,
        gravitational_parameter=Quantity(3.986004418e14, Unit.METER_CUBED_PER_SECOND_SQUARED),
        rotation_rate=Quantity(7.292115e-5, Unit.RADIAN_PER_SECOND),
    )
    layer = AtmosphereLayer(
        lower_altitude=Quantity(0.0, Unit.METER),
        upper_altitude=Quantity(11.0, Unit.KILOMETER),
        base_temperature=Quantity(288.15, Unit.KELVIN),
        lapse_rate=Quantity(-0.0065, Unit.KELVIN_PER_METER),
        base_pressure=Quantity(101325.0, Unit.PASCAL),
    )
    model = AtmosphereModel((layer,), earth)

    assert model.reference_earth is earth
    with pytest.raises((AttributeError, TypeError)):
        model.layers = ()  # type: ignore[misc]
    with pytest.raises(ValueError, match="contiguous"):
        AtmosphereModel((layer, layer), earth)
####


def test_angle_and_numeric_policies_are_deterministic() -> None:
    assert wrap_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert Longitude(3.0 * math.pi).wrapped() == Longitude(-math.pi)
    longitude, latitude = normalize_longitude_latitude(0.25, math.pi + 0.2)
    assert longitude == Longitude(0.25 + math.pi).wrapped()
    assert latitude.radians == pytest.approx(-0.2)
    with pytest.raises(TypeError, match="coupled"):
        Latitude(0.5).wrapped()
    with pytest.raises(ValueError, match="latitude"):
        Latitude(math.pi)
    with pytest.raises(ValueError, match="max_iterations"):
        NumericalTolerances(max_iterations=0)
####
