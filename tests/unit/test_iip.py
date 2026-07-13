from __future__ import annotations

import pytest

from taoryx.contracts import Frame, FrameQuantityVector3, FrameVector3, GeodeticCoordinates, Latitude, Longitude, Quantity, Unit, Vector3
from taoryx.earth import resolve_ellipsoid_parameters
from taoryx.equations import CartesianVector3
from taoryx.iip import IIPState, iip_derivatives, initial_impact_point


def test_iip_derivatives_add_gravity_and_ballistic_drag() -> None:
    state = IIPState(CartesianVector3(1.0, 2.0, 3.0), CartesianVector3(100.0, 0.0, 0.0))
    result = iip_derivatives(
        state,
        CartesianVector3(0.0, 0.0, -9.8),
        1.2,
        100.0,
        CartesianVector3(0.0, 0.0, 0.0),
    )

    assert result.position_rate == state.velocity
    assert result.velocity_rate.x == pytest.approx(0.5 * 9.8 * 1.2 * 100.0 / 100.0 * 100.0)
    assert result.velocity_rate.z == pytest.approx(-9.8)


def test_initial_impact_point_propagates_to_requested_altitude() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    position = FrameQuantityVector3(Vector3(6378.137 + 1.0, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER)
    velocity = FrameVector3(Vector3(-1.0, 0.0, 0.0), Frame.ECFC)

    def derivative(state):
        return (state.values[3], state.values[4], state.values[5], 0.0, 0.0, 0.0)

    result = initial_impact_point(
        position,
        velocity,
        derivative,
        Quantity(0.0, Unit.KILOMETER),
        GeodeticCoordinates(Longitude(0.0), Latitude(0.0), Quantity(0.0, Unit.KILOMETER)),
        parameters,
        initial_step=0.25,
    )

    assert result.impact_time_seconds > 0.0
    assert result.impact_coordinates.altitude.value <= 0.1
    assert result.steps > 0
