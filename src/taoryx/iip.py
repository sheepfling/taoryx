"""Initial-impact-point derivative contracts."""

from __future__ import annotations

from dataclasses import dataclass

from .contracts import Angle, Frame, FrameQuantityVector3, FrameVector3, GeodeticCoordinates, Quantity, Vector3
from .coordinates import ecfc_position_to_geodetic
from .earth import EllipsoidParameters
from .equations import CartesianVector3, iip_aerodynamic_acceleration_vector
from .geodesy import sodano_inverse
from .integration import rkf45_step
from .simulation import DerivativeModel, SimulationState


@dataclass(frozen=True, slots=True)
class IIPState:
    """Position and velocity for the auxiliary ballistic IIP trajectory."""

    position: CartesianVector3
    velocity: CartesianVector3
####


@dataclass(frozen=True, slots=True)
class IIPDerivatives:
    """Position and velocity derivatives in the same explicitly named frame."""

    position_rate: CartesianVector3
    velocity_rate: CartesianVector3
####


@dataclass(frozen=True, slots=True)
class InitialImpactPointResult:
    """Impact state and range-safety observables from IIP propagation."""

    impact_time_seconds: float
    impact_position: FrameQuantityVector3
    impact_coordinates: GeodeticCoordinates
    range: Quantity
    azimuth: Angle
    steps: int
####


def iip_derivatives(
    state: IIPState,
    gravity_acceleration: CartesianVector3,
    air_density: float,
    ballistic_coefficient: float,
    wind_velocity: CartesianVector3,
) -> IIPDerivatives:
    """Evaluate TAOS-ALG-IIP-001 and equations 2-266 through 2-270.

    The equation helpers intentionally preserve the manual's printed positive
    wind-direction sign for the simplified aerodynamic term.
    """

    air_relative_velocity = CartesianVector3(
        state.velocity.x - wind_velocity.x,
        state.velocity.y - wind_velocity.y,
        state.velocity.z - wind_velocity.z,
    )
    gravity_magnitude = (gravity_acceleration.x**2 + gravity_acceleration.y**2 + gravity_acceleration.z**2) ** 0.5
    aerodynamic = iip_aerodynamic_acceleration_vector(
        gravity_magnitude,
        air_density,
        ballistic_coefficient,
        air_relative_velocity,
    )
    return IIPDerivatives(
        state.velocity,
        CartesianVector3(
            gravity_acceleration.x + aerodynamic.x,
            gravity_acceleration.y + aerodynamic.y,
            gravity_acceleration.z + aerodynamic.z,
        ),
    )
####


def initial_impact_point(
    position: FrameQuantityVector3,
    velocity: FrameVector3,
    derivative_model: DerivativeModel,
    impact_altitude: Quantity,
    tangent_plane_origin: GeodeticCoordinates,
    parameters: EllipsoidParameters,
    *,
    initial_step: float = 10.0,
    absolute_tolerance: float = 1e-3,
    relative_tolerance: float = 1e-8,
    max_steps: int = 10000,
) -> InitialImpactPointResult:
    """Evaluate TAOS-ALG-IIP-003 by propagating to a geodetic altitude."""

    if position.frame is not Frame.ECFC or velocity.frame is not Frame.ECFC:
        raise ValueError("IIP position and velocity must be expressed in ECFC")
    target_altitude = impact_altitude.to(position.unit)
    current_coordinates = ecfc_position_to_geodetic(position, parameters)
    if current_coordinates.altitude.value <= target_altitude.value:
        inverse = sodano_inverse(tangent_plane_origin, current_coordinates, parameters)
        return InitialImpactPointResult(
            0.0,
            position,
            current_coordinates,
            inverse.distance,
            inverse.forward_azimuth,
            0,
        )
    ####
    state = SimulationState(0.0, (position.vector.x, position.vector.y, position.vector.z, velocity.vector.x, velocity.vector.y, velocity.vector.z), "ecfc")
    step = initial_step
    for count in range(1, max_steps + 1):
        result = rkf45_step(derivative_model, state, step, absolute_tolerance, relative_tolerance)
        state = result.state
        position_vector = FrameQuantityVector3(Vector3(*state.values[:3]), Frame.ECFC, position.unit)
        current_coordinates = ecfc_position_to_geodetic(position_vector, parameters)
        if current_coordinates.altitude.value <= target_altitude.value:
            inverse = sodano_inverse(tangent_plane_origin, current_coordinates, parameters)
            return InitialImpactPointResult(
                state.time,
                position_vector,
                current_coordinates,
                inverse.distance,
                inverse.forward_azimuth,
                count,
            )
        ####
        step = result.next_step
    ####
    raise RuntimeError("IIP propagation exceeded max_steps before impact")
####
