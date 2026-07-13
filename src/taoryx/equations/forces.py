"""Aerodynamic and propulsive force helpers."""

from __future__ import annotations

import math

from .frames import BodyAxes, WindAxes
from .geodesy import CartesianVector3


def body_windward_meridian_unit_vector(wind_axes: WindAxes, body_axes: BodyAxes) -> CartesianVector3:
    """Return the body-frame windward-meridian unit vector."""

    dot_y = wind_axes.x.x * body_axes.y.x + wind_axes.x.y * body_axes.y.y + wind_axes.x.z * body_axes.y.z
    dot_z = wind_axes.x.x * body_axes.z.x + wind_axes.x.y * body_axes.z.y + wind_axes.x.z * body_axes.z.z
    magnitude = math.hypot(dot_y, dot_z)
    if magnitude == 0.0:
        raise ValueError("windward meridian unit vector is undefined when total angle of attack is zero")
    ####
    return CartesianVector3(
        (dot_y * body_axes.y.x + dot_z * body_axes.z.x) / magnitude,
        (dot_y * body_axes.y.y + dot_z * body_axes.z.y) / magnitude,
        (dot_y * body_axes.y.z + dot_z * body_axes.z.z) / magnitude,
    )
####


def aerodynamic_force_from_axial_and_normal_coefficients(
    wind_axes: WindAxes,
    body_axes: BodyAxes,
    dynamic_pressure: float,
    reference_area: float,
    axial_coefficient: float,
    normal_coefficient: float,
) -> CartesianVector3:
    """Return the aerodynamic force from axial and normal coefficients."""

    force_scale = dynamic_pressure * reference_area
    windward_meridian = body_windward_meridian_unit_vector(wind_axes, body_axes)
    return CartesianVector3(
        -(axial_coefficient * body_axes.x.x + normal_coefficient * windward_meridian.x) * force_scale,
        -(axial_coefficient * body_axes.x.y + normal_coefficient * windward_meridian.y) * force_scale,
        -(axial_coefficient * body_axes.x.z + normal_coefficient * windward_meridian.z) * force_scale,
    )
####


def aerodynamic_force_from_lift_drag_side_coefficients(
    wind_axes: WindAxes,
    dynamic_pressure: float,
    reference_area: float,
    lift_coefficient: float,
    drag_coefficient: float,
    side_force_coefficient: float,
) -> CartesianVector3:
    """Return the aerodynamic force from lift, drag, and side-force coefficients."""

    force_scale = dynamic_pressure * reference_area
    return CartesianVector3(
        -(drag_coefficient * wind_axes.x.x - side_force_coefficient * wind_axes.y.x + lift_coefficient * wind_axes.z.x) * force_scale,
        -(drag_coefficient * wind_axes.x.y - side_force_coefficient * wind_axes.y.y + lift_coefficient * wind_axes.z.y) * force_scale,
        -(drag_coefficient * wind_axes.x.z - side_force_coefficient * wind_axes.y.z + lift_coefficient * wind_axes.z.z) * force_scale,
    )
####


def aerodynamic_force_from_body_axis_coefficients(
    body_axes: BodyAxes,
    dynamic_pressure: float,
    reference_area: float,
    x_coefficient: float,
    y_coefficient: float,
    z_coefficient: float,
) -> CartesianVector3:
    """Return the aerodynamic force from body-axis coefficients."""

    force_scale = dynamic_pressure * reference_area
    return CartesianVector3(
        (x_coefficient * body_axes.x.x + y_coefficient * body_axes.y.x + z_coefficient * body_axes.z.x) * force_scale,
        (x_coefficient * body_axes.x.y + y_coefficient * body_axes.y.y + z_coefficient * body_axes.z.y) * force_scale,
        (x_coefficient * body_axes.x.z + y_coefficient * body_axes.y.z + z_coefficient * body_axes.z.z) * force_scale,
    )
####


def propulsive_force_vector(
    thrust_magnitude: float,
    epsilon_1_radians: float,
    epsilon_2_radians: float,
) -> CartesianVector3:
    """Return the body-frame propulsive force vector."""

    cosine_ep1 = math.cos(epsilon_1_radians)
    sine_ep1 = math.sin(epsilon_1_radians)
    return CartesianVector3(
        thrust_magnitude * cosine_ep1,
        -thrust_magnitude * sine_ep1 * math.cos(epsilon_2_radians),
        -thrust_magnitude * sine_ep1 * math.sin(epsilon_2_radians),
    )
####


def specific_load_vector_from_accelerations(
    inertial_acceleration: CartesianVector3,
    gravity_acceleration: CartesianVector3,
) -> CartesianVector3:
    """Return the specific-load vector from inertial and gravity accelerations."""

    return CartesianVector3(
        inertial_acceleration.x - gravity_acceleration.x,
        inertial_acceleration.y - gravity_acceleration.y,
        inertial_acceleration.z - gravity_acceleration.z,
    )
####


def specific_load_vector_from_forces(
    aerodynamic_force: CartesianVector3,
    propulsive_force: CartesianVector3,
    mass: float,
) -> CartesianVector3:
    """Return the specific-load vector from aerodynamic and propulsive forces."""

    if mass == 0.0:
        raise ValueError("mass must be nonzero")
    ####
    return CartesianVector3(
        (aerodynamic_force.x + propulsive_force.x) / mass,
        (aerodynamic_force.y + propulsive_force.y) / mass,
        (aerodynamic_force.z + propulsive_force.z) / mass,
    )
####


def specific_load_components(
    specific_load_vector: CartesianVector3,
    body_axes: BodyAxes,
) -> tuple[float, float, float, float]:
    """Return the axial, lateral, and total specific-load magnitudes."""

    nx = specific_load_vector.x * body_axes.x.x + specific_load_vector.y * body_axes.x.y + specific_load_vector.z * body_axes.x.z
    ny = specific_load_vector.x * body_axes.y.x + specific_load_vector.y * body_axes.y.y + specific_load_vector.z * body_axes.y.z
    nz = specific_load_vector.x * body_axes.z.x + specific_load_vector.y * body_axes.z.y + specific_load_vector.z * body_axes.z.z
    ntotal = math.sqrt(
        specific_load_vector.x * specific_load_vector.x
        + specific_load_vector.y * specific_load_vector.y
        + specific_load_vector.z * specific_load_vector.z
    )
    return nx, ny, nz, ntotal
####


def normal_specific_load_magnitude(
    specific_load_vector: CartesianVector3,
    body_axes: BodyAxes,
) -> float:
    """Return the normal, or lateral, specific-load magnitude."""

    _, ny, nz, _ = specific_load_components(specific_load_vector, body_axes)
    return math.sqrt(ny * ny + nz * nz)
####


def full_table_thrust(vacuum_thrust: float, ambient_pressure: float, nozzle_exit_area: float) -> float:
    """Return thrust as vacuum thrust minus ambient pressure times nozzle exit area."""

    return vacuum_thrust - ambient_pressure * nozzle_exit_area
####
