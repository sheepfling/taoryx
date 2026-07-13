"""Typed ellipsoid geometry algorithms from the TAOS catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Latitude, Quantity


@dataclass(frozen=True, slots=True)
class EllipsoidParameters:
    """Equivalent shape parameters for an oblate ellipsoid."""

    equatorial_radius: Quantity
    polar_radius: Quantity
    eccentricity: float
    flattening: float


@dataclass(frozen=True, slots=True)
class EllipsoidalSurfaceGeometry:
    """Surface and normal-intercept geometry at one geodetic latitude."""

    normal_distance: Quantity
    surface_equatorial_radius: Quantity
    surface_polar_coordinate: Quantity
    axis_intercept: Quantity


def resolve_ellipsoid_parameters(
    equatorial_radius: Quantity,
    *,
    eccentricity: float | None = None,
    flattening: float | None = None,
) -> EllipsoidParameters:
    """Resolve equivalent ellipsoid shape parameters from equation 2-18.

    Exactly one of ``eccentricity`` or ``flattening`` is normally supplied.
    Supplying both is supported only when they describe the same ellipsoid.
    """

    if equatorial_radius.unit.dimension != "length":
        raise ValueError("equatorial radius must have length units")
    if equatorial_radius.value <= 0.0:
        raise ValueError("equatorial radius must be positive")
    if eccentricity is None and flattening is None:
        raise ValueError("either eccentricity or flattening must be provided")
    if eccentricity is not None:
        if not math.isfinite(eccentricity) or not 0.0 <= eccentricity < 1.0:
            raise ValueError("eccentricity must be finite and in [0, 1)")
        derived_flattening = 1.0 - math.sqrt(1.0 - eccentricity * eccentricity)
    else:
        derived_flattening = None
    if flattening is not None and (not math.isfinite(flattening) or not 0.0 <= flattening < 1.0):
        raise ValueError("flattening must be finite and in [0, 1)")
    if flattening is not None and derived_flattening is not None and not math.isclose(
        flattening, derived_flattening, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError("eccentricity and flattening are inconsistent")
    if flattening is None:
        assert derived_flattening is not None
        flattening = derived_flattening
    if eccentricity is None:
        eccentricity = math.sqrt(flattening * (2.0 - flattening))
    polar_radius = Quantity(equatorial_radius.value * (1.0 - flattening), equatorial_radius.unit)
    return EllipsoidParameters(equatorial_radius, polar_radius, eccentricity, flattening)
####


def ellipsoidal_surface_geometry(
    parameters: EllipsoidParameters,
    latitude: Latitude,
) -> EllipsoidalSurfaceGeometry:
    """Evaluate equations 2-22 through 2-29 for a geodetic latitude."""

    sine = math.sin(latitude.radians)
    cosine = math.cos(latitude.radians)
    eccentricity_squared = parameters.eccentricity * parameters.eccentricity
    normal_distance_value = parameters.equatorial_radius.value / math.sqrt(1.0 - eccentricity_squared * sine * sine)
    surface_equatorial_radius = normal_distance_value * cosine
    surface_polar_coordinate = normal_distance_value * (1.0 - eccentricity_squared) * sine
    axis_intercept = normal_distance_value * eccentricity_squared * sine
    unit = parameters.equatorial_radius.unit
    return EllipsoidalSurfaceGeometry(
        Quantity(normal_distance_value, unit),
        Quantity(surface_equatorial_radius, unit),
        Quantity(surface_polar_coordinate, unit),
        Quantity(axis_intercept, unit),
    )
####
