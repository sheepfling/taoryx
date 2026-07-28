"""Shared setup for canonical rotating-Earth trim fixtures."""

from __future__ import annotations

import math
import re

NOMINAL_EARTH_RATE_RAD_S = 7.2921151467e-5


def earth_tangential_speed_mps(position_x_m: float, earth_omega_rad_s: float) -> float:
    """Return the eastward ECI speed at an equatorial x-axis fixture point."""

    return earth_omega_rad_s * position_x_m


def replace_earth_rate(source: str, earth_omega_rad_s: float) -> str:
    """Replace the declared Earth rate without changing other source fields."""

    return re.sub(
        r"(\*earth\s+[^\n]*?\bomega=)[0-9.eE+-]+",
        rf"\g<1>{earth_omega_rad_s:.16g}",
        source,
        count=1,
    )


def add_ground_rotation_to_initial_velocity(source: str, earth_omega_rad_s: float) -> str:
    """Add Earth ground speed to an east-flight ECIC initial condition.

    The canonical fixtures place the vehicle at ``y=z=0`` on the equator and
    point body-forward east. Their existing ``ydt`` is air-relative speed for
    the zero-rate source case, so the rotating-Earth inertial speed is the
    existing value plus ``omega * x``.
    """

    pattern = re.compile(r"(\*initial\s+ecic[^\n]*\bx=)([0-9.eE+-]+)([^\n]*\bydt=)([0-9.eE+-]+)([^\n]*)")

    def update(match: re.Match[str]) -> str:
        position_x_m = float(match.group(2))
        air_relative_east_mps = float(match.group(4))
        inertial_east_mps = air_relative_east_mps + earth_tangential_speed_mps(position_x_m, earth_omega_rad_s)
        return f"{match.group(1)}{position_x_m:.16g}{match.group(3)}{inertial_east_mps:.16g}{match.group(5)}"

    updated, count = pattern.subn(update, source, count=1)
    if count != 1:
        raise ValueError("rotating-Earth fixture requires one ECIC initial condition with x and ydt")
    return updated


def rotating_fixture_source(source: str, earth_omega_rad_s: float) -> str:
    """Apply the canonical equatorial rotating-Earth source transformation."""

    if not math.isfinite(earth_omega_rad_s):
        raise ValueError("Earth rate must be finite")
    return add_ground_rotation_to_initial_velocity(replace_earth_rate(source, earth_omega_rad_s), earth_omega_rad_s)
