"""Problem-level wind resolution."""

from __future__ import annotations

import math
from dataclasses import dataclass

from taoryx.contracts import Frame, FrameVector3, Vector3


@dataclass(frozen=True, slots=True)
class WindResult:
    """Wind in local east/north/down and ECFC components."""

    local: Vector3
    ecfc: FrameVector3


def evaluate_wind(
    *,
    magnitude: float | None = None,
    heading: float | None = None,
    east: float | None = None,
    north: float | None = None,
    down: float = 0.0,
    longitude: float = 0.0,
    latitude: float = 0.0,
) -> WindResult:
    """Resolve speed/heading or E/N/D input and rotate local wind to ECFC.

    Heading is measured clockwise from north. This implements
    ``TAOS-ALG-PRB-012`` and the local-frame convention of equations 2-19--2-21.
    """

    if magnitude is not None or heading is not None:
        if magnitude is None or heading is None:
            raise ValueError("magnitude and heading must be supplied together")
        east_value = magnitude * math.sin(heading)
        north_value = magnitude * math.cos(heading)
    elif east is not None and north is not None:
        east_value, north_value = east, north
    else:
        raise ValueError("wind requires magnitude/heading or east/north components")
    local = Vector3(float(north_value), float(east_value), float(down))
    north_axis = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    east_axis = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    down_axis = Vector3(-math.cos(latitude) * math.cos(longitude), -math.cos(latitude) * math.sin(longitude), -math.sin(latitude))
    ecfc = north_axis.scaled(local.x) + east_axis.scaled(local.y) + down_axis.scaled(local.z)
    return WindResult(local, FrameVector3(ecfc, Frame.ECFC))
####
