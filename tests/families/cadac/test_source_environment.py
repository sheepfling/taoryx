from __future__ import annotations

import math

import pytest
from taoryx.families.cadac.aim5 import atmosphere76 as aim5_atmosphere76
from taoryx.families.cadac.source_environment import (
    CADAC_SOURCE_EARTH_RADIUS_M,
    atmosphere76,
    cadac_source_inverse_square_gravity_mps2,
)


def test_source_atmosphere_matches_legacy_sea_level_reference() -> None:
    density, pressure, temperature = atmosphere76(0.0)

    assert density == pytest.approx(1.225)
    assert pressure == pytest.approx(101_325.0)
    assert temperature == pytest.approx(288.15)
    assert aim5_atmosphere76(0.0) == pytest.approx((density, pressure, temperature))
    ####


def test_source_inverse_square_gravity_is_finite_and_decreases_with_altitude() -> None:
    sea_level = cadac_source_inverse_square_gravity_mps2(0.0)
    high_altitude = cadac_source_inverse_square_gravity_mps2(100_000.0)

    assert sea_level == pytest.approx(9.819743861784135)
    assert math.isfinite(high_altitude)
    assert 0.0 < high_altitude < sea_level
    ####


@pytest.mark.parametrize("altitude_m", (math.nan, math.inf, -math.inf))
def test_source_environment_rejects_nonfinite_altitudes(altitude_m: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        atmosphere76(altitude_m)
    with pytest.raises(ValueError, match="finite"):
        cadac_source_inverse_square_gravity_mps2(altitude_m)
    ####


def test_source_gravity_rejects_nonpositive_radius() -> None:
    with pytest.raises(ValueError, match="radius"):
        cadac_source_inverse_square_gravity_mps2(-CADAC_SOURCE_EARTH_RADIUS_M)
    ####
