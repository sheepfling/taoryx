from __future__ import annotations

import math

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime import EnvironmentSample, ExponentialAtmosphereProvider, StaticEnvironmentProvider
from taoryx.runtime.environment_runtime import evaluate_wind


def test_speed_heading_wind_resolves_north_and_preserves_ecfc_frame() -> None:
    result = evaluate_wind(magnitude=10.0, heading=0.0, longitude=0.0, latitude=0.0)

    assert result.local == Vector3(10.0, 0.0, 0.0)
    assert result.ecfc == FrameVector3(Vector3(0.0, 0.0, 10.0), Frame.ECFC)
    ####


def test_environment_provider_is_replayable_and_carries_optional_weather() -> None:
    sample = EnvironmentSample(
        density=1.2,
        pressure=101325.0,
        temperature=288.15,
        speed_of_sound=340.3,
        wind=FrameVector3(Vector3(0.0, 5.0, 0.0), Frame.ECFC),
        humidity=0.4,
        cloud_fraction=0.2,
        rain_rate=0.0,
    )
    provider = StaticEnvironmentProvider(sample)

    assert provider.sample(time=12.0, position=FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC)) is sample
    assert math.isclose(provider.sample(time=0.0, position=sample.wind).humidity or 0.0, 0.4)
    ####


def test_evaluate_wind_rejects_mixed_representations() -> None:
    with pytest.raises(ValueError, match="magnitude and heading"):
        evaluate_wind(magnitude=10.0)
    with pytest.raises(ValueError, match="wind requires"):
        evaluate_wind(east=1.0)
    ####


def test_exponential_atmosphere_resolves_altitude_and_vacuum_wind() -> None:
    provider = ExponentialAtmosphereProvider(
        reference_radius_m=6_371_000.0,
        wind=FrameVector3(Vector3(0.0, 35.0, 0.0), Frame.ECFC),
    )
    sea_level = provider.sample(time=0.0, position=FrameVector3(Vector3(6_371_000.0, 0.0, 0.0), Frame.ECFC))
    high_altitude = provider.sample(time=0.0, position=FrameVector3(Vector3(6_591_000.0, 0.0, 0.0), Frame.ECFC))

    assert sea_level.density == pytest.approx(1.225)
    assert sea_level.wind.vector == Vector3(0.0, 35.0, 0.0)
    assert high_altitude.density == 0.0
    assert high_altitude.speed_of_sound > 0.0
    ####
