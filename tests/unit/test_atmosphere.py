from __future__ import annotations

import pytest

from taoryx.atmosphere import (
    AtmosphereLayerSpec,
    HighAltitudeAtmosphereRow,
    atmosphere_properties,
    geopotential_altitude,
    high_altitude_atmosphere,
    prepare_atmosphere_model,
)
from taoryx.contracts import Latitude, Quantity, Unit
from taoryx.equations import (
    atmos_gravity_inverse_square,
    effective_geopotential_earth_radius,
    geopotential_altitude_closed_form,
    lambert_sea_level_gravity,
)


def test_geopotential_reference_normalizes_length_units_and_matches_equations() -> None:
    latitude = Latitude(0.5)
    result = geopotential_altitude(Quantity(10.0, Unit.KILOMETER), latitude, Quantity(6378.137, Unit.KILOMETER))
    sea_level_gravity = lambert_sea_level_gravity(latitude.radians)
    effective_radius = effective_geopotential_earth_radius(sea_level_gravity, latitude.radians)

    assert result.sea_level_gravity == pytest.approx(sea_level_gravity)
    assert result.effective_radius.value == pytest.approx(effective_radius)
    assert result.gravity == pytest.approx(atmos_gravity_inverse_square(sea_level_gravity, effective_radius, 10_000.0))
    assert result.geopotential_altitude.value == pytest.approx(
        geopotential_altitude_closed_form(sea_level_gravity, 9.80665, effective_radius, 10_000.0)
    )
    assert result.effective_radius.unit is Unit.METER
    assert result.geopotential_altitude.unit is Unit.METER
####


def test_geopotential_altitude_rejects_invalid_domain() -> None:
    with pytest.raises(ValueError, match="inverse-square"):
        geopotential_altitude(Quantity(-10_000_000.0, Unit.METER), Latitude(0.0), Quantity(6378.137, Unit.KILOMETER))
####


def _layer(*, gradient: float, base_pressure: float = 101325.0) -> AtmosphereLayerSpec:
    return AtmosphereLayerSpec(
        Quantity(0.0, Unit.KILOMETER),
        Quantity(10.0, Unit.KILOMETER),
        Quantity(0.0, Unit.METER),
        Quantity(288.15, Unit.KELVIN),
        Quantity(gradient, Unit.KELVIN_PER_METER),
        28.9644,
        0.0,
        Quantity(base_pressure, Unit.PASCAL),
    )
####


def test_prepare_and_evaluate_atmosphere_layers_support_isothermal_and_gradient_branches() -> None:
    model = prepare_atmosphere_model([_layer(gradient=0.0)], Latitude(0.0), Quantity(6378.137, Unit.KILOMETER))
    sea_level = atmosphere_properties(model, Quantity(0.0, Unit.METER))
    elevated = atmosphere_properties(model, Quantity(5.0, Unit.KILOMETER))

    assert sea_level.temperature == pytest.approx(288.15)
    assert sea_level.pressure == pytest.approx(101325.0)
    assert elevated.pressure < sea_level.pressure
    assert elevated.density < sea_level.density

    gradient_model = prepare_atmosphere_model([_layer(gradient=-0.0065)], Latitude(0.0), Quantity(6378.137, Unit.KILOMETER))
    gradient_properties = atmosphere_properties(gradient_model, Quantity(5.0, Unit.KILOMETER))
    assert gradient_properties.temperature < elevated.temperature
    assert gradient_properties.speed_of_sound < elevated.speed_of_sound
####


def test_prepare_atmosphere_model_rejects_discontinuous_layers() -> None:
    first = _layer(gradient=0.0)
    second = AtmosphereLayerSpec(
        Quantity(11.0, Unit.KILOMETER),
        Quantity(20.0, Unit.KILOMETER),
        Quantity(10_000.0, Unit.METER),
        Quantity(250.0, Unit.KELVIN),
        Quantity(0.0, Unit.KELVIN_PER_METER),
        28.9644,
        0.0,
        Quantity(25000.0, Unit.PASCAL),
    )

    with pytest.raises(ValueError, match="contiguous"):
        prepare_atmosphere_model([first, second], Latitude(0.0), Quantity(6378.137, Unit.KILOMETER))
####


def test_high_altitude_atmosphere_interpolates_temperature_pressure_and_density() -> None:
    rows = (
        HighAltitudeAtmosphereRow(Quantity(146.0, Unit.KILOMETER), Quantity(500.0, Unit.KELVIN), Quantity(100.0, Unit.PASCAL), Quantity(0.001, Unit.KILOGRAM_PER_CUBIC_METER)),
        HighAltitudeAtmosphereRow(Quantity(246.0, Unit.KILOMETER), Quantity(700.0, Unit.KELVIN), Quantity(10.0, Unit.PASCAL), Quantity(0.0001, Unit.KILOGRAM_PER_CUBIC_METER)),
    )
    result = high_altitude_atmosphere(rows, Quantity(196.0, Unit.KILOMETER))

    assert result.temperature.value == pytest.approx(600.0)
    assert result.pressure.value == pytest.approx((100.0 * 10.0) ** 0.5)
    assert result.density.value == pytest.approx((0.001 * 0.0001) ** 0.5)
####


def test_high_altitude_atmosphere_rejects_out_of_range_altitude() -> None:
    row = HighAltitudeAtmosphereRow(Quantity(146.0, Unit.KILOMETER), Quantity(500.0, Unit.KELVIN), Quantity(100.0, Unit.PASCAL), Quantity(0.001, Unit.KILOGRAM_PER_CUBIC_METER))
    with pytest.raises(ValueError, match="outside"):
        high_altitude_atmosphere((row,), Quantity(147.0, Unit.KILOMETER))
####
