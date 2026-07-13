from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    atmos_aerostatic_pressure_gradient,
    atmos_gravity_inverse_square,
    atmos_kinematic_viscosity,
    atmos_perfect_gas_pressure,
    atmos_speed_of_sound,
    atmospheric_density,
    atmospheric_temperature_from_molecular_temperature,
    effective_geopotential_earth_radius,
    geopotential_altitude_closed_form,
    geopotential_altitude_integral,
    lambert_sea_level_gravity,
    layer_molecular_temperature,
    layer_molecular_weight,
    molecular_temperature_from_definition,
    molecular_temperature_gradient_constant,
    pressure_geopotential_differential,
    pressure_gradient_layer,
    pressure_isothermal_layer,
    pressure_molecular_temperature,
    pressure_separable_differential,
)


def test_atmosphere_base_relations_follow_the_manual_formulas() -> None:
    latitude = math.radians(45.5425)
    sea_level_gravity = lambert_sea_level_gravity(latitude)
    effective_radius = effective_geopotential_earth_radius(sea_level_gravity, latitude)

    assert atmos_aerostatic_pressure_gradient(1.2, 9.8) == pytest.approx(-11.76)
    assert atmos_perfect_gas_pressure(1.2, 8314.32, 28.9644, 250.0) == pytest.approx(86115.9216141194)
    assert molecular_temperature_gradient_constant(-0.0065) == pytest.approx(-0.0065)
    assert molecular_temperature_from_definition(28.9644, 28.9644, 250.0) == pytest.approx(250.0)
    assert geopotential_altitude_integral([9.8, 9.8, 9.8], 1000.0, 9.8) == pytest.approx(2000.0)
    assert atmos_gravity_inverse_square(sea_level_gravity, effective_radius, 10_000.0) == pytest.approx(
        sea_level_gravity * (effective_radius / (effective_radius + 10_000.0)) ** 2
    )
    assert geopotential_altitude_closed_form(sea_level_gravity, 9.80665, effective_radius, 10_000.0) == pytest.approx(
        sea_level_gravity / 9.80665 * (effective_radius * 10_000.0 / (effective_radius + 10_000.0))
    )
    assert lambert_sea_level_gravity(latitude) == pytest.approx(
        9.806160
        * (
            1.0
            - 0.0026373 * math.cos(2.0 * latitude)
            + 0.0000059 * math.cos(2.0 * latitude) ** 2
        )
    )
####


def test_atmosphere_layer_pressure_and_properties_follow_the_manual_formulas() -> None:
    base_pressure = 101325.0
    base_molecular_temperature = 300.0
    molecular_temperature = 280.0
    sea_level_gravity = 9.80665
    sea_level_molecular_weight = 28.9644
    universal_gas_constant = 8314.32
    geopotential_altitude = 1500.0
    base_geopotential_altitude = 1000.0

    assert layer_molecular_temperature(base_molecular_temperature, -0.0065, geopotential_altitude, base_geopotential_altitude) == pytest.approx(
        296.75
    )
    assert layer_molecular_weight(28.0, 0.001, geopotential_altitude, base_geopotential_altitude) == pytest.approx(28.5)
    assert atmospheric_temperature_from_molecular_temperature(molecular_temperature, 28.9644, sea_level_molecular_weight) == pytest.approx(280.0)
    assert pressure_geopotential_differential(1.2, 9.8, 100.0) == pytest.approx(-1176.0)
    assert pressure_molecular_temperature(1.2, universal_gas_constant, molecular_temperature, sea_level_molecular_weight) == pytest.approx(
        96449.83220781373
    )
    assert pressure_separable_differential(base_pressure, sea_level_gravity, sea_level_molecular_weight, universal_gas_constant, molecular_temperature, 10.0) == pytest.approx(
        -123.62806095202315
    )
    assert pressure_isothermal_layer(base_pressure, sea_level_gravity, sea_level_molecular_weight, universal_gas_constant, base_molecular_temperature, geopotential_altitude, base_geopotential_altitude) == pytest.approx(
        95716.86537259835
    )
    assert pressure_gradient_layer(
        base_pressure,
        base_molecular_temperature,
        molecular_temperature,
        sea_level_gravity,
        sea_level_molecular_weight,
        universal_gas_constant,
        -0.0065,
        geopotential_altitude,
        base_geopotential_altitude,
    ) == pytest.approx(70507.22503122287)
    assert atmospheric_density(pressure=101325.0, sea_level_molecular_weight=28.9644, universal_gas_constant=8314.32, molecular_temperature=288.15) == pytest.approx(
        1.2249991558877122
    )
    assert atmos_speed_of_sound(288.15, 28.9644, 1.4, 8314.32) == pytest.approx(340.2941077869353)
    assert atmos_kinematic_viscosity(1.225, 288.15) == pytest.approx(1.4607185943490473e-05)
####
