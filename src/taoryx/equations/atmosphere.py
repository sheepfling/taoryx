"""Atmosphere equation helpers."""

from __future__ import annotations

import math
from collections.abc import Sequence


def atmos_aerostatic_pressure_gradient(density: float, gravity: float) -> float:
    """Return the aerostatic pressure gradient from density and gravity."""

    return -density * gravity
####


def atmos_perfect_gas_pressure(
    density: float,
    universal_gas_constant: float,
    molecular_weight: float,
    temperature: float,
) -> float:
    """Return pressure from the perfect-gas relation used by TAOS."""

    return density * universal_gas_constant * temperature / molecular_weight
####


def molecular_temperature_gradient_constant(gradient: float) -> float:
    """Return the constant molecular-temperature gradient within a layer."""

    return gradient
####


def molecular_temperature_from_definition(
    sea_level_molecular_weight: float,
    molecular_weight: float,
    temperature: float,
) -> float:
    """Return molecular temperature from the TAOS definition."""

    return sea_level_molecular_weight * temperature / molecular_weight
####


def geopotential_altitude_integral(
    gravity_samples: Sequence[float],
    altitude_step: float,
    reference_geopotential: float,
) -> float:
    """Return geopotential altitude from a sampled gravity profile."""

    if not gravity_samples:
        return 0.0
    ####
    total = 0.0
    previous = gravity_samples[0]
    for current in gravity_samples[1:]:
        total += 0.5 * (previous + current) * altitude_step
        previous = current
    ####
    return total / reference_geopotential
####


def atmos_gravity_inverse_square(
    sea_level_gravity: float,
    effective_earth_radius: float,
    altitude: float,
) -> float:
    """Return the altitude-varying gravity from the inverse-square law."""

    return sea_level_gravity * (effective_earth_radius / (effective_earth_radius + altitude)) ** 2
####


def geopotential_altitude_closed_form(
    sea_level_gravity: float,
    reference_geopotential: float,
    effective_earth_radius: float,
    altitude: float,
) -> float:
    """Return the closed-form geopotential altitude relation."""

    return (
        sea_level_gravity
        / reference_geopotential
        * (effective_earth_radius * altitude / (effective_earth_radius + altitude))
    )
####


def lambert_sea_level_gravity(latitude_radians: float) -> float:
    """Return Lambert sea-level gravity for the atmosphere model latitude."""

    cosine_double_latitude = math.cos(2.0 * latitude_radians)
    return 9.806160 * (
        1.0 - 0.0026373 * cosine_double_latitude + 0.0000059 * cosine_double_latitude * cosine_double_latitude
    )
####


def effective_geopotential_earth_radius(
    sea_level_gravity: float,
    latitude_radians: float,
) -> float:
    """Return the effective earth radius used in geopotential calculations."""

    cosine_double_latitude = math.cos(2.0 * latitude_radians)
    cosine_quadruple_latitude = math.cos(4.0 * latitude_radians)
    return 2.0 * sea_level_gravity / (
        3.085462e-6
        + 2.27e-9 * cosine_double_latitude
        - 2.0e-12 * cosine_quadruple_latitude
    )
####


def layer_molecular_temperature(
    base_molecular_temperature: float,
    gradient: float,
    geopotential_altitude: float,
    base_geopotential_altitude: float,
) -> float:
    """Return the molecular temperature inside a layer."""

    return base_molecular_temperature + gradient * (geopotential_altitude - base_geopotential_altitude)
####


def layer_molecular_weight(
    base_molecular_weight: float,
    gradient: float,
    geopotential_altitude: float,
    base_geopotential_altitude: float,
) -> float:
    """Return the molecular weight inside a layer."""

    return base_molecular_weight + gradient * (geopotential_altitude - base_geopotential_altitude)
####


def atmospheric_temperature_from_molecular_temperature(
    molecular_temperature: float,
    molecular_weight: float,
    sea_level_molecular_weight: float,
) -> float:
    """Return atmospheric temperature from molecular temperature."""

    return molecular_weight * molecular_temperature / sea_level_molecular_weight
####


def pressure_geopotential_differential(
    density: float,
    gravity: float,
    geopotential_altitude_step: float,
) -> float:
    """Return the pressure change across a geopotential-altitude step."""

    return -density * gravity * geopotential_altitude_step
####


def pressure_molecular_temperature(
    density: float,
    universal_gas_constant: float,
    molecular_temperature: float,
    sea_level_molecular_weight: float,
) -> float:
    """Return pressure in terms of molecular temperature."""

    return density * universal_gas_constant * molecular_temperature / sea_level_molecular_weight
####


def pressure_separable_differential(
    pressure: float,
    sea_level_gravity: float,
    sea_level_molecular_weight: float,
    universal_gas_constant: float,
    molecular_temperature: float,
    geopotential_altitude_step: float,
) -> float:
    """Return the pressure step implied by the separable differential form."""

    exponent = -sea_level_gravity * sea_level_molecular_weight / (universal_gas_constant * molecular_temperature)
    return pressure * exponent * geopotential_altitude_step
####


def pressure_isothermal_layer(
    base_pressure: float,
    sea_level_gravity: float,
    sea_level_molecular_weight: float,
    universal_gas_constant: float,
    base_molecular_temperature: float,
    geopotential_altitude: float,
    base_geopotential_altitude: float,
) -> float:
    """Return pressure in an isothermal atmosphere layer."""

    exponent = -sea_level_gravity * sea_level_molecular_weight * (
        geopotential_altitude - base_geopotential_altitude
    ) / (universal_gas_constant * base_molecular_temperature)
    return base_pressure * math.exp(exponent)
####


def pressure_gradient_layer(
    base_pressure: float,
    base_molecular_temperature: float,
    molecular_temperature: float,
    sea_level_gravity: float,
    sea_level_molecular_weight: float,
    universal_gas_constant: float,
    molecular_temperature_gradient_value: float,
    geopotential_altitude: float,
    base_geopotential_altitude: float,
) -> float:
    """Return pressure in a nonisothermal atmosphere layer."""

    if molecular_temperature_gradient_value == 0.0:
        return pressure_isothermal_layer(
            base_pressure,
            sea_level_gravity,
            sea_level_molecular_weight,
            universal_gas_constant,
            base_molecular_temperature,
            geopotential_altitude,
            base_geopotential_altitude,
        )
    ####
    exponent = sea_level_gravity * sea_level_molecular_weight / (
        universal_gas_constant * molecular_temperature_gradient_value
    )
    return base_pressure * (base_molecular_temperature / molecular_temperature) ** exponent
####


def atmospheric_density(
    pressure: float,
    sea_level_molecular_weight: float,
    universal_gas_constant: float,
    molecular_temperature: float,
) -> float:
    """Return atmospheric density."""

    return pressure * sea_level_molecular_weight / (universal_gas_constant * molecular_temperature)
####


def atmos_speed_of_sound(
    molecular_temperature: float,
    sea_level_molecular_weight: float,
    ratio_of_specific_heats: float,
    universal_gas_constant: float,
) -> float:
    """Return the speed of sound from molecular temperature."""

    return math.sqrt(ratio_of_specific_heats * universal_gas_constant * molecular_temperature / sea_level_molecular_weight)
####


def atmos_kinematic_viscosity(density: float, molecular_temperature: float) -> float:
    """Return the kinematic viscosity from Sutherland's formula."""

    return (1.0 / density) * 1.458e-6 * molecular_temperature ** 1.5 / (molecular_temperature + 110.4)
####
