"""Typed atmosphere reference and geopotential calculations."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from .contracts import Latitude, Quantity, Unit
from .equations import (
    atmos_gravity_inverse_square,
    atmos_kinematic_viscosity,
    atmos_speed_of_sound,
    atmospheric_density,
    atmospheric_temperature_from_molecular_temperature,
    effective_geopotential_earth_radius,
    geopotential_altitude_closed_form,
    lambert_sea_level_gravity,
    layer_molecular_temperature,
    layer_molecular_weight,
    pressure_gradient_layer,
)


@dataclass(frozen=True, slots=True)
class GeopotentialReference:
    """Atmosphere reference values at one model latitude and altitude."""

    sea_level_gravity: float
    effective_radius: Quantity
    gravity: float
    geopotential_altitude: Quantity
####


@dataclass(frozen=True, slots=True)
class AtmosphereLayerSpec:
    """One TAOS atmosphere layer before setup precomputes SI values."""

    lower_altitude: Quantity
    upper_altitude: Quantity
    base_geopotential_altitude: Quantity
    base_molecular_temperature: Quantity
    molecular_temperature_gradient: Quantity
    base_molecular_weight: float
    molecular_weight_gradient: float
    base_pressure: Quantity

    def __post_init__(self) -> None:
        if self.lower_altitude.unit.dimension != "length" or self.upper_altitude.unit.dimension != "length":
            raise ValueError("atmosphere layer bounds require length units")
        if self.lower_altitude.si_value >= self.upper_altitude.si_value:
            raise ValueError("atmosphere layer upper bound must exceed lower bound")
        if self.base_geopotential_altitude.unit.dimension != "length":
            raise ValueError("base geopotential altitude requires length units")
        if self.base_molecular_temperature.unit.dimension != "temperature":
            raise ValueError("base molecular temperature requires temperature units")
        if self.molecular_temperature_gradient.unit.dimension != "temperature_gradient":
            raise ValueError("molecular temperature gradient requires temperature-gradient units")
        if self.base_pressure.unit.dimension != "pressure" or self.base_pressure.si_value <= 0.0:
            raise ValueError("base pressure must be positive and use pressure units")
        if not math.isfinite(self.base_molecular_weight) or self.base_molecular_weight <= 0.0:
            raise ValueError("base molecular weight must be positive and finite")
        if not math.isfinite(self.molecular_weight_gradient):
            raise ValueError("molecular weight gradient must be finite")
    ####
####


@dataclass(frozen=True, slots=True)
class PreparedAtmosphereLayer:
    """SI-normalized atmosphere layer used during repeated evaluations."""

    lower_altitude: float
    upper_altitude: float
    base_geopotential_altitude: float
    base_molecular_temperature: float
    molecular_temperature_gradient: float
    base_molecular_weight: float
    molecular_weight_gradient: float
    base_pressure: float
####


@dataclass(frozen=True, slots=True)
class PreparedAtmosphereModel:
    """Prepared atmosphere constants and contiguous lookup layers."""

    layers: tuple[PreparedAtmosphereLayer, ...]
    model_latitude: Latitude
    equatorial_radius: Quantity
    sea_level_gravity: float
    effective_radius: float
    reference_gravity: float
    sea_level_molecular_weight: float
    universal_gas_constant: float
    ratio_of_specific_heats: float
####


@dataclass(frozen=True, slots=True)
class AtmosphereProperties:
    """Thermodynamic atmosphere values at one geometric altitude."""

    geometric_altitude: Quantity
    geopotential_altitude: Quantity
    molecular_temperature: float
    molecular_weight: float
    temperature: float
    pressure: float
    density: float
    speed_of_sound: float
    kinematic_viscosity: float
####


@dataclass(frozen=True, slots=True)
class HighAltitudeAtmosphereRow:
    """One high-altitude atmosphere table row."""

    altitude: Quantity
    temperature: Quantity
    pressure: Quantity
    density: Quantity

    def __post_init__(self) -> None:
        if self.altitude.unit.dimension != "length":
            raise ValueError("high-altitude row requires length units")
        if self.temperature.unit.dimension != "temperature":
            raise ValueError("high-altitude temperature requires temperature units")
        if self.pressure.unit.dimension != "pressure" or self.pressure.value <= 0.0:
            raise ValueError("high-altitude pressure must be positive")
        if self.density.unit.dimension != "density" or self.density.value <= 0.0 or not math.isfinite(self.density.value):
            raise ValueError("high-altitude density must be positive and finite")
    ####
####


def high_altitude_atmosphere(
    rows: Sequence[HighAltitudeAtmosphereRow],
    geometric_altitude: Quantity,
) -> HighAltitudeAtmosphereRow:
    """Evaluate TAOS-ALG-ENV-005 by table interpolation through 1000 km.

    Temperature is interpolated linearly. Pressure and density are
    interpolated in log space, matching the manual's supplemental-table
    convention. Values outside the supplied table are rejected explicitly.
    """

    if not rows:
        raise ValueError("high-altitude atmosphere requires at least one row")
    if geometric_altitude.unit.dimension != "length":
        raise ValueError("geometric altitude must have length units")
    altitude = geometric_altitude.si_value
    normalized = sorted(rows, key=lambda row: row.altitude.si_value)
    if any(left.altitude.si_value >= right.altitude.si_value for left, right in zip(normalized, normalized[1:], strict=False)):
        raise ValueError("high-altitude rows must have strictly increasing altitudes")
    if altitude < normalized[0].altitude.si_value or altitude > normalized[-1].altitude.si_value:
        raise ValueError("geometric altitude is outside the high-altitude table")
    if len(normalized) == 1:
        if altitude != normalized[0].altitude.si_value:
            raise ValueError("a single high-altitude row only supports its exact altitude")
        return normalized[0]
    upper_index = next((index for index, row in enumerate(normalized) if altitude <= row.altitude.si_value), len(normalized) - 1)
    if altitude == normalized[upper_index].altitude.si_value:
        return normalized[upper_index]
    lower = normalized[upper_index - 1]
    upper = normalized[upper_index]
    fraction = (altitude - lower.altitude.si_value) / (upper.altitude.si_value - lower.altitude.si_value)
    temperature = lower.temperature.si_value + fraction * (upper.temperature.si_value - lower.temperature.si_value)
    pressure = math.exp(math.log(lower.pressure.si_value) + fraction * (math.log(upper.pressure.si_value) - math.log(lower.pressure.si_value)))
    density = math.exp(math.log(lower.density.si_value) + fraction * (math.log(upper.density.si_value) - math.log(lower.density.si_value)))
    return HighAltitudeAtmosphereRow(
        Quantity(altitude, Unit.METER),
        Quantity(temperature, Unit.KELVIN),
        Quantity(pressure, Unit.PASCAL),
        Quantity(density, Unit.KILOGRAM_PER_CUBIC_METER),
    )
####


def prepare_atmosphere_model(
    layers: Sequence[AtmosphereLayerSpec],
    model_latitude: Latitude,
    equatorial_radius: Quantity,
    *,
    reference_gravity: float = 9.80665,
    sea_level_molecular_weight: float = 28.9644,
    universal_gas_constant: float = 8314.462618,
    ratio_of_specific_heats: float = 1.4,
) -> PreparedAtmosphereModel:
    """Evaluate TAOS-ALG-ENV-002 setup and layer precomputation.

    Layer base pressures are supplied by the selected historical atmosphere
    table; setup validates their order and normalizes all stored quantities.
    """

    if not layers:
        raise ValueError("atmosphere setup requires at least one layer")
    if equatorial_radius.unit.dimension != "length" or equatorial_radius.si_value <= 0.0:
        raise ValueError("equatorial radius must be a positive length")
    if reference_gravity <= 0.0 or sea_level_molecular_weight <= 0.0 or universal_gas_constant <= 0.0 or ratio_of_specific_heats <= 1.0:
        raise ValueError("atmosphere constants must be physically valid")
    normalized: list[PreparedAtmosphereLayer] = []
    previous_upper: float | None = None
    for layer in layers:
        lower = layer.lower_altitude.si_value
        upper = layer.upper_altitude.si_value
        if previous_upper is not None and not math.isclose(previous_upper, lower, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("atmosphere layers must be contiguous")
        normalized.append(
            PreparedAtmosphereLayer(
                lower,
                upper,
                layer.base_geopotential_altitude.si_value,
                layer.base_molecular_temperature.si_value,
                layer.molecular_temperature_gradient.to(Unit.KELVIN_PER_METER).value,
                layer.base_molecular_weight,
                layer.molecular_weight_gradient,
                layer.base_pressure.to(Unit.PASCAL).value,
            )
        )
        previous_upper = upper
    sea_level_gravity = lambert_sea_level_gravity(model_latitude.radians)
    effective_radius = effective_geopotential_earth_radius(sea_level_gravity, model_latitude.radians)
    return PreparedAtmosphereModel(
        tuple(normalized),
        model_latitude,
        equatorial_radius.to(Unit.METER),
        sea_level_gravity,
        effective_radius,
        reference_gravity,
        sea_level_molecular_weight,
        universal_gas_constant,
        ratio_of_specific_heats,
    )
####


def atmosphere_properties(model: PreparedAtmosphereModel, geometric_altitude: Quantity) -> AtmosphereProperties:
    """Evaluate TAOS-ALG-ENV-004 and equations 2-185 through 2-195."""

    if geometric_altitude.unit.dimension != "length":
        raise ValueError("geometric altitude must have length units")
    altitude = geometric_altitude.si_value
    layer = next((item for item in model.layers if item.lower_altitude <= altitude <= item.upper_altitude), None)
    if layer is None:
        raise ValueError("geometric altitude is outside the prepared atmosphere")
    geopotential = geopotential_altitude_closed_form(model.sea_level_gravity, model.reference_gravity, model.effective_radius, altitude)
    molecular_temperature = layer_molecular_temperature(
        layer.base_molecular_temperature,
        layer.molecular_temperature_gradient,
        geopotential,
        layer.base_geopotential_altitude,
    )
    molecular_weight = layer_molecular_weight(
        layer.base_molecular_weight,
        layer.molecular_weight_gradient,
        geopotential,
        layer.base_geopotential_altitude,
    )
    if molecular_temperature <= 0.0 or molecular_weight <= 0.0:
        raise ValueError("atmosphere layer produced a nonphysical molecular state")
    temperature = atmospheric_temperature_from_molecular_temperature(
        molecular_temperature,
        molecular_weight,
        model.sea_level_molecular_weight,
    )
    pressure = pressure_gradient_layer(
        layer.base_pressure,
        layer.base_molecular_temperature,
        molecular_temperature,
        model.sea_level_gravity,
        model.sea_level_molecular_weight,
        model.universal_gas_constant,
        layer.molecular_temperature_gradient,
        geopotential,
        layer.base_geopotential_altitude,
    )
    density = atmospheric_density(pressure, model.sea_level_molecular_weight, model.universal_gas_constant, molecular_temperature)
    speed_of_sound = atmos_speed_of_sound(
        molecular_temperature,
        model.sea_level_molecular_weight,
        model.ratio_of_specific_heats,
        model.universal_gas_constant,
    )
    viscosity = atmos_kinematic_viscosity(density, molecular_temperature)
    return AtmosphereProperties(
        Quantity(altitude, Unit.METER),
        Quantity(geopotential, Unit.METER),
        molecular_temperature,
        molecular_weight,
        temperature,
        pressure,
        density,
        speed_of_sound,
        viscosity,
    )
####


def geopotential_altitude(
    geometric_altitude: Quantity,
    model_latitude: Latitude,
    equatorial_radius: Quantity,
    *,
    reference_gravity: float = 9.80665,
) -> GeopotentialReference:
    """Evaluate TAOS-ALG-ENV-003 and equations 2-176 through 2-184.

    Length inputs may use meters or kilometers; all returned lengths use SI
    meters so the atmosphere equations cannot silently mix scales.
    """

    if geometric_altitude.unit.dimension != "length" or equatorial_radius.unit.dimension != "length":
        raise ValueError("atmosphere altitude and equatorial radius require length units")
    if equatorial_radius.si_value <= 0.0:
        raise ValueError("equatorial radius must be positive")
    if reference_gravity <= 0.0:
        raise ValueError("reference gravity must be positive")
    altitude = geometric_altitude.to(Unit.METER).value
    sea_level_gravity = lambert_sea_level_gravity(model_latitude.radians)
    effective_radius = effective_geopotential_earth_radius(sea_level_gravity, model_latitude.radians)
    denominator = effective_radius + altitude
    if denominator <= 0.0:
        raise ValueError("geometric altitude is outside the inverse-square atmosphere domain")
    gravity = atmos_gravity_inverse_square(sea_level_gravity, effective_radius, altitude)
    geopotential = geopotential_altitude_closed_form(sea_level_gravity, reference_gravity, effective_radius, altitude)
    return GeopotentialReference(
        sea_level_gravity,
        Quantity(effective_radius, Unit.METER),
        gravity,
        Quantity(geopotential, Unit.METER),
    )
####
