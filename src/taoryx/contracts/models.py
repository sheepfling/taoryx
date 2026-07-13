"""Immutable Earth and atmosphere model definitions."""

from __future__ import annotations

from dataclasses import dataclass

from .units import Quantity


@dataclass(frozen=True, slots=True)
class EarthModel:
    """Reference Earth constants required by coordinate and gravity algorithms."""

    equatorial_radius: Quantity
    flattening: float
    gravitational_parameter: Quantity
    rotation_rate: Quantity

    def __post_init__(self) -> None:
        if self.equatorial_radius.unit.dimension != "length":
            raise ValueError("equatorial radius must have length units")
        if not 0.0 <= self.flattening < 1.0:
            raise ValueError("flattening must be in [0, 1)")
        if self.gravitational_parameter.unit.dimension != "length^3/time^2":
            raise ValueError("gravitational parameter requires length^3/time^2 units")
        if self.rotation_rate.unit.dimension != "angular_rate":
            raise ValueError("rotation rate must have angular-rate units")
        ####
####


@dataclass(frozen=True, slots=True)
class AtmosphereLayer:
    """One immutable layer of a tabulated or standard atmosphere model."""

    lower_altitude: Quantity
    upper_altitude: Quantity
    base_temperature: Quantity
    lapse_rate: Quantity
    base_pressure: Quantity

    def __post_init__(self) -> None:
        if self.lower_altitude.unit.dimension != "length" or self.upper_altitude.unit.dimension != "length":
            raise ValueError("atmosphere layer altitudes must have length units")
        if self.lower_altitude.si_value >= self.upper_altitude.si_value:
            raise ValueError("atmosphere layer upper altitude must exceed lower altitude")
        if self.base_temperature.unit.dimension != "temperature":
            raise ValueError("atmosphere base temperature requires temperature units")
        if self.lapse_rate.unit.dimension != "temperature_gradient":
            raise ValueError("atmosphere lapse rate requires temperature-gradient units")
        if self.base_pressure.unit.dimension != "pressure":
            raise ValueError("atmosphere base pressure requires pressure units")
        ####
####


@dataclass(frozen=True, slots=True)
class AtmosphereModel:
    """Ordered immutable atmosphere layers."""

    layers: tuple[AtmosphereLayer, ...]
    reference_earth: EarthModel

    def __post_init__(self) -> None:
        if not self.layers:
            raise ValueError("atmosphere model requires at least one layer")
        for previous, current in zip(self.layers, self.layers[1:], strict=False):
            if previous.upper_altitude.si_value != current.lower_altitude.si_value:
                raise ValueError("atmosphere layers must be contiguous")
            ####
        ####
####
