"""Small explicit unit system for algorithm boundaries."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum


class Unit(StrEnum):
    """Supported units and their SI conversion metadata."""

    METER = "m"
    KILOMETER = "km"
    SECOND = "s"
    RADIAN = "rad"
    DEGREE = "deg"
    KILOGRAM = "kg"
    KILOGRAM_PER_CUBIC_METER = "kg/m^3"
    KELVIN = "K"
    METER_PER_SECOND = "m/s"
    METER_PER_SECOND_SQUARED = "m/s^2"
    METER_CUBED_PER_SECOND_SQUARED = "m^3/s^2"
    KELVIN_PER_METER = "K/m"
    RADIAN_PER_SECOND = "rad/s"
    PASCAL = "Pa"

    @property
    def dimension(self) -> str:
        return {
            Unit.METER: "length", Unit.KILOMETER: "length", Unit.SECOND: "time",
            Unit.RADIAN: "angle", Unit.DEGREE: "angle", Unit.KILOGRAM: "mass", Unit.KILOGRAM_PER_CUBIC_METER: "density",
            Unit.KELVIN: "temperature",
            Unit.METER_PER_SECOND: "speed", Unit.METER_PER_SECOND_SQUARED: "acceleration",
            Unit.METER_CUBED_PER_SECOND_SQUARED: "length^3/time^2", Unit.KELVIN_PER_METER: "temperature_gradient",
            Unit.RADIAN_PER_SECOND: "angular_rate", Unit.PASCAL: "pressure",
        }[self]
    ####

    @property
    def scale_to_si(self) -> float:
        return {
            Unit.METER: 1.0, Unit.KILOMETER: 1_000.0, Unit.SECOND: 1.0,
            Unit.RADIAN: 1.0, Unit.DEGREE: 3.141592653589793 / 180.0,
            Unit.KILOGRAM: 1.0, Unit.KILOGRAM_PER_CUBIC_METER: 1.0, Unit.KELVIN: 1.0, Unit.METER_PER_SECOND: 1.0,
            Unit.METER_PER_SECOND_SQUARED: 1.0, Unit.METER_CUBED_PER_SECOND_SQUARED: 1.0,
            Unit.KELVIN_PER_METER: 1.0, Unit.RADIAN_PER_SECOND: 1.0,
            Unit.PASCAL: 1.0,
        }[self]
    ####


@dataclass(frozen=True, slots=True)
class Quantity:
    """Finite scalar paired with a unit and convertible only within a dimension."""

    value: float
    unit: Unit

    def __post_init__(self) -> None:
        if not isinstance(self.value, (int, float)) or not math.isfinite(float(self.value)):
            raise ValueError("quantity value must be finite")
        ####

    def to(self, unit: Unit) -> Quantity:
        if self.unit.dimension != unit.dimension:
            raise ValueError(f"cannot convert {self.unit.dimension} to {unit.dimension}")
        return Quantity(self.value * self.unit.scale_to_si / unit.scale_to_si, unit)
    ####

    @property
    def si_value(self) -> float:
        return self.value * self.unit.scale_to_si
    ####
####
