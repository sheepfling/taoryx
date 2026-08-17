"""Evidence-bearing Mach-drag schedules shared by interceptor runtime tiers."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

DragScheduleOrigin = Literal[
    "observed",
    "reported",
    "derived",
    "inferred",
    "archetype_assumption",
    "calibrated",
    "simulation_assumption",
]
DragScheduleConfidence = Literal["unknown", "low", "medium", "high"]


@dataclass(frozen=True, slots=True)
class ManeuverDragEvaluation:
    """Reduced-order drag increment from achieved aerodynamic normal force."""

    factor: float
    normal_force_coefficient: float
    drag_coefficient: float
    drag_n: float

    def __post_init__(self) -> None:
        values = (
            self.factor,
            self.normal_force_coefficient,
            self.drag_coefficient,
            self.drag_n,
        )
        if any(not math.isfinite(item) or item < 0.0 for item in values):
            raise ValueError("maneuver-drag values must be finite and nonnegative")
        ####

    ####


class DragCoefficientPoint(BaseModel):
    """One dimensionless drag-coefficient ordinate at a Mach abscissa."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mach: float = Field(ge=0.0)
    coefficient: float = Field(gt=0.0)

    @model_validator(mode="after")
    def validate_finite(self) -> Self:
        if not math.isfinite(self.mach) or not math.isfinite(self.coefficient):
            raise ValueError("drag schedule points must be finite")
        return self
        ####

    ####


class DragCoefficientSchedule(BaseModel):
    """Immutable piecewise-linear drag schedule with evidence lineage.

    Endpoints are held outside the authored Mach domain. That policy keeps the
    reduced-order runtime deterministic and prevents undocumented polynomial
    extrapolation from dominating high-speed studies.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    contract: Literal["taoryx.parametric-interceptors.drag-schedule/v1"] = "taoryx.parametric-interceptors.drag-schedule/v1"
    schedule_id: str = Field(default="developer-drag-schedule-v1", pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    points: tuple[DragCoefficientPoint, ...] = Field(min_length=2)
    interpolation: Literal["linear"] = "linear"
    extrapolation: Literal["hold"] = "hold"
    origin: DragScheduleOrigin = "simulation_assumption"
    source_record_ids: tuple[str, ...] = ()
    confidence: DragScheduleConfidence = "low"
    method: str = "developer-authored Mach/Cd schedule"

    @model_validator(mode="after")
    def validate_schedule(self) -> Self:
        mach_values = tuple(item.mach for item in self.points)
        if any(right <= left for left, right in zip(mach_values, mach_values[1:], strict=False)):
            raise ValueError("drag schedule Mach points must be strictly increasing")
        if self.origin in {"observed", "reported"} and not self.source_record_ids:
            raise ValueError(f"{self.origin} drag schedule requires at least one source_record_id")
        if self.origin == "derived" and not self.method:
            raise ValueError("derived drag schedule requires a method")
        return self
        ####

    def coefficient_at(self, mach: float) -> float:
        """Return held-endpoint, linearly interpolated coefficient at Mach."""

        if not math.isfinite(mach) or mach < 0.0:
            raise ValueError("drag schedule evaluation Mach must be finite and nonnegative")
        if mach <= self.points[0].mach:
            return self.points[0].coefficient
        if mach >= self.points[-1].mach:
            return self.points[-1].coefficient
        for left, right in zip(self.points, self.points[1:], strict=False):
            if mach <= right.mach:
                fraction = (mach - left.mach) / (right.mach - left.mach)
                return left.coefficient + fraction * (right.coefficient - left.coefficient)
        raise RuntimeError("validated drag schedule did not bracket Mach")
        ####

    @property
    def fingerprint(self) -> str:
        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


def evaluate_maneuver_drag(
    *,
    dynamic_pressure_pa: float,
    reference_area_m2: float,
    mass_kg: float,
    aerodynamic_lateral_acceleration_mps2: float,
    maneuver_drag_factor: float,
) -> ManeuverDragEvaluation:
    """Evaluate ``Cdm = k * Cn^2`` from achieved aerodynamic authority.

    This is a transparent load-dependent drag surrogate. It is not a fitted
    induced-drag polar, angle-of-attack model, or aerodynamic coefficient deck.
    """

    values = (
        dynamic_pressure_pa,
        reference_area_m2,
        mass_kg,
        aerodynamic_lateral_acceleration_mps2,
        maneuver_drag_factor,
    )
    if any(not math.isfinite(item) for item in values):
        raise ValueError("maneuver-drag inputs must be finite")
    if dynamic_pressure_pa < 0.0 or aerodynamic_lateral_acceleration_mps2 < 0.0 or maneuver_drag_factor < 0.0:
        raise ValueError("maneuver-drag pressure, acceleration, and factor must be nonnegative")
    if reference_area_m2 <= 0.0 or mass_kg <= 0.0:
        raise ValueError("maneuver-drag reference area and mass must be positive")

    pressure_area = dynamic_pressure_pa * reference_area_m2
    if pressure_area <= 1.0e-15:
        if aerodynamic_lateral_acceleration_mps2 > 1.0e-12:
            raise ValueError("nonzero aerodynamic lateral acceleration requires positive dynamic pressure")
        return ManeuverDragEvaluation(
            factor=maneuver_drag_factor,
            normal_force_coefficient=0.0,
            drag_coefficient=0.0,
            drag_n=0.0,
        )

    normal_force_coefficient = mass_kg * aerodynamic_lateral_acceleration_mps2 / pressure_area
    drag_coefficient = maneuver_drag_factor * normal_force_coefficient**2
    return ManeuverDragEvaluation(
        factor=maneuver_drag_factor,
        normal_force_coefficient=normal_force_coefficient,
        drag_coefficient=drag_coefficient,
        drag_n=pressure_area * drag_coefficient,
    )
    ####


__all__ = [
    "DragCoefficientPoint",
    "DragCoefficientSchedule",
    "DragScheduleConfidence",
    "DragScheduleOrigin",
    "ManeuverDragEvaluation",
    "evaluate_maneuver_drag",
]
####
