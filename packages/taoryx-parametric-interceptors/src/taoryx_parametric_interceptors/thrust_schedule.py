"""Evidence-bearing normalized thrust-time schedules for interceptor motors."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

ThrustScheduleOrigin = Literal[
    "observed",
    "reported",
    "derived",
    "inferred",
    "archetype_assumption",
    "calibrated",
    "simulation_assumption",
]
ThrustScheduleConfidence = Literal["unknown", "low", "medium", "high"]
ThrustScheduleInterpolation = Literal["linear", "step_previous"]


class ThrustProfilePoint(BaseModel):
    """One raw thrust multiplier at a normalized active-burn fraction."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    burn_fraction: float = Field(ge=0.0, le=1.0)
    multiplier: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite(self) -> Self:
        if not math.isfinite(self.burn_fraction) or not math.isfinite(self.multiplier):
            raise ValueError("thrust schedule points must be finite")
        return self
        ####

    ####


class ThrustProfileSchedule(BaseModel):
    """Immutable, unit-mean thrust shape over one active motor pulse.

    Authors provide relative multipliers, not a pre-normalized force curve. The
    runtime divides them by their integrated area, preserving nominal total
    impulse while redistributing thrust and propellant consumption in time.
    The same normalized shape is applied independently to each active pulse;
    dual-pulse amplitude and propellant allocation remain separate controls.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    contract: Literal["taoryx.parametric-interceptors.thrust-profile-schedule/v1"] = "taoryx.parametric-interceptors.thrust-profile-schedule/v1"
    schedule_id: str = Field(default="developer-thrust-profile-v1", pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    points: tuple[ThrustProfilePoint, ...] = Field(min_length=2)
    interpolation: ThrustScheduleInterpolation = "linear"
    normalization: Literal["unit_mean_active_burn"] = "unit_mean_active_burn"
    origin: ThrustScheduleOrigin = "simulation_assumption"
    source_record_ids: tuple[str, ...] = ()
    confidence: ThrustScheduleConfidence = "low"
    method: str = "developer-authored normalized thrust-time shape"

    @model_validator(mode="after")
    def validate_schedule(self) -> Self:
        fractions = tuple(item.burn_fraction for item in self.points)
        if not math.isclose(fractions[0], 0.0, rel_tol=0.0, abs_tol=1.0e-15):
            raise ValueError("thrust schedule must start at burn_fraction 0")
        if not math.isclose(fractions[-1], 1.0, rel_tol=0.0, abs_tol=1.0e-15):
            raise ValueError("thrust schedule must end at burn_fraction 1")
        if any(right <= left for left, right in zip(fractions, fractions[1:], strict=False)):
            raise ValueError("thrust schedule burn fractions must be strictly increasing")
        if self.raw_area <= 0.0:
            raise ValueError("thrust schedule integrated multiplier must be positive")
        if self.origin in {"observed", "reported"} and not self.source_record_ids:
            raise ValueError(f"{self.origin} thrust schedule requires at least one source_record_id")
        if self.origin == "derived" and not self.method:
            raise ValueError("derived thrust schedule requires a method")
        return self
        ####

    @property
    def raw_area(self) -> float:
        """Return the unnormalized multiplier integral over burn fraction."""

        return sum(self._segment_area(left, right) for left, right in zip(self.points, self.points[1:], strict=False))
        ####

    @property
    def normalized_peak(self) -> float:
        """Return the peak multiplier after unit-mean normalization."""

        return max(item.multiplier for item in self.points) / self.raw_area
        ####

    def multiplier_at(self, burn_fraction: float) -> float:
        """Return the unit-mean thrust multiplier at one burn fraction."""

        value = self._validated_fraction(burn_fraction)
        if value >= 1.0:
            return self.points[-1].multiplier / self.raw_area
        left, right = self._bracketing_segment(value)
        if self.interpolation == "step_previous":
            raw = left.multiplier
        else:
            fraction = (value - left.burn_fraction) / (right.burn_fraction - left.burn_fraction)
            raw = left.multiplier + fraction * (right.multiplier - left.multiplier)
        return raw / self.raw_area
        ####

    def cumulative_fraction_at(self, burn_fraction: float) -> float:
        """Return normalized impulse and propellant fraction consumed."""

        value = self._validated_fraction(burn_fraction)
        if value <= 0.0:
            return 0.0
        if value >= 1.0:
            return 1.0
        area = 0.0
        for left, right in zip(self.points, self.points[1:], strict=False):
            if value >= right.burn_fraction:
                area += self._segment_area(left, right)
                continue
            width = value - left.burn_fraction
            if self.interpolation == "step_previous":
                area += left.multiplier * width
            else:
                slope = (right.multiplier - left.multiplier) / (right.burn_fraction - left.burn_fraction)
                area += left.multiplier * width + 0.5 * slope * width**2
            break
        return min(1.0, max(0.0, area / self.raw_area))
        ####

    @property
    def fingerprint(self) -> str:
        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    def _segment_area(self, left: ThrustProfilePoint, right: ThrustProfilePoint) -> float:
        width = right.burn_fraction - left.burn_fraction
        if self.interpolation == "step_previous":
            return left.multiplier * width
        return 0.5 * (left.multiplier + right.multiplier) * width
        ####

    def _bracketing_segment(self, value: float) -> tuple[ThrustProfilePoint, ThrustProfilePoint]:
        for left, right in zip(self.points, self.points[1:], strict=False):
            if value < right.burn_fraction:
                return left, right
        raise RuntimeError("validated thrust schedule did not bracket burn fraction")
        ####

    @staticmethod
    def _validated_fraction(burn_fraction: float) -> float:
        if not math.isfinite(burn_fraction) or not 0.0 <= burn_fraction <= 1.0:
            raise ValueError("thrust schedule burn fraction must be finite and lie in [0, 1]")
        return burn_fraction
        ####

    ####


__all__ = [
    "ThrustProfilePoint",
    "ThrustProfileSchedule",
    "ThrustScheduleConfidence",
    "ThrustScheduleInterpolation",
    "ThrustScheduleOrigin",
]
####
