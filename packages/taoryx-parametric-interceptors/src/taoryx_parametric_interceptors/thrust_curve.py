"""Evidence-bearing absolute time-thrust curves for ergonomic motor intake."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .thrust_schedule import (
    ThrustProfilePoint,
    ThrustProfileSchedule,
    ThrustScheduleConfidence,
    ThrustScheduleInterpolation,
    ThrustScheduleOrigin,
)
from .units import canonicalize_interceptor_value


class AbsoluteThrustCurvePoint(BaseModel):
    """One source-unit time/thrust ordinate."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time: float = Field(ge=0.0)
    thrust: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite(self) -> Self:
        if not math.isfinite(self.time) or not math.isfinite(self.thrust):
            raise ValueError("absolute thrust-curve points must be finite")
        return self
        ####

    ####


class AbsoluteThrustCurve(BaseModel):
    """Immutable source-unit curve compiled into the normalized motor API.

    This first contract describes one continuous active-burn interval. A
    dual-pulse program needs explicit pulse timing/allocation rather than
    interpreting a zero-valued interval as an undocumented coast phase.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    contract: Literal["taoryx.parametric-interceptors.absolute-thrust-curve/v1"] = "taoryx.parametric-interceptors.absolute-thrust-curve/v1"
    curve_id: str = Field(default="developer-absolute-thrust-curve-v1", pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    points: tuple[AbsoluteThrustCurvePoint, ...] = Field(min_length=2)
    time_unit: str = "s"
    thrust_unit: str = "N"
    interpolation: ThrustScheduleInterpolation = "linear"
    origin: ThrustScheduleOrigin = "simulation_assumption"
    source_record_ids: tuple[str, ...] = ()
    confidence: ThrustScheduleConfidence = "low"
    method: str = "developer-authored absolute time-thrust curve"

    @model_validator(mode="after")
    def validate_curve(self) -> Self:
        times = self.canonical_times_s
        if not math.isclose(times[0], 0.0, rel_tol=0.0, abs_tol=1.0e-15):
            raise ValueError("absolute thrust curve must start at time zero")
        if any(right <= left for left, right in zip(times, times[1:], strict=False)):
            raise ValueError("absolute thrust-curve times must be strictly increasing")
        if self.total_impulse_n_s <= 0.0:
            raise ValueError("absolute thrust curve integrated impulse must be positive")
        if self.origin in {"observed", "reported"} and not self.source_record_ids:
            raise ValueError(f"{self.origin} absolute thrust curve requires at least one source_record_id")
        if self.origin == "derived" and not self.method:
            raise ValueError("derived absolute thrust curve requires a method")
        return self
        ####

    @property
    def canonical_times_s(self) -> tuple[float, ...]:
        return tuple(canonicalize_interceptor_value("burn_time_s", item.time, self.time_unit).canonical_value for item in self.points)
        ####

    @property
    def canonical_thrusts_n(self) -> tuple[float, ...]:
        return tuple(canonicalize_interceptor_value("nominal_thrust_n", item.thrust, self.thrust_unit).canonical_value for item in self.points)
        ####

    @property
    def duration_s(self) -> float:
        return self.canonical_times_s[-1]
        ####

    @property
    def total_impulse_n_s(self) -> float:
        times = self.canonical_times_s
        thrusts = self.canonical_thrusts_n
        area = 0.0
        for index, (left_time, right_time) in enumerate(zip(times, times[1:], strict=False)):
            width = right_time - left_time
            if self.interpolation == "step_previous":
                area += thrusts[index] * width
            else:
                area += 0.5 * (thrusts[index] + thrusts[index + 1]) * width
        return area
        ####

    @property
    def mean_thrust_n(self) -> float:
        return self.total_impulse_n_s / self.duration_s
        ####

    def normalized_schedule(self) -> ThrustProfileSchedule:
        """Return the exact unit-mean shape implied by this absolute curve."""

        mean = self.mean_thrust_n
        schedule_origin: ThrustScheduleOrigin = "derived" if self.origin in {"observed", "reported", "derived"} else self.origin
        return ThrustProfileSchedule(
            schedule_id=f"{self.curve_id}-normalized-v1",
            points=tuple(
                ThrustProfilePoint(
                    burn_fraction=time_s / self.duration_s,
                    multiplier=thrust_n / mean,
                )
                for time_s, thrust_n in zip(
                    self.canonical_times_s,
                    self.canonical_thrusts_n,
                    strict=True,
                )
            ),
            interpolation=self.interpolation,
            origin=schedule_origin,
            source_record_ids=self.source_record_ids,
            confidence=self.confidence,
            method=(f"normalized from absolute curve {self.curve_id}; {self.method}; time divided by curve duration and thrust divided by integrated mean"),
        )
        ####

    @property
    def fingerprint(self) -> str:
        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


class DualPulseThrustProgram(BaseModel):
    """Two explicit absolute pulse curves separated by a sourced coast."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    contract: Literal["taoryx.parametric-interceptors.dual-pulse-thrust-program/v1"] = "taoryx.parametric-interceptors.dual-pulse-thrust-program/v1"
    program_id: str = Field(default="developer-dual-pulse-program-v1", pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    first_pulse: AbsoluteThrustCurve
    inter_pulse_coast_time: float = Field(ge=0.0)
    coast_time_unit: str = "s"
    second_pulse: AbsoluteThrustCurve
    origin: ThrustScheduleOrigin = "simulation_assumption"
    source_record_ids: tuple[str, ...] = ()
    confidence: ThrustScheduleConfidence = "low"
    method: str = "developer-authored dual-pulse timing and curve association"

    @model_validator(mode="after")
    def validate_program(self) -> Self:
        if not math.isfinite(self.inter_pulse_coast_time):
            raise ValueError("dual-pulse coast time must be finite")
        _ = self.inter_pulse_coast_time_s
        if self.origin in {"observed", "reported"} and not self.source_record_ids:
            raise ValueError(f"{self.origin} dual-pulse program requires at least one source_record_id")
        if self.origin == "derived" and not self.method:
            raise ValueError("derived dual-pulse program requires a method")
        return self
        ####

    @property
    def inter_pulse_coast_time_s(self) -> float:
        return canonicalize_interceptor_value(
            "inter_pulse_coast_time_s",
            self.inter_pulse_coast_time,
            self.coast_time_unit,
        ).canonical_value
        ####

    @property
    def active_burn_time_s(self) -> float:
        return self.first_pulse.duration_s + self.second_pulse.duration_s
        ####

    @property
    def duration_s(self) -> float:
        return self.active_burn_time_s + self.inter_pulse_coast_time_s
        ####

    @property
    def total_impulse_n_s(self) -> float:
        return self.first_pulse.total_impulse_n_s + self.second_pulse.total_impulse_n_s
        ####

    @property
    def mean_active_burn_thrust_n(self) -> float:
        return self.total_impulse_n_s / self.active_burn_time_s
        ####

    @property
    def second_to_first_mean_thrust_ratio(self) -> float:
        return self.second_pulse.mean_thrust_n / self.first_pulse.mean_thrust_n
        ####

    @property
    def fingerprint(self) -> str:
        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


__all__ = [
    "AbsoluteThrustCurve",
    "AbsoluteThrustCurvePoint",
    "DualPulseThrustProgram",
]
####
