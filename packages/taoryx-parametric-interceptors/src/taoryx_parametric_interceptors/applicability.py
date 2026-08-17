"""Advisory operating-domain evaluation for parametric interceptor surrogates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .profile import ResolvedInterceptorProfile

ApplicabilityStatus = Literal[
    "not_declared",
    "within_declared_envelope",
    "outside_declared_envelope",
]

APPLICABILITY_CONTRACT = "taoryx.parametric-interceptors.applicability/v1"
APPLICABILITY_ENFORCEMENT = "advisory"


@dataclass(frozen=True, slots=True)
class InterceptorApplicabilityEvaluation:
    """One deterministic applicability result at a runtime boundary."""

    declared: bool
    status: ApplicabilityStatus
    reason: str
    altitude_m: float
    mach: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.altitude_m) or self.altitude_m < 0.0:
            raise ValueError("applicability altitude must be finite and nonnegative")
        if not math.isfinite(self.mach) or self.mach < 0.0:
            raise ValueError("applicability Mach must be finite and nonnegative")
        if self.declared != (self.status != "not_declared"):
            raise ValueError("applicability declared flag and status disagree")
        if self.status == "not_declared" and self.reason != "not_declared":
            raise ValueError("an undeclared applicability envelope requires the not_declared reason")
        if self.status == "within_declared_envelope" and self.reason != "none":
            raise ValueError("an in-envelope applicability result requires the none reason")
        if self.status == "outside_declared_envelope" and self.reason in {"", "none", "not_declared"}:
            raise ValueError("an out-of-envelope applicability result requires an active reason")
        ####

    ####


@dataclass(frozen=True, slots=True)
class InterceptorApplicabilityEnvelope:
    """Optional altitude/Mach domain that never fabricates missing bounds."""

    altitude_min_m: float | None = None
    altitude_max_m: float | None = None
    mach_min: float | None = None
    mach_max: float | None = None

    def __post_init__(self) -> None:
        values = (self.altitude_min_m, self.altitude_max_m, self.mach_min, self.mach_max)
        if any(item is not None and (not math.isfinite(item) or item < 0.0) for item in values):
            raise ValueError("applicability bounds must be finite and nonnegative")
        if self.altitude_min_m is not None and self.altitude_max_m is not None and self.altitude_min_m >= self.altitude_max_m:
            raise ValueError("applicability altitude minimum must be below its maximum")
        if self.mach_min is not None and self.mach_max is not None and self.mach_min >= self.mach_max:
            raise ValueError("applicability Mach minimum must be below its maximum")
        ####

    @classmethod
    def from_profile(cls, profile: ResolvedInterceptorProfile) -> InterceptorApplicabilityEnvelope:
        """Build the envelope from authored/resolved optional parameters."""

        def optional_number(name: str) -> float | None:
            return profile.number(name) if name in profile.parameters else None
            ####

        return cls(
            altitude_min_m=optional_number("applicability_altitude_min_m"),
            altitude_max_m=optional_number("applicability_altitude_max_m"),
            mach_min=optional_number("applicability_mach_min"),
            mach_max=optional_number("applicability_mach_max"),
        )
        ####

    @property
    def declared(self) -> bool:
        return any(item is not None for item in (self.altitude_min_m, self.altitude_max_m, self.mach_min, self.mach_max))
        ####

    @property
    def declared_bounds(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, value in (
                ("altitude_min_m", self.altitude_min_m),
                ("altitude_max_m", self.altitude_max_m),
                ("mach_min", self.mach_min),
                ("mach_max", self.mach_max),
            )
            if value is not None
        )
        ####

    def evaluate(self, *, altitude_m: float, mach: float) -> InterceptorApplicabilityEvaluation:
        """Evaluate declared bounds without gating or extrapolation claims."""

        if not math.isfinite(altitude_m) or altitude_m < 0.0:
            raise ValueError("applicability altitude must be finite and nonnegative")
        if not math.isfinite(mach) or mach < 0.0:
            raise ValueError("applicability Mach must be finite and nonnegative")
        if not self.declared:
            return InterceptorApplicabilityEvaluation(
                declared=False,
                status="not_declared",
                reason="not_declared",
                altitude_m=altitude_m,
                mach=mach,
            )

        reasons: list[str] = []
        if self.altitude_min_m is not None and altitude_m < self.altitude_min_m:
            reasons.append("below_declared_altitude")
        if self.altitude_max_m is not None and altitude_m > self.altitude_max_m:
            reasons.append("above_declared_altitude")
        if self.mach_min is not None and mach < self.mach_min:
            reasons.append("below_declared_mach")
        if self.mach_max is not None and mach > self.mach_max:
            reasons.append("above_declared_mach")
        return InterceptorApplicabilityEvaluation(
            declared=True,
            status=("outside_declared_envelope" if reasons else "within_declared_envelope"),
            reason=("+".join(reasons) if reasons else "none"),
            altitude_m=altitude_m,
            mach=mach,
        )
        ####

    ####


__all__ = [
    "APPLICABILITY_CONTRACT",
    "APPLICABILITY_ENFORCEMENT",
    "ApplicabilityStatus",
    "InterceptorApplicabilityEnvelope",
    "InterceptorApplicabilityEvaluation",
]
####
