"""Declarative sensor-clock contracts for accepted-truth scheduling."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

SensorSampleMode = Literal["instantaneous", "interval"]
SensorTruthPolicy = Literal["boundary", "accepted-segment"]
SensorRatePolicy = Literal["split", "accumulate"]


@dataclass(frozen=True, slots=True)
class SensorClockSpec:
    """A sensor cadence that can constrain the model's next truth boundary.

    This is deliberately a clock contract, not a measurement model.  A sensor
    provider may later consume the committed state or accepted segment, apply
    noise and latency, and publish a measurement without changing the truth
    timeline.
    """

    name: str
    kind: str
    cadence_s: float
    phase_s: float = 0.0
    sample_mode: SensorSampleMode = "instantaneous"
    delivery_s: float = 0.0
    truth_policy: SensorTruthPolicy = "boundary"
    rate_policy: SensorRatePolicy = "split"

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("sensor clock name must not be empty")
        if not self.kind.strip():
            raise ValueError("sensor clock kind must not be empty")
        if not math.isfinite(self.cadence_s) or self.cadence_s <= 0.0:
            raise ValueError("sensor cadence must be finite and positive")
        if not math.isfinite(self.phase_s) or self.phase_s < 0.0:
            raise ValueError("sensor phase must be finite and non-negative")
        if not math.isfinite(self.delivery_s) or self.delivery_s < 0.0:
            raise ValueError("sensor delivery latency must be finite and non-negative")
        if self.sample_mode == "instantaneous" and self.truth_policy != "boundary":
            raise ValueError("instantaneous sensors require truth_policy='boundary'")
        if self.sample_mode == "interval" and self.truth_policy != "accepted-segment":
            raise ValueError("interval sensors require truth_policy='accepted-segment'")
        if self.sample_mode == "instantaneous" and self.rate_policy != "split":
            raise ValueError("instantaneous sensors require rate_policy='split'")
        if self.sample_mode == "interval" and self.rate_policy != "accumulate":
            raise ValueError("interval sensors require rate_policy='accumulate'")
        ####

    def next_truth_time(self, current_time: float) -> float:
        """Return the next clock boundary strictly after ``current_time``."""

        if not math.isfinite(current_time):
            raise ValueError("current time must be finite")
        if current_time < self.phase_s:
            return self.phase_s
        index = math.floor((current_time - self.phase_s) / self.cadence_s + 1e-12) + 1
        return self.phase_s + index * self.cadence_s
        ####

    def to_metadata(self) -> dict[str, object]:
        """Return a JSON-compatible declaration for inspection and artifacts."""

        return {
            "name": self.name,
            "kind": self.kind,
            "cadence_s": self.cadence_s,
            "phase_s": self.phase_s,
            "sample_mode": self.sample_mode,
            "delivery_s": self.delivery_s,
            "truth_policy": self.truth_policy,
            "rate_policy": self.rate_policy,
        }
        ####
    ####
