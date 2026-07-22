"""Signed control-effect probes for the convention firewall.

The evaluator is supplied by a vehicle adapter and must return dimensional
force/moment channels in the canonical body frame.  The harness checks the
central signed derivative, opposite-command antisymmetry, and declared
expected sign without assuming fixed-wing or rotorcraft semantics.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Literal

ControlResponseEvaluator = Callable[[Mapping[str, float]], Mapping[str, float]]
Sign = Literal[-1, 0, 1] | None


@dataclass(frozen=True, slots=True)
class ControlDirectionProbe:
    """One signed control perturbation and the response channel to inspect."""

    control: str
    response: str
    delta: float
    expected_sign: Sign
    tolerance: float = 1e-9
    antisymmetry_tolerance: float | None = None

    def __post_init__(self) -> None:
        if not self.control or not self.response:
            raise ValueError("control direction probe requires control and response names")
        if self.delta <= 0.0:
            raise ValueError("control direction probe delta must be positive")
        if self.tolerance < 0.0:
            raise ValueError("control direction probe tolerance must be nonnegative")
        if self.antisymmetry_tolerance is not None and self.antisymmetry_tolerance < 0.0:
            raise ValueError("control direction antisymmetry tolerance must be nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlDirectionResult:
    """Auditable result for one control-effect probe."""

    probe: ControlDirectionProbe
    positive: float
    negative: float
    derivative: float
    antisymmetry_error: float
    passed: bool


def audit_control_directions(
    evaluator: ControlResponseEvaluator,
    baseline: Mapping[str, float],
    probes: tuple[ControlDirectionProbe, ...],
) -> tuple[ControlDirectionResult, ...]:
    """Probe all declared control directions through one plant evaluator.

    ``evaluator(commands)`` must return named force/moment responses.  Controls
    not under test remain at their baseline values.  A zero expected sign is a
    deliberate near-zero-effect assertion, useful for checking decoupling.
    """

    results: list[ControlDirectionResult] = []
    for probe in probes:
        baseline_response = float(evaluator(baseline)[probe.response])
        plus = dict(baseline)
        minus = dict(baseline)
        plus[probe.control] = float(plus.get(probe.control, 0.0)) + probe.delta
        minus[probe.control] = float(minus.get(probe.control, 0.0)) - probe.delta
        positive = float(evaluator(plus)[probe.response]) - baseline_response
        negative = float(evaluator(minus)[probe.response]) - baseline_response
        derivative = (positive - negative) / (2.0 * probe.delta)
        antisymmetry = (positive + negative) / 2.0
        sign_pass = (
            True
            if probe.expected_sign is None
            else abs(derivative) <= probe.tolerance
            if probe.expected_sign == 0
            else derivative * probe.expected_sign > probe.tolerance
        )
        results.append(
            ControlDirectionResult(
                probe=probe,
                positive=positive,
                negative=negative,
                derivative=derivative,
                antisymmetry_error=abs(antisymmetry),
                passed=bool(
                    sign_pass
                    and abs(antisymmetry)
                    <= max(probe.antisymmetry_tolerance or probe.tolerance, 1e-12)
                ),
            )
        )
    return tuple(results)
    ####
