"""Unit-aware objective scoring for reproducible mission evidence.

This module evaluates external mission contracts.  It does not add syntax to
the historical TAOS language and it does not replace plant-level assertions.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence, Set
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import yaml

Comparison = Literal["absolute_error", "within_band", "minimum", "maximum", "event"]
MetricSource = Literal["final", "max", "max_abs", "summary"]
Severity = Literal["required", "advisory"]


@dataclass(frozen=True, slots=True)
class ObjectiveSpec:
    """One measurable, unit-bearing acceptance objective."""

    id: str
    kind: str
    channel: str
    target: float | bool | None
    tolerance: float | None
    unit: str
    comparison: Comparison = "absolute_error"
    source: MetricSource = "final"
    weight: float = 1.0
    phase: str | None = None
    severity: Severity = "required"

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.channel.strip():
            raise ValueError("objective id and channel must not be empty")
        if not self.unit.strip():
            raise ValueError(f"objective {self.id!r} must declare a unit")
        if not math.isfinite(self.weight) or self.weight <= 0.0:
            raise ValueError(f"objective {self.id!r} weight must be positive and finite")
        if self.comparison == "event":
            if self.tolerance is not None:
                raise ValueError(f"event objective {self.id!r} cannot have a numeric tolerance")
        else:
            if not isinstance(self.target, (int, float)) or isinstance(self.target, bool):
                raise ValueError(f"objective {self.id!r} requires a numeric target")
            if self.tolerance is None or not math.isfinite(self.tolerance) or self.tolerance <= 0.0:
                raise ValueError(f"objective {self.id!r} requires a positive tolerance")
            if not math.isfinite(float(self.target)):
                raise ValueError(f"objective {self.id!r} target must be finite")
        ####


@dataclass(frozen=True, slots=True)
class ObjectiveResult:
    """Raw and normalized result for one objective."""

    id: str
    kind: str
    status: Literal["pass", "fail", "blocked"]
    actual: float | bool | None
    target: float | bool | None
    error: float | None
    tolerance: float | None
    slack: float | None
    normalized_error: float | None
    unit: str
    time_s: float | None
    weight: float
    severity: Severity
    source: MetricSource
    message: str | None = None

    def as_dict(self) -> dict[str, object]:
        return asdict(self)
        ####


@dataclass(frozen=True, slots=True)
class ObjectiveCatalog:
    """External objective declarations for one scenario or family."""

    schema_version: int
    id: str
    objectives: tuple[ObjectiveSpec, ...]


def load_objective_catalog(path: str | Path) -> ObjectiveCatalog:
    """Load typed objective metadata from YAML without adding problem syntax."""

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"objective catalog must be a mapping: {path}")
    raw_objectives = payload.get("objectives", ())
    if not isinstance(raw_objectives, Sequence) or isinstance(raw_objectives, (str, bytes)):
        raise ValueError(f"objective catalog objectives must be a sequence: {path}")
    try:
        objectives = tuple(ObjectiveSpec(**item) for item in raw_objectives if isinstance(item, Mapping))
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid objective in catalog {path}: {error}") from error
    if len(objectives) != len(raw_objectives):
        raise ValueError(f"objective catalog contains a non-mapping objective: {path}")
    identifiers = [objective.id for objective in objectives]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError(f"objective catalog IDs must be unique: {path}")
    return ObjectiveCatalog(
        int(payload.get("schema_version", 1)),
        str(payload.get("id", Path(path).stem)),
        objectives,
    )
    ####


def _numeric(value: object, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result
    ####


def score_objective(
    spec: ObjectiveSpec,
    observed: Mapping[str, object],
    *,
    time_s: float | None = None,
    completed_events: Set[str] = frozenset(),
) -> ObjectiveResult:
    """Score one objective against canonical telemetry or completed events."""

    if spec.comparison == "event":
        actual = spec.id in completed_events or spec.channel in completed_events
        return ObjectiveResult(
            spec.id,
            spec.kind,
            "pass" if actual else "fail",
            actual,
            True,
            0.0 if actual else 1.0,
            None,
            0.0 if actual else -1.0,
            0.0 if actual else 1.0,
            spec.unit,
            time_s,
            spec.weight,
            spec.severity,
            spec.source,
            None if actual else "declared event did not occur",
        )
    if spec.channel not in observed or observed[spec.channel] is None:
        return ObjectiveResult(
            spec.id,
            spec.kind,
            "blocked",
            None,
            spec.target,
            None,
            spec.tolerance,
            None,
            None,
            spec.unit,
            time_s,
            spec.weight,
            spec.severity,
            spec.source,
            f"required channel {spec.channel!r} is unavailable",
        )
    actual_numeric = _numeric(observed[spec.channel], label=spec.channel)
    target = _numeric(spec.target, label=f"objective {spec.id} target")
    tolerance = _numeric(spec.tolerance, label=f"objective {spec.id} tolerance")
    if spec.comparison in {"absolute_error", "within_band"}:
        error = abs(actual_numeric - target)
    elif spec.comparison == "minimum":
        error = max(0.0, target - actual_numeric)
    else:
        error = max(0.0, actual_numeric - target)
    slack = tolerance - error
    normalized = error / tolerance
    return ObjectiveResult(
        spec.id,
        spec.kind,
        "pass" if error <= tolerance else "fail",
        actual_numeric,
        target,
        error,
        tolerance,
        slack,
        normalized,
        spec.unit,
        time_s,
        spec.weight,
        spec.severity,
        spec.source,
    )
    ####


def score_objectives(
    specs: Sequence[ObjectiveSpec],
    observed: Mapping[str, object],
    *,
    time_s: float | None = None,
    completed_events: Set[str] = frozenset(),
    termination: Mapping[str, object] | None = None,
    scenario_contract_sha256: str | None = None,
) -> dict[str, object]:
    """Return deterministic objective records and a claim-safe composite."""

    results = tuple(score_objective(spec, observed, time_s=time_s, completed_events=completed_events) for spec in specs)
    weighted_score = sum(
        spec.weight
        * (
            max(0.0, 1.0 - float(result.normalized_error))
            if result.normalized_error is not None and result.status == "pass"
            else 0.0
        )
        for spec, result in zip(specs, results, strict=True)
    )
    total_weight = sum(spec.weight for spec in specs)
    required = tuple(result for result in results if result.severity == "required")
    advisory = tuple(result for result in results if result.severity == "advisory")
    required_blocked = any(result.status == "blocked" for result in required)
    required_passed = sum(result.status == "pass" for result in required)
    if required_blocked:
        status = "blocked"
    elif any(result.status == "fail" for result in required):
        status = "fail"
    elif any(result.status != "pass" for result in advisory):
        status = "diagnostic"
    else:
        status = "pass"
    normalized_values = [
        float(result.normalized_error)
        for result in required
        if result.normalized_error is not None
    ]
    return {
        "status": status,
        "score": 100.0 * weighted_score / total_weight if total_weight else 0.0,
        "required_objectives": len(required),
        "required_passed": required_passed,
        "advisory_objectives": len(advisory),
        "worst_required_normalized_error": max(normalized_values, default=None),
        "objectives": [result.as_dict() for result in results],
        "termination": dict(termination or {}),
        "scenario_contract_sha256": scenario_contract_sha256,
    }
    ####
