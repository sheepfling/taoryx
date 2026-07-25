"""Provider-neutral trajectory assessment and evidence channels.

The evaluation envelope keeps model validity, qualification, preflight
feasibility, and execution outcome separate.  It is intentionally independent
of a simulator backend and can therefore be attached to Taoryx, analytical,
replay, or future provider results without changing the physics transition.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

ValidityStatus = Literal["valid", "invalid", "blocked", "not_run"]
QualificationStatus = Literal["qualified", "extended", "unqualified", "blocked", "not_run"]
FeasibilityStatus = Literal[
    "feasible",
    "likely_feasible",
    "unknown",
    "likely_infeasible",
    "infeasible",
    "not_run",
]
OutcomeStatus = Literal[
    "completed",
    "completed_degraded",
    "partial",
    "resource_limited",
    "envelope_limited",
    "time_limited",
    "capability_missing",
    "infeasible_preflight",
    "aborted",
    "numerical_failure",
    "not_run",
]
ChannelSource = Literal["requested", "achieved", "resource", "event", "diagnostic"]
ChannelStatus = Literal["available", "unavailable", "invalid"]
AssessmentStatus = Literal["pass", "fail", "blocked", "advisory"]


class EvidenceChannel(BaseModel):
    """One unit-bearing requested, achieved, resource, or event channel."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    value: float | bool | None = None
    unit: str | None = None
    frame: str | None = None
    source: ChannelSource
    status: ChannelStatus = "available"
    time_s: float | None = Field(default=None, ge=0.0)
    provenance: str | None = None

    @model_validator(mode="after")
    def require_unit_for_numeric(self) -> EvidenceChannel:
        if isinstance(self.value, (int, float)) and not isinstance(self.value, bool) and not self.unit:
            raise ValueError(f"numeric evidence channel {self.id!r} must declare a unit")
        if self.status == "available" and self.value is None:
            raise ValueError(f"available evidence channel {self.id!r} requires a value")
        return self
        ####
    ####


class EvaluationMetric(BaseModel):
    """One scored metric with explicit target, tolerance, and quality scale."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    actual: float | bool | None = None
    target: float | bool | None = None
    tolerance: float | None = Field(default=None, gt=0.0)
    quality_limit: float | None = Field(default=None, gt=0.0)
    slack: float | None = None
    normalized_error: float | None = None
    quality_normalized_error: float | None = None
    unit: str = Field(min_length=1)
    status: AssessmentStatus
    severity: Literal["required", "advisory"] = "required"
    source: str = Field(min_length=1)
    time_s: float | None = Field(default=None, ge=0.0)


class EvaluationGate(BaseModel):
    """A non-score validity or evidence gate."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: AssessmentStatus
    message: str = ""
    metric_ids: tuple[str, ...] = ()


class TrajectoryEvaluation(BaseModel):
    """Complete claim-bounded assessment for one resolved trajectory."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    scenario_id: str = Field(min_length=1)
    scenario_contract_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    validity: ValidityStatus
    qualification: QualificationStatus
    feasibility: FeasibilityStatus
    outcome: OutcomeStatus
    metrics: tuple[EvaluationMetric, ...] = ()
    gates: tuple[EvaluationGate, ...] = ()
    requested_controls: tuple[EvidenceChannel, ...] = ()
    achieved_controls: tuple[EvidenceChannel, ...] = ()
    resources: tuple[EvidenceChannel, ...] = ()
    events: tuple[EvidenceChannel, ...] = ()
    closure: tuple[EvaluationMetric, ...] = ()
    convergence: tuple[EvaluationMetric, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def unique_measurements(self) -> TrajectoryEvaluation:
        metric_ids = [item.id for item in (*self.metrics, *self.closure, *self.convergence)]
        if len(metric_ids) != len(set(metric_ids)):
            raise ValueError("trajectory evaluation metric IDs must be unique")
        channel_ids = [
            item.id
            for item in (
                *self.requested_controls,
                *self.achieved_controls,
                *self.resources,
                *self.events,
            )
        ]
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError("trajectory evaluation channel IDs must be unique")
        return self
        ####
    ####

    @property
    def required_gates_pass(self) -> bool:
        """Return whether all required evidence metrics and gates pass."""

        return all(item.status in {"pass", "advisory"} for item in self.gates) and all(
            item.status in {"pass", "advisory"} for item in self.metrics if item.severity == "required"
        ) and all(
            item.status in {"pass", "advisory"}
            for item in (*self.closure, *self.convergence)
            if item.severity == "required"
        )
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return stable JSON-compatible assessment data."""

        return self.model_dump(mode="json")
        ####


def objective_report_to_evaluation(
    report: Mapping[str, Any],
    *,
    scenario_id: str,
    validity: ValidityStatus,
    qualification: QualificationStatus,
    feasibility: FeasibilityStatus,
    outcome: OutcomeStatus,
    claim_boundary: str,
    scenario_contract_sha256: str | None = None,
    requested_controls: Sequence[EvidenceChannel] = (),
    achieved_controls: Sequence[EvidenceChannel] = (),
    resources: Sequence[EvidenceChannel] = (),
    events: Sequence[EvidenceChannel] = (),
    closure: Sequence[EvaluationMetric] = (),
    convergence: Sequence[EvaluationMetric] = (),
) -> TrajectoryEvaluation:
    """Promote one objective-score report into the neutral evidence contract.

    ``score_objectives`` remains the single source of truth for objective
    arithmetic.  This adapter only preserves its results in the typed
    trajectory-evaluation envelope and adds an explicit aggregate gate.  A
    score of ``diagnostic`` therefore remains an advisory result, while a
    blocked or failed required objective remains a failed evidence gate.
    """

    raw_objectives = report.get("objectives", ())
    if not isinstance(raw_objectives, Sequence) or isinstance(raw_objectives, (str, bytes)):
        raise ValueError("objective report objectives must be a sequence")
    metrics: list[EvaluationMetric] = []
    for raw in raw_objectives:
        if not isinstance(raw, Mapping):
            raise ValueError("objective report contains a non-mapping objective")
        status = raw.get("status")
        if status not in {"pass", "fail", "blocked"}:
            raise ValueError(f"objective report contains invalid status: {status!r}")
        severity = raw.get("severity", "required")
        if severity not in {"required", "advisory"}:
            raise ValueError(f"objective report contains invalid severity: {severity!r}")
        metrics.append(
            EvaluationMetric(
                id=str(raw.get("id", "")),
                actual=raw.get("actual"),
                target=raw.get("target"),
                tolerance=raw.get("tolerance"),
                quality_limit=raw.get("quality_limit"),
                slack=raw.get("slack"),
                normalized_error=raw.get("normalized_error"),
                quality_normalized_error=raw.get("quality_normalized_error"),
                unit=str(raw.get("unit", "")),
                status=status,
                severity=severity,
                source=str(raw.get("source", "objective-report")),
                time_s=raw.get("time_s"),
            )
        )
    report_status = report.get("status")
    if report_status == "pass":
        aggregate_status: AssessmentStatus = "pass"
    elif report_status == "diagnostic":
        aggregate_status = "advisory"
    elif report_status == "blocked":
        aggregate_status = "blocked"
    else:
        aggregate_status = "fail"
    return TrajectoryEvaluation(
        scenario_id=scenario_id,
        scenario_contract_sha256=scenario_contract_sha256,
        validity=validity,
        qualification=qualification,
        feasibility=feasibility,
        outcome=outcome,
        metrics=tuple(metrics),
        gates=(
            EvaluationGate(
                id="objective-report",
                status=aggregate_status,
                message=f"objective report status: {report_status!r}",
                metric_ids=tuple(metric.id for metric in metrics),
            ),
        ),
        requested_controls=tuple(requested_controls),
        achieved_controls=tuple(achieved_controls),
        resources=tuple(resources),
        events=tuple(events),
        closure=tuple(closure),
        convergence=tuple(convergence),
        claim_boundary=claim_boundary,
    )
    ####


__all__ = [
    "AssessmentStatus",
    "ChannelSource",
    "ChannelStatus",
    "EvaluationGate",
    "EvaluationMetric",
    "EvidenceChannel",
    "FeasibilityStatus",
    "OutcomeStatus",
    "objective_report_to_evaluation",
    "QualificationStatus",
    "TrajectoryEvaluation",
    "ValidityStatus",
]
####
