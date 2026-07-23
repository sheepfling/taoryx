"""Promotion gates for vehicle/segment evidence before route composition.

The composition layer describes how segments are connected.  This module
describes whether an individual segment has earned the right to participate in
that composition.  A controller can be numerically stable while its segment
goal, entry state, handoff, or termination contract is still unproven.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.composition import RuntimeCheck, RuntimeCompositionReport, SegmentRuntimeEvaluation
from taoryx.segmentation import GoalKind, SegmentationCatalog, SegmentationScenario

PromotionCheck = Literal["entry", "goal", "transition", "exit", "quality", "termination"]
PromotionStatus = Literal["promoted", "blocked", "rejected"]


class SegmentPromotionSpec(BaseModel):
    """Required evidence contract for one vehicle/segment pair."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    vehicle: str = Field(min_length=1)
    scenario_id: str = Field(min_length=1)
    segment_id: str = Field(min_length=1)
    goal_kind: GoalKind
    required_checks: tuple[PromotionCheck, ...] = Field(min_length=1)
    required_channels: tuple[str, ...] = ()
    focused_test: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_checks(self) -> SegmentPromotionSpec:
        if len(self.required_checks) != len(set(self.required_checks)):
            raise ValueError(f"promotion {self.id!r} repeats a required check")
        if any(not name.strip() for name in self.required_channels):
            raise ValueError(f"promotion {self.id!r} contains an empty required channel")
        if len(self.required_channels) != len(set(self.required_channels)):
            raise ValueError(f"promotion {self.id!r} repeats a required channel")
        return self
        ####


class SegmentPromotionCatalog(BaseModel):
    """Machine-readable promotion matrix for the segment library."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = 1
    id: str = Field(min_length=1)
    promotions: tuple[SegmentPromotionSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_ids(self) -> SegmentPromotionCatalog:
        identifiers = [promotion.id for promotion in self.promotions]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("segment promotion IDs must be unique")
        return self
        ####


class SegmentPromotionResult(BaseModel):
    """Evidence decision and deterministic stamp for one promotion row."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    promotion_id: str
    vehicle: str
    scenario_id: str
    segment_id: str
    status: PromotionStatus
    required_checks: tuple[PromotionCheck, ...]
    checks: tuple[RuntimeCheck, ...] = ()
    evidence_hash: str | None = None
    messages: tuple[str, ...] = ()

    @property
    def promoted(self) -> bool:
        """Return whether this segment has earned composition promotion."""

        return self.status == "promoted"
        ####


class SegmentPromotionReport(BaseModel):
    """Aggregate promotion decisions used as a route-composition gate."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    scenario_id: str
    vehicle: str
    status: PromotionStatus
    results: tuple[SegmentPromotionResult, ...]
    messages: tuple[str, ...] = ()

    @property
    def composition_ready(self) -> bool:
        """Return whether every declared segment promotion passed."""

        return self.status == "promoted"

    def require_composition_ready(self) -> None:
        """Raise when a route attempts to use an unpromoted segment."""

        if not self.composition_ready:
            details = "; ".join(self.messages) or "one or more segments are not promoted"
            raise ValueError(f"composition promotion for {self.scenario_id!r} is not ready: {details}")
        ####


def load_segment_promotion_catalog(path: str | Path) -> SegmentPromotionCatalog:
    """Load a typed promotion matrix from YAML."""

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"segment promotion catalog must be a mapping: {path}")
    raw_promotions = payload.get("promotions", ())
    if not isinstance(raw_promotions, Sequence) or isinstance(raw_promotions, (str, bytes)):
        raise ValueError(f"segment promotion catalog promotions must be a sequence: {path}")
    if len(raw_promotions) != len([item for item in raw_promotions if isinstance(item, Mapping)]):
        raise ValueError(f"segment promotion catalog contains a non-mapping promotion: {path}")
    return SegmentPromotionCatalog(
        schema_version=int(payload.get("schema_version", 1)),
        id=str(payload.get("id", Path(path).stem)),
        promotions=tuple(SegmentPromotionSpec(**item) for item in raw_promotions),
    )
    ####


def validate_promotion_coverage(
    promotion_catalog: SegmentPromotionCatalog,
    segmentation_catalog: SegmentationCatalog,
) -> tuple[str, ...]:
    """Return catalog coverage errors without claiming runtime promotion."""

    declared = {(promotion.scenario_id, promotion.segment_id): promotion for promotion in promotion_catalog.promotions}
    errors: list[str] = []
    expected: set[tuple[str, str]] = set()
    for scenario in segmentation_catalog.scenarios:
        for segment in scenario.segments:
            key = (scenario.id, segment.id)
            expected.add(key)
            promotion = declared.get(key)
            if promotion is None:
                errors.append(f"missing promotion row for {scenario.id}/{segment.id}")
                continue
            if promotion.vehicle != scenario.vehicle:
                errors.append(f"promotion {promotion.id!r} vehicle does not match scenario {scenario.id!r}")
            if segment.goal is None or segment.goal.kind != promotion.goal_kind:
                actual = None if segment.goal is None else segment.goal.kind
                errors.append(f"promotion {promotion.id!r} goal kind {promotion.goal_kind!r} does not match {actual!r}")
    for key, promotion in declared.items():
        if key not in expected:
            errors.append(f"promotion {promotion.id!r} references unknown {promotion.scenario_id}/{promotion.segment_id}")
    return tuple(errors)
    ####


def evaluate_segment_promotion(
    spec: SegmentPromotionSpec,
    scenario: SegmentationScenario,
    runtime_report: RuntimeCompositionReport,
) -> SegmentPromotionResult:
    """Convert one runtime segment result into a promotion decision."""

    messages: list[str] = []
    segment = next((item for item in scenario.segments if item.id == spec.segment_id), None)
    runtime_segment = next((item for item in runtime_report.segments if item.segment_id == spec.segment_id), None)
    if scenario.id != spec.scenario_id or scenario.vehicle != spec.vehicle:
        messages.append("promotion row does not match the evaluated scenario")
    if segment is None:
        messages.append(f"scenario has no segment {spec.segment_id!r}")
    elif segment.goal is None or segment.goal.kind != spec.goal_kind:
        messages.append("promotion goal kind does not match the scenario goal")
    if runtime_segment is None:
        messages.append("runtime report has no evidence for this segment")
    if messages:
        return SegmentPromotionResult(
            promotion_id=spec.id,
            vehicle=spec.vehicle,
            scenario_id=spec.scenario_id,
            segment_id=spec.segment_id,
            status="blocked",
            required_checks=spec.required_checks,
            messages=tuple(messages),
        )
    assert runtime_segment is not None
    checks = tuple(check for name in spec.required_checks for check in _checks_for(runtime_segment, name))
    missing = [name for name in spec.required_checks if not _checks_for(runtime_segment, name)]
    required_channel_checks = tuple(
        check
        for channel in spec.required_channels
        for check in runtime_segment.quality_checks
        if check.name == f"quality:required-channel:{channel}"
    )
    missing_channel_evidence = tuple(
        channel
        for channel in spec.required_channels
        if not any(check.name == f"quality:required-channel:{channel}" for check in required_channel_checks)
    )
    for channel in spec.required_channels:
        if channel in missing_channel_evidence:
            messages.append(f"required runtime channel evidence {channel!r} was not emitted; evaluate with RuntimeEvaluationOptions.required_channels")
        elif any(check.status != "pass" for check in required_channel_checks if check.name == f"quality:required-channel:{channel}"):
            messages.append(f"required runtime channel {channel!r} did not pass")
    checks = (*checks, *required_channel_checks)
    if missing or missing_channel_evidence or any(check.status != "pass" for check in required_channel_checks):
        messages.extend(f"required evidence category {name!r} was not emitted" for name in missing)
    if runtime_segment.status == "fail":
        messages.append("segment runtime evidence contains a failed check")
        status: PromotionStatus = "rejected"
    elif runtime_segment.status == "blocked" or missing or missing_channel_evidence or any(check.status != "pass" for check in required_channel_checks):
        messages.append("segment runtime evidence is incomplete")
        status = "blocked"
    elif any(check.status != "pass" for check in checks):
        messages.append("every required evidence check must be pass before promotion")
        status = "blocked"
    else:
        status = "promoted"
    evidence_hash = _evidence_hash(spec, runtime_segment, checks) if status == "promoted" else None
    return SegmentPromotionResult(
        promotion_id=spec.id,
        vehicle=spec.vehicle,
        scenario_id=spec.scenario_id,
        segment_id=spec.segment_id,
        status=status,
        required_checks=spec.required_checks,
        checks=checks,
        evidence_hash=evidence_hash,
        messages=tuple(messages),
    )
    ####


def evaluate_promotion_catalog(
    catalog: SegmentPromotionCatalog,
    scenario: SegmentationScenario,
    runtime_report: RuntimeCompositionReport,
) -> SegmentPromotionReport:
    """Evaluate all promotion rows belonging to one scenario."""

    specs = tuple(item for item in catalog.promotions if item.scenario_id == scenario.id)
    results = tuple(evaluate_segment_promotion(spec, scenario, runtime_report) for spec in specs)
    messages: tuple[str, ...]
    if not results:
        status: PromotionStatus = "blocked"
        messages = (f"promotion catalog has no rows for scenario {scenario.id!r}",)
    elif any(result.status == "rejected" for result in results):
        status = "rejected"
        messages = tuple(f"{result.segment_id}: {message}" for result in results for message in result.messages)
    elif any(result.status == "blocked" for result in results):
        status = "blocked"
        messages = tuple(f"{result.segment_id}: {message}" for result in results for message in result.messages)
    else:
        status = "promoted"
        messages = ()
    return SegmentPromotionReport(
        scenario_id=scenario.id,
        vehicle=scenario.vehicle,
        status=status,
        results=results,
        messages=messages,
    )
    ####


def _checks_for(runtime_segment: SegmentRuntimeEvaluation, category: PromotionCheck) -> tuple[RuntimeCheck, ...]:
    if category == "termination":
        checks = tuple(check for check in runtime_segment.exit_checks if check.name.endswith("termination-reason"))
    else:
        checks = tuple(getattr(runtime_segment, f"{category}_checks"))
    return tuple(check for check in checks if check.status != "not_run")
    ####


def _evidence_hash(
    spec: SegmentPromotionSpec,
    runtime_segment: SegmentRuntimeEvaluation,
    checks: tuple[RuntimeCheck, ...],
) -> str:
    payload = {
        "spec": spec.model_dump(mode="json"),
        "segment": runtime_segment.model_dump(mode="json"),
        "required_checks": [check.model_dump(mode="json") for check in checks],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


__all__ = [
    "PromotionCheck",
    "PromotionStatus",
    "SegmentPromotionCatalog",
    "SegmentPromotionReport",
    "SegmentPromotionResult",
    "SegmentPromotionSpec",
    "evaluate_promotion_catalog",
    "evaluate_segment_promotion",
    "load_segment_promotion_catalog",
    "validate_promotion_coverage",
]
