"""Normalize family execution evidence into one Mission Composition evaluation record.

Family runners retain their source-appropriate telemetry, objective evaluator,
and detailed evidence.  This module is intentionally a projection *after* the
family result exists: it does not re-evaluate truth, manufacture unavailable
control/resource channels, or promote a nominal result to qualification.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Literal

from .composition_control_trace import validate_committed_control_trace
from .trajectory.evaluation import (
    AssessmentStatus,
    EvaluationGate,
    EvaluationMetric,
    EvidenceChannel,
    FeasibilityStatus,
    OutcomeStatus,
    TrajectoryEvaluation,
)
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_preflight import VehicleExecutionPreflight

if TYPE_CHECKING:
    from .plugins import PluginCatalog


def build_composition_trajectory_evaluation(
    composition: CompiledVehicleComposition,
    preflight: VehicleExecutionPreflight,
    truth_evaluation: Mapping[str, object],
    *,
    runtime: Mapping[str, object],
    envelope: Mapping[str, object],
    claim_boundary: str,
    status_trace: Mapping[str, object] | None = None,
    control_trace: Mapping[str, object] | None = None,
    plugins: PluginCatalog | None = None,
) -> TrajectoryEvaluation:
    """Project an existing independent truth report into the neutral envelope.

    The accepted forms are the current truth-objective report (`results`) and
    the retained replay/passive report (`required_objectives`).  Both remain
    attached verbatim as ``objective_report.json``; this envelope gives UI,
    search, and catalog clients a stable outcome/gate vocabulary.
    """

    objective_pass = truth_evaluation.get("mission_pass") is True
    numerical_pass = _optional_bool(runtime.get("numerical_valid"))
    if numerical_pass is None:
        numerical_pass = _optional_bool(truth_evaluation.get("numerical_pass"))
    execution_limited = _execution_limited(runtime)
    envelope_pass = _optional_bool(envelope.get("pass"))
    hard_gates_pass = _optional_bool(runtime.get("hard_gates_passed"))
    metrics = tuple(_objective_metrics(truth_evaluation))
    requested_controls, achieved_controls, resources = _declared_evidence_channels(
        composition,
        status_trace,
        control_trace,
        plugins=plugins,
    )
    return TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid" if execution_limited or numerical_pass is not False else "invalid",
        qualification="unqualified",
        feasibility=_preflight_feasibility(preflight),
        outcome=_outcome(objective_pass, numerical_pass, envelope_pass, hard_gates_pass, execution_limited),
        metrics=metrics,
        gates=(
            EvaluationGate(
                id="semantic-preflight",
                status="pass" if preflight.status == "translation_ready" else "blocked",
                message="exact family translator accepted the resolved composition"
                if preflight.status == "translation_ready"
                else "; ".join(preflight.diagnostics),
            ),
            EvaluationGate(
                id="independent-truth-objectives",
                status="pass" if objective_pass else "fail",
                message="family objective report scanned committed truth telemetry",
                metric_ids=tuple(metric.id for metric in metrics),
            ),
            _gate("runtime-hard-gates", hard_gates_pass, "family runtime hard-gate result"),
            _gate("envelope", envelope_pass, "family envelope result"),
            _variant_runtime_evidence_gate(runtime),
            _graph_execution_gate(runtime),
        ),
        requested_controls=requested_controls,
        achieved_controls=achieved_controls,
        resources=resources,
        claim_boundary=(
            "This normalized Mission Composition result projects the family-owned independent truth report and declared "
            "runtime/envelope gates. It neither recomputes family physics nor promotes a nominal result beyond "
            "the selected fidelity/evidence tier. "
            + claim_boundary
        ),
    )
    ####


def _preflight_feasibility(preflight: VehicleExecutionPreflight) -> FeasibilityStatus:
    """Map the retained family estimate without upgrading its evidence class.

    Semantic translation readiness only establishes that a family translator
    accepted the composition. It does not turn a deliberately conservative
    ``likely_feasible`` or ``unknown`` estimate into a confirmed feasible
    mission. This mapping preserves the finer capability vocabulary while
    keeping the public evaluation envelope's stable feasibility terms.
    """

    if preflight.status != "translation_ready":
        return "unknown"
    capability = preflight.capability_estimate
    if not isinstance(capability, Mapping):
        return "unknown"
    feasibility = capability.get("feasibility")
    if not isinstance(feasibility, str):
        return "unknown"
    mapping: dict[str, FeasibilityStatus] = {
        "feasible": "feasible",
        "likely_feasible": "likely_feasible",
        "unknown": "unknown",
        "likely_infeasible": "likely_infeasible",
        "certainly_infeasible": "infeasible",
    }
    return mapping.get(feasibility, "unknown")
    ####


def _declared_evidence_channels(
    composition: CompiledVehicleComposition,
    status_trace: Mapping[str, object] | None,
    control_trace: Mapping[str, object] | None,
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[tuple[EvidenceChannel, ...], tuple[EvidenceChannel, ...], tuple[EvidenceChannel, ...]]:
    """Project only run evidence that the composed interface can substantiate.

    A batch status trace is committed truth/status evidence, not a control
    command log. A dedicated semantic action trace can provide the held command
    that ended at the final committed truth boundary. It never turns a
    pseudo-response coordinate into a physical effector. Resource channels can
    be available when the final committed status sample provides a scalar or
    boolean value.
    """

    contract = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    control_time_s, requested_values, achieved_values = _final_control_sample(
        composition,
        control_trace,
        plugins=plugins,
    )
    requested = tuple(
        _control_evidence_channel(
            channel.id,
            channel.canonical_unit,
            channel.frame,
            requested_values.get(channel.id),
            control_time_s,
            source="requested",
            role="semantic action",
        )
        for channel in contract.action_channels
        if channel.availability in {"available", "available_in_batch"}
    )
    achieved = tuple(
        _control_evidence_channel(
            channel.id,
            channel.canonical_unit,
            channel.frame,
            achieved_values.get(channel.id),
            control_time_s,
            source="achieved",
            role="physical effector",
        )
        for channel in contract.effector_channels
        if channel.availability in {"available", "available_in_batch"}
    )
    final_time_s, final_values = _final_status_sample(status_trace)
    resources: list[EvidenceChannel] = []
    for channel in contract.resource_channels:
        value = final_values.get(channel.id)
        if isinstance(value, bool) or isinstance(value, int | float):
            resources.append(
                EvidenceChannel(
                    id=channel.id,
                    value=value,
                    unit=channel.canonical_unit,
                    frame=channel.frame,
                    source="resource",
                    status="available",
                    time_s=final_time_s,
                    provenance="final committed truth/status sample from status_trace.json",
                )
            )
        else:
            resources.append(
                EvidenceChannel(
                    id=channel.id,
                    source="resource",
                    status="unavailable",
                    provenance=(
                        "No scalar or boolean final committed resource value was present in status_trace.json; "
                        "this normalized result does not infer one."
                    ),
                )
            )
    return requested, achieved, tuple(resources)
    ####


def _final_control_sample(
    composition: CompiledVehicleComposition,
    control_trace: Mapping[str, object] | None,
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[float | None, Mapping[str, object], Mapping[str, object]]:
    """Return the final held command/effector sample after identity validation."""

    if control_trace is None:
        return None, {}, {}
    validate_committed_control_trace(composition, control_trace, plugins=plugins)
    samples = control_trace.get("samples")
    if not isinstance(samples, Sequence) or isinstance(samples, str | bytes) or not samples:
        raise ValueError("validated committed control trace has no samples")
    final = samples[-1]
    if not isinstance(final, Mapping):
        raise ValueError("validated committed control trace has an invalid final sample")
    requested = final.get("requested_actions")
    achieved = final.get("achieved_effectors")
    return (
        _finite(final.get("committed_truth_time_s")),
        requested if isinstance(requested, Mapping) else {},
        achieved if isinstance(achieved, Mapping) else {},
    )
    ####


def _control_evidence_channel(
    identifier: str,
    unit: str | None,
    frame: str | None,
    value: object | None,
    time_s: float | None,
    *,
    source: Literal["requested", "achieved"],
    role: str,
) -> EvidenceChannel:
    """Project one exact final scalar/boolean control value when available."""

    if isinstance(value, bool) or isinstance(value, int | float):
        return EvidenceChannel(
            id=f"{source}.{identifier}",
            value=value,
            unit=unit,
            frame=frame,
            source=source,
            status="available",
            time_s=time_s,
            provenance=(
                f"final held {role} value from semantic_action_trace.json at the committed truth boundary; "
                "it is retained as recorded and is not inferred from trajectory behavior."
            ),
        )
    return EvidenceChannel(
        id=f"{source}.{identifier}",
        source=source,
        status="unavailable",
        provenance=(
            f"The batch executor did not emit a standardized {role} trace for this declared channel; "
            "the absence is retained rather than inferred from native controls."
        ),
    )
    ####


def _final_status_sample(status_trace: Mapping[str, object] | None) -> tuple[float | None, Mapping[str, object]]:
    """Return the last committed status mapping without trusting partial data."""

    if status_trace is None:
        return None, {}
    samples = status_trace.get("samples")
    if not isinstance(samples, Sequence) or isinstance(samples, str | bytes) or not samples:
        return None, {}
    final = samples[-1]
    if not isinstance(final, Mapping):
        return None, {}
    time_s = _finite(final.get("time_s"))
    values = final.get("values")
    return time_s, values if isinstance(values, Mapping) else {}
    ####


def _objective_metrics(report: Mapping[str, object]) -> list[EvaluationMetric]:
    """Map heterogeneous current objective records to dimensionless margins."""

    raw = report.get("results", report.get("required_objectives", report.get("objectives", ())))
    if not isinstance(raw, Sequence) or isinstance(raw, str | bytes):
        return []
    metrics: list[EvaluationMetric] = []
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping):
            continue
        identifier = str(item.get("id", f"objective-{index + 1}"))
        status = _assessment_status(item.get("status", item.get("truth_result")))
        error = _finite(item.get("closest_error"))
        tolerance = _finite(item.get("tolerance"))
        margin = _finite(item.get("margin"))
        if error is not None and tolerance is not None and tolerance > 0.0:
            actual = error / tolerance
            slack = margin / tolerance if margin is not None else 1.0 - actual
        else:
            actual = None
            slack = None
        metrics.append(
            EvaluationMetric(
                id=f"objective.{identifier}.normalized_error",
                actual=actual,
                target=0.0 if actual is not None else None,
                tolerance=1.0 if actual is not None else None,
                slack=slack,
                normalized_error=actual,
                unit="1",
                status=status,
                severity="required" if item.get("required", True) else "advisory",
                source="independent_truth_telemetry",
                time_s=_finite(item.get("truth_time_s")),
            )
        )
    return metrics
    ####


def _assessment_status(value: object) -> AssessmentStatus:
    text = str(value).lower()
    if text in {"pass", "passed"}:
        return "pass"
    if text in {"blocked", "not_applicable"}:
        return "blocked"
    return "fail"
    ####


def _gate(identifier: str, value: bool | None, message: str) -> EvaluationGate:
    return EvaluationGate(
        id=identifier,
        status="advisory" if value is None else "pass" if value else "fail",
        message=message if value is not None else f"{message} unavailable in this family result",
    )
    ####


def _graph_execution_gate(runtime: Mapping[str, object]) -> EvaluationGate:
    """Expose graph-dispatch evidence without making it a truth-pass proxy."""

    artifact = runtime.get("mission_graph_execution")
    if not isinstance(artifact, Mapping):
        return EvaluationGate(
            id="mission-graph-execution",
            status="advisory",
            message="no graph-dispatch artifact was emitted; controller sequencing is not asserted",
        )
    observation_status = artifact.get("observation_status")
    if observation_status == "observed":
        completed = artifact.get("completed_nominal_success_path")
        return EvaluationGate(
            id="mission-graph-execution",
            status="advisory",
            message=(
                "executor emitted committed graph dispatches; nominal success-path completion="
                f"{completed!r}. Independent truth objectives remain the mission pass authority."
            ),
        )
    if observation_status == "unobserved":
        return EvaluationGate(
            id="mission-graph-execution",
            status="advisory",
            message="runner emitted no committed controller graph dispatches; graph completion is not asserted",
        )
    return EvaluationGate(
        id="mission-graph-execution",
        status="advisory",
        message="graph-dispatch artifact has no recognized observation status; controller sequencing is not asserted",
    )
    ####


def _variant_runtime_evidence_gate(runtime: Mapping[str, object]) -> EvaluationGate:
    """Expose selected-variant consumption without inventing a modifier claim."""

    artifact = runtime.get("variant_runtime_evidence")
    if not isinstance(artifact, Mapping):
        return EvaluationGate(
            id="variant-runtime-evidence",
            status="advisory",
            message="no variant runtime-consumption artifact was emitted",
        )
    status = artifact.get("status")
    if status == "pass":
        return EvaluationGate(
            id="variant-runtime-evidence",
            status="pass",
            message="every selected variant matched its declared native input and committed-status relation",
        )
    if status == "not_applicable":
        return EvaluationGate(
            id="variant-runtime-evidence",
            status="advisory",
            message="no variant was selected for this composition",
        )
    return EvaluationGate(
        id="variant-runtime-evidence",
        status="fail",
        message="a selected variant did not verify declared runtime consumption and committed-status evidence",
    )
    ####


def _optional_bool(value: object) -> bool | None:
    return value if isinstance(value, bool) else None
    ####


def _execution_limited(runtime: Mapping[str, object]) -> bool:
    """Recognize a family-owned, non-error execution cap.

    A capped prefix has not completed its mission objectives, but the emitted
    telemetry is still a valid bounded execution artifact.  Do not infer this
    from an exit code alone: only a family executor may label the limit.
    """

    return runtime.get("execution_limit_reason") == "max_steps"
    ####


def _finite(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if not isinstance(value, int | float | str):
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None
    ####


def _outcome(
    objective_pass: bool,
    numerical_pass: bool | None,
    envelope_pass: bool | None,
    hard_gates_pass: bool | None,
    execution_limited: bool,
) -> OutcomeStatus:
    if execution_limited:
        return "time_limited"
    if numerical_pass is False:
        return "numerical_failure"
    if envelope_pass is False:
        return "envelope_limited"
    if objective_pass and hard_gates_pass is not False:
        return "completed"
    return "partial"
    ####


__all__ = ["build_composition_trajectory_evaluation"]
