from __future__ import annotations

import pytest

from taoryx.objectives import ObjectiveSpec, score_objectives
from taoryx.trajectory import (
    EvaluationGate,
    EvaluationMetric,
    EvidenceChannel,
    TrajectoryEvaluation,
    objective_report_to_evaluation,
)


def test_evaluation_separates_validity_qualification_feasibility_and_outcome() -> None:
    report = TrajectoryEvaluation(
        scenario_id="x8-rectangle-v1",
        validity="valid",
        qualification="extended",
        feasibility="likely_feasible",
        outcome="completed_degraded",
        metrics=(
            EvaluationMetric(
                id="return-error",
                actual=12.0,
                target=0.0,
                tolerance=20.0,
                quality_limit=5.0,
                slack=8.0,
                normalized_error=0.6,
                quality_normalized_error=2.4,
                unit="m",
                status="pass",
                source="telemetry",
            ),
        ),
        gates=(EvaluationGate(id="closure", status="pass", metric_ids=("return-error",)),),
        requested_controls=(EvidenceChannel(id="bank-command", value=0.2, unit="rad", source="requested"),),
        achieved_controls=(EvidenceChannel(id="bank-actual", value=0.19, unit="rad", source="achieved"),),
        resources=(EvidenceChannel(id="fuel-remaining", value=1.2, unit="kg", source="resource"),),
        claim_boundary="local source-bounded research evidence",
    )

    assert report.required_gates_pass
    assert report.as_dict()["outcome"] == "completed_degraded"
    assert report.requested_controls[0].id != report.achieved_controls[0].id


def test_numeric_evidence_requires_units_and_available_values() -> None:
    with pytest.raises(ValueError, match="must declare a unit"):
        EvidenceChannel(id="speed", value=10.0, source="diagnostic")
    with pytest.raises(ValueError, match="requires a value"):
        EvidenceChannel(id="missing", source="resource")


def test_evaluation_rejects_duplicate_metric_ids_across_closure_and_metrics() -> None:
    metric = EvaluationMetric(id="closure", unit="1", status="pass", source="rhs")
    with pytest.raises(ValueError, match="metric IDs"):
        TrajectoryEvaluation(
            scenario_id="duplicate",
            validity="valid",
            qualification="qualified",
            feasibility="feasible",
            outcome="completed",
            metrics=(metric,),
            closure=(metric,),
            claim_boundary="test",
        )


def test_objective_report_is_promoted_without_recomputing_score() -> None:
    report = score_objectives(
        (
            ObjectiveSpec("capture", "waypoint", "range_m", 0.0, 10.0, "m", quality_limit=2.0),
        ),
        {"range_m": 6.0},
    )
    evaluation = objective_report_to_evaluation(
        report,
        scenario_id="demo",
        validity="valid",
        qualification="extended",
        feasibility="feasible",
        outcome="completed_degraded",
        claim_boundary="bounded research evidence",
        closure=(
            EvaluationMetric(
                id="force-closure",
                actual=1.0e-9,
                target=0.0,
                tolerance=1.0e-8,
                unit="1",
                status="pass",
                source="rhs",
            ),
        ),
    )
    assert evaluation.metrics[0].normalized_error == report["objectives"][0]["normalized_error"]
    assert evaluation.required_gates_pass
    assert evaluation.gates[0].status == "pass"


def test_failed_closure_is_not_hidden_by_a_passing_objective_report() -> None:
    report = score_objectives(
        (ObjectiveSpec("capture", "waypoint", "range_m", 0.0, 10.0, "m"),),
        {"range_m": 0.0},
    )
    evaluation = objective_report_to_evaluation(
        report,
        scenario_id="closure-failure",
        validity="valid",
        qualification="extended",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="bounded research evidence",
        closure=(
            EvaluationMetric(
                id="force-closure",
                actual=1.0,
                target=0.0,
                tolerance=1.0e-8,
                unit="1",
                status="fail",
                source="independent-finite-difference",
            ),
        ),
    )
    assert evaluation.gates[0].status == "pass"
    assert not evaluation.required_gates_pass
