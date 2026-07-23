from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.composition import RuntimeEvaluationOptions, TrajectoryBuilder, evaluate_composition_runtime
from taoryx.outputs import DynamicsKind, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry
from taoryx.segment_promotion import (
    SegmentPromotionSpec,
    evaluate_promotion_catalog,
    evaluate_segment_promotion,
    load_segment_promotion_catalog,
    validate_promotion_coverage,
)
from taoryx.segmentation import load_catalog

ROOT = Path(__file__).resolve().parents[2]


def test_guidance_segment_capability_matrix_distinguishes_evidence_from_candidates() -> None:
    payload = yaml.safe_load((ROOT / "verification/segment_capability_matrix.yaml").read_text(encoding="utf-8"))
    rows = {row["id"]: row for row in payload["segments"]}

    assert rows["x15-moving-target-pronav"]["status"] == "partial"
    assert rows["hummingbird-moving-target-pronav"]["status"] == "candidate"
    assert rows["x15-moving-target-pronav"]["focused_test"].startswith("tests/e2e/")
    assert rows["hummingbird-moving-target-pronav"]["focused_test"].startswith("tests/e2e/")
    assert all(name.startswith("pro_nav_") for row in rows.values() for name in row["required_channels"])


def test_promotion_matrix_covers_every_declared_lifecycle_segment() -> None:
    segmentation = load_catalog(ROOT / "verification/segmentation_catalog.yaml")
    promotions = load_segment_promotion_catalog(ROOT / "verification/segment_promotion.yaml")

    assert validate_promotion_coverage(promotions, segmentation) == ()
    assert len(promotions.promotions) == sum(len(scenario.segments) for scenario in segmentation.scenarios)
    assert all(item.focused_test.startswith("tests/") and "::test_" in item.focused_test for item in promotions.promotions)
    for item in promotions.promotions:
        test_path, test_name = item.focused_test.split("::", maxsplit=1)
        assert (ROOT / test_path).is_file(), item.focused_test
        assert test_name.startswith("test_")


def _scenario_and_artifact(*, altitude: list[float], termination: dict[str, object] | None = None):
    builder = TrajectoryBuilder(
        "promotion-demo",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use(
        "waypoint",
        "capture",
        duration_s=float(len(altitude) - 1),
        target={"position.altitude.geodetic": 10.0},
        tolerance={"position.altitude.geodetic": 0.5},
    )
    scenario = builder.build()
    channel = TelemetryChannel(
        source_name="altitude_m",
        semantic_name="position.altitude.geodetic",
        values=altitude,
    )
    vehicle = VehicleTelemetry(
        vehicle_id="demo",
        name="demo",
        kind=VehicleKind.GENERIC,
        dynamics=DynamicsKind.POINT_MASS_3DOF,
        times=[float(index) for index in range(len(altitude))],
        channels={channel.semantic_name: channel},
        segments=[SegmentSpan(number=1, title="capture", start_time=0.0, end_time=float(len(altitude) - 1))],
    )
    artifact = RunArtifact(
        problem="demo.prb",
        vehicles={"demo": vehicle},
        termination=termination or {"completed": True, "stop_reason": "stop_condition"},
    )
    return scenario, artifact


def test_promotion_stamp_requires_all_declared_categories_to_pass() -> None:
    scenario, artifact = _scenario_and_artifact(altitude=[10.0, 10.0])
    runtime = evaluate_composition_runtime(scenario, artifact)
    spec = SegmentPromotionSpec(
        id="demo-capture",
        vehicle="demo",
        scenario_id=scenario.id,
        segment_id="capture",
        goal_kind="waypoint",
        required_checks=("entry", "goal", "quality", "exit", "termination"),
        focused_test="tests/unit/test_segment_promotion.py::test_promotion_stamp_requires_all_declared_categories_to_pass",
        claim_boundary="test-only waypoint capture",
    )

    result = evaluate_segment_promotion(spec, scenario, runtime)

    assert result.promoted
    assert result.evidence_hash is not None
    assert len(result.evidence_hash) == 64


def test_promotion_stamp_requires_explicit_guidance_channels() -> None:
    scenario, artifact = _scenario_and_artifact(altitude=[10.0, 10.0])
    vehicle = artifact.vehicles["demo"]
    channels = dict(vehicle.channels)
    for name, values in (
        ("pro_nav_active", [1.0, 1.0]),
        ("pro_nav_los_range_m", [100.0, 25.0]),
        ("pro_nav_closing_velocity_m_s", [10.0, 0.0]),
    ):
        channels[name] = TelemetryChannel(source_name=name, semantic_name=name, values=values)
    artifact = artifact.model_copy(update={"vehicles": {"demo": vehicle.model_copy(update={"channels": channels})}})
    runtime = evaluate_composition_runtime(
        scenario,
        artifact,
        options=RuntimeEvaluationOptions(
            required_channels=("pro_nav_active", "pro_nav_los_range_m", "pro_nav_closing_velocity_m_s")
        ),
    )
    spec = SegmentPromotionSpec(
        id="demo-intercept",
        vehicle="demo",
        scenario_id=scenario.id,
        segment_id="capture",
        goal_kind="waypoint",
        required_checks=("goal", "quality", "exit", "termination"),
        required_channels=("pro_nav_active", "pro_nav_los_range_m", "pro_nav_closing_velocity_m_s"),
        focused_test="tests/unit/test_segment_promotion.py::test_promotion_stamp_requires_explicit_guidance_channels",
        claim_boundary="test-only guidance channel contract",
    )

    result = evaluate_segment_promotion(spec, scenario, runtime)

    assert result.promoted


def test_incomplete_termination_blocks_promotion() -> None:
    scenario, artifact = _scenario_and_artifact(
        altitude=[10.0, 10.0],
        termination={"completed": False, "stop_reason": "max_steps"},
    )
    runtime = evaluate_composition_runtime(scenario, artifact)
    spec = SegmentPromotionSpec(
        id="demo-capture",
        vehicle="demo",
        scenario_id=scenario.id,
        segment_id="capture",
        goal_kind="waypoint",
        required_checks=("goal", "exit", "termination"),
        focused_test="tests/unit/test_segment_promotion.py::test_incomplete_termination_blocks_promotion",
        claim_boundary="test-only waypoint capture",
    )

    result = evaluate_segment_promotion(spec, scenario, runtime)

    assert result.status == "rejected"
    assert not result.promoted


def test_catalog_report_requires_every_row_before_composition() -> None:
    scenario, artifact = _scenario_and_artifact(altitude=[10.0, 10.0])
    runtime = evaluate_composition_runtime(scenario, artifact)
    catalog = load_segment_promotion_catalog(ROOT / "verification/segment_promotion.yaml")

    report = evaluate_promotion_catalog(catalog, scenario, runtime)

    assert report.status == "blocked"
    with pytest.raises(ValueError, match="not ready"):
        report.require_composition_ready()
