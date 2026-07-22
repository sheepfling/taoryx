from __future__ import annotations

import pytest

from taoryx.objectives import ObjectiveSpec, load_objective_catalog, score_objective, score_objectives


def test_objective_preserves_error_tolerance_and_slack() -> None:
    spec = ObjectiveSpec(
        id="mid-waypoint",
        kind="waypoint",
        channel="east_m",
        target=300.0,
        tolerance=2.0,
        unit="m",
    )
    result = score_objective(spec, {"east_m": 301.4}, time_s=42.0)
    assert result.status == "pass"
    assert result.error == pytest.approx(1.4)
    assert result.slack == pytest.approx(0.6)
    assert result.normalized_error == pytest.approx(0.7)
    assert result.unit == "m"
    ####


def test_composite_cannot_hide_failed_required_objective() -> None:
    specs = (
        ObjectiveSpec("near", "waypoint", "north_m", 2.0, 2.0, "m", weight=1.0),
        ObjectiveSpec("far", "waypoint", "east_m", 0.0, 2.0, "m", weight=1.0),
    )
    report = score_objectives(specs, {"north_m": 2.0, "east_m": 40_000.0})
    assert report["status"] == "fail"
    assert report["score"] == 50.0
    assert report["worst_required_normalized_error"] == 20_000.0
    ####


def test_missing_channel_blocks_required_goal_and_event_scores() -> None:
    state = ObjectiveSpec("altitude", "capture", "altitude_m", 2.0, 0.1, "m")
    event = ObjectiveSpec("release", "separation", "release", True, None, "event", comparison="event")
    report = score_objectives((state, event), {}, completed_events=())
    assert report["status"] == "blocked"
    assert report["score"] == 0.0
    assert [item["status"] for item in report["objectives"]] == ["blocked", "fail"]
    ####


def test_objectives_require_units_and_positive_tolerances() -> None:
    with pytest.raises(ValueError, match="must declare a unit"):
        ObjectiveSpec("bad", "capture", "altitude_m", 2.0, 0.1, "")
    with pytest.raises(ValueError, match="positive tolerance"):
        ObjectiveSpec("bad", "capture", "altitude_m", 2.0, 0.0, "m")
    ####


def test_objective_catalog_loads_external_yaml(tmp_path) -> None:
    path = tmp_path / "objectives.yaml"
    path.write_text(
        """schema_version: 1
id: demo-goals
objectives:
  - id: altitude
    kind: altitude_capture
    channel: altitude_m
    target: 2.0
    tolerance: 0.1
    unit: m
""",
        encoding="utf-8",
    )
    catalog = load_objective_catalog(path)
    assert catalog.id == "demo-goals"
    assert catalog.objectives[0].unit == "m"
    ####
