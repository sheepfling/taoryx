"""Focused checks for the requested DAVE-ML Alpha 3 qualification slice."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.daveml


def _report(name: str) -> dict[str, object]:
    return json.loads((ROOT / "verification" / name).read_text(encoding="utf-8"))


def test_requested_alpha3_gates_are_verified_with_explicit_hl20_disposition() -> None:
    report = _report("daveml_alpha3_qualification.json")

    assert report["status"] == "verified"
    assert all(gate["status"] == "pass" for gate in report["gates"].values())
    assert report["deferred_families"]["reference_hl20_mod_k"]
    assert report["lineage_policy"]["parent_vehicle_capability"] == "separate from deployment child claims"


def test_f16_overlay_and_reductions_preserve_claim_boundaries() -> None:
    overlay = _report("daveml_f16_overlay_qualification.json")
    reduction = _report("daveml_f16_reduction_qualification.json")

    assert overlay["status"] == "verified"
    assert all(overlay["checks"].values())
    assert overlay["contract"]["latency_s"] == 0.0
    assert reduction["status"] == "verified"
    assert reduction["reductions"]["f16-point-mass-3dof-v1"]["status"] == "verified_local_projection"
    assert reduction["reductions"]["f16-attitude-response-pseudo6dof-v1"]["status"] == "verified_surrogate_response"
    assert "nonlinear source-equivalence" in reduction["nonclaims"]


def test_nesc_staging_lineage_keeps_deployment_child_separate() -> None:
    lineage = _report("daveml_nesc_staging_lineage.json")
    reduction = _report("daveml_nesc_reduction_qualification.json")

    assert lineage["status"] == "verified"
    assert [event["event"] for event in lineage["events"]][:3] == [
        "liftoff_stage1_ignition",
        "stage1_burnout",
        "stage1_jettison_stage2_ignition",
    ]
    assert lineage["lineage"]["parent_qualification_unchanged_by_child"] is True
    assert reduction["reduction"]["status"] == "verified_bounded_replay"
    assert reduction["pseudo_reduction_disposition"]["status"] == "deferred"


def test_a320_common_channels_match_without_comparing_rotational_authorities() -> None:
    report = _report("daveml_a320_matched_comparison.json")

    assert report["status"] == "verified"
    assert report["metrics"]["matched_point_count"] == 3
    assert report["metrics"]["max_common_channel_absolute_error"] == pytest.approx(0.0)
    assert set(report["noncomparable_channels"]) >= {"roll_moment_nm", "pitch_moment_nm", "yaw_moment_nm"}
    assert all(item["pass"] for point in report["matched_points"] for item in point["common_channels"])
