from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
import yaml

from taoryx.trajectory import load_f16_reference_plant, solve_f16_source_trim

ROOT = Path(__file__).resolve().parents[2]


def test_f16_operating_point_resolves_trim_into_runtime_state() -> None:
    """The F-16 trim artifact is explicit enough to seed a plant adapter."""

    catalog = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/qualification/operating-points.yaml").read_text(
            encoding="utf-8"
        )
    )
    evidence = json.loads(
        (ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(
            encoding="utf-8"
        )
    )
    point = catalog["points"][0]
    assert point["id"] == "f16-sea-level-152mps"
    assert point["state"]["alpha_deg"] == pytest.approx(evidence["state"]["alpha_deg"])
    assert point["controls"]["elevator_deg"] == pytest.approx(evidence["controls"]["elevator_deg"])
    assert point["controls"]["source_powerLeverAngle_pct"] == pytest.approx(evidence["controls"]["power_pct"])
    assert point["controls"]["throttle_fraction"] == pytest.approx(evidence["controls"]["power_pct"] / 100.0)
    alpha_rad = math.radians(point["state"]["alpha_deg"])
    speed = point["environment"]["true_airspeed_m_s"]
    velocity = point["state"]["body_velocity_m_s"]
    assert velocity["u"] == pytest.approx(speed * math.cos(alpha_rad))
    assert velocity["w"] == pytest.approx(speed * math.sin(alpha_rad))
    assert point["residuals"]["max_residual"] == pytest.approx(evidence["max_residual"])
    assert point["residuals"]["max_residual"] < 2.0e-8
    ####


def test_f16_operating_point_catalog_retrims_all_declared_schedule_witnesses() -> None:
    """Every catalog point resolves through the same nonlinear source plant."""

    catalog = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/qualification/operating-points.yaml").read_text(
            encoding="utf-8"
        )
    )
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    assert len(catalog["points"]) == 7
    for point in catalog["points"]:
        resolved = solve_f16_source_trim(
            source,
            point_id=point["id"],
            altitude_m=float(point["environment"]["geometric_altitude_m"]),
            true_airspeed_m_s=float(point["environment"]["true_airspeed_m_s"]),
            initial_alpha_deg=float(point["state"]["alpha_deg"]),
            initial_elevator_deg=float(point["controls"]["elevator_deg"]),
            initial_throttle_fraction=float(point["controls"]["throttle_fraction"]),
        )
        assert resolved.trim.success is True
        assert resolved.max_residual <= 3.0e-4
        assert resolved.body_state["v_m_s"] == pytest.approx(0.0)
        assert resolved.controls["aileron_deg"] == pytest.approx(0.0)
        assert resolved.controls["rudder_deg"] == pytest.approx(0.0)
    ####


def test_f16_operating_point_evidence_keeps_schedule_claim_local() -> None:
    """Generated schedule evidence must not silently promote a gain schedule."""

    evidence = json.loads(
        (ROOT / "verification/f16_operating_points_evidence.json").read_text(encoding="utf-8")
    )
    assert evidence["status"] == "development_schedule_witness_passed"
    assert evidence["point_count"] == 7
    assert "continuous transitions" in evidence["claim_boundary"]
    assert all(point["linearization"]["derivative_consistent"] for point in evidence["points"])
    assert all(point["local_lqr"]["hurwitz"] for point in evidence["points"])
    assert all(point["local_lqr"]["trim_hold_validation"]["saturation_fraction"] == 0.0 for point in evidence["points"])
    ####
