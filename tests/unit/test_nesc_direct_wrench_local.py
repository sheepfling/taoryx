"""Regression checks for the NESC engineering direct-wrench screen."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_nesc_direct_wrench_screen_preserves_missing_source_attitude_boundary() -> None:
    payload = json.loads((ROOT / "verification/alpha3_nesc_direct_wrench/manifest.json").read_text(encoding="utf-8"))
    assert payload["status"] == "nominal_case_pass"
    assert payload["evaluation"]["mission_pass"] is True
    assert payload["claim"]["evidence_tier"] == "T3_direct_wrench_bridge"
    assert payload["claim"]["direct_body_moment_injection"] is True
    assert payload["claim"]["physical_effector_allocation"] is False
    boundary = payload["source_boundary"]
    assert boundary["source_attitude_available"] is False
    assert boundary["source_gimbal_available"] is False
    assert boundary["source_force_available"] is False
    assert boundary["translation_derived_force"] is True
    assert payload["evaluation"]["wrench_statuses_observed"] == ["feasible"]
    ####
