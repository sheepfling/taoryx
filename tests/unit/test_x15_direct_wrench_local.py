"""Regression checks for the X-15 local direct-wrench screen."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_x15_direct_wrench_screen_is_auditable_but_not_physical_effector_control() -> None:
    payload = json.loads((ROOT / "verification/alpha3_x15_direct_wrench/manifest.json").read_text(encoding="utf-8"))
    assert payload["status"] == "nominal_case_pass"
    assert payload["evaluation"]["mission_pass"] is True
    assert payload["claim"]["evidence_tier"] == "T3_direct_wrench_bridge"
    assert payload["claim"]["direct_body_moment_injection"] is True
    assert payload["claim"]["physical_effector_allocation"] is False
    assert payload["source_trim_boundary"]["direct_bias_is_not_source_trim"] is True
    assert payload["evaluation"]["wrench_statuses_observed"] == ["feasible"]
    ####
