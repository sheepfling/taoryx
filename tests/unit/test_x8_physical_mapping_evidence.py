"""Tests for the fail-closed X8 physical elevon mapping witness."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_x8_mapping_witness_records_source_selected_sign_and_alternate() -> None:
    """Both hypotheses remain auditable while the paper selects one."""

    artifact = json.loads(
        (ROOT / "verification/alpha3_x8_physical_mapping/manifest.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "X8_physical_left_right_mapping_resolved_by_source_equation"
    assert artifact["claim"]["promotion_authorized"] is True
    assert artifact["claim"]["direct_body_moment_injection"] is False
    assert artifact["source_status"]["status"] == "verify_before_use"
    assert len(artifact["hypotheses"]) == 2
    assert {item["differential_sign"] for item in artifact["hypotheses"]} == {1, -1}
    assert all(item["invertible"] for item in artifact["hypotheses"])
    assert artifact["discrimination"]["source_selects_one_hypothesis"] is True
    assert artifact["resolution"]["selected_mapping"]["differential_sign"] == 1
    assert artifact["resolution"]["inverse"] == "delta_el = delta_e + delta_a; delta_er = delta_e - delta_a"
    ####
