"""Tests for the Alpha 3 cross-fidelity report builder."""

from __future__ import annotations

from pathlib import Path

from tools.build_alpha3_cross_fidelity_reports import build_reports


def test_cross_fidelity_reports_cover_all_target_families(tmp_path: Path) -> None:
    manifest = build_reports(output=tmp_path)

    assert manifest["status"] == "pass"
    assert manifest["family_count"] == 9
    assert {item["family_id"] for item in manifest["reports"]} == {
        "a320",
        "b747",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "skywalker_x8",
        "tumbling_body",
        "x15",
    }
    ####
