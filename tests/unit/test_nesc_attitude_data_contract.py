"""Regression checks for the fail-closed NESC attitude-data audit."""

from __future__ import annotations

import json
from pathlib import Path

from tools.validate_nesc_attitude_data_contract import build_contract


def test_nesc_attitude_data_contract_records_missing_parent_effectors(tmp_path: Path) -> None:
    report = build_contract(tmp_path)
    assert report["status"] == "blocked_parent_attitude_data_unavailable"
    aggregate = report["aggregate"]
    assert aggregate["control_input_count"] == 0
    assert aggregate["gimbal_candidate_count"] == 0
    assert aggregate["attitude_input_count"] == 0
    assert (tmp_path / "attitude_data_contract_board.png").is_file()
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == report["status"]
####
