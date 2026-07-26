"""Regression checks for the fresh-process DAVE-ML release gate."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_daveml_release_gate_is_verified_and_hashes_artifacts() -> None:
    report = json.loads((ROOT / "verification/daveml_release_gate.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert len(report["runs"]) == 19
    assert all(item["returncode"] == 0 for item in report["runs"])
    assert len(report["artifacts"]) == 17
    assert all(len(value) == 64 for value in report["artifacts"].values())
