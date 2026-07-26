"""Regression checks for the DAVE-ML reduced-order tuning artifact."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_tuning_evidence_is_controllable_bounded_and_explicitly_reduced() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_tuning_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["controllable"] is True
    assert report["hurwitz"] is True
    assert report["excluded_states"]["u_m_s"]
    assert len(report["gain"]) == 3
    assert all(len(row) == 5 for row in report["gain"])
    assert not report["bounded_probe"]["saturated"]
    assert all(len(value) == 64 for value in report["provenance"].values())

