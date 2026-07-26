from __future__ import annotations

import json
from pathlib import Path

from tools.validate_daveml_layer_dispositions import validate_dispositions

ROOT = Path(__file__).resolve().parents[2]


def test_daveml_layer_dispositions_are_explicit_and_evidence_backed() -> None:
    report = validate_dispositions(ROOT / "verification/daveml_family_layer_dispositions.yaml")

    assert report["status"] == "verified"
    assert report["family_count"] == 5
    assert report["layer_count"] >= 15
    statuses = {
        layer["status"]
        for family in report["families"]
        for layer in family["layers"].values()
    }
    assert "external_overlay_required" in statuses
    assert "not_applicable" in statuses
    assert "not_promoted" in statuses


def test_checked_in_layer_disposition_report_is_verified() -> None:
    report = json.loads((ROOT / "verification/daveml_family_layer_dispositions.json").read_text(encoding="utf-8"))

    assert report["status"] == "verified"
    assert report["layer_count"] >= 15
