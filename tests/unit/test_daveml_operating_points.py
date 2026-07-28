"""Regression checks for the DaveML family operating-point catalog."""

import json
from pathlib import Path

import pytest

from tools.validate_daveml_operating_points import validate

pytestmark = pytest.mark.daveml
ROOT = Path(__file__).resolve().parents[2]


def test_operating_point_catalog_covers_all_contract_families() -> None:
    report = validate()
    assert report["status"] == "verified"
    assert report["family_count"] == 5
    assert sum(item["applicability"] == "required" for item in report["families"]) == 4
    nesc = next(item for item in report["families"] if item["family_id"] == "reference_nesc_two_stage_rocket")
    assert nesc["applicability"] == "not_applicable"
    assert "open-loop" in nesc["non_applicable_reason"]


def test_generated_operating_point_report_is_current() -> None:
    assert json.loads((ROOT / "verification/daveml_operating_point_catalog.json").read_text(encoding="utf-8")) == validate()
