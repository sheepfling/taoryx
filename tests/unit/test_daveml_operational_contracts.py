"""Regression checks for the shared DaveML operational contract registry."""

import json
from pathlib import Path

import pytest

from tools.validate_daveml_operational_contracts import validate

pytestmark = pytest.mark.daveml
ROOT = Path(__file__).resolve().parents[2]


def test_all_promoted_daveml_families_have_verified_operational_contracts() -> None:
    report = validate()
    assert report["status"] == "verified"
    assert report["family_count"] == 5
    assert {item["qualification_class"] for item in report["families"]} == {
        "reference_exact",
        "derived_exact",
        "surrogate_composite",
    }
    assert all(item["status"] == "verified" for item in report["families"])


@pytest.mark.artifact
def test_generated_contract_report_is_current() -> None:
    report = validate()
    checked = json.loads((ROOT / "verification/daveml_operational_contracts.json").read_text(encoding="utf-8"))
    assert checked == report
