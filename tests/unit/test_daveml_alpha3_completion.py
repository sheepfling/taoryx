from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.validate_daveml_alpha3_completion import validate

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.daveml


def test_alpha3_completion_registry_is_evidence_backed() -> None:
    report = validate(ROOT / "verification/daveml_alpha3_completion.yaml")

    assert report["status"] == "verified_with_known_gaps"
    assert report["family_count"] == 5
    assert report["pending_reduction_equivalence"]
    assert not report["failures"]


def test_alpha3_registry_has_no_temporary_provenance_paths() -> None:
    report = json.loads((ROOT / "verification/daveml_alpha3_completion.json").read_text(encoding="utf-8"))

    serialized = json.dumps(report).upper()
    assert "INBOX" not in serialized
    assert "/TMP/" not in serialized
