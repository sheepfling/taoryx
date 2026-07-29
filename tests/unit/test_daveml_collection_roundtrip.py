"""Regression checks for the fresh-process collection round trip."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_collection_roundtrip_evidence_is_fresh_process_verified() -> None:
    report = json.loads((ROOT / "verification/daveml_collection_roundtrip_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["subprocess_returncode"] == 0
    assert report["levels"]["L2"] == "verified_canonical_structure"
    assert report["levels"]["L3"] == "verified_canonical_numeric_and_checkdata"
