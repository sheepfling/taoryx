"""Regression checks for the authored A320 composite DAVE-ML collection."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.trajectory import CollectionManifest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_a320_pseudo6dof_collection_roundtrip_is_fresh_process_verified() -> None:
    report = json.loads((ROOT / "families/a320_openap_jsbsim_pseudo6dof/validation/roundtrip-report.json").read_text(encoding="utf-8"))
    manifest = CollectionManifest.model_validate(json.loads((ROOT / "families/a320_openap_jsbsim_pseudo6dof/collection-manifest.json").read_text(encoding="utf-8")))

    assert report["status"] == "verified"
    assert report["claim_boundary"].startswith("Taoryx-authored surrogate-composite")
    assert len(report["documents"]) == 5
    assert all(document["fresh_process_reimport"] == "verified" for document in report["documents"])
    assert all(document["structural_diff_count"] == 0 for document in report["documents"])
    assert all(document["numeric_diff_count"] == 0 for document in report["documents"])
    assert manifest.qualification_class == "surrogate_composite"
    assert manifest.source_exact is False
    assert manifest.roundtrip_status == "verified"
