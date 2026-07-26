"""Regression checks for the generated atmosphere binding evidence."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_daveml_atmosphere_evidence_is_hash_linked_and_sampled() -> None:
    report = json.loads((ROOT / "verification/daveml_atmosphere_binding.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["model_id"] == "daveml-atmosphere-1976"
    assert len(report["source"]["sha256"]) == 64
    assert len(report["samples"]) == 3
    assert report["samples"][0]["density_kg_m3"] > report["samples"][-1]["density_kg_m3"]
    assert report["samples"][0]["speed_of_sound_m_s"] > 300.0
