from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_imu_profile_catalog_pins_notional_upstream_corpus() -> None:
    catalog = json.loads((ROOT / "resources" / "sensors" / "imu_profiles" / "catalog.json").read_text(encoding="utf-8"))

    assert catalog["source_repository"] == "https://github.com/sheepfling/imu-error-model"
    assert len(catalog["source_commit"]) == 40
    assert all(character in "0123456789abcdef" for character in catalog["source_commit"])
    assert catalog["authority"] == "notional_example_only"
    assert len(catalog["profiles"]) == len(set(catalog["profiles"])) == 14
    assert "hg1700ag58" in catalog["profiles"]
