from __future__ import annotations

import json
from pathlib import Path

from tools.validate_alpha3_showcase_catalog import build_catalog


def test_alpha3_showcase_catalog_binds_all_families_to_evidence_and_boards(tmp_path: Path) -> None:
    report = build_catalog(tmp_path / "catalog")

    assert report["status"] == "development_catalog_verified"
    assert report["family_count"] == 9
    assert report["board_count"] == 9
    assert report["paired_fidelity_count"] == 9
    assert report["cross_fidelity_pass_count"] == 9
    assert report["failures"] == []
    persisted = json.loads((tmp_path / "catalog" / "manifest.json").read_text(encoding="utf-8"))
    assert persisted["catalog"]["recipe_count"] == 10
    hummingbird = next(item for item in persisted["families"] if item["family_id"] == "hummingbird")
    assert hummingbird["board"]["path"] == "verification/alpha3_hummingbird_directional/directional_translation_evidence_board.png"
    assert hummingbird["auxiliary_evidence"]["path"] == "verification/alpha3_hummingbird_directional/manifest.json"
    assert hummingbird["supplemental_evidence"][0]["path"] == "verification/alpha3_hummingbird_native_horizontal/manifest.json"
    assert hummingbird["supplemental_evidence"][1]["path"] == "verification/alpha3_hummingbird_native_vertical/manifest.json"
    assert {item["family_id"] for item in persisted["families"]} == {
        "skywalker_x8",
        "b747",
        "a320",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
    }
    ####
