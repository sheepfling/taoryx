from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from taoryx.showcase import ShowcaseArchetypeCatalog
from tools.build_daveml_showcase_composites import build

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.daveml


def test_daveml_showcase_catalog_uses_common_archetypes() -> None:
    payload = yaml.safe_load((ROOT / "verification/daveml_showcase_catalog.yaml").read_text(encoding="utf-8"))
    catalog_payload = yaml.safe_load((ROOT / "verification/showcase_archetype_catalog.yaml").read_text(encoding="utf-8"))
    catalog = ShowcaseArchetypeCatalog(**catalog_payload)

    assert len(payload["boards"]) == 5
    assert {recipe.id for recipe in catalog.recipes} >= {board["id"] for board in payload["boards"]}


def test_daveml_showcase_builder_emits_all_five_evidence_boards(tmp_path: Path) -> None:
    report = build(ROOT / "verification/daveml_showcase_catalog.yaml", tmp_path)

    assert report["status"] == "verified"
    assert len(report["boards"]) == 5
    for item in report["boards"]:
        board = tmp_path / item["board_id"]
        manifest = json.loads((board / "manifest.json").read_text(encoding="utf-8"))
        assert manifest["artifact_contract_hash"] == item["artifact_contract_hash"]
        assert (board / "evidence-board.png").is_file()
        assert json.loads((board / "evidence-summary.json").read_text(encoding="utf-8"))["evidence"]
        for run in manifest["run_artifacts"]:
            assert run["realization"]["fidelity"] == run["fidelity"]
            if run["control_realization"] == "direct_wrench":
                assert run["realization"]["physical_effectors"] == []


def test_daveml_showcase_preserves_a320_fidelity_split_and_synthetic_child(tmp_path: Path) -> None:
    build(ROOT / "verification/daveml_showcase_catalog.yaml", tmp_path)

    comparison = json.loads((tmp_path / "a320-derived-versus-surrogate-v1" / "manifest.json").read_text(encoding="utf-8"))
    assert comparison["fidelity_profiles"] == ["point_mass_3dof", "pseudo_6dof"]
    assert {run["fidelity"] for run in comparison["run_artifacts"]} == {"point_mass_3dof", "pseudo_6dof"}
    deployment = json.loads((tmp_path / "nesc-synthetic-passive-child-deployment-v1" / "manifest.json").read_text(encoding="utf-8"))
    assert deployment["object_lineage"]
    telemetry = json.loads((tmp_path / "nesc-synthetic-passive-child-deployment-v1" / "telemetry.json").read_text(encoding="utf-8"))
    assert telemetry["metadata"]["synthetic_child"]["shape"] == "cylinder"
    assert "synthetic" in deployment["claim"].lower()
