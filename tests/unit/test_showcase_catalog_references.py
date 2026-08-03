from __future__ import annotations

import hashlib
import json
from pathlib import Path

from tools.validate_showcase_catalog_references import validate_catalog


def _canonical_run() -> dict[str, object]:
    boundary = "catalog-reference-test"
    digest = hashlib.sha256(boundary.encode("utf-8")).hexdigest()
    return {
        "schema_version": "taoryx.showcase/v1alpha1",
        "run_id": "catalog-reference-run",
        "showcase_id": "org.taoryx.showcase.test.catalog",
        "vehicle_binding_id": "synthetic.catalog-v1",
        "fidelity": "point_mass_3dof",
        "control_realization": "force_model",
        "realization": {
            "fidelity": "point_mass_3dof",
            "control_realization": "force_model",
            "realization_id": "catalog-reference-realization-v1",
            "state_schema": ["position_velocity_mass_resource"],
            "semantic_command_mapping": {"acceleration": "declared force-model command"},
            "physical_effectors": [],
            "available_physics": ["synthetic translational equations"],
            "claim": "synthetic point-mass catalog reference",
            "nonclaims": ["physical attitude"],
            "evidence_grade": "synthetic",
        },
        "scenario_contract_sha256": digest,
        "outcome": "completed",
        "claim": "synthetic point-mass catalog reference",
        "nonclaims": ["physical attitude"],
        "files": [{"path": "manifest.json", "sha256": digest, "media_type": "application/json", "required": True}],
        "board": {"profile": "test-board", "modules": ["trajectory_3d"]},
        "archetypes": ["mission_geometry"],
    }


def test_catalog_reference_validator_resolves_canonical_children(tmp_path: Path) -> None:
    packet = tmp_path / "packet"
    packet.mkdir()
    run = _canonical_run()
    (packet / "manifest.json").write_text(json.dumps({"run_artifacts": [run]}) + "\n", encoding="utf-8")
    catalog = tmp_path / "catalog.json"
    catalog.write_text(json.dumps({"families": [{"id": "test", "packet": "packet", "status": "development"}]}) + "\n", encoding="utf-8")

    report = validate_catalog(catalog)

    assert report["status"] == "pass"
    assert report["resolved_record_count"] == 1
    assert report["resolved"][0]["children"][0]["control_realization"] == "force_model"
    ####


def test_catalog_reference_validator_rejects_aggregate_only_child(tmp_path: Path) -> None:
    packet = tmp_path / "aggregate"
    packet.mkdir()
    (packet / "manifest.json").write_text(json.dumps({"cases": [], "claim_boundary": "evidence only"}) + "\n", encoding="utf-8")
    catalog = tmp_path / "catalog.json"
    catalog.write_text(json.dumps({"families": [{"id": "aggregate", "packet": "aggregate"}]}) + "\n", encoding="utf-8")

    report = validate_catalog(catalog)

    assert report["status"] == "blocked"
    assert report["findings"][0]["code"] == "child_manifest_invalid"
    ####
