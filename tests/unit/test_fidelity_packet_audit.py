from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path

import pytest

from tools.audit_fidelity_packet import audit


def _packet(tmp_path: Path, *, corrupt: bool = False) -> Path:
    source = b"evidence"
    digest = hashlib.sha256(source).hexdigest()
    if corrupt:
        digest = "0" * 64
    manifest = {"families": [{"id": "demo", "long_validation": {"objective_evaluation": {
        "status": "pass", "score": 100.0, "objectives": [{
            "id": "duration", "status": "pass", "severity": "required", "weight": 1.0,
            "normalized_error": 0.0,
        }],
    }}}], "tiers": ["3dof"], "files": {"evidence.txt": digest}}
    archive = tmp_path / "packet.zip"
    with zipfile.ZipFile(archive, "w") as handle:
        handle.writestr("evidence.txt", source)
        handle.writestr("manifest.json", json.dumps(manifest))
    return archive


def test_packet_audit_checks_manifest_hashes(tmp_path: Path) -> None:
    report = audit(_packet(tmp_path))
    assert report["status"] == "pass"
    assert report["hashes_checked"] == 1


def test_packet_audit_rejects_stale_hash(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="hash audit failed"):
        audit(_packet(tmp_path, corrupt=True))


def test_objective_composite_recomputation_rejects_tampering(tmp_path: Path) -> None:
    source = b"evidence"
    objective = {
        "id": "duration",
        "status": "pass",
        "severity": "required",
        "weight": 1.0,
        "normalized_error": 0.25,
    }
    manifest = {
        "families": [{"id": "demo", "long_validation": {"objective_evaluation": {
            "status": "pass", "score": 75.0, "objectives": [objective]
        }}}],
        "tiers": ["3dof"],
        "files": {"evidence.txt": hashlib.sha256(source).hexdigest()},
    }
    archive = tmp_path / "scored.zip"
    with zipfile.ZipFile(archive, "w") as handle:
        handle.writestr("evidence.txt", source)
        handle.writestr("manifest.json", json.dumps(manifest))
    report = audit(archive)
    assert report["objective_composites"]["demo"]["score"] == 75.0
    manifest["families"][0]["long_validation"]["objective_evaluation"]["score"] = 74.0
    with zipfile.ZipFile(archive, "w") as handle:
        handle.writestr("evidence.txt", source)
        handle.writestr("manifest.json", json.dumps(manifest))
    with pytest.raises(ValueError, match="objective score is not reproducible"):
        audit(archive)
