from __future__ import annotations

import json
import re
import zipfile
from pathlib import Path

import pytest

from taoryx.trajectory import load_family_catalog

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "alpha2_family_catalog.yaml"
T7 = ROOT / "artifacts" / "verification" / "alpha2" / "t7_release"
####


def _release_artifact(name: str) -> Path:
    path = T7 / name
    if not path.is_file():
        pytest.skip("Alpha 2 T7 artifacts have not been generated; run tools/dev.py alpha2-release")
    return path
    ####


def test_alpha2_t7_freezes_catalog_families_and_fidelity_profiles() -> None:
    """The public snapshot is derived from the checked-in catalog."""

    snapshot = json.loads(_release_artifact("schema-reference.json").read_text(encoding="utf-8"))
    catalog = load_family_catalog(CATALOG)
    assert snapshot["freeze"]["status"] == "pass"
    assert snapshot["catalog_sha256"]
    assert [family["family_id"] for family in snapshot["families"]] == [family.family_id for family in catalog.families]
    assert snapshot["fidelity_profiles"] == ["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]
    ####


def test_alpha2_t7_replay_and_claim_matrix_are_explicit() -> None:
    """T7 records replay success and keeps unsupported claims excluded."""

    replay = json.loads(_release_artifact("reproducibility-report.json").read_text(encoding="utf-8"))
    claims = json.loads(_release_artifact("claim-matrix.json").read_text(encoding="utf-8"))
    assert replay["status"] == "pass"
    assert replay["mode"] == "clean_source_snapshot"
    assert replay["mismatches"] == []
    assert any(row["id"] == "historical_taos_runtime_compatibility" and row["status"] == "excluded" for row in claims["rows"])
    ####


def test_alpha2_t7_packet_contains_hashed_release_inputs() -> None:
    """The release ZIP is self-contained and path-sanitized."""

    packet = _release_artifact("evidence-packet.zip")
    with zipfile.ZipFile(packet) as archive:
        names = set(archive.namelist())
        prefix = "taoryx-alpha2-release-v1/"
        assert prefix + "bundle-manifest.json" in names
        assert prefix + "release/schema-reference.pdf" in names
        assert prefix + "evidence/alpha2/t6_dual_launch_glider/dual-launch-glider.png" in names
        assert all(not re.search(r"/Users/[A-Za-z0-9_.-]+/", archive.read(name).decode("utf-8")) for name in names if name.endswith((".json", ".yaml", ".md", ".prb", ".py", ".toml", ".txt")))
    ####
