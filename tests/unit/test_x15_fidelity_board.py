from __future__ import annotations

import json
from pathlib import Path

from tools.render_x15_fidelity_board import render


def test_x15_fidelity_board_reproduces_current_paired_evidence(tmp_path: Path) -> None:
    output = tmp_path / "x15-board"
    manifest = render(output)

    assert manifest["status"] == "verified"
    assert manifest["family_id"] == "x15"
    assert manifest["claim_boundary"]["proves"]
    assert len(manifest["fidelity_records"]) == 2
    assert all(record["mission_pass"] is True for record in manifest["fidelity_records"])

    board = output / "x15_fidelity_evidence_board.png"
    manifest_path = output / "x15_fidelity_evidence_board_manifest.json"
    assert board.is_file()
    assert manifest_path.is_file()
    persisted = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert persisted["board"]["sha256"] == manifest["board"]["sha256"]
    assert "controlled terminal" in " ".join(manifest["claim_boundary"]["nonclaims"])
    ####
