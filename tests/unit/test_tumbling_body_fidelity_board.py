from __future__ import annotations

import json
from pathlib import Path

from tools.render_tumbling_body_fidelity_board import render


def test_tumbling_body_fidelity_board_reproduces_passive_evidence(tmp_path: Path) -> None:
    output = tmp_path / "tumbling-board"
    manifest = render(output)

    assert manifest["status"] == "verified"
    assert manifest["family_id"] == "tumbling_body"
    assert manifest["representative_shape"] == "cylinder"
    assert len(manifest["fidelity_records"]) == 2
    assert all(record["classification"] == "impact" for record in manifest["fidelity_records"])
    assert "No controller" in manifest["claim_boundary"]["nonclaims"][0]

    board = output / "tumbling_body_fidelity_evidence_board.png"
    persisted = json.loads(
        (output / "tumbling_body_fidelity_evidence_board_manifest.json").read_text(encoding="utf-8")
    )
    assert board.is_file()
    assert persisted["board"]["sha256"] == manifest["board"]["sha256"]
    ####
