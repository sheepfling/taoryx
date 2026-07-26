"""Regression checks for the fail-closed DAVE-ML runtime CLI path."""

import json
from pathlib import Path

from taoryx.runtime.cli import main


def test_daveml_cli_smoke_verifies_f16_family(tmp_path: Path, capsys) -> None:
    output = tmp_path / "smoke.json"
    assert main(["daveml", "smoke", "--family", "reference_f16_s119", "--output", str(output)]) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "verified"
    assert "propulsion" in payload["roles_hash_verified"]
    assert json.loads(capsys.readouterr().out)["family_id"] == "reference_f16_s119"


def test_daveml_cli_smoke_verifies_hl20_family(capsys) -> None:
    assert main(["daveml", "smoke", "--family", "reference_hl20_mod_k"]) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "verified"
