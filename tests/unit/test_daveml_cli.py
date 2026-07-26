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


def test_daveml_cli_smoke_verifies_nesc_family(capsys) -> None:
    assert main(["daveml", "smoke", "--family", "reference_nesc_two_stage_rocket"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "verified"
    assert "aerodynamics" in payload["roles_hash_verified"]


def test_daveml_cli_smoke_verifies_a320_derived_exact_family(tmp_path: Path, capsys) -> None:
    output = tmp_path / "a320-smoke.json"
    assert main(["daveml", "smoke", "--family", "a320_openap_3dof", "--output", str(output)]) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "verified"
    assert payload["qualification_class"] == "derived_exact"
    assert payload["trim"]["success"]
    assert payload["objectives"]["status"] == "pass"
    assert json.loads(capsys.readouterr().out)["family_id"] == "a320_openap_3dof"


def test_daveml_cli_composite_smoke_preserves_surrogate_boundary(tmp_path: Path, capsys) -> None:
    output = tmp_path / "a320-pseudo-smoke.json"
    assert main(["daveml", "composite-smoke", "--family", "a320_openap_jsbsim_pseudo6dof", "--output", str(output)]) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "verified"
    assert payload["qualification_class"] == "surrogate_composite"
    assert "not full 6-DOF qualification" in payload["claim_boundary"]
    assert payload["roundtrip"]["status"] == "verified"
    assert json.loads(capsys.readouterr().out)["family_id"] == "a320_openap_jsbsim_pseudo6dof"
