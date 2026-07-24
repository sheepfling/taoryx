from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("audit_alpha1_release", ROOT / "tools/audit_alpha1_release.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_alpha1_audit_reports_all_release_gates() -> None:
    result = MODULE.audit(ROOT)
    gates = {item["id"]: item for item in result["gates"]}

    assert tuple(gates) == tuple(f"R{index}" for index in range(10))
    assert gates["R0"]["status"] == "pass"
    assert gates["R1"]["status"] in {"partial", "pass"}
    assert gates["R7"]["status"] == "pass"
    assert any(item.startswith("x15:") for item in result["vehicle_evidence_deferred_work"])
    assert gates["R8"]["status"] == "pass"
    assert result["lowest_unmet_gate"] in {None, *gates}
    assert result["status"] in {"pass", "partial", "blocked", "not_run"}
####


def test_alpha1_audit_preserves_claim_boundary() -> None:
    result = MODULE.audit(ROOT)

    assert "historical TAOS 96.0 compatibility" in result["claim_boundary"]
    assert "flight qualification" in result["claim_boundary"]
####
