"""Tests for the HL-20 source-load direct-wrench screen."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_hl20_direct_wrench_screen_is_explicitly_non_surface_evidence() -> None:
    artifact = json.loads((ROOT / "verification/alpha3_hl20_direct_wrench/manifest.json").read_text(encoding="utf-8"))
    assert artifact["status"] == "nominal_case_pass"
    assert artifact["fidelity"] == "rigid_body_6dof_direct_wrench"
    assert artifact["claim"]["evidence_tier"] == "T3_direct_wrench_bridge"
    assert artifact["claim"]["direct_body_moment_injection"] is True
    assert artifact["claim"]["physical_effector_allocation"] is False
    assert artifact["evaluation"]["mission_pass"] is True
    assert artifact["evaluation"]["wrench_statuses_observed"] == ["feasible"]
    assert "load-balance bias" in artifact["control_path"]
    ####
