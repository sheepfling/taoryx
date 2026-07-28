from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/validate_hummingbird_physical_lqr.py"

pytestmark = [pytest.mark.slow, pytest.mark.dof6, pytest.mark.hummingbird]


def test_hummingbird_individual_rotor_physical_lqr_closes_the_local_chain(tmp_path: Path) -> None:
    """The Hummingbird uses all four source motors and declared motor lag."""

    output = tmp_path / "hummingbird_individual_rotor_physical_lqr.json"
    environment = dict(os.environ)
    source = str(ROOT / "src")
    environment["PYTHONPATH"] = source if not environment.get("PYTHONPATH") else source + os.pathsep + environment["PYTHONPATH"]
    completed = subprocess.run(
        [sys.executable, str(TOOL), "--output", str(output)],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    payload = json.loads(output.read_text(encoding="utf-8"))

    assert payload["schema"] == "taoryx.hummingbird-individual-rotor-physical-lqr/v1alpha1"
    assert payload["claim"]["status"] == "local_nonlinear_individual_rotor_validation"
    assert payload["claim"]["earned_controller_evidence_tier"] == "T5_nonlinearly_validated"
    assert payload["claim"]["direct_body_moment_injection"] is False
    assert payload["operating_condition"]["force_model"] == "individual-rotor-source-equations"
    assert payload["linearization"]["provenance"]["derivative_consistent"]
    assert payload["trim"]["success"]
    assert tuple(payload["effectiveness"]["effector_names"]) == tuple(f"rotor-{index}-speed" for index in range(1, 5))
    assert payload["nonlinear_validation"]["metrics"]["final_feedback_error_norm"] < (
        payload["nonlinear_validation"]["metrics"]["initial_feedback_error_norm"] * 0.05
    )
    assert payload["nonlinear_validation"]["metrics"]["final_controlled_actual_residual"] <= 0.002
    assert payload["nonlinear_validation"]["metrics"]["allocation_statuses"] == ["feasible"]
    assert any(sample["lag_active"] for sample in payload["nonlinear_validation"]["samples"])
    assert all(
        0.0 <= speed <= 1500.0
        for sample in payload["nonlinear_validation"]["samples"]
        for speed in sample["actual_effectors"].values()
    )
    ####
