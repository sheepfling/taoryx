from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/validate_b747_physical_lqr.py"

pytestmark = [pytest.mark.slow, pytest.mark.b747, pytest.mark.dof6]


def test_b747_condition3_physical_surface_lqr_closes_the_local_chain(tmp_path: Path) -> None:
    """The B747 proof trims and recovers through source-table surfaces only."""

    output = tmp_path / "b747_condition3_physical_surface_lqr.json"
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

    assert payload["schema"] == "taoryx.b747-condition3-physical-surface-lqr/v1alpha1"
    assert payload["claim"]["status"] == "local_nonlinear_surface_validation"
    assert payload["claim"]["earned_controller_evidence_tier"] == "T5_nonlinearly_validated"
    assert payload["claim"]["direct_body_moment_injection"] is False
    assert payload["linearization"]["provenance"]["derivative_consistent"]
    assert payload["trim"]["success"]
    assert payload["authority_preflight"]["status"] == "passed"
    assert payload["authority_preflight"]["metrics"]["controllability_rank"] == 9.0
    assert payload["authority_preflight"]["metrics"]["required_state_count"] == 9.0
    assert payload["authority_preflight"]["blockers"] == []
    assert tuple(payload["linearization"]["control_names"]) == (
        "elevator-deg",
        "aileron-deg",
        "rudder-deg",
        "throttle",
    )
    assert tuple(payload["effectiveness"]["effector_names"]) == (
        "elevator-deg",
        "aileron-deg",
        "rudder-deg",
        "throttle",
    )

    acceptance = payload["acceptance"]
    metrics = payload["nonlinear_validation"]["metrics"]
    final_error_fraction = metrics["final_normalized_feedback_error_norm"] / max(
        metrics["initial_normalized_feedback_error_norm"],
        1.0e-12,
    )
    assert final_error_fraction <= acceptance["final_normalized_feedback_error_fraction_of_initial"]
    assert metrics["final_controlled_actual_residual"] <= acceptance["final_controlled_actual_residual_nm"]
    assert metrics["saturation_fraction"] <= acceptance["maximum_saturation_fraction"]
    assert metrics["maximum_continuous_saturation_duration_s"] <= acceptance["maximum_continuous_saturation_duration_s"]
    assert not set(metrics["allocation_statuses"]) & set(acceptance["disallowed_allocation_statuses"])
    assert "partially_achievable" in metrics["allocation_statuses"]

    for name, contract in payload["actuator_contract"].items():
        assert contract["time_constant_s"] == pytest.approx(0.0), name
        assert contract["rate_limit_per_s"] is None, name
    for sample in payload["nonlinear_validation"]["samples"]:
        for name, contract in payload["actuator_contract"].items():
            value = sample["actual_effectors"][name]
            assert contract["lower"] <= value <= contract["upper"], (name, value, contract)

    trials = payload["tuning_trials"]
    assert len(trials) >= 3
    selected_id = payload["selected_design"]["id"]
    assert any(trial["design"]["id"] == selected_id and trial["passed"] for trial in trials)
    ####
