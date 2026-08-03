from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/validate_x8_physical_lqr.py"

pytestmark = [pytest.mark.slow, pytest.mark.x8]


def test_x8_table_coordinate_physical_lqr_artifact_closes_the_local_chain(tmp_path: Path) -> None:
    """The X8 proof retains actual table-coordinate limits and nonclaims."""

    output = tmp_path / "x8_table_coordinate_physical_lqr.json"
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

    assert payload["schema"] == "taoryx.x8-table-coordinate-physical-lqr/v1alpha1"
    assert payload["claim"]["status"] == "local_nonlinear_table_coordinate_validation"
    assert payload["claim"]["earned_controller_evidence_tier"] == "T4_physically_allocated_source_coordinate"
    assert payload["claim"]["nonlinear_table_coordinate_evidence"] == "passed"
    assert payload["claim"]["direct_body_moment_injection"] is False
    assert "left/right" in " ".join(payload["claim"]["nonclaims"]).lower()
    assert payload["physical_mapping"]["status"] == "resolved_by_source_equation"
    assert len(payload["physical_mapping"]["hypotheses"]) == 2
    assert payload["physical_mapping"]["selected_hypothesis"]["differential_sign"] == 1
    assert payload["linearization"]["provenance"]["derivative_consistent"]
    assert payload["authority_preflight"]["status"] == "passed"
    assert payload["authority_preflight"]["metrics"]["controllability_rank"] == 4.0
    assert payload["authority_preflight"]["metrics"]["required_state_count"] == 4.0
    assert payload["authority_preflight"]["blockers"] == []
    assert payload["nonlinear_validation"]["metrics"]["final_feedback_error_norm"] < (
        payload["nonlinear_validation"]["metrics"]["initial_feedback_error_norm"] * 0.05
    )
    assert payload["nonlinear_validation"]["metrics"]["final_controlled_actual_residual"] <= 0.005
    assert payload["nonlinear_validation"]["metrics"]["maximum_continuous_saturation_duration_s"] <= 0.25
    assert not (
        set(payload["nonlinear_validation"]["metrics"]["allocation_statuses"])
        & set(payload["acceptance"]["disallowed_allocation_statuses"])
    )
    ####
