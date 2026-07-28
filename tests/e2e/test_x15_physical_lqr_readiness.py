from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/validate_x15_physical_lqr.py"

pytestmark = [pytest.mark.slow, pytest.mark.x15, pytest.mark.dof6]


def test_x15_physical_lqr_readiness_fails_closed_before_trim(tmp_path: Path) -> None:
    """The X-15 cannot hide a missing trim behind a direct-wrench controller."""

    output = tmp_path / "x15_physical_lqr_readiness.json"
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

    assert payload["schema"] == "taoryx.x15-physical-lqr-readiness/v1alpha1"
    assert payload["status"] == "blocked_before_T1_trim"
    assert payload["claim"]["earned_controller_evidence_tier"] == "T0_structural"
    assert payload["claim"]["direct_body_moment_injection"] is False
    assert payload["claim"]["physical_lqr_synthesis_attempted"] is False
    assert payload["claim"]["constrained_allocator_attempted"] is False
    assert payload["claim"]["nonlinear_controller_validation_attempted"] is False
    assert payload["evidence_tiers"]["T1_trimmed"] == "blocked"
    assert payload["trim_attempts"]["unpowered_release_glide"]["status"] == "blocked"
    assert payload["trim_attempts"]["frozen_full_thrust_t0"]["status"] == "blocked"
    blocker_ids = set(payload["promotion_gate"]["blockers"])
    assert {
        "no_source_bounded_x15_trim_operating_point",
        "xlr99_deck_is_time_indexed_full_thrust_not_a_throttle_map",
        "rcs_geometry_and_impulse_unavailable",
    }.issubset(blocker_ids)
    assert payload["source_control_contract"]["propulsion"]["not_a_throttle_map"] is True
    ####
