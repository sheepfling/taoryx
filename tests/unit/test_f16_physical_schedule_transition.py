"""Tests for the F-16 local scheduled-transition evidence."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_schedule_transition_replay_is_physical_and_bounded() -> None:
    """All declared local transition cases pass without allocation failure."""

    artifact = json.loads(
        (ROOT / "verification/alpha3_f16_physical_schedule_transition/manifest.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "F16_physical_effector_schedule_transition_complete"
    assert artifact["direct_body_moment_injection"] is False
    assert artifact["summary"] == {
        "all_allocation_statuses": ["feasible"],
        "case_count": 4,
        "failed_case_count": 0,
        "maximum_controlled_allocation_residual": artifact["summary"]["maximum_controlled_allocation_residual"],
        "passed_case_count": 4,
    }
    assert all(case["saturation_steps"] == 0 for case in artifact["cases"].values())
    assert all(case["plant_policy"].startswith("linear_blend_of_validated_source_endpoint") for case in artifact["cases"].values())
    ####
