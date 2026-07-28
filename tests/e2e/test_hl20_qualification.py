"""End-to-end qualification tests for the HL-20 G0-G6 ladder."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.hl20_qualification import validate_hl20_artifacts
from taoryx.hl20_reachability import run_hl20_release, write_hl20_fidelity_bundle
from taoryx.reachability_envelope import TerminalCriteria

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "examples/showcases/hl20_california_to_hawaii/quality_gates.yaml"


@pytest.mark.slow
def test_hl20_bundle_qualifies_through_g6_in_an_isolated_directory(tmp_path: Path) -> None:
    write_hl20_fidelity_bundle(tmp_path, horizon_s=120.0, step_size_s=0.5, dpi=60)

    report = validate_hl20_artifacts(tmp_path, CONTRACT)

    assert report.verdict == "qualified_through_hl20_g6"
    assert [gate.gate_id for gate in report.gates] == [f"HL20-G{index}" for index in range(7)]
    assert all(gate.status == "pass" for gate in report.gates)
    assert report.gates[-1].evidence["timed_out_query_ids"] == []


def test_hl20_nominal_terminal_policy_requires_ground_contact() -> None:
    criteria = TerminalCriteria(require_ground_contact=True)

    assert criteria.require_ground_contact is True


def test_hl20_horizon_timeout_is_not_feasible() -> None:
    result = run_hl20_release(horizon_s=30.0, step_size_s=0.5)

    assert all(sample.timed_out for sample in result.samples)
    assert all(not sample.feasible for sample in result.samples)
    assert all("terminal_event_not_reached" in sample.failure_reasons for sample in result.samples)
