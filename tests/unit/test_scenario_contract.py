from __future__ import annotations

from pathlib import Path

from taoryx.scenario_contract import ScenarioContract, compare_contracts
from taoryx.validation import DeclaredDiscontinuity, continuity_audit

HASH = "a" * 64


def _contract(tier: str, *, duration_s: float = 10.0) -> ScenarioContract:
    return ScenarioContract(
        scenario_id="demo-parity-v1",
        vehicle="demo",
        family="point-mass",
        dynamics_tier=tier,
        initial_state_sha256=HASH,
        environment_sha256=HASH,
        vehicle_model_sha256=HASH,
        propulsion_model_sha256=HASH,
        mass_model_sha256=HASH,
        command_history_sha256=HASH,
        event_schedule_sha256=HASH,
        termination_policy_sha256=HASH,
        duration_s=duration_s,
        integrator="rk4",
        output_rate_hz=10.0,
    )


def test_only_fidelity_tier_may_differ_for_parity(tmp_path: Path) -> None:
    report = compare_contracts(_contract("3dof"), _contract("pseudo_6dof"))
    assert report["comparable"] is True
    assert report["mismatches"] == {}
    assert report["parity_sha256"]
    path = _contract("3dof").write_json(tmp_path / "contract.json")
    assert ScenarioContract.read_json(path) == _contract("3dof")
    ####


def test_contract_mismatch_is_explicit() -> None:
    report = compare_contracts(_contract("3dof"), _contract("6dof", duration_s=11.0))
    assert report["comparable"] is False
    assert "duration_s" in report["mismatches"]
    assert report["parity_sha256"] is None
    ####


def test_continuity_allows_only_declared_event_channels() -> None:
    samples = (
        {"time_s": 0.0, "position_m": 0.0, "mass_kg": 10.0},
        {"time_s": 1.0, "position_m": 1.0, "mass_kg": 9.0},
    )
    permitted = continuity_audit(
        samples,
        {"position_m": 2.0, "mass_kg": 0.01},
        declared_events=(DeclaredDiscontinuity("stage-separation", 1.0, ("mass_kg",), "jettison"),),
    )
    assert permitted["passed"] is True
    assert len(permitted["declared_event_jumps"]) == 1
    rejected = continuity_audit(
        samples,
        {"position_m": 0.001, "mass_kg": 9.0},
        declared_events=(DeclaredDiscontinuity("stage-separation", 1.0, ("mass_kg",), "jettison"),),
    )
    assert rejected["passed"] is False
    assert rejected["unexplained_violations"]
    ####
