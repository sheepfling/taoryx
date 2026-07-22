"""Vehicle-independent contract fixture for new point-mass families."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.scenario_contract import ScenarioContract, compare_contracts

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification/generic_point_mass_contract.yaml"
PROBLEM = ROOT / "examples/verification/kinematic_bridge/generic_point_mass.prb"


def _digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _bridge_problem(source: Path, destination: Path) -> Path:
    lines = source.read_text(encoding="utf-8").splitlines()
    title_index = next(index for index, line in enumerate(lines) if line.startswith("*title"))
    lines[title_index + 1:title_index + 1] = [
        "*mode kinematic-6dof",
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
    ####


@pytest.mark.dof3
def test_generic_point_mass_contract_accepts_a_new_family_without_adapter(tmp_path: Path) -> None:
    """The generic fixture runs in both lower tiers with identical translation."""

    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    assert catalog["scenario_id"] == "generic-point-mass-contract-v1"
    point = run_files(PROBLEM, (), output_dir=tmp_path / "point", max_steps=200, profile=GrammarProfile.TAORYX)
    bridge_problem = _bridge_problem(PROBLEM, tmp_path / "bridge.prb")
    bridge = run_files(bridge_problem, (), output_dir=tmp_path / "bridge", max_steps=200, profile=GrammarProfile.TAORYX)
    for report in (point, bridge):
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        assert report.results and report.results[0].completed
    point_history = point.results[0].states["1"]
    bridge_history = bridge.results[0].states["1"]
    assert len(point_history) == len(bridge_history)
    for point_state, bridge_state in zip(point_history, bridge_history, strict=True):
        for channel in ("x", "y", "z", "xdt", "ydt", "zdt", "mass"):
            assert bridge_state.named[channel] == pytest.approx(point_state.named[channel], abs=1.0e-9)
    ####


@pytest.mark.dof3
def test_generic_point_mass_contract_digest_only_excludes_dynamics_tier() -> None:
    """Contract comparison accepts the bridge and refuses physical mismatches."""

    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    physical = catalog["physical_inputs"]
    common = dict(
        scenario_id=catalog["scenario_id"],
        vehicle="generic-point-mass",
        family="generic",
        initial_state_sha256=_digest(physical["initial_state"]),
        environment_sha256=_digest(physical["environment"]),
        vehicle_model_sha256=_digest({"problem": PROBLEM.read_bytes().hex()}),
        propulsion_model_sha256=_digest(None),
        mass_model_sha256=_digest(physical["initial_state"]["mass_kg"]),
        command_history_sha256=_digest(physical["command_history"]),
        event_schedule_sha256=_digest(physical["event_schedule"]),
        termination_policy_sha256=_digest(physical["termination_policy"]),
        duration_s=float(catalog["duration_s"]),
        unit_system="si",
        integrator="rk4",
        output_rate_hz=10.0,
    )
    point = ScenarioContract(dynamics_tier="3dof", **common)
    bridge = ScenarioContract(dynamics_tier="pseudo_6dof", **common)
    comparable = compare_contracts(point, bridge)
    assert comparable["comparable"] is True
    rigid = ScenarioContract(dynamics_tier="6dof", vehicle_model_sha256=_digest("different-plant"), **{key: value for key, value in common.items() if key != "vehicle_model_sha256"})
    mismatch = compare_contracts(point, rigid)
    assert mismatch["comparable"] is False
    assert "vehicle_model_sha256" in mismatch["mismatches"]
    ####
