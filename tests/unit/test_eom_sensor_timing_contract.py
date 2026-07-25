"""Keep sensor timing tied to accepted EOM truth boundaries."""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).parents[2]
####


def _contract() -> dict[str, object]:
    with (ROOT / "verification/eom_sensor_timing_contract.yaml").open(encoding="utf-8") as stream:
        contract = yaml.safe_load(stream)
    assert isinstance(contract, dict)
    return contract
####


def test_eom_contract_makes_only_committed_boundaries_sensor_visible() -> None:
    contract = _contract()
    phases = {phase["id"]: phase for phase in contract["phases"]}
    assert phases["committed_boundary"]["sensor_access"] == "allowed"
    assert phases["sensor_sample"]["sensor_access"] == "allowed"
    assert phases["integration_stage"]["sensor_access"] == "forbidden"
    assert phases["accepted_boundary"]["state"] == "atomically_commit_accepted_state"
    assert contract["commit_rules"]["rejected_solver_stages_are_discarded"] is True
####


def test_eom_contract_forbids_posthoc_sensor_interpolation() -> None:
    contract = _contract()
    sensors = contract["sensor_rules"]
    assert sensors["instantaneous_measurement"]["use_interpolated_runtime_state"] is False
    assert sensors["interval_measurement"]["use_posthoc_linear_state_interpolation"] is False
    assert sensors["rate_mismatch"]["sensor_faster_than_eom"].startswith("split_or_substep")
    assert "sensor_interpolates_between_published_fixed_steps" in contract["forbidden_patterns"]
    language = contract["language_extension"]
    assert language["profile"] == "taoryx"
    assert language["instantaneous"]["truth_policy"] == "boundary"
    assert language["interval"]["truth_policy"] == "accepted-segment"
    assert language["unsupported_historical_profile"] == "taos96"
####
