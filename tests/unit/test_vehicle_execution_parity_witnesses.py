"""Regression coverage for the catalog-level registered parity witness gate."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.batch_episode_parity_dispatch import verify_serialized_declared_batch_episode_parity
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.composition_policy import PolicyDecision, run_composition_policy
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_execution_parity_witnesses import validate_vehicle_execution_parity_witnesses

ROOT = Path(__file__).resolve().parents[2]


def test_every_registered_batch_episode_pair_has_a_passing_replay_witness() -> None:
    report = validate_vehicle_execution_parity_witnesses()

    assert report["status"] == "pass"
    assert report["registered_binding_count"] == 11
    assert report["witness_count"] == 11
    records = report["records"]
    assert isinstance(records, list)
    assert len(records) == 11
    assert all(item["status"] == "pass" for item in records)
    for item in records:
        replay = item["replay_report"]
        assert replay["status"] == "pass"
        parity = replay["batch_episode_parity"]
        assert parity["availability"] == "registered"
        assert parity["report"]["status"] == "pass"
    ####


def test_declared_parity_dispatch_refuses_a_pair_without_registered_evidence() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml")
    )

    with pytest.raises(ValueError, match="no declared batch/episode parity adapter"):
        verify_serialized_declared_batch_episode_parity(composition, {})
    ####


@pytest.mark.parametrize(
    "composition_name",
    (
        "x15_local_direct_wrench_screen_compose.yaml",
        "hl20_local_direct_wrench_screen_compose.yaml",
    ),
)
def test_local_direct_wrench_parity_replays_the_advertised_mass_resource(composition_name: str) -> None:
    """The independent batch replay must retain every configured resource channel."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / composition_name)
    )
    episode = open_vehicle_composition_episode(composition)
    issued = False

    def policy(_observation: object, _contract: object) -> PolicyDecision | None:
        nonlocal issued
        if issued:
            return None
        issued = True
        return PolicyDecision(
            {
                "wrench.force.command": [1000.0, 0.0, 0.0],
                "wrench.moment.command": [0.0, 50.0, 0.0],
            },
            0.004,
        )
        ####

    try:
        trace = run_composition_policy(episode, policy, authority_profile_id="direct_wrench")
    finally:
        episode.close()
    payload = trace.as_dict()
    status_values = payload["steps"][0]["status_frame"]["values"]

    assert isinstance(status_values["resources.mass.total"], float)
    assert status_values["resources.mass.total"] > 0.0
    assert verify_serialized_declared_batch_episode_parity(composition, payload).status == "pass"
    ####
