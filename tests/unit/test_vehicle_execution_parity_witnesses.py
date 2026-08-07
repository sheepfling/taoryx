"""Regression coverage for the catalog-level registered parity witness gate."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.batch_episode_parity_dispatch import verify_serialized_declared_batch_episode_parity
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
