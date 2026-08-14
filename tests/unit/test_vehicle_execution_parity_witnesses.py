"""Regression coverage for the catalog-level registered parity witness gate."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.batch_episode_parity_dispatch import verify_serialized_declared_batch_episode_parity
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.composition_policy import PolicyDecision, run_composition_policy
from taoryx.plugins import discover_plugins
from taoryx.vehicle_catalog_resources import vehicle_catalog_resources
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_execution_parity_witnesses import (
    load_vehicle_execution_parity_witness_catalog,
    validate_vehicle_execution_parity_witnesses,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    ("plugin_id", "expected_witness_ids"),
    (
        ("taoryx.a320", ("a320-3dof-kinematic-guidance", "a320-pseudo6dof-kinematic-guidance")),
        ("taoryx.f16", ("f16-3dof-kinematic-guidance", "f16-pseudo6dof-kinematic-guidance")),
        ("taoryx.hummingbird", ("hummingbird-pseudo6dof-body-motion",)),
        (
            "taoryx.source-table-fixed-wing",
            (
                "x8-3dof-native-bridge",
                "x8-pseudo6dof-native-bridge",
                "b747-3dof-native-bridge",
                "b747-pseudo6dof-native-bridge",
            ),
        ),
        ("taoryx.x15", ("x15-local-direct-wrench",)),
        ("taoryx.hl20", ("hl20-local-direct-wrench",)),
        ("taoryx.nesc", ()),
        ("taoryx.passive-bodies", ()),
    ),
)
def test_selected_plugin_parity_witness_resources_never_fall_back_to_the_aggregate(
    plugin_id: str,
    expected_witness_ids: tuple[str, ...],
) -> None:
    """Check data ownership only; do not construct or replay a vehicle."""

    plugins = discover_plugins(include_external=False, selected=(plugin_id,))
    resources = vehicle_catalog_resources(
        "verification/vehicle_execution_parity_witnesses.yaml",
        plugins=plugins,
        required=False,
    )
    witnesses = load_vehicle_execution_parity_witness_catalog(plugins=plugins)

    assert tuple(item.id for item in witnesses.witnesses) == expected_witness_ids
    if not expected_witness_ids:
        assert resources == ()
        return
    fragment = plugins.records("vehicle_catalog_fragment")
    assert len(fragment) == 1
    assert len(resources) == 1
    assert fragment[0].value.resource_package in resources[0].parts
    ####


@pytest.mark.parametrize(
    ("plugin_id", "family_id"),
    (
        ("taoryx.nesc", "reference_nesc_two_stage_rocket"),
        ("taoryx.passive-bodies", "tumbling_body"),
    ),
)
def test_selected_plugin_without_a_registered_pair_reports_empty_parity_evidence(
    plugin_id: str,
    family_id: str,
) -> None:
    """No-parity families do not need a placeholder or an aggregate trace."""

    report = validate_vehicle_execution_parity_witnesses(
        family_ids=(family_id,),
        plugins=discover_plugins(include_external=False, selected=(plugin_id,)),
    )

    assert report["status"] == "pass"
    assert report["registered_binding_count"] == 0
    assert report["witness_count"] == 0
    ####


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


def test_parity_witness_gate_can_execute_only_one_vehicle_plugin() -> None:
    report = validate_vehicle_execution_parity_witnesses(family_ids=("hummingbird",))

    assert report["status"] == "pass"
    assert report["family_filter"] == ["hummingbird"]
    assert report["registered_binding_count"] == 1
    assert report["witness_count"] == 1
    records = report["records"]
    assert isinstance(records, list)
    assert [item["family_id"] for item in records] == ["hummingbird"]
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
