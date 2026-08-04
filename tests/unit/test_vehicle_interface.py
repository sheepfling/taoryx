"""Regression coverage for the resolved vehicle-interface contract."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest

from taoryx.interface_channel_value_spaces import (
    load_interface_channel_value_space_catalog,
    validate_interface_channel_value_space_coverage,
)
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_interface import (
    bind_declared_sensor_profile,
    build_vehicle_interface_catalog_report,
    interface_contract_for_composition,
    resolve_vehicle_interface_contract,
    validate_authority_action_values,
    validate_interface_channel_value,
    validate_vehicle_interface_contract,
)

ROOT = Path(__file__).resolve().parents[2]


def test_x8_pseudo_interface_exposes_a_native_bridge_without_effector_promotion() -> None:
    contract = resolve_vehicle_interface_contract("skywalker_x8", "pseudo_6dof")

    assert contract.control_realization == "response_law"
    assert contract.authority_profile("native_control_bridge").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {
        "propulsion.command.fraction",
        "control.longitudinal.bridge.command",
        "control.lateral.bridge.command",
    }
    assert contract.effector_channels == ()
    assert "physical actuator evidence" in contract.action_channels[0].claim_boundary
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_hummingbird_pseudo_interface_exposes_response_controls_and_battery_status() -> None:
    contract = resolve_vehicle_interface_contract("hummingbird", "pseudo_6dof")

    assert contract.authority_profile("body_motion_response").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {
        "attitude.roll.command",
        "attitude.pitch.command",
        "attitude.yaw.command",
        "propulsion.command.fraction",
        "propulsion.enable",
    }
    assert "resources.battery.fraction_remaining" in {channel.id for channel in contract.resource_channels}
    assert "resources.mass.total" in {channel.id for channel in contract.resource_channels}
    assert "propulsion.output.thrust.aggregate" in {channel.id for channel in contract.status_channels}
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_authority_action_validation_applies_declared_bounds_and_value_spaces_before_native_mapping() -> None:
    contract = resolve_vehicle_interface_contract("hummingbird", "pseudo_6dof")

    profile = validate_authority_action_values(
        contract,
        "body_motion_response",
        {"attitude.yaw.command": 1.5, "propulsion.command.fraction": 0.8},
    )
    assert profile.id == "body_motion_response"
    with pytest.raises(ValueError, match="exceeds its declared upper bound"):
        validate_authority_action_values(
            contract,
            "body_motion_response",
            {"propulsion.command.fraction": 1.1},
        )
    with pytest.raises(ValueError, match="outside 'body_motion_response'"):
        validate_authority_action_values(
            contract,
            "body_motion_response",
            {"wrench.moment.command": [0.0, 0.0, 0.0]},
        )
    ####


def test_interface_value_spaces_make_angle_and_attitude_topology_explicit() -> None:
    x8 = resolve_vehicle_interface_contract("skywalker_x8", "pseudo_6dof")
    x8_status = {channel.id: channel for channel in x8.status_channels}
    x8_actions = {channel.id: channel for channel in x8.action_channels}

    assert x8_status["flight.heading"].value_space is not None
    assert x8_status["flight.heading"].value_space.topology == "periodic_circle"
    assert x8_status["flight.heading"].value_space.period == pytest.approx(360.0)
    assert x8_status["flight.heading"].as_dict()["value_space_source"] == "interface_channel_value_space_catalog"
    assert x8_actions["control.lateral.bridge.command"].value_space is not None
    assert x8_actions["control.lateral.bridge.command"].value_space.topology == "bounded_interval"
    turn_direction = next(channel for channel in x8.parameter_channels if channel.id == "segment.fly_by_turn.turn_direction")
    assert turn_direction.binding["options"] == ["left", "right"]

    passive = resolve_vehicle_interface_contract("tumbling_body", "pseudo_6dof")
    quaternion = next(channel for channel in passive.status_channels if channel.id == "attitude.quaternion")
    assert quaternion.value_space is not None
    assert quaternion.value_space.topology == "rotation_group_so3"
    validate_interface_channel_value(quaternion, [1.0, 0.0, 0.0, 0.0], context="test")
    with pytest.raises(ValueError, match="unit norm"):
        validate_interface_channel_value(quaternion, [2.0, 0.0, 0.0, 0.0], context="test")
    ####


def test_every_resolved_public_interface_channel_has_an_explicit_catalog_entry() -> None:
    """Public controls/status remain explicit rather than unit- or name-derived."""

    catalog = load_interface_channel_value_space_catalog()
    resolved = load_resolved_vehicle_composition_catalog()
    channels = [
        channel
        for vehicle in resolved.vehicles
        for fidelity in vehicle.family.family.tiers
        for channel in (
            *interface_contract_for_composition(vehicle, fidelity).parameter_channels,
            *interface_contract_for_composition(vehicle, fidelity).action_channels,
            *interface_contract_for_composition(vehicle, fidelity).effector_channels,
            *interface_contract_for_composition(vehicle, fidelity).status_channels,
            *interface_contract_for_composition(vehicle, fidelity).resource_channels,
            *interface_contract_for_composition(vehicle, fidelity).diagnostic_channels,
        )
    ]

    assert {channel.id for channel in channels} <= set(catalog)
    assert all(channel.as_dict()["value_space_source"] == "interface_channel_value_space_catalog" for channel in channels)
    with pytest.raises(ValueError, match="lack explicit value-space contracts"):
        validate_interface_channel_value_space_coverage(["new.public.channel"])
    ####


def test_rocket_and_passive_contracts_do_not_fabricate_control_authority() -> None:
    rocket = resolve_vehicle_interface_contract("reference_nesc_two_stage_rocket", "pseudo_6dof")
    passive = resolve_vehicle_interface_contract("tumbling_body", "point_mass_3dof")

    assert rocket.authority_profile("no_external_action").availability == "not_available"
    assert passive.authority_profile("no_external_action").availability == "not_applicable"
    assert rocket.action_channels == ()
    assert passive.action_channels == ()
    assert validate_vehicle_interface_contract(rocket) == ()
    assert validate_vehicle_interface_contract(passive) == ()
    ####


def test_passive_pseudo_interface_reuses_rigid_truth_in_batch_without_a_response_law() -> None:
    contract = resolve_vehicle_interface_contract("tumbling_body", "pseudo_6dof")

    assert contract.control_realization == "uncontrolled"
    assert contract.authority_profile("no_external_action").availability == "not_applicable"
    assert contract.action_channels == ()
    status = {channel.id: channel for channel in contract.status_channels}
    assert status["position.local"].availability == "available_in_batch"
    assert status["attitude.quaternion"].availability == "available_in_batch"
    assert status["body_rate"].availability == "available_in_batch"
    resources = {channel.id: channel for channel in contract.resource_channels}
    assert resources["resources.mass.total"].availability == "available_in_batch"
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_x15_staged_batch_interface_publishes_resource_status_without_inventing_an_episode() -> None:
    contract = resolve_vehicle_interface_contract("x15", "pseudo_6dof")

    assert contract.authority_profile("no_external_action").availability == "not_available"
    assert contract.observation_profile("truth_debug").availability == "unavailable_at_runtime"
    resources = {channel.id: channel for channel in contract.resource_channels}
    assert resources["resources.mass.total"].availability == "available_in_batch"
    assert resources["resources.booster.attached"].availability == "available_in_batch"
    assert resources["resources.booster.propellant.consumed"].availability == "available_in_batch"
    assert "9,000 kg" in resources["resources.booster.propellant.consumed"].claim_boundary
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_x15_direct_wrench_screen_publishes_stepwise_bounded_wrench_authority() -> None:
    contract = resolve_vehicle_interface_contract("x15", "rigid_body_6dof_direct_wrench")

    assert contract.control_realization == "direct_wrench"
    assert contract.authority_profile("direct_wrench").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {"wrench.force.command", "wrench.moment.command"}
    assert all(channel.availability == "available" for channel in contract.action_channels)
    assert {channel.binding["native_action"] for channel in contract.action_channels} == {"force_body_n", "moment_body_nm"}
    status = {channel.id: channel for channel in contract.status_channels}
    assert {
        "velocity.body",
        "body_rate",
        "control.wrench.requested.force",
        "control.wrench.requested.moment",
        "control.wrench.achieved.force",
        "control.wrench.achieved.moment",
        "control.wrench.residual.force",
        "control.wrench.residual.moment",
        "control.wrench.status",
        "control.wrench.saturated",
    } <= set(status)
    assert all(channel.availability == "available" for channel in status.values())
    diagnostics = {channel.id: channel for channel in contract.diagnostic_channels}
    assert diagnostics["control.realization"].binding["batch_telemetry"] == "control_realization"
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_hl20_direct_wrench_screen_publishes_the_same_explicit_bridge_without_surface_claims() -> None:
    contract = resolve_vehicle_interface_contract("hl20_mod_k", "rigid_body_6dof_direct_wrench")

    assert contract.control_realization == "direct_wrench"
    assert contract.authority_profile("direct_wrench").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {"wrench.force.command", "wrench.moment.command"}
    assert all(channel.availability == "available" for channel in contract.action_channels)
    status = {channel.id: channel for channel in contract.status_channels}
    assert status["control.wrench.achieved.force"].availability == "available"
    assert status["control.physical_effector_allocation"].availability == "available"
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_composition_interface_does_not_borrow_the_x15_local_screen_episode_for_high_energy_mission() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_direct_wrench_compose.yaml")
    )
    contract = resolve_vehicle_composition_interface_contract(composition)

    assert contract.authority_profile("direct_wrench").availability == "unavailable_at_runtime"
    assert all(channel.availability == "unavailable_at_runtime" for channel in contract.action_channels)
    assert contract.observation_profile("truth_debug").availability == "unavailable_at_runtime"
    assert {record["mission"] for record in contract.execution_records} == {"rocket_aircraft_high_energy_v1"}
    assert validate_vehicle_interface_contract(contract) == ()
    ####


@pytest.mark.parametrize("family_id", ("a320_openap_3dof", "f16_s119"))
def test_reduced_airbreather_batch_interface_exposes_common_kinematics_without_effector_promotion(
    family_id: str,
) -> None:
    contract = resolve_vehicle_interface_contract(family_id, "pseudo_6dof")

    status = {channel.id: channel for channel in contract.status_channels}
    assert {
        "position.north",
        "position.east",
        "position.altitude",
        "velocity.speed",
        "flight.path_angle",
        "flight.heading",
        "attitude.euler",
        "body_rate",
        "aero.dynamic_pressure",
    } <= set(status)
    assert status["attitude.euler"].availability == "available"
    assert status["attitude.euler"].provenance == "engineering_surrogate"
    resources = {channel.id: channel for channel in contract.resource_channels}
    assert resources["resources.mass.total"].availability == "available"
    assert contract.effector_channels == ()
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_declared_sensor_binding_is_fingerprinted_and_rejects_unavailable_channels() -> None:
    contract = resolve_vehicle_interface_contract("skywalker_x8", "point_mass_3dof")
    sensor_contract = bind_declared_sensor_profile(
        contract,
        channel_ids=("execution.time", "position.altitude", "velocity.speed"),
        cadence_s=0.1,
        latency_s=0.02,
    )

    profile = sensor_contract.observation_profile("declared_sensor")
    assert sensor_contract.fingerprint != contract.fingerprint
    assert profile.availability == "available"
    assert profile.cadence_s == pytest.approx(0.1)
    assert profile.latency_s == pytest.approx(0.02)
    with pytest.raises(ValueError, match="unavailable channel"):
        bind_declared_sensor_profile(
            contract,
            channel_ids=("effector.left_elevon.actual",),
            cadence_s=0.1,
            latency_s=0.0,
        )
    hummingbird = resolve_vehicle_interface_contract("hummingbird", "pseudo_6dof")
    with pytest.raises(ValueError, match="requires a scalar channel"):
        bind_declared_sensor_profile(
            hummingbird,
            channel_ids=("attitude.euler",),
            cadence_s=0.1,
            latency_s=0.0,
            channel_errors={"attitude.euler": {"gaussian_stddev": 0.01}},
        )
    ####


def test_interface_fingerprint_is_deterministic_and_cli_exports_validation(
    capsys: pytest.CaptureFixture[str],
) -> None:
    first = resolve_vehicle_interface_contract("b747", "pseudo_6dof")
    second = resolve_vehicle_interface_contract("b747", "pseudo_6dof")

    assert first.fingerprint == second.fingerprint
    assert main(["vehicle", "interface", "b747", "pseudo_6dof"]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))
    assert payload["schema"] == "taoryx.vehicle-interface/v1alpha1"
    assert payload["fingerprint_sha256"] == first.fingerprint
    assert payload["validation"] == {"status": "pass", "findings": []}
    ####


def test_composition_interface_cli_resolves_the_declared_sensor_contract(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    composition = compile_vehicle_composition(request)
    path = tmp_path / "x8-composition.json"
    composition.write_json(path)

    assert main(["vehicle", "interface-composition", str(path)]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))

    profile = next(item for item in cast(list[dict[str, object]], payload["observation_profiles"]) if item["id"] == "declared_sensor")
    assert payload["composition_identity_sha256"] == composition.identity_sha256
    assert profile["availability"] == "available"
    assert profile["cadence_s"] == pytest.approx(0.1)
    assert profile["latency_s"] == pytest.approx(0.05)
    ####


def test_every_advertised_fidelity_resolves_a_fail_closed_interface_contract() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    for vehicle in catalog.vehicles:
        for fidelity in vehicle.family.family.tiers:
            contract = resolve_vehicle_interface_contract(vehicle.family.family_id, fidelity, catalog=catalog)

            assert validate_vehicle_interface_contract(contract) == ()
            available_profiles = [item for item in contract.authority_profiles if item.availability == "available"]
            if available_profiles:
                assert any(
                    record["operation"] == "episode" and record["status"] == "runnable"
                    for record in contract.execution_records
                )
    ####


def test_catalog_interface_report_matches_all_declared_episode_profiles(
    capsys: pytest.CaptureFixture[str],
) -> None:
    report = build_vehicle_interface_catalog_report()

    assert report["status"] == "pass"
    assert report["family_count"] == 9
    assert report["interface_count"] == 36
    assert report["error_count"] == 0
    interfaces = cast(list[dict[str, object]], report["interfaces"])
    runnable = [item for item in interfaces if item["episode_runnable"]]
    assert {item["family_id"] for item in runnable} == {
        "a320_openap_3dof",
        "b747",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "skywalker_x8",
        "x15",
    }
    assert all(item["available_authority_profiles"] for item in runnable)

    assert main(["vehicle", "interface-report"]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))
    assert payload["schema"] == "taoryx.vehicle-interface-catalog-report/v1alpha1"
    assert payload["status"] == "pass"
    ####
