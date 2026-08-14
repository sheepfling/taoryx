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
from taoryx.plugins import discover_plugins
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_interface import (
    InterfaceChannelBinding,
    bind_declared_sensor_profile,
    build_vehicle_interface_catalog_report,
    interface_contract_for_composition,
    project_committed_status_values,
    resolve_vehicle_interface_contract,
    validate_authority_action_values,
    validate_interface_channel_value,
    validate_vehicle_interface_contract,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("family_id", ["a320_openap_3dof", "f16_s119"])
@pytest.mark.parametrize("fidelity", ["point_mass_3dof", "pseudo_6dof"])
def test_reduced_fixed_wing_interfaces_advertise_mutually_exclusive_stream_profiles(
    family_id: str,
    fidelity: str,
) -> None:
    contract = resolve_vehicle_interface_contract(family_id, fidelity)  # type: ignore[arg-type]
    advertisement = contract.authority_advertisement()

    expected_profiles = {"kinematic_guidance", "reduced_pilot_command", "live_waypoint_guidance"}
    if family_id == "f16_s119" and fidelity == "pseudo_6dof":
        expected_profiles.add("body_rate_command")
    assert contract.default_authority_profile_id == "kinematic_guidance"
    assert advertisement.default_authority_profile_id == "kinematic_guidance"
    assert {profile.id for profile in advertisement.available_profiles} == expected_profiles
    for profile in advertisement.available_profiles:
        assert profile.command_owner == "caller"
        assert profile.selection_scope == "session"
        assert profile.switching_policy == "explicit_bumpless"
        assert profile.lowering_chain

    waypoint = contract.authority_profile("live_waypoint_guidance")
    assert waypoint.authority == "mission"
    assert waypoint.action_ids == (
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
    )
    channels = {channel.id: channel for channel in contract.action_channels}
    assert channels["navigation.waypoint.north.command"].frame == "NED"
    assert channels["navigation.waypoint.altitude.command"].frame is None
    assert channels["navigation.waypoint.altitude.command"].canonical_unit == "m"
    assert channels["pilot.lateral.command"].canonical_unit == "dimensionless"
    if "body_rate_command" in expected_profiles:
        body_rate = contract.authority_profile("body_rate_command")
        assert body_rate.authority == "body_motion"
        assert all(channels[identifier].frame == "body" for identifier in body_rate.action_ids if identifier.startswith("body_rate."))
    else:
        assert "body_rate_command" not in {profile.id for profile in contract.authority_profiles}
    ####


def test_x8_pseudo_interface_separates_kinematic_guidance_from_source_load_probes() -> None:
    contract = resolve_vehicle_interface_contract("skywalker_x8", "pseudo_6dof")

    assert contract.control_realization == "response_law"
    assert contract.authority_profile("kinematic_guidance").availability == "available"
    assert contract.authority_profile("native_control_bridge").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {
        "guidance.override.enabled",
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
        "propulsion.command.fraction",
        "control.longitudinal.bridge.command",
        "control.lateral.bridge.command",
    }
    actions = {channel.id: channel for channel in contract.action_channels}
    assert actions["guidance.speed.command"].binding["tuning_eligible"] is True
    assert actions["guidance.bank.command"].binding["tuning_eligible"] is True
    assert actions["propulsion.command.fraction"].binding["tuning_eligible"] is False
    status = {channel.id: channel for channel in contract.status_channels}
    assert status["attitude.euler"].canonical_unit == "deg"
    assert status["body_rate"].canonical_unit == "rad/s"
    assert contract.effector_channels == ()
    assert "physical actuator evidence" in actions["propulsion.command.fraction"].claim_boundary
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_interface_binding_validates_common_source_and_transform_contracts() -> None:
    binding = InterfaceChannelBinding.model_validate(
        {
            "batch_telemetry": "speed_m_s",
            "derived_from": "batch_telemetry.speed_m_s",
            "transform": "norm",
            "frame": "body",
        }
    )

    assert binding.source_kinds == ("batch_telemetry", "derived")
    assert binding["batch_telemetry"] == "speed_m_s"
    with pytest.raises(ValueError, match="transform requires derived_from"):
        InterfaceChannelBinding.model_validate({"transform": "norm"})
    with pytest.raises(ValueError, match="source_unit requires an explicit scale"):
        InterfaceChannelBinding.model_validate({"runtime_state": "mass", "source_unit": "lb"})
    ####


def test_status_projection_uses_declared_episode_or_batch_binding_present_in_truth_sample() -> None:
    """A dual-route binding must not require the other route's raw alias."""

    plugins = discover_plugins(include_external=False, selected=("taoryx.f16",))
    contract = resolve_vehicle_interface_contract("f16_s119", "pseudo_6dof", plugins=plugins)

    episode_values = project_committed_status_values(
        contract,
        time_s=0.0,
        execution_status="active",
        raw_values={
            "body_rate.roll": 0.1,
            "body_rate.pitch": -0.2,
            "body_rate.yaw": 0.3,
        },
    )
    assert {
        key: episode_values[key]
        for key in ("body_rate.roll", "body_rate.pitch", "body_rate.yaw")
    } == {
        "body_rate.roll": 0.1,
        "body_rate.pitch": -0.2,
        "body_rate.yaw": 0.3,
    }

    batch_values = project_committed_status_values(
        contract,
        time_s=0.0,
        execution_status="active",
        raw_values={"body_rate_rad_s": [0.4, -0.5, 0.6]},
    )
    assert {
        key: batch_values[key]
        for key in ("body_rate.roll", "body_rate.pitch", "body_rate.yaw")
    } == {
        "body_rate.roll": 0.4,
        "body_rate.pitch": -0.5,
        "body_rate.yaw": 0.6,
    }
    ####


def test_hummingbird_pseudo_interface_exposes_response_controls_and_battery_status() -> None:
    contract = resolve_vehicle_interface_contract("hummingbird", "pseudo_6dof")

    assert contract.authority_profile("body_motion_response").availability == "available"
    assert {
        profile.id: (profile.scheme_id, profile.switching_policy)
        for profile in contract.authority_profiles
    } == {
        "body_motion_response": ("body_motion.attitude", "explicit_bumpless"),
        "velocity_yaw_command": ("kinematic.velocity", "explicit_bumpless"),
        "live_waypoint_guidance": ("mission.waypoint", "explicit_bumpless"),
    }
    assert {channel.id for channel in contract.action_channels} == {
        "attitude.roll.command",
        "attitude.pitch.command",
        "attitude.yaw.command",
        "propulsion.command.fraction",
        "propulsion.enable",
        "velocity.north.command",
        "velocity.east.command",
        "velocity.vertical.command",
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
        "navigation.waypoint.vertical_speed.command",
    }
    assert "resources.battery.fraction_remaining" in {channel.id for channel in contract.resource_channels}
    assert "resources.mass.total" in {channel.id for channel in contract.resource_channels}
    assert "propulsion.output.thrust.aggregate" in {channel.id for channel in contract.status_channels}
    assert {
        "velocity.vertical",
        "velocity.horizontal.speed",
        "attitude.roll",
        "attitude.pitch",
        "attitude.yaw",
        "propulsion.enabled",
        "propulsion.output.thrust.fraction",
        "guidance.waypoint.range",
        "guidance.waypoint.captured",
        "guidance.waypoint.status",
    } <= {channel.id for channel in contract.status_channels}
    actions = {channel.id: channel for channel in contract.action_channels}
    assert actions["velocity.vertical.command"].canonical_unit == "m/s"
    assert actions["navigation.waypoint.vertical_speed.command"].canonical_unit == "m/s"
    assert actions["navigation.waypoint.speed.command"].binding["feedback_channel_id"] == "velocity.horizontal.speed"
    assert actions["propulsion.command.fraction"].binding["feedback_channel_id"] == "propulsion.output.thrust.fraction"
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
    numeric = [channel for channel in channels if channel.value_type in {"scalar", "vector3", "vector4"}]
    assert all(channel.canonical_unit is not None or channel.quantity_semantics is not None for channel in numeric)
    assert {
        (channel.id, channel.quantity_semantics)
        for channel in numeric
        if channel.canonical_unit is None
    } == {
        ("control.allocation.residual_norm", "mixed_wrench_norm"),
        ("control.allocation.saturation_count", "count"),
        ("control.feedback_norm", "normalized_error"),
        ("control.source_effectiveness_rank", "count"),
        ("control.wrench.residual_norm", "mixed_wrench_norm"),
    }
    with pytest.raises(ValueError, match="lack explicit value-space contracts"):
        validate_interface_channel_value_space_coverage(["new.public.channel"])
    ####


def test_source_surface_and_schedule_channels_have_explicit_semantic_topology() -> None:
    """New controller-screen telemetry stays typed beyond its native fixture."""

    catalog = load_interface_channel_value_space_catalog()
    expected = {
        "control.controller.method": "finite_set",
        "control.schedule.node_id": "finite_set",
        "control.schedule.selection": "finite_set",
        "effector.surface.symmetric_stabilator.position": "bounded_interval",
        "effector.surface.upper_left_body_flap.position": "bounded_interval",
        "aerodynamics.mach": "positive_half_line",
        "aerodynamics.alpha": "bounded_interval",
        "aerodynamics.beta": "bounded_interval",
        "control.source_effectiveness_rank": "positive_half_line",
        "control.lqi.integral_error.vertical_speed": "euclidean_scalar",
        "aerodynamics.pitch_coefficient": "euclidean_scalar",
        "control.pitch_moment.residual": "euclidean_scalar",
        "trim.full_state.status": "finite_set",
    }
    assert {identifier: catalog[identifier] for identifier in expected} == expected
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
    assert {"aerodynamics.drag_force", "aerodynamics.projected_area", "angular_rate.norm"} <= set(status)
    assert all(status[item].availability == "available_in_batch" for item in {
        "aerodynamics.drag_force",
        "aerodynamics.projected_area",
        "angular_rate.norm",
    })
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
    resources = {channel.id: channel for channel in contract.resource_channels}
    assert resources["resources.mass.total"].binding == {
        "batch_telemetry": "mass_kg",
        "episode_value": "mass_kg",
    }
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
    assert diagnostics["control.realization"].binding == {
        "batch_telemetry": "control_realization",
        "episode_value": "control_realization",
    }
    assert diagnostics["control.wrench.residual_norm"].binding == {
        "batch_telemetry": "wrench_residual_norm",
        "episode_value": "wrench_residual_norm",
    }
    assert diagnostics["control.feedback_norm"].binding == {
        "batch_telemetry": "feedback_norm",
        "episode_value": "feedback_norm",
    }
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_f16_physical_screen_publishes_requested_to_achieved_wrench_residuals() -> None:
    """F-16 local screens expose allocation error rather than a sidecar-only norm."""

    contract = resolve_vehicle_interface_contract("f16_s119", "rigid_body_6dof_surface_allocated")

    status = {channel.id: channel for channel in contract.status_channels}
    assert {
        "control.wrench.residual.force",
        "control.wrench.residual.moment",
    } <= set(status)
    assert status["control.wrench.residual.force"].binding == {
        "batch_telemetry": "residual_force_body_n",
        "frame": "body_frd",
    }
    assert status["control.wrench.residual.moment"].binding == {
        "batch_telemetry": "residual_moment_body_nm",
        "frame": "body_frd",
    }
    diagnostics = {channel.id: channel for channel in contract.diagnostic_channels}
    assert diagnostics["control.allocation.residual_norm"].binding == {"batch_telemetry": "allocation_residual_norm"}
    assert diagnostics["control.allocation.saturation_count"].binding == {"batch_telemetry": "saturation_count"}
    assert validate_vehicle_interface_contract(contract) == ()
    ####


@pytest.mark.parametrize(
    ("model_id", "fidelity", "residual_binding"),
    (
        ("hummingbird", "rigid_body_6dof_surface_allocated", "allocation_residual_norm"),
        ("skywalker_x8", "rigid_body_6dof_surface_allocated", "allocation_controlled_residual_norm"),
        ("b747", "rigid_body_6dof_surface_allocated", "allocation_controlled_residual_norm"),
    ),
)
def test_physical_allocator_interfaces_publish_numeric_health_diagnostics(
    model_id: str,
    fidelity: str,
    residual_binding: str,
) -> None:
    """All committed physical-allocation screens disclose numerical allocator health."""

    contract = resolve_vehicle_interface_contract(model_id, fidelity)  # type: ignore[arg-type]

    diagnostics = {channel.id: channel for channel in contract.diagnostic_channels}
    assert diagnostics["control.allocation.residual_norm"].binding == {"batch_telemetry": residual_binding}
    assert diagnostics["control.allocation.saturation_count"].binding == {"batch_telemetry": "saturation_count"}
    assert diagnostics["control.allocation.residual_norm"].value_type == "scalar"
    assert diagnostics["control.allocation.saturation_count"].value_type == "scalar"
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_hl20_direct_wrench_screen_publishes_the_same_explicit_bridge_without_surface_claims() -> None:
    contract = resolve_vehicle_interface_contract("hl20_mod_k", "rigid_body_6dof_direct_wrench")

    assert contract.control_realization == "direct_wrench"
    assert contract.authority_profile("direct_wrench").availability == "available"
    assert {channel.id for channel in contract.action_channels} == {"wrench.force.command", "wrench.moment.command"}
    assert all(channel.availability == "available" for channel in contract.action_channels)
    resources = {channel.id: channel for channel in contract.resource_channels}
    assert resources["resources.mass.total"].binding == {
        "batch_telemetry": "mass_kg",
        "episode_value": "mass_kg",
    }
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
    assert {record.mission for record in contract.execution_records} == {"rocket_aircraft_high_energy_v1"}
    assert validate_vehicle_interface_contract(contract) == ()
    ####


def test_composition_interface_names_an_exact_mission_without_an_execution_binding() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml")
    )
    contract = resolve_vehicle_composition_interface_contract(composition)

    assert contract.execution_records == ()
    assert "lifting_body_glide_energy_management_v1" in contract.claim_boundary
    assert "unbound_mission" not in contract.claim_boundary
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
                    record.operation == "episode" and record.status == "runnable"
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


def test_interface_report_can_validate_only_one_vehicle_plugin() -> None:
    report = build_vehicle_interface_catalog_report(family_ids=("hummingbird",))

    assert report["status"] == "pass"
    assert report["family_filter"] == ["hummingbird"]
    assert report["family_count"] == 1
    assert report["interface_count"] == 4
    interfaces = cast(list[dict[str, object]], report["interfaces"])
    assert {item["family_id"] for item in interfaces} == {"hummingbird"}
    ####


def test_interface_report_can_validate_only_reduced_fidelities_for_one_plugin() -> None:
    """A quick family gate need not reconstruct its physical controller tiers."""

    report = build_vehicle_interface_catalog_report(
        family_ids=("f16_s119",),
        fidelity_ids=("point_mass_3dof", "pseudo_6dof"),
    )

    assert report["status"] == "pass"
    assert report["family_filter"] == ["f16_s119"]
    assert report["fidelity_filter"] == ["point_mass_3dof", "pseudo_6dof"]
    assert report["interface_count"] == 2
    interfaces = cast(list[dict[str, object]], report["interfaces"])
    assert {(item["family_id"], item["fidelity"]) for item in interfaces} == {
        ("f16_s119", "point_mass_3dof"),
        ("f16_s119", "pseudo_6dof"),
    }
    ####
