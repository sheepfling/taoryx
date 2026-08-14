"""Fast vertical Composition proof for the runnable Hummingbird pseudo-6DOF model."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition

from taoryx.composition_episode import ActionFrame, open_vehicle_composition_episode
from taoryx.composition_result_catalog import index_composition_results
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.native_mission_composition import configuration_instance_from_vehicle_request
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
    MissionCompositionSwitchAuthorityRequest,
)
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.hummingbird.mission-composition"
MODEL_ID = "hummingbird"
MISSION_ID = "multirotor_pad_box_yaw_recovery_land_v1"
PSEUDO_COMPOSITION = "hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml"
PHYSICAL_LQI_COMPOSITION = "hummingbird_local_individual_rotor_lqi_screen_compose.yaml"
HORIZONTAL_LQI_COMPOSITION = "hummingbird_local_horizontal_translation_lqi_screen_compose.yaml"
VERTICAL_LQI_COMPOSITION = "hummingbird_local_vertical_translation_lqi_screen_compose.yaml"
DIRECT_WRENCH_COMPOSITION = "hummingbird_local_direct_wrench_screen_compose.yaml"
HOVER_CAMPAIGN_ID = "hummingbird-source-rotor-local-lqi-v1"
VERTICAL_CAMPAIGN_ID = "hummingbird-source-rotor-vertical-lqi-v1"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the selected family once for the focused slice."""

    return discover_plugins(include_external=False, selected=("taoryx.hummingbird",))
    ####


def _pseudo_composition():
    """Compile the documented Hummingbird pseudo-6DOF request through Composition."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / PSEUDO_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _physical_lqi_composition():
    """Compile the documented source-hover motor/LQI Composition screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / PHYSICAL_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _horizontal_lqi_composition():
    """Compile the documented source-local horizontal LQI Composition screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / HORIZONTAL_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _vertical_lqi_composition():
    """Compile the documented source-local vertical force/LQI screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / VERTICAL_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _direct_wrench_composition():
    """Compile the bounded source-hover direct-wrench comparator."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / DIRECT_WRENCH_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_hummingbird_advertisement_builds_a_complete_pseudo6dof_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in exposes its controls, mission data, and shared LQI campaign."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    assert plan["schema"] == "taoryx.model-authoring-plan/v1"
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_id"] == MODEL_ID
    assert selection["physical_family"] == "multirotor"
    assert selection["fidelity"] == "pseudo_6dof"
    assert selection["mission_template_id"] == MISSION_ID
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "attitude.roll.command",
        "attitude.pitch.command",
        "attitude.yaw.command",
        "propulsion.command.fraction",
        "propulsion.enable",
    }
    authorities = cast(list[dict[str, Any]], controller["authorities"])
    assert {item["id"]: item["scheme_id"] for item in authorities} == {
        "body_motion_response": "body_motion.attitude",
        "velocity_yaw_command": "kinematic.velocity",
        "live_waypoint_guidance": "mission.waypoint",
    }
    assert all(item["switching_policy"] == "explicit_bumpless" for item in authorities)
    assert all(item["lowering_chain"] for item in authorities)
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert [item["id"] for item in campaigns] == ["hummingbird-pseudo-hover-attitude-v1"]
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_hummingbird_composition_compiles_and_executes_the_documented_mission(
    tmp_path: Path,
) -> None:
    """The aggregate-thrust pseudo model completes the declared pad-box mission."""

    composition = _pseudo_composition()
    result = execute_hummingbird_pseudo_composition(composition, tmp_path / "hummingbird-mission")

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.control_realization == "response_law"
    assert result.mission_pass is True
    assert result.runtime["adapter_id"] == "taoryx.multirotor.aggregate_thrust_pseudo6dof.v1"
    assert result.runtime["physical_motor_allocation"] is False
    assert (result.output_dir / "controller_transitions.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    assert (result.output_dir / "sensor_observations.json").is_file()
    ####


def test_hummingbird_composition_episode_accepts_the_advertised_body_motion_contract(
    tmp_path: Path,
) -> None:
    """The batch composition also provides a checkpointable semantic episode."""

    episode = open_vehicle_composition_episode(_pseudo_composition(), seed=7, integration_step_s=0.02)
    contract = episode.interface_contract
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "body_motion_response",
            {
                "attitude.yaw.command": 0.5,
                "propulsion.command.fraction": 0.8,
            },
            0.1,
        )
    )

    assert result.status_frame is not None
    assert result.applied_action["yaw_rad"] == pytest.approx(0.5)
    assert result.applied_action["thrust_ratio"] == pytest.approx(0.8)
    assert result.status_frame.values["propulsion.output.thrust.aggregate"] > 0.0
    assert result.status_frame.values["control.physical_motor_allocation"] is False
    checkpoint = episode.save_checkpoint(tmp_path / "hummingbird.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset(seed=7)
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_hummingbird_session_selects_velocity_then_retargets_a_live_waypoint(plugins: PluginCatalog) -> None:
    """One standard stream can move from velocity control to live guidance."""

    provider = plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml"
    )
    configuration = configuration_instance_from_vehicle_request(provider, request).model_copy(
        update={"startup_authority_profile_id": "velocity_yaw_command"}
    )
    prepared = provider.validate_configuration(configuration)
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="hummingbird-flexible-controls",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            seed=13,
            integration_step_s=0.02,
        )
    )

    assert descriptor.active_authority_profile_id == "velocity_yaw_command"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id: item.scheme_id for item in descriptor.authority_profiles} == {
        "body_motion_response": "body_motion.attitude",
        "velocity_yaw_command": "kinematic.velocity",
        "live_waypoint_guidance": "mission.waypoint",
    }
    assert {item.id: item.unit for item in descriptor.action_schema} == {
        "velocity.north.command": "m/s",
        "velocity.east.command": "m/s",
        "velocity.vertical.command": "m/s",
        "attitude.yaw.command": "rad",
        "propulsion.enable": None,
    }
    assert descriptor.agent_action_space is not None
    assert descriptor.agent_action_space.kind == "dict"
    assert descriptor.initial_observation.values["guidance.waypoint.status"] == "inactive"

    velocity_step = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"velocity.north.command": 2.0},
            duration_s=0.5,
            expected_sequence=0,
        )
    )
    assert velocity_step.observation.values["position.north"] > 0.0
    assert velocity_step.observation.values["position.altitude"] == pytest.approx(2.0, abs=0.15)
    assert velocity_step.observation.values["control.controller.method"] == "bounded_velocity_response"
    assert velocity_step.lowering_evidence["lowering_mode"] == "velocity_yaw_command"
    assert set(velocity_step.lowered_action) == {
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "thrust_ratio",
        "motors_enabled",
    }
    velocity_feedback = {item.channel_id: item for item in velocity_step.control_feedback}
    assert velocity_feedback["velocity.north.command"].disposition == "applied_as_requested"
    assert velocity_feedback["velocity.north.command"].achievement_status == "observed"
    assert velocity_feedback["velocity.east.command"].disposition == "held"
    assert velocity_feedback["propulsion.enable"].achieved_value is True

    transition = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="live_waypoint_guidance",
            expected_sequence=1,
            command_source_id="remote-waypoint-client",
        )
    )
    assert transition.active_authority_profile_id == "live_waypoint_guidance"
    assert transition.command_source_id == "remote-waypoint-client"
    assert {item.id for item in transition.action_schema} == {
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
        "navigation.waypoint.vertical_speed.command",
        "attitude.yaw.command",
        "propulsion.enable",
    }

    waypoint_action = {
        "navigation.waypoint.north.command": 4.0,
        "navigation.waypoint.east.command": 2.0,
        "navigation.waypoint.altitude.command": 3.0,
        "navigation.waypoint.capture_radius.command": 0.2,
        "navigation.waypoint.speed.command": 2.0,
        "navigation.waypoint.vertical_speed.command": 1.0,
        "attitude.yaw.command": 1.0,
        "propulsion.enable": True,
    }
    waypoint_step = None
    for index in range(35):
        waypoint_step = manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action=waypoint_action if index == 0 else {},
                duration_s=0.1,
                expected_sequence=1 + index,
            )
        )
        if waypoint_step.observation.values["guidance.waypoint.captured"]:
            break
    assert waypoint_step is not None
    assert waypoint_step.observation.values["guidance.waypoint.captured"] is True
    assert waypoint_step.observation.values["guidance.waypoint.status"] == "captured"
    assert waypoint_step.observation.values["control.controller.method"] == "waypoint_velocity_cascade"
    assert waypoint_step.observation.values["guidance.waypoint.range"] <= 0.2
    assert waypoint_step.lowering_evidence["lowering_mode"] == "live_waypoint_guidance"
    waypoint_feedback = {item.channel_id: item for item in waypoint_step.control_feedback}
    assert waypoint_feedback["navigation.waypoint.north.command"].achievement_status == "observed"
    assert waypoint_feedback["navigation.waypoint.capture_radius.command"].achievement_status == "not_observed"
    ####


def test_hummingbird_waypoint_checkpoint_and_battery_availability_are_explicit(
    tmp_path: Path,
) -> None:
    episode = open_vehicle_composition_episode(_pseudo_composition(), seed=19, integration_step_s=0.02)
    contract = episode.interface_contract
    episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "live_waypoint_guidance",
            {
                "navigation.waypoint.north.command": 3.0,
                "navigation.waypoint.east.command": -1.0,
                "navigation.waypoint.altitude.command": 2.5,
            },
            0.2,
        )
    )
    checkpoint = episode.save_checkpoint(tmp_path / "hummingbird-waypoint.checkpoint.json")
    expected = episode.observe().as_dict()

    episode.reset(seed=19)
    restored = episode.load_checkpoint(checkpoint)
    assert restored.as_dict() == expected
    assert episode.active_authority_profile_id == "live_waypoint_guidance"
    assert restored.values["control_lowering"]["waypoint_target_m"] == [3.0, -1.0, 2.5]

    episode._state = replace(episode._state, battery_fraction=0.0)
    availability = episode.control_authority_availability(
        "live_waypoint_guidance",
        episode.observe(),
    )
    assert availability["runtime_availability"] == "depleted"
    assert availability["reason_codes"] == ("battery_depleted",)
    assert availability["available_action_ids"] == ()
    assert set(cast(dict[str, tuple[str, ...]], availability["unavailable_action_reasons"])) == set(
        contract.authority_profile("live_waypoint_guidance").action_ids
    )
    ####


def test_hummingbird_declared_lqi_campaign_runs_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The Hummingbird declares data while core supplies the common LQI solver."""

    report = plugins.build_controller_tuning_campaign_registry().registration("hummingbird-pseudo-hover-attitude-v1").run()

    assert report.status == "candidate_ready"
    assert report.nodes
    assert all(node.lqr is not None and node.lqr.method == "lqi" for node in report.nodes)
    assert all(node.lqr is not None and node.lqr.integral_output_names for node in report.nodes)
    ####


def test_hummingbird_physical_rotor_campaign_is_selectable_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The physical screen exposes a reusable rotor-coordinate LQI candidate path."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id="hummingbird_local_individual_rotor_lqi_screen_v1",
    )

    selection = cast(dict[str, object], plan["selection"])
    controller = cast(dict[str, object], plan["controller_automation"])
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert selection["fidelity"] == "rigid_body_6dof_surface_allocated"
    assert selection["realization_id"] == "rigid_body_6dof_surface_allocated"
    assert controller["status"] == "provider_managed_with_campaign"
    assert [item["id"] for item in campaigns] == ["hummingbird-source-rotor-local-lqi-v1"]
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    assert local_screen["control_realization"] == "individual_rotor_source_lqi_allocation"
    assert local_screen["operations"] == ["batch"]
    assert local_screen["physical_effector_allocation"] is True
    screen_controller = cast(dict[str, object], local_screen["controller"])
    assert screen_controller["method"] == "lqi"
    assert screen_controller["integral_output_names"] == ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"]
    assert [item["id"] for item in cast(list[dict[str, object]], local_screen["effector_controls"])] == [
        "effector.rotor.1.speed.position",
        "effector.rotor.2.speed.position",
        "effector.rotor.3.speed.position",
        "effector.rotor.4.speed.position",
    ]

    report = plugins.build_controller_tuning_campaign_registry().registration(HOVER_CAMPAIGN_ID).run()
    node = report.nodes[0]
    assert report.status == "candidate_ready"
    assert node.lqr is not None and node.lqr.method == "lqi"
    assert node.lqr is not None and node.lqr.integral_output_names == (
        "roll_error_rad",
        "pitch_error_rad",
        "yaw_error_rad",
    )
    assert node.lqr is not None and node.lqr.best is not None
    assert node.lqr.best.control_names == ("moment_x_nm", "moment_y_nm", "moment_z_nm")
    ####


def test_hummingbird_hover_physical_lqi_screen_applies_the_exact_common_tuning_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The selected source-hover candidate is bound before rotor allocation."""

    registration = plugins.build_controller_tuning_campaign_registry().registration(HOVER_CAMPAIGN_ID)
    contexts = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))

    assert len(contexts) == 1
    batch = execute_vehicle_composition_batch(
        _physical_lqi_composition(),
        tmp_path / "hummingbird-physical-lqi-tuned",
        tuning_context=contexts[0],
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    binding = cast(dict[str, object], runtime["tuning_binding"])

    assert batch.passed is True
    assert binding["campaign_id"] == HOVER_CAMPAIGN_ID
    assert binding["candidate_profile_id"] == contexts[0].candidate_profile_id
    assert binding["applied_gain_fingerprint_sha256"] == contexts[0].resolved_gain_fingerprint_sha256
    ####


def test_hummingbird_vertical_campaign_declares_its_own_force_and_moment_coordinates(
    plugins: PluginCatalog,
) -> None:
    """Vertical translation cannot accidentally reuse the attitude-only candidate."""

    report = plugins.build_controller_tuning_campaign_registry().registration(VERTICAL_CAMPAIGN_ID).run()
    node = report.nodes[0]

    assert report.status == "candidate_ready"
    assert node.lqr is not None and node.lqr.best is not None
    assert node.lqr.best.control_names == ("force_z_n", "moment_x_nm", "moment_y_nm", "moment_z_nm")
    assert node.lqr.best.integral_output_names == (
        "roll_error_rad",
        "pitch_error_rad",
        "yaw_error_rad",
        "w_m_s",
    )
    ####


def test_hummingbird_catalog_advertises_the_same_runnable_endpoints_as_the_slice() -> None:
    """The public catalog does not imply unimplemented motor-level endpoints."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            MISSION_ID,
            "pseudo_6dof",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "hummingbird_aggregate_thrust_pseudo_batch.v1"),
        ("episode", "hummingbird_aggregate_thrust_episode.v1"),
    }
    ####


def test_hummingbird_source_hover_lqi_screen_runs_through_the_composition_factory(tmp_path: Path) -> None:
    """The physical path uses output-integrating wrench feedback and real rotors."""

    composition = _physical_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "hummingbird-physical-lqi")
    payload = batch.as_dict()

    assert batch.passed is True
    assert payload["status"] == "development_local_screen_pass"
    assert payload["screen_pass"] is True
    assert payload["runtime"]["controller_method"] == "lqi"
    assert payload["runtime"]["physical_motor_allocation"] is True
    assert payload["runtime"]["duration_s"] == pytest.approx(1.0)
    assert payload["control_trace"]["achieved_effector_channel_count"] == 4
    assert payload["control_screen"]["mission_pass"] is True
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], preflight["capability_estimate"])
    assert capability["schema"] == "taoryx.concrete-capability-preflight/v1alpha1"
    assert capability["composition_identity_sha256"] == composition.identity_sha256
    assert capability["adapter_id"] == "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1"
    derived_mission = cast(dict[str, object], cast(dict[str, object], payload["preflight"])["derived_mission"])
    derived_capability = cast(dict[str, object], derived_mission["capability"])
    assert derived_capability["physical_screen_status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], derived_capability["physical_screen_execution"])["operations"] == ["validate", "batch"]
    assert (batch.output_dir / "nonlinear_validation.json").is_file()
    robustness = json.loads((batch.output_dir / "robustness_report.json").read_text(encoding="utf-8"))
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    assert robustness["release_evidence_kind"] == "robustness"
    assert robustness["release_evidence_subject"]["composition_identity_sha256"] == composition.identity_sha256
    assert robustness["id"] == "hummingbird-hover-fixed-lqi-mass-variation"
    assert robustness["pass"] is True
    assert [case["id"] for case in robustness["cases"]] == ["mass-0.85x", "mass-1.00x", "mass-1.15x"]
    assert all(case["metrics"]["final_attitude_rate_error_fraction"] < 1.0 for case in robustness["cases"])
    assert all(case["metrics"]["saturation_fraction"] == 0.0 for case in robustness["cases"])
    ####


def test_hummingbird_horizontal_lqi_screen_runs_through_the_composition_factory(tmp_path: Path) -> None:
    """The physical path closes bounded horizontal goals through LQI and actual rotors."""

    composition = _horizontal_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "hummingbird-horizontal-lqi")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    results = cast(list[dict[str, object]], control_screen["results"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], preflight["capability_estimate"])

    assert batch.passed is True
    assert batch.binding.factory_id == "hummingbird_local_horizontal_translation_lqi_screen.v1"
    assert payload["status"] == "development_local_screen_pass"
    assert payload["runtime"]["controller_method"] == "lqi"
    assert payload["runtime"]["physical_motor_allocation"] is True
    assert payload["runtime"]["duration_s"] == pytest.approx(38.0)
    assert control_screen["mission_pass"] is True
    assert all(item["status"] == "pass" for item in results)
    assert capability["adapter_id"] == "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1"
    interface = resolve_vehicle_composition_interface_contract(composition)
    assert {"position.local", "velocity.local"} <= {channel.id for channel in interface.status_channels}
    assert payload["control_trace"]["achieved_effector_channel_count"] == 4
    ####


def test_hummingbird_vertical_lqi_screen_runs_through_collective_force_and_actual_rotors(tmp_path: Path) -> None:
    """The local vertical capture route remains a source-plant allocation proof."""

    composition = _vertical_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "hummingbird-vertical-lqi")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    results = cast(list[dict[str, object]], control_screen["results"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], preflight["capability_estimate"])
    derived_capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])

    assert batch.passed is True
    assert batch.binding.factory_id == "hummingbird_local_vertical_translation_lqi_screen.v1"
    assert payload["status"] == "development_local_screen_pass"
    assert payload["runtime"]["controller_id"] == "hummingbird.local_source_hover_vertical_force_lqi.v1"
    assert payload["runtime"]["controller_method"] == "lqi"
    assert payload["runtime"]["physical_motor_allocation"] is True
    assert payload["runtime"]["duration_s"] == pytest.approx(16.0)
    assert control_screen["mission_pass"] is True
    assert all(item["status"] == "pass" for item in results)
    mass_variation = cast(dict[str, object], payload["runtime"]["mass_variation"])
    assert mass_variation["status"] == "passed"
    assert mass_variation["mass_factors"] == [0.85, 1.0, 1.15]
    assert mass_variation["controller_policy"] == "one_fixed_nominal_mass_lqi_design_across_each_retrimmed_case"
    assert all(item["status"] == "pass" for item in cast(list[dict[str, object]], mass_variation["cases"]))
    assert any(item["id"] == "fixed_nominal_lqi_mass_variation" for item in results)
    assert capability["adapter_id"] == "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1"
    assert derived_capability["mass_variation_status"] == "executed_by_this_vertical_lqi_screen"
    assert derived_capability["mass_variation_factors"] == [0.85, 1.0, 1.15]
    interface = resolve_vehicle_composition_interface_contract(composition)
    assert "control.lqi.integral_error.vertical_speed" in {channel.id for channel in interface.status_channels}
    assert payload["control_trace"]["achieved_effector_channel_count"] == 4
    assert (batch.output_dir / "truth_telemetry.csv").is_file()
    assert (batch.output_dir / "mass_variation_report.json").is_file()
    robustness = json.loads((batch.output_dir / "robustness_report.json").read_text(encoding="utf-8"))
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["id"] == "hummingbird-vertical-fixed-lqi-mass-variation"
    assert robustness["pass"] is True
    assert [case["id"] for case in robustness["cases"]] == ["mass-0.85x", "mass-1x", "mass-1.15x"]
    assert all(case["metrics"]["minimum_phase_capture_dwell_s"] >= 0.8 for case in robustness["cases"])
    assert all(case["metrics"]["saturation_fraction"] == 0.0 for case in robustness["cases"])
    assert robustness["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    ####


def test_hummingbird_local_direct_wrench_screen_runs_through_public_composition(tmp_path: Path) -> None:
    """The direct screen is a bounded batch comparator, never a rotor claim."""

    composition = _direct_wrench_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "hummingbird-local-direct-wrench")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])

    assert batch.passed is True
    assert batch.binding.factory_id == "hummingbird_local_direct_wrench_screen.v1"
    assert payload["status"] == "development_local_screen_pass"
    assert control_screen["screen_pass"] is True
    assert control_screen["observed_wrench_statuses"] == ["feasible"]
    assert control_screen["physical_effector_allocation"] is False
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    assert (batch.output_dir / "resource_ledger.json").is_file()
    results = index_composition_results(batch.output_dir)
    assert results["status"] == "pass"
    assert results["local_controller_screen_count"] == 1
    assert results["records"][0]["outcome"] == "local_screen_pass"
    assert results["records"][0]["resource_ledger_evidence"]["status"] == "verified"
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], preflight["capability_estimate"])
    assert capability["adapter_id"] == "taoryx.hummingbird.local_direct_wrench_screen.capability.v1"
    ####


def test_hummingbird_direct_wrench_interface_advertises_exact_batch_outputs_not_an_episode() -> None:
    """Internal LQR requests and pinned mass are visible without a caller action seam."""

    interface = resolve_vehicle_composition_interface_contract(_direct_wrench_composition())
    actions = {channel.id: channel for channel in interface.action_channels}
    resources = {channel.id: channel for channel in interface.resource_channels}
    status = {channel.id: channel for channel in interface.status_channels}

    assert set(actions) == {"wrench.force.command", "wrench.moment.command"}
    assert all(channel.availability == "available_in_batch" for channel in actions.values())
    assert all(channel.binding["external_override"] is False for channel in actions.values())
    assert all(channel.binding["internal_controller_trace"] is True for channel in actions.values())
    assert resources["resources.mass.total"].availability == "available_in_batch"
    assert resources["resources.mass.total"].binding["batch_telemetry"] == "mass_kg"
    assert status["control.physical_effector_allocation"].availability == "available_in_batch"
    assert status["control.wrench.status"].availability == "available_in_batch"
    ####


def test_hummingbird_physical_lqi_advertisement_is_batch_ready_but_not_qualified() -> None:
    """The physical screen is discoverable without overstating its local proof."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            "hummingbird_local_individual_rotor_lqi_screen_v1",
            "rigid_body_6dof_surface_allocated",
        )
    )
    selection = cast(dict[str, object], kit["selection"])
    maturity = cast(dict[str, object], selection["operational_maturity"])
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert maturity["scope"] == "local_controller_screen"
    assert maturity["endpoint_maturity"] == "batch_ready"
    assert maturity["fidelity_promotion_status"] == "development"
    assert {(item["operation"], item["factory_id"]) for item in endpoints} == {
        ("batch", "hummingbird_local_individual_rotor_lqi_screen.v1"),
    }
    ####


def test_hummingbird_horizontal_lqi_advertisement_is_batch_ready_but_stays_local() -> None:
    """The new route is discoverable without promoting altitude or qualification capability."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            "hummingbird_local_horizontal_translation_lqi_screen_v1",
            "rigid_body_6dof_surface_allocated",
        )
    )
    maturity = cast(dict[str, object], cast(dict[str, object], kit["selection"])["operational_maturity"])
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert maturity["scope"] == "local_controller_screen"
    assert maturity["endpoint_maturity"] == "batch_ready"
    assert {(item["operation"], item["factory_id"]) for item in endpoints} == {
        ("batch", "hummingbird_local_horizontal_translation_lqi_screen.v1"),
    }
    ####


def test_hummingbird_vertical_lqi_advertisement_is_batch_ready_but_stays_local() -> None:
    """The vertical collective route is discoverable without a landing claim."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            "hummingbird_local_vertical_translation_lqi_screen_v1",
            "rigid_body_6dof_surface_allocated",
        )
    )
    maturity = cast(dict[str, object], cast(dict[str, object], kit["selection"])["operational_maturity"])
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert maturity["scope"] == "local_controller_screen"
    assert maturity["endpoint_maturity"] == "batch_ready"
    assert {(item["operation"], item["factory_id"]) for item in endpoints} == {
        ("batch", "hummingbird_local_vertical_translation_lqi_screen.v1"),
    }
    ####


def test_hummingbird_horizontal_lqi_authoring_plan_selects_its_exact_screen(plugins: PluginCatalog) -> None:
    """New model authors see the horizontal reference layer, cadence, and actual rotors."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id="hummingbird_local_horizontal_translation_lqi_screen_v1",
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    screen = cast(dict[str, object], controller["local_controller_screen"])
    controller_detail = cast(dict[str, object], screen["controller"])

    assert screen["id"] == "hummingbird-source-horizontal-translation-individual-rotor-lqi-screen-v1"
    assert controller_detail["method"] == "lqi"
    assert controller_detail["reference_layer"] == "bounded_horizontal_position_error_to_tilt_yaw_reference"
    assert controller_detail["screen_duration_s"] == pytest.approx(38.0)
    endpoint = cast(dict[str, object], plan["focused_endpoint_verification"])
    matching = cast(list[dict[str, object]], endpoint["endpoints"])
    horizontal = next(item for item in matching if item["id"] == "hummingbird-source-horizontal-translation-rotor-lqi")
    assert endpoint["selected_endpoint_status"] == "matching_endpoint_available"
    assert horizontal["matches_selected_mission_and_fidelity"] is True
    assert horizontal["command"] == "taoryx vehicle verify hummingbird-source-horizontal-translation-rotor-lqi"
    ####


def test_hummingbird_vertical_lqi_authoring_plan_selects_collective_force(plugins: PluginCatalog) -> None:
    """New authors see the bounded vertical layer and controlled wrench axes."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id="hummingbird_local_vertical_translation_lqi_screen_v1",
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    screen = cast(dict[str, object], controller["local_controller_screen"])
    controller_detail = cast(dict[str, object], screen["controller"])

    assert screen["id"] == "hummingbird-source-vertical-translation-individual-rotor-lqi-screen-v1"
    assert controller_detail["method"] == "lqi"
    assert controller_detail["reference_layer"] == "bounded_down_position_error_to_vertical_speed_reference"
    assert controller_detail["controlled_wrench_axes"] == ["force_z_n", "moment_x_nm", "moment_y_nm", "moment_z_nm"]
    assert controller_detail["screen_duration_s"] == pytest.approx(16.0)
    endpoint = cast(dict[str, object], plan["focused_endpoint_verification"])
    matching = cast(list[dict[str, object]], endpoint["endpoints"])
    vertical = next(item for item in matching if item["id"] == "hummingbird-source-vertical-translation-rotor-lqi")
    assert endpoint["selected_endpoint_status"] == "matching_endpoint_available"
    assert vertical["matches_selected_mission_and_fidelity"] is True
    assert vertical["command"] == "taoryx vehicle verify hummingbird-source-vertical-translation-rotor-lqi"
    ####


def test_hummingbird_direct_wrench_catalog_advertises_the_narrow_batch_endpoint() -> None:
    """The direct comparator is discoverable without implying physical allocation."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            "hummingbird_local_direct_wrench_screen_v1",
            "rigid_body_6dof_direct_wrench",
        )
    )
    selection = cast(dict[str, object], kit["selection"])
    maturity = cast(dict[str, object], selection["operational_maturity"])
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert maturity["scope"] == "local_controller_screen"
    assert maturity["endpoint_maturity"] == "batch_ready"
    assert {(item["operation"], item["factory_id"]) for item in endpoints} == {
        ("batch", "hummingbird_local_direct_wrench_screen.v1"),
    }
    ####
