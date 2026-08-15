"""Fast vertical Composition proof for the runnable A320 pseudo-6DOF reduction."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.a320_reduced_execution import execute_a320_reduced_composition
from taoryx.trajectory.a320_adapter import A320Pseudo6DOFControlPlant, build_a320_local_native_coordinate_lqi_screen_config
from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
from taoryx_a320.resources import model_resource_root

from taoryx.composition_episode import (
    ActionFrame,
    ReducedFixedWingCompositionEpisode,
    open_vehicle_composition_episode,
)
from taoryx.composition_result_catalog import build_composition_release_catalog
from taoryx.generic_tuning import validate_nonlinear_native_coordinate_lqi
from taoryx.local_native_coordinate_lqi import apply_tuning_context_to_native_coordinate_lqi_config
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins, plugin_catalog_scope
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    configuration_instance_from_vehicle_request,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionCloseSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.a320.mission-composition"
MODEL_ID = "a320_openap_3dof"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
PSEUDO_COMPOSITION = "a320_racetrack_capability_pseudo6dof_compose.yaml"
NATIVE_LQI_COMPOSITION = "a320_local_native_coordinate_lqi_screen_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused slice."""

    return discover_plugins(include_external=False, selected=("taoryx.a320",))
    ####


def _pseudo_composition():
    """Compile the documented A320 pseudo-6DOF request through Composition."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / PSEUDO_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _native_lqi_composition():
    """Compile the pinned local native-control LQI endpoint through Composition."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / NATIVE_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_a320_advertisement_builds_a_complete_pseudo6dof_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in advertises its data, controls, navigation, and shared tuner."""

    revision = plugins.plugin_revision("taoryx.a320")
    assert revision.version == "0.1.0a0"
    assert revision.api_version == "1"
    owned_contributions = {
        (contribution.kind, contribution.id)
        for contribution in plugins.contributions
        if contribution.plugin.id == "taoryx.a320"
    }
    assert revision.contribution_count == len(owned_contributions)
    assert {
        ("model", MODEL_ID),
        ("mission_composition_provider", PROVIDER_ID),
        ("controller_tuning_campaign", "a320-point-cruise-performance-lqr-v1"),
        ("controller_tuning_campaign", "a320-pseudo-cruise-attitude-v1"),
        ("local_native_coordinate_lqi_screen_definition", "a320-pseudo-cruise-native-coordinate-lqi-screen-v1"),
        ("local_controller_screen_advertisement", "a320-pseudo-cruise-native-coordinate-lqi-screen-v1"),
    } <= owned_contributions
    assert len(revision.fingerprint) == 64

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        local_controller_screens=plugins.build_local_controller_screen_advertisement_registry(),
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    assert plan["schema"] == "taoryx.model-authoring-plan/v1"
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_id"] == MODEL_ID
    assert selection["model_version"] == "1.0.0+composition-v1"
    assert len(cast(str, selection["model_metadata_fingerprint"])) == 64
    assert selection["physical_family"] == "powered_fixed_wing"
    assert selection["fidelity"] == "pseudo_6dof"
    assert selection["mission_template_id"] == MISSION_ID
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
    }
    assert controller["default_authority_id"] == "kinematic_guidance"
    assert controller["channel_projection"] == "default_authority"
    available_channels = cast(list[dict[str, Any]], controller["available_channels"])
    assert {item["id"] for item in available_channels} > {item["id"] for item in channels}
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "kinematic_guidance",
        "reduced_pilot_command",
        "live_waypoint_guidance",
    }
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert [item["id"] for item in campaigns] == ["a320-pseudo-cruise-attitude-v1"]
    assert controller["local_controller_screen"] is None
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_a320_composition_compiles_and_executes_the_documented_racetrack(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The advertised A320 route reaches its OpenAP-backed batch executor."""

    composition = _pseudo_composition()
    with plugin_catalog_scope(plugins):
        result = execute_a320_reduced_composition(composition, tmp_path / "a320-racetrack")

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.control_realization == "response_law"
    assert result.mission_pass is True
    assert result.runtime["adapter_id"] == "taoryx.fixed_wing.openap.v1"
    assert result.truth_evaluation["required_passed"] == 4
    assert (result.output_dir / "trim.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    status_trace = json.loads((result.output_dir / "status_trace.json").read_text(encoding="utf-8"))
    first = cast(dict[str, object], cast(list[dict[str, object]], status_trace["samples"])[0]["values"])
    assert {"propulsion.thrust", "control.throttle.realized", "resources.mass.fuel_flow"} <= set(first)
    assert float(first["propulsion.thrust"]) > 0.0
    assert 0.0 <= float(first["control.throttle.realized"]) <= 1.0
    assert math.isfinite(float(first["resources.mass.fuel_flow"]))
    ####


def test_a320_composition_episode_accepts_the_advertised_guidance_contract(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The same composition provides a checkpointable interactive episode."""

    episode = open_vehicle_composition_episode(_pseudo_composition(), plugins=plugins)
    contract = episode.interface_contract
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "kinematic_guidance",
            {
                "guidance.speed.command": 230.0,
                "guidance.flight_path_angle.command": 2.0,
                "guidance.heading.command": 0.0,
                "guidance.bank.command": 8.0,
            },
            1.0,
        )
    )

    assert isinstance(episode, ReducedFixedWingCompositionEpisode)
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "response_law"
    assert result.status_frame.values["position.east"] > 0.0
    checkpoint = episode.save_checkpoint(tmp_path / "a320.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_a320_declared_performance_and_attitude_campaigns_run_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The A320 contributes data while core selects LQR or LQI as declared."""

    expected_methods = {
        "a320-point-cruise-performance-lqr-v1": ("lqr", False),
        "a320-pseudo-cruise-attitude-v1": ("lqi", True),
    }
    registry = plugins.build_controller_tuning_campaign_registry()
    for campaign_id, (method, expects_integral_outputs) in expected_methods.items():
        report = registry.registration(campaign_id).run()
        assert report.status == "candidate_ready"
        assert report.nodes
        assert all(node.lqr is not None and node.lqr.method == method for node in report.nodes)
        assert all(bool(node.lqr is not None and node.lqr.integral_output_names) == expects_integral_outputs for node in report.nodes)
    ####


def test_a320_pseudo_lqi_candidate_recovers_through_declared_native_controls(
    plugins: PluginCatalog,
) -> None:
    """The generic LQI design is executable through the response-law plant.

    The validation deliberately stops at the model's aileron/elevator/rudder
    coordinates.  It demonstrates nonlinear native-control recovery without
    promoting the JSBSim surrogate composite to physical surface allocation.
    """

    report = plugins.build_controller_tuning_campaign_registry().registration("a320-pseudo-cruise-attitude-v1").run()
    node = report.nodes[0]
    candidate = node.lqr.best if node.lqr is not None else None
    assert node.trim is not None
    assert candidate is not None
    assert candidate.lqi is not None
    plant = A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository(model_resource_root()))
    initial_state = dict(node.trim.state)
    initial_state.update(
        {
            "bank_angle_rad": 0.08,
            "roll_rate_rad_s": 0.02,
            "beta_rad": 0.01,
            "yaw_rate_rad_s": 0.01,
        }
    )
    validation = validate_nonlinear_native_coordinate_lqi(
        plant,
        node.trim,
        candidate,
        initial_state=initial_state,
        duration_s=20.0,
        dt_s=0.02,
        assessment_state_names=(
            "alpha_rad",
            "beta_rad",
            "roll_rate_rad_s",
            "pitch_rate_rad_s",
            "yaw_rate_rad_s",
            "bank_angle_rad",
        ),
        control_lower={name: -0.2 for name in candidate.control_names},
        control_upper={name: 0.2 for name in candidate.control_names},
        integral_lower={"bank_angle_rad": -0.5},
        integral_upper={"bank_angle_rad": 0.5},
        environment={"thrust_mode": "cruise"},
    )

    assert validation.integrators_exercised
    assert validation.final_normalized_feedback_error_norm < validation.initial_normalized_feedback_error_norm * 0.01
    assert validation.control_saturation_fraction == 0.0
    assert validation.saturated_controls == ()
    payload = validation.as_dict()
    assert payload["control_realization"] == "native_named_coordinates"
    assert payload["candidate"]["method"] == "lqi"
    ####


def test_a320_native_coordinate_lqi_binds_the_exact_common_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The runtime can publish a receipt only for its retained six-state LQI contract."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("a320-pseudo-cruise-attitude-v1")
    contexts = registration.application_contexts(registration.run_cached(tmp_path / "a320-tuning-cache"))
    assert len(contexts) == 1
    config = build_a320_local_native_coordinate_lqi_screen_config()

    applied, receipt = apply_tuning_context_to_native_coordinate_lqi_config(config, contexts[0])

    assert applied.candidate.state_names == contexts[0].state_names
    assert applied.candidate.control_names == contexts[0].control_names
    assert applied.candidate.lqi is not None
    assert applied.candidate.lqi.output_names == contexts[0].integral_output_names
    assert receipt.campaign_id == "a320-pseudo-cruise-attitude-v1"
    assert receipt.controller_method == "lqi"
    ####


def test_a320_native_coordinate_lqi_is_a_first_class_batch_composition_endpoint(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The retained common-host candidate executes through the public local-screen path."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        local_controller_screens=plugins.build_local_controller_screen_advertisement_registry(),
        fidelity="pseudo_6dof",
        realization_id="jsbsim_surrogate_composite_pseudo6dof",
        mission_template_id="a320_local_native_coordinate_lqi_screen_v1",
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    advertisement = cast(dict[str, object], controller["local_controller_screen"])
    assert advertisement["control_realization"] == "native_named_coordinates"
    assert advertisement["operations"] == ["batch"]
    assert advertisement["action_trace"] == "emits_committed_interval_trace_without_batch_visible_actions"
    assert advertisement["physical_effector_allocation"] is False
    assert cast(dict[str, object], advertisement["controller"])["method"] == "lqi"
    assert [item["id"] for item in cast(list[dict[str, object]], advertisement["native_controls"])] == [
        "aileron_rad",
        "elevator_rad",
        "rudder_rad",
    ]

    composition = _native_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "a320-native-lqi", plugins=plugins)
    result = batch.execution

    assert composition.mission == "a320_local_native_coordinate_lqi_screen_v1"
    assert batch.binding.factory_id == "local_native_coordinate_lqi_screen.v1"
    assert batch.passed is True
    assert result.preflight.status == "translation_ready"
    assert result.screen_pass is True
    assert result.screen.validation.integrators_exercised
    assert result.screen.final_error_fraction < 0.01
    assert result.screen.validation.control_saturation_fraction == 0.0
    screen = json.loads((result.output_dir / "local_screen.json").read_text(encoding="utf-8"))
    objective = json.loads((result.output_dir / "objective_report.json").read_text(encoding="utf-8"))
    convergence = json.loads((result.output_dir / "convergence_report.json").read_text(encoding="utf-8"))
    assert screen["control_realization"] == "native_named_coordinates"
    assert screen["physical_effector_allocation"] is False
    assert screen["controller"]["tuning_campaign_id"] == "a320-pseudo-cruise-attitude-v1"
    assert screen["native_control_names"] == ["aileron_rad", "elevator_rad", "rudder_rad"]
    assert screen["control_limits"] == {
        "lower": {"aileron_rad": -0.2, "elevator_rad": -0.2, "rudder_rad": -0.2},
        "upper": {"aileron_rad": 0.2, "elevator_rad": 0.2, "rudder_rad": 0.2},
        "saturation_fraction_limit": 0.0,
    }
    assert screen["evaluation"]["dt_s"] == cast(dict[str, object], advertisement["controller"])["fixed_cadence_s"]
    assert objective["mission_pass"] is True
    assert all(item["required"] is True and item["status"] == "pass" for item in objective["results"])
    assert convergence["schema"] == "taoryx.endpoint-convergence-screen/v1alpha1"
    assert convergence["status"] == "pass"
    assert convergence["pass"] is True
    assert convergence["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    assert convergence["release_evidence_kind"] == "convergence"
    assert cast(dict[str, object], convergence["release_evidence_subject"])["composition_identity_sha256"] == composition.identity_sha256
    release_catalog = build_composition_release_catalog(result.output_dir)
    assert release_catalog["status"] == "ready"
    assert release_catalog["packets"][0]["release_evidence"]["convergence_report.json"]["contract_status"] == "typed_bound"
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert action_trace["samples"]
    status_trace = json.loads((result.output_dir / "status_trace.json").read_text(encoding="utf-8"))
    status_values = cast(dict[str, object], cast(list[dict[str, object]], status_trace["samples"])[-1]["values"])
    assert status_values["control.realization"] == "response_law"
    assert len(cast(list[float], status_values["body_rate"])) == 3
    assert (result.output_dir / "resource_ledger.json").is_file()
    ####


def test_a320_point_performance_campaign_is_selectable_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The OpenAP point-mass product advertises an exact local LQR candidate route."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="point_mass_3dof",
        realization_id="point_mass_3dof",
        mission_template_id=MISSION_ID,
    )

    selection = cast(dict[str, object], plan["selection"])
    controller = cast(dict[str, object], plan["controller_automation"])
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert selection["fidelity"] == "point_mass_3dof"
    assert selection["realization_id"] == "point_mass_3dof"
    assert controller["status"] == "campaign_registered"
    assert [item["id"] for item in campaigns] == ["a320-point-cruise-performance-lqr-v1"]
    ####


def test_a320_reduced_family_adapter_is_public_and_operation_probed(
    plugins: PluginCatalog,
) -> None:
    """The same two reduced products back the public adapter and the batch routes."""

    registry = plugins.build_family_adapter_registry()
    registration = registry.registration(MODEL_ID)

    assert registration.status == "available"
    assert registration.supported_tiers == ("point_mass_3dof", "pseudo_6dof")
    for tier in registration.supported_tiers:
        check = registry.check(MODEL_ID, tier)
        assert check.status == "pass"
        assert check.probe is not None
        operations = {item.operation: item.status for item in check.probe.operations}
        assert operations["state_derivative"] == "pass"
        assert operations["trim"] == "pass"
        assert operations["linearize"] == "pass"
        assert operations["effectiveness"] == "not_applicable"
        assert operations["allocate"] == "not_applicable"
    ####


def test_a320_catalog_advertises_the_same_runnable_endpoints_as_the_slice() -> None:
    """The public catalog names only the batch and episode endpoints exercised here."""

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
        ("batch", "reduced_fixed_wing_openap.v1"),
        ("episode", "reduced_fixed_wing_a320_episode.v1"),
    }
    interface = resolve_vehicle_composition_interface_contract(_pseudo_composition())
    status = {item.id: item for item in interface.status_channels}
    resources = {item.id: item for item in interface.resource_channels}
    assert {"aero.dynamic_pressure", "propulsion.thrust", "control.throttle.realized"} <= set(status)
    assert resources["resources.mass.fuel_flow"].availability == "available"
    assert resources["resources.mass.fuel_flow"].binding == {"batch_telemetry": "fuel_flow_kg_s"}
    ####


def test_a320_jsbsim_surrogate_realization_uses_the_exact_common_batch_and_session_paths(
    plugins: PluginCatalog,
) -> None:
    """The named source composite is a response-law route, not a second or physical plant claim."""

    provider = plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    source_request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / PSEUDO_COMPOSITION)
    configuration = configuration_instance_from_vehicle_request(
        provider,
        source_request,
        realization_id="jsbsim_surrogate_composite_pseudo6dof",
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="a320-jsbsim-surrogate-vertical",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=11),
    )
    response = build_registry_mission_composition_runner(provider).run(request)

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    primary = response.result.objects[0]
    assert response.result.status == "completed"
    assert primary.realization_id == "jsbsim_surrogate_composite_pseudo6dof"
    assert {item.id for item in primary.channels} == {
        "position.local.x",
        "position.local.y",
        "position.local.z",
        "velocity.speed",
        "mass.total",
        "mass.fuel_flow",
        "aerodynamics.dynamic_pressure",
        "propulsion.thrust",
        "control.throttle.realized",
    }

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="a320-jsbsim-surrogate-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            seed=37,
            integration_step_s=0.02,
        )
    )
    assert descriptor.realization_id == "jsbsim_surrogate_composite_pseudo6dof"
    assert set(descriptor.initial_observation.values) >= {
        "propulsion.thrust",
        "control.throttle.realized",
        "resources.mass.total",
        "resources.mass.fuel_flow",
    }
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####
