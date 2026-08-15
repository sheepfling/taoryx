"""Focused vertical proof for the standalone low-fidelity debug-models plug-in."""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.trajectory.contract_probe_mission_composition import build_contract_probe_configuration
from taoryx.trajectory.reference_mission_composition import (
    ReferenceBallisticLaunch,
    ReferenceWaypoint,
    ReferenceWaypointCourseStart,
)

import taoryx.mission_workflow_endpoint as workflow_endpoints
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import MissionWorkflowEndpointCatalogFragment, PluginCatalog, discover_plugins
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
    MissionCompositionSwitchAuthorityRequest,
)
from tools.extract_debug_models_plugin_assets import check as check_debug_models_plugin_assets
from tools.extract_debug_models_plugin_assets import extract as extract_debug_models_plugin_assets

PACKAGE_ID = "taoryx.debug-models"
PACKAGE_VERSION = "0.1.0a0"
REFERENCE_PROVIDER_ID = "taoryx.reference.mission-composition"
PROBE_PROVIDER_ID = "taoryx.debug.mission-composition-contract-probe"
_ENDPOINT_IDS = (
    "reference-ballistic-3dof-batch",
    "reference-waypoint-3dof-batch",
    "debug-contract-probe-batch",
)


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the debug-models package for this fast vertical proof."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused workflow route attempts an aggregate plug-in scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused debug-models execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    monkeypatch.setattr(workflow_endpoints, "discover_plugins", unexpected_discovery)
    ####


def _open_request(provider: Any, session_id: str, prepared: Any) -> MissionCompositionOpenSessionRequest:
    """Build a version-pinned public session request for one focused provider."""

    return MissionCompositionOpenSessionRequest(
        session_id=session_id,
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
    )
    ####


def test_debug_models_packaged_data_is_a_current_exact_extract(tmp_path: Path) -> None:
    """The debug-models wheel cannot retain stale endpoint fixtures."""

    assert check_debug_models_plugin_assets() == ()

    temporary_target = tmp_path / "debug-models-package-data"
    extract_debug_models_plugin_assets(temporary_target)
    (temporary_target / "obsolete-debug-model-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_debug_models_plugin_assets(temporary_target) == ("unexpected generated debug-models asset: obsolete-debug-model-asset.txt",)
    ####


def test_debug_models_publish_versioned_models_controls_and_endpoint_ownership(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Each nonphysical model is independently discoverable and precisely advertised."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-debug-models"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    owned_contributions = {
        (contribution.kind, contribution.id)
        for contribution in plugins.contributions
        if contribution.plugin.id == PACKAGE_ID
    }
    assert revision.contribution_count == len(owned_contributions)
    assert {
        ("mission_composition_provider", REFERENCE_PROVIDER_ID),
        ("mission_composition_provider", PROBE_PROVIDER_ID),
        ("mission_workflow_endpoint_catalog", "taoryx.debug-models.workflow-endpoints"),
    } <= owned_contributions
    assert len(revision.fingerprint) == 64
    assert tuple(item.id for item in plugins.records("mission_composition_provider")) == (
        REFERENCE_PROVIDER_ID,
        PROBE_PROVIDER_ID,
    )
    contribution = plugins.contribution(
        "mission_workflow_endpoint_catalog",
        "taoryx.debug-models.workflow-endpoints",
    )
    fragment = contribution.value
    assert contribution.plugin.id == PACKAGE_ID
    assert isinstance(fragment, MissionWorkflowEndpointCatalogFragment)
    assert fragment.endpoint_ids == _ENDPOINT_IDS

    _forbid_catalog_rediscovery(monkeypatch)
    providers = plugins.build_mission_composition_provider_registry()
    expected_models = {
        REFERENCE_PROVIDER_ID: (
            ("reference_ballistic_3dof", "1.0.0", "ballistic_3dof"),
            (
                "reference_constant_velocity_waypoint_3dof",
                "1.0.0",
                "constant_velocity_waypoint_3dof",
            ),
        ),
        PROBE_PROVIDER_ID: (("contract_probe_vehicle", "1.0.0", "contract_probe"),),
    }
    for provider_id, expected in expected_models.items():
        provider = providers.provider(provider_id)
        native_provider = provider.resolve_provider()
        assert provider.metadata.version == PACKAGE_VERSION
        assert native_provider.plugin_catalog is plugins
        assert tuple((item.id, item.version, item.model_kind) for item in provider.list_models()) == expected
        for model in provider.list_models():
            schema = provider.get_model_schema(model.id)
            assert schema.model_version == model.version
            assert model.output_schema.model_version == model.version
            assert len(model.metadata_fingerprint) == 64
            assert len(model.configuration_schema_fingerprint) == 64
            assert len(model.output_schema_fingerprint) == 64
            assert model.common_runner_operations == ("batch", "step")

    plans = {
        "reference_ballistic_3dof": build_model_authoring_plan(
            providers,
            plugins.build_controller_tuning_campaign_registry(),
            REFERENCE_PROVIDER_ID,
            "reference_ballistic_3dof",
            family_adapters=plugins.build_family_adapter_registry(),
            fidelity="point_mass_3dof",
            realization_id="analytical_point_mass",
            mission_template_id="reference_ballistic_3dof_repeatable_sequence_v1",
        ),
        "reference_constant_velocity_waypoint_3dof": build_model_authoring_plan(
            providers,
            plugins.build_controller_tuning_campaign_registry(),
            REFERENCE_PROVIDER_ID,
            "reference_constant_velocity_waypoint_3dof",
            family_adapters=plugins.build_family_adapter_registry(),
            fidelity="point_mass_3dof",
            realization_id="analytical_point_mass",
            mission_template_id="reference_constant_velocity_waypoint_3dof_repeatable_sequence_v1",
        ),
        "contract_probe_vehicle": build_model_authoring_plan(
            providers,
            plugins.build_controller_tuning_campaign_registry(),
            PROBE_PROVIDER_ID,
            "contract_probe_vehicle",
            family_adapters=plugins.build_family_adapter_registry(),
            fidelity="medium",
            realization_id="medium",
            mission_template_id="deployment_walkthrough",
        ),
    }
    expected_controls = {
        "reference_ballistic_3dof": ("not_applicable", "open_loop_coast", {"open_loop_coast"}),
        "reference_constant_velocity_waypoint_3dof": (
            "provider_managed",
            "configured_waypoint_guidance",
            {
                "configured_waypoint_guidance",
                "kinematic_velocity_command",
                "live_waypoint_guidance",
            },
        ),
        "contract_probe_vehicle": (
            "not_applicable",
            "debug_guidance_control",
            {
                "synthetic_mission_authority",
                "debug_guidance_control",
                "debug_discrete_control",
                "debug_event_control",
            },
        ),
    }
    expected_endpoint_ids = {
        "reference_ballistic_3dof": "reference-ballistic-3dof-batch",
        "reference_constant_velocity_waypoint_3dof": "reference-waypoint-3dof-batch",
        "contract_probe_vehicle": "debug-contract-probe-batch",
    }
    for model_id, plan in plans.items():
        controller = cast(dict[str, object], plan["controller_automation"])
        endpoint = cast(dict[str, object], plan["focused_endpoint_verification"])
        status, default_authority_id, authority_ids = expected_controls[model_id]
        assert plan["status"] == "ready_to_author"
        assert controller["status"] == status
        assert controller["default_authority_id"] == default_authority_id
        assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == authority_ids
        assert controller["campaigns"] == []
        assert endpoint["selected_endpoint_status"] == "matching_endpoint_available"
        assert endpoint["selected_endpoint_ids"] == [expected_endpoint_ids[model_id]]
    ####


def test_debug_models_stream_controls_with_explicit_owner_and_feedback(
    plugins: PluginCatalog,
) -> None:
    """Open-loop, guidance, waypoint, and synthetic control modes stay package-scoped."""

    providers = plugins.build_mission_composition_provider_registry()
    reference = providers.provider(REFERENCE_PROVIDER_ID)
    native_reference = reference.resolve_provider()

    ballistic = native_reference.prepare_ballistic(
        ReferenceBallisticLaunch(
            altitude_m=1000.0,
            speed_m_s=120.0,
            heading_deg=30.0,
            flight_path_angle_deg=10.0,
        ),
        1.0,
    )
    ballistic_manager = MissionCompositionSessionManager(reference)
    ballistic_descriptor = ballistic_manager.open(_open_request(reference, "debug-ballistic-session", ballistic))
    assert ballistic_descriptor.active_authority_profile_id == "open_loop_coast"
    assert ballistic_descriptor.action_schema == ()
    ballistic_step = ballistic_manager.step(
        MissionCompositionSessionStepRequest(
            session_id=ballistic_descriptor.session_id,
            action={},
            duration_s=0.25,
            expected_sequence=0,
        )
    )
    assert ballistic_step.requested_action == {}
    assert ballistic_step.observation.control_authority is not None
    assert ballistic_step.observation.control_authority.command_owner == "open_loop"

    waypoint = native_reference.prepare_waypoint_course(
        ReferenceWaypointCourseStart(
            altitude_m=1000.0,
            speed_m_s=100.0,
            heading_deg=0.0,
        ),
        (ReferenceWaypoint(north_m=1000.0, east_m=0.0, altitude_m=1000.0),),
    )
    waypoint_manager = MissionCompositionSessionManager(reference)
    waypoint_descriptor = waypoint_manager.open(_open_request(reference, "debug-waypoint-session", waypoint))
    assert {item.id for item in waypoint_descriptor.authority_profiles} == {
        "configured_waypoint_guidance",
        "kinematic_velocity_command",
        "live_waypoint_guidance",
    }
    kinematic = waypoint_manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=waypoint_descriptor.session_id,
            authority_profile_id="kinematic_velocity_command",
            expected_sequence=0,
            command_source_id="focused-kinematic-client",
        )
    )
    assert [item.id for item in kinematic.action_schema] == [
        "guidance.speed.command",
        "guidance.heading.command",
        "guidance.flight_path_angle.command",
    ]
    turned = waypoint_manager.step(
        MissionCompositionSessionStepRequest(
            session_id=waypoint_descriptor.session_id,
            action={"guidance.heading.command": 90.0},
            duration_s=0.5,
            expected_sequence=0,
        )
    )
    heading_feedback = {item.channel_id: item for item in turned.control_feedback}["guidance.heading.command"]
    assert heading_feedback.disposition == "applied_as_requested"
    assert heading_feedback.achievement_status == "observed"
    assert heading_feedback.achieved_value == pytest.approx(90.0)
    live = waypoint_manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=waypoint_descriptor.session_id,
            authority_profile_id="live_waypoint_guidance",
            expected_sequence=1,
            command_source_id="focused-waypoint-client",
        )
    )
    assert [item.id for item in live.action_schema] == [
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
    ]
    waypoint_step = waypoint_manager.step(
        MissionCompositionSessionStepRequest(
            session_id=waypoint_descriptor.session_id,
            action={
                "navigation.waypoint.north.command": 200.0,
                "navigation.waypoint.east.command": 400.0,
                "navigation.waypoint.altitude.command": 1200.0,
                "navigation.waypoint.capture_radius.command": 10.0,
                "navigation.waypoint.speed.command": 80.0,
            },
            duration_s=0.5,
            expected_sequence=1,
        )
    )
    waypoint_feedback = {item.channel_id: item for item in waypoint_step.control_feedback}["navigation.waypoint.north.command"]
    assert waypoint_feedback.disposition == "applied_as_requested"
    assert waypoint_feedback.achievement_status == "observed"

    probe = providers.provider(PROBE_PROVIDER_ID)
    native_probe = probe.resolve_provider()
    probe_prepared = native_probe.validate_configuration(build_contract_probe_configuration(native_probe, fidelity="medium"))
    probe_manager = MissionCompositionSessionManager(probe)
    probe_descriptor = probe_manager.open(_open_request(probe, "debug-contract-probe-session", probe_prepared))
    assert probe_descriptor.active_authority_profile_id == "debug_guidance_control"
    assert {item.id for item in probe_descriptor.authority_profiles} == {
        "debug_guidance_control",
        "debug_discrete_control",
        "debug_event_control",
    }
    probe_step = probe_manager.step(
        MissionCompositionSessionStepRequest(
            session_id=probe_descriptor.session_id,
            action={"guidance.heading.command": 90.0},
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    probe_feedback = {item.channel_id: item for item in probe_step.control_feedback}["guidance.heading.command"]
    assert probe_feedback.disposition == "applied_as_requested"
    assert probe_step.observation.values["attitude.heading_deg"] == pytest.approx(90.0)
    events = probe_manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=probe_descriptor.session_id,
            authority_profile_id="debug_event_control",
            expected_sequence=1,
            command_source_id="focused-event-client",
        )
    )
    assert [item.id for item in events.action_schema] == [
        "payload.release.command",
        "payload.arm.command",
    ]
    armed = probe_manager.step(
        MissionCompositionSessionStepRequest(
            session_id=probe_descriptor.session_id,
            action={"payload.arm.command": "arm"},
            duration_s=0.1,
            expected_sequence=1,
        )
    )
    assert armed.events == ("payload_armed",)
    ####


def test_debug_model_endpoint_catalog_stays_scoped_and_loads_package_data(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Endpoint verification uses only the selected package and its owned assets."""

    _forbid_catalog_rediscovery(monkeypatch)
    source_catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert tuple(item.id for item in source_catalog.endpoints) == _ENDPOINT_IDS
    missing_canonical = Path("/tmp/taoryx-debug-models-no-canonical-workflow-catalog.yaml")
    monkeypatch.setattr(workflow_endpoints, "MISSION_WORKFLOW_ENDPOINT_SPECS", missing_canonical)
    package_catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert tuple(item.id for item in package_catalog.endpoints) == _ENDPOINT_IDS
    for endpoint_id in _ENDPOINT_IDS:
        report = workflow_endpoints.verify_mission_workflow_endpoint(
            endpoint_id,
            execute=True,
            plugins=plugins,
        )
        assert report["status"] == "pass", report
        records = cast(dict[str, object], report["records"])
        advertisement = cast(dict[str, object], records["advertisement"])
        assert advertisement["provider_version"] == PACKAGE_VERSION
    ####
