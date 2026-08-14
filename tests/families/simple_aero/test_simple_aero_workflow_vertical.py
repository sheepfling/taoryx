"""Focused vertical proof for the standalone Simple Aero workflow plug-in."""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import pytest
import yaml

import taoryx.mission_workflow_endpoint as workflow_endpoints
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import MissionWorkflowEndpointCatalogFragment, PluginCatalog, discover_plugins
from taoryx.trajectory import build_simple_aero_template_configuration
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.session_contract import (
    MissionCompositionCloseSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
    MissionCompositionSwitchAuthorityRequest,
)
from tools.extract_simple_aero_plugin_assets import check as check_simple_aero_plugin_assets
from tools.extract_simple_aero_plugin_assets import extract as extract_simple_aero_plugin_assets

PACKAGE_ID = "taoryx.simple-aero"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.simple-aero.mission-composition"
MODEL_ID = "simple_aero"
MODEL_VERSION = "1.0.0+mission-composition-v1"
MISSION_ID = "fixed_ld_baseline"
_WORKFLOW_ENDPOINT_ID = "simple-aero-fixed-ld-batch"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the Simple Aero plug-in for this fast workflow proof."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the focused workflow provider through its selected plug-in catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared(provider: Any) -> Any:
    """Build the reviewed baseline through the package-owned public schema."""

    return provider.validate_configuration(
        build_simple_aero_template_configuration(
            MISSION_ID,
            schema=provider.get_model_schema(MODEL_ID),
        )
    )
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused public route falls back to a broad plug-in scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused Simple Aero execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    import taoryx.mission_workflow_endpoint as mission_workflow_endpoint

    monkeypatch.setattr(mission_workflow_endpoint, "discover_plugins", unexpected_discovery)
    ####


def test_simple_aero_packaged_data_is_a_current_exact_extract(tmp_path: Path) -> None:
    """The Simple Aero wheel cannot retain stale fixtures or sibling workflows."""

    assert check_simple_aero_plugin_assets() == ()

    temporary_target = tmp_path / "simple-aero-package-data"
    extract_simple_aero_plugin_assets(temporary_target)
    family_catalog = yaml.safe_load((temporary_target / "verification/alpha2_family_catalog.yaml").read_text(encoding="utf-8"))
    assert [family["family_id"] for family in family_catalog["families"]] == ["simple_aero"]

    (temporary_target / "obsolete-simple-aero-asset.txt").write_text("obsolete\n", encoding="utf-8")
    assert check_simple_aero_plugin_assets(temporary_target) == ("unexpected generated Simple Aero asset: obsolete-simple-aero-asset.txt",)
    ####


def test_simple_aero_focused_provider_advertises_versioned_workflow_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The package owns its model API, generated schedule, and direct throttle mode."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-simple-aero"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count == 3
    assert len(revision.fingerprint) == 64
    contribution = plugins.contribution("mission_workflow_endpoint_catalog", "taoryx.simple-aero.workflow-endpoints")
    fragment = contribution.value
    assert contribution.plugin.id == PACKAGE_ID
    assert isinstance(fragment, MissionWorkflowEndpointCatalogFragment)
    assert fragment.endpoint_ids == (_WORKFLOW_ENDPOINT_ID,)

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)
    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert model.common_runner_operations == ("batch", "step")
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="point_mass_3dof",
        realization_id="fixed_ld_point_mass",
        mission_template_id=MISSION_ID,
    )
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["default_authority_id"] == "generated_mission_commands"
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "generated_mission_commands",
        "direct_throttle_command",
    }
    assert {item["id"] for item in cast(list[dict[str, object]], controller["available_channels"])} == {
        "command.bank",
        "command.throttle",
        "propulsion.command.fraction",
    }
    assert controller["campaigns"] == []
    ####


def test_simple_aero_focused_batch_and_session_routes_retain_the_selected_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The low-fidelity batch and streamed throttle surfaces stay package-scoped."""

    provider = _provider(plugins)
    prepared = _prepared(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="simple-aero-focused-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )
    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == PACKAGE_VERSION
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (MODEL_ID, "point_mass_3dof", "fixed_ld_point_mass"),
    ]
    assert len(response.result.objects[0].samples) == 3

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="simple-aero-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
        )
    )
    assert descriptor.active_authority_profile_id == "generated_mission_commands"
    assert descriptor.action_schema == ()
    generated = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.5,
            expected_sequence=0,
        )
    )
    assert generated.lowering_evidence["bank_effect"] == "telemetry_only_non_steering"

    switched = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="direct_throttle_command",
            expected_sequence=1,
            command_source_id="focused-test",
        )
    )
    assert [item.id for item in switched.action_schema] == ["propulsion.command.fraction"]
    direct = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"propulsion.command.fraction": 0.4},
            duration_s=0.5,
            expected_sequence=1,
        )
    )
    assert direct.lowered_action["command.throttle"] == pytest.approx(0.4)
    feedback = direct.control_feedback[0]
    assert feedback.channel_id == "propulsion.command.fraction"
    assert feedback.disposition == "applied_as_requested"
    assert feedback.achievement_status == "observed"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####


def test_simple_aero_workflow_endpoint_stays_scoped_and_has_a_package_data_fallback(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The endpoint scope and installed-wheel fallback both stay package-owned."""

    _forbid_catalog_rediscovery(monkeypatch)
    source_catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert [item.id for item in source_catalog.endpoints] == [_WORKFLOW_ENDPOINT_ID]
    missing_canonical = Path("/tmp/taoryx-simple-aero-no-canonical-workflow-catalog.yaml")
    monkeypatch.setattr(workflow_endpoints, "MISSION_WORKFLOW_ENDPOINT_SPECS", missing_canonical)
    catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert [item.id for item in catalog.endpoints] == [_WORKFLOW_ENDPOINT_ID]
    report = workflow_endpoints.verify_mission_workflow_endpoint(
        _WORKFLOW_ENDPOINT_ID,
        execute=True,
        plugins=plugins,
    )
    assert report["status"] == "pass", report
    assert report["records"]["advertisement"]["provider_version"] == PACKAGE_VERSION  # type: ignore[index]
    ####
