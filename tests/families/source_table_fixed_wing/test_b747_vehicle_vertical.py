"""Focused vertical proof for the B747 low-fidelity source-table contract."""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import pytest
from taoryx_source_table_fixed_wing.resources import model_resource_root

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.evaluation import TrajectoryEvaluation
from taoryx.trajectory.native_mission_composition import (
    compile_prepared_vehicle_composition,
    configuration_instance_from_vehicle_request,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionCloseSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

MODEL_ID = "b747"
MODEL_VERSION = "composition-registry-v1"
PACKAGE_ID = "taoryx.source-table-fixed-wing"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.b747.mission-composition"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
PSEUDO_COMPOSITION = "b747_racetrack_capability_pseudo6dof_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the shared source-table package for the B747 slice."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the B747 provider from the caller's selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF B747 racetrack witness."""

    request = load_vehicle_composition_request(
        model_resource_root() / "examples/vehicle_composition" / PSEUDO_COMPOSITION
    )
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if the focused B747 route expands into an aggregate plug-in scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused B747 execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    import taoryx.batch_episode_parity_dispatch as batch_episode_parity_dispatch
    import taoryx.composition_episode as composition_episode
    import taoryx.mission_capability as mission_capability
    import taoryx.vehicle_batch_execution as vehicle_batch_execution
    import taoryx.vehicle_execution_preflight as vehicle_execution_preflight
    import taoryx.vehicle_interface as vehicle_interface
    import taoryx.vehicle_runtime_lowering as vehicle_runtime_lowering

    monkeypatch.setattr(batch_episode_parity_dispatch, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(composition_episode, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(mission_capability, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_batch_execution, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_execution_preflight, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_interface, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_runtime_lowering, "discover_plugins", unexpected_discovery)
    ####


def test_b747_focused_provider_advertises_versioned_lower_tier_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The B747 provider exposes only its user-facing guidance contract at lower tiers."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-source-table-fixed-wing"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)

    assert tuple(item.id for item in provider.list_models()) == (MODEL_ID,)
    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64
    assert model.common_runner_operations == ("batch", "step")

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

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["default_authority_id"] == "kinematic_guidance"
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "kinematic_guidance",
        "native_control_bridge",
    }
    channels = cast(list[dict[str, object]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "guidance.override.enabled",
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
        "propulsion.command.fraction",
        "control.longitudinal.bridge.command",
    }
    assert not any(str(item["id"]).startswith("effector.") for item in channels)
    ####


def test_b747_composition_compiler_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Direct pseudo-6DOF composition compilation keeps the caller's scope."""

    request = load_vehicle_composition_request(
        model_resource_root() / "examples/vehicle_composition" / PSEUDO_COMPOSITION
    )
    _forbid_catalog_rediscovery(monkeypatch)

    composition = compile_vehicle_composition(request, plugins=plugins)

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.observation.profile_id == "truth_debug"
    ####


def test_b747_bounded_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """A bounded native batch prefix stays scoped and reports its time limit.

    The complete transport-scale racetrack remains an execution-witness proof.
    This fast vertical seam deliberately checks the same batch factory's
    request/catalog/status path without making a multi-minute full route part
    of every B747 metadata or controls edit.
    """

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    composition = compile_prepared_vehicle_composition(provider, prepared)
    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "b747-bounded-batch",
        max_steps=3,
        plugins=plugins,
    )

    payload = batch.as_dict()
    runtime = cast(dict[str, object], payload["runtime"])
    evaluation = TrajectoryEvaluation.model_validate_json((batch.output_dir / "evaluation.json").read_text(encoding="utf-8"))
    assert batch.binding.factory_id == "language_backed_powered_fixed_wing.v1"
    assert batch.request.max_steps == 3
    assert batch.packet.host_execution.request.max_steps == 3
    assert not batch.passed
    assert runtime["execution_limit_reason"] == "max_steps"
    assert evaluation.outcome == "time_limited"
    ####


def test_b747_common_session_route_retains_scope_and_high_level_guidance(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The session API stays scoped and accepts the advertised guidance authority."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="b747-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            authority_profile_id="kinematic_guidance",
            seed=13,
            integration_step_s=0.02,
        )
    )

    assert descriptor.active_authority_profile_id == "kinematic_guidance"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id for item in descriptor.action_schema} == {
        "guidance.override.enabled",
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
    }
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"guidance.override.enabled": True, "guidance.bank.command": 8.0},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    feedback = {item.channel_id: item for item in stepped.control_feedback}
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert feedback["guidance.override.enabled"].disposition == "applied_as_requested"
    assert feedback["guidance.bank.command"].disposition == "applied_as_requested"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####
