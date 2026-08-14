"""Fast vertical Composition proof for the NESC source-replay vehicle."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition
from taoryx_nesc.resources import model_resource_root

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    configuration_instance_from_vehicle_request,
)
from taoryx.vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses
from tools.extract_nesc_plugin_assets import check as check_nesc_plugin_assets
from tools.extract_nesc_plugin_assets import extract as extract_nesc_plugin_assets

PROVIDER_ID = "taoryx.nesc.mission-composition"
MODEL_ID = "reference_nesc_two_stage_rocket"
MODEL_VERSION = "composition-registry-v1"
PACKAGE_ID = "taoryx.nesc"
PACKAGE_VERSION = "0.1.0a0"
MISSION_ID = "staged_rocket_launch_target_state_v1"
FIDELITY = "pseudo_6dof"
COMPOSITION = "nesc_staged_source_replay_pseudo6dof_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the NESC family once for the focused vertical slice."""

    return discover_plugins(include_external=False, selected=("taoryx.nesc",))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the NESC provider through the selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF source-replay composition."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / COMPOSITION)
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _source_replay_composition(
    plugins: PluginCatalog,
    name: str = COMPOSITION,
) -> CompiledVehicleComposition:
    """Compile the documented NESC source-replay request."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / name)
    return compile_vehicle_composition(request, plugins=plugins)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused NESC route tries to expand into an aggregate scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused NESC execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    import taoryx.batch_episode_parity_dispatch as batch_episode_parity_dispatch
    import taoryx.mission_capability as mission_capability
    import taoryx.vehicle_batch_execution as vehicle_batch_execution
    import taoryx.vehicle_execution_preflight as vehicle_execution_preflight
    import taoryx.vehicle_interface as vehicle_interface
    import taoryx.vehicle_runtime_lowering as vehicle_runtime_lowering

    monkeypatch.setattr(batch_episode_parity_dispatch, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(mission_capability, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_batch_execution, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_execution_preflight, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_interface, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_runtime_lowering, "discover_plugins", unexpected_discovery)
    ####


def test_nesc_packaged_data_is_a_current_exact_extract(tmp_path: Path) -> None:
    """The NESC wheel cannot retain stale source or deployment assets."""

    assert check_nesc_plugin_assets() == ()

    temporary_target = tmp_path / "nesc-package-data"
    extract_nesc_plugin_assets(temporary_target)
    (temporary_target / "obsolete-nesc-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_nesc_plugin_assets(temporary_target) == ("unexpected generated NESC asset: obsolete-nesc-asset.txt",)
    ####


def test_nesc_advertisement_builds_a_complete_source_replay_authoring_plan(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The plug-in advertises source data and intentionally no control campaign."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-nesc"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)

    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["physical_family"] == "mass_depleting_rocket"
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["channels"] == []
    assert controller["campaigns"] == []
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_nesc_composition_compiler_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Direct source-replay composition compilation does not widen discovery."""

    _forbid_catalog_rediscovery(monkeypatch)
    composition = _source_replay_composition(plugins)

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == FIDELITY
    assert composition.observation.profile_id == "truth_debug"
    ####


def test_nesc_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The normal batch API retains NESC's selected package scope."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="nesc-focused-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == PACKAGE_VERSION
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (MODEL_ID, FIDELITY, FIDELITY),
    ]
    assert len(response.result.objects[0].samples) == 3
    ####


@pytest.mark.parametrize(
    ("composition_name", "fidelity", "control_realization"),
    (
        ("nesc_staged_source_replay_3dof_compose.yaml", "point_mass_3dof", "uncontrolled_source_replay"),
        ("nesc_staged_source_replay_pseudo6dof_compose.yaml", "pseudo_6dof", "response_law"),
    ),
)
def test_nesc_composition_executes_the_advertised_source_replay(
    tmp_path: Path,
    composition_name: str,
    fidelity: str,
    control_realization: str,
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Every runnable NESC fidelity writes stage truth, status, and provenance."""

    _forbid_catalog_rediscovery(monkeypatch)
    result = execute_nesc_source_replay_composition(
        _source_replay_composition(plugins, composition_name),
        tmp_path / f"nesc-source-replay-{fidelity}",
        plugins=plugins,
    )

    assert result.composition.family_id == MODEL_ID
    assert result.composition.fidelity == fidelity
    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["participating_nonlinear_plant"] is False
    assert result.runtime["control_realization"] == control_realization
    assert result.runtime["physical_gimbal_allocation"] is False
    assert result.truth_evaluation["required_passed"] == 4
    assert result.envelope["phase_order"] == ["stage1_burn", "stack_coast", "stage2_burn", "orbit_coast"]
    action_trace = cast(
        dict[str, object],
        json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8")),
    )
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "source_provenance.json").is_file()
    ####


def test_nesc_public_adapters_exercise_the_exact_source_replay(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Both public reductions probe the pinned history through the common replay seam."""

    _forbid_catalog_rediscovery(monkeypatch)
    registry = plugins.build_family_adapter_registry()
    for tier in ("point_mass_3dof", "pseudo_6dof"):
        check = registry.check(MODEL_ID, tier)
        assert check.status == "pass"
        assert check.probe is not None
        operations = {item.operation: item.status for item in check.probe.operations}
        assert operations["replay"] == "pass"
        assert operations["state_derivative"] == "not_applicable"
        assert operations["trim"] == "not_applicable"
        assert operations["effectiveness"] == "not_applicable"
        assert operations["allocate"] == "not_applicable"
    ####


def test_nesc_provider_advertises_a_batch_only_source_replay_endpoint(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The NESC-only provider exposes no interactive controller or episode claim."""

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)

    assert provider.metadata.version == PACKAGE_VERSION
    assert model.common_runner_operations == ("batch",)
    assert "batch" in model.operations
    assert "step" not in model.operations
    assert all(not support.operations for support in model.control_scheme_support)
    assert {support.availability for support in model.control_scheme_support} <= {"not_available", "planned"}
    assert model.version
    assert len(model.metadata_fingerprint) == 64
    ####


def test_nesc_reduced_witness_gate_does_not_expand_to_other_plugins(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The focused gate covers the two replay endpoints, not the passive child."""

    _forbid_catalog_rediscovery(monkeypatch)
    witnesses = validate_vehicle_execution_witnesses(
        witness_ids=("nesc-3dof-batch", "nesc-pseudo6dof-batch"),
        plugins=plugins,
    )

    assert witnesses["status"] == "pass", witnesses
    assert witnesses["witness_count"] == 2
    assert witnesses["runnable_binding_count"] == 2
    assert witnesses["variant_witness_count"] == 0
    assert witnesses["graph_extension_witness_count"] == 0
    ####
