"""Focused vertical proof for selected cross-plug-in released-body execution."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx_nesc.resources import model_resource_root

from taoryx.deployment import DeploymentResolutionError
from taoryx.plugins import discover_plugins, plugin_catalog_scope
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    configuration_instance_from_vehicle_request,
)
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

_NESC_CHILD_COMPOSITION = model_resource_root() / "examples/vehicle_composition/nesc_staged_source_replay_with_passive_child_pseudo6dof_compose.yaml"
_NESC_PARENT_COMPOSITION = model_resource_root() / "examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml"
_PASSIVE_RUNTIME_ID = "taoryx.passive-bodies.local-atmosphere-release.v1"
_NESC_PROVIDER_ID = "taoryx.nesc.mission-composition"


def _nesc_child_composition():
    """Compile the documented parent/child binding without aggregate discovery."""

    return compile_vehicle_composition(load_vehicle_composition_request(_NESC_CHILD_COMPOSITION))
    ####


def test_passive_bodies_plugin_exposes_its_child_runtime_independently() -> None:
    """The reusable passive package owns its adapter, provider, and child runtime."""

    catalog = discover_plugins(include_external=False, selected=("taoryx.passive-bodies",))

    assert tuple(item.id for item in catalog.plugins) == ("taoryx.passive-bodies",)
    assert catalog.build_deployment_child_runtime_registry().ids == (_PASSIVE_RUNTIME_ID,)
    assert tuple(item.family_id for item in catalog.build_family_adapter_registry().registrations) == ("tumbling_body",)
    providers = catalog.build_mission_composition_provider_registry()
    assert [model.id for model in providers.providers[0].list_models()] == ["tumbling_body"]
    ####


def test_selected_parent_and_passive_plugins_execute_an_independent_child(
    tmp_path: Path,
) -> None:
    """A composition-selected child is propagated without mutating parent replay truth."""

    composition = _nesc_child_composition()
    catalog = discover_plugins(
        include_external=False,
        selected=("taoryx.nesc", "taoryx.passive-bodies"),
    )

    batch = execute_vehicle_composition_batch(composition, tmp_path / "nesc-with-passive-child", plugins=catalog)

    assert batch.passed is True
    payload = batch.execution.as_dict()
    assert payload["parent_mission_pass"] is True
    assert payload["deployment_pass"] is True
    children = payload["deployment_children"]
    assert isinstance(children, list) and len(children) == 1
    child = children[0]
    assert child["runtime_plugin_id"] == "taoryx.passive-bodies"
    assert child["runtime_id"] == _PASSIVE_RUNTIME_ID
    assert child["state_transfer"] == "source_replay_kinematic_projection"
    assert child["parent_plugin_id"] == "taoryx.nesc"
    assert child["parent_model_id"] == "reference_nesc_two_stage_rocket"
    assert child["parent_family_id"] == "reference_nesc_two_stage_rocket"
    assert child["parent_composition_id"] == composition.id
    assert child["requested_fidelity"] == "pseudo_6dof"
    assert child["realized_fidelity"] == "rigid_body_6dof"
    assert child["status"] == "completed"
    assert child["release_state"]["frame"] == "local_tangent"
    assert child["release_state"]["provenance"]["state_transfer"] == "source_replay_kinematic_projection"
    assert len(child["telemetry"]) > 2
    relationships = payload["entity_relationships"]
    assert relationships == [
        {
            "kind": "separation",
            "parent_object_id": "nesc-primary",
            "child_object_id": "nesc-synthetic-cylinder-1",
            "deployment_id": "stage_separation_lineage",
            "event_id": "stage_separation",
            "state_transfer": "source_replay_kinematic_projection",
            "spawn_time_s": pytest.approx(37.4),
        }
    ]
    assert (batch.output_dir / "deployment_children.json").is_file()
    assert (batch.output_dir / "entity_relationships.json").is_file()
    assert (batch.output_dir / "deployment_children" / "01-nesc-synthetic-cylinder-1" / "truth_telemetry.csv").is_file()
    child_payload = json.loads((batch.output_dir / "deployment_children.json").read_text(encoding="utf-8"))
    assert child_payload[0]["child_object_id"] == "nesc-synthetic-cylinder-1"
    ####


def test_common_mission_composition_api_returns_the_selected_cross_plugin_child() -> None:
    """The standard multi-object API retains child data, lineage, and fidelity."""

    catalog = discover_plugins(
        include_external=False,
        selected=("taoryx.nesc", "taoryx.passive-bodies"),
    )
    provider = catalog.build_mission_composition_provider_registry().provider(_NESC_PROVIDER_ID)
    parent_request = load_vehicle_composition_request(_NESC_PARENT_COMPOSITION)
    child_request = load_vehicle_composition_request(_NESC_CHILD_COMPOSITION)
    configuration = configuration_instance_from_vehicle_request(provider, parent_request)
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="nesc-common-api-passive-child",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        deployment_bindings=child_request.deployment_bindings,
        output=MissionCompositionOutputSelection(mode="all", include_segments=True),
    )

    with plugin_catalog_scope(catalog):
        response = build_registry_mission_composition_runner(provider).run(request)

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    result = response.result
    assert result.status == "completed"
    assert len(result.objects) == 2
    child = next(item for item in result.objects if item.parent_object_id is not None)
    assert child.model_id == "nesc-synthetic-cylinder"
    assert child.realization_id == _PASSIVE_RUNTIME_ID
    assert child.role == "synthetic_released_body"
    assert child.fidelity == "rigid_body_6dof"
    assert child.samples[0].time_s == pytest.approx(37.4)
    assert {channel.id for channel in child.channels} >= {
        "position.local.x",
        "position.local.y",
        "position.local.z",
        "velocity.local.x",
        "velocity.local.y",
        "velocity.local.z",
        "mass.total",
        "aerodynamics.drag_force",
        "aerodynamics.projected_area",
        "angular_rate.norm",
    }
    assert len(result.relationships) == 1
    relationship = result.relationships[0]
    assert relationship.state_transfer == "source_replay_kinematic_projection"
    assert relationship.initial_state.values == child.samples[0].values
    event = next(item for item in result.events if item.id == relationship.event_id)
    assert event.category == "separation"
    assert event.data["runtime_plugin_id"] == "taoryx.passive-bodies"
    assert event.data["requested_fidelity"] == "pseudo_6dof"
    assert event.data["realized_fidelity"] == "rigid_body_6dof"
    ####


def test_common_mission_composition_api_rejects_an_unadvertised_child_binding() -> None:
    """A supplied child binding is never silently ignored by the common API."""

    catalog = discover_plugins(
        include_external=False,
        selected=("taoryx.nesc", "taoryx.passive-bodies"),
    )
    provider = catalog.build_mission_composition_provider_registry().provider(_NESC_PROVIDER_ID)
    parent_request = load_vehicle_composition_request(_NESC_PARENT_COMPOSITION)
    child_request = load_vehicle_composition_request(_NESC_CHILD_COMPOSITION)
    configuration = configuration_instance_from_vehicle_request(provider, parent_request)
    prepared = provider.validate_configuration(configuration)
    invalid_binding = child_request.deployment_bindings[0].model_copy(update={"deployment_id": "unadvertised-release"})
    request = MissionCompositionRunRequest(
        request_id="nesc-common-api-unadvertised-child",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        deployment_bindings=(invalid_binding,),
    )

    with plugin_catalog_scope(catalog):
        response = build_registry_mission_composition_runner(provider).run(request)

    assert response.kind == "failure"
    assert response.failure.diagnostics[0].code == "deployment-not-advertised"
    ####


def test_selected_parent_without_passive_plugin_fails_closed(
    tmp_path: Path,
) -> None:
    """A composition binding cannot silently fall back to an aggregate catalog."""

    composition = _nesc_child_composition()
    catalog = discover_plugins(include_external=False, selected=("taoryx.nesc",))

    with pytest.raises(DeploymentResolutionError, match="taoryx.passive-bodies"):
        execute_vehicle_composition_batch(composition, tmp_path / "nesc-missing-passive-child", plugins=catalog)
    ####
