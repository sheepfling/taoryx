"""Focused Composition proof for the source-generated dual-launch glider."""

from __future__ import annotations

import math
from typing import Any, Literal, cast

import pytest
from taoryx.trajectory.dual_launch_mission_composition import build_dual_launch_example_configuration
from taoryx.trajectory.native_mission_composition import build_registry_mission_composition_runner
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
)

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.execution_contract import (
    MissionCompositionExecutionError,
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
)

PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "dual_launch_glider"
FIDELITY = "point_mass_3dof"
REALIZATION_ID = "generated_native_problem"
EXPECTED_CHANNELS = {
    "position.local.north",
    "position.local.east",
    "position.local.down",
    "velocity.local.north",
    "velocity.local.east",
    "velocity.local.down",
    "mass.total",
    "propulsion.thrust",
    "propulsion.mass_flow",
    "control.bank.commanded",
    "control.throttle.commanded",
    "aerodynamics.dynamic_pressure",
    "diagnostics.segment_index",
}


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover installed providers once for the focused vertical slice."""

    return discover_plugins(include_external=False)
    ####


def _prepared(
    provider: RegistryMissionCompositionProvider,
    launch_mode: Literal["air_release", "attached_booster"],
):
    """Build one exact launch-form request through the public schema."""

    return provider.validate_configuration(build_dual_launch_example_configuration(launch_mode, provider.get_model_schema(MODEL_ID)))
    ####


def test_dual_launch_advertisement_builds_a_complete_point_mass_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """Point-mass batch readiness is explicit without inventing child dynamics or tuning."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=REALIZATION_ID,
        mission_template_id="attached_booster_waypoint",
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["physical_family"] == "glider"
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert set(cast(list[str], data_contract["core_output_channels"])) == {
        channel for channel in EXPECTED_CHANNELS if channel.startswith(("position.", "velocity."))
    }
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in channels} == {"command.bank", "command.throttle"}
    assert all(item["availability"] == "available_in_batch" for item in channels)
    assert controller["campaigns"] == []
    segment_automation = cast(dict[str, object], plan["segment_automation"])
    assert [item["segment_id"] for item in cast(list[dict[str, str]], segment_automation["instances"])] == [
        "attached_booster",
        "release_glide",
        "terminal_guidance",
    ]
    ####


@pytest.mark.parametrize(
    ("launch_mode", "mission_id", "maximum_samples"),
    (
        ("air_release", "air_release_waypoint", 1),
        ("attached_booster", "attached_booster_waypoint", 17),
    ),
)
def test_each_launch_form_executes_through_the_public_common_runner(
    launch_mode: Literal["air_release", "attached_booster"],
    mission_id: str,
    maximum_samples: int,
) -> None:
    """Both source forms return finite normalized primary telemetry and lifecycle evidence."""

    provider = RegistryMissionCompositionProvider()
    prepared = _prepared(provider, launch_mode)
    request = MissionCompositionRunRequest(
        request_id=f"dual-launch-vertical-{launch_mode}",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=maximum_samples),
    )
    response = build_registry_mission_composition_runner(provider).run(request)

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    result = response.result
    assert result.status == "completed"
    assert result.primary_model_id == MODEL_ID
    assert len(result.objects) == 1
    assert len(result.relationships) == 0
    primary = result.objects[0]
    assert primary.realization_id == REALIZATION_ID
    assert primary.fidelity == FIDELITY
    assert len(primary.samples) == maximum_samples
    assert {item.id for item in primary.channels} == EXPECTED_CHANNELS
    assert all(set(item.values) == EXPECTED_CHANNELS for item in primary.samples)
    assert all(math.isfinite(float(item.values["mass.total"])) for item in primary.samples)
    assert all(math.isfinite(float(item.values["velocity.local.down"])) for item in primary.samples)
    assert all(0.0 <= float(item.values["control.throttle.commanded"]) <= 1.0 for item in primary.samples)

    model = provider.model(MODEL_ID)
    mission = next(item for item in model.mission_templates if item.id == mission_id)
    batch = next(item for item in mission.operations if item.fidelity == FIDELITY and item.operation == "batch")
    assert batch.status == "available"
    assert batch.common_runner_status == "registered"
    if launch_mode == "attached_booster":
        separation = next(item for item in result.events if item.category == "separation")
        assert separation.kind == "booster-glider-separation"
        assert separation.parent_object_id is None
    else:
        assert not [item for item in result.events if item.category == "separation"]
    ####


def test_dual_launch_advertises_and_enforces_batch_only_and_child_boundaries() -> None:
    """Higher fidelities, sessions, and independently propagated children stay unavailable."""

    provider = RegistryMissionCompositionProvider()
    model = provider.model(MODEL_ID)
    attached = next(item for item in model.mission_templates if item.id == "attached_booster_waypoint")
    point_step = next(item for item in attached.operations if item.fidelity == FIDELITY and item.operation == "step")
    assert point_step.status == "blocked"
    assert point_step.common_runner_status == "not_available"
    assert point_step.blockers == ("no interactive dual-launch Mission Composition session binding is registered",)
    assert attached.compatible_fidelities == (FIDELITY,)
    assert not [item for item in attached.operations if item.fidelity == "pseudo_6dof"]
    pseudo = next(item for item in model.fidelities if item.id == "pseudo_6dof")
    assert pseudo.operations == ("validate",)
    assert pseudo.blockers == ("no native pseudo_or_rigid_body_dual_launch_executor",)
    deployment = model.deployments[0]
    assert deployment.status == "declared"
    assert deployment.lifecycle == "event_only"
    assert deployment.blockers == ("independent_parent_and_child_trajectory_binding",)

    with pytest.raises(MissionCompositionExecutionError) as error:
        MissionCompositionSessionManager(provider).open(
            MissionCompositionOpenSessionRequest(
                session_id="dual-launch-workflow-boundary",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=_prepared(provider, "attached_booster"),
            )
        )

    assert error.value.diagnostic.code == "interactive-operation-not-available"
    assert error.value.diagnostic.phase == "preflight"
    assert error.value.diagnostic.details["blockers"] == list(point_step.blockers)
    ####
