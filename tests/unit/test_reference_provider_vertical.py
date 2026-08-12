"""Focused vertical proofs for runnable non-physical provider fixtures."""

from __future__ import annotations

from typing import cast

import pytest

from taoryx.model_authoring import (
    author_configuration,
    build_model_authoring_plan,
    custom_sequence,
    run_prepared_mission_composition,
    segment_occurrence,
    select_variant,
)
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.configuration_contract import ConfigurationContractError
from taoryx.trajectory.contract_probe_mission_composition import build_contract_probe_configuration
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection

REFERENCE_PROVIDER_ID = "taoryx.reference.mission-composition"
PROBE_PROVIDER_ID = "taoryx.debug.mission-composition-contract-probe"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover all installed provider packages once for this small slice."""

    return discover_plugins(include_external=False)
    ####


@pytest.mark.parametrize(
    ("model_id", "mission_template_id", "controller_status"),
    (
        (
            "reference_ballistic_3dof",
            "reference_ballistic_3dof_repeatable_sequence_v1",
            "not_applicable",
        ),
        (
            "reference_constant_velocity_waypoint_3dof",
            "reference_constant_velocity_waypoint_3dof_repeatable_sequence_v1",
            "provider_managed",
        ),
    ),
)
def test_analytical_reference_models_advertise_exact_batch_and_step_endpoints(
    plugins: PluginCatalog,
    model_id: str,
    mission_template_id: str,
    controller_status: str,
) -> None:
    """Reference fixtures expose their real runner lifecycle without vehicle claims."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        REFERENCE_PROVIDER_ID,
        model_id,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        mission_template_id=mission_template_id,
    )

    execution = cast(dict[str, object], plan["execution_advertisement"])
    controller = cast(dict[str, object], plan["controller_automation"])
    mission = cast(dict[str, object], plan["navigation_automation"])["mission_template"]

    assert plan["status"] == "ready_to_author"
    assert cast(dict[str, object], plan["selection"])["mission_template_id"] == mission_template_id
    assert cast(dict[str, object], mission)["segment_sequence"]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_and_step_ready"
    assert execution["available_operations"] == ["validate", "batch", "step"]
    assert execution["blocked_operations"] == []
    assert controller["status"] == controller_status
    assert controller["campaigns"] == []
    ####


def test_analytical_reference_models_run_through_the_advertised_common_batch_endpoint(
    plugins: PluginCatalog,
) -> None:
    """Both reference paths produce normalized segments, telemetry, and status."""

    providers = plugins.build_mission_composition_provider_registry()
    provider = providers.provider(REFERENCE_PROVIDER_ID)
    cases = (
        (
            "reference_ballistic_3dof",
            select_variant(
                "launch_state",
                altitude_m=1000.0,
                speed_m_s=150.0,
                heading_deg=0.0,
                flight_path_angle_deg=20.0,
            ),
            custom_sequence(
                segment_occurrence("ballistic_coast", duration_s=2.0),
                segment_occurrence("ballistic_coast", duration_s=2.0),
            ),
            4.0,
        ),
        (
            "reference_constant_velocity_waypoint_3dof",
            select_variant(
                "initial_state",
                altitude_m=1000.0,
                speed_m_s=100.0,
                heading_deg=90.0,
            ),
            custom_sequence(
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="outbound",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=500.0,
                    waypoint_altitude_m=1000.0,
                ),
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="return",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=0.0,
                    waypoint_altitude_m=1000.0,
                ),
            ),
            10.0,
        ),
    )

    for model_id, initialization, segments, final_time_s in cases:
        prepared = author_configuration(
            provider,
            configuration_id=f"reference-vertical-{model_id}",
            model_id=model_id,
            fidelity="point_mass_3dof",
            realization_id="analytical_point_mass",
            values={"initialization": initialization, "segments": segments},
        )
        response = run_prepared_mission_composition(
            providers,
            REFERENCE_PROVIDER_ID,
            prepared,
            output=MissionCompositionOutputSelection(mode="all", cadence_s=1.0, maximum_samples_per_object=17),
        )

        assert response.kind == "trajectory"
        assert response.result.status == "completed"
        assert response.result.primary_model_id == model_id
        primary = response.result.objects[0]
        assert primary.realization_id == "analytical_point_mass"
        assert primary.fidelity == "point_mass_3dof"
        assert primary.samples[-1].time_s == pytest.approx(final_time_s)
        assert primary.segments
        assert all(set(sample.values) == {channel.id for channel in primary.channels} for sample in primary.samples)
    ####


@pytest.mark.parametrize("fidelity", ("coarse", "medium"))
def test_contract_probe_retains_its_exact_batch_advertisement_and_full_result_shape(
    plugins: PluginCatalog,
    fidelity: str,
) -> None:
    """The synthetic probe is first class for integration, never a physics claim."""

    providers = plugins.build_mission_composition_provider_registry()
    plan = build_model_authoring_plan(
        providers,
        plugins.build_controller_tuning_campaign_registry(),
        PROBE_PROVIDER_ID,
        "contract_probe_vehicle",
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=fidelity,
        realization_id=fidelity,
        mission_template_id="deployment_walkthrough",
    )
    execution = cast(dict[str, object], plan["execution_advertisement"])

    assert plan["status"] == "ready_to_author"
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_and_step_ready"
    assert execution["available_operations"] == ["validate", "batch", "step"]
    assert execution["blocked_operations"] == []

    probe = providers.provider(PROBE_PROVIDER_ID)
    prepared = probe.validate_configuration(build_contract_probe_configuration(probe, fidelity=fidelity))  # type: ignore[arg-type]
    response = run_prepared_mission_composition(
        providers,
        PROBE_PROVIDER_ID,
        prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )

    assert response.kind == "trajectory"
    assert response.result.status == "completed"
    assert response.result.primary_model_id == "contract_probe_vehicle"
    assert response.result.objects[0].fidelity == fidelity
    assert response.result.objects[0].realization_id == fidelity
    assert len(response.result.objects) == 3
    assert response.result.relationships
    assert response.result.diagnostics
    assert "no physical or qualification claim" in response.result.claim_boundary
    ####


def test_contract_probe_preflight_rejects_unadvertised_missions_and_fidelities(
    plugins: PluginCatalog,
) -> None:
    """Schema examples are not silently promoted into runnable probe missions."""

    probe = plugins.build_mission_composition_provider_registry().provider(PROBE_PROVIDER_ID)

    with pytest.raises(ConfigurationContractError, match="realization-unavailable"):
        probe.validate_configuration(build_contract_probe_configuration(probe, fidelity="high"))  # type: ignore[arg-type]

    unadvertised = build_contract_probe_configuration(probe).model_copy(
        update={"mission_template_id": "contract_walkthrough"}
    )
    with pytest.raises(ConfigurationContractError, match="unknown-mission-template"):
        probe.validate_configuration(unadvertised)  # type: ignore[arg-type]
    ####
