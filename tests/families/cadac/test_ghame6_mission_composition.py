from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ghame6_mission_composition import (
    GHAME6_FIDELITY_ID,
    GHAME6_MODEL_ID,
    GHAME6_MODEL_VERSION,
    GHAME6_RADAR_MODEL_ID,
    GHAME6_SATELLITE_MODEL_ID,
    CadacGhame6MissionCompositionProvider,
    build_default_ghame6_configuration,
    register_ghame6_mission_composition,
)
from taoryx.families.cadac.ghame6_plugin import Ghame6VehiclePlugin
from test_ghame6 import _write_ghame6_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(tmp_path: Path) -> CadacGhame6MissionCompositionProvider:
    return CadacGhame6MissionCompositionProvider(Ghame6VehiclePlugin(_write_ghame6_case(tmp_path)))


####


def test_ghame6_provider_advertises_t4_envelope_with_phase_reported_t3_rcs(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == GHAME6_MODEL_ID
    assert model.model_kind == "mission_composition"
    assert model.common_runner_operations == ("batch",)
    assert model.fidelities[0].id == GHAME6_FIDELITY_ID
    assert model.fidelities[0].dynamics_fidelity == "rigid_body_6dof"
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("mixed",)
    assert model.output_schema.entity_output.supports_multiple_entities
    assert not model.output_schema.entity_output.supports_dynamic_spawning
    assert "no TVC" in model.fidelities[0].claim_boundary
    controls = model.realizations[0].controls
    assert controls.status == "available"
    assert {item.id for item in controls.channels} >= {
        "actuator.aileron.deflection",
        "actuator.elevator.deflection",
        "actuator.rudder.deflection",
        "rcs.thrust_vector.direction",
        "rcs.lateral_acceleration.command",
        "rcs.normal_acceleration.command",
    }
    assert {item.canonical_unit for item in controls.channels if item.id.startswith("actuator.")} == {"deg"}
    assert {item.canonical_unit for item in controls.channels if item.id in {"rcs.lateral_acceleration.command", "rcs.normal_acceleration.command"}} == {"g"}
    assert controls.rl_action_space(operation="batch").kind == "box"


####


def test_ghame6_common_runner_returns_three_root_actors_and_phase_evidence(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ghame6_configuration(
        provider,
        overrides={
            "aileron_command_deg": 1.0,
            "elevator_command_deg": 2.0,
            "enable_boost_cutoff": True,
            "boost_cutoff_time_s": 0.65,
            "enable_terminal_lock": True,
            "terminal_lock_time_s": 0.85,
            "end_time_s": 1.2,
            "sample_step_s": 0.05,
            "random_seed": 7,
        },
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ghame6_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id="ghame6-phase-smoke",
            provider_id="cadac",
            provider_version=GHAME6_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    result = response.result

    assert result.status == "completed"
    assert result.primary_model_id == GHAME6_MODEL_ID
    assert tuple(item.model_id for item in result.objects) == (
        GHAME6_MODEL_ID,
        GHAME6_SATELLITE_MODEL_ID,
        GHAME6_RADAR_MODEL_ID,
    )
    assert all(item.parent_object_id is None for item in result.objects)
    hyper = result.objects[0]
    assert {sample.values["source_phase"] for sample in hyper.samples} == {
        "atmospheric_surfaces",
        "transfer_angle_rcs",
        "transfer_vector_rcs",
        "interceptor_glideslope_rcs",
        "interceptor_terminal_rcs",
    }
    assert {sample.values["runtime_fidelity"] for sample in hyper.samples} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert tuple(event.kind for event in result.events if event.kind.startswith("source_event_")) == tuple(f"source_event_{index}" for index in range(5))
    assert any(event.kind == "radar_track_update" and event.object_id == "ghame6-radar-1" for event in result.events)
    native_track = next(event for event in result.events if event.kind == "native_relative_state_track")
    assert native_track.data["schema_id"] == "taoryx.tracking.relative-state/v1"
    assert native_track.data["payload"]["target_id"] == "ghame6-satellite-1"
    assert result.relationships == ()


####


def test_ghame6_core_selection_retains_truth_for_all_three_source_actors(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ghame6_configuration(
        provider,
        overrides={"end_time_s": 0.1, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ghame6-core",
            provider_id="cadac",
            provider_version=GHAME6_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert [channel.id for channel in result.objects[0].channels] == [
        "position_inertial_m",
        "velocity_inertial_mps",
        "quaternion_wxyz",
        "body_rates_inertial_rad_s",
    ]
    assert [channel.id for channel in result.objects[1].channels] == [
        "position_inertial_m",
        "velocity_inertial_mps",
    ]
    assert [channel.id for channel in result.objects[2].channels] == [
        "position_inertial_m",
        "velocity_inertial_mps",
    ]


####


def test_ghame6_rejects_isolated_rcs_phase_as_the_run_realization(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ghame6_configuration(provider)
    payload = configuration.model_dump()
    payload["realization_id"] = "cadac-source-phase.transfer_vector_rcs"
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="supports only realization"):
        provider.validate_configuration(wrong)
    ####


####
