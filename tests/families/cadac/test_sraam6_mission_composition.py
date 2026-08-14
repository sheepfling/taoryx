from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.sraam6_mission_composition import (
    SRAAM6_FIDELITY_ID,
    SRAAM6_FIN_REALIZATION_ID,
    SRAAM6_MODEL_ID,
    SRAAM6_TARGET_MODEL_ID,
    SRAAM6_TVC_PHASE_ID,
    SRAAM6_TVC_REALIZATION_ID,
    CadacSraam6MissionCompositionProvider,
    build_default_sraam6_configuration,
    register_sraam6_mission_composition,
)
from taoryx.families.cadac.sraam6_plugin import Sraam6VehiclePlugin
from test_sraam6 import _write_sraam6_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)
from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionResetSessionRequest,
    MissionCompositionSessionStepRequest,
)


def _provider(tmp_path: Path) -> CadacSraam6MissionCompositionProvider:
    return CadacSraam6MissionCompositionProvider(Sraam6VehiclePlugin(_write_sraam6_case(tmp_path)))


####


def test_sraam6_provider_advertises_available_fins_and_blocked_tvc(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == SRAAM6_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == SRAAM6_FIDELITY_ID
    assert model.fidelities[0].operations == ("validate", "batch", "step")
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("aerodynamic_surfaces",)
    assert [(item.id, item.status) for item in model.realizations] == [
        (SRAAM6_FIN_REALIZATION_ID, "available"),
        (SRAAM6_TVC_REALIZATION_ID, "blocked"),
    ]
    assert model.realizations[1].actuator_types == ("thrust_vectoring",)
    outputs = {channel.id: channel for channel in provider.get_model_output_schema(SRAAM6_MODEL_ID).channels}
    assert outputs["requested_fins_deg"].operations == ("batch", "step")
    assert outputs["achieved_fins_deg"].operations == ("batch", "step")


####


def test_sraam6_common_runner_returns_missile_and_target_root_objects(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_sraam6_configuration(
        provider,
        overrides={"end_time_s": 0.12, "sample_step_s": 0.02},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_sraam6_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="sraam6-smoke",
        provider_id="cadac",
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )

    response = registry.run(request)
    assert response.kind == "trajectory", response.model_dump(mode="json")
    result = response.result

    assert result.primary_model_id == SRAAM6_MODEL_ID
    assert [item.model_id for item in result.objects] == [SRAAM6_MODEL_ID, SRAAM6_TARGET_MODEL_ID]
    assert all(item.parent_object_id is None for item in result.objects)
    assert result.objects[0].fidelity == SRAAM6_FIDELITY_ID
    assert result.objects[1].fidelity == "point_mass_3dof"
    assert {item.id for item in result.objects[0].channels} >= {
        "position_ned_m",
        "quaternion_wxyz",
        "requested_fins_deg",
        "achieved_fins_deg",
        "target_range_m",
        "mass_kg",
    }
    assert {item.id for item in result.objects[1].channels} >= {
        "position_ned_m",
        "heading_deg",
        "normal_load_g",
    }
    first_missile = result.objects[0].samples[0].values
    assert first_missile["requested_fins_deg"] != first_missile["achieved_fins_deg"] or all(value == 0.0 for value in first_missile["requested_fins_deg"])


####


def test_sraam6_core_output_selection_keeps_full_rigid_body_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_sraam6_configuration(
        provider,
        overrides={"end_time_s": 0.02, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="sraam6-core",
        provider_id="cadac",
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )

    result = provider.execute_batch(request)

    assert [item.id for item in result.objects[0].channels] == [
        "position_ned_m",
        "velocity_ned_mps",
        "quaternion_wxyz",
        "body_rates_rad_s",
    ]
    assert [item.id for item in result.objects[1].channels] == [
        "position_ned_m",
        "velocity_ned_mps",
    ]


####


def test_sraam6_persistent_session_owns_target_seeker_and_native_sensor_bus(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_sraam6_configuration(
        provider,
        overrides={"end_time_s": 0.04, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="sraam6-persistent",
            provider_id="cadac",
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            seed=31,
            integration_step_s=0.01,
        )
    )

    assert descriptor.action_schema == ()
    assert descriptor.initial_observation.values["native_relative_state_track"]["valid"]
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="sraam6-persistent", duration_s=0.02))

    assert step.time_start_s == 0.0
    assert step.time_end_s == 0.02
    assert step.observation.lifecycle == "active"
    assert isinstance(step.observation.values["normal_acceleration_g"], float)
    assert step.observation.values["source_seeker_state"]["seeker_mode"] >= 2
    assert len(step.observation.values["requested_control_deg"]) == 3
    assert len(step.observation.values["achieved_control_deg"]) == 3
    assert len(step.observation.values["requested_fins_deg"]) == 4
    assert len(step.observation.values["achieved_fins_deg"]) == 4
    track = step.observation.values["native_relative_state_track"]
    assert track["sensor_id"] == "sraam6-native-relative-state"
    assert track["sequence"] == 2
    assert track["payload"]["target_id"] == "sraam6-target"
    assert [packet.sequence for packet in provider.session_sensor_packets("sraam6-persistent")] == [0, 1, 2]

    reset = provider.reset_session(MissionCompositionResetSessionRequest(session_id="sraam6-persistent"))
    assert reset.sequence == 0
    assert reset.time_s == 0.0
    assert reset.values == descriptor.initial_observation.values


####


def test_sraam6_optional_tvc_validates_but_cannot_execute_batch(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_sraam6_configuration(provider, phase_id=SRAAM6_TVC_PHASE_ID)
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="sraam6-tvc-blocked",
        provider_id="cadac",
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )

    assert prepared.configuration.realization_id == SRAAM6_TVC_REALIZATION_ID
    with pytest.raises(ValueError, match="not registered for batch execution"):
        provider.execute_batch(request)
    ####


####
