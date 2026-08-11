from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_srbm_mission_composition import (
    ADS6_SRBM_FIDELITY_ID,
    ADS6_SRBM_MODEL_ID,
    ADS6_SRBM_REALIZATION_ID,
    CadacAds6SrbmMissionCompositionProvider,
    build_default_ads6_srbm_configuration,
    register_ads6_srbm_mission_composition,
)
from taoryx.families.cadac.ads6_srbm_plugin import Ads6SrbmVehiclePlugin
from test_ads6_srbm import _write_ads6_srbm_case

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


def _provider(tmp_path: Path, **case_options: object) -> CadacAds6SrbmMissionCompositionProvider:
    return CadacAds6SrbmMissionCompositionProvider(Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path, **case_options)))


####


def test_ads6_srbm_provider_advertises_one_exact_t2_response_law(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == ADS6_SRBM_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert [item.id for item in model.fidelities] == [ADS6_SRBM_FIDELITY_ID]
    assert [(item.id, item.status) for item in model.realizations] == [(ADS6_SRBM_REALIZATION_ID, "available")]
    assert model.fidelities[0].runtime_fidelity == "pseudo_6dof"
    assert model.fidelities[0].control_realization == "response_law"
    assert model.presentation.properties[0].value == "ROCKET5"
    outputs = {channel.id: channel for channel in provider.get_model_output_schema(ADS6_SRBM_MODEL_ID).channels}
    assert outputs["normal_command_g"].operations == ("batch", "step")
    assert outputs["lateral_command_g"].operations == ("batch", "step")
    assert outputs["normal_acceleration_g"].canonical_unit == "g"
    assert outputs["lateral_acceleration_g"].canonical_unit == "g"


####


def test_ads6_srbm_common_runner_emits_translation_as_core_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_srbm_configuration(
        provider,
        overrides={"end_time_s": 0.05, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ads6_srbm_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id="ads6-srbm-core",
            provider_id="cadac",
            provider_version="0.9.0",
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )
    result = response.result
    rocket = result.objects[0]

    assert result.status == "completed"
    assert result.primary_model_id == ADS6_SRBM_MODEL_ID
    assert len(result.objects) == 1
    assert rocket.parent_object_id is None
    assert rocket.fidelity == ADS6_SRBM_FIDELITY_ID
    assert rocket.realization_id == ADS6_SRBM_REALIZATION_ID
    assert [channel.id for channel in rocket.channels] == ["position_ned_m", "velocity_ned_mps"]
    assert "alpha_deg" not in rocket.samples[0].values


####


def test_ads6_srbm_all_output_keeps_response_states_as_telemetry(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_srbm_configuration(
        provider,
        overrides={"end_time_s": 0.05, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-srbm-all",
            provider_id="cadac",
            provider_version="0.9.0",
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    values = result.objects[0].samples[-1].values
    assert values["phase"] == "endo_ascent"
    assert isinstance(values["alpha_deg"], float)
    assert isinstance(values["pitch_response_rate_rad_s"], float)
    assert "quaternion_wxyz" not in values
    assert "body_rates_rad_s" not in values
    assert isinstance(values["normal_acceleration_g"], float)
    assert isinstance(values["lateral_acceleration_g"], float)


####


def test_ads6_srbm_persistent_session_owns_response_state_and_native_sensor_bus(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_srbm_configuration(
        provider,
        overrides={"end_time_s": 0.04, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="ads6-srbm-persistent",
            provider_id="cadac",
            provider_version="0.9.0",
            prepared_configuration=prepared,
            seed=23,
            integration_step_s=0.01,
        )
    )

    assert descriptor.action_schema == ()
    assert descriptor.integration_step_s == 0.01
    assert descriptor.initial_observation.values["native_relative_state_track"]["valid"]
    first = provider.step_session(MissionCompositionSessionStepRequest(session_id="ads6-srbm-persistent", duration_s=0.02))

    assert first.time_start_s == 0.0
    assert first.time_end_s == 0.02
    assert first.observation.lifecycle == "active"
    assert isinstance(first.observation.values["normal_acceleration_g"], float)
    assert isinstance(first.observation.values["lateral_acceleration_g"], float)
    track = first.observation.values["native_relative_state_track"]
    assert track["sensor_id"] == "ads6-srbm-native-relative-state"
    assert track["sequence"] == 2
    assert track["payload"]["target_id"] == "ads6-srbm-target"
    assert [packet.sequence for packet in provider.session_sensor_packets("ads6-srbm-persistent")] == [0, 1, 2]

    reset = provider.reset_session(MissionCompositionResetSessionRequest(session_id="ads6-srbm-persistent"))
    assert reset.sequence == 0
    assert reset.time_s == 0.0
    assert reset.values == descriptor.initial_observation.values


####


def test_ads6_srbm_projected_events_include_exo_and_reentry_transitions(tmp_path: Path) -> None:
    provider = _provider(
        tmp_path,
        altitude_m=1_010.0,
        speed_mps=200.0,
        flight_path_deg=-80.0,
        endo_boundary_m=1_000.0,
        end_time_s=0.15,
    )
    configuration = build_default_ads6_srbm_configuration(
        provider,
        overrides={"end_time_s": 0.15, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-srbm-transition",
            provider_id="cadac",
            provider_version="0.9.0",
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    phases = tuple(event.data["phase"] for event in result.events if event.kind == "cadac-source-phase-transition")
    assert phases == ("exo_ballistic", "endo_reentry")


####


def test_ads6_srbm_provider_rejects_wrong_fidelity(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_srbm_configuration(provider)
    payload = configuration.model_dump()
    payload["fidelity"] = "point_mass_3dof"
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="requires fidelity"):
        provider.validate_configuration(wrong)
    ####


####


def test_ads6_srbm_executor_rejects_neighboring_model_request(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_srbm_configuration(provider)
    prepared = provider.validate_configuration(configuration)
    wrong = MissionCompositionRunRequest(
        request_id="ads6-srbm-wrong-provider",
        provider_id="another-provider",
        provider_version="0.9.0",
        prepared_configuration=prepared,
    )

    with pytest.raises(ValueError, match="another provider/model"):
        provider.execute_batch(wrong)
    ####


####
