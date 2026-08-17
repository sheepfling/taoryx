from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.aim5_mission_composition import (
    AIM5_MODEL_ID,
    AIM5_TARGET_MODEL_ID,
    CadacAim5MissionCompositionProvider,
    build_default_aim5_configuration,
    register_aim5_mission_composition,
)
from taoryx.families.cadac.aim5_plugin import Aim5PluginOverrides, Aim5VehiclePlugin
from test_aim5 import AERO, INPUT, PROP

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


def _provider(tmp_path: Path) -> CadacAim5MissionCompositionProvider:
    path = tmp_path / "input.asc"
    path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return CadacAim5MissionCompositionProvider(Aim5VehiclePlugin(path))


####


def test_aim5_provider_advertises_exact_batch_model(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert provider.metadata.id == "cadac"
    assert model.id == AIM5_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == "pseudo_6dof"
    assert model.fidelities[0].promotion_status == "development"
    assert model.realizations[0].input_realization == "provider_defined"
    assert "step" in model.realizations[0].operations
    assert {channel.id for channel in model.output_schema.telemetry_channels} >= {
        "unit_los_vehicle",
        "line_of_sight_rate_vehicle_rad_s",
        "normal_acceleration_g",
        "lateral_acceleration_g",
    }


####


def test_aim5_common_runner_returns_missile_and_target_as_independent_root_objects(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_aim5_configuration(
        provider,
        overrides={"end_time_s": 0.2, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_aim5_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="aim5-smoke",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )

    result = registry.run(request).result

    assert result.primary_model_id == AIM5_MODEL_ID
    assert [item.model_id for item in result.objects] == [AIM5_MODEL_ID, AIM5_TARGET_MODEL_ID]
    assert all(item.parent_object_id is None for item in result.objects)
    assert result.objects[0].fidelity == "pseudo_6dof"
    assert result.objects[1].fidelity == "point_mass_3dof"
    assert {item.id for item in result.objects[0].channels} >= {"position_ned_m", "range_m", "mass_kg"}
    assert {item.id for item in result.objects[1].channels} >= {"position_ned_m", "heading_deg"}


####


def test_aim5_core_output_selection_keeps_target_as_separate_entity(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_aim5_configuration(
        provider,
        overrides={"end_time_s": 0.05, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="aim5-core",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )

    result = provider.execute_batch(request)

    assert [item.id for item in result.objects[0].channels] == ["position_ned_m", "velocity_ned_mps"]
    assert [item.id for item in result.objects[1].channels] == ["position_ned_m", "velocity_ned_mps"]


####


def test_aim5_configuration_routes_pseudo6_response_law_tuning_into_the_plugin(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_aim5_configuration(
        provider,
        overrides={
            "alpha_max_deg": 30.0,
            "rate_loop_time_constant_s": 0.075,
            "proportional_integral_ratio": 1.2,
            "acceleration_loop_gain_rad_s2": 48.0,
            "end_time_s": 0.05,
            "sample_step_s": 0.05,
        },
    )
    prepared = provider.validate_configuration(configuration)
    captured: list[Aim5PluginOverrides] = []
    original = provider._plugin.run_batch

    def capture(overrides: Aim5PluginOverrides | None = None) -> object:
        assert overrides is not None
        captured.append(overrides)
        return original(overrides)

    monkeypatch.setattr(provider._plugin, "run_batch", capture)
    provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="aim5-response-law-tuning",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert prepared.resolved["response_law"]["alpha_max_deg"] == 30.0
    assert captured[0].rate_loop_time_constant_s == 0.075
    assert captured[0].proportional_integral_ratio == 1.2
    assert captured[0].acceleration_loop_gain_rad_s2 == 48.0


####


def test_aim5_persistent_session_owns_source_state_and_native_sensor_bus(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_aim5_configuration(
        provider,
        overrides={"end_time_s": 0.04, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="aim5-persistent",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            seed=41,
            integration_step_s=0.01,
        )
    )

    assert descriptor.action_schema == ()
    assert descriptor.integration_step_s == 0.01
    assert descriptor.initial_observation.values["native_relative_state_track"]["valid"]
    first = provider.step_session(MissionCompositionSessionStepRequest(session_id="aim5-persistent", duration_s=0.02))

    assert first.time_start_s == 0.0
    assert first.time_end_s == 0.02
    assert first.observation.lifecycle == "active"
    assert first.observation.values["source_seeker_state"]["initialized"]
    track = first.observation.values["native_relative_state_track"]
    assert track["sensor_id"] == "aim5-native-relative-state"
    assert track["sequence"] == 2
    assert track["payload"]["target_id"] == "aim5-target"
    assert [packet.sequence for packet in provider.session_sensor_packets("aim5-persistent")] == [0, 1, 2]

    reset = provider.reset_session(MissionCompositionResetSessionRequest(session_id="aim5-persistent"))
    assert reset.sequence == 0
    assert reset.time_s == 0.0
    assert reset.values == descriptor.initial_observation.values


####
