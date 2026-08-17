from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.agm6_mission_composition import (
    AGM6_AIRCRAFT_MODEL_ID,
    AGM6_FIDELITY_ID,
    AGM6_MISSION_ID,
    AGM6_MODEL_ID,
    AGM6_REALIZATION_ID,
    AGM6_TARGET_MODEL_ID,
    CadacAgm6MissionCompositionProvider,
    build_default_agm6_configuration,
    register_agm6_mission_composition,
)
from taoryx.families.cadac.agm6_plugin import Agm6PluginOverrides, Agm6VehiclePlugin
from test_agm6 import _write_agm6_case

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


def _provider(tmp_path: Path) -> CadacAgm6MissionCompositionProvider:
    return CadacAgm6MissionCompositionProvider(Agm6VehiclePlugin(_write_agm6_case(tmp_path)))


####


def test_agm6_provider_advertises_one_available_physical_fin_realization(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == AGM6_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == AGM6_FIDELITY_ID
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("aerodynamic_surfaces",)
    assert [(item.id, item.status) for item in model.realizations] == [
        (AGM6_REALIZATION_ID, "available"),
    ]
    assert model.mission_templates[0].id == AGM6_MISSION_ID


####


def test_agm6_common_runner_returns_three_independent_root_objects(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(
        provider,
        overrides={"end_time_s": 0.12, "sample_step_s": 0.02, "random_seed": 13},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_agm6_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="agm6-smoke",
        provider_id="cadac",
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )

    result = registry.run(request).result

    assert result.primary_model_id == AGM6_MODEL_ID
    assert [item.model_id for item in result.objects] == [
        AGM6_MODEL_ID,
        AGM6_TARGET_MODEL_ID,
        AGM6_AIRCRAFT_MODEL_ID,
    ]
    assert all(item.parent_object_id is None for item in result.objects)
    assert [item.fidelity for item in result.objects] == [
        AGM6_FIDELITY_ID,
        "point_mass_3dof",
        "point_mass_3dof",
    ]
    assert {item.id for item in result.objects[0].channels} >= {
        "position_ned_m",
        "quaternion_wxyz",
        "requested_fins_deg",
        "achieved_fins_deg",
        "datalink_track_sequence",
        "ins_model_effective",
    }
    assert {item.id for item in result.objects[1].channels} >= {
        "position_ned_m",
        "heading_deg",
        "normal_load_g",
    }
    assert {item.id for item in result.objects[2].channels} >= {
        "position_ned_m",
        "bank_deg",
        "normal_load_g",
    }
    assert any(event.kind == "cadac-target-track-update" for event in result.events)
    assert {diagnostic.code for diagnostic in result.diagnostics} >= {
        "cadac-parity-pending",
        "cadac-iir-sensor-partial",
        "cadac-ins-truth-aligned-substitution",
    }


####


def test_agm6_core_selection_keeps_rigid_body_and_both_point_mass_truths(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(
        provider,
        overrides={"end_time_s": 0.02, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="agm6-core",
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
    assert [item.id for item in result.objects[2].channels] == [
        "position_ned_m",
        "velocity_ned_mps",
    ]


####


def test_agm6_persistent_session_owns_three_actor_sensor_and_control_state(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(
        provider,
        overrides={"end_time_s": 0.04, "sample_step_s": 0.01, "random_seed": 17},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="agm6-persistent",
            provider_id="cadac",
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            seed=31,
            integration_step_s=0.001,
        )
    )

    assert descriptor.action_schema == ()
    assert descriptor.initial_observation.values["native_relative_state_track"]["valid"]
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="agm6-persistent", duration_s=0.002))

    assert step.time_start_s == 0.0
    assert step.time_end_s == 0.002
    assert step.observation.lifecycle == "active"
    assert step.observation.values["source_sensor_and_datalink_state"]["source_schedule"] == "missile_then_target_then_tracking_aircraft"
    assert isinstance(step.observation.values["normal_acceleration_g"], float)
    assert len(step.observation.values["requested_fins_deg"]) == 4
    assert len(step.observation.values["achieved_fins_deg"]) == 4
    track = step.observation.values["native_relative_state_track"]
    assert track["sensor_id"] == "agm6-native-relative-state"
    assert track["sequence"] == 2
    assert track["payload"]["target_id"] == "agm6-target"
    assert [packet.sequence for packet in provider.session_sensor_packets("agm6-persistent")] == [0, 1, 2]

    reset = provider.reset_session(MissionCompositionResetSessionRequest(session_id="agm6-persistent"))
    assert reset.sequence == 0
    assert reset.time_s == 0.0
    assert reset.values == descriptor.initial_observation.values


####


def test_agm6_default_configuration_routes_target_aircraft_seed_and_runtime_overrides(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(
        provider,
        overrides={
            "target_lateral_acceleration_g": 0.4,
            "aircraft_option": 1,
            "aircraft_turn_g": 2.0,
            "random_seed": 99,
            "end_time_s": 0.05,
            "sample_step_s": 0.01,
        },
    )
    prepared = provider.validate_configuration(configuration)

    assert prepared.resolved["target"]["lateral_acceleration_g"] == 0.4
    assert prepared.resolved["aircraft"]["aircraft_option"] == 1
    assert prepared.resolved["aircraft"]["turn_g"] == 2.0
    assert prepared.resolved["stochastic"]["random_seed"] == 99


####


def test_agm6_configuration_routes_public_tuning_into_the_plugin(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(
        provider,
        overrides={
            "fin_position_limit_deg": 12.0,
            "fin_rate_limit_deg_s": 310.0,
            "fin_natural_frequency_rad_s": 220.0,
            "fin_damping_ratio": 0.95,
            "seeker_acquisition_range_m": 7_000.0,
            "seeker_filter_gain_per_s": 2.8,
            "seeker_filter_natural_frequency_rad_s": 17.0,
            "seeker_filter_damping_ratio": 0.8,
            "structural_limit_g": 16.0,
            "propulsion_throttle": 0.75,
            "end_time_s": 0.01,
            "sample_step_s": 0.005,
        },
    )
    prepared = provider.validate_configuration(configuration)
    captured: list[Agm6PluginOverrides] = []
    original = provider._plugin.run_batch

    def capture(overrides: Agm6PluginOverrides | None = None) -> object:
        assert overrides is not None
        captured.append(overrides)
        return original(overrides)

    monkeypatch.setattr(provider._plugin, "run_batch", capture)
    provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="agm6-tuning",
            provider_id="cadac",
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert prepared.resolved["actuation"]["fin_position_limit_deg"] == 12.0
    assert prepared.resolved["seeker"]["acquisition_range_m"] == 7_000.0
    assert captured[0].fin_rate_limit_deg_s == 310.0
    assert captured[0].seeker_filter_natural_frequency_rad_s == 17.0
    assert captured[0].structural_limit_g == 16.0
    assert captured[0].propulsion_throttle == 0.75


####


def test_agm6_rejects_non_exact_realization(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_agm6_configuration(provider)
    payload = configuration.model_dump()
    payload["realization_id"] = "cadac-nearest-fallback"
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="supports only realization"):
        provider.validate_configuration(wrong)
    ####


####
