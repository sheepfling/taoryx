from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_aircraft_mission_composition import (
    ADS6_AIRCRAFT_FIDELITY_ID,
    ADS6_AIRCRAFT_MODEL_ID,
    ADS6_AIRCRAFT_MODEL_VERSION,
    ADS6_AIRCRAFT_REALIZATION_ID,
    CadacAds6AircraftMissionCompositionProvider,
    build_default_ads6_aircraft_configuration,
    register_ads6_aircraft_mission_composition,
)
from taoryx.families.cadac.ads6_aircraft_plugin import Ads6AircraftVehiclePlugin
from test_ads6_aircraft import _write_ads6_aircraft_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(
    tmp_path: Path,
    **case_options: object,
) -> CadacAds6AircraftMissionCompositionProvider:
    return CadacAds6AircraftMissionCompositionProvider(Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path, **case_options)))


####


def test_ads6_aircraft_provider_advertises_exact_t1_force_model(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == ADS6_AIRCRAFT_MODEL_ID
    assert model.version == ADS6_AIRCRAFT_MODEL_VERSION
    assert model.common_runner_operations == ("batch", "step")
    assert [item.id for item in model.fidelities] == [ADS6_AIRCRAFT_FIDELITY_ID]
    assert model.fidelities[0].runtime_fidelity == "point_mass_3dof"
    assert model.fidelities[0].control_realization == "force_model"
    assert [(item.id, item.status) for item in model.realizations] == [(ADS6_AIRCRAFT_REALIZATION_ID, "available")]
    assert model.presentation.properties[0].value == "AIRCRAFT3"


####


def test_ads6_aircraft_common_runner_keeps_only_translation_as_core_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_aircraft_configuration(
        provider,
        overrides={"end_time_s": 0.05, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ads6_aircraft_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id="ads6-aircraft-core",
            provider_id="cadac",
            provider_version=ADS6_AIRCRAFT_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )
    result = response.result
    aircraft = result.objects[0]

    assert result.status == "completed"
    assert result.primary_model_id == ADS6_AIRCRAFT_MODEL_ID
    assert aircraft.fidelity == ADS6_AIRCRAFT_FIDELITY_ID
    assert aircraft.realization_id == ADS6_AIRCRAFT_REALIZATION_ID
    assert [channel.id for channel in aircraft.channels] == ["position_ned_m", "velocity_ned_mps"]
    assert "bank_deg" not in aircraft.samples[0].values


####


def test_ads6_aircraft_all_output_keeps_bank_and_load_as_telemetry(tmp_path: Path) -> None:
    provider = _provider(
        tmp_path,
        guidance_option=1,
        turn_load_g=1.5,
        maneuver_start_s=0.0,
        maneuver_stop_s=0.2,
        bank_time_constant_s=0.02,
        load_factor_time_constant_s=0.02,
    )
    configuration = build_default_ads6_aircraft_configuration(
        provider,
        overrides={"end_time_s": 0.1, "sample_step_s": 0.05},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-aircraft-all",
            provider_id="cadac",
            provider_version=ADS6_AIRCRAFT_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    values = result.objects[0].samples[-1].values
    assert isinstance(values["bank_deg"], float)
    assert isinstance(values["normal_load_factor_g"], float)
    assert "quaternion_wxyz" not in values
    assert "body_rates_rad_s" not in values
    assert any(event.kind == "cadac-aircraft-maneuver-window" for event in result.events)


####


def test_ads6_aircraft_provider_requires_explicit_escape_track(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_aircraft_configuration(
        provider,
        overrides={
            "guidance_option": 2,
            "guidance_gain": 1.0,
            "maneuver_start_s": 0.0,
            "maneuver_stop_s": 1.0,
        },
    )

    with pytest.raises(ValueError, match="requires threat_track_enabled"):
        provider.validate_configuration(configuration)
    ####
    valid = build_default_ads6_aircraft_configuration(
        provider,
        overrides={
            "guidance_option": 2,
            "guidance_gain": 1.0,
            "maneuver_start_s": 0.0,
            "maneuver_stop_s": 1.0,
            "threat_track_enabled": True,
            "threat_north_m": 1_000.0,
            "threat_east_m": 0.0,
            "threat_down_m": -1_000.0,
            "threat_velocity_north_mps": 100.0,
            "threat_velocity_east_mps": 20.0,
            "threat_velocity_down_mps": 0.0,
            "end_time_s": 0.05,
        },
    )
    provider.validate_configuration(valid)


####


def test_ads6_aircraft_provider_rejects_wrong_fidelity_and_neighbor_request(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_aircraft_configuration(provider)
    payload = configuration.model_dump()
    payload["fidelity"] = "pseudo_6dof"
    wrong_fidelity = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="requires fidelity"):
        provider.validate_configuration(wrong_fidelity)
    ####
    prepared = provider.validate_configuration(configuration)
    wrong_request = MissionCompositionRunRequest(
        request_id="ads6-aircraft-wrong-provider",
        provider_id="another-provider",
        provider_version=ADS6_AIRCRAFT_MODEL_VERSION,
        prepared_configuration=prepared,
    )
    with pytest.raises(ValueError, match="another provider/model"):
        provider.execute_batch(wrong_request)
    ####


####
