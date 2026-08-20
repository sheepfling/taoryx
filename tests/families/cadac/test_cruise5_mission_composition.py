from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.cruise5_mission_composition import (
    CRUISE5_FIDELITY_ID,
    CRUISE5_MODEL_ID,
    CRUISE5_TRANSLATION_FIDELITY_ID,
    CadacCruise5MissionCompositionProvider,
    build_default_cruise5_configuration,
    register_cruise5_mission_composition,
)
from taoryx.families.cadac.cruise5_plugin import Cruise5VehiclePlugin
from test_cruise5 import _case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(tmp_path: Path) -> CadacCruise5MissionCompositionProvider:
    return CadacCruise5MissionCompositionProvider(Cruise5VehiclePlugin(_case(tmp_path)))


####


def test_cruise5_provider_advertises_pseudo_and_validate_only_translation_tiers(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]
    fidelity_by_id = {item.id: item for item in model.fidelities}
    realization_by_id = {item.id: item for item in model.realizations}
    assert model.id == CRUISE5_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert fidelity_by_id[CRUISE5_FIDELITY_ID].dynamics_fidelity == "pseudo_6dof"
    assert fidelity_by_id[CRUISE5_FIDELITY_ID].operations == ("validate", "batch", "step")
    assert fidelity_by_id[CRUISE5_TRANSLATION_FIDELITY_ID].operations == ("validate",)
    assert realization_by_id["cadac-source-compatibility"].status == "available"
    assert realization_by_id["cadac-source-phase.translation_only"].status == "blocked"


####


def test_cruise5_common_runner_returns_round_earth_pseudo_vehicle(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_cruise5_configuration(
        provider,
        overrides={"end_time_s": 1.0, "sample_step_s": 0.5},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_cruise5_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="cruise5-smoke",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )
    result = registry.run(request).result
    assert result.primary_model_id == CRUISE5_MODEL_ID
    assert len(result.objects) == 1
    vehicle = result.objects[0]
    assert vehicle.fidelity == CRUISE5_FIDELITY_ID
    assert [sample.time_s for sample in vehicle.samples] == pytest.approx((0.0, 0.5, 1.0))
    assert {channel.id for channel in vehicle.channels} >= {
        "longitude_deg",
        "velocity_geographic_mps",
        "mach",
        "thrust_n",
        "alpha_deg",
        "bank_deg",
        "waypoint_ground_range_m",
    }
    assert vehicle.samples[0].values["mach"] == pytest.approx(0.772811, rel=3.0e-5)


####


def test_cruise5_core_selection_keeps_geodetic_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_cruise5_configuration(
        provider,
        overrides={"end_time_s": 0.05, "sample_step_s": 0.5},
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="cruise5-core",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )
    result = provider.execute_batch(request)
    assert [channel.id for channel in result.objects[0].channels] == [
        "longitude_deg",
        "latitude_deg",
        "altitude_m",
        "velocity_geographic_mps",
    ]


####


def test_cruise5_translation_phase_validates_but_cannot_execute(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_cruise5_configuration(provider, phase_id="translation_only")
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == CRUISE5_TRANSLATION_FIDELITY_ID
    request = MissionCompositionRunRequest(
        request_id="cruise5-t1",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
    )
    with pytest.raises(ValueError, match="validation-only"):
        provider.execute_batch(request)
    ####


####
