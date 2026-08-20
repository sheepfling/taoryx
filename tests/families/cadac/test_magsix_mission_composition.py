from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.magsix_mission_composition import (
    MAGSIX_ATTITUDE_FIDELITY_ID,
    MAGSIX_ATTITUDE_PHASE_ID,
    MAGSIX_MODEL_ID,
    MAGSIX_TRAJECTORY_FIDELITY_ID,
    CadacMagsixMissionCompositionProvider,
    build_default_magsix_configuration,
    register_magsix_mission_composition,
)
from taoryx.families.cadac.magsix_plugin import MagsixVehiclePlugin
from test_magsix import _case

from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunnerRegistry, MissionCompositionRunRequest


def _provider(tmp_path: Path) -> CadacMagsixMissionCompositionProvider:
    return CadacMagsixMissionCompositionProvider(MagsixVehiclePlugin(_case(tmp_path)))


####


def test_magsix_provider_exposes_t1_runtime_and_t2_validation(tmp_path: Path) -> None:
    model = _provider(tmp_path).list_models()[0]
    assert model.id == MAGSIX_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert tuple(item.id for item in model.fidelities) == (MAGSIX_TRAJECTORY_FIDELITY_ID, MAGSIX_ATTITUDE_FIDELITY_ID)
    assert model.realizations[0].status == "available"
    assert model.realizations[1].status == "blocked"


####


def test_magsix_common_runner_returns_trajectory_only_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_magsix_configuration(
        provider,
        overrides={"end_time_dnt": 0.05, "sample_step_dnt": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_magsix_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="magsix-smoke",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )
    result = registry.run(request).result
    assert result.primary_model_id == MAGSIX_MODEL_ID
    rotor = result.objects[0]
    assert rotor.fidelity == "point_mass_3dof"
    assert {channel.id for channel in rotor.channels} >= {"position_ned_m", "spin_rpm", "source_time_dnt"}
    assert len(rotor.samples) > 1


####


def test_magsix_restricted_attitude_is_validate_only(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_magsix_configuration(provider, phase_id=MAGSIX_ATTITUDE_PHASE_ID)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == MAGSIX_ATTITUDE_FIDELITY_ID
    request = MissionCompositionRunRequest(
        request_id="magsix-attitude",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
    )
    with pytest.raises(ValueError, match="validation-only"):
        provider.execute_batch(request)
    ####


####
