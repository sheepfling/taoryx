from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.ghame3_mission_composition import (
    GHAME3_FIDELITY_ID,
    GHAME3_MODEL_ID,
    CadacGhame3MissionCompositionProvider,
    build_default_ghame3_configuration,
    register_ghame3_mission_composition,
)
from taoryx.families.cadac.ghame3_plugin import Ghame3VehiclePlugin
from test_ghame3 import _case

from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunnerRegistry, MissionCompositionRunRequest


def _provider(tmp_path: Path) -> CadacGhame3MissionCompositionProvider:
    return CadacGhame3MissionCompositionProvider(Ghame3VehiclePlugin(_case(tmp_path)))


####


def test_ghame3_provider_is_unambiguously_point_mass(tmp_path: Path) -> None:
    model = _provider(tmp_path).list_models()[0]
    assert model.id == GHAME3_MODEL_ID
    assert model.common_runner_operations == ("batch",)
    assert len(model.fidelities) == 1
    assert model.fidelities[0].id == GHAME3_FIDELITY_ID
    assert model.fidelities[0].dynamics_fidelity == "point_mass_3dof"
    assert model.fidelities[0].control_realization == "force_model"


####


def test_ghame3_common_runner_returns_point_mass_truth_and_events(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ghame3_configuration(provider, overrides={"end_time_s": 0.05, "sample_step_s": 0.01})
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ghame3_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="ghame3-smoke",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )
    result = registry.run(request).result
    assert result.primary_model_id == GHAME3_MODEL_ID
    vehicle = result.objects[0]
    assert vehicle.fidelity == "point_mass_3dof"
    assert {channel.id for channel in vehicle.channels} >= {"longitude_deg", "mach", "alpha_deg", "thrust_n"}
    assert len(result.events) == 1
    assert result.events[0].time_s == 0.03


####
