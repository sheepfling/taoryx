from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.falcon6_mission_composition import (
    FALCON6_FIDELITY_ID,
    FALCON6_MODEL_ID,
    CadacFalcon6MissionCompositionProvider,
    build_default_falcon6_configuration,
    register_falcon6_mission_composition,
)
from taoryx.families.cadac.falcon6_plugin import Falcon6VehiclePlugin
from test_falcon6 import _write_falcon_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(tmp_path: Path) -> CadacFalcon6MissionCompositionProvider:
    return CadacFalcon6MissionCompositionProvider(Falcon6VehiclePlugin(_write_falcon_case(tmp_path)))


####


def test_falcon6_provider_advertises_physical_surface_6dof(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]
    assert model.id == FALCON6_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == FALCON6_FIDELITY_ID
    assert model.fidelities[0].dynamics_fidelity == "rigid_body_6dof"
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("aerodynamic_surfaces",)
    assert model.realizations[0].status == "available"
    controls = model.realizations[0].controls
    assert controls.status == "available"
    assert tuple(item.id for item in controls.channels) == (
        "actuator.aileron.deflection",
        "actuator.elevator.deflection",
        "actuator.rudder.deflection",
    )
    assert {item.canonical_unit for item in controls.channels} == {"deg"}
    assert all(item.operations == ("batch",) for item in controls.channels)
    assert [item.native_binding.provider_binding["configuration_path"] for item in controls.channels if item.native_binding] == [
        ["physical_controls", "aileron_command_deg"],
        ["physical_controls", "elevator_command_deg"],
        ["physical_controls", "rudder_command_deg"],
    ]
    assert controls.rl_action_space(operation="batch").kind == "box"


####


def test_falcon6_common_runner_returns_one_physical_aircraft(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_falcon6_configuration(
        provider,
        overrides={
            "elevator_command_deg": 5.0,
            "end_time_s": 0.05,
            "sample_step_s": 0.01,
        },
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_falcon6_mission_composition(provider, registry)
    request = MissionCompositionRunRequest(
        request_id="falcon6-smoke",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all"),
    )
    response = registry.run(request)
    assert response.kind == "trajectory", response.model_dump(mode="json")
    result = response.result
    assert result.primary_model_id == FALCON6_MODEL_ID
    assert len(result.objects) == 1
    aircraft = result.objects[0]
    assert aircraft.fidelity == FALCON6_FIDELITY_ID
    assert {channel.id for channel in aircraft.channels} >= {
        "position_ned_m",
        "quaternion_wxyz",
        "requested_surfaces_deg",
        "achieved_surfaces_deg",
        "surface_position_limited",
        "surface_rate_limited",
        "moment_body_nm",
    }
    assert aircraft.samples[-1].values["requested_surfaces_deg"][1] == 5.0


####


def test_falcon6_core_selection_keeps_rigid_body_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_falcon6_configuration(
        provider,
        overrides={"end_time_s": 0.01, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id="falcon6-core",
        provider_id="cadac",
        provider_version="0.5.0",
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )
    result = provider.execute_batch(request)
    assert [channel.id for channel in result.objects[0].channels] == [
        "position_ned_m",
        "velocity_ned_mps",
        "quaternion_wxyz",
        "body_rates_rad_s",
    ]


####
