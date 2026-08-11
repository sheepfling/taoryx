from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.rocket6g_mission_composition import (
    ROCKET6G_FIDELITY_ID,
    ROCKET6G_MODEL_ID,
    CadacRocket6gMissionCompositionProvider,
    build_default_rocket6g_configuration,
    register_rocket6g_mission_composition,
)
from taoryx.families.cadac.rocket6g_plugin import Rocket6gVehiclePlugin
from test_rocket6g import _write_rocket_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(tmp_path: Path) -> CadacRocket6gMissionCompositionProvider:
    return CadacRocket6gMissionCompositionProvider(Rocket6gVehiclePlugin(_write_rocket_case(tmp_path)))


####


def test_rocket6g_provider_advertises_phase_aware_mixed_effector_6dof(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == ROCKET6G_MODEL_ID
    assert model.common_runner_operations == ("batch",)
    assert model.fidelities[0].id == ROCKET6G_FIDELITY_ID
    assert model.fidelities[0].dynamics_fidelity == "rigid_body_6dof"
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("mixed",)
    assert model.capabilities.supports_staging
    assert model.capabilities.supports_multiple_stages
    assert not model.output_schema.entity_output.supports_dynamic_spawning
    controls = model.realizations[0].controls
    assert controls.status == "available"
    assert tuple(item.id for item in controls.channels) == (
        "tvc.pitch.deflection",
        "tvc.yaw.deflection",
        "rcs.thrust_vector.direction",
        "rcs.roll.attitude_command",
        "rcs.pitch.attitude_command",
        "rcs.yaw.attitude_command",
    )
    assert {item.canonical_unit for item in controls.channels if item.id.startswith("tvc.")} == {"deg"}
    assert next(item for item in controls.channels if item.id == "rcs.thrust_vector.direction").value_space.topology == "unit_direction"
    assert controls.rl_action_space(operation="batch").kind == "box"


####


def test_rocket6g_common_runner_returns_phase_and_stage_telemetry(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_rocket6g_configuration(
        provider,
        overrides={
            "tvc_pitch_command_deg": 1.0,
            "thrust_vector_unit_body": (1.0, 0.01, -0.01),
            "enable_boost_cutoff": True,
            "boost_cutoff_time_s": 4.2,
            "end_time_s": 4.8,
            "sample_step_s": 0.1,
        },
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_rocket6g_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id="rocket6g-phase-smoke",
            provider_id="cadac",
            provider_version="0.5.0",
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    result = response.result
    vehicle = result.objects[0]
    phases = {sample.values["source_phase"] for sample in vehicle.samples}
    runtime_fidelities = {sample.values["runtime_fidelity"] for sample in vehicle.samples}
    stages = {sample.values["active_stage"] for sample in vehicle.samples}

    assert result.primary_model_id == ROCKET6G_MODEL_ID
    assert vehicle.fidelity == ROCKET6G_FIDELITY_ID
    assert phases >= {"aggregate_rcs", "mixed_tvc_rcs"}
    assert runtime_fidelities >= {
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    assert stages == {1, 2, 3}
    assert tuple(event.kind for event in result.events) == tuple(f"source_event_{index}" for index in range(5))
    assert vehicle.samples[-1].values["propulsion_mode"] == 0


####


def test_rocket6g_core_selection_keeps_rigid_body_truth_only(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_rocket6g_configuration(
        provider,
        overrides={"end_time_s": 0.2, "sample_step_s": 0.1},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="rocket6g-core",
            provider_id="cadac",
            provider_version="0.5.0",
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


####


def test_rocket6g_source_program_is_not_selectable_as_one_isolated_phase(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_rocket6g_configuration(provider)
    payload = configuration.model_dump()
    payload["realization_id"] = "cadac-source-phase.aggregate_rcs"
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="supports only realization"):
        provider.validate_configuration(wrong)
    ####


####
