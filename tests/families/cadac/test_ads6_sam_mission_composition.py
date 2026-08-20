from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_sam_mission_composition import (
    ADS6_SAM_FIN_REALIZATION_ID,
    ADS6_SAM_MODEL_ID,
    ADS6_SAM_MODEL_VERSION,
    ADS6_SAM_RCS_REALIZATION_ID,
    ADS6_SAM_T3_FIDELITY_ID,
    ADS6_SAM_T4_FIDELITY_ID,
    ADS6_SAM_TVC_REALIZATION_ID,
    CadacAds6SamMissionCompositionProvider,
    build_default_ads6_sam_configuration,
    register_ads6_sam_mission_composition,
)
from taoryx.families.cadac.ads6_sam_plugin import Ads6SamPluginOverrides, Ads6SamVehiclePlugin
from test_ads6_sam import _write_ads6_sam_case

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
)


def _provider(tmp_path: Path) -> CadacAds6SamMissionCompositionProvider:
    return CadacAds6SamMissionCompositionProvider(Ads6SamVehiclePlugin(_write_ads6_sam_case(tmp_path)))


####


def test_ads6_sam_provider_advertises_three_exact_realizations(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == ADS6_SAM_MODEL_ID
    assert model.common_runner_operations == ("batch", "step")
    assert [item.id for item in model.fidelities] == [ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_T3_FIDELITY_ID]
    assert [(item.id, item.status) for item in model.realizations] == [
        (ADS6_SAM_FIN_REALIZATION_ID, "available"),
        (ADS6_SAM_TVC_REALIZATION_ID, "available"),
        (ADS6_SAM_RCS_REALIZATION_ID, "available"),
    ]
    assert model.realizations[0].actuator_types == ("aerodynamic_surfaces",)
    assert model.realizations[1].actuator_types == ("thrust_vectoring",)
    assert model.realizations[2].input_realization == "direct_wrench"
    fin_controls, tvc_controls, rcs_controls = (item.controls for item in model.realizations)
    assert fin_controls.status == tvc_controls.status == rcs_controls.status == "available"
    assert tuple(item.id for item in fin_controls.channels) == (
        "actuator.roll.command",
        "actuator.pitch.command",
        "actuator.yaw.command",
    )
    assert tuple(item.id for item in tvc_controls.channels) == (
        "tvc.pitch.deflection",
        "tvc.yaw.deflection",
    )
    assert {item.canonical_unit for item in (*fin_controls.channels, *tvc_controls.channels)} == {"deg"}
    assert {
        item.id: item.canonical_unit for item in rcs_controls.channels if item.id in {"rcs.lateral_acceleration.command", "rcs.normal_acceleration.command"}
    } == {
        "rcs.lateral_acceleration.command": "g",
        "rcs.normal_acceleration.command": "g",
    }
    thrust_vector = next(item for item in rcs_controls.channels if item.id == "rcs.thrust_vector.direction")
    assert thrust_vector.shape == (3,)
    assert thrust_vector.value_space.topology == "unit_direction"
    assert rcs_controls.rl_action_space(operation="batch").kind == "box"


####


@pytest.mark.parametrize(
    ("phase", "expected_fidelity", "expected_realization"),
    (
        ("fin_control", ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_FIN_REALIZATION_ID),
        ("tvc_control", ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_TVC_REALIZATION_ID),
        ("aggregate_rcs", ADS6_SAM_T3_FIDELITY_ID, ADS6_SAM_RCS_REALIZATION_ID),
    ),
)
def test_ads6_sam_common_runner_preserves_exact_phase_fidelity(
    tmp_path: Path,
    phase: str,
    expected_fidelity: str,
    expected_realization: str,
) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_sam_configuration(
        provider,
        phase_id=phase,
        overrides={"end_time_s": 0.01, "sample_step_s": 0.005},
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ads6_sam_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id=f"ads6-sam-{phase}",
            provider_id="cadac",
            provider_version=ADS6_SAM_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    result = response.result
    vehicle = result.objects[0]

    assert result.status == "completed"
    assert result.primary_model_id == ADS6_SAM_MODEL_ID
    assert len(result.objects) == 1
    assert vehicle.parent_object_id is None
    assert vehicle.fidelity == expected_fidelity
    assert vehicle.realization_id == expected_realization
    assert {sample.values["source_phase"] for sample in vehicle.samples} == {phase}
    assert {sample.values["fidelity"] for sample in vehicle.samples} == {expected_fidelity}


####


def test_ads6_sam_core_selection_keeps_full_rigid_body_truth(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_sam_configuration(
        provider,
        overrides={"end_time_s": 0.01, "sample_step_s": 0.005},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-sam-core",
            provider_id="cadac",
            provider_version=ADS6_SAM_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert [channel.id for channel in result.objects[0].channels] == [
        "position_ned_m",
        "velocity_ned_mps",
        "quaternion_wxyz",
        "body_rates_rad_s",
    ]


####


def test_ads6_sam_configuration_routes_public_effector_tuning_into_the_plugin(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_sam_configuration(
        provider,
        overrides={
            "fin_position_limit_deg": 14.0,
            "fin_rate_limit_deg_s": 320.0,
            "fin_natural_frequency_rad_s": 225.0,
            "fin_damping_ratio": 0.8,
            "tvc_position_limit_deg": 6.0,
            "tvc_rate_limit_deg_s": 160.0,
            "tvc_natural_frequency_rad_s": 90.0,
            "tvc_damping_ratio": 0.9,
            "tvc_initial_gain": 0.6,
            "end_time_s": 0.01,
            "sample_step_s": 0.005,
        },
    )
    prepared = provider.validate_configuration(configuration)
    captured: list[Ads6SamPluginOverrides] = []
    original = provider._plugin.run_batch

    def capture(overrides: Ads6SamPluginOverrides | None = None) -> object:
        assert overrides is not None
        captured.append(overrides)
        return original(overrides)

    monkeypatch.setattr(provider._plugin, "run_batch", capture)
    provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-sam-tuning",
            provider_id="cadac",
            provider_version=ADS6_SAM_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert prepared.resolved["actuation"]["fin_position_limit_deg"] == 14.0
    assert captured[0].fin_rate_limit_deg_s == 320.0
    assert captured[0].tvc_natural_frequency_rad_s == 90.0
    assert captured[0].tvc_initial_gain == 0.6


####


def test_ads6_sam_rejects_phase_fidelity_mismatch(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_sam_configuration(provider, phase_id="aggregate_rcs")
    payload = configuration.model_dump()
    payload["fidelity"] = ADS6_SAM_T4_FIDELITY_ID
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="requires fidelity"):
        provider.validate_configuration(wrong)
    ####


####
