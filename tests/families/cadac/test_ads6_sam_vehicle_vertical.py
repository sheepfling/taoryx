"""Focused vertical proof for the standalone CADAC ADS6 SAM plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_sam_mission_composition import (
    ADS6_SAM_FIN_PHASE_ID,
    ADS6_SAM_FIN_REALIZATION_ID,
    ADS6_SAM_MODEL_ID,
    ADS6_SAM_RCS_PHASE_ID,
    ADS6_SAM_RCS_REALIZATION_ID,
    ADS6_SAM_T3_FIDELITY_ID,
    ADS6_SAM_T4_FIDELITY_ID,
    ADS6_SAM_TVC_PHASE_ID,
    ADS6_SAM_TVC_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ads6_sam import _write_ads6_sam_case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import (
    ConfigurableTrajectoryProviderRegistry,
    ConfigurationGroupValue,
    ConfigurationNodeValue,
    ConfigurationParameterValue,
    TrajectoryConfigurationInstance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
)
from taoryx.trajectory.mission_composition import MissionCompositionOpenSessionRequest

CADAC_PROVIDER_ID = "cadac"


def _provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind precisely one standalone ADS6 SAM physical plant."""

    return CadacSourceCaseBindings(
        ads6_sam_case_path=_write_ads6_sam_case(tmp_path),
    ).build_provider(selected_model_ids=(ADS6_SAM_MODEL_ID,))
    ####


def _configuration(
    provider: CadacMissionCompositionProvider,
    *,
    phase_id: str,
    commands: dict[str, object],
) -> TrajectoryConfigurationInstance:
    """Create a catalog configuration with explicit caller-owned commands."""

    configuration = build_default_cadac_configuration(
        provider,
        ADS6_SAM_MODEL_ID,
        phase_id=phase_id,
    )
    if not isinstance(configuration.root, ConfigurationGroupValue):
        raise TypeError("ADS6 SAM configuration root must be a group")
    ####
    command_values: dict[str, ConfigurationNodeValue] = {}
    for parameter_id, value in commands.items():
        command_values[parameter_id] = ConfigurationParameterValue(
            value=value,
            unit=_command_unit(parameter_id),
        )
    ####
    root_values = dict(configuration.root.values)
    root_values["commands"] = ConfigurationGroupValue(values=command_values)
    root_values["runtime"] = ConfigurationGroupValue(
        values={
            "end_time_s": ConfigurationParameterValue(value=0.02, unit="s"),
            "sample_step_s": ConfigurationParameterValue(value=0.01, unit="s"),
        }
    )
    return configuration.model_copy(update={"root": ConfigurationGroupValue(values=root_values)})
    ####


def _command_unit(parameter_id: str) -> str | None:
    if parameter_id in {
        "roll_command_deg",
        "pitch_command_deg",
        "yaw_command_deg",
        "roll_attitude_command_deg",
        "pitch_attitude_command_deg",
        "yaw_attitude_command_deg",
        "alpha_command_deg",
        "beta_command_deg",
    }:
        return "deg"
    ####
    if parameter_id in {"lateral_acceleration_command_g", "normal_acceleration_command_g"}:
        return "g"
    ####
    return None
    ####


def test_standalone_ads6_sam_advertises_exact_batch_controls_and_explicit_session_limit(tmp_path: Path) -> None:
    """The isolated SAM exposes all three plant boundaries without a fake session."""

    provider = _provider(tmp_path)
    model = next(item for item in provider.list_models() if item.id == ADS6_SAM_MODEL_ID)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert tuple(item.id for item in provider.list_models()) == (ADS6_SAM_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    assert runner.registrations() == ((CADAC_PROVIDER_ID, ADS6_SAM_MODEL_ID),)
    assert model.common_runner_operations == ("batch", "step")
    assert [(item.id, item.fidelity_aliases) for item in model.realizations] == [
        (ADS6_SAM_FIN_REALIZATION_ID, (ADS6_SAM_T4_FIDELITY_ID,)),
        (ADS6_SAM_TVC_REALIZATION_ID, (ADS6_SAM_T4_FIDELITY_ID,)),
        (ADS6_SAM_RCS_REALIZATION_ID, (ADS6_SAM_T3_FIDELITY_ID,)),
    ]
    assert [tuple(channel.id for channel in item.controls.channels) for item in model.realizations] == [
        ("actuator.roll.command", "actuator.pitch.command", "actuator.yaw.command"),
        ("tvc.pitch.deflection", "tvc.yaw.deflection"),
        (
            "rcs.roll.attitude_command",
            "rcs.pitch.attitude_command",
            "rcs.yaw.attitude_command",
            "rcs.alpha.command",
            "rcs.beta.command",
            "rcs.lateral_acceleration.command",
            "rcs.normal_acceleration.command",
            "rcs.thrust_vector.direction",
        ),
    ]
    assert all(channel.operations == ("batch",) for item in model.realizations for channel in item.controls.channels)
    assert all(authority.command_owner == "caller" for item in model.realizations for authority in item.controls.authorities)
    assert all(authority.operations == ("batch",) for item in model.realizations for authority in item.controls.authorities)
    assert audit.status == "pass", audit.model_dump(mode="json")

    integration = provider.get_model_integration_contract(ADS6_SAM_MODEL_ID)
    assert integration.step.status == "available"
    assert integration.step.state_semantics == "core_batch_replay"
    assert integration.step.action_semantics == "read_only_replay"
    assert integration.sensor_integration.status == "available"
    assert integration.sensor_integration.sensor_bus_status == "blocked"
    assert any("standalone SAM surface owns only the physical plant" in blocker for blocker in integration.sensor_integration.blockers)
    assert integration.controller_analysis.ownership == "external_at_source_boundary"
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"

    configuration = build_default_cadac_configuration(provider, ADS6_SAM_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    with pytest.raises(ValueError, match="no installed native persistent session binding"):
        provider.open_session(
            MissionCompositionOpenSessionRequest(
                session_id="cadac-ads6-sam-must-not-fabricate-session",
                provider_id=CADAC_PROVIDER_ID,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.01,
            )
        )
    ####


@pytest.mark.parametrize(
    ("phase_id", "fidelity", "realization_id", "commands", "requested_channel", "requested_values", "realized_channel"),
    (
        (
            ADS6_SAM_FIN_PHASE_ID,
            ADS6_SAM_T4_FIDELITY_ID,
            ADS6_SAM_FIN_REALIZATION_ID,
            {
                "roll_command_deg": 1.0,
                "pitch_command_deg": 2.0,
                "yaw_command_deg": -3.0,
            },
            "requested_control_deg",
            (1.0, 2.0, -3.0),
            "achieved_control_deg",
        ),
        (
            ADS6_SAM_TVC_PHASE_ID,
            ADS6_SAM_T4_FIDELITY_ID,
            ADS6_SAM_TVC_REALIZATION_ID,
            {
                "pitch_command_deg": 2.0,
                "yaw_command_deg": -3.0,
                "tvc_mode": 2,
            },
            "requested_tvc_pitch_yaw_deg",
            (2.0, -3.0),
            "achieved_tvc_pitch_yaw_deg",
        ),
        (
            ADS6_SAM_RCS_PHASE_ID,
            ADS6_SAM_T3_FIDELITY_ID,
            ADS6_SAM_RCS_REALIZATION_ID,
            {
                "rcs_moment_mode": 21,
                "rcs_force_mode": 2,
                "roll_attitude_command_deg": 1.0,
                "pitch_attitude_command_deg": 2.0,
                "yaw_attitude_command_deg": -3.0,
                "alpha_command_deg": 1.0,
                "beta_command_deg": -1.0,
                "lateral_acceleration_command_g": 1.0,
                "normal_acceleration_command_g": -1.0,
                "thrust_vector_unit_body": (1.0, 0.0, 0.0),
            },
            "requested_rcs_acceleration_g",
            (1.0, -1.0),
            "achieved_lateral_normal_acceleration_g",
        ),
    ),
)
def test_standalone_ads6_sam_projects_each_caller_control_boundary_through_catalog_batch(
    tmp_path: Path,
    phase_id: str,
    fidelity: str,
    realization_id: str,
    commands: dict[str, object],
    requested_channel: str,
    requested_values: tuple[float, ...],
    realized_channel: str,
) -> None:
    """Every realization reaches its advertised requested/realized evidence pair."""

    provider = _provider(tmp_path)
    model = next(item for item in provider.list_models() if item.id == ADS6_SAM_MODEL_ID)
    realization = next(item for item in model.realizations if item.id == realization_id)
    mission = next(item for item in model.mission_templates if any(operation.realization_id == realization_id for operation in item.operations))
    configuration = _configuration(provider, phase_id=phase_id, commands=commands)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        ADS6_SAM_MODEL_ID,
        fidelity=fidelity,
        realization_id=realization_id,
        mission_template_id=mission.id,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "campaign_registration_required"
    assert controller["control_status"] == "available"
    assert [channel["id"] for channel in controller["channels"]] == [channel.id for channel in realization.controls.channels]
    assert all(channel["operations"] == ["batch"] for channel in controller["channels"])
    assert controller["authorities"][0]["command_owner"] == "caller"
    assert controller["authorities"][0]["operations"] == ["batch"]

    response = runner.run(
        MissionCompositionRunRequest(
            request_id=f"cadac-ads6-sam-{phase_id}-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (ADS6_SAM_MODEL_ID, fidelity, realization_id),
    ]
    values = response.result.objects[0].samples[-1].values
    assert tuple(values[requested_channel]) == pytest.approx(requested_values)
    assert any(abs(float(value)) > 0.0 for value in values[realized_channel])
    ####
