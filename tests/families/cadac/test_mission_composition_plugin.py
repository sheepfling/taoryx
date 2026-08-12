from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_aircraft_plugin import Ads6AircraftVehiclePlugin
from taoryx.families.cadac.ads6_engagement_plugin import Ads6EngagementPlugin
from taoryx.families.cadac.ads6_sam_plugin import Ads6SamVehiclePlugin
from taoryx.families.cadac.ads6_srbm_plugin import Ads6SrbmVehiclePlugin
from taoryx.families.cadac.agm6_plugin import Agm6VehiclePlugin
from taoryx.families.cadac.aim5_plugin import Aim5VehiclePlugin
from taoryx.families.cadac.cruise5_plugin import Cruise5VehiclePlugin
from taoryx.families.cadac.falcon6_plugin import Falcon6VehiclePlugin
from taoryx.families.cadac.ghame3_plugin import Ghame3VehiclePlugin
from taoryx.families.cadac.ghame6_plugin import Ghame6VehiclePlugin
from taoryx.families.cadac.magsix_plugin import MagsixVehiclePlugin
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from taoryx.families.cadac.rocket6g_plugin import Rocket6gVehiclePlugin
from taoryx.families.cadac.sraam6_mission_composition import SRAAM6_TVC_REALIZATION_ID
from taoryx.families.cadac.sraam6_plugin import Sraam6VehiclePlugin
from test_ads6_aircraft import _write_ads6_aircraft_case
from test_ads6_engagement import _write_aircraft_engagement, _write_source_controller_aircraft_engagement
from test_ads6_sam import _write_ads6_sam_case
from test_ads6_srbm import _write_ads6_srbm_case
from test_agm6 import _write_agm6_case
from test_aim5 import AERO, INPUT, PROP
from test_cruise5 import _case as _write_cruise_case
from test_falcon6 import _write_falcon_case
from test_ghame3 import _case as _write_ghame3_case
from test_ghame6 import _write_ghame6_case
from test_magsix import _case as _write_magsix_case
from test_rocket6g import _write_rocket_case
from test_sraam6 import _write_sraam6_case

from taoryx.trajectory.execution_contract import MissionCompositionRunnerRegistry
from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionDescriptor,
    MissionCompositionSessionStepRequest,
    MissionCompositionSessionStepResult,
)


def _write_case(tmp_path: Path) -> Path:
    path = tmp_path / "input.asc"
    path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return path


####


def _assert_persistent_analysis_outputs(
    provider: CadacMissionCompositionProvider,
    model_id: str,
) -> None:
    """Keep persistent CADAC metadata, standard outputs, and analysis evidence aligned."""

    model = next(item for item in provider.list_models() if item.id == model_id)
    contract = provider.get_model_integration_contract(model_id)
    channels = {channel.id: channel for channel in model.output_schema.channels}

    assert "step" in model.operations
    assert all("step" in fidelity.operations for fidelity in model.fidelities)
    for channel_id in (*contract.controller_analysis.command_output_channel_ids, *contract.controller_analysis.response_output_channel_ids):
        assert "step" in channels[channel_id].operations
    ####


def _assert_source_program_authority(
    descriptor: MissionCompositionSessionDescriptor,
    step: MissionCompositionSessionStepResult,
) -> None:
    """Require CADAC sessions to expose source ownership without fake actions."""

    assert descriptor.action_schema == ()
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert descriptor.default_authority_profile_id == "source_program_control"
    assert descriptor.active_authority_profile_id == "source_program_control"
    assert descriptor.command_source_id == "cadac_source_program"
    assert len(descriptor.authority_profiles) == 1
    profile = descriptor.authority_profiles[0]
    assert profile.action_ids == ()
    assert profile.command_owner == "source_program"
    assert profile.scheme_id == "provider.program"
    assert profile.selection_scope == "provider"
    assert profile.switching_policy == "provider_managed"
    assert descriptor.initial_observation.control_authority is not None
    assert descriptor.initial_observation.control_authority.active_profile_id == profile.id
    assert step.authority_profile_id == profile.id
    assert step.command_source_id == "cadac_source_program"
    assert step.lowered_action == {}
    assert step.lowering_evidence["command_owner"] == "source_program"
    assert step.observation.control_authority == descriptor.initial_observation.control_authority
    ####


####


def test_catalog_provider_discovers_every_dynamic_cadac_actor(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    models = provider.list_models()

    assert provider.metadata.model_count == 17
    assert len(models) == 17
    assert "cadac.ads6.radar" not in {item.id for item in models}
    assert "cadac.agm6.ground_target" in {item.id for item in models}
    assert "cadac.ghame6.ground_site" not in {item.id for item in models}
    assert "cadac.aim5.missile" in {item.id for item in models}
    assert "cadac.rocket6g.launch_vehicle" in {item.id for item in models}


####


def test_catalog_provider_delegates_the_installed_aim5_persistent_session(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.aim5.missile",
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="catalog-aim5",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    contract = provider.get_model_integration_contract("cadac.aim5.missile")
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="catalog-aim5", duration_s=0.01))

    assert descriptor.provider_version == provider.metadata.version
    _assert_source_program_authority(descriptor, step)
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert contract.step.state_semantics == "persistent_native_state"
    assert contract.sensor_integration.sensor_bus_status == "available"
    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert set(contract.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(contract.controller_analysis.response_output_channel_ids) >= {"normal_acceleration_g", "lateral_acceleration_g"}
    _assert_persistent_analysis_outputs(provider, "cadac.aim5.missile")


####


def test_catalog_provider_delegates_the_installed_ads6_srbm_persistent_session(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ads6_srbm_plugin=Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.ads6.srbm",
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="catalog-ads6-srbm",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    contract = provider.get_model_integration_contract("cadac.ads6.srbm")
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="catalog-ads6-srbm", duration_s=0.01))

    assert descriptor.provider_version == provider.metadata.version
    _assert_source_program_authority(descriptor, step)
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert contract.step.state_semantics == "persistent_native_state"
    assert contract.sensor_integration.sensor_bus_status == "available"
    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert set(contract.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(contract.controller_analysis.response_output_channel_ids) >= {"normal_acceleration_g", "lateral_acceleration_g"}
    _assert_persistent_analysis_outputs(provider, "cadac.ads6.srbm")


####


def test_catalog_provider_delegates_the_installed_ads6_package_persistent_session(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(
        ads6_engagement_plugin=Ads6EngagementPlugin(_write_source_controller_aircraft_engagement(tmp_path))
    )
    configuration = build_default_cadac_configuration(provider, "cadac.ads6.engagement")
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="catalog-ads6-engagement",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    contract = provider.get_model_integration_contract("cadac.ads6.engagement")
    step = provider.step_session(
        MissionCompositionSessionStepRequest(session_id="catalog-ads6-engagement", duration_s=0.01)
    )

    assert descriptor.provider_version == provider.metadata.version
    _assert_source_program_authority(descriptor, step)
    assert step.observation.values["native_relative_state_tracks"]["ads6-engagement-m1-native-relative-state"]["valid"]
    assert contract.step.state_semantics == "persistent_native_state"
    assert contract.sensor_integration.sensor_bus_status == "available"
    assert contract.sensor_integration.blockers == ()
    assert contract.environment.execution_profile == "cadac_compat"
    assert contract.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert contract.environment.gravity_owner == "cadac_compatibility_runtime"
    assert contract.environment.host_environment_status == "blocked"
    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert set(contract.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(contract.controller_analysis.response_output_channel_ids) >= {
        "achieved_lateral_acceleration_g",
        "achieved_normal_acceleration_g",
    }
    _assert_persistent_analysis_outputs(provider, "cadac.ads6.engagement")


####


def test_catalog_provider_delegates_the_installed_sraam6_persistent_session(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(sraam6_plugin=Sraam6VehiclePlugin(_write_sraam6_case(tmp_path)))
    configuration = build_default_cadac_configuration(provider, "cadac.sraam6.missile")
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="catalog-sraam6",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    contract = provider.get_model_integration_contract("cadac.sraam6.missile")
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="catalog-sraam6", duration_s=0.01))

    assert descriptor.provider_version == provider.metadata.version
    _assert_source_program_authority(descriptor, step)
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert contract.step.state_semantics == "persistent_native_state"
    assert contract.sensor_integration.sensor_bus_status == "available"
    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert set(contract.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(contract.controller_analysis.response_output_channel_ids) >= {"normal_acceleration_g", "lateral_acceleration_g"}
    _assert_persistent_analysis_outputs(provider, "cadac.sraam6.missile")


####


def test_catalog_provider_delegates_the_installed_agm6_persistent_session(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(agm6_plugin=Agm6VehiclePlugin(_write_agm6_case(tmp_path)))
    configuration = build_default_cadac_configuration(provider, "cadac.agm6.missile")
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="catalog-agm6",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.001,
        )
    )
    contract = provider.get_model_integration_contract("cadac.agm6.missile")
    step = provider.step_session(MissionCompositionSessionStepRequest(session_id="catalog-agm6", duration_s=0.001))

    assert descriptor.provider_version == provider.metadata.version
    _assert_source_program_authority(descriptor, step)
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert contract.step.state_semantics == "persistent_native_state"
    assert contract.sensor_integration.sensor_bus_status == "available"
    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert set(contract.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(contract.controller_analysis.response_output_channel_ids) >= {"normal_acceleration_g", "lateral_acceleration_g"}
    _assert_persistent_analysis_outputs(provider, "cadac.agm6.missile")


####


def test_planned_falcon_plugin_preserves_physical_surface_fidelity(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    model = next(item for item in provider.list_models() if item.id == "cadac.falcon6.aircraft")
    configuration = build_default_cadac_configuration(provider, model.id)

    provider.validate_configuration(configuration)

    assert model.common_runner_operations == ()
    assert model.fidelities[0].id == "rigid_body_6dof_surface_allocated"
    assert model.realizations[0].input_realization == "actuator_allocated"
    assert model.realizations[0].actuator_types == ("aerodynamic_surfaces",)


####


def test_rocket_plugin_keeps_phase_specific_direct_and_physical_control_realizations(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    model = next(item for item in provider.list_models() if item.id == "cadac.rocket6g.launch_vehicle")
    realization_by_id = {item.id: item for item in model.realizations}

    assert realization_by_id["cadac-source-phase.aggregate_rcs"].input_realization == "direct_wrench"
    assert realization_by_id["cadac-source-phase.physical_tvc"].input_realization == "actuator_allocated"
    assert realization_by_id["cadac-source-phase.physical_tvc"].actuator_types == ("thrust_vectoring",)
    assert realization_by_id["cadac-source-phase.mixed_tvc_rcs"].actuator_types == ("mixed",)


####


def test_planned_phase_validation_rejects_wrong_fidelity(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.cruise5.cruise_vehicle",
        phase_id="translation_only",
    )
    payload = configuration.model_dump()
    payload["fidelity"] = "pseudo_6dof"
    wrong = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="not requested fidelity"):
        provider.validate_configuration(wrong)
    ####


####


def test_catalog_provider_registers_only_installed_exact_runtimes(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(aim5_plugin=Aim5VehiclePlugin(_write_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.aim5.missile")
    assert not registry.has_executor("cadac", "cadac.falcon6.aircraft")


####


def test_catalog_provider_can_discover_without_installing_any_runtime() -> None:
    provider = CadacMissionCompositionProvider()
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert provider.metadata.model_count == 17
    assert not provider.has_installed_aim5_runtime()
    assert not registry.has_executor("cadac", "cadac.aim5.missile")
    aim5 = next(item for item in provider.list_models() if item.id == "cadac.aim5.missile")
    assert aim5.common_runner_operations == ()
    for model in provider.list_models():
        output = provider.get_model_output_schema(model.id)
        assert all(channel.quantity for channel in (*output.core_channels, *output.telemetry_channels))


####


def test_explicit_source_bindings_install_only_the_bound_model(tmp_path: Path) -> None:
    provider = CadacSourceCaseBindings(
        falcon6_case_path=_write_falcon_case(tmp_path),
    ).build_provider()
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert provider.has_installed_falcon6_runtime()
    assert not provider.has_installed_aim5_runtime()
    assert registry.has_executor("cadac", "cadac.falcon6.aircraft")
    assert not registry.has_executor("cadac", "cadac.aim5.missile")
    falcon = next(item for item in provider.list_models() if item.id == "cadac.falcon6.aircraft")
    assert falcon.realizations[0].controls.status == "available"
    ####


def test_installed_direct_plant_publishes_strict_integration_and_control_output_contract(tmp_path: Path) -> None:
    provider = CadacSourceCaseBindings(
        falcon6_case_path=_write_falcon_case(tmp_path),
    ).build_provider()
    falcon = next(item for item in provider.list_models() if item.id == "cadac.falcon6.aircraft")
    contract = provider.get_model_integration_contract(falcon.id)

    assert contract.step.status == "blocked"
    assert contract.step.state_semantics == "batch_only"
    assert contract.environment.execution_profile == "cadac_compat"
    assert contract.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert contract.environment.host_environment_status == "blocked"
    assert contract.controller_analysis.ownership == "external_at_source_boundary"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert "requested_surfaces_deg" in contract.controller_analysis.command_output_channel_ids
    assert "achieved_surfaces_deg" in contract.controller_analysis.response_output_channel_ids

    output_ids = {channel.id for channel in falcon.output_schema.channels}
    for control in falcon.realizations[0].controls.channels:
        assert control.quantity
        evidence = control.provider_binding["output_evidence"]
        assert evidence["requested"]["channel_id"] in output_ids
        assert all(item["channel_id"] in output_ids for item in evidence["realized"])
    ####


def test_source_owned_controller_is_analysis_ready_but_not_assumed_stable(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(
        ads6_aircraft_plugin=Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path)),
    )
    contract = provider.get_model_integration_contract("cadac.ads6.aircraft")

    assert contract.controller_analysis.ownership == "source_owned"
    assert contract.controller_analysis.time_domain_analysis == "available"
    assert contract.controller_analysis.controller_comparison == "available"
    assert contract.controller_analysis.local_linear_stability == "blocked"
    assert "commanded_bank_deg" in contract.controller_analysis.command_output_channel_ids
    assert "bank_state_deg" in contract.controller_analysis.response_output_channel_ids
    ####


def test_standard_source_binding_requires_the_complete_upstream_layout(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="ADS6/input_AC_Straight and level.asc"):
        CadacSourceCaseBindings.from_standard_checkout(tmp_path)
    ####


####


def test_ads6_sam_package_actor_binding_requires_an_explicit_missile_selection(tmp_path: Path) -> None:
    source_case = _write_aircraft_engagement(tmp_path)
    unselected = Ads6SamVehiclePlugin(source_case)
    selected = Ads6SamVehiclePlugin(source_case, missile_actor_index=0)

    assert "exactly one standalone MISSILE6" in unselected.validate_installation()[0]
    assert selected.validate_installation() == ()
    assert selected.missile_actor_index == 0


####


def test_catalog_provider_registers_both_installed_exact_runtimes(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5"
    aim_root.mkdir()
    falcon_root = tmp_path / "falcon6"
    falcon_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)
    assert registry.has_executor("cadac", "cadac.aim5.missile")
    assert registry.has_executor("cadac", "cadac.falcon6.aircraft")
    falcon = next(item for item in provider.list_models() if item.id == "cadac.falcon6.aircraft")
    assert falcon.common_runner_operations == ("batch",)
    assert provider.has_installed_falcon6_runtime()


####


def test_catalog_provider_registers_cruise5_as_third_exact_runtime(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5"
    aim_root.mkdir()
    cruise_root = tmp_path / "cruise5"
    cruise_root.mkdir()
    falcon_root = tmp_path / "falcon6"
    falcon_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(cruise_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)
    assert registry.has_executor("cadac", "cadac.aim5.missile")
    assert registry.has_executor("cadac", "cadac.cruise5.cruise_vehicle")
    assert registry.has_executor("cadac", "cadac.falcon6.aircraft")
    cruise = next(item for item in provider.list_models() if item.id == "cadac.cruise5.cruise_vehicle")
    assert cruise.common_runner_operations == ("batch",)
    assert provider.has_installed_cruise5_runtime()


####


def test_installed_cruise5_keeps_translation_tier_validate_only(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.cruise5.cruise_vehicle",
        phase_id="translation_only",
    )
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "point_mass_3dof"
    cruise = next(item for item in provider.list_models() if item.id == "cadac.cruise5.cruise_vehicle")
    tier = next(item for item in cruise.fidelities if item.id == "point_mass_3dof")
    assert tier.operations == ("validate",)


####


def test_catalog_provider_registers_four_reference_runtimes(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5-reference"
    aim_root.mkdir()
    cruise_root = tmp_path / "cruise5-reference"
    cruise_root.mkdir()
    falcon_root = tmp_path / "falcon6-reference"
    falcon_root.mkdir()
    ghame_root = tmp_path / "ghame3-reference"
    ghame_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(cruise_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
        ghame3_plugin=Ghame3VehiclePlugin(_write_ghame3_case(ghame_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.aim5.missile")
    assert registry.has_executor("cadac", "cadac.cruise5.cruise_vehicle")
    assert registry.has_executor("cadac", "cadac.falcon6.aircraft")
    assert registry.has_executor("cadac", "cadac.ghame3.hypersonic_vehicle")
    ghame = next(item for item in provider.list_models() if item.id == "cadac.ghame3.hypersonic_vehicle")
    assert ghame.common_runner_operations == ("batch",)
    assert ghame.fidelities[0].id == "point_mass_3dof"
    assert provider.has_installed_ghame3_runtime()


####


def test_installed_ghame3_rejects_non_source_phase(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ghame3_plugin=Ghame3VehiclePlugin(_write_ghame3_case(tmp_path)))
    with pytest.raises(ValueError, match="source phase 'source_model'"):
        build_default_cadac_configuration(
            provider,
            "cadac.ghame3.hypersonic_vehicle",
            phase_id="translation_only",
        )
    ####


####


def test_catalog_provider_registers_five_reference_runtimes(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5-all"
    aim_root.mkdir()
    cruise_root = tmp_path / "cruise5-all"
    cruise_root.mkdir()
    falcon_root = tmp_path / "falcon6-all"
    falcon_root.mkdir()
    ghame_root = tmp_path / "ghame3-all"
    ghame_root.mkdir()
    magsix_root = tmp_path / "magsix-all"
    magsix_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(cruise_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
        ghame3_plugin=Ghame3VehiclePlugin(_write_ghame3_case(ghame_root)),
        magsix_plugin=MagsixVehiclePlugin(_write_magsix_case(magsix_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.aim5.missile")
    assert registry.has_executor("cadac", "cadac.cruise5.cruise_vehicle")
    assert registry.has_executor("cadac", "cadac.falcon6.aircraft")
    assert registry.has_executor("cadac", "cadac.ghame3.hypersonic_vehicle")
    assert registry.has_executor("cadac", "cadac.magsix.vehicle")
    assert provider.has_installed_magsix_runtime()
    magsix = next(item for item in provider.list_models() if item.id == "cadac.magsix.vehicle")
    assert magsix.common_runner_operations == ("batch",)
    assert tuple(item.id for item in magsix.fidelities) == ("point_mass_3dof", "pseudo_6dof")


####


def test_installed_magsix_keeps_restricted_attitude_validate_only(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(magsix_plugin=MagsixVehiclePlugin(_write_magsix_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.magsix.vehicle",
        phase_id="restricted_attitude",
    )
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "pseudo_6dof"
    model = next(item for item in provider.list_models() if item.id == "cadac.magsix.vehicle")
    tier = next(item for item in model.fidelities if item.id == "pseudo_6dof")
    assert tier.operations == ("validate",)


####


def test_catalog_provider_registers_six_reference_runtimes_including_rocket6g(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5-six"
    aim_root.mkdir()
    cruise_root = tmp_path / "cruise5-six"
    cruise_root.mkdir()
    falcon_root = tmp_path / "falcon6-six"
    falcon_root.mkdir()
    ghame_root = tmp_path / "ghame3-six"
    ghame_root.mkdir()
    magsix_root = tmp_path / "magsix-six"
    magsix_root.mkdir()
    rocket_root = tmp_path / "rocket6g-six"
    rocket_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(cruise_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
        ghame3_plugin=Ghame3VehiclePlugin(_write_ghame3_case(ghame_root)),
        magsix_plugin=MagsixVehiclePlugin(_write_magsix_case(magsix_root)),
        rocket6g_plugin=Rocket6gVehiclePlugin(_write_rocket_case(rocket_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.rocket6g.launch_vehicle")
    assert provider.has_installed_rocket6g_runtime()
    rocket = next(item for item in provider.list_models() if item.id == "cadac.rocket6g.launch_vehicle")
    assert rocket.common_runner_operations == ("batch",)
    assert rocket.fidelities[0].actuator_types == ("mixed",)
    configuration = build_default_cadac_configuration(provider, rocket.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_catalog_provider_registers_seven_reference_runtimes_including_sraam6(tmp_path: Path) -> None:
    aim_root = tmp_path / "aim5-seven"
    aim_root.mkdir()
    cruise_root = tmp_path / "cruise5-seven"
    cruise_root.mkdir()
    falcon_root = tmp_path / "falcon6-seven"
    falcon_root.mkdir()
    ghame_root = tmp_path / "ghame3-seven"
    ghame_root.mkdir()
    magsix_root = tmp_path / "magsix-seven"
    magsix_root.mkdir()
    rocket_root = tmp_path / "rocket6g-seven"
    rocket_root.mkdir()
    sraam_root = tmp_path / "sraam6-seven"
    sraam_root.mkdir()
    provider = CadacMissionCompositionProvider(
        aim5_plugin=Aim5VehiclePlugin(_write_case(aim_root)),
        cruise5_plugin=Cruise5VehiclePlugin(_write_cruise_case(cruise_root)),
        falcon6_plugin=Falcon6VehiclePlugin(_write_falcon_case(falcon_root)),
        ghame3_plugin=Ghame3VehiclePlugin(_write_ghame3_case(ghame_root)),
        magsix_plugin=MagsixVehiclePlugin(_write_magsix_case(magsix_root)),
        rocket6g_plugin=Rocket6gVehiclePlugin(_write_rocket_case(rocket_root)),
        sraam6_plugin=Sraam6VehiclePlugin(_write_sraam6_case(sraam_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.sraam6.missile")
    assert provider.has_installed_sraam6_runtime()
    sraam = next(item for item in provider.list_models() if item.id == "cadac.sraam6.missile")
    assert sraam.common_runner_operations == ("batch", "step")
    assert sraam.fidelities[0].input_realization == "actuator_allocated"
    assert sraam.fidelities[0].actuator_types == ("aerodynamic_surfaces",)
    configuration = build_default_cadac_configuration(provider, sraam.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_installed_sraam6_keeps_optional_tvc_phase_validate_only(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(sraam6_plugin=Sraam6VehiclePlugin(_write_sraam6_case(tmp_path)))
    configuration = build_default_cadac_configuration(
        provider,
        "cadac.sraam6.missile",
        phase_id="optional_tvc",
    )
    prepared = provider.validate_configuration(configuration)
    model = next(item for item in provider.list_models() if item.id == "cadac.sraam6.missile")
    tvc = next(item for item in model.realizations if item.id == SRAAM6_TVC_REALIZATION_ID)

    assert prepared.configuration.realization_id == SRAAM6_TVC_REALIZATION_ID
    assert prepared.configuration.mission_template_id is None
    assert tvc.status == "blocked"
    assert tvc.operations == ("validate",)


####


def test_catalog_provider_registers_eight_reference_runtimes_including_agm6(tmp_path: Path) -> None:
    agm_root = tmp_path / "agm6-eight"
    agm_root.mkdir()
    provider = CadacMissionCompositionProvider(
        agm6_plugin=Agm6VehiclePlugin(_write_agm6_case(agm_root)),
    )
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.agm6.missile")
    assert provider.has_installed_agm6_runtime()
    agm = next(item for item in provider.list_models() if item.id == "cadac.agm6.missile")
    assert agm.common_runner_operations == ("batch", "step")
    assert agm.fidelities[0].input_realization == "actuator_allocated"
    configuration = build_default_cadac_configuration(provider, agm.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_catalog_provider_registers_agm6_three_actor_runtime_without_fallback(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(agm6_plugin=Agm6VehiclePlugin(_write_agm6_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.agm6.missile")
    assert not registry.has_executor("cadac", "cadac.agm6.ground_target")
    assert not registry.has_executor("cadac", "cadac.agm6.aircraft")
    assert provider.has_installed_agm6_runtime()
    model = next(item for item in provider.list_models() if item.id == "cadac.agm6.missile")
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("aerodynamic_surfaces",)
    configuration = build_default_cadac_configuration(provider, model.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_catalog_provider_registers_ninth_reference_runtime_including_ghame6(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ghame6_plugin=Ghame6VehiclePlugin(_write_ghame6_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.ghame6.hypersonic_vehicle")
    assert not registry.has_executor("cadac", "cadac.ghame6.satellite")
    assert not registry.has_executor("cadac", "cadac.ghame6.ground_site")
    assert provider.has_installed_ghame6_runtime()
    model = next(item for item in provider.list_models() if item.id == "cadac.ghame6.hypersonic_vehicle")
    assert model.common_runner_operations == ("batch",)
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.fidelities[0].actuator_types == ("mixed",)
    assert "no TVC" in model.fidelities[0].claim_boundary
    configuration = build_default_cadac_configuration(provider, model.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_catalog_provider_registers_tenth_reference_runtime_as_ads6_sam_only(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ads6_sam_plugin=Ads6SamVehiclePlugin(_write_ads6_sam_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.ads6.sam")
    assert not registry.has_executor("cadac", "cadac.ads6.srbm")
    assert not registry.has_executor("cadac", "cadac.ads6.aircraft")
    assert provider.has_installed_ads6_sam_runtime()
    model = next(item for item in provider.list_models() if item.id == "cadac.ads6.sam")
    assert model.common_runner_operations == ("batch",)
    assert [item.id for item in model.fidelities] == [
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    ]
    configuration = build_default_cadac_configuration(
        provider,
        model.id,
        phase_id="aggregate_rcs",
    )
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "rigid_body_6dof_direct_wrench"


####


def test_catalog_provider_registers_eleventh_reference_runtime_as_ads6_srbm_exactly(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ads6_srbm_plugin=Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.ads6.srbm")
    assert not registry.has_executor("cadac", "cadac.ads6.sam")
    assert not registry.has_executor("cadac", "cadac.ads6.aircraft")
    assert provider.has_installed_ads6_srbm_runtime()
    model = next(item for item in provider.list_models() if item.id == "cadac.ads6.srbm")
    assert model.common_runner_operations == ("batch", "step")
    assert [item.id for item in model.fidelities] == ["pseudo_6dof"]
    assert model.realizations[0].input_realization == "provider_defined"
    configuration = build_default_cadac_configuration(provider, model.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "pseudo_6dof"


####


def test_catalog_provider_registers_twelfth_reference_runtime_as_ads6_aircraft_exactly(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ads6_aircraft_plugin=Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert registry.has_executor("cadac", "cadac.ads6.aircraft")
    assert not registry.has_executor("cadac", "cadac.ads6.sam")
    assert not registry.has_executor("cadac", "cadac.ads6.srbm")
    assert provider.has_installed_ads6_aircraft_runtime()
    model = next(item for item in provider.list_models() if item.id == "cadac.ads6.aircraft")
    assert model.common_runner_operations == ("batch",)
    assert [item.id for item in model.fidelities] == ["point_mass_3dof"]
    assert model.fidelities[0].control_realization == "force_model"
    configuration = build_default_cadac_configuration(provider, model.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.fidelity == "point_mass_3dof"


####


def test_catalog_provider_registers_source_ordered_ads6_package_as_additional_exact_model(tmp_path: Path) -> None:
    provider = CadacMissionCompositionProvider(ads6_engagement_plugin=Ads6EngagementPlugin(_write_aircraft_engagement(tmp_path)))
    registry = MissionCompositionRunnerRegistry()

    provider.register_runnable_models(registry)

    assert provider.metadata.model_count == 18
    assert provider.has_installed_ads6_engagement_runtime()
    assert registry.has_executor("cadac", "cadac.ads6.engagement")
    assert not registry.has_executor("cadac", "cadac.ads6.radar")
    package = next(item for item in provider.list_models() if item.id == "cadac.ads6.engagement")
    assert package.model_kind == "mission_composition"
    assert package.common_runner_operations == ("batch", "step")
    configuration = build_default_cadac_configuration(provider, package.id)
    prepared = provider.validate_configuration(configuration)
    assert prepared.configuration.model_id == "cadac.ads6.engagement"
    assert prepared.configuration.fidelity == "rigid_body_6dof_surface_allocated"


####


def test_ads6_sensor_contracts_identify_the_package_level_persistent_owner(tmp_path: Path) -> None:
    sam_root = tmp_path / "sam"
    sam_root.mkdir()
    sam_provider = CadacMissionCompositionProvider(ads6_sam_plugin=Ads6SamVehiclePlugin(_write_ads6_sam_case(sam_root)))
    sam_contract = sam_provider.get_model_integration_contract("cadac.ads6.sam").sensor_integration

    assert sam_contract.sensor_bus_status == "blocked"
    assert any("standalone SAM surface owns only the physical plant" in blocker for blocker in sam_contract.blockers)

    engagement_root = tmp_path / "engagement"
    engagement_root.mkdir()
    package_provider = CadacMissionCompositionProvider(ads6_engagement_plugin=Ads6EngagementPlugin(_write_aircraft_engagement(engagement_root)))
    package_contract = package_provider.get_model_integration_contract("cadac.ads6.engagement").sensor_integration

    assert package_contract.sensor_bus_status == "available"
    assert package_contract.blockers == ()


####
