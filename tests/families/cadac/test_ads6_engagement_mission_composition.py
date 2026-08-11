from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_engagement_mission_composition import (
    ADS6_ENGAGEMENT_MODEL_ID,
    ADS6_ENGAGEMENT_MODEL_VERSION,
    CadacAds6EngagementMissionCompositionProvider,
    build_default_ads6_engagement_configuration,
    register_ads6_engagement_mission_composition,
)
from taoryx.families.cadac.ads6_engagement_plugin import Ads6EngagementPlugin
from taoryx.families.cadac.controller_analysis import (
    analyze_controller_trace,
    cadac_controller_trace_from_samples,
)
from test_ads6_engagement import (
    _write_aircraft_engagement,
    _write_source_controller_aircraft_engagement,
    _write_srbm_engagement,
    _write_three_aircraft_engagement,
)

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


def _provider(
    tmp_path: Path,
    *,
    target_kind: str = "aircraft",
) -> CadacAds6EngagementMissionCompositionProvider:
    source_path = _write_aircraft_engagement(tmp_path) if target_kind == "aircraft" else _write_srbm_engagement(tmp_path)
    return CadacAds6EngagementMissionCompositionProvider(Ads6EngagementPlugin(source_path))


####


def test_ads6_engagement_provider_advertises_one_exact_multi_root_package(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    model = provider.list_models()[0]

    assert model.id == ADS6_ENGAGEMENT_MODEL_ID
    assert model.version == ADS6_ENGAGEMENT_MODEL_VERSION
    assert model.model_kind == "mission_composition"
    assert model.common_runner_operations == ("batch", "step")
    assert model.output_schema.entity_output.supports_multiple_entities is True
    assert model.output_schema.entity_output.supports_dynamic_spawning is False
    assert model.presentation.properties[0].value == "aircraft"
    channels = {channel.id: channel for channel in model.output_schema.channels}
    assert channels["normal_command_g"].operations == ("batch", "step")
    assert channels["achieved_normal_acceleration_g"].operations == ("batch", "step")


####


def test_ads6_engagement_common_runner_returns_source_ordered_independent_roots(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={
            "command_law": "hold",
            "end_time_s": 0.03,
            "sample_step_s": 0.01,
        },
    )
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    register_ads6_engagement_mission_composition(provider, registry)
    response = registry.run(
        MissionCompositionRunRequest(
            request_id="ads6-package-aircraft",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    result = response.result

    assert result.status == "completed"
    assert result.primary_model_id == ADS6_ENGAGEMENT_MODEL_ID
    assert tuple(item.object_id for item in result.objects) == ("m1", "a1", "f1")
    assert tuple(item.parent_object_id for item in result.objects) == (None, None, None)
    assert tuple(item.fidelity for item in result.objects) == (
        "rigid_body_6dof_surface_allocated",
        "point_mass_3dof",
        "static_sensor",
    )
    assert any(event.kind == "launch_command" and event.time_s == pytest.approx(0.0) for event in result.events)
    assert any(event.kind == "missile_launch" and event.time_s == pytest.approx(0.01) for event in result.events)
    assert result.relationships == ()
    assert result.objects[0].samples[0].values["held"] is True
    assert result.objects[0].samples[-1].values["held"] is False
    assert "quaternion_wxyz" not in result.objects[1].samples[-1].values
    assert result.objects[2].samples[-1].values["actor_kind"] == "radar"
    assert result.diagnostics[0].details["launch_schedule_semantics"] == "documented_radar_latched_next_epoch"


####


def test_ads6_engagement_srbm_provider_reports_mixed_t4_t2_static_roots(tmp_path: Path) -> None:
    provider = _provider(tmp_path, target_kind="srbm")
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={"command_law": "hold", "end_time_s": 0.04, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-package-srbm",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core"),
        )
    )

    assert tuple(item.object_id for item in result.objects) == ("m1", "r1", "f1")
    assert {item.fidelity for item in result.objects} == {
        "rigid_body_6dof_surface_allocated",
        "pseudo_6dof",
        "static_sensor",
    }
    assert all(tuple(channel.id for channel in item.channels) == ("position_ned_m", "velocity_ned_mps") for item in result.objects)
    assert any(event.kind == "launch_command" and event.data["reason"] == "apogee_prediction" for event in result.events)


####


def test_ads6_engagement_default_configuration_runs_interleaved_source_controller(tmp_path: Path) -> None:
    provider = CadacAds6EngagementMissionCompositionProvider(Ads6EngagementPlugin(_write_source_controller_aircraft_engagement(tmp_path)))
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={"end_time_s": 0.10, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    package = prepared.resolved["package"]
    result = provider.execute_batch(
        MissionCompositionRunRequest(
            request_id="ads6-package-source-controller",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert package["command_law"] == "source_controller"
    missile = result.objects[0]
    telemetry = missile.samples[-1].values["telemetry"]
    assert telemetry["sensor_mode"] == 14
    assert telemetry["guidance_mode"] == 7
    assert telemetry["controller_mode"] == 3
    assert any(event.kind == "seeker_lock" for event in result.events)
    diagnostic = result.diagnostics[0]
    assert diagnostic.code == "cadac-ads6-package-source-controller-boundary"
    assert diagnostic.details["source_controller_active"] is True
    assert diagnostic.details["sam_controller_trace_count"] == 1
    assert diagnostic.details["sam_controller_sample_count"] > 0
    assert diagnostic.details["sam_controller_event_count"] == 2
    assert "complete source SAM seeker/INS/guidance" not in diagnostic.message
    trace = cadac_controller_trace_from_samples(
        missile.samples,
        controller_id="ads6-source-controller",
        model_id=ADS6_ENGAGEMENT_MODEL_ID,
        reference_channel_id="normal_command_g",
        response_channel_id="achieved_normal_acceleration_g",
        unit="g",
    )
    report = analyze_controller_trace(trace, settling_absolute_tolerance=0.1)
    assert report.sample_count == len(missile.samples)
    assert report.formal_stability_claim is False


####


def test_ads6_engagement_persistent_session_owns_package_controller_and_native_tracks(tmp_path: Path) -> None:
    provider = CadacAds6EngagementMissionCompositionProvider(
        Ads6EngagementPlugin(_write_source_controller_aircraft_engagement(tmp_path))
    )
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={"end_time_s": 0.04, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="ads6-engagement-persistent",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            seed=31,
            integration_step_s=0.01,
        )
    )

    assert descriptor.action_schema == ()
    initial_tracks = descriptor.initial_observation.values["native_relative_state_tracks"]
    assert initial_tracks["ads6-engagement-m1-native-relative-state"]["valid"]
    step = provider.step_session(
        MissionCompositionSessionStepRequest(session_id="ads6-engagement-persistent", duration_s=0.02)
    )

    assert step.time_start_s == 0.0
    assert step.time_end_s == 0.02
    assert step.observation.lifecycle == "active"
    assert step.observation.values["source_controller_state"]["source_schedule"] == "sam_then_target_then_radar"
    assert isinstance(step.observation.values["normal_command_g"], float)
    assert isinstance(step.observation.values["achieved_lateral_normal_acceleration_g"], tuple)
    track = step.observation.values["native_relative_state_tracks"]["ads6-engagement-m1-native-relative-state"]
    assert track["sequence"] == 2
    assert track["payload"]["target_id"] == "a1"
    assert [packet.sequence for packet in provider.session_sensor_packets("ads6-engagement-persistent")] == [0, 1, 2]

    reset = provider.reset_session(MissionCompositionResetSessionRequest(session_id="ads6-engagement-persistent"))
    assert reset.sequence == 0
    assert reset.time_s == 0.0
    assert reset.values == descriptor.initial_observation.values


####


def test_ads6_engagement_session_publishes_one_native_track_per_source_pair(tmp_path: Path) -> None:
    provider = CadacAds6EngagementMissionCompositionProvider(
        Ads6EngagementPlugin(_write_three_aircraft_engagement(tmp_path))
    )
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={"command_law": "hold", "end_time_s": 0.02, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="ads6-engagement-three-pair-tracks",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    initial_tracks = descriptor.initial_observation.values["native_relative_state_tracks"]

    assert set(initial_tracks) == {
        "ads6-engagement-m1-native-relative-state",
        "ads6-engagement-m2-native-relative-state",
        "ads6-engagement-m3-native-relative-state",
    }
    step = provider.step_session(
        MissionCompositionSessionStepRequest(session_id="ads6-engagement-three-pair-tracks", duration_s=0.01)
    )
    tracks = step.observation.values["native_relative_state_tracks"]

    assert [tracks[f"ads6-engagement-m{index}-native-relative-state"]["payload"]["target_id"] for index in range(1, 4)] == [
        "a1",
        "a2",
        "a3",
    ]
    assert [tracks[f"ads6-engagement-m{index}-native-relative-state"]["sequence"] for index in range(1, 4)] == [1, 1, 1]
    assert [packet.sequence for packet in provider.session_sensor_packets("ads6-engagement-three-pair-tracks")] == [0, 0, 0, 1, 1, 1]


####


def test_ads6_engagement_provider_rejects_wrong_envelope_and_neighbor_request(tmp_path: Path) -> None:
    provider = _provider(tmp_path)
    configuration = build_default_ads6_engagement_configuration(provider)
    payload = configuration.model_dump()
    payload["fidelity"] = "pseudo_6dof"
    wrong_fidelity = type(configuration).model_validate(payload)

    with pytest.raises(ValueError, match="requires SAM envelope fidelity"):
        provider.validate_configuration(wrong_fidelity)
    ####
    prepared = provider.validate_configuration(configuration)
    wrong_request = MissionCompositionRunRequest(
        request_id="ads6-package-wrong-provider",
        provider_id="another-provider",
        provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
        prepared_configuration=prepared,
    )
    with pytest.raises(ValueError, match="another provider/model"):
        provider.execute_batch(wrong_request)
    ####


####
