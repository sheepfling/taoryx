"""Pilot ladder for provider-neutral, selectable low-fidelity controls."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.trajectory.reference_mission_composition import (
    ReferenceBallisticLaunch,
    ReferenceMissionCompositionProvider,
    ReferenceWaypoint,
    ReferenceWaypointCourseStart,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
    MissionCompositionSwitchAuthorityRequest,
)

from taoryx.composition_episode import ActionFrame
from taoryx.trajectory.configuration_contract import PreparedTrajectoryConfiguration
from taoryx.trajectory.contract_probe_mission_composition import (
    ContractProbeMissionCompositionProvider,
    build_contract_probe_configuration,
)
from taoryx.trajectory.execution_contract import MissionCompositionExecutionError


def _open_request(
    provider: object,
    session_id: str,
    prepared: PreparedTrajectoryConfiguration,
) -> MissionCompositionOpenSessionRequest:
    metadata = getattr(provider, "metadata")
    return MissionCompositionOpenSessionRequest(
        session_id=session_id,
        provider_id=metadata.id,
        provider_version=metadata.version,
        prepared_configuration=prepared,
    )
    ####


def test_ballistic_session_is_explicit_zero_action_open_loop() -> None:
    provider = ReferenceMissionCompositionProvider()
    prepared = provider.prepare_ballistic(
        ReferenceBallisticLaunch(
            altitude_m=1000.0,
            speed_m_s=120.0,
            heading_deg=30.0,
            flight_path_angle_deg=10.0,
        ),
        10.0,
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(_open_request(provider, "ballistic-open-loop", prepared))

    assert descriptor.active_authority_profile_id == "open_loop_coast"
    assert descriptor.authority_profiles[0].scheme_id == "open_loop.coast"
    assert descriptor.authority_profiles[0].scheme_layer == "open_loop"
    assert descriptor.authority_profiles[0].consumer_roles == (
        "provider",
        "test_engineer",
    )
    assert descriptor.authority_profiles[0].streaming_preference == "provider_managed"
    assert descriptor.command_source_id is None
    assert descriptor.action_schema == ()
    assert descriptor.initial_observation.control_authority is not None
    assert descriptor.initial_observation.control_authority.command_owner == "open_loop"
    step = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.5,
        )
    )
    assert step.requested_action == {}
    assert step.lowered_action == {}
    assert step.lowering_evidence["native_transition"] == "propagate_ballistic_state"
    assert step.observation.values["position.north_m"] > 0.0

    with pytest.raises(MissionCompositionExecutionError, match="unknown-action-channel"):
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action={"guidance.heading.command": 90.0},
                duration_s=0.1,
            )
        )
    ####


def test_waypoint_session_switches_configured_kinematic_and_live_profiles(
    tmp_path: Path,
) -> None:
    provider = ReferenceMissionCompositionProvider()
    prepared = provider.prepare_waypoint_course(
        ReferenceWaypointCourseStart(
            north_m=0.0,
            east_m=0.0,
            altitude_m=1000.0,
            speed_m_s=100.0,
            heading_deg=0.0,
        ),
        (ReferenceWaypoint(north_m=1000.0, east_m=0.0, altitude_m=1000.0),),
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(_open_request(provider, "waypoint-switching", prepared))
    assert descriptor.active_authority_profile_id == "configured_waypoint_guidance"
    assert {
        item.id: item.scheme_id for item in descriptor.authority_profiles
    } == {
        "configured_waypoint_guidance": "mission.waypoint",
        "kinematic_velocity_command": "kinematic.flight_path",
        "live_waypoint_guidance": "mission.waypoint",
    }
    configured = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=1.0,
        )
    )
    assert configured.command_source_id is None

    kinematic = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="kinematic_velocity_command",
            expected_sequence=1,
            command_source_id="pilot-controller",
        )
    )
    assert kinematic.time_s == configured.time_end_s
    assert kinematic.sequence == configured.sequence
    assert {item.id: item.unit for item in kinematic.action_schema} == {
        "guidance.speed.command": "m/s",
        "guidance.heading.command": "deg",
        "guidance.flight_path_angle.command": "deg",
    }
    turned = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"guidance.heading.command": 90.0},
            duration_s=1.0,
            expected_sequence=1,
        )
    )
    assert turned.observation.values["position.east_m"] > 0.0
    assert turned.observation.values["velocity.speed_m_s"] == pytest.approx(100.0)
    assert turned.lowering_evidence["native_transition"] == "propagate_constant_velocity_waypoint_state"

    live = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="live_waypoint_guidance",
            expected_sequence=2,
            command_source_id="remote-waypoint-stream",
        )
    )
    assert live.command_source_id == "remote-waypoint-stream"
    assert {item.id for item in live.action_schema} == {
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
    }
    manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "navigation.waypoint.north.command": 200.0,
                "navigation.waypoint.east.command": 400.0,
                "navigation.waypoint.altitude.command": 1200.0,
                "navigation.waypoint.capture_radius.command": 10.0,
                "navigation.waypoint.speed.command": 80.0,
            },
            duration_s=1.0,
            expected_sequence=2,
        )
    )

    episode = provider.open_session_episode(prepared)
    episode.select_authority_profile("kinematic_velocity_command")
    frame = ActionFrame(
        episode.interface_contract.id,
        episode.interface_contract.fingerprint,
        "kinematic_velocity_command",
        {"guidance.heading.command": 45.0},
        0.5,
    )
    checkpoint_time = episode.step_frame(frame).time_end_s
    checkpoint = episode.save_checkpoint(tmp_path / "waypoint-session.json")
    episode.step_frame(frame)
    restored = episode.load_checkpoint(checkpoint)
    assert restored.time_s == checkpoint_time
    assert episode.active_authority_profile_id == "kinematic_velocity_command"
    ####


def test_contract_probe_live_schema_preserves_types_choices_and_event_policy() -> None:
    provider = ContractProbeMissionCompositionProvider()
    prepared = provider.validate_configuration(build_contract_probe_configuration(provider, fidelity="medium"))
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(_open_request(provider, "probe-control-types", prepared))
    assert {
        item.id: item.scheme_id for item in descriptor.authority_profiles
    } == {
        "debug_guidance_control": "debug.mixed",
        "debug_discrete_control": "debug.discrete",
        "debug_event_control": "debug.event",
    }

    actions = {item.id: item for item in descriptor.action_schema}
    assert actions["attitude.quaternion.command"].shape == (4,)
    assert actions["guidance.heading.command"].unit == "deg"
    assert actions["guidance.heading_rate.command"].unit == "deg/s"
    observations = {item.id: item for item in descriptor.observation_schema}
    assert observations["mode.index"].data_type == "int64"
    assert observations["diagnostics.payload"].data_type == "json"
    assert observations["event.marker"].sampling_semantics == "event"

    discrete = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="debug_discrete_control",
        )
    )
    discrete_actions = {item.id: item for item in discrete.action_schema}
    assert discrete_actions["autopilot.mode.select"].choices == ("manual", "hold", "track")
    with pytest.raises(MissionCompositionExecutionError, match="invalid-action-value"):
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action={"autopilot.mode.select": "not-a-mode"},
                duration_s=0.1,
            )
        )

    event_profile = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="debug_event_control",
        )
    )
    event_actions = {item.id: item for item in event_profile.action_schema}
    assert event_actions["payload.arm.command"].choices == ("arm",)
    first = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"payload.arm.command": "arm"},
            duration_s=0.1,
        )
    )
    repeated = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"payload.arm.command": "arm"},
            duration_s=0.1,
        )
    )
    assert first.events == ("payload_armed",)
    assert repeated.events == ()
    assert repeated.applied_action == {}
    assert repeated.diagnostics
    ####
