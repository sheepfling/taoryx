"""Regression coverage for profile-bound composition policy execution."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.composition_policy import (
    PolicyDecision,
    replay_composition_policy_trace,
    replay_composition_policy_trace_file,
    replay_serialized_composition_policy_trace,
    run_composition_policy,
    write_composition_policy_trace,
)
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def test_policy_harness_uses_x8_semantic_bridge_and_preserves_native_application() -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_capability_3dof_compose.yaml"))
    calls = 0

    def policy(observation, contract):
        nonlocal calls
        assert observation.observation_profile_id == "truth_debug"
        assert observation.values["position.altitude"] == pytest.approx(178.0, abs=0.1)
        assert contract.id == "skywalker_x8/point_mass_3dof"
        calls += 1
        if calls == 2:
            return None
        return PolicyDecision({"propulsion.command.fraction": 0.6}, 0.1)
        ####

    trace = run_composition_policy(
        episode,
        policy,
        authority_profile_id="native_control_bridge",
        observation_profile_id="truth_debug",
    )

    assert trace.stopped_by_policy is True
    assert len(trace.steps) == 1
    assert trace.steps[0].applied_action["throttle"] == pytest.approx(0.6)
    assert trace.steps[0].applied_semantic_action is not None
    assert trace.steps[0].applied_semantic_action["propulsion.command.fraction"] == pytest.approx(0.6)
    assert trace.final_status.values["execution.time"] == pytest.approx(0.1)
    assert trace.as_dict()["schema"] == "taoryx.composition-policy-trace/v1alpha1"
    ####


def test_policy_harness_clamps_hummingbird_thrust_without_promoting_motors() -> None:
    episode = open_vehicle_composition_episode(_composition("hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml"))
    calls = 0

    def policy(observation, contract):
        nonlocal calls
        assert observation.values["control.physical_motor_allocation"] is False
        assert contract.control_realization == "response_law"
        calls += 1
        if calls == 2:
            return None
        return PolicyDecision(
            {
                "attitude.yaw.command": 0.5,
                "propulsion.command.fraction": 2.0,
            },
            0.1,
        )
        ####

    trace = run_composition_policy(
        episode,
        policy,
        authority_profile_id="body_motion_response",
        observation_profile_id="truth_debug",
    )

    applied = trace.steps[0]
    assert applied.applied_action["yaw_rad"] == pytest.approx(0.5)
    assert applied.applied_action["thrust_ratio"] == pytest.approx(1.0)
    assert applied.applied_semantic_action is not None
    assert applied.applied_semantic_action["propulsion.command.fraction"] == pytest.approx(1.0)
    assert trace.final_status.values["control.physical_motor_allocation"] is False
    ####


def test_policy_harness_rejects_planned_or_unavailable_profiles() -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_capability_3dof_compose.yaml"))

    with pytest.raises(KeyError, match="unknown authority profile"):
        run_composition_policy(episode, lambda observation, contract: None, authority_profile_id="direct_wrench")
    ####


def test_policy_harness_defaults_to_the_composition_declared_sensor_profile() -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_sensor_episode_3dof_compose.yaml"))
    calls = 0

    def policy(observation, contract):
        nonlocal calls
        calls += 1
        assert observation.observation_profile_id == "declared_sensor"
        if calls == 1:
            assert observation.source_time_s is None
            assert not any(observation.valid.values())
            return PolicyDecision({"propulsion.command.fraction": 0.6}, 0.06)
        assert observation.source_time_s == pytest.approx(0.0)
        assert observation.valid["position.altitude"] is True
        return None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")

    assert trace.observation_profile_id == "declared_sensor"
    assert trace.final_observation.source_time_s == pytest.approx(0.0)
    ####


@pytest.mark.parametrize(
    ("request_name", "authority_profile_id", "action"),
    (
        (
            "x8_racetrack_capability_3dof_compose.yaml",
            "native_control_bridge",
            {"propulsion.command.fraction": 0.6},
        ),
        (
            "hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml",
            "body_motion_response",
            {"attitude.yaw.command": 0.5, "propulsion.command.fraction": 0.8},
        ),
    ),
)
def test_policy_trace_replays_the_same_semantic_action_stream_at_committed_boundaries(
    request_name: str,
    authority_profile_id: str,
    action: dict[str, float],
) -> None:
    composition = _composition(request_name)
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision(action, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id=authority_profile_id)
    replay = replay_composition_policy_trace(composition, trace)

    assert trace.composition_id == composition.id
    assert trace.composition_identity_sha256 == composition.identity_sha256
    assert replay.step_count == 1
    assert replay.final_time_s == pytest.approx(trace.final_status.time_s)
    assert replay.as_dict()["status"] == "pass"
    ####


def test_policy_trace_replay_refuses_a_nearby_composition_identity() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    trace = run_composition_policy(
        episode,
        lambda observation, contract: None,
        authority_profile_id="native_control_bridge",
    )
    incompatible = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / "x8_racetrack_capability_compose.yaml")
    )

    with pytest.raises(ValueError, match="composition identity"):
        replay_composition_policy_trace(incompatible, trace)
    ####


def test_persisted_policy_trace_replays_through_the_public_cli(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    compiled_path = tmp_path / "composition.json"
    composition.write_json(compiled_path)
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.6}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    trace_path = write_composition_policy_trace(trace, tmp_path / "policy-trace.json")
    report_path = tmp_path / "policy-replay.json"

    assert main(["vehicle", "replay-policy", str(compiled_path), str(trace_path), "--output", str(report_path)]) == 0
    assert "wrote" in capsys.readouterr().out
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "pass"
    assert payload["composition_identity_sha256"] == composition.identity_sha256
    assert replay_composition_policy_trace_file(composition, trace_path).step_count == 1
    ####


def test_persisted_policy_trace_rejects_modified_public_frame(tmp_path: Path) -> None:
    composition = _composition("hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    trace = run_composition_policy(
        episode,
        lambda observation, contract: PolicyDecision({"attitude.yaw.command": 0.5}, 0.1)
        if observation.time_s == 0.0
        else None,
        authority_profile_id="body_motion_response",
    )
    payload = trace.as_dict()
    final_status = payload["final_status"]
    assert isinstance(final_status, dict)
    final_status["time_s"] = 999.0
    path = tmp_path / "tampered-policy-trace.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="final status"):
        replay_composition_policy_trace_file(composition, path)
    with pytest.raises(ValueError, match="final status"):
        replay_serialized_composition_policy_trace(composition, payload)
    ####
