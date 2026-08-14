"""Regression coverage for declared batch/episode action-trace parity."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx.a320_reduced_batch_episode_parity import verify_serialized_a320_reduced_batch_episode_parity
from taoryx.composition_batch_episode_parity import (
    verify_composition_batch_episode_parity,
    verify_serialized_composition_batch_episode_parity,
)

from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.composition_policy import PolicyDecision, run_composition_policy, write_composition_policy_trace
from taoryx.language_backed_batch_episode_parity import verify_serialized_language_backed_batch_episode_parity
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request


def _composition(name: str):
    from pathlib import Path

    root = Path(__file__).resolve().parents[2]
    return compile_vehicle_composition(load_vehicle_composition_request(root / "examples/vehicle_composition" / name))
    ####


def test_hummingbird_batch_and_episode_replay_the_same_semantic_trace_at_sensor_boundaries() -> None:
    composition = _composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition, seed=7, integration_step_s=0.02)
    decisions = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal decisions
        decisions += 1
        if decisions == 1:
            return PolicyDecision(
                {"attitude.yaw.command": 0.5, "propulsion.command.fraction": 0.8},
                0.06,
            )
        if decisions == 2:
            return PolicyDecision(
                {
                    "attitude.roll.command": 0.15,
                    "attitude.pitch.command": -0.1,
                    "propulsion.command.fraction": 0.7,
                },
                0.10,
            )
        return None
        ####

    trace = run_composition_policy(
        episode,
        policy,
        authority_profile_id="body_motion_response",
        observation_profile_id="truth_debug",
    )
    report = verify_composition_batch_episode_parity(composition, trace)

    assert trace.integration_step_s == pytest.approx(0.02)
    assert report.status == "pass"
    assert report.batch_factory_id == "hummingbird_aggregate_thrust_pseudo_batch.v1"
    assert len(report.steps) == 2
    assert all(item.status == "pass" for item in report.steps)
    assert report.as_dict()["schema"] == "taoryx.composition-batch-episode-parity/v1alpha1"
    ####


def test_x8_batch_and_episode_replay_the_same_native_bridge_trace() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        if calls == 1:
            return PolicyDecision({"propulsion.command.fraction": 0.6}, 0.1)
        if calls == 2:
            return PolicyDecision({"propulsion.command.fraction": 0.45}, 0.2)
        return None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    report = verify_serialized_language_backed_batch_episode_parity(composition, trace.as_dict())

    assert report.status == "pass"
    assert report.batch_factory_id == "language_backed_powered_fixed_wing.v1"
    assert len(report.steps) == 2
    assert all(item.status == "pass" for item in report.steps)
    assert report.as_dict()["adapter_id"] == "taoryx.language_backed.action_trace_batch_episode_parity.v1"
    ####


def test_b747_batch_and_episode_replay_the_same_native_bridge_trace() -> None:
    composition = _composition("b747_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.55}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(
        episode,
        policy,
        authority_profile_id="native_control_bridge",
    )
    report = verify_serialized_language_backed_batch_episode_parity(composition, trace.as_dict())

    assert report.status == "pass"
    assert report.batch_factory_id == "language_backed_powered_fixed_wing.v1"
    assert len(report.steps) == 1
    ####


@pytest.mark.parametrize(
    "composition_name",
    [
        "x8_racetrack_capability_compose.yaml",
        "b747_racetrack_capability_pseudo6dof_compose.yaml",
    ],
)
def test_language_backed_pseudo6dof_batch_and_episode_replay_the_same_trace(composition_name: str) -> None:
    composition = _composition(composition_name)
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.5}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    report = verify_serialized_language_backed_batch_episode_parity(composition, trace.as_dict())

    assert report.status == "pass"
    assert report.interface_id.endswith("/pseudo_6dof")
    assert len(report.steps) == 1
    ####


@pytest.mark.parametrize(
    "composition_name",
    [
        "a320_racetrack_capability_3dof_compose.yaml",
        "a320_racetrack_capability_pseudo6dof_compose.yaml",
    ],
)
def test_a320_batch_and_episode_replay_the_same_kinematic_guidance_trace(composition_name: str) -> None:
    composition = _composition(composition_name)
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        if calls == 1:
            return PolicyDecision({"guidance.heading.command": 0.0, "guidance.speed.command": 230.0}, 0.4)
        if calls == 2:
            return PolicyDecision({"guidance.bank.command": 6.0, "guidance.flight_path_angle.command": 1.0}, 0.6)
        return None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="kinematic_guidance")
    report = verify_serialized_a320_reduced_batch_episode_parity(composition, trace.as_dict())

    assert report.status == "pass"
    assert report.batch_factory_id == "reduced_fixed_wing_openap.v1"
    assert len(report.steps) == 2
    assert all(item.status == "pass" for item in report.steps)
    ####


def test_a320_pseudo6dof_parity_trace_has_the_same_public_cli_and_python_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("a320_racetrack_capability_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"guidance.heading.command": 0.0, "guidance.bank.command": 5.0}, 0.4) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="kinematic_guidance")
    composition_path = tmp_path / "a320-pseudo-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "a320-pseudo-trace.json")

    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    assert payload["adapter_id"] == "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1"
    ####


def test_f16_pseudo6dof_parity_trace_has_the_same_public_cli_and_python_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("f16_racetrack_capability_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"guidance.heading.command": 0.0, "guidance.bank.command": 5.0}, 0.4) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="kinematic_guidance")
    composition_path = tmp_path / "f16-pseudo-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "f16-pseudo-trace.json")

    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    assert payload["adapter_id"] == "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1"
    ####


def test_persisted_hummingbird_trace_has_the_same_public_cli_and_python_parity_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition, seed=7, integration_step_s=0.02)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return (
            PolicyDecision({"attitude.yaw.command": 0.25, "propulsion.command.fraction": 0.75}, 0.06)
            if calls == 1
            else None
        )
        ####

    trace = run_composition_policy(
        episode,
        policy,
        authority_profile_id="body_motion_response",
        observation_profile_id="truth_debug",
    )
    composition_path = tmp_path / "hummingbird-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "hummingbird-trace.json")

    report = verify_serialized_composition_batch_episode_parity(composition, json.loads(trace_path.read_text(encoding="utf-8")))
    assert report.status == "pass"
    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    cli = json.loads(capsys.readouterr().out)
    assert cli["status"] == "pass"
    assert cli["adapter_id"] == "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1"
    ####


def test_x8_parity_trace_has_the_same_public_cli_and_python_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.6}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    composition_path = tmp_path / "x8-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "x8-trace.json")

    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    assert payload["adapter_id"] == "taoryx.language_backed.action_trace_batch_episode_parity.v1"
    ####


def test_b747_parity_trace_has_the_same_public_cli_and_python_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("b747_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.55}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    composition_path = tmp_path / "b747-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "b747-trace.json")

    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    assert payload["adapter_id"] == "taoryx.language_backed.action_trace_batch_episode_parity.v1"
    ####


def test_x8_pseudo6dof_parity_trace_has_the_same_public_cli_and_python_verdict(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("x8_racetrack_capability_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    calls = 0

    def policy(observation, contract):
        del observation, contract
        nonlocal calls
        calls += 1
        return PolicyDecision({"propulsion.command.fraction": 0.5}, 0.1) if calls == 1 else None
        ####

    trace = run_composition_policy(episode, policy, authority_profile_id="native_control_bridge")
    composition_path = tmp_path / "x8-pseudo-composition.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    trace_path = write_composition_policy_trace(trace, tmp_path / "x8-pseudo-trace.json")

    assert main(["vehicle", "batch-episode-parity", str(composition_path), str(trace_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    assert payload["interface_id"].endswith("/pseudo_6dof")
    ####


def test_parity_refuses_an_unregistered_family_instead_of_substituting_hummingbird_dynamics() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    trace = run_composition_policy(
        episode,
        lambda observation, contract: None,
        authority_profile_id="native_control_bridge",
    )

    with pytest.raises(ValueError, match="no batch/episode action-trace parity adapter"):
        verify_composition_batch_episode_parity(composition, trace)
    ####
