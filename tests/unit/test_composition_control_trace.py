"""Tests for committed semantic action-trace artifacts."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition_control_trace import (
    BatchControlSample,
    build_committed_control_trace,
    build_uncontrolled_committed_control_trace,
    validate_committed_control_trace,
    validate_committed_control_trace_against_status,
)
from taoryx.composition_sensor_trace import BatchTruthSample
from taoryx.composition_status_trace import build_committed_status_trace
from taoryx.vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _hummingbird_composition() -> CompiledVehicleComposition:
    return compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )
    ####


def _nesc_composition() -> CompiledVehicleComposition:
    return compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/nesc_staged_source_replay_3dof_compose.yaml")
    )
    ####


def _sample() -> BatchControlSample:
    return BatchControlSample(
        interval_start_time_s=0.0,
        committed_truth_time_s=0.02,
        requested_actions={
            "attitude.roll.command": 0.1,
            "attitude.pitch.command": -0.2,
            "attitude.yaw.command": 0.3,
            "propulsion.command.fraction": 0.6,
            "propulsion.enable": True,
        },
        achieved_effectors={},
    )
    ####


def _x15_truth_sample(time_s: float, *, phase: str) -> BatchTruthSample:
    """Supply the complete source-owned status row for an action-free replay."""

    return BatchTruthSample(
        time_s=time_s,
        execution_status="completed" if phase == "glide" else "active",
        raw_values={
            "time_s": time_s,
            "position_m": [100.0 + time_s, 200.0, 10_000.0 - time_s],
            "velocity_m_s": [500.0, 0.0, -5.0],
            "mass_kg": 10_000.0 - time_s,
            "phase": phase,
            "attitude_rad": [0.1, 0.2, 0.3],
            "attitude_rate_rad_s": [0.01, 0.02, 0.03],
        },
    )
    ####


def test_committed_control_trace_preserves_held_actions_without_inventing_effectors() -> None:
    composition = _hummingbird_composition()
    trace = build_committed_control_trace(composition, (_sample(),))

    validate_committed_control_trace(composition, trace)

    assert trace["sampling"] == "held_action_interval_to_committed_truth_boundary"
    assert trace["authority_profile_id"] == "body_motion_response"
    assert trace["command_owner"] == "caller"
    assert trace["lowering_chain"] == [
        "held_attitude_and_aggregate_thrust",
        "hummingbird_pseudo_6dof_response_law",
    ]
    assert trace["achieved_effector_channels"] == []
    sample = trace["samples"][0]
    assert sample["requested_actions"]["propulsion.enable"] is True
    ####


def test_committed_control_trace_rejects_missing_declared_action() -> None:
    composition = _hummingbird_composition()
    incomplete = BatchControlSample(
        0.0,
        0.02,
        {"attitude.roll.command": 0.0},
        {},
    )

    with pytest.raises(ValueError, match="omits declared channel"):
        build_committed_control_trace(composition, (incomplete,))
    ####


def test_committed_control_trace_rejects_a_different_composition_identity() -> None:
    composition = _hummingbird_composition()
    trace = build_committed_control_trace(composition, (_sample(),))
    trace["composition_identity_sha256"] = "0" * 64

    with pytest.raises(ValueError, match="fingerprint"):
        validate_committed_control_trace(composition, trace)
    ####


def test_uncontrolled_control_trace_proves_absence_of_public_actions_without_fabricating_commands() -> None:
    composition = _nesc_composition()
    trace = build_uncontrolled_committed_control_trace(composition, (0.0, 1.0, 2.0))

    validate_committed_control_trace(composition, trace)

    assert trace["requested_action_channels"] == []
    assert trace["achieved_effector_channels"] == []
    assert [sample["requested_actions"] for sample in trace["samples"]] == [{}, {}, {}]
    assert trace["samples"][1]["interval_start_time_s"] == 0.0
    ####


def test_committed_control_trace_requires_actual_status_boundaries() -> None:
    """A locally ordered action trace cannot invent an uncommitted interval."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    )
    status_trace = build_committed_status_trace(
        composition,
        (_x15_truth_sample(0.0, phase="boost"), _x15_truth_sample(25.0, phase="glide")),
    )
    trace = build_uncontrolled_committed_control_trace(composition, (0.0, 25.0))

    validate_committed_control_trace_against_status(composition, trace, status_trace=status_trace)

    samples = trace["samples"]
    assert isinstance(samples, list)
    second = samples[1]
    assert isinstance(second, dict)
    second["interval_start_time_s"] = 12.5
    with pytest.raises(ValueError, match="not a committed status boundary"):
        validate_committed_control_trace_against_status(composition, trace, status_trace=status_trace)
    ####
