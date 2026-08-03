from __future__ import annotations

import json

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.control import ProportionalHoldController, SegmentController, SegmentPlan, SegmentSchedule
from taoryx.modes import DynamicsMode, Kinematic6DofState
from taoryx.runtime import (
    ControlSpec,
    EventAction,
    EventSpec,
    InteractiveSession,
    InteractiveStatus,
    OutputSubscription,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    StatusSpec,
)
from taoryx.runtime.engine import compute_trajectories

pytestmark = pytest.mark.algorithms


def _problem() -> RuntimeProblem:
    def derivative(state: RuntimeState) -> tuple[float, float]:
        return state.named.get("throttle", 0.0), state.named.get("fin_pitch", 0.0)

    vehicle = RuntimeVehicle(
        "player",
        RuntimeState(0.0, (0.0, 0.0), value_names=("downrange", "pitch")),
        derivative=derivative,
        step_size=0.1,
    )
    return RuntimeProblem({vehicle.name: vehicle})
####


def _controls() -> tuple[ControlSpec, ...]:
    return (
        ControlSpec("throttle", unit="fraction", default=0.0, lower=0.0, upper=1.0),
        ControlSpec("fin_pitch", unit="normalized", default=0.0, lower=-1.0, upper=1.0, slew_rate=2.0),
    )
####


def test_step_applies_bounded_named_commands_and_routes_them_to_derivative() -> None:
    session = InteractiveSession(_problem(), _controls())

    snapshot = session.step(0.1, {"throttle": 2.0, "fin_pitch": 1.0})

    assert snapshot.time_end == pytest.approx(0.1)
    assert snapshot.states["player"].values == pytest.approx((0.1, 0.02))
    assert [(item.name, item.applied, item.clamped) for item in snapshot.commands] == [
        ("throttle", 1.0, True),
        ("fin_pitch", 0.2, True),
    ]
####


def test_load_evaluations_preserve_state_and_control_activation_timing() -> None:
    """A command boundary is retained through every private solver evaluation."""

    vehicle = RuntimeVehicle(
        "timed-player",
        RuntimeState(0.0, (0.0,), value_names=("downrange",)),
        derivative=lambda state: (state.named.get("throttle", 0.0),),
        environment_evaluator=lambda _values: {"density": 1.225},
        step_size=0.1,
    )
    session = InteractiveSession(
        RuntimeProblem({vehicle.name: vehicle}),
        (ControlSpec("throttle", lower=0.0, upper=1.0),),
    )

    session.step(0.1, {"throttle": 0.6})
    first_records = tuple(vehicle.load_evaluation_history)
    assert first_records
    assert {record.phase for record in first_records} >= {
        "solver_stage_environment",
        "solver_stage_rhs",
        "committed_truth_environment",
        "committed_truth_rhs",
    }
    assert all(record.achieved_control_time_s == pytest.approx(0.0) for record in first_records)
    assert all(record.achieved_control_time_s <= record.state_time_s for record in first_records)
    assert max(record.state_time_s for record in first_records) == pytest.approx(0.1)

    session.step(0.1, {"throttle": 0.2})
    second_records = vehicle.load_evaluation_history[len(first_records) :]
    assert second_records
    assert all(record.achieved_control_time_s == pytest.approx(0.1) for record in second_records)
    assert all(record.achieved_control_time_s <= record.state_time_s for record in second_records)
####


def test_adaptive_interactive_step_reaches_the_requested_accepted_truth_boundary() -> None:
    """An adaptive inner step may shrink, but must not shorten the API step."""

    vehicle = RuntimeVehicle(
        "adaptive",
        RuntimeState(0.0, (1.0,), value_names=("state",)),
        derivative=lambda state: (25.0 * state.values[0],),
        step_size=0.05,
        integrator="rkf45",
        absolute_tolerance=1.0e-12,
        relative_tolerance=1.0e-12,
    )
    session = InteractiveSession(RuntimeProblem({vehicle.name: vehicle}))

    snapshot = session.step(0.1)

    assert snapshot.time_start == pytest.approx(0.0)
    assert snapshot.time_end == pytest.approx(0.1)
    assert vehicle.state.time == pytest.approx(0.1)
    assert len(vehicle.history) > 2
    ####


def test_pause_resume_interrupt_and_artifact_are_explicit(tmp_path) -> None:
    session = InteractiveSession(_problem(), _controls())
    session.step(0.1, {"throttle": 0.5})
    session.pause()
    assert session.status is InteractiveStatus.PAUSED
    session.resume()
    session.step(0.1, {"throttle": 0.5})
    session.interrupt()
    assert session.status is InteractiveStatus.INTERRUPTED
    artifact = session.artifact()
    destination = artifact.write_json(tmp_path / "interactive-artifact.json")
    assert destination.exists()
    batch_artifact = session.to_run_artifact()
    assert batch_artifact.vehicles["player"].times == pytest.approx((0.0, 0.1, 0.2))
    assert batch_artifact.vehicles["player"].channels["downrange"].values == pytest.approx((0.0, 0.05, 0.1))
    with pytest.raises(RuntimeError, match="cannot step"):
        session.step(0.1)
####


def test_command_stream_replay_is_deterministic() -> None:
    first = InteractiveSession(_problem(), _controls())
    first.step(0.1, {"throttle": 0.3})
    first.step(0.1, {"throttle": 0.8, "fin_pitch": -0.4})

    replay = InteractiveSession(_problem(), _controls())
    snapshots = replay.replay(first.command_history)

    assert [snapshot.as_dict() for snapshot in snapshots] == [snapshot.as_dict() for snapshot in first.snapshots]
####


def test_unknown_and_invalid_commands_are_rejected() -> None:
    session = InteractiveSession(_problem(), _controls())
    with pytest.raises(ValueError, match="unknown interactive control"):
        session.step(0.1, {"rudder": 0.2})
    with pytest.raises(ValueError, match="must be finite"):
        session.step(0.1, {"throttle": float("nan")})
####


def test_control_model_transforms_player_command_before_derivative() -> None:
    problem = _problem()
    problem.vehicles["player"].derivative = lambda state: (state.named.get("elevator", 0.0), 0.0)
    session = InteractiveSession(
        problem,
        (ControlSpec("stick", lower=-1.0, upper=1.0),),
        control_model=lambda _vehicle, _state, commands: {"elevator": 2.0 * commands["stick"]},
    )

    snapshot = session.step(0.1, {"stick": 0.25})

    assert snapshot.states["player"].values[0] == pytest.approx(0.05)


def test_segment_controller_drives_time_steppable_runtime() -> None:
    segment_controller = SegmentController(
        ProportionalHoldController({"downrange": "throttle"}, {"downrange": 2.0}),
        SegmentSchedule((SegmentPlan("capture", 0.0, 1.0, {"downrange": 1.0}),)),
    )
    session = InteractiveSession(
        _problem(),
        (ControlSpec("throttle", lower=0.0, upper=1.0),),
        segment_controllers={"player": segment_controller},
    )

    snapshot = session.step(0.1)

    assert snapshot.states["player"].values[0] == pytest.approx(0.1)
    assert snapshot.commands[0].applied == pytest.approx(1.0)
    assert session.command_history[0].commands == {"throttle": 2.0}
####


def test_interactive_session_advances_kinematic_6dof_sidecar() -> None:
    vehicle = RuntimeVehicle(
        "six-dof",
        RuntimeState(0.0, (0.0,), value_names=("energy",)),
        derivative=lambda _state: (1.0,),
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=Kinematic6DofState(
            0.0,
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
            FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC),
        ),
        body_rate_provider=lambda _state: Vector3(0.0, 0.0, 1.0),
    )
    session = InteractiveSession(RuntimeProblem({vehicle.name: vehicle}))

    snapshot = session.step(0.1)

    assert snapshot.states["six-dof"].values == pytest.approx((0.1,))
    assert vehicle.kinematic_state is not None
    assert vehicle.kinematic_state.time == pytest.approx(0.1)
    assert vehicle.kinematic_state.attitude.z > 0.0


def test_interactive_signal_status_and_output_contracts_are_artifact_visible() -> None:
    session = InteractiveSession(
        _problem(),
        _controls(),
        status_specs=(StatusSpec("downrange_status", source="downrange", unit="m"),),
        event_specs=(EventSpec("crossed", lambda state: state.named.get("downrange", 0.0) >= 0.05, EventAction.SIGNAL, "crossed-boundary"),),
        output_subscriptions=(OutputSubscription(("position.downrange",), sample_interval=0.1),),
    )

    snapshot = session.step(0.1, {"throttle": 0.5})

    assert snapshot.runtime_events[0].signal == "crossed-boundary"
    assert snapshot.statuses["player"]["downrange_status"] == pytest.approx(0.05)
    artifact = session.to_run_artifact()
    assert artifact.commands[0]["duration"] == 0.1
    assert artifact.events[0]["action"] == "signal"
    assert artifact.visualization["subscriptions"]


def test_batch_and_interactive_replay_share_the_same_state_contract() -> None:
    batch = _problem()
    batch_result = compute_trajectories(batch, max_steps=2)

    interactive = InteractiveSession(_problem())
    interactive.step(0.1)
    interactive.step(0.1)

    assert batch_result.states["player"][-1].values == pytest.approx(interactive.problem.vehicles["player"].state.values)
    assert batch_result.states["player"][-1].time == pytest.approx(interactive.time)


def test_interactive_checkpoint_restores_command_history_and_continues(tmp_path) -> None:
    first = InteractiveSession(_problem(), _controls())
    first.step(0.1, {"throttle": 0.3})
    first.pause()
    checkpoint = first.save_checkpoint(tmp_path / "interactive.checkpoint.json", model_fingerprint="demo-model-v1")

    restored = InteractiveSession.load_checkpoint(
        checkpoint,
        _problem(),
        model_fingerprint="demo-model-v1",
        controls=_controls(),
    )
    assert restored.status is InteractiveStatus.PAUSED
    assert restored.time == pytest.approx(0.1)
    assert restored.command_history == first.command_history
    assert restored.problem.vehicles["player"].state.values == pytest.approx(first.problem.vehicles["player"].state.values)

    first.resume()
    restored.resume()
    expected = first.step(0.1, {"throttle": 0.8})
    actual = restored.step(0.1, {"throttle": 0.8})
    assert actual.as_dict() == expected.as_dict()
    assert restored.command_history == first.command_history


def test_interactive_checkpoint_rejects_tampering_and_wrong_model(tmp_path) -> None:
    session = InteractiveSession(_problem(), _controls())
    checkpoint = session.save_checkpoint(tmp_path / "interactive-integrity.json", model_fingerprint="model-a")

    with pytest.raises(ValueError, match="model fingerprint"):
        InteractiveSession.load_checkpoint(checkpoint, _problem(), model_fingerprint="model-b", controls=_controls())

    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["session"]["status"] = "interrupted"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="integrity"):
        InteractiveSession.load_checkpoint(checkpoint, _problem(), model_fingerprint="model-a", controls=_controls())
####
