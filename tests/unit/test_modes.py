from __future__ import annotations

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.language.problem_parser import parse_problem_text
from taoryx.modes import DynamicsMode, Kinematic6DofState, Quaternion
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import compute_trajectories, integrate_active_vehicles
from taoryx.runtime.lowering import _dynamics_mode, _unsupported_features, lower_problem_document
from taoryx.state import PointMassRates, PointMassState


def test_mode_selector_is_explicit_and_defaults_are_not_ambiguous() -> None:
    document = parse_problem_text("(demo)\n*mode kinematic-6dof\n*end\n")

    assert document.diagnostics == []
    assert document.problems[0].blocks[0].mode == "kinematic-6dof"
    assert DynamicsMode.KINEMATIC_6DOF.value == "kinematic-6dof"


@pytest.mark.parametrize("mode", tuple(DynamicsMode))
def test_every_declared_mode_is_lexically_recognized(mode: DynamicsMode) -> None:
    document = parse_problem_text(f"(demo)\n*mode {mode.value}\n*end\n")

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    assert document.problems[0].blocks[0].mode == mode.value


def test_omitted_mode_is_manual_compatible_point_mass() -> None:
    document = parse_problem_text("(demo)\n*end\n")

    assert _dynamics_mode(document.problems[0]) is DynamicsMode.POINT_MASS


def test_rigid_body_mode_is_rejected_instead_of_falling_back() -> None:
    document = parse_problem_text("(demo)\n*mode rigid-body-6dof\n*end\n")

    assert _unsupported_features(document.problems[0]) == ("mode rigid-body-6dof",)


@pytest.mark.parametrize(("directive", "expected"), (("3dof", DynamicsMode.POINT_MASS), ("6dof", DynamicsMode.RIGID_BODY_6DOF)))
def test_successor_dof_directive_selects_runtime_mode(directive: str, expected: DynamicsMode) -> None:
    document = parse_problem_text(f"(demo)\n*{directive}\n*end\n", profile="taoryx")

    assert _dynamics_mode(document.problems[0]) is expected


def test_successor_6dof_directive_lowers_to_rigid_body_runtime() -> None:
    document = parse_problem_text(
        "(rigid-body-smoke)\n"
        "*6dof\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecic x=20925646.3255 y=0 z=0 xdt=0 ydt=300 zdt=0 time=0 mass=100\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        profile="taoryx",
    )

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    lowered = lower_problem_document(document)

    assert lowered.unsupported_features == ()
    vehicle = lowered.cases[0].problem.vehicles["1"]
    assert vehicle.dynamics_mode is DynamicsMode.RIGID_BODY_6DOF
    assert vehicle.state.value_names[6:10] == ("qw", "qx", "qy", "qz")
    result = compute_trajectories(lowered.cases[0].problem, max_steps=10)
    assert result.stop_reason == "stop_condition"
    assert result.states["1"][-1].time == pytest.approx(0.2)


def test_kinematic_problem_mode_tracks_a_lagged_prescribed_attitude() -> None:
    document = parse_problem_text(
        "(kinematic-bridge-smoke)\n"
        "*mode kinematic-6dof\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=30 lag-s=0.5 max-rate-deg-s=180\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecic x=7000000 y=0 z=0 xdt=0 ydt=100 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dtprnt=0.1 dt=0.1\n"
        "    *when time>1.0 stop\n"
        "*end\n",
        profile="taoryx",
    )

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    lowered = lower_problem_document(document)
    assert lowered.unsupported_features == ()
    vehicle = lowered.cases[0].problem.vehicles["1"]
    assert vehicle.dynamics_mode is DynamicsMode.KINEMATIC_6DOF
    assert vehicle.kinematic_state is not None
    result = compute_trajectories(lowered.cases[0].problem, max_steps=20)
    history = result.states["1"]
    assert history[-1].time == pytest.approx(1.0)
    assert history[-1].named["qz"] > 0.0
    assert history[-1].named["qz"] < 0.3
    assert history[-1].named["qw"] > 0.9


def test_kinematic_problem_mode_accepts_prescribed_body_rates() -> None:
    document = parse_problem_text(
        "(kinematic-rate-smoke)\n"
        "*mode kinematic-6dof\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime status attitude mode=rate yaw-rate-deg-s=90\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecic x=7000000 y=0 z=0 xdt=0 ydt=100 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dtprnt=0.1 dt=0.1\n"
        "    *when time>0.5 stop\n"
        "*end\n",
        profile="taoryx",
    )

    lowered = lower_problem_document(document)
    result = compute_trajectories(lowered.cases[0].problem, max_steps=10)
    final = result.states["1"][-1].named
    assert final["qz"] > 0.1


def test_unknown_mode_is_source_located() -> None:
    document = parse_problem_text("(demo)\n*mode six-dof\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "invalid-dynamics-mode")
    assert diagnostic.location.line == 2


def test_kinematic_attitude_propagates_controller_rate_and_stays_normalized() -> None:
    state = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC),
        Quaternion.identity(),
    )

    advanced = state.with_attitude_rate(Vector3(0.0, 0.0, 1.0), 0.1)

    norm = sum(value * value for value in (advanced.attitude.w, advanced.attitude.x, advanced.attitude.y, advanced.attitude.z))
    assert advanced.time == pytest.approx(0.1)
    assert norm == pytest.approx(1.0)
    assert advanced.attitude.z > 0.0


def test_zero_body_rate_preserves_attitude() -> None:
    state = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(4.0, 5.0, 6.0), Frame.ECFC),
        Quaternion(0.8, 0.0, 0.6, 0.0).normalized(),
    )

    advanced = state.with_attitude_rate(Vector3(0.0, 0.0, 0.0), 0.25)

    assert advanced.attitude == state.attitude
    assert advanced.position == state.position
    assert advanced.velocity == state.velocity


def test_kinematic_state_requires_ecfc_vectors() -> None:
    with pytest.raises(ValueError, match="must use ECFC"):
        Kinematic6DofState(
            0.0,
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.BODY),
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        )


def test_kinematic_state_rejects_nonpositive_attitude_step() -> None:
    state = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
    )

    with pytest.raises(ValueError, match="step size"):
        state.with_attitude_rate(Vector3(0.0, 0.0, 1.0), 0.0)


def test_runtime_kinematic_mode_advances_controller_attitude_sidecar() -> None:
    kinematic = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC),
    )
    vehicle = RuntimeVehicle(
        "demo",
        RuntimeState(0.0, (0.0,), named={"time": 0.0}),
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=kinematic,
        body_rate_provider=lambda state: Vector3(0.0, 0.0, 1.0),
    )

    integrate_active_vehicles(RuntimeProblem({"demo": vehicle}), 0.1)

    assert vehicle.kinematic_state is not None
    assert vehicle.kinematic_state.time == pytest.approx(0.1)
    assert vehicle.kinematic_state.attitude.z > 0.0


def test_kinematic_attitude_sidecar_does_not_mutate_translational_state() -> None:
    kinematic = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC),
    )
    vehicle = RuntimeVehicle(
        "demo",
        RuntimeState(0.0, (10.0, 20.0), named={"time": 0.0}),
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=kinematic,
        body_rate_provider=lambda state: Vector3(0.0, 0.0, 1.0),
    )

    integrate_active_vehicles(RuntimeProblem({"demo": vehicle}), 0.1)

    assert vehicle.state.values == (10.0, 20.0)
    assert vehicle.state.time == pytest.approx(0.1)
    assert vehicle.kinematic_state is not None
    assert vehicle.kinematic_state.attitude.z > 0.0


def test_point_mass_mode_integrates_constant_force_into_translation() -> None:
    initial = PointMassState.from_core_values(0.0, [0.0] * 6, mass=1.0)

    def force_derivative(state: PointMassState) -> PointMassRates:
        return PointMassRates(
            FrameVector3(state.earth_relative_velocity.vector, Frame.ECFC),
            FrameVector3(Vector3(2.0, 0.0, 0.0), Frame.ECFC),
            0.0,
            0.0,
            0.0,
        )
        ####
    ####

    vehicle = RuntimeVehicle("point", RuntimeState.from_point_mass_state(initial), point_mass_derivative=force_derivative, integrator="rk4")
    integrate_active_vehicles(RuntimeProblem({"point": vehicle}), 1.0)
    result = vehicle.state.to_point_mass_state()

    assert result.position.vector.x == pytest.approx(1.0)
    assert result.earth_relative_velocity.vector.x == pytest.approx(2.0)


def test_kinematic_mode_integrates_force_and_controller_rate_as_separate_channels() -> None:
    initial = PointMassState.from_core_values(0.0, [0.0] * 6, mass=1.0)
    kinematic = Kinematic6DofState(
        initial.time,
        initial.position,
        initial.earth_relative_velocity,
    )

    def force_derivative(state: PointMassState) -> PointMassRates:
        return PointMassRates(
            FrameVector3(state.earth_relative_velocity.vector, Frame.ECFC),
            FrameVector3(Vector3(2.0, 0.0, 0.0), Frame.ECFC),
            0.0,
            0.0,
            0.0,
        )
        ####
    ####

    vehicle = RuntimeVehicle(
        "kinematic",
        RuntimeState.from_point_mass_state(initial),
        point_mass_derivative=force_derivative,
        integrator="rk4",
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=kinematic,
        body_rate_provider=lambda state: Vector3(0.0, 0.0, 1.0),
    )
    integrate_active_vehicles(RuntimeProblem({"kinematic": vehicle}), 1.0)

    assert vehicle.kinematic_state is not None
    assert vehicle.kinematic_state.position.vector.x == pytest.approx(1.0)
    assert vehicle.kinematic_state.velocity.vector.x == pytest.approx(2.0)
    assert vehicle.kinematic_state.attitude.z > 0.0
