from __future__ import annotations

import math

import pytest

from taoryx.contracts import EarthModel, Frame, FrameVector3, Quantity, Unit, Vector3
from taoryx.dynamics import assemble_point_mass_rates
from taoryx.integration import rk4_step
from taoryx.modes import Quaternion
from taoryx.rigid_body import RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment
from taoryx.runtime.lowering import _runtime_propnav_command, _runtime_pure_propnav_active
from taoryx.simulation.contracts import SimulationState
from taoryx.state import PointMassState

pytestmark = pytest.mark.algorithms


def test_terminal_guidance_can_select_pure_propnav_without_route_following() -> None:
    assert not _runtime_pure_propnav_active({"terminal-guidance-mode": "route", "terminal-start-s": "10"}, 10.0)
    assert _runtime_pure_propnav_active({"terminal-guidance-mode": "propnav", "terminal-start-s": "10"}, 10.0)
    ####


def test_propnav_command_reports_moving_earth_target_los_state() -> None:
    state = RigidBody6DofState(
        0.0,
        FrameVector3(Vector3(7_000_000.0, 0.0, 0.0), Frame.ECIC),
        FrameVector3(Vector3(0.0, 7_500.0, 0.0), Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        1_000.0,
        0.0,
    )
    command = _runtime_propnav_command(
        state,
        {"latitude-deg": "0.0", "longitude-deg": "10.0", "altitude-m": "0.0"},
        4.0,
        3.986004418e14,
        7.2921151467e-5,
    )

    assert float(command["range"]) > 0.0
    assert math.isfinite(float(command["closing"]))
    assert math.isfinite(float(command["azimuth_rate"]))
    assert math.isfinite(cast_vector(command["demand"]).norm())
    ####


def test_rigid_body_translation_matches_independent_point_mass_reference() -> None:
    earth = EarthModel(
        Quantity(6_378_137.0, Unit.METER),
        0.0,
        Quantity(0.0, Unit.METER_CUBED_PER_SECOND_SQUARED),
        Quantity(0.0, Unit.RADIAN_PER_SECOND),
    )
    force = FrameVector3(Vector3(10.0, 20.0, 30.0), Frame.ECFC)
    point = PointMassState(
        0.0,
        FrameVector3(Vector3(1_000_000.0, 2_000_000.0, 3_000_000.0), Frame.ECFC),
        FrameVector3(Vector3(100.0, 200.0, 300.0), Frame.ECFC),
        1_000.0,
    )
    point_rates = assemble_point_mass_rates(point, force, 0.0, point.earth_relative_velocity.vector.norm(), earth)
    rigid = RigidBody6DofState(
        0.0,
        FrameVector3(point.position.vector, Frame.ECIC),
        FrameVector3(point.earth_relative_velocity.vector, Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        point.mass,
        0.0,
    )
    model = RigidBody6DofModel(
        Vector3(10.0, 20.0, 30.0),
        lambda _state: RigidBodyForceMoment(force.vector, Vector3(0.0, 0.0, 0.0)),
        gravity=lambda _state: Vector3(0.0, 0.0, 0.0),
    )

    rigid_rates = model.derivative(rigid)
    assert rigid_rates[:6] == pytest.approx(point_rates.to_core_values())

    def point_derivative(simulation: SimulationState) -> tuple[float, ...]:
        current = PointMassState.from_simulation_state(simulation)
        return assemble_point_mass_rates(current, force, 0.0, current.earth_relative_velocity.vector.norm(), earth).to_values()
        ####

    point_step = rk4_step(point_derivative, point.to_simulation_state(), 0.5)
    assert point_step.values[:6] == pytest.approx(
        (
            1_000_000.0 + 50.00125,
            2_000_000.0 + 100.0025,
            3_000_000.0 + 150.00375,
            100.0 + 0.005,
            200.0 + 0.01,
            300.0 + 0.015,
        )
    )
    ####


def cast_vector(value: object) -> Vector3:
    assert isinstance(value, Vector3)
    return value
    ####
