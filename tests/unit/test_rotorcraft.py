from __future__ import annotations

import pytest

from taoryx.contracts import Vector3
from taoryx.rotorcraft import QuadRotorAllocation, RotorCommandSet


def test_quad_allocation_round_trips_collective_and_moment() -> None:
    allocation = QuadRotorAllocation(0.17, 5.57e-6, 1.36e-7)
    commands = allocation.allocate(469.124102661955, Vector3(0.002, -0.003, 0.0004))
    assert commands.rms_speed_rad_s == pytest.approx(469.124102661955, rel=1.0e-6)
    moment = allocation.differential_moment_source(commands)
    assert moment.x == pytest.approx(0.002, abs=1.0e-7)
    assert moment.y == pytest.approx(-0.003, abs=1.0e-7)
    assert moment.z == pytest.approx(0.0004, abs=1.0e-7)
    ####


def test_rotor_command_set_rejects_invalid_speed() -> None:
    with pytest.raises(ValueError, match="finite and nonnegative"):
        RotorCommandSet(-1.0, 1.0, 1.0, 1.0)
    ####


def test_individual_rotor_source_model_matches_the_declared_hover_and_roll_response() -> None:
    """The stronger source mode evaluates every physical rotor, not a wrench bridge."""

    allocation = QuadRotorAllocation(
        0.17,
        5.57e-6,
        1.36e-7,
        rotor_drag_xy_coefficient_n_s_per_m=1.19e-4,
        rotor_drag_z_coefficient_n_s_per_m=2.32e-4,
        translational_lift_coefficient_n_s2_per_m2=3.39e-3,
        frame_drag_coefficients_n_s2_per_m2=Vector3(0.005, 0.005, 0.010),
    )
    hover = RotorCommandSet(*(469.124102661955 for _ in range(4)))
    hover_force, hover_moment = allocation.source_force_moment(hover, Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    assert allocation.individual_rotor_source_available
    assert hover_force.x == pytest.approx(0.0)
    assert hover_force.y == pytest.approx(0.0)
    assert hover_force.z == pytest.approx(4.903325, rel=1.0e-12)
    assert hover_moment.x == pytest.approx(0.0, abs=1.0e-12)
    assert hover_moment.y == pytest.approx(0.0, abs=1.0e-12)
    assert hover_moment.z == pytest.approx(0.0, abs=1.0e-12)

    # This is the published generated +100 rad/s roll-response fixture:
    # [569.124, 369.124, 369.124, 569.124] rad/s.
    roll = RotorCommandSet(569.124102661955, 369.124102661955, 369.124102661955, 569.124102661955)
    roll_force, roll_moment = allocation.source_force_moment(roll, Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    assert roll_force.z == pytest.approx(5.126125, rel=1.0e-12)
    assert roll_moment.x == pytest.approx(0.251285166331, rel=1.0e-12)
    assert roll_moment.y == pytest.approx(0.0, abs=1.0e-12)
    assert roll_moment.z == pytest.approx(0.0, abs=1.0e-12)
    ####
