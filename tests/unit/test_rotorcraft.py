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
