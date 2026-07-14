from __future__ import annotations

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.common import RuntimeState
from taoryx.state import PointMassRates, PointMassState


def _state() -> PointMassState:
    return PointMassState(
        12.0,
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(4.0, 5.0, 6.0), Frame.ECFC),
        7.0,
        8.0,
        9.0,
    )


def test_point_mass_state_has_documented_augmented_order() -> None:
    state = _state()

    assert PointMassState.STATE_NAMES == (
        "x_ecfc", "y_ecfc", "z_ecfc", "xdot_ecfc", "ydot_ecfc", "zdot_ecfc", "mass", "path_length", "ground_range"
    )
    assert state.to_values() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)


def test_point_mass_state_round_trips_through_generic_integrator_contract() -> None:
    state = _state()

    assert PointMassState.from_simulation_state(state.to_simulation_state()) == state


def test_point_mass_state_rejects_wrong_frame_or_dimension() -> None:
    with pytest.raises(ValueError, match="ECFC"):
        PointMassState(0.0, FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC), FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC), 1.0)
    with pytest.raises(ValueError, match="requires 9"):
        PointMassState.from_values(0.0, [0.0] * 8)


def test_point_mass_rates_match_state_order() -> None:
    rates = PointMassRates(
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(4.0, 5.0, 6.0), Frame.ECFC),
        -1.0,
        7.0,
        8.0,
    )

    assert rates.to_values() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -1.0, 7.0, 8.0)


def test_runtime_state_adapter_preserves_canonical_physical_state() -> None:
    state = _state()

    runtime = RuntimeState.from_point_mass_state(state)

    assert runtime.to_point_mass_state() == state
