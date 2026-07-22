from __future__ import annotations

import pytest

from taoryx.control_directions import ControlDirectionProbe, audit_control_directions


def test_direction_harness_checks_signed_effect_about_nonzero_baseline() -> None:
    def evaluator(commands: dict[str, float]) -> dict[str, float]:
        return {"pitch_moment": 10.0 + 2.0 * commands["elevator"]}

    results = audit_control_directions(
        evaluator,
        {"elevator": 0.0},
        (ControlDirectionProbe("elevator", "pitch_moment", 0.1, expected_sign=1),),
    )

    assert results[0].passed
    assert results[0].derivative == pytest.approx(2.0)
    assert results[0].antisymmetry_error == pytest.approx(0.0)


def test_direction_harness_detects_flipped_sign() -> None:
    def evaluator(commands: dict[str, float]) -> dict[str, float]:
        return {"yaw_moment": -commands["rudder"]}

    result = audit_control_directions(
        evaluator,
        {"rudder": 0.0},
        (ControlDirectionProbe("rudder", "yaw_moment", 0.1, expected_sign=1),),
    )[0]

    assert not result.passed
    assert result.derivative == pytest.approx(-1.0)


def test_direction_harness_supports_expected_decoupling() -> None:
    result = audit_control_directions(
        lambda commands: {"side_force": 0.0},
        {"collective_elevon": 0.0},
        (ControlDirectionProbe("collective_elevon", "side_force", 1.0, expected_sign=0),),
    )[0]

    assert result.passed
