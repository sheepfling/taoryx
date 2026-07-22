from __future__ import annotations

import pytest

from taoryx.validation import PhaseWindow, continuity_audit, require_bounded, require_change_of_sign, require_monotonic, require_net_change

HISTORY = tuple({"time_s": float(index), "altitude_m": float(10 - index), "signed": float(index - 2)} for index in range(5))


def test_phase_window_selects_samples_by_runtime_time() -> None:
    phase = PhaseWindow("descent", 1.0, 3.0)

    assert [sample["time_s"] for sample in phase.select(HISTORY)] == [1.0, 2.0, 3.0]


def test_phase_window_rejects_empty_or_invalid_ranges() -> None:
    with pytest.raises(ValueError):
        PhaseWindow("", 0.0, 1.0)
    with pytest.raises(ValueError):
        PhaseWindow("bad", 2.0, 2.0)


def test_monotonic_and_bounded_checks() -> None:
    require_monotonic(HISTORY, "altitude_m", "decreasing")
    require_bounded(HISTORY, "altitude_m", minimum=6.0, maximum=10.0)


def test_monotonic_check_reports_directional_failure() -> None:
    with pytest.raises(AssertionError, match="altitude_m"):
        require_monotonic(HISTORY, "altitude_m", "increasing")


def test_net_change_checks_phase_displacement_without_overconstraining_samples() -> None:
    require_net_change(HISTORY, "altitude_m", "decreasing", minimum=2.0)


def test_sign_change_check_catches_bank_reversal_shape() -> None:
    require_change_of_sign(HISTORY, "signed", minimum_before=1.0, minimum_after=1.0)


def test_continuity_audit_rejects_unexplained_state_jump() -> None:
    history = tuple({"time_s": float(index), "position_m": value} for index, value in enumerate((0.0, 1.0, 9.0, 10.0)))

    report = continuity_audit(history, {"position_m": 2.0})

    assert report["passed"] is False
    assert len(report["unexplained_violations"]) == 1
    assert report["unexplained_violations"][0]["left_time_s"] == 1.0


def test_continuity_audit_allows_declared_event_jump() -> None:
    history = tuple({"time_s": float(index), "position_m": value} for index, value in enumerate((0.0, 1.0, 9.0, 10.0)))

    report = continuity_audit(history, {"position_m": 2.0}, event_times=(2.0,))

    assert report["passed"] is True
    assert len(report["declared_event_jumps"]) == 1
