from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.trajectory import (
    ControlArbitrator,
    ControlAuthorityError,
    ControlFrame,
    ControlSchema,
    ReferencePointMassProvider,
    load_case_intent,
    load_family_catalog,
    resolve_case,
)
from taoryx.trajectory.taoryx_adapter import TaoryxPointMassAdapter

ROOT = Path(__file__).resolve().parents[2]
CATALOG = load_family_catalog(ROOT / "verification" / "alpha2_family_catalog.yaml")
CASE = resolve_case(
    load_case_intent(ROOT / "tests" / "fixtures" / "alpha2_case_contracts" / "case-heavy.yaml"),
    CATALOG,
)
####


def _schema(**updates: object) -> ControlSchema:
    values: dict[str, object] = {
        "id": "control.test",
        "minimum": -1.0,
        "maximum": 1.0,
        "authority_modes": ("autopilot", "commanded", "overlay", "direct", "mixed"),
        "default_authority": "commanded",
        "hold_behavior": "hold",
    }
    values.update(updates)
    return ControlSchema.model_validate(values)
####


def test_authority_modes_select_the_declared_source() -> None:
    arbitrator = ControlArbitrator((_schema(),))

    assert arbitrator.apply(0.0, 0.1, ControlFrame(autopilot={"control.test": 0.2})).values["control.test"] == pytest.approx(0.0)
    assert (
        arbitrator.apply(
            0.1,
            0.1,
            ControlFrame(values={"control.test": 0.3}, autopilot={"control.test": 0.2}, authority={"control.test": "autopilot"}),
        ).values["control.test"]
        == pytest.approx(0.2)
    )
    assert (
        arbitrator.apply(
            0.2,
            0.1,
            ControlFrame(values={"control.test": 0.1}, autopilot={"control.test": 0.2}, authority={"control.test": "overlay"}),
        ).values["control.test"]
        == pytest.approx(0.3)
    )
    assert (
        arbitrator.apply(
            0.3,
            0.1,
            ControlFrame(direct={"control.test": -0.4}, authority={"control.test": "direct"}),
        ).values["control.test"]
        == pytest.approx(-0.4)
    )
####


def test_cadence_holds_and_then_accepts_a_new_command() -> None:
    arbitrator = ControlArbitrator((_schema(cadence_s=0.2),))
    first = arbitrator.apply(0.0, 0.1, ControlFrame(values={"control.test": 0.4}))
    held = arbitrator.apply(0.1, 0.1, ControlFrame(values={"control.test": 0.8}))
    accepted = arbitrator.apply(0.2, 0.1, ControlFrame(values={"control.test": 0.8}))

    assert first.values["control.test"] == pytest.approx(0.4)
    assert held.values["control.test"] == pytest.approx(0.4)
    assert held.decisions[0].stale
    assert held.decisions[0].held
    assert accepted.values["control.test"] == pytest.approx(0.8)
####


def test_bounds_overlay_limits_and_rate_limits_are_audited() -> None:
    arbitrator = ControlArbitrator(
        (
            _schema(
                overlay_minimum=-0.1,
                overlay_maximum=0.1,
                rate_limit_per_s=0.2,
            ),
        )
    )
    result = arbitrator.apply(
        0.0,
        0.25,
        ControlFrame(values={"control.test": 5.0}, autopilot={"control.test": 0.0}, authority={"control.test": "overlay"}),
    )

    assert result.values["control.test"] == pytest.approx(0.05)
    assert result.decisions[0].clamped
    assert result.decisions[0].overlay_limited
    assert result.decisions[0].rate_limited
    assert result.diagnostics == ("control-limited:control.test",)
####


def test_rate_command_mode_integrates_requested_rate_and_reports_achieved_rate() -> None:
    arbitrator = ControlArbitrator(
        (
            _schema(
                unit="deg",
                command_modes=("absolute", "rate"),
                rate_unit="deg/s",
                rate_limit_per_s=10.0,
            ),
        )
    )

    first = arbitrator.apply(0.0, 0.5, ControlFrame(rates={"control.test": 2.0}))
    second = arbitrator.apply(
        0.5,
        0.5,
        ControlFrame(rates={"control.test": -1.0}, input_modes={"control.test": "rate"}),
    )

    assert first.values["control.test"] == pytest.approx(1.0)
    assert first.decisions[0].command_mode == "rate"
    assert first.decisions[0].requested_rate == pytest.approx(2.0)
    assert first.decisions[0].realized_rate == pytest.approx(2.0)
    assert second.values["control.test"] == pytest.approx(0.5)
    assert second.decisions[0].realized_rate == pytest.approx(-1.0)
####


def test_rate_command_mode_respects_hard_actuator_rate_limit() -> None:
    arbitrator = ControlArbitrator(
        (
            _schema(
                command_modes=("absolute", "rate"),
                rate_unit="unit/s",
                rate_limit_per_s=1.0,
            ),
        )
    )

    result = arbitrator.apply(0.0, 0.5, ControlFrame(rates={"control.test": 3.0}))

    assert result.values["control.test"] == pytest.approx(0.5)
    assert result.decisions[0].requested_rate == pytest.approx(3.0)
    assert result.decisions[0].realized_rate == pytest.approx(1.0)
    assert result.decisions[0].rate_limited
####


def test_invalid_frame_uses_declared_failsafe() -> None:
    arbitrator = ControlArbitrator((_schema(hold_behavior="failsafe", failsafe_value=0.25),))
    result = arbitrator.apply(0.0, 0.1, ControlFrame(values={"control.test": 0.9}, valid=False))

    assert result.values["control.test"] == pytest.approx(0.25)
    assert result.decisions[0].source == "failsafe"
####


def test_inactive_channel_uses_its_declared_fallback() -> None:
    arbitrator = ControlArbitrator((_schema(hold_behavior="default", default=0.1),))
    result = arbitrator.apply(0.0, 0.1, ControlFrame(values={"control.test": 0.9}, active={"control.test": False}))

    assert result.values["control.test"] == pytest.approx(0.1)
    assert not result.decisions[0].active
    assert result.decisions[0].source == "default"
####


def test_authority_rejects_unknown_channels_and_ambiguous_mixed_mode() -> None:
    arbitrator = ControlArbitrator((_schema(),))
    with pytest.raises(ControlAuthorityError, match="unknown-control"):
        arbitrator.apply(0.0, 0.1, ControlFrame(values={"control.unknown": 0.0}))
    with pytest.raises(ControlAuthorityError, match="mixed-authority-selection-required"):
        arbitrator.apply(0.0, 0.1, ControlFrame(authority={"control.test": "mixed"}))
####


def test_provider_sessions_return_applied_control_telemetry() -> None:
    frame = ControlFrame(values={"command.throttle": 1.0, "command.bank": 90.0})
    for provider in (ReferencePointMassProvider(), TaoryxPointMassAdapter()):
        compiled = provider.compile(CASE)
        session = provider.new_session(compiled)
        first = session.step(0.05, frame)
        assert first.applied_controls["command.throttle"] == pytest.approx(0.05)
        assert first.applied_controls["command.bank"] == pytest.approx(2.25)
        assert "control-limited:command.throttle" in first.diagnostics
        assert "control-limited:command.bank" in first.diagnostics
####
