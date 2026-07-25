"""Deterministic, provider-neutral control authority arbitration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .contracts import AuthorityMode, ControlFrame, ControlInputMode, ControlSchema

DecisionSource = Literal[
    "autopilot",
    "commanded",
    "overlay",
    "direct",
    "hold",
    "default",
    "failsafe",
    "overlay-default",
]
####


class ControlAuthorityError(ValueError):
    """Fail-closed control arbitration error with a stable diagnostic code."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        prefix = f"{code}: "
        if field is not None:
            prefix = f"{prefix}{field}: "
        super().__init__(prefix + message)
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlDecision:
    """Auditable decision for one channel at one accepted boundary."""

    control_id: str
    authority: AuthorityMode
    source: DecisionSource
    candidate: float
    selected: float
    applied: float
    held: bool = False
    clamped: bool = False
    overlay_limited: bool = False
    rate_limited: bool = False
    stale: bool = False
    active: bool = True
    overlay: float = 0.0
    command_mode: ControlInputMode = "absolute"
    requested_rate: float | None = None
    realized_rate: float | None = None

    def to_dict(self) -> dict[str, object]:
        """Return stable JSON-compatible telemetry."""

        return {
            "control_id": self.control_id,
            "authority": self.authority,
            "source": self.source,
            "candidate": self.candidate,
            "selected": self.selected,
            "applied": self.applied,
            "held": self.held,
            "clamped": self.clamped,
            "overlay_limited": self.overlay_limited,
            "rate_limited": self.rate_limited,
            "stale": self.stale,
            "active": self.active,
            "overlay": self.overlay,
            "command_mode": self.command_mode,
            "requested_rate": self.requested_rate,
            "realized_rate": self.realized_rate,
        }
        ####
####


@dataclass(frozen=True, slots=True)
class ArbitrationResult:
    """Result of applying the common authority policy to one frame."""

    values: dict[str, float]
    decisions: tuple[ControlDecision, ...]
    diagnostics: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        """Return stable JSON-compatible arbitration telemetry."""

        return {
            "values": dict(self.values),
            "decisions": [decision.to_dict() for decision in self.decisions],
            "diagnostics": list(self.diagnostics),
        }
        ####
####


@dataclass(slots=True)
class _ControlMemory:
    applied: float | None = None
    selected: float | None = None
    accepted_time_s: float | None = None
####


class ControlArbitrator:
    """Apply authority, cadence, fallback, bounds, and rate rules.

    The arbiter owns no physics and imports no provider/runtime module.  It is
    intentionally reusable by reference providers, native adapters, and
    future pseudo-6DOF or rigid-body bindings.
    """

    def __init__(self, controls: tuple[ControlSchema, ...]) -> None:
        self._controls = {control.id: control for control in controls}
        if len(self._controls) != len(controls):
            raise ControlAuthorityError("duplicate-control", "control IDs must be unique")
        self._memory: dict[str, _ControlMemory] = {control.id: _ControlMemory(applied=control.default) for control in controls}
        self._last_time_s: float | None = None
        self._validate_schemas()
        ####

    def _validate_schemas(self) -> None:
        """Reject internally inconsistent authority declarations."""

        for control in self._controls.values():
            if control.default_authority not in control.authority_modes:
                raise ControlAuthorityError(
                    "invalid-authority-default",
                    "default authority is not in authority_modes",
                    field=control.id,
                )
            if not control.command_modes or len(set(control.command_modes)) != len(control.command_modes):
                raise ControlAuthorityError(
                    "invalid-command-modes",
                    "command_modes must be nonempty and unique",
                    field=control.id,
                )
            if control.default_command_mode not in control.command_modes:
                raise ControlAuthorityError(
                    "invalid-command-mode-default",
                    "default command mode is not in command_modes",
                    field=control.id,
                )
            if "rate" in control.command_modes and not control.rate_unit:
                raise ControlAuthorityError(
                    "missing-rate-unit",
                    "rate_unit is required when rate mode is supported",
                    field=control.id,
                )
            if control.minimum is not None and control.maximum is not None and control.minimum > control.maximum:
                raise ControlAuthorityError("invalid-control-bounds", "minimum exceeds maximum", field=control.id)
            if control.failsafe_value is not None and control.minimum is not None and control.failsafe_value < control.minimum:
                raise ControlAuthorityError("invalid-failsafe", "failsafe is below minimum", field=control.id)
            if control.failsafe_value is not None and control.maximum is not None and control.failsafe_value > control.maximum:
                raise ControlAuthorityError("invalid-failsafe", "failsafe exceeds maximum", field=control.id)
            if control.overlay_minimum is not None and control.overlay_maximum is not None and control.overlay_minimum > control.overlay_maximum:
                raise ControlAuthorityError("invalid-overlay-bounds", "overlay minimum exceeds maximum", field=control.id)
        ####

    def reset(self) -> None:
        """Clear held values and cadence history for a new session."""

        for control_id, memory in self._memory.items():
            memory.applied = self._controls[control_id].default
            memory.selected = None
            memory.accepted_time_s = None
        self._last_time_s = None
        ####

    def _check_frame(self, frame: ControlFrame) -> None:
        """Reject unknown channels before any control is applied."""

        known = set(self._controls)
        for label, values in (
            ("values", frame.values),
            ("rates", frame.rates),
            ("autopilot", frame.autopilot),
            ("autopilot_rates", frame.autopilot_rates),
            ("direct", frame.direct),
            ("direct_rates", frame.direct_rates),
            ("authority", frame.authority),
            ("input_modes", frame.input_modes),
            ("active", frame.active),
        ):
            unknown = sorted(set(values) - known)
            if unknown:
                raise ControlAuthorityError("unknown-control", f"unknown control channel(s): {', '.join(unknown)}", field=label)
        ####

    def _input_mode(self, control: ControlSchema, frame: ControlFrame) -> ControlInputMode:
        """Resolve absolute versus rate input without accepting ambiguity."""

        if control.id in frame.values and control.id in frame.rates:
            raise ControlAuthorityError(
                "ambiguous-command-input",
                "absolute and rate values were supplied for the same channel",
                field=control.id,
            )
        selected = frame.input_modes.get(control.id, control.default_command_mode)
        rate_channels = frame.rates.keys() | frame.autopilot_rates.keys() | frame.direct_rates.keys()
        if control.id in rate_channels and control.id not in frame.input_modes:
            selected = "rate"
        if selected not in control.command_modes:
            raise ControlAuthorityError(
                "unsupported-command-mode",
                f"command mode {selected!r} is not declared for this channel",
                field=control.id,
            )
        if selected == "rate" and control.id in frame.values:
            raise ControlAuthorityError(
                "command-mode-value-mismatch",
                "rate mode requires a rate value, not an absolute value",
                field=control.id,
            )
        if selected == "absolute" and control.id in frame.rates:
            raise ControlAuthorityError(
                "command-mode-value-mismatch",
                "absolute mode requires an absolute value, not a rate value",
                field=control.id,
            )
        return selected
        ####

    @staticmethod
    def _finite(value: float, control_id: str) -> float:
        """Require finite command values."""

        numeric = float(value)
        if not math.isfinite(numeric):
            raise ControlAuthorityError("invalid-control-value", "control value must be finite", field=control_id)
        return numeric
        ####

    def _fallback(self, control: ControlSchema, memory: _ControlMemory) -> tuple[float, DecisionSource]:
        """Select the declared missing-input fallback."""

        if control.hold_behavior == "hold" and memory.applied is not None:
            return memory.applied, "hold"
        if control.hold_behavior == "failsafe":
            return control.failsafe_value if control.failsafe_value is not None else control.default, "failsafe"
        return control.default, "default"
        ####

    def _mode(self, control: ControlSchema, frame: ControlFrame) -> AuthorityMode:
        """Resolve per-frame authority without allowing unsupported modes."""

        selected = frame.authority.get(control.id, control.default_authority)
        if selected == "mixed":
            raise ControlAuthorityError(
                "mixed-authority-selection-required",
                "mixed authority requires an explicit non-mixed per-frame selection",
                field=control.id,
            )
        if selected not in control.authority_modes:
            raise ControlAuthorityError(
                "unsupported-authority",
                f"authority {selected!r} is not declared for this channel",
                field=control.id,
            )
        return selected
        ####

    def _candidate(
        self,
        control: ControlSchema,
        frame: ControlFrame,
        memory: _ControlMemory,
        mode: AuthorityMode,
        command_mode: ControlInputMode,
    ) -> tuple[float | None, DecisionSource, float, bool, bool]:
        """Return source candidate, source label, overlay, and limit flags."""

        if mode == "autopilot":
            source_values = frame.autopilot_rates if command_mode == "rate" else frame.autopilot
            if control.id in source_values:
                return self._finite(source_values[control.id], control.id), "autopilot", 0.0, False, False
        elif mode == "commanded":
            source_values = frame.rates if command_mode == "rate" else frame.values
            if control.id in source_values:
                return self._finite(source_values[control.id], control.id), "commanded", 0.0, False, False
        elif mode == "direct":
            source_values = frame.direct_rates if command_mode == "rate" else frame.direct
            if control.id in source_values:
                return self._finite(source_values[control.id], control.id), "direct", 0.0, False, False
        elif mode == "overlay":
            base_values = frame.autopilot_rates if command_mode == "rate" else frame.autopilot
            bias_values = frame.rates if command_mode == "rate" else frame.values
            base = base_values.get(control.id)
            bias = bias_values.get(control.id, 0.0)
            if base is None and memory.selected is None and control.id not in bias_values:
                return None, "overlay-default", 0.0, True, False
            base_value = self._finite(base if base is not None else memory.selected if memory.selected is not None else control.default, control.id)
            raw_overlay = self._finite(bias, control.id)
            overlay = raw_overlay
            if control.overlay_minimum is not None:
                overlay = max(control.overlay_minimum, overlay)
            if control.overlay_maximum is not None:
                overlay = min(control.overlay_maximum, overlay)
            return base_value + overlay, "overlay", overlay, False, overlay != raw_overlay
        return None, "failsafe", 0.0, True, False
        ####

    def apply(self, time_s: float, duration_s: float, frame: ControlFrame | None = None) -> ArbitrationResult:
        """Apply one boundary frame and return values plus audit decisions."""

        if not math.isfinite(time_s) or not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ControlAuthorityError("invalid-transition", "time and positive duration must be finite")
        if self._last_time_s is not None and time_s < self._last_time_s - 1e-12:
            raise ControlAuthorityError("time-reversal", "control transition time moved backwards")
        current = frame or ControlFrame()
        self._check_frame(current)
        decisions: list[ControlDecision] = []
        values: dict[str, float] = {}
        for control in self._controls.values():
            memory = self._memory[control.id]
            mode = self._mode(control, current)
            command_mode = self._input_mode(control, current)
            inactive = control.id in current.active and not current.active[control.id]
            candidate, source, overlay, missing, overlay_limited = self._candidate(control, current, memory, mode, command_mode)
            stale = False
            held = False
            effective_mode = command_mode
            if control.cadence_s > 0.0 and memory.accepted_time_s is not None:
                stale = time_s - memory.accepted_time_s < control.cadence_s - 1e-12
            if not current.valid:
                candidate = None
                missing = True
                source = "failsafe"
            if inactive:
                candidate = None
                missing = True
                source = "failsafe"
            if stale or missing:
                fallback, fallback_source = self._fallback(control, memory)
                if stale and current.valid and not inactive and memory.selected is not None:
                    candidate = memory.applied if command_mode == "rate" and memory.applied is not None else memory.selected
                    source = "hold"
                    held = True
                    effective_mode = "absolute"
                else:
                    candidate = fallback
                    source = fallback_source
                    effective_mode = "absolute"
            assert candidate is not None
            selected = self._finite(candidate, control.id)
            if not stale and not missing and current.valid:
                memory.selected = selected
                memory.accepted_time_s = time_s
            lower = control.minimum if control.minimum is not None else -math.inf
            upper = control.maximum if control.maximum is not None else math.inf
            previous = memory.applied if memory.applied is not None else control.default
            requested_rate = selected if effective_mode == "rate" else None
            rate_limited = False
            if effective_mode == "rate":
                bounded_rate = selected
                if control.rate_limit_per_s is not None:
                    bounded_rate = min(control.rate_limit_per_s, max(-control.rate_limit_per_s, bounded_rate))
                    rate_limited = bounded_rate != selected
                bounded = previous + bounded_rate * duration_s
            else:
                bounded = selected
            if effective_mode == "absolute" and control.rate_limit_per_s is not None and memory.applied is not None:
                max_delta = control.rate_limit_per_s * duration_s
                bounded = min(memory.applied + max_delta, max(memory.applied - max_delta, bounded))
                rate_limited = bounded != min(upper, max(lower, selected))
            bounded_before_absolute_limits = bounded
            bounded = min(upper, max(lower, bounded))
            clamped = overlay_limited or bounded != bounded_before_absolute_limits
            if effective_mode == "absolute":
                clamped = clamped or bounded != selected
            applied = float(bounded)
            realized_rate = (applied - previous) / duration_s
            memory.applied = applied
            values[control.id] = applied
            decisions.append(
                ControlDecision(
                    control.id,
                    mode,
                    source,
                    selected,
                    selected,
                    applied,
                    held=held,
                    clamped=clamped,
                    overlay_limited=overlay_limited,
                    rate_limited=rate_limited,
                    stale=stale,
                    active=not inactive,
                    overlay=overlay,
                    command_mode=effective_mode,
                    requested_rate=requested_rate,
                    realized_rate=realized_rate,
                )
            )
        self._last_time_s = time_s
        diagnostics = tuple(
            f"control-limited:{decision.control_id}"
            for decision in decisions
            if decision.clamped or decision.rate_limited
        )
        return ArbitrationResult(values, tuple(decisions), diagnostics)
        ####


__all__ = ["ArbitrationResult", "ControlArbitrator", "ControlAuthorityError", "ControlDecision"]
####
