"""Provider-neutral lifecycle contracts and a tiny analytical reference provider."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from .authority import ControlArbitrator
from .contracts import ControlFrame, FidelityProfile, ResolvedCase
from .evaluation import TrajectoryEvaluation


@dataclass(frozen=True, slots=True)
class ProviderCapabilities:
    """Advertised provider capability set."""

    provider_id: str
    provider_kind: str
    families: tuple[str, ...]
    fidelities: tuple[FidelityProfile, ...]
    supports_batch: bool = True
    supports_interactive: bool = True
    supports_checkpoint: bool = False
    statuses: Mapping[str, str] = field(default_factory=dict)
####


@dataclass(frozen=True, slots=True)
class TranslationEntry:
    """One requested concept and its provider realization."""

    requested: str
    realized: str | None
    status: str
    detail: str = ""
####


@dataclass(frozen=True, slots=True)
class TranslationReport:
    """Complete, explicit provider translation result."""

    provider_id: str
    entries: tuple[TranslationEntry, ...]

    @property
    def errors(self) -> tuple[TranslationEntry, ...]:
        """Return required concepts that were not realized."""

        return tuple(entry for entry in self.entries if entry.status == "unsupported")
        ####

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-compatible translation report."""

        return {
            "provider_id": self.provider_id,
            "entries": [
                {
                    "requested": entry.requested,
                    "realized": entry.realized,
                    "status": entry.status,
                    "detail": entry.detail,
                }
                for entry in self.entries
            ],
            "errors": [entry.requested for entry in self.errors],
        }
        ####
####


@dataclass(frozen=True, slots=True)
class CompiledCase:
    """Provider-owned compilation envelope exposed without native internals."""

    provider_id: str
    case: ResolvedCase
    translation: TranslationReport
####


@dataclass(frozen=True, slots=True)
class SessionState:
    """Normalized state observation at one accepted time boundary."""

    time_s: float
    values: Mapping[str, float]
####


@dataclass(frozen=True, slots=True)
class StepResult:
    """Result of one provider transition."""

    time_start_s: float
    time_end_s: float
    state: SessionState
    applied_controls: Mapping[str, float]
    events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    control_decisions: tuple[Mapping[str, object], ...] = ()
    requested_controls: Mapping[str, float] = field(default_factory=dict)
    achieved_controls: Mapping[str, float] = field(default_factory=dict)
    resource_observations: Mapping[str, float] = field(default_factory=dict)
####


@dataclass(frozen=True, slots=True)
class TrajectoryResult:
    """Normalized batch result produced by any provider."""

    provider_id: str
    case_id: str
    status: str
    samples: tuple[SessionState, ...]
    applied_controls: tuple[Mapping[str, float], ...]
    events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    requested_controls: tuple[Mapping[str, float], ...] = ()
    resource_observations: tuple[Mapping[str, float], ...] = ()
    evaluation: TrajectoryEvaluation | None = None

    def to_dict(self) -> dict[str, object]:
        """Return a stable machine-readable result envelope."""

        return {
            "provider_id": self.provider_id,
            "case_id": self.case_id,
            "status": self.status,
            "samples": [{"time_s": sample.time_s, "values": dict(sample.values)} for sample in self.samples],
            "applied_controls": [dict(frame) for frame in self.applied_controls],
            "events": list(self.events),
            "diagnostics": list(self.diagnostics),
            "requested_controls": [dict(frame) for frame in self.requested_controls],
            "resource_observations": [dict(frame) for frame in self.resource_observations],
            "evaluation": self.evaluation.as_dict() if self.evaluation is not None else None,
        }
        ####

    def write_json(self, path: str | Path) -> None:
        """Write normalized result telemetry."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        ####
####


class ProviderSession(Protocol):
    """Lifecycle required by the host for an interactive-capable provider."""

    def reset(self) -> SessionState:
        """Reset to the resolved initial state."""
        ...

    def step(self, duration_s: float, controls: ControlFrame | None = None) -> StepResult:
        """Advance exactly one accepted transition."""
        ...

    def run_to_completion(self, controls: Sequence[ControlFrame] = ()) -> TrajectoryResult:
        """Run from reset to the case completion boundary."""
        ...
####


class TrajectoryProvider(Protocol):
    """Provider interface selected by the common host."""

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Return the provider's negotiated capabilities."""
        ...

    def compile(self, case: ResolvedCase) -> CompiledCase:
        """Compile a resolved case or fail closed."""
        ...

    def new_session(self, compiled: CompiledCase) -> ProviderSession:
        """Create a fresh session from one compiled case."""
        ...
####


class ProviderRegistry:
    """Explicit provider discovery and selection registry."""

    def __init__(self, providers: Sequence[TrajectoryProvider] = ()) -> None:
        self._providers: dict[str, TrajectoryProvider] = {}
        for provider in providers:
            self.register(provider)
        ####
    ####

    def register(self, provider: TrajectoryProvider) -> None:
        """Register one provider under its stable ID."""

        provider_id = provider.capabilities.provider_id
        if provider_id in self._providers:
            raise ValueError(f"duplicate trajectory provider {provider_id!r}")
        self._providers[provider_id] = provider
        ####

    def provider(self, provider_id: str) -> TrajectoryProvider:
        """Select one provider or return an explicit discovery error."""

        try:
            return self._providers[provider_id]
        except KeyError as error:
            raise KeyError(f"unknown trajectory provider {provider_id!r}") from error
        ####

    def capabilities(self) -> tuple[ProviderCapabilities, ...]:
        """Return capabilities in deterministic provider-ID order."""

        return tuple(self._providers[key].capabilities for key in sorted(self._providers))
        ####
####


def _case_number(case: ResolvedCase, parameter_id: str, fallback: float) -> float:
    """Read one numeric resolved parameter with an explicit fallback."""

    value = case.parameters.get(parameter_id)
    if value is None or not isinstance(value.value, (int, float)):
        return fallback
    return float(value.value)
    ####


class ReferencePointMassProvider:
    """Non-Taoryx constant-acceleration provider for host conformance tests."""

    def __init__(self) -> None:
        self._capabilities = ProviderCapabilities(
            provider_id="reference.point_mass",
            provider_kind="analytical",
            families=("simple_aero",),
            fidelities=("point_mass_3dof",),
            supports_batch=True,
            supports_interactive=True,
            statuses={"batch": "native", "interactive": "native", "moments": "unsupported"},
        )
        ####

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Return the reference provider capabilities."""

        return self._capabilities
        ####

    def compile(self, case: ResolvedCase) -> CompiledCase:
        """Compile only the simple point-mass contract."""

        if case.family not in self.capabilities.families:
            raise ValueError(f"reference provider does not support family {case.family!r}")
        if case.fidelity not in self.capabilities.fidelities:
            raise ValueError(f"reference provider does not support fidelity {case.fidelity!r}")
        entries = (
            TranslationEntry("family", case.family, "native"),
            TranslationEntry("fidelity", case.fidelity, "native"),
            TranslationEntry("vehicle.mass.initial", "constant-mass", "approximated", "mass is fixed for the reference fixture"),
            TranslationEntry("rigid_body_moments", None, "unsupported", "point-mass reference provider"),
        )
        report = TranslationReport(self.capabilities.provider_id, entries)
        return CompiledCase(self.capabilities.provider_id, case, report)
        ####

    def new_session(self, compiled: CompiledCase) -> ProviderSession:
        """Create a reference session after checking its translation report."""

        if compiled.translation.errors:
            # Unsupported optional concepts are retained in the report.  The
            # reference fixture has no requested rigid-body requirement, so it
            # remains executable while preserving the limitation.
            pass
        return _ReferenceSession(compiled)
        ####


class _ReferenceSession:
    """Constant-acceleration implementation kept private to the reference provider."""

    def __init__(self, compiled: CompiledCase) -> None:
        self.compiled = compiled
        self.case = compiled.case
        self._arbitrator = ControlArbitrator(self.case.controls)
        self._reset_values()
        ####

    def _reset_values(self) -> None:
        """Initialize mutable state without replacing the session object."""

        self._time = 0.0
        self._downrange = 0.0
        self._speed = _case_number(self.case, "mission.initial_speed", 0.0)
        self._altitude = _case_number(self.case, "mission.initial_altitude", 0.0)
        self._arbitrator.reset()
        self._history: list[SessionState] = [self._state()]
        self._controls: list[Mapping[str, float]] = []
        self._requested_controls: list[Mapping[str, float]] = []
        self._resources: list[Mapping[str, float]] = []
        self._diagnostics: list[str] = []
        ####

    def _state(self) -> SessionState:
        return SessionState(
            self._time,
            {
                "position.downrange_m": self._downrange,
                "position.altitude_m": self._altitude,
                "velocity.m_s": self._speed,
            },
        )
        ####

    def reset(self) -> SessionState:
        """Reset the reference state and command history."""

        self._reset_values()
        return self._state()
        ####

    def step(self, duration_s: float, controls: ControlFrame | None = None) -> StepResult:
        """Advance with exact constant-acceleration kinematics."""

        if duration_s <= 0.0:
            raise ValueError("provider step duration must be positive")
        frame = controls or ControlFrame()
        arbitration = self._arbitrator.apply(self._time, duration_s, frame)
        applied = arbitration.values
        throttle = float(applied.get("command.throttle", 0.0))
        mass = _case_number(self.case, "vehicle.mass.initial", 1.0)
        thrust = _case_number(self.case, "vehicle.booster.thrust", 0.0)
        acceleration = throttle * thrust / mass
        start = self._time
        self._downrange += self._speed * duration_s + 0.5 * acceleration * duration_s * duration_s
        self._speed += acceleration * duration_s
        self._time += duration_s
        state = self._state()
        self._history.append(state)
        self._controls.append(dict(applied))
        requested = {**frame.values, **frame.rates}
        self._requested_controls.append(requested)
        self._resources.append({})
        self._diagnostics.extend(arbitration.diagnostics)
        return StepResult(
            start,
            self._time,
            state,
            dict(applied),
            diagnostics=arbitration.diagnostics,
            control_decisions=tuple(decision.to_dict() for decision in arbitration.decisions),
            requested_controls=requested,
            achieved_controls=dict(applied),
        )
        ####

    def run_to_completion(self, controls: Sequence[ControlFrame] = ()) -> TrajectoryResult:
        """Run for the configured duration using the supplied frames."""

        self.reset()
        duration = _case_number(self.case, "mission.duration", 1.0)
        dt = _case_number(self.case, "runtime.time_step", duration)
        frames = tuple(controls)
        index = 0
        while self._time < duration - 1e-12:
            step = min(dt, duration - self._time)
            frame = frames[index] if index < len(frames) else ControlFrame()
            self.step(step, frame)
            index += 1
        return TrajectoryResult(
            self.compiled.provider_id,
            self.case.case_id,
            "completed",
            tuple(self._history),
            tuple(self._controls),
            diagnostics=tuple(self._diagnostics),
            requested_controls=tuple(self._requested_controls),
            resource_observations=tuple(self._resources),
        )
        ####


__all__ = [
    "CompiledCase",
    "ControlFrame",
    "ProviderCapabilities",
    "ProviderRegistry",
    "ProviderSession",
    "ReferencePointMassProvider",
    "SessionState",
    "StepResult",
    "TrajectoryProvider",
    "TrajectoryResult",
    "TranslationEntry",
    "TranslationReport",
]
####
