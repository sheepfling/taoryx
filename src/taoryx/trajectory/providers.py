"""Provider-neutral lifecycle contracts and explicit host-side registries."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from importlib import import_module
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from .contracts import ControlFrame, FidelityProfile, ResolvedCase
from .evaluation import TrajectoryEvaluation

if TYPE_CHECKING:
    from taoryx_simple_aero.provider import ReferencePointMassProvider


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


def __getattr__(name: str) -> object:
    """Preserve the former provider import when its plug-in is installed."""

    if name == "ReferencePointMassProvider":
        try:
            module = import_module("taoryx_simple_aero.provider")
        except ModuleNotFoundError as error:
            raise ModuleNotFoundError(
                "ReferencePointMassProvider moved to the optional 'taoryx-simple-aero' plug-in"
            ) from error
        return getattr(module, name)
    raise AttributeError(name)
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
