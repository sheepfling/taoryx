"""Explicit controller-backend selection for resolved Taoryx cases.

The registry is deliberately small.  It selects a named backend from a
resolved controller design; it does not contain vehicle or mission branches.
Backends that are retained only for regression are visible in the registry but
do not expose a qualification-eligible factory.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal

ControllerBackendStatus = Literal["implemented", "declared", "legacy_baseline"]
ControllerBackendFactory = Callable[..., object]


@dataclass(frozen=True, slots=True)
class ControllerBackendSpec:
    """One selectable controller implementation and its claim boundary."""

    id: str
    implementation: str
    status: ControllerBackendStatus
    qualification_eligible: bool
    factory: ControllerBackendFactory | None = None
    note: str = ""


class ControllerBackendRegistry:
    """Immutable-by-convention registry used at case resolution time."""

    def __init__(self, specs: Mapping[str, ControllerBackendSpec] | Sequence[ControllerBackendSpec] = ()) -> None:
        self._specs: dict[str, ControllerBackendSpec] = {}
        for spec in specs.values() if isinstance(specs, Mapping) else specs:
            self.register(spec)
        ####
    ####

    @property
    def ids(self) -> tuple[str, ...]:
        """Return registered backend IDs in deterministic order."""

        return tuple(sorted(self._specs))
        ####
    ####

    def register(self, spec: ControllerBackendSpec, *, replace: bool = False) -> None:
        """Register one backend, rejecting accidental duplicate selection IDs."""

        if not spec.id.strip():
            raise ValueError("controller backend ID must not be empty")
        if spec.id in self._specs and not replace:
            raise ValueError(f"controller backend is already registered: {spec.id!r}")
        if spec.status == "implemented" and spec.factory is None:
            raise ValueError(f"implemented controller backend {spec.id!r} requires a factory")
        if spec.qualification_eligible and spec.status == "legacy_baseline":
            raise ValueError(f"legacy controller backend {spec.id!r} cannot be qualification eligible")
        self._specs[spec.id] = spec
        ####
    ####

    def get(self, identifier: str) -> ControllerBackendSpec:
        """Resolve a backend declaration without invoking it."""

        try:
            return self._specs[identifier]
        except KeyError as error:
            raise KeyError(f"unknown controller backend {identifier!r}; available: {', '.join(self.ids)}") from error
        ####
    ####

    def require_factory(self, identifier: str) -> ControllerBackendFactory:
        """Return an executable factory or fail with a stable diagnostic."""

        spec = self.get(identifier)
        if spec.factory is None:
            raise ValueError(f"controller backend {identifier!r} is declared but has no executable factory: {spec.note}")
        return spec.factory
        ####
    ####

    def build(self, identifier: str, *args: Any, **kwargs: Any) -> object:
        """Build a controller through the selected backend factory."""

        return self.require_factory(identifier)(*args, **kwargs)
        ####
    ####


def default_controller_backend_registry() -> ControllerBackendRegistry:
    """Return the repository's explicit baseline/candidate backend registry."""

    from .controller_design import build_lqi_controller, build_lqr_controller

    return ControllerBackendRegistry(
        {
            "lqr": ControllerBackendSpec(
                id="lqr",
                implementation="lqr",
                status="implemented",
                qualification_eligible=True,
                factory=build_lqr_controller,
                note="Fixed-point LQR built from a named trim and linearization.",
            ),
            "gain_scheduled_lqr": ControllerBackendSpec(
                id="gain_scheduled_lqr",
                implementation="gain_scheduled_lqr",
                status="declared",
                qualification_eligible=True,
                note="Use the runtime GainScheduledLqrController adapter after a schedule builder is supplied.",
            ),
            "rslqr": ControllerBackendSpec(
                id="rslqr",
                implementation="rslqr",
                status="declared",
                qualification_eligible=True,
                note="Declared architecture; integral/robust scheduling implementation remains a promotion gate.",
            ),
            "lqi": ControllerBackendSpec(
                id="lqi",
                implementation="lqi",
                status="implemented",
                qualification_eligible=True,
                factory=build_lqi_controller,
                note="Explicit output-integrating LQI; a physical scheduled envelope remains a promotion gate.",
            ),
            "legacy_pid_baseline": ControllerBackendSpec(
                id="legacy_pid_baseline",
                implementation="pid",
                status="legacy_baseline",
                qualification_eligible=False,
                note="Regression-only baseline; cannot receive controller_qualified.",
            ),
        }
    )
    ####


__all__ = [
    "ControllerBackendFactory",
    "ControllerBackendRegistry",
    "ControllerBackendSpec",
    "ControllerBackendStatus",
    "default_controller_backend_registry",
]
####
