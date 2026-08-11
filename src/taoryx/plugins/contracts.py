"""Versioned contracts for installable Taoryx plug-ins.

The plug-in API is deliberately smaller than any one vehicle implementation.
It carries stable identity, compatibility, and typed contributions into the
host without making discovery construct a plant or execute a trajectory.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field

PLUGIN_API_VERSION = "1"
PLUGIN_ENTRY_POINT_GROUP = "taoryx.plugins"

ContributionKind = Literal[
    "batch_episode_parity_verifier",
    "controller_tuning_campaign",
    "episode_factory",
    "execution_factory",
    "family_adapter",
    "local_controller_screen_advertisement",
    "mission_capability_adapter",
    "mission_composition_provider",
    "model",
    "model_format",
    "reachability_provider",
    "semantic_preflight_handler",
    "trajectory_provider",
]
CONTRIBUTION_KINDS: tuple[ContributionKind, ...] = (
    "batch_episode_parity_verifier",
    "controller_tuning_campaign",
    "episode_factory",
    "execution_factory",
    "family_adapter",
    "local_controller_screen_advertisement",
    "mission_capability_adapter",
    "mission_composition_provider",
    "model",
    "model_format",
    "reachability_provider",
    "semantic_preflight_handler",
    "trajectory_provider",
)


class PluginError(RuntimeError):
    """Base class for public plug-in loading and registration failures."""


class PluginCompatibilityError(PluginError):
    """Raised when a plug-in targets another host API version."""


class PluginCollisionError(PluginError):
    """Raised when two contributions claim the same typed identity."""


class PluginLoadError(PluginError):
    """Raised when an entry point cannot produce a valid plug-in."""


class PluginMetadata(BaseModel):
    """Immutable identity and compatibility declaration for one plug-in."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    package: str = Field(min_length=1)
    version: str = Field(min_length=1)
    api_version: str = Field(min_length=1)
    description: str = Field(min_length=1)

    def public_dict(self) -> dict[str, str]:
        """Return a deterministic public metadata projection."""

        return self.model_dump(mode="json")
        ####

    ####


class PluginDiagnostic(BaseModel):
    """Structured discovery diagnostic retained by non-strict scans."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: Literal["loaded", "disabled", "incompatible", "failed"]
    plugin_id: str
    entry_point: str | None = None
    message: str

    def public_dict(self) -> dict[str, str | None]:
        """Return a JSON-safe diagnostic."""

        return self.model_dump(mode="json")
        ####

    ####


@dataclass(frozen=True, slots=True)
class PluginContribution:
    """One value contributed to a typed host registry."""

    kind: ContributionKind
    id: str
    plugin: PluginMetadata
    value: object

    def public_dict(self) -> dict[str, str]:
        """Describe ownership without serializing executable Python values."""

        return {
            "kind": self.kind,
            "id": self.id,
            "plugin_id": self.plugin.id,
            "package": self.plugin.package,
            "package_version": self.plugin.version,
        }
        ####

    ####


class TaoryxPlugin(Protocol):
    """Installable plug-in surface loaded through Python entry points."""

    @property
    def metadata(self) -> PluginMetadata:
        """Return identity without registering or constructing models."""

        ...

    def register(self, registrar: PluginRegistrar) -> None:
        """Contribute factories and providers to the supplied staging registrar."""

        ...


PluginRegisterCallback = Callable[["PluginRegistrar"], None]


@dataclass(frozen=True, slots=True)
class PluginDefinition:
    """Small concrete plug-in implementation for package entry points."""

    metadata: PluginMetadata
    register_callback: PluginRegisterCallback

    def register(self, registrar: PluginRegistrar) -> None:
        """Invoke the package-owned registration callback."""

        self.register_callback(registrar)
        ####

    ####


class PluginRegistrar:
    """Per-plug-in staging registrar with typed convenience methods.

    A discovery scan creates one registrar per plug-in and merges it only
    after registration succeeds. This prevents a failed plug-in from leaving
    a partially mutated global registry.
    """

    def __init__(self, plugin: PluginMetadata) -> None:
        self._plugin = plugin
        self._contributions: dict[tuple[ContributionKind, str], PluginContribution] = {}
        self._frozen = False
        ####

    @property
    def plugin(self) -> PluginMetadata:
        """Return the owner attached to every staged contribution."""

        return self._plugin
        ####

    def register(self, kind: ContributionKind, identifier: str, value: object) -> None:
        """Stage one explicitly identified contribution."""

        if self._frozen:
            raise PluginError(f"plug-in registrar for {self._plugin.id!r} is frozen")
        if kind not in CONTRIBUTION_KINDS:
            raise ValueError(f"unknown Taoryx contribution kind {kind!r}")
        normalized = identifier.strip()
        if not normalized or any(character.isspace() for character in normalized):
            raise ValueError("plug-in contribution IDs must be non-empty and contain no whitespace")
        key = (kind, normalized)
        if key in self._contributions:
            raise PluginCollisionError(
                f"plug-in {self._plugin.id!r} registered duplicate {kind} contribution {normalized!r}"
            )
        self._contributions[key] = PluginContribution(kind, normalized, self._plugin, value)
        ####

    def register_family_adapter(self, registration: object) -> None:
        """Stage a :class:`FamilyAdapterRegistration` by family identity."""

        self.register("family_adapter", _string_attribute(registration, "family_id"), registration)
        ####

    def register_trajectory_provider(self, provider: object) -> None:
        """Stage a provider-neutral trajectory provider by capability ID."""

        capabilities = getattr(provider, "capabilities", None)
        self.register("trajectory_provider", _string_attribute(capabilities, "provider_id"), provider)
        ####

    def register_mission_composition_provider(self, provider: object) -> None:
        """Stage a configurable Mission Composition provider by metadata ID."""

        metadata = getattr(provider, "metadata", None)
        self.register("mission_composition_provider", _string_attribute(metadata, "id"), provider)
        ####

    def register_mission_capability_adapter(self, adapter: object) -> None:
        """Stage a family-owned capability planner by its declared identity."""

        self.register("mission_capability_adapter", _string_attribute(adapter, "id"), adapter)
        ####

    def register_model(self, identifier: str, model: object) -> None:
        """Stage one model/catalog contribution under an explicit stable ID."""

        self.register("model", identifier, model)
        ####

    def register_model_format(self, identifier: str, handler: object) -> None:
        """Stage one optional model-format handler under a stable format ID."""

        self.register("model_format", identifier, handler)
        ####

    def register_reachability_provider(self, provider: object) -> None:
        """Stage an optional reachability workbench by provider metadata ID."""

        metadata = getattr(provider, "metadata", None)
        self.register("reachability_provider", _string_attribute(metadata, "id"), provider)
        ####

    def register_semantic_preflight_handler(self, handler: object) -> None:
        """Stage a semantic translator preflight by its declared identity."""

        self.register(
            "semantic_preflight_handler",
            _string_attribute(handler, "translator_id"),
            handler,
        )
        ####

    def register_execution_factory(self, identifier: str, factory: object) -> None:
        """Stage one callable execution factory under its advertised ID.

        New plug-ins should mark a one-argument typed factory with
        :func:`taoryx.vehicle_batch_execution.batch_factory_request_v1`.
        Existing ``(composition, output_dir, max_steps)`` callables remain a
        temporary compatibility surface.
        """

        if not callable(factory):
            raise TypeError("execution factory contributions must be callable")
        self.register("execution_factory", identifier, factory)
        ####

    def register_episode_factory(self, identifier: str, factory: object) -> None:
        """Stage one callable interactive-episode constructor."""

        if not callable(factory):
            raise TypeError("episode factory contributions must be callable")
        self.register("episode_factory", identifier, factory)
        ####

    def register_batch_episode_parity_verifier(self, identifier: str, verifier: object) -> None:
        """Stage one exact batch/episode parity verifier by adapter ID."""

        if not callable(verifier):
            raise TypeError("batch/episode parity verifier contributions must be callable")
        self.register("batch_episode_parity_verifier", identifier, verifier)
        ####

    def register_controller_tuning_campaign(self, registration: object) -> None:
        """Stage one model-owned campaign for the common tuning runner."""

        self.register(
            "controller_tuning_campaign",
            _string_attribute(registration, "id"),
            registration,
        )
        ####

    def register_local_controller_screen_advertisement(self, registration: object) -> None:
        """Stage static metadata for one exact plug-in-owned local screen."""

        self.register(
            "local_controller_screen_advertisement",
            _string_attribute(registration, "id"),
            registration,
        )
        ####

    def freeze(self) -> tuple[PluginContribution, ...]:
        """Close registration and retain package-declared contribution order."""

        self._frozen = True
        return tuple(self._contributions.values())
        ####

    ####


def _string_attribute(value: Any, name: str) -> str:
    """Read one duck-typed registration identity with a useful error."""

    identifier = getattr(value, name, None)
    if not isinstance(identifier, str) or not identifier.strip():
        raise TypeError(f"plug-in contribution requires string attribute {name!r}")
    return identifier
    ####


__all__ = [
    "CONTRIBUTION_KINDS",
    "ContributionKind",
    "PLUGIN_API_VERSION",
    "PLUGIN_ENTRY_POINT_GROUP",
    "PluginCollisionError",
    "PluginCompatibilityError",
    "PluginContribution",
    "PluginDefinition",
    "PluginDiagnostic",
    "PluginError",
    "PluginLoadError",
    "PluginMetadata",
    "PluginRegistrar",
    "TaoryxPlugin",
]
####
