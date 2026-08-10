"""Deterministic discovery and aggregation for installed Taoryx plug-ins."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from importlib import metadata as importlib_metadata
from typing import TYPE_CHECKING, Literal, Protocol, cast

from .contracts import (
    PLUGIN_API_VERSION,
    PLUGIN_ENTRY_POINT_GROUP,
    ContributionKind,
    PluginCollisionError,
    PluginCompatibilityError,
    PluginContribution,
    PluginDiagnostic,
    PluginLoadError,
    PluginMetadata,
    PluginRegistrar,
    TaoryxPlugin,
)

if TYPE_CHECKING:
    from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
    from taoryx.family_adapter_registry import FamilyAdapterRegistry
    from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisementRegistry
    from taoryx.reachability_registry import ReachabilityProviderRegistry
    from taoryx.trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
    from taoryx.trajectory.providers import ProviderRegistry


class PluginEntryPoint(Protocol):
    """Minimal entry-point surface accepted by discovery and tests."""

    name: str
    value: str

    def load(self) -> object:
        """Load the referenced Python object."""

        ...


@dataclass(frozen=True, slots=True)
class PluginCatalog:
    """Immutable, provenance-bearing view of one plug-in discovery scan."""

    plugins: tuple[PluginMetadata, ...]
    contributions: tuple[PluginContribution, ...]
    diagnostics: tuple[PluginDiagnostic, ...]

    def __post_init__(self) -> None:
        plugin_ids = tuple(item.id for item in self.plugins)
        if len(plugin_ids) != len(set(plugin_ids)):
            raise ValueError("plug-in catalog contains duplicate plug-in IDs")
        keys = tuple((item.kind, item.id) for item in self.contributions)
        if len(keys) != len(set(keys)):
            raise ValueError("plug-in catalog contains duplicate typed contribution IDs")
        ####

    @property
    def fingerprint(self) -> str:
        """Hash installed identities and contribution ownership deterministically."""

        payload = {
            "api_version": PLUGIN_API_VERSION,
            "plugins": [item.public_dict() for item in self.plugins],
            "contributions": [item.public_dict() for item in self.contributions],
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    def plugin(self, plugin_id: str) -> PluginMetadata:
        """Return one loaded plug-in identity."""

        match = next((item for item in self.plugins if item.id == plugin_id), None)
        if match is None:
            raise KeyError(f"unknown Taoryx plug-in {plugin_id!r}")
        return match
        ####

    def records(self, kind: ContributionKind) -> tuple[PluginContribution, ...]:
        """Return one typed registry in stable package-declared order."""

        return tuple(item for item in self.contributions if item.kind == kind)
        ####

    def values(self, kind: ContributionKind) -> tuple[object, ...]:
        """Return executable values for one typed registry."""

        return tuple(item.value for item in self.records(kind))
        ####

    def contribution(self, kind: ContributionKind, identifier: str) -> PluginContribution:
        """Resolve one typed identity without cross-registry fallback."""

        match = next(
            (item for item in self.contributions if item.kind == kind and item.id == identifier),
            None,
        )
        if match is None:
            raise KeyError(f"unknown {kind} plug-in contribution {identifier!r}")
        return match
        ####

    def build_family_adapter_registry(self) -> FamilyAdapterRegistry:
        """Project family contributions into the established typed registry."""

        from taoryx.family_adapter_registry import FamilyAdapterRegistration, FamilyAdapterRegistry

        registrations = cast(
            tuple[FamilyAdapterRegistration, ...],
            self.values("family_adapter"),
        )
        return FamilyAdapterRegistry(registrations)
        ####

    def build_trajectory_provider_registry(self) -> ProviderRegistry:
        """Project provider contributions into the neutral provider registry."""

        from taoryx.trajectory.providers import ProviderRegistry, TrajectoryProvider

        providers = cast(tuple[TrajectoryProvider, ...], self.values("trajectory_provider"))
        return ProviderRegistry(providers)
        ####

    def build_mission_composition_provider_registry(self) -> ConfigurableTrajectoryProviderRegistry:
        """Project composer-provider contributions into the typed public registry."""

        from taoryx.trajectory.configuration_contract import (
            ConfigurableTrajectoryProvider,
            ConfigurableTrajectoryProviderRegistry,
        )

        providers = cast(
            tuple[ConfigurableTrajectoryProvider, ...],
            self.values("mission_composition_provider"),
        )
        return ConfigurableTrajectoryProviderRegistry(providers)
        ####

    def build_reachability_provider_registry(self) -> ReachabilityProviderRegistry:
        """Project optional workbench contributions into the reachability registry."""

        from taoryx.reachability_registry import ReachabilityProvider, ReachabilityProviderRegistry

        providers = cast(tuple[ReachabilityProvider, ...], self.values("reachability_provider"))
        return ReachabilityProviderRegistry(providers)
        ####

    def build_controller_tuning_campaign_registry(self) -> ControllerTuningCampaignRegistry:
        """Project model-owned campaigns into the common tuning registry."""

        from taoryx.controller_tuning_registry import (
            ControllerTuningCampaignRegistration,
            ControllerTuningCampaignRegistry,
        )

        registrations = cast(
            tuple[ControllerTuningCampaignRegistration, ...],
            self.values("controller_tuning_campaign"),
        )
        return ControllerTuningCampaignRegistry(registrations)
        ####

    def build_local_controller_screen_advertisement_registry(self) -> LocalControllerScreenAdvertisementRegistry:
        """Project static plug-in controller-screen metadata into one exact registry."""

        from taoryx.local_controller_screen_advertisements import (
            LocalControllerScreenAdvertisement,
            LocalControllerScreenAdvertisementRegistry,
        )

        registrations = cast(
            tuple[LocalControllerScreenAdvertisement, ...],
            self.values("local_controller_screen_advertisement"),
        )
        return LocalControllerScreenAdvertisementRegistry(registrations=registrations)
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the discovery result without serializing factories or providers."""

        return {
            "schema": "taoryx.plugin-catalog/v1",
            "api_version": PLUGIN_API_VERSION,
            "fingerprint": self.fingerprint,
            "plugins": [item.public_dict() for item in self.plugins],
            "contributions": [item.public_dict() for item in self.contributions],
            "diagnostics": [item.public_dict() for item in self.diagnostics],
        }
        ####

    ####


class _CatalogBuilder:
    """Mutable atomic merge state kept private to one discovery scan."""

    def __init__(self) -> None:
        self.plugins: dict[str, PluginMetadata] = {}
        self.contributions: dict[tuple[ContributionKind, str], PluginContribution] = {}
        ####

    def add(self, plugin: PluginMetadata, contributions: Sequence[PluginContribution]) -> None:
        """Merge one fully staged plug-in or reject the entire plug-in."""

        if plugin.id in self.plugins:
            existing_plugin = self.plugins[plugin.id]
            raise PluginCollisionError(f"plug-in ID {plugin.id!r} is provided by both {existing_plugin.package!r} and {plugin.package!r}")
        for contribution in contributions:
            key = (contribution.kind, contribution.id)
            if key in self.contributions:
                existing_contribution = self.contributions[key]
                raise PluginCollisionError(
                    f"{contribution.kind} contribution {contribution.id!r} is provided by both {existing_contribution.plugin.id!r} and {plugin.id!r}"
                )
        self.plugins[plugin.id] = plugin
        for contribution in contributions:
            self.contributions[(contribution.kind, contribution.id)] = contribution
        ####

    def build(self, diagnostics: Sequence[PluginDiagnostic]) -> PluginCatalog:
        """Freeze the scan in deterministic identity order."""

        return PluginCatalog(
            tuple(self.plugins[key] for key in sorted(self.plugins)),
            tuple(self.contributions.values()),
            tuple(diagnostics),
        )
        ####

    ####


def discover_plugins(
    *,
    include_builtin: bool = True,
    include_external: bool = True,
    disabled: Iterable[str] = (),
    entry_points: Sequence[PluginEntryPoint] | None = None,
    strict: bool = True,
) -> PluginCatalog:
    """Load compatible plug-ins and atomically merge their contributions.

    Supplying ``entry_points`` replaces installed-package discovery and is the
    deterministic seam used by tests and embedded hosts. In non-strict mode a
    failing or colliding plug-in is omitted and represented by a diagnostic.
    """

    disabled_ids = frozenset(item.strip() for item in disabled if item.strip())
    builder = _CatalogBuilder()
    diagnostics: list[PluginDiagnostic] = []
    selected_entry_points = (tuple(entry_points) if entry_points is not None else _installed_entry_points()) if include_external else ()
    external_ids = frozenset(item.name for item in selected_entry_points)

    if include_builtin:
        for plugin in _builtin_plugins(excluded_ids=external_ids):
            _load_plugin(
                plugin,
                entry_point=None,
                expected_id=plugin.metadata.id,
                disabled_ids=disabled_ids,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )

    if include_external:
        for entry_point in sorted(selected_entry_points, key=lambda item: (item.name, item.value)):
            if entry_point.name in disabled_ids:
                diagnostics.append(
                    PluginDiagnostic(
                        status="disabled",
                        plugin_id=entry_point.name,
                        entry_point=entry_point.value,
                        message="disabled by host configuration",
                    )
                )
                continue
            try:
                loaded = entry_point.load()
                plugin = _coerce_plugin(loaded, entry_point.value)
            except Exception as error:  # noqa: BLE001 - entry points are an untrusted package boundary
                _handle_failure(
                    PluginLoadError(f"could not load plug-in entry point {entry_point.value!r}: {error}"),
                    plugin_id=entry_point.name,
                    entry_point=entry_point.value,
                    builder=builder,
                    diagnostics=diagnostics,
                    strict=strict,
                )
                continue
            _load_plugin(
                plugin,
                entry_point=entry_point.value,
                expected_id=entry_point.name,
                disabled_ids=disabled_ids,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
    return builder.build(diagnostics)
    ####


def _load_plugin(
    plugin: TaoryxPlugin,
    *,
    entry_point: str | None,
    expected_id: str,
    disabled_ids: frozenset[str],
    builder: _CatalogBuilder,
    diagnostics: list[PluginDiagnostic],
    strict: bool,
) -> None:
    """Validate, stage, and atomically merge one already loaded plug-in."""

    plugin_id = expected_id
    try:
        metadata = plugin.metadata
        plugin_id = metadata.id
        if metadata.id != expected_id:
            raise PluginLoadError(f"entry-point name {expected_id!r} does not match plug-in metadata ID {metadata.id!r}")
        if metadata.id in disabled_ids:
            diagnostics.append(
                PluginDiagnostic(
                    status="disabled",
                    plugin_id=metadata.id,
                    entry_point=entry_point,
                    message="disabled by host configuration",
                )
            )
            return
        if metadata.api_version != PLUGIN_API_VERSION:
            raise PluginCompatibilityError(f"plug-in {metadata.id!r} targets API {metadata.api_version!r}; host API is {PLUGIN_API_VERSION!r}")
        registrar = PluginRegistrar(metadata)
        plugin.register(registrar)
        contributions = registrar.freeze()
        builder.add(metadata, contributions)
    except Exception as error:  # noqa: BLE001 - plug-in callbacks are an installed-code boundary
        _handle_failure(
            error,
            plugin_id=plugin_id,
            entry_point=entry_point,
            builder=builder,
            diagnostics=diagnostics,
            strict=strict,
        )
        return
    diagnostics.append(
        PluginDiagnostic(
            status="loaded",
            plugin_id=metadata.id,
            entry_point=entry_point,
            message=f"registered {len(contributions)} contribution(s)",
        )
    )
    ####


def _handle_failure(
    error: Exception,
    *,
    plugin_id: str,
    entry_point: str | None,
    builder: _CatalogBuilder,
    diagnostics: list[PluginDiagnostic],
    strict: bool,
) -> None:
    """Raise strict failures or retain a non-strict diagnostic."""

    del builder  # Documents that failed plug-ins never mutate the merge state.
    if strict:
        if isinstance(error, PluginErrorTypes):
            raise error
        raise PluginLoadError(f"plug-in {plugin_id!r} registration failed: {error}") from error
    status: Literal["incompatible", "failed"] = "incompatible" if isinstance(error, PluginCompatibilityError) else "failed"
    diagnostics.append(
        PluginDiagnostic(
            status=status,
            plugin_id=plugin_id,
            entry_point=entry_point,
            message=str(error),
        )
    )
    ####


PluginErrorTypes = (PluginCollisionError, PluginCompatibilityError, PluginLoadError)


def _coerce_plugin(value: object, entry_point: str) -> TaoryxPlugin:
    """Accept a plug-in object or a zero-argument plug-in factory."""

    candidate = value
    if not _looks_like_plugin(candidate) and callable(candidate):
        candidate = candidate()
    if not _looks_like_plugin(candidate):
        raise PluginLoadError(f"entry point {entry_point!r} must expose a TaoryxPlugin object or zero-argument factory")
    metadata = getattr(candidate, "metadata")
    if not isinstance(metadata, PluginMetadata):
        raise PluginLoadError(f"entry point {entry_point!r} metadata must be PluginMetadata")
    return cast(TaoryxPlugin, candidate)
    ####


def _looks_like_plugin(value: object) -> bool:
    """Check the small runtime protocol without importing plug-in packages."""

    return isinstance(getattr(value, "metadata", None), PluginMetadata) and callable(getattr(value, "register", None))
    ####


def _installed_entry_points() -> tuple[PluginEntryPoint, ...]:
    """Read only the versioned Taoryx entry-point group."""

    selected = importlib_metadata.entry_points().select(group=PLUGIN_ENTRY_POINT_GROUP)
    return cast(tuple[PluginEntryPoint, ...], tuple(selected))
    ####


def _builtin_plugins(*, excluded_ids: frozenset[str]) -> tuple[TaoryxPlugin, ...]:
    """Import source-checkout fallbacks only when discovery is requested."""

    from taoryx.builtin_plugins import builtin_plugins

    return builtin_plugins(excluded_ids=excluded_ids)
    ####


__all__ = ["PluginCatalog", "PluginEntryPoint", "discover_plugins"]
####
