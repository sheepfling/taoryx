"""Deterministic discovery and aggregation for installed Taoryx plug-ins."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Iterable, Iterator, Sequence
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from importlib import metadata as importlib_metadata
from typing import TYPE_CHECKING, Literal, Protocol, cast

from .contracts import (
    PLUGIN_API_VERSION,
    PLUGIN_ENTRY_POINT_GROUP,
    ContributionKind,
    DeferredFamilyAdapterRegistration,
    DeferredMissionCompositionProvider,
    PluginCollisionError,
    PluginCompatibilityError,
    PluginContribution,
    PluginDiagnostic,
    PluginLoadError,
    PluginMetadata,
    PluginRegistrar,
    PluginRevisionMetadata,
    TaoryxPlugin,
)

if TYPE_CHECKING:
    from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
    from taoryx.deployment import DeploymentChildRuntimeRegistry
    from taoryx.family_adapter_registry import FamilyAdapterRegistry
    from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisementRegistry
    from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenRegistry
    from taoryx.local_native_coordinate_lqi_screen_registry import LocalNativeCoordinateLqiScreenRegistry
    from taoryx.reachability_registry import ReachabilityProviderRegistry
    from taoryx.trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
    from taoryx.trajectory.providers import ProviderRegistry
    from taoryx.vehicle_trim_adapters import VehicleTrimEvidenceBinding


class PluginEntryPoint(Protocol):
    """Minimal entry-point surface accepted by discovery and tests."""

    @property
    def name(self) -> str:
        """Return the stable plug-in identifier declared by the distribution."""

        ...

    @property
    def value(self) -> str:
        """Return the standard ``module:attribute`` load target."""

        ...

    def load(self) -> object:
        """Load the referenced Python object."""

        ...


@dataclass(frozen=True, slots=True)
class PluginEntryPointDeclaration:
    """One non-executing declaration of an available Taoryx plug-in."""

    plugin_id: str
    target: str
    origin: Literal["installed", "source_checkout", "provided"]
    distribution: str | None = None
    version: str | None = None

    def public_dict(self) -> dict[str, str | None]:
        """Return the UI/API-safe entry-point advertisement."""

        return {
            "plugin_id": self.plugin_id,
            "target": self.target,
            "origin": self.origin,
            "distribution": self.distribution,
            "version": self.version,
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class _EntryPointCandidate:
    """Keep a loadable entry point paired with its public declaration."""

    entry_point: PluginEntryPoint
    declaration: PluginEntryPointDeclaration

    ####


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
        self._validate_native_coordinate_lqi_screen_advertisements()
        ####

    def _validate_native_coordinate_lqi_screen_advertisements(self) -> None:
        """Keep executable native-LQI screens and UI/agent metadata identical.

        A native-coordinate screen has two intentional registrations: its
        typed executable definition lets core preflight and batch dispatch find
        the package-owned runtime, while the generic local-screen record lets
        authoring, UI, and agent clients inspect it without constructing a
        plant.  They are two projections of one endpoint, not two independent
        contracts that a plug-in may let drift.
        """

        definition_records = self.records("local_native_coordinate_lqi_screen_definition")
        if not definition_records:
            return

        from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisement
        from taoryx.local_native_coordinate_lqi_screen_registry import LocalNativeCoordinateLqiScreenDefinition

        definitions = {
            item.id: (item.plugin, cast(LocalNativeCoordinateLqiScreenDefinition, item.value))
            for item in definition_records
        }
        advertisements = {
            item.id: (item.plugin, cast(LocalControllerScreenAdvertisement, item.value))
            for item in self.records("local_controller_screen_advertisement")
        }
        for identifier, (definition_owner, definition) in definitions.items():
            advertised = advertisements.get(identifier)
            if advertised is None:
                raise ValueError(
                    "local native-coordinate LQI screen "
                    f"{identifier!r} must also register a local controller-screen advertisement"
                )
            advertisement_owner, advertisement = advertised
            if definition_owner.id != advertisement_owner.id:
                raise ValueError(
                    "local native-coordinate LQI screen "
                    f"{identifier!r} and its advertisement must be owned by the same plug-in"
                )
            if (
                advertisement.family_id != definition.family_id
                or advertisement.mission_template_id != definition.mission_id
                or advertisement.fidelity != definition.fidelity
            ):
                raise ValueError(
                    "local native-coordinate LQI screen "
                    f"{identifier!r} has a mismatched advertisement endpoint"
                )
            if definition.public_advertisement() != advertisement.public_advertisement():
                raise ValueError(
                    "local native-coordinate LQI screen "
                    f"{identifier!r} has a mismatched executable and advertised contract"
                )
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

    def plugin_revision(self, plugin_id: str) -> PluginRevisionMetadata:
        """Return one plug-in's scoped revision without catalog-wide coupling.

        The global catalog fingerprint remains the right token for an
        aggregate host.  This narrower projection is for focused UIs, agents,
        and caches that need to know whether *this* plug-in's owned
        advertisements changed while other plug-ins were installed, removed,
        or updated.
        """

        plugin = self.plugin(plugin_id)
        contributions = sorted(
            (item.public_dict() for item in self.contributions if item.plugin.id == plugin.id),
            key=lambda item: (item["kind"], item["id"]),
        )
        payload = {
            "schema": "taoryx.plugin-revision/v1",
            "host_api_version": PLUGIN_API_VERSION,
            "plugin": plugin.public_dict(),
            "contributions": contributions,
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return PluginRevisionMetadata(
            plugin_id=plugin.id,
            package=plugin.package,
            version=plugin.version,
            api_version=plugin.api_version,
            contribution_count=len(contributions),
            fingerprint=hashlib.sha256(encoded).hexdigest(),
        )
        ####

    def plugin_fingerprint(self, plugin_id: str) -> str:
        """Return the deterministic revision fingerprint for one plug-in."""

        return self.plugin_revision(plugin_id).fingerprint
        ####

    @property
    def plugin_revisions(self) -> tuple[PluginRevisionMetadata, ...]:
        """Return every loaded plug-in revision in discovery identity order."""

        return tuple(self.plugin_revision(item.id) for item in self.plugins)
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
            tuple(item.resolve_registration() if isinstance(item, DeferredFamilyAdapterRegistration) else item for item in self.values("family_adapter")),
        )
        return FamilyAdapterRegistry(registrations)
        ####

    def trim_evidence_binding(self, family_id: str) -> VehicleTrimEvidenceBinding:
        """Resolve one selected family-owned trim binding without touching peers."""

        contribution = self.contribution("trim_evidence_binding", family_id)
        value = contribution.value
        resolver = getattr(value, "resolve_binding", None)
        binding = resolver() if callable(resolver) else value
        from taoryx.vehicle_trim_adapters import VehicleTrimEvidenceBinding

        if not isinstance(binding, VehicleTrimEvidenceBinding):
            raise TypeError(
                f"trim-evidence contribution {family_id!r} from {contribution.plugin.id!r} returned an invalid contract"
            )
        if binding.family_id != family_id:
            raise ValueError(
                f"trim-evidence contribution {family_id!r} from {contribution.plugin.id!r} advertised {binding.family_id!r}"
            )
        return binding
        ####

    def build_trajectory_provider_registry(self) -> ProviderRegistry:
        """Project provider contributions into the neutral provider registry."""

        from taoryx.trajectory.providers import ProviderRegistry, TrajectoryProvider

        providers = cast(tuple[TrajectoryProvider, ...], self.values("trajectory_provider"))
        return ProviderRegistry(providers)
        ####

    def build_mission_composition_provider_registry(self) -> ConfigurableTrajectoryProviderRegistry:
        """Project composer-provider contributions into the typed public registry.

        Deferred providers receive this catalog before a caller resolves one.
        That preserves lazy discovery while making a selected host scope
        available to the provider's later batch and session paths.
        """

        from taoryx.trajectory.configuration_contract import (
            ConfigurableTrajectoryProvider,
            ConfigurableTrajectoryProviderRegistry,
        )

        values = self.values("mission_composition_provider")
        for provider in values:
            if isinstance(provider, DeferredMissionCompositionProvider):
                provider.bind_plugin_catalog(self)
        providers = cast(
            tuple[ConfigurableTrajectoryProvider, ...],
            values,
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

    def build_deployment_child_runtime_registry(self) -> DeploymentChildRuntimeRegistry:
        """Project selected child-propagation runtimes into their exact registry."""

        from taoryx.deployment import DeploymentChildRuntime, DeploymentChildRuntimeRegistry

        runtimes = cast(tuple[DeploymentChildRuntime, ...], self.values("deployment_child_runtime"))
        return DeploymentChildRuntimeRegistry(runtimes)
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

    def build_local_direct_wrench_screen_registry(self) -> LocalDirectWrenchScreenRegistry:
        """Project plug-in-owned direct-wrench screens into exact runtime lookup."""

        from taoryx.local_direct_wrench_screen_registry import (
            LocalDirectWrenchScreenDefinition,
            LocalDirectWrenchScreenRegistry,
        )

        definitions = cast(
            tuple[LocalDirectWrenchScreenDefinition, ...],
            self.values("local_direct_wrench_screen_definition"),
        )
        return LocalDirectWrenchScreenRegistry(definitions=definitions)
        ####

    def build_local_native_coordinate_lqi_screen_registry(self) -> LocalNativeCoordinateLqiScreenRegistry:
        """Project plug-in-owned named-coordinate LQI screens into one exact registry."""

        from taoryx.local_native_coordinate_lqi_screen_registry import (
            LocalNativeCoordinateLqiScreenDefinition,
            LocalNativeCoordinateLqiScreenRegistry,
        )

        definitions = cast(
            tuple[LocalNativeCoordinateLqiScreenDefinition, ...],
            self.values("local_native_coordinate_lqi_screen_definition"),
        )
        return LocalNativeCoordinateLqiScreenRegistry(definitions=definitions)
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the discovery result without serializing factories or providers."""

        return {
            "schema": "taoryx.plugin-catalog/v1",
            "api_version": PLUGIN_API_VERSION,
            "fingerprint": self.fingerprint,
            "plugins": [item.public_dict() for item in self.plugins],
            "plugin_revisions": [item.public_dict() for item in self.plugin_revisions],
            "contributions": [item.public_dict() for item in self.contributions],
            "diagnostics": [item.public_dict() for item in self.diagnostics],
        }
        ####

    ####


_ACTIVE_PLUGIN_CATALOG: ContextVar[PluginCatalog | None] = ContextVar("taoryx_active_plugin_catalog", default=None)


def current_plugin_catalog() -> PluginCatalog | None:
    """Return the catalog scoped by an in-progress plug-in execution, if any."""

    return _ACTIVE_PLUGIN_CATALOG.get()
    ####


@contextmanager
def plugin_catalog_scope(catalog: PluginCatalog) -> Iterator[PluginCatalog]:
    """Carry one explicit host selection through nested family runtime calls.

    A vehicle factory may invoke generic preflight, capability, or episode
    helpers internally.  Those helpers must reuse the caller's focused catalog
    rather than launch a second, catalogue-wide discovery scan.
    """

    token = _ACTIVE_PLUGIN_CATALOG.set(catalog)
    try:
        yield catalog
    finally:
        _ACTIVE_PLUGIN_CATALOG.reset(token)
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
    selected: Iterable[str] | None = None,
    disabled: Iterable[str] = (),
    entry_points: Sequence[PluginEntryPoint] | None = None,
    strict: bool = True,
) -> PluginCatalog:
    """Load compatible plug-ins and atomically merge their contributions.

    Every candidate first passes through the standard entry-point declaration
    path.  That gives installed wheels and source-checkout fallbacks the same
    names, targets, ordering, collision behavior, and metadata checks.
    Supplying ``entry_points`` replaces installed-package discovery and is the
    deterministic seam used by tests and embedded hosts. ``selected`` is an
    explicit host scope: plug-ins outside it are not imported or diagnosed.
    In non-strict mode a failing or colliding selected plug-in is omitted and
    represented by a diagnostic.
    """

    disabled_ids = frozenset(item.strip() for item in disabled if item.strip())
    selected_ids = None if selected is None else frozenset(item.strip() for item in selected if item.strip())
    builder = _CatalogBuilder()
    diagnostics: list[PluginDiagnostic] = []
    candidates = _entry_point_candidates(
        include_builtin=include_builtin,
        include_external=include_external,
        entry_points=entry_points,
    )
    if selected_ids is not None:
        declared_ids = frozenset(item.declaration.plugin_id for item in candidates)
        for plugin_id in sorted(selected_ids - declared_ids):
            _handle_failure(
                PluginLoadError(f"selected plug-in {plugin_id!r} has no available {PLUGIN_ENTRY_POINT_GROUP!r} declaration"),
                plugin_id=plugin_id,
                entry_point=None,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
        candidates = tuple(item for item in candidates if item.declaration.plugin_id in selected_ids)

    for candidate in _validated_entry_point_candidates(
        candidates,
        builder=builder,
        diagnostics=diagnostics,
        strict=strict,
    ):
        declaration = candidate.declaration
        entry_point = candidate.entry_point
        if declaration.plugin_id in disabled_ids:
            diagnostics.append(
                PluginDiagnostic(
                    status="disabled",
                    plugin_id=declaration.plugin_id,
                    entry_point=declaration.target,
                    message="disabled by host configuration",
                )
            )
            continue
        try:
            loaded = entry_point.load()
            plugin = _coerce_plugin(loaded, declaration.target)
        except ModuleNotFoundError as error:
            if declaration.origin == "source_checkout" and _missing_source_entry_point_module(error, declaration.target):
                continue
            _handle_failure(
                PluginLoadError(f"could not load plug-in entry point {declaration.target!r}: {error}"),
                plugin_id=declaration.plugin_id,
                entry_point=declaration.target,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
            continue
        except Exception as error:  # noqa: BLE001 - entry points are an untrusted package boundary
            _handle_failure(
                PluginLoadError(f"could not load plug-in entry point {declaration.target!r}: {error}"),
                plugin_id=declaration.plugin_id,
                entry_point=declaration.target,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
            continue
        _load_plugin(
            plugin,
            entry_point=declaration.target,
            expected_id=declaration.plugin_id,
            expected_package=declaration.distribution,
            expected_version=declaration.version,
            disabled_ids=disabled_ids,
            builder=builder,
            diagnostics=diagnostics,
            strict=strict,
        )
    return builder.build(diagnostics)
    ####


def declared_plugin_entry_points(
    *,
    include_builtin: bool = True,
    include_external: bool = True,
    entry_points: Sequence[PluginEntryPoint] | None = None,
) -> tuple[PluginEntryPointDeclaration, ...]:
    """List deterministic plug-in declarations without importing their targets.

    This is the entry-point-level inventory intended for installers, UIs, and
    compliance tooling.  The result uses installed distribution metadata when
    available and the equivalent sibling ``pyproject.toml`` declaration only
    for source-checkout fallback.
    """

    candidates = _entry_point_candidates(
        include_builtin=include_builtin,
        include_external=include_external,
        entry_points=entry_points,
        source_active_only=False,
    )
    return tuple(item.declaration for item in _validated_entry_point_candidates(candidates, strict=True))
    ####


def _entry_point_candidates(
    *,
    include_builtin: bool,
    include_external: bool,
    entry_points: Sequence[PluginEntryPoint] | None,
    source_active_only: bool = True,
) -> tuple[_EntryPointCandidate, ...]:
    """Resolve installed and checkout declarations before any plug-in import."""

    external_entry_points = (
        tuple(entry_points) if entry_points is not None else _installed_entry_points()
    ) if include_external else ()
    external_origin: Literal["installed", "provided"] = "provided" if entry_points is not None else "installed"
    external = tuple(
        _EntryPointCandidate(
            entry_point=item,
            declaration=_entry_point_declaration(item, origin=external_origin),
        )
        for item in external_entry_points
    )
    if not include_builtin:
        return external
    try:
        from taoryx.builtin_plugins import source_plugin_entry_points

        source_entry_points = source_plugin_entry_points(active_only=source_active_only)
    except RuntimeError as error:
        raise PluginLoadError(f"could not read source-checkout plug-in entry points: {error}") from error
    external_ids = frozenset(item.declaration.plugin_id for item in external)
    source = tuple(
        _EntryPointCandidate(
            entry_point=item,
            declaration=PluginEntryPointDeclaration(
                plugin_id=item.name,
                target=item.value,
                origin="source_checkout",
                distribution=item.distribution,
                version=item.version,
            ),
        )
        for item in source_entry_points
        if item.name not in external_ids
    )
    return (*source, *external)
    ####


def _entry_point_declaration(
    entry_point: PluginEntryPoint,
    *,
    origin: Literal["installed", "provided"],
) -> PluginEntryPointDeclaration:
    """Project a standard entry point into a non-executing public record."""

    name = getattr(entry_point, "name", None)
    target = getattr(entry_point, "value", None)
    if not isinstance(name, str) or not isinstance(target, str):
        raise PluginLoadError("plug-in entry point must expose string 'name' and 'value' attributes")
    distribution, version = _entry_point_distribution_metadata(entry_point)
    return PluginEntryPointDeclaration(
        plugin_id=name,
        target=target,
        origin=origin,
        distribution=distribution,
        version=version,
    )
    ####


def _entry_point_distribution_metadata(entry_point: PluginEntryPoint) -> tuple[str | None, str | None]:
    """Read distribution identity when a standard entry point provides it."""

    distribution_name = getattr(entry_point, "distribution", None)
    version = getattr(entry_point, "version", None)
    if isinstance(distribution_name, str) and distribution_name.strip():
        return distribution_name.strip(), version.strip() if isinstance(version, str) and version.strip() else None
    distribution = getattr(entry_point, "dist", None)
    if distribution is None:
        return None, None
    try:
        metadata = distribution.metadata
        name = metadata.get("Name") if metadata is not None else None
        distribution_version = distribution.version
    except (AttributeError, TypeError, ValueError):
        return None, None
    return (
        name.strip() if isinstance(name, str) and name.strip() else None,
        distribution_version.strip() if isinstance(distribution_version, str) and distribution_version.strip() else None,
    )
    ####


def _validated_entry_point_candidates(
    candidates: Sequence[_EntryPointCandidate],
    *,
    builder: _CatalogBuilder | None = None,
    diagnostics: list[PluginDiagnostic] | None = None,
    strict: bool,
) -> tuple[_EntryPointCandidate, ...]:
    """Reject ambiguous or malformed declarations before their targets load."""

    by_plugin_id: dict[str, list[_EntryPointCandidate]] = {}
    for candidate in candidates:
        by_plugin_id.setdefault(candidate.declaration.plugin_id, []).append(candidate)
    valid: list[_EntryPointCandidate] = []
    for plugin_id in sorted(by_plugin_id):
        declared = tuple(sorted(by_plugin_id[plugin_id], key=lambda item: (item.declaration.target, item.declaration.origin)))
        for candidate in declared:
            error = _entry_point_declaration_error(candidate.declaration)
            if error is None:
                continue
            _record_entry_point_failure(
                error,
                plugin_id=candidate.declaration.plugin_id,
                entry_point=candidate.declaration.target,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
            break
        else:
            if len(declared) == 1:
                valid.append(declared[0])
                continue
            targets = ", ".join(
                f"{item.declaration.target!r} ({item.declaration.origin})" for item in declared
            )
            _record_entry_point_failure(
                PluginLoadError(f"plug-in entry-point name {plugin_id!r} is declared more than once: {targets}"),
                plugin_id=plugin_id,
                entry_point=None,
                builder=builder,
                diagnostics=diagnostics,
                strict=strict,
            )
    return tuple(valid)
    ####


def _entry_point_declaration_error(declaration: PluginEntryPointDeclaration) -> PluginLoadError | None:
    """Validate the stable name/target shape before importing plug-in code."""

    if not re.fullmatch(r"[a-z0-9]+(?:[._-][a-z0-9]+)*", declaration.plugin_id):
        return PluginLoadError(f"plug-in entry-point name {declaration.plugin_id!r} is not a valid plug-in ID")
    module_name, separator, attribute_path = declaration.target.partition(":")
    target_parts = (*module_name.split("."), *attribute_path.split("."))
    if (
        not separator
        or not module_name
        or not attribute_path
        or any(not item.isidentifier() for item in target_parts)
    ):
        return PluginLoadError(
            f"plug-in entry point {declaration.plugin_id!r} must use a valid 'module:attribute' target, "
            f"observed {declaration.target!r}"
        )
    return None
    ####


def _record_entry_point_failure(
    error: PluginLoadError,
    *,
    plugin_id: str,
    entry_point: str | None,
    builder: _CatalogBuilder | None,
    diagnostics: list[PluginDiagnostic] | None,
    strict: bool,
) -> None:
    """Apply strict/non-strict semantics before a declaration can import code."""

    del builder  # Declaration failures are pre-registration and cannot mutate a catalog.
    if strict:
        raise error
    if diagnostics is None:
        raise error
    diagnostics.append(
        PluginDiagnostic(
            status="failed",
            plugin_id=plugin_id,
            entry_point=entry_point,
            message=str(error),
        )
    )
    ####


def _missing_source_entry_point_module(error: ModuleNotFoundError, target: str) -> bool:
    """Allow a partial checkout to omit a sibling source package as before."""

    module_name = target.partition(":")[0]
    return error.name == module_name.split(".", maxsplit=1)[0]
    ####


def _normalized_distribution_name(name: str) -> str:
    """Normalize a distribution identity using the PEP 503 comparison rule."""

    return re.sub(r"[-_.]+", "-", name).lower()
    ####


def _load_plugin(
    plugin: TaoryxPlugin,
    *,
    entry_point: str | None,
    expected_id: str,
    expected_package: str | None,
    expected_version: str | None,
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
        if expected_package is not None and _normalized_distribution_name(metadata.package) != _normalized_distribution_name(expected_package):
            raise PluginLoadError(
                f"entry point {entry_point!r} declares distribution {expected_package!r}, "
                f"but plug-in metadata advertises package {metadata.package!r}"
            )
        if expected_version is not None and metadata.version != expected_version:
            raise PluginLoadError(
                f"entry point {entry_point!r} declares distribution version {expected_version!r}, "
                f"but plug-in metadata advertises version {metadata.version!r}"
            )
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


__all__ = [
    "PluginCatalog",
    "PluginEntryPoint",
    "PluginEntryPointDeclaration",
    "current_plugin_catalog",
    "declared_plugin_entry_points",
    "discover_plugins",
    "plugin_catalog_scope",
]
####
