"""Versioned contracts for installable Taoryx plug-ins.

The plug-in API is deliberately smaller than any one vehicle implementation.
It carries stable identity, compatibility, and typed contributions into the
host without making discovery construct a plant or execute a trajectory.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field

if TYPE_CHECKING:
    from .discovery import PluginCatalog

PLUGIN_API_VERSION = "1"
PLUGIN_ENTRY_POINT_GROUP = "taoryx.plugins"
BATCH_FACTORY_REQUEST_CONTRACT = "taoryx.vehicle-batch-factory-request/v1alpha1"

ContributionKind = Literal[
    "batch_episode_parity_verifier",
    "controller_tuning_campaign",
    "deployment_child_runtime",
    "episode_factory",
    "execution_factory",
    "family_adapter",
    "local_controller_screen_advertisement",
    "local_direct_wrench_screen_definition",
    "local_native_coordinate_lqi_screen_definition",
    "mission_capability_adapter",
    "mission_composition_provider",
    "mission_workflow_endpoint_catalog",
    "model",
    "model_format",
    "reachability_provider",
    "semantic_preflight_handler",
    "trajectory_provider",
    "trim_evidence_binding",
    "vehicle_catalog_fragment",
    "vehicle_catalog_overlay_fragment",
    "vehicle_interface_extension",
]
CONTRIBUTION_KINDS: tuple[ContributionKind, ...] = (
    "batch_episode_parity_verifier",
    "controller_tuning_campaign",
    "deployment_child_runtime",
    "episode_factory",
    "execution_factory",
    "family_adapter",
    "local_controller_screen_advertisement",
    "local_direct_wrench_screen_definition",
    "local_native_coordinate_lqi_screen_definition",
    "mission_capability_adapter",
    "mission_composition_provider",
    "mission_workflow_endpoint_catalog",
    "model",
    "model_format",
    "reachability_provider",
    "semantic_preflight_handler",
    "trajectory_provider",
    "trim_evidence_binding",
    "vehicle_catalog_fragment",
    "vehicle_catalog_overlay_fragment",
    "vehicle_interface_extension",
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


class PluginRevisionMetadata(BaseModel):
    """Scoped installed revision projection for one discoverable plug-in.

    ``version`` is the package-maintained release or compatibility identifier.
    ``fingerprint`` is derived from the plug-in identity and only the typed
    contributions owned by that plug-in in the current host catalog.  A client
    can therefore refresh one plug-in's advertisements without treating an
    unrelated installed package as a change to that plug-in.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.plugin-revision/v1"] = Field(
        default="taoryx.plugin-revision/v1",
        alias="schema",
        serialization_alias="schema",
    )
    plugin_id: str = Field(min_length=1)
    package: str = Field(min_length=1)
    version: str = Field(min_length=1)
    api_version: str = Field(min_length=1)
    contribution_count: int = Field(ge=0)
    fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")

    def public_dict(self) -> dict[str, object]:
        """Return the stable JSON projection used by discovery clients."""

        return self.model_dump(mode="json", by_alias=True)
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


@dataclass(frozen=True, slots=True)
class MissionWorkflowEndpointCatalogFragment:
    """Package-owned nonphysical-workflow endpoint catalog fragment.

    The fragment is resource-only: discovery can advertise its stable identity
    without parsing endpoint YAML or constructing a model provider. The common
    workflow verifier loads and validates it only when a caller requests the
    endpoint records.
    """

    id: str
    resource_package: str
    resource: str
    endpoint_ids: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("workflow endpoint catalog fragment ID must not be empty")
        if not self.resource_package.strip():
            raise ValueError("workflow endpoint catalog fragment resource_package must not be empty")
        if not self.resource.strip():
            raise ValueError("workflow endpoint catalog fragment resource must not be empty")
        if not self.endpoint_ids or any(not item.strip() for item in self.endpoint_ids):
            raise ValueError("workflow endpoint catalog fragment must name at least one endpoint")
        if len(self.endpoint_ids) != len(set(self.endpoint_ids)):
            raise ValueError("workflow endpoint catalog fragment has duplicate endpoint IDs")
        ####

    ####


@dataclass(frozen=True, slots=True)
class VehicleCatalogFragment:
    """Package-owned root for one vehicle family's checked-in catalog data.

    The declaration is intentionally metadata-only: plug-in discovery records
    the owner and resource root without reading YAML, constructing a plant, or
    walking a source checkout.  A caller that supplies a selected
    :class:`PluginCatalog` can then resolve only these contributed resources,
    rather than relying on the core's compatibility package list.
    """

    id: str
    resource_package: str
    resource_root: str = "data"
    family_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("vehicle catalog fragment ID must not be empty")
        if not self.resource_package.strip():
            raise ValueError("vehicle catalog fragment resource_package must not be empty")
        normalized_root = self.resource_root.strip().strip("/")
        if not normalized_root or normalized_root == "." or ".." in normalized_root.split("/"):
            raise ValueError("vehicle catalog fragment resource_root must be a nonempty relative path")
        if any(not family_id.strip() for family_id in self.family_ids):
            raise ValueError("vehicle catalog fragment family_ids must not contain blanks")
        if len(self.family_ids) != len(set(self.family_ids)):
            raise ValueError("vehicle catalog fragment family_ids must be unique")
        ####

    ####


@dataclass(frozen=True, slots=True)
class VehicleCatalogOverlayFragment:
    """Package-owned additive catalog rows for an already selected family.

    An overlay is deliberately distinct from a family catalog fragment. It can
    append a narrowly owned mission, binding, or witness to a vehicle package,
    but cannot make that vehicle discoverable by itself. The host must select
    every named base fragment before it resolves the overlay's resources.
    """

    id: str
    resource_package: str
    extends_fragment_ids: tuple[str, ...]
    resource_root: str = "data"
    family_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("vehicle catalog overlay fragment ID must not be empty")
        if not self.resource_package.strip():
            raise ValueError("vehicle catalog overlay fragment resource_package must not be empty")
        normalized_root = self.resource_root.strip().strip("/")
        if not normalized_root or normalized_root == "." or ".." in normalized_root.split("/"):
            raise ValueError("vehicle catalog overlay fragment resource_root must be a nonempty relative path")
        if not self.extends_fragment_ids or any(not identifier.strip() for identifier in self.extends_fragment_ids):
            raise ValueError("vehicle catalog overlay fragment must name at least one base fragment")
        if len(self.extends_fragment_ids) != len(set(self.extends_fragment_ids)):
            raise ValueError("vehicle catalog overlay fragment extends_fragment_ids must be unique")
        if not self.family_ids or any(not family_id.strip() for family_id in self.family_ids):
            raise ValueError("vehicle catalog overlay fragment family_ids must name at least one family")
        if len(self.family_ids) != len(set(self.family_ids)):
            raise ValueError("vehicle catalog overlay fragment family_ids must be unique")
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


class DeferredMissionCompositionProvider:
    """Lazy proxy for a family-owned Mission Composition provider.

    Plug-in discovery needs the stable contribution ID, but it should not have
    to parse a family's catalogs or construct its configuration schemas.  The
    proxy exposes that declared ID to registry construction and resolves the
    real object only when a host asks for provider metadata or a configuration
    API.
    """

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self._identifier = identifier
        self._factory = factory
        self._provider: object | None = None
        self._plugin_catalog: PluginCatalog | None = None
        ####

    @property
    def __taoryx_provider_id__(self) -> str:
        """Return the registration-time identity without constructing the provider.

        ``ConfigurableTrajectoryProviderRegistry`` recognizes this narrow
        internal advertisement so it can index deferred providers without
        turning provider-registry construction into a numerical-runtime load.
        The concrete provider ID is still checked when it is first resolved.
        """

        return self._identifier
        ####

    def bind_plugin_catalog(self, catalog: PluginCatalog) -> None:
        """Retain the host's selected catalog for deferred provider construction.

        This binding is intentionally made while the host builds its provider
        registry, before the factory is resolved.  It lets a focused provider
        retain the exact plug-in scope for later batch and session execution
        without making ordinary discovery eagerly construct its schemas.
        """

        self._plugin_catalog = catalog
        ####

    def _within_plugin_catalog(self, callback: Callable[[], Any]) -> Any:
        """Run a deferred-provider operation under its selected host scope."""

        if self._plugin_catalog is None:
            return callback()
        from .discovery import plugin_catalog_scope

        with plugin_catalog_scope(self._plugin_catalog):
            return callback()
        ####

    def _call_provider(self, name: str, *args: object, **kwargs: object) -> Any:
        """Call one concrete provider method without widening its plug-in scope."""

        method = getattr(self.resolve_provider(), name)
        if not callable(method):
            raise TypeError(f"deferred Mission Composition provider attribute {name!r} is not callable")
        return self._within_plugin_catalog(lambda: method(*args, **kwargs))
        ####

    def resolve_provider(self) -> object:
        """Return the cached concrete provider, validating its advertised ID."""

        if self._provider is None:
            provider = self._within_plugin_catalog(self._factory)
            metadata = getattr(provider, "metadata", None)
            if _string_attribute(metadata, "id") != self._identifier:
                raise PluginError(f"deferred Mission Composition provider {self._identifier!r} resolved a mismatched metadata ID")
            self._provider = provider
        return self._provider
        ####

    @property
    def metadata(self) -> Any:
        """Return concrete provider metadata when a host needs it."""

        return self._within_plugin_catalog(lambda: getattr(self.resolve_provider(), "metadata"))
        ####

    def list_models(self) -> Any:
        """Delegate model discovery only after a host selects this provider."""

        return self._call_provider("list_models")
        ####

    def get_model_schema(self, model_id: str) -> Any:
        """Delegate schema lookup to the selected concrete provider."""

        return self._call_provider("get_model_schema", model_id)
        ####

    def get_model_output_schema(self, model_id: str) -> Any:
        """Delegate output-schema lookup to the selected concrete provider."""

        return self._call_provider("get_model_output_schema", model_id)
        ####

    def validate_configuration(self, configuration: object) -> Any:
        """Delegate configuration validation to the selected concrete provider."""

        return self._call_provider("validate_configuration", configuration)
        ####

    def build_runner(self) -> Any:
        """Delegate the optional common execution surface through the lazy boundary.

        ``RunnableMissionCompositionProvider`` is a runtime-checkable protocol.
        Exposing this method directly (rather than only through ``__getattr__``)
        lets a focused deferred provider satisfy that protocol without eagerly
        constructing its package-owned runner during discovery.
        """

        return self._call_provider("build_runner")
        ####

    def __getattr__(self, name: str) -> Any:
        """Preserve provider-specific helpers such as ``model`` transparently."""

        attribute = getattr(self.resolve_provider(), name)
        if not callable(attribute):
            return attribute

        def delegated(*args: object, **kwargs: object) -> Any:
            return self._call_provider(name, *args, **kwargs)
            ####

        return delegated
        ####

    ####


class DeferredFamilyAdapterRegistration:
    """Lazy proxy for one family adapter's numerical registration.

    The stable family identity is sufficient for plug-in discovery.  The
    concrete registration, and therefore its plant-facing numerical imports,
    are created only when a host asks to build the family-adapter registry.
    """

    def __init__(self, family_id: str, factory: Callable[[], object]) -> None:
        self.family_id = family_id
        self._factory = factory
        self._registration: object | None = None
        ####

    def resolve_registration(self) -> object:
        """Return the concrete registration and validate its family identity."""

        if self._registration is None:
            registration = self._factory()
            if _string_attribute(registration, "family_id") != self.family_id:
                raise PluginError(f"deferred family adapter {self.family_id!r} resolved a mismatched family identity")
            self._registration = registration
        return self._registration
        ####

    def __getattr__(self, name: str) -> Any:
        """Expose the concrete registration when a host needs more than identity."""

        return getattr(self.resolve_registration(), name)
        ####

    ####


class DeferredTrimEvidenceBinding:
    """Lazy proxy for one family-owned source trim-evidence binding.

    Discovery needs to advertise that a family can solve its declared trim
    worklist, but it must not import a DAVE-ML evaluator or package resources
    until a caller selects that exact source family.
    """

    def __init__(self, family_id: str, factory: Callable[[], object]) -> None:
        self.family_id = family_id
        self._factory = factory
        self._binding: object | None = None
        ####

    def resolve_binding(self) -> object:
        """Return the cached binding and validate its stable family identity."""

        if self._binding is None:
            binding = self._factory()
            if _string_attribute(binding, "family_id") != self.family_id:
                raise PluginError(f"deferred trim-evidence binding {self.family_id!r} resolved a mismatched family identity")
            self._binding = binding
        return self._binding
        ####

    ####


class DeferredSemanticPreflightHandler:
    """Lightweight semantic-preflight registration for a lazy family module."""

    def __init__(self, translator_id: str, handler: Callable[[Any], object]) -> None:
        self.translator_id = translator_id
        self.handler = handler
        ####

    ####


class DeferredVehicleInterfaceExtension:
    """Lazy family-owned augmentation for a resolved semantic interface.

    Interface discovery should not import a family's controller or reduced
    response implementation.  The host retains the stable family identity at
    plug-in discovery, then constructs the small interface augmentation only
    when that exact family/fidelity contract is requested.
    """

    def __init__(self, family_id: str, factory: Callable[[], object]) -> None:
        self.family_id = family_id
        self._factory = factory
        self._extension: object | None = None
        ####

    def resolve_extension(self) -> object:
        """Return the package-owned extension and validate its family ID."""

        if self._extension is None:
            extension = self._factory()
            if _string_attribute(extension, "family_id") != self.family_id:
                raise PluginError(
                    f"deferred vehicle interface extension {self.family_id!r} resolved a mismatched family identity"
                )
            self._extension = extension
        return self._extension
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
            raise PluginCollisionError(f"plug-in {self._plugin.id!r} registered duplicate {kind} contribution {normalized!r}")
        self._contributions[key] = PluginContribution(kind, normalized, self._plugin, value)
        ####

    def register_family_adapter(self, registration: object) -> None:
        """Stage a :class:`FamilyAdapterRegistration` by family identity."""

        self.register("family_adapter", _string_attribute(registration, "family_id"), registration)
        ####

    def register_family_adapter_factory(
        self,
        family_id: str,
        factory: Callable[[], object],
    ) -> DeferredFamilyAdapterRegistration:
        """Stage a numerical family-adapter factory without constructing its plant.

        The factory's concrete registration is checked against ``family_id``
        when the family-adapter registry is built.
        """

        if not callable(factory):
            raise TypeError("family adapter factory must be callable")
        normalized = family_id.strip()
        registration = DeferredFamilyAdapterRegistration(normalized, factory)
        self.register("family_adapter", normalized, registration)
        return registration
        ####

    def register_trim_evidence_binding_factory(
        self,
        family_id: str,
        factory: Callable[[], object],
    ) -> DeferredTrimEvidenceBinding:
        """Stage a family-owned trim binding without loading its source evaluator.

        The contribution ID is the source family ID used by the generic trim
        API. The concrete binding is checked only when that family is selected.
        """

        if not callable(factory):
            raise TypeError("trim evidence binding factory must be callable")
        normalized = family_id.strip()
        binding = DeferredTrimEvidenceBinding(normalized, factory)
        self.register("trim_evidence_binding", normalized, binding)
        return binding
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

    def register_mission_composition_provider_factory(
        self,
        identifier: str,
        factory: Callable[[], object],
    ) -> None:
        """Stage a provider factory without constructing its catalog at discovery.

        ``identifier`` is the declared provider metadata ID.  The host checks
        that the object built by ``factory`` has the same ID before exposing
        it through the normal provider API.
        """

        if not callable(factory):
            raise TypeError("Mission Composition provider factory must be callable")
        normalized = identifier.strip()
        self.register(
            "mission_composition_provider",
            normalized,
            DeferredMissionCompositionProvider(normalized, factory),
        )
        ####

    def register_mission_workflow_endpoint_catalog_fragment(
        self,
        fragment: MissionWorkflowEndpointCatalogFragment,
    ) -> None:
        """Stage one package-owned workflow endpoint fragment by stable identity."""

        if not isinstance(fragment, MissionWorkflowEndpointCatalogFragment):
            raise TypeError("workflow endpoint catalog contribution must use MissionWorkflowEndpointCatalogFragment")
        self.register("mission_workflow_endpoint_catalog", fragment.id, fragment)
        ####

    def register_vehicle_catalog_fragment(self, fragment: VehicleCatalogFragment) -> None:
        """Stage one package-owned vehicle-catalog root by stable identity."""

        if not isinstance(fragment, VehicleCatalogFragment):
            raise TypeError("vehicle catalog contribution must use VehicleCatalogFragment")
        self.register("vehicle_catalog_fragment", fragment.id, fragment)
        ####

    def register_vehicle_catalog_overlay_fragment(self, fragment: VehicleCatalogOverlayFragment) -> None:
        """Stage additive data owned by an optional vehicle-workflow overlay."""

        if not isinstance(fragment, VehicleCatalogOverlayFragment):
            raise TypeError("vehicle catalog overlay contribution must use VehicleCatalogOverlayFragment")
        self.register("vehicle_catalog_overlay_fragment", fragment.id, fragment)
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

    def register_semantic_preflight_handler_callback(
        self,
        translator_id: str,
        handler: Callable[[Any], object],
    ) -> None:
        """Stage a lazy family preflight callback without importing its host type."""

        if not callable(handler):
            raise TypeError("semantic preflight handler callback must be callable")
        normalized = translator_id.strip()
        self.register(
            "semantic_preflight_handler",
            normalized,
            DeferredSemanticPreflightHandler(normalized, handler),
        )
        ####

    def register_execution_factory(self, identifier: str, factory: object) -> None:
        """Stage one callable execution factory under its advertised ID.

        New plug-ins should use :meth:`register_execution_factory_request_v1`
        so discovery does not need to import the batch host. Existing
        ``(composition, output_dir, max_steps)`` callables remain a temporary
        compatibility surface.
        """

        if not callable(factory):
            raise TypeError("execution factory contributions must be callable")
        self.register("execution_factory", identifier, factory)
        ####

    def register_execution_factory_request_v1(self, identifier: str, factory: object) -> None:
        """Stage a typed batch factory without importing the batch host at discovery.

        The common host recognizes the stable marker placed on ``factory`` and
        supplies one complete ``VehicleBatchExecutionRequest`` when that
        endpoint is actually invoked.
        """

        if not callable(factory):
            raise TypeError("typed batch execution factories must be callable")
        setattr(factory, "__taoryx_batch_factory_contract__", BATCH_FACTORY_REQUEST_CONTRACT)
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

    def register_deployment_child_runtime(self, runtime: object) -> None:
        """Stage one independently installable child-propagation runtime."""

        self.register("deployment_child_runtime", _string_attribute(runtime, "id"), runtime)
        ####

    def register_local_controller_screen_advertisement(self, registration: object) -> None:
        """Stage static metadata for one exact plug-in-owned local screen."""

        self.register(
            "local_controller_screen_advertisement",
            _string_attribute(registration, "id"),
            registration,
        )
        ####

    def register_local_direct_wrench_screen_definition(self, definition: object) -> None:
        """Stage one executable direct-wrench screen owned by a vehicle plug-in.

        The common host resolves this typed contribution during preflight,
        batch execution, interactive episodes, and authoring introspection.
        Keeping it distinct from a generic static advertisement prevents core
        from importing a family module by name to find its screen factory.
        """

        self.register(
            "local_direct_wrench_screen_definition",
            _string_attribute(definition, "id"),
            definition,
        )
        ####

    def register_local_native_coordinate_lqi_screen_definition(self, definition: object) -> None:
        """Stage one executable named-coordinate LQI screen owned by a plug-in.

        The definition carries its exact configuration factory as well as the
        static advertisement.  It is distinct from a generic controller-screen
        advertisement because the common batch/preflight host must resolve the
        family-owned runtime without importing a model module by name.
        """

        self.register(
            "local_native_coordinate_lqi_screen_definition",
            _string_attribute(definition, "id"),
            definition,
        )
        ####

    def register_vehicle_interface_extension_factory(
        self,
        family_id: str,
        factory: Callable[[], object],
    ) -> DeferredVehicleInterfaceExtension:
        """Stage a lazy family-owned semantic-interface augmentation.

        The extension can add only family-specific advertised actions,
        readbacks, resources, diagnostics, and authority profiles.  Core
        retains interface validation and rejects duplicate public IDs.
        """

        if not callable(factory):
            raise TypeError("vehicle interface extension factory must be callable")
        normalized = family_id.strip()
        extension = DeferredVehicleInterfaceExtension(normalized, factory)
        self.register("vehicle_interface_extension", normalized, extension)
        return extension
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
    "BATCH_FACTORY_REQUEST_CONTRACT",
    "ContributionKind",
    "DeferredFamilyAdapterRegistration",
    "DeferredMissionCompositionProvider",
    "DeferredSemanticPreflightHandler",
    "DeferredTrimEvidenceBinding",
    "DeferredVehicleInterfaceExtension",
    "MissionWorkflowEndpointCatalogFragment",
    "VehicleCatalogFragment",
    "VehicleCatalogOverlayFragment",
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
