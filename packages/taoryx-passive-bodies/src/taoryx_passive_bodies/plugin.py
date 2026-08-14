"""Plug-in registration for reusable passive released-body models."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment

from .resources import model_resource_root

if TYPE_CHECKING:
    from taoryx.deployment import DeploymentBinding, DeploymentChildExecution, DeploymentReleaseRequest
    from taoryx.family_adapter import StandardFamilyAdapter
    from taoryx.family_adapter_registry import FamilyAdapterRegistration
    from taoryx.fidelity_contracts import FidelityTier
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.vehicle_composition import CompiledVehicleComposition

_PROVIDER_ID = "taoryx.passive-bodies.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_CAPABILITY_ID = "taoryx.passive_tumbling_release.capability.v1"
_CHILD_RUNTIME_ID = "taoryx.passive-bodies.local-atmosphere-release.v1"
_FAMILY_ID = "tumbling_body"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.passive-bodies.vehicle-catalog",
    resource_package="taoryx_passive_bodies",
    family_ids=(_FAMILY_ID,),
)


class _LazyMissionCapabilityAdapter:
    """Keep passive release identity cheap until a composition selects it."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> object:
        """Build and validate the package-owned capability adapter on first use."""

        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy passive capability adapter resolved mismatched identity {self.id!r}")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Inspect only the selected composition before loading its planner."""

        return bool(getattr(self._resolve(), "supports")(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Delegate the selected passive release estimate to its owner."""

        return getattr(self._resolve(), "estimate")(composition)
        ####

    ####


class _LazyDeploymentChildRuntime:
    """Advertise a child-runtime identity without constructing its propagator."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> object:
        """Build and validate the child propagator when a parent binds it."""

        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy passive child runtime resolved mismatched identity {self.id!r}")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, binding: DeploymentBinding) -> bool:
        """Load the propagator only when a parent asks whether it can bind."""

        return bool(getattr(self._resolve(), "supports")(binding))
        ####

    def execute(self, request: DeploymentReleaseRequest) -> DeploymentChildExecution:
        """Delegate accepted releases to the package-owned propagator."""

        return getattr(self._resolve(), "execute")(request)
        ####

    ####


def _passive_tumbling_capability_adapter() -> object:
    """Load the passive release planner only when a mission selects it."""

    from .capabilities import PassiveTumblingReleaseCapabilityAdapter

    return PassiveTumblingReleaseCapabilityAdapter()
    ####


def _preflight_passive_tumbling_release(composition: CompiledVehicleComposition) -> object:
    """Load the passive semantic preflight only when it is dispatched."""

    from .preflight import _preflight_passive_tumbling_release as preflight

    return preflight(composition)
    ####


def _passive_local_atmosphere_release_runtime() -> object:
    """Load the child propagator only when an explicit parent binding uses it."""

    from .deployment_runtime import PassiveLocalAtmosphereReleaseRuntime

    return PassiveLocalAtmosphereReleaseRuntime()
    ####


def _passive_body_interface_extension() -> object:
    """Load passive direct-release truth readback only when selected."""

    from taoryx.passive_body_interface import passive_body_interface_extension

    return passive_body_interface_extension()
    ####


def _passive_tumbling_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Expose the two executable passive-body reductions without fake controls."""

    from taoryx.family_adapter import AdapterChannel, FamilyAdapterDescriptor, StandardFamilyAdapter

    state_channels = [
        AdapterChannel("position_m", "m", "state", frame="local_tangent"),
        AdapterChannel("velocity_m_s", "m/s", "state", frame="local_tangent"),
        AdapterChannel("projected_area_m2", "m^2", "state", frame="body"),
        AdapterChannel("drag_force_n", "N", "state", frame="local_tangent"),
    ]
    if tier == "pseudo_6dof":
        state_channels.extend(
            (
                AdapterChannel("attitude_quaternion", "dimensionless", "state", frame="body-to-local"),
                AdapterChannel("attitude_rate_rad_s", "rad/s", "state", frame="body"),
                AdapterChannel("angular_rate_norm_rad_s", "rad/s", "state", frame="body"),
            )
        )
        validity_envelope = "Declared direct release only; native rigid-body passive-tumble equations are reused for committed truth telemetry."
    elif tier == "point_mass_3dof":
        state_channels.append(AdapterChannel("angular_rate_norm_rad_s", "rad/s", "state", frame="body"))
        validity_envelope = "Declared direct release only; orientation-averaged projected area is an explicit 3DOF reduction."
    else:
        raise ValueError(f"tumbling_body: unsupported passive adapter tier {tier!r}")
    descriptor = FamilyAdapterDescriptor(
        family_id="tumbling_body",
        adapter_id="taoryx.passive_body.rigid_aero.v1",
        physical_family="passive_ballistic_tumbling_body",
        tier=tier,
        state_channels=tuple(state_channels),
        resource_channels=(AdapterChannel("mass_kg", "kg", "resource"),),
        control_realization_override="uncontrolled",
        evidence_status="development",
        validity_envelope=validity_envelope,
        omitted_physics=(
            "controller and control law",
            "wrench command and actuator allocation",
            "actuator dynamics",
            "shape-specific aerodynamic-moment and damping qualification",
        ),
    )
    return StandardFamilyAdapter.passive(descriptor)
    ####


def _registration() -> FamilyAdapterRegistration:
    """Return the passive family adapter only when the host requests it."""

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        "tumbling_body",
        "taoryx.passive_body.rigid_aero.v1",
        "available",
        _passive_tumbling_adapter,
        supported_tiers=("point_mass_3dof", "pseudo_6dof"),
        note="batch-only passive direct-release reductions; no controller, derivative, trim, allocator, or physical-effector path is implied",
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build a passive-bodies-only portable composition provider.

    The deferred plug-in proxy caches this object per discovery catalog. Do
    not add a module-global cache here: a broad host must not make a later
    focused host inherit its wider execution scope.
    """

    from taoryx.family_manifest import load_unified_family_manifest_catalog
    from taoryx.horizontal_fidelity import load_horizontal_registry
    from taoryx.plugins import current_plugin_catalog
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
    from taoryx.vehicle_composition_registry import (
        load_resolved_vehicle_composition_catalog,
        load_vehicle_composition_registry,
    )

    root = model_resource_root()
    pseudo = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml")
    horizontal = load_horizontal_registry(root / "verification/horizontal_fidelity_registry.yaml")
    manifests = load_unified_family_manifest_catalog(
        horizontal=horizontal,
        pseudo=pseudo,
        root=root,
        validate_source_imports=False,
    )
    registry = load_vehicle_composition_registry(root / "verification/vehicle_composition_registry.yaml")
    catalog = load_resolved_vehicle_composition_catalog(registry=registry, manifests=manifests)
    return CatalogMissionCompositionProvider(
        catalog,
        provider_id=_PROVIDER_ID,
        provider_name="TAORYX Passive Bodies Mission Composition",
        provider_short_name="Passive Bodies",
        provider_summary="Reusable passive released-body models and direct-release configuration surface.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _execute_passive_tumbling(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    """Run the explicit direct-release witness without a controller path."""

    if max_steps is not None:
        raise ValueError("--max-steps is not available for the passive direct-release factory")
    from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition

    return execute_passive_tumbling_composition(composition, output_dir)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only passive-body-owned data, runtime, and semantic contracts."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_vehicle_interface_extension_factory(_FAMILY_ID, _passive_body_interface_extension)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    registrar.register_mission_capability_adapter(_LazyMissionCapabilityAdapter(_CAPABILITY_ID, _passive_tumbling_capability_adapter))
    registrar.register_semantic_preflight_handler_callback(_CAPABILITY_ID, _preflight_passive_tumbling_release)
    registrar.register_deployment_child_runtime(_LazyDeploymentChildRuntime(_CHILD_RUNTIME_ID, _passive_local_atmosphere_release_runtime))
    registrar.register_execution_factory("passive_tumbling_direct_release.v1", _execute_passive_tumbling)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.passive-bodies",
        package="taoryx-passive-bodies",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Reusable passive released bodies, direct-release witnesses, and independent child propagation.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
