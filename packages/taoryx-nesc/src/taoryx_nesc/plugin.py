"""Plug-in registration for the source-replay NESC two-stage rocket family."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment

from .resources import model_resource_root

if TYPE_CHECKING:
    from taoryx.family_adapter import StandardFamilyAdapter
    from taoryx.family_adapter_probes import AdapterProbeCase
    from taoryx.family_adapter_registry import FamilyAdapterRegistration
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.vehicle_batch_execution import VehicleBatchExecutionRequest
    from taoryx.vehicle_composition import CompiledVehicleComposition

_FAMILY_ID = "reference_nesc_two_stage_rocket"
_PROVIDER_ID = "taoryx.nesc.mission-composition"
_TRANSLATOR_ID = "taoryx.nesc_staged_source_replay.capability.v1"
_PACKAGE_VERSION = "0.1.0a0"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.nesc.vehicle-catalog",
    resource_package="taoryx_nesc",
    family_ids=(_FAMILY_ID,),
)


class _LazyNescCapabilityAdapter:
    """Retain NESC capability identity without importing replay translation on discovery."""

    id = _TRANSLATOR_ID

    def __init__(self) -> None:
        self._delegate: object | None = None
        ####

    def _resolve(self) -> object:
        """Create the family-owned capability adapter only when a composition needs it."""

        if self._delegate is None:
            from .capabilities import NescSourceReplayCapabilityAdapter

            candidate = NescSourceReplayCapabilityAdapter()
            if candidate.id != self.id:
                raise ValueError(f"lazy NESC capability adapter resolved mismatched identity {candidate.id!r}")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Delegate exact family/mission/fidelity ownership when it is selected."""

        return bool(getattr(self._resolve(), "supports")(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Delegate NESC feasibility and source-replay planning on demand."""

        return getattr(self._resolve(), "estimate")(composition)
        ####

    ####


def _nesc_source_replay_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Exercise the exact packaged source history through the replay seam."""

    from taoryx.family_adapter_probes import AdapterProbeCase

    del adapter
    artifact = model_resource_root() / "verification/daveml_nesc_reduction_qualification.json"
    return AdapterProbeCase(state={}, effectors={}, replay_request={"source_artifact": str(artifact)})
    ####


def _registration() -> FamilyAdapterRegistration:
    """Return the NESC-owned replay adapter registration without loading data."""

    from taoryx.nesc_adapter import build_nesc_replay_adapter

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        _FAMILY_ID,
        "taoryx.rocket.variable_mass_nesc.v1",
        "available",
        build_nesc_replay_adapter,
        probe_factory=_nesc_source_replay_probe,
        supported_tiers=("point_mass_3dof", "pseudo_6dof"),
        note="pinned source replay adapter; active segment translation remains intentionally unavailable",
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build a NESC-only portable Mission Composition provider.

    The deferred plug-in proxy caches this object per discovery catalog. Do
    not add a module-global cache here: an aggregate host must not make a later
    focused NESC host inherit its broader execution scope.
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
        provider_name="TAORYX NESC Mission Composition",
        provider_short_name="NESC",
        provider_summary="Pinned NASA/NESC two-stage source replay and named pseudo-6DOF response surrogate.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _execute_nesc_source_replay(request: VehicleBatchExecutionRequest) -> Any:
    """Run the batch-native source replay through its family-owned runtime."""

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the NESC source-replay factory")
    from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition

    return execute_nesc_source_replay_composition(request.composition, request.output_dir)
    ####


def _preflight_nesc_source_replay(composition: CompiledVehicleComposition) -> Any:
    """Load the source-replay semantic preflight only when it is selected."""

    from .preflight import preflight_nesc_source_replay

    return preflight_nesc_source_replay(composition)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only NESC-owned adapters, metadata, preflight, and execution."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    registrar.register_mission_capability_adapter(_LazyNescCapabilityAdapter())
    registrar.register_semantic_preflight_handler_callback(_TRANSLATOR_ID, _preflight_nesc_source_replay)
    registrar.register_execution_factory_request_v1("nesc_source_replay.v1", _execute_nesc_source_replay)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.nesc",
        package="taoryx-nesc",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="NASA/NESC two-stage source replay, response surrogate, and stage-separation parent contract.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
