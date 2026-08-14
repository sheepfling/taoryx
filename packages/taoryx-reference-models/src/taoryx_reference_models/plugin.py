"""Compatibility aggregate plug-in for installed vehicle fragments.

This package deliberately owns no vehicle plant. Family packages register
their adapters, execution factories, and focused providers directly; this
compatibility plug-in retains the historical aggregate Mission Composition
provider for catalogue consumers that intentionally install the full model
set.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from taoryx.plugins import (
    PluginDefinition,
    PluginMetadata,
    PluginRegistrar,
    VehicleCatalogFragment,
    current_plugin_catalog,
)

_REFERENCE_PROVIDER_ID = "taoryx.registry.mission-composition"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.reference-models.vehicle-catalog",
    resource_package="taoryx_reference_models",
)
if TYPE_CHECKING:
    from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider


def _registry_provider() -> RegistryMissionCompositionProvider:
    """Build the legacy aggregate over the exact catalog that selected it."""

    from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
    from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

    plugins = current_plugin_catalog()
    if plugins is None:
        raise RuntimeError("the compatibility aggregate provider must be resolved through a selected plug-in catalog")
    return RegistryMissionCompositionProvider(
        load_resolved_vehicle_composition_catalog(plugins=plugins),
        plugin_catalog=plugins,
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only the aggregate projection over installed family fragments."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    registrar.register_mission_composition_provider_factory(_REFERENCE_PROVIDER_ID, _registry_provider)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.reference-models",
        package="taoryx-reference-models",
        version="0.1.0a0",
        api_version="1",
        description="Compatibility aggregate Mission Composition provider for installed Taoryx vehicle plug-ins.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
