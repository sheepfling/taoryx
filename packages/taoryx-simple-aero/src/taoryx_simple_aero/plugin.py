"""Taoryx entry point for the Simple Aero provider package."""

from __future__ import annotations

from taoryx.plugins import (
    MissionWorkflowEndpointCatalogFragment,
    PluginDefinition,
    PluginMetadata,
    PluginRegistrar,
    current_plugin_catalog,
)

from .provider import ReferencePointMassProvider

_PACKAGE_VERSION = "0.1.0a0"
_MISSION_COMPOSITION_PROVIDER_ID = "taoryx.simple-aero.mission-composition"
_WORKFLOW_ENDPOINT_FRAGMENT = MissionWorkflowEndpointCatalogFragment(
    id="taoryx.simple-aero.workflow-endpoints",
    resource_package="taoryx_simple_aero",
    resource="data/verification/mission_workflow_endpoint_specs.yaml",
    endpoint_ids=("simple-aero-fixed-ld-batch",),
)


def _mission_composition_provider() -> object:
    """Build the focused workflow API only after a host selects it."""

    from .mission_composition import SimpleAeroMissionCompositionProvider

    return SimpleAeroMissionCompositionProvider(
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the neutral provider and the focused workflow API independently."""

    registrar.register_trajectory_provider(ReferencePointMassProvider())
    registrar.register_mission_composition_provider_factory(
        _MISSION_COMPOSITION_PROVIDER_ID,
        _mission_composition_provider,
    )
    registrar.register_mission_workflow_endpoint_catalog_fragment(_WORKFLOW_ENDPOINT_FRAGMENT)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.simple-aero",
        package="taoryx-simple-aero",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Analytical Simple Aero reference models and trajectory providers.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
