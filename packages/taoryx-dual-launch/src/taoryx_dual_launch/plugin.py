"""Taoryx entry point for the package-owned dual-launch workflow."""

from __future__ import annotations

from taoryx.plugins import (
    MissionWorkflowEndpointCatalogFragment,
    PluginDefinition,
    PluginMetadata,
    PluginRegistrar,
    current_plugin_catalog,
)

_PACKAGE_VERSION = "0.1.0a0"
_MISSION_COMPOSITION_PROVIDER_ID = "taoryx.dual-launch.mission-composition"
_WORKFLOW_ENDPOINT_FRAGMENT = MissionWorkflowEndpointCatalogFragment(
    id="taoryx.dual-launch.workflow-endpoints",
    resource_package="taoryx_dual_launch",
    resource="data/verification/mission_workflow_endpoint_specs.yaml",
    endpoint_ids=("dual-launch-attached-booster-batch",),
)


def _mission_composition_provider() -> object:
    """Build the one-model focused provider only when a host selects it."""

    from .mission_composition import DualLaunchMissionCompositionProvider

    return DualLaunchMissionCompositionProvider(
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the workflow API and its package-owned endpoint witness."""

    registrar.register_mission_composition_provider_factory(
        _MISSION_COMPOSITION_PROVIDER_ID,
        _mission_composition_provider,
    )
    registrar.register_mission_workflow_endpoint_catalog_fragment(_WORKFLOW_ENDPOINT_FRAGMENT)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.dual-launch",
        package="taoryx-dual-launch",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Package-owned source-generated dual-launch glider Mission Composition workflow.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
