"""Registration for development-only Mission Composition providers."""

from __future__ import annotations

from taoryx.plugins import (
    MissionWorkflowEndpointCatalogFragment,
    PluginDefinition,
    PluginMetadata,
    PluginRegistrar,
    current_plugin_catalog,
)

_REFERENCE_PROVIDER_ID = "taoryx.reference.mission-composition"
_CONTRACT_PROBE_PROVIDER_ID = "taoryx.debug.mission-composition-contract-probe"
_PACKAGE_VERSION = "0.1.0a0"
_WORKFLOW_ENDPOINT_FRAGMENT = MissionWorkflowEndpointCatalogFragment(
    id="taoryx.debug-models.workflow-endpoints",
    resource_package="taoryx_debug_models",
    resource="data/verification/mission_workflow_endpoint_specs.yaml",
    endpoint_ids=(
        "reference-ballistic-3dof-batch",
        "reference-waypoint-3dof-batch",
        "debug-contract-probe-batch",
    ),
)


def _reference_provider() -> object:
    """Construct the analytical provider only when a host selects it."""

    from taoryx.trajectory.reference_mission_composition import ReferenceMissionCompositionProvider

    return ReferenceMissionCompositionProvider(
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
    )
    ####


def _contract_probe_provider() -> object:
    """Construct the contract probe only when a host selects it."""

    from taoryx.trajectory.contract_probe_mission_composition import ContractProbeMissionCompositionProvider

    return ContractProbeMissionCompositionProvider(
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish nonphysical providers independently of vehicle packages."""

    registrar.register_mission_composition_provider_factory(_REFERENCE_PROVIDER_ID, _reference_provider)
    registrar.register_mission_composition_provider_factory(_CONTRACT_PROBE_PROVIDER_ID, _contract_probe_provider)
    registrar.register_mission_workflow_endpoint_catalog_fragment(_WORKFLOW_ENDPOINT_FRAGMENT)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.debug-models",
        package="taoryx-debug-models",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Development-only analytical and contract-probe Mission Composition models.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
