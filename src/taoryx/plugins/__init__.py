"""Public SDK and discovery surface for installable Taoryx plug-ins."""

from .contracts import (
    CONTRIBUTION_KINDS,
    PLUGIN_API_VERSION,
    PLUGIN_ENTRY_POINT_GROUP,
    ContributionKind,
    PluginCollisionError,
    PluginCompatibilityError,
    PluginContribution,
    PluginDefinition,
    PluginDiagnostic,
    PluginError,
    PluginLoadError,
    PluginMetadata,
    PluginRegistrar,
    TaoryxPlugin,
)
from .discovery import PluginCatalog, PluginEntryPoint, discover_plugins

__all__ = [
    "CONTRIBUTION_KINDS",
    "ContributionKind",
    "PLUGIN_API_VERSION",
    "PLUGIN_ENTRY_POINT_GROUP",
    "PluginCatalog",
    "PluginCollisionError",
    "PluginCompatibilityError",
    "PluginContribution",
    "PluginDefinition",
    "PluginDiagnostic",
    "PluginEntryPoint",
    "PluginError",
    "PluginLoadError",
    "PluginMetadata",
    "PluginRegistrar",
    "TaoryxPlugin",
    "discover_plugins",
]
####
