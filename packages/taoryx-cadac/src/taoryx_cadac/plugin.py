"""Taoryx entry point for the source-bound CADAC catalog."""

from __future__ import annotations

from functools import lru_cache
from typing import TYPE_CHECKING

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar

if TYPE_CHECKING:
    from taoryx.families.cadac.mission_composition_plugin import CadacMissionCompositionProvider


_PROVIDER_ID = "cadac"


@lru_cache(maxsize=1)
def _catalog_provider() -> CadacMissionCompositionProvider:
    """Build only installed CADAC runtimes for the public trajectory host."""

    from taoryx.families.cadac.mission_composition_plugin import CadacMissionCompositionProvider

    return CadacMissionCompositionProvider(publish_unbound_models=False)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the catalog identity without constructing CADAC schemas at discovery."""

    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _catalog_provider)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.cadac",
        package="taoryx-cadac",
        version="0.1.0a0",
        api_version="1",
        description="Source-bound CADAC vehicle catalog and canonical-table converter.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
