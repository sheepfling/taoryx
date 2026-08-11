"""Taoryx entry point for the source-bound CADAC catalog."""

from __future__ import annotations

from functools import lru_cache

from taoryx.families.cadac.mission_composition_plugin import CadacMissionCompositionProvider

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar


@lru_cache(maxsize=1)
def _catalog_provider() -> CadacMissionCompositionProvider:
    """Build metadata only; source cases are bound explicitly at execution time."""

    return CadacMissionCompositionProvider()
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the source-independent CADAC Mission Composition catalog."""

    registrar.register_mission_composition_provider(_catalog_provider())
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
