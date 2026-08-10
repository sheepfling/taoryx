"""Taoryx entry point for the Simple Aero provider package."""

from __future__ import annotations

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar

from .provider import ReferencePointMassProvider


def _register(registrar: PluginRegistrar) -> None:
    """Publish lightweight provider instances without executing a case."""

    registrar.register_trajectory_provider(ReferencePointMassProvider())
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.simple-aero",
        package="taoryx-simple-aero",
        version="0.1.0a0",
        api_version="1",
        description="Analytical Simple Aero reference models and trajectory providers.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####

