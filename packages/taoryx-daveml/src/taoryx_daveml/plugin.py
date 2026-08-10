"""Taoryx entry point for optional DAVE-ML format support."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from types import ModuleType

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar


@dataclass(frozen=True, slots=True)
class DAVEMLFormatHandler:
    """Lazy module facade that keeps discovery free of model parsing work."""

    format_id: str = "daveml"

    def import_module(self) -> ModuleType:
        """Return the public DAVE-ML import module on explicit use."""

        return import_module("taoryx.trajectory.daveml_import")
        ####

    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish DAVE-ML capability without opening a source document."""

    registrar.register_model_format("daveml", DAVEMLFormatHandler())
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.daveml",
        package="taoryx-daveml",
        version="0.1.0a0",
        api_version="1",
        description="DAVE-ML import, semantic, evaluation, collection, and replay support.",
    ),
    register_callback=_register,
)

__all__ = ["DAVEMLFormatHandler", "PLUGIN"]
####
