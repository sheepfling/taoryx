"""Development-only Taoryx model plug-in registration."""

from importlib import import_module


def __getattr__(name: str) -> object:
    """Avoid constructing providers merely to inspect package metadata."""

    if name == "PLUGIN":
        return getattr(import_module("taoryx_debug_models.plugin"), name)
    raise AttributeError(name)
    ####

__all__ = ["PLUGIN"]
