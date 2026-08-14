"""Packaged Hummingbird multirotor plug-in assets and registration."""

from importlib import import_module


def __getattr__(name: str) -> object:
    """Keep package-resource lookup free of plug-in registration imports."""

    if name == "PLUGIN":
        return getattr(import_module("taoryx_hummingbird.plugin"), name)
    raise AttributeError(name)
    ####

__all__ = ["PLUGIN"]
