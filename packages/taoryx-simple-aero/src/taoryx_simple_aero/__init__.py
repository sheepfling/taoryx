"""Simple Aero analytical providers for Taoryx."""

from importlib import import_module


def __getattr__(name: str) -> object:
    """Keep the provider import lazy during plug-in and resource discovery."""

    if name == "ReferencePointMassProvider":
        return getattr(import_module("taoryx_simple_aero.provider"), name)
    raise AttributeError(name)
    ####


__all__ = ["ReferencePointMassProvider"]
####
