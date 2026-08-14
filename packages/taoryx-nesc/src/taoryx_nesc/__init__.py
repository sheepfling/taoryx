"""NESC source-replay plug-in distribution for Taoryx."""

from importlib import import_module


def __getattr__(name: str) -> object:
    """Keep package-resource lookup free of plug-in registration imports."""

    if name == "PLUGIN":
        return getattr(import_module("taoryx_nesc.plugin"), name)
    raise AttributeError(name)
    ####


__all__ = ["PLUGIN"]
