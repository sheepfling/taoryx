"""Optional reachability workbench for Taoryx."""

from importlib import import_module


def __getattr__(name: str) -> object:
    """Keep workbench implementation imports lazy during discovery."""

    if name == "TaoryxReachabilityProvider":
        return getattr(import_module("taoryx_reachability.provider"), name)
    raise AttributeError(name)
    ####


__all__ = ["TaoryxReachabilityProvider"]
####
