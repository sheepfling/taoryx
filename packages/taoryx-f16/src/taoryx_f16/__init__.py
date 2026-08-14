"""F-16 S.119 plug-in distribution namespace."""

from __future__ import annotations

from importlib import import_module
from typing import Any


def __getattr__(name: str) -> Any:
    """Load plug-in registration lazily so resource discovery stays cheap."""

    if name == "PLUGIN":
        return getattr(import_module("taoryx_f16.plugin"), name)
    raise AttributeError(name)
    ####


__all__ = ["PLUGIN"]
