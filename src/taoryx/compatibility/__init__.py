"""Explicit compatibility adapters for pre-plug-in TAORYX entry points.

New host and vehicle-package code should depend on typed plug-in discovery and
an explicit :class:`taoryx.plugins.discovery.PluginCatalog`.  Modules in this
namespace preserve older aggregate behaviour for existing callers while that
surface is retired in a controlled way.
"""

from __future__ import annotations

__all__: list[str] = []
