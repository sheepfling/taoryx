"""Source-checkout fallback discovery for sibling plug-in distributions."""

from __future__ import annotations

from importlib import import_module

from taoryx.plugins.contracts import TaoryxPlugin

_SOURCE_PLUGIN_MODULES: tuple[tuple[str, str], ...] = (
    ("taoryx.daveml", "taoryx_daveml.plugin"),
    ("taoryx.reachability", "taoryx_reachability.plugin"),
    ("taoryx.reference-models", "taoryx_reference_models.plugin"),
    ("taoryx.simple-aero", "taoryx_simple_aero.plugin"),
)


def builtin_plugins(*, excluded_ids: frozenset[str] = frozenset()) -> tuple[TaoryxPlugin, ...]:
    """Load sibling source packages not already represented by entry points.

    Installed wheels are discovered from standard distribution metadata. This
    fallback exists only so a monorepo checkout can exercise the same packages
    directly from their independent ``src`` roots.
    """

    plugins: list[TaoryxPlugin] = []
    for plugin_id, module_name in _SOURCE_PLUGIN_MODULES:
        if plugin_id in excluded_ids:
            continue
        try:
            module = import_module(module_name)
        except ModuleNotFoundError as error:
            if error.name == module_name.split(".", maxsplit=1)[0]:
                continue
            raise
        plugin = getattr(module, "PLUGIN", None)
        if plugin is None:
            raise RuntimeError(f"source plug-in module {module_name!r} does not expose PLUGIN")
        plugins.append(plugin)
    return tuple(plugins)
    ####


__all__ = ["builtin_plugins"]
####
