"""Source-checkout entry-point declarations for sibling plug-in distributions."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from sys import path as python_path
from typing import cast

from taoryx.plugins.contracts import PLUGIN_ENTRY_POINT_GROUP, PluginMetadata, TaoryxPlugin


@dataclass(frozen=True, slots=True)
class SourcePluginEntryPoint:
    """One plug-in entry point declared by a sibling source distribution.

    The declaration is read from the same ``pyproject.toml`` that produces an
    installed wheel's standard Python entry point.  It is intentionally not a
    second hand-maintained plug-in inventory.
    """

    name: str
    value: str
    distribution: str
    version: str
    project: Path

    def load(self) -> object:
        """Load the declared target using standard ``module:attribute`` syntax."""

        module_name, separator, attribute_path = self.value.partition(":")
        if not separator or not module_name or not attribute_path:
            raise RuntimeError(
                f"source plug-in entry point {self.name!r} in {self.project} must use 'module:attribute' syntax"
            )
        target: object = import_module(module_name)
        for attribute in attribute_path.split("."):
            if not attribute:
                raise RuntimeError(
                    f"source plug-in entry point {self.name!r} in {self.project} has an invalid target {self.value!r}"
                )
            target = getattr(target, attribute)
        return target
        ####

    ####


def source_plugin_entry_points(
    *,
    root: Path | None = None,
    active_only: bool = False,
) -> tuple[SourcePluginEntryPoint, ...]:
    """Read checkout fallback declarations from sibling project metadata.

    An installed environment never needs this helper: its distribution metadata
    supplies the same declarations.  In a source checkout it keeps fallback
    discovery aligned with the published wheel contract without importing a
    plug-in merely to list it.
    """

    repository_root = root if root is not None else _source_repository_root()
    if repository_root is None:
        return ()
    declarations: list[SourcePluginEntryPoint] = []
    active_source_roots = _active_source_roots() if active_only else frozenset()
    for project in sorted((repository_root / "packages").glob("*/pyproject.toml")):
        if active_only and (project.parent / "src").resolve() not in active_source_roots:
            continue
        try:
            document = tomllib.loads(project.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, tomllib.TOMLDecodeError) as error:
            raise RuntimeError(f"could not read source plug-in project {project}: {error}") from error
        project_metadata = document.get("project")
        if not isinstance(project_metadata, dict):
            raise RuntimeError(f"source plug-in project {project} has no [project] table")
        entry_points = project_metadata.get("entry-points")
        if entry_points is None:
            continue
        if not isinstance(entry_points, dict):
            raise RuntimeError(f"source plug-in project {project} has an invalid [project.entry-points] table")
        group = entry_points.get(PLUGIN_ENTRY_POINT_GROUP)
        if group is None:
            continue
        if not isinstance(group, dict):
            raise RuntimeError(
                f"source plug-in project {project} has an invalid [project.entry-points.{PLUGIN_ENTRY_POINT_GROUP!r}] table"
            )
        distribution = project_metadata.get("name")
        version = project_metadata.get("version")
        if not isinstance(distribution, str) or not distribution.strip():
            raise RuntimeError(f"source plug-in project {project} has no nonempty project name")
        if not isinstance(version, str) or not version.strip():
            raise RuntimeError(f"source plug-in project {project} has no nonempty project version")
        for name, value in group.items():
            if not isinstance(name, str) or not name.strip() or not isinstance(value, str) or not value.strip():
                raise RuntimeError(
                    f"source plug-in project {project} has an invalid {PLUGIN_ENTRY_POINT_GROUP!r} declaration"
                )
            declarations.append(
                SourcePluginEntryPoint(
                    name=name.strip(),
                    value=value.strip(),
                    distribution=distribution.strip(),
                    version=version.strip(),
                    project=project,
                )
            )
    duplicate_ids = sorted(
        name
        for name in {item.name for item in declarations}
        if sum(item.name == name for item in declarations) > 1
    )
    if duplicate_ids:
        raise RuntimeError(
            "source plug-in entry-point declarations have duplicate names: " + ", ".join(repr(item) for item in duplicate_ids)
        )
    return tuple(sorted(declarations, key=lambda item: (item.name, item.value, item.distribution)))
    ####


def builtin_plugins(
    *,
    excluded_ids: frozenset[str] = frozenset(),
    selected_ids: frozenset[str] | None = None,
) -> tuple[TaoryxPlugin, ...]:
    """Load source entry points for legacy callers of the checkout fallback.

    Discovery itself consumes :func:`source_plugin_entry_points` directly so
    installed and checkout paths share one entry-point validation flow.  This
    small compatibility wrapper remains for callers that still expect concrete
    source plug-in objects.
    """

    plugins: list[TaoryxPlugin] = []
    for entry_point in source_plugin_entry_points(active_only=True):
        if entry_point.name in excluded_ids or (selected_ids is not None and entry_point.name not in selected_ids):
            continue
        try:
            plugin = entry_point.load()
        except ModuleNotFoundError as error:
            module_name = entry_point.value.partition(":")[0]
            if error.name == module_name.split(".", maxsplit=1)[0]:
                continue
            raise
        if not isinstance(getattr(plugin, "metadata", None), PluginMetadata) or not callable(getattr(plugin, "register", None)):
            raise RuntimeError(f"source plug-in entry point {entry_point.value!r} does not expose a Taoryx plug-in")
        plugins.append(cast(TaoryxPlugin, plugin))
    return tuple(plugins)
    ####


def _source_repository_root() -> Path | None:
    """Return the repository root only when core is running from a checkout."""

    candidate = Path(__file__).resolve().parents[3]
    return candidate if (candidate / "packages").is_dir() else None
    ####


def _active_source_roots() -> frozenset[Path]:
    """Return explicit ``src`` roots that a partial checkout activated."""

    roots: set[Path] = set()
    for entry in python_path:
        if not entry:
            continue
        try:
            roots.add(Path(entry).resolve())
        except OSError:
            continue
    return frozenset(roots)
    ####


__all__ = ["SourcePluginEntryPoint", "builtin_plugins", "source_plugin_entry_points"]
