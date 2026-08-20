"""Validate the direct plug-in developer route without importing model code."""

from __future__ import annotations

import argparse
import ast
import tomllib
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from scripts.bootstrap import PROFILE_PROJECTS

ROOT = Path(__file__).resolve().parents[1]
PLUGIN_ENTRY_POINT_GROUP = "taoryx.plugins"
COMPATIBILITY_DISTRIBUTION = "taoryx-reference-models"
OPTIONAL_SOURCE_BOUND_DISTRIBUTION = "taoryx-cadac"
LEGACY_PROVIDER_MODULE = "taoryx.trajectory.registry_mission_composition"
LEGACY_TRAJECTORY_FACADE_MODULE = "taoryx.trajectory"
SELECTED_CATALOG_RESOURCE_RESOLVER = ROOT / "src" / "taoryx" / "vehicle_catalog_resources.py"
COMPATIBILITY_CATALOG_RESOURCE_RESOLVER = ROOT / "src" / "taoryx" / "compatibility" / "vehicle_catalog_resources.py"


@dataclass(frozen=True, slots=True)
class PluginProject:
    """One package project projected from its declared build metadata."""

    path: Path
    distribution: str
    entry_points: tuple[tuple[str, str], ...]
    dependencies: tuple[str, ...]

    @property
    def source_root(self) -> Path:
        """Return the package's declared source root."""

        return self.path.parent / "src"
        ####

    ####


def _load_mapping(path: Path) -> dict[str, Any]:
    """Read one project file with a useful structural error."""

    try:
        document = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"could not read plug-in project {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"plug-in project {path} is not a TOML mapping")
    return document
    ####


def _projects() -> tuple[PluginProject, ...]:
    """Return every package that declares the Taoryx entry-point group."""

    projects: list[PluginProject] = []
    for path in sorted((ROOT / "packages").glob("*/pyproject.toml")):
        document = _load_mapping(path)
        metadata = document.get("project")
        if not isinstance(metadata, dict):
            raise ValueError(f"plug-in project {path} has no [project] table")
        distribution = metadata.get("name")
        if not isinstance(distribution, str) or not distribution:
            raise ValueError(f"plug-in project {path} has no project name")
        entry_points = metadata.get("entry-points")
        group = entry_points.get(PLUGIN_ENTRY_POINT_GROUP) if isinstance(entry_points, dict) else None
        if group is None:
            continue
        if not isinstance(group, dict) or not group:
            raise ValueError(f"plug-in project {path} has an invalid {PLUGIN_ENTRY_POINT_GROUP!r} declaration")
        declared = tuple(sorted((name, value) for name, value in group.items() if isinstance(name, str) and isinstance(value, str)))
        if len(declared) != len(group) or not declared:
            raise ValueError(f"plug-in project {path} has a non-string entry-point declaration")
        dependencies = metadata.get("dependencies", ())
        if not isinstance(dependencies, list) or not all(isinstance(item, str) for item in dependencies):
            raise ValueError(f"plug-in project {path} has invalid project dependencies")
        projects.append(
            PluginProject(
                path=path,
                distribution=distribution,
                entry_points=declared,
                dependencies=tuple(dependencies),
            )
        )
    if not projects:
        raise ValueError("no Taoryx plug-in projects declare an entry point")
    return tuple(projects)
    ####


def _normalized_distribution(name: str) -> str:
    """Normalize a distribution name for exact package-dependency comparison."""

    return name.casefold().replace("_", "-").replace(".", "-")
    ####


def _dependency_distribution(specification: str) -> str:
    """Extract the distribution token before extras, markers, or version bounds."""

    for delimiter in ("[", "<", ">", "=", "!", "~", ";", " "):
        specification = specification.partition(delimiter)[0]
    return _normalized_distribution(specification)
    ####


def _transitive_plugin_dependencies(
    project: PluginProject,
    projects: dict[str, PluginProject],
) -> set[str]:
    """Return the local plug-in dependency closure needed by an offline profile."""

    dependencies: set[str] = set()
    pending = [_dependency_distribution(item) for item in project.dependencies]
    while pending:
        distribution = pending.pop()
        if distribution in dependencies or distribution not in projects:
            continue
        dependencies.add(distribution)
        pending.extend(_dependency_distribution(item) for item in projects[distribution].dependencies)
    return dependencies
    ####


def _imports_from_module(source: Path, module: str) -> tuple[str, ...]:
    """Return direct imports of one module or package from a source file."""

    try:
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    except (OSError, UnicodeError, SyntaxError) as error:
        raise ValueError(f"could not inspect plug-in source {source}: {error}") from error
    imports: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names if alias.name == module or alias.name.startswith(f"{module}."))
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            if node.module == module or node.module.startswith(f"{module}."):
                imports.append(node.module)
    return tuple(imports)
    ####


def _compatibility_imports(source: Path) -> tuple[str, ...]:
    """Return direct imports of the compatibility aggregate."""

    return _imports_from_module(source, "taoryx_reference_models")
    ####


def _trajectory_facade_imports(source: Path) -> tuple[str, ...]:
    """Return imports of the broad legacy convenience facade only.

    Direct packages may import a narrow core contract such as
    ``taoryx.trajectory.catalog_mission_composition``.  Importing the package
    root, however, can resolve one of its historical all-model lazy exports
    and therefore bypasses package ownership. Keep that root facade solely for
    compatibility callers.
    """

    try:
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    except (OSError, UnicodeError, SyntaxError) as error:
        raise ValueError(f"could not inspect plug-in source {source}: {error}") from error
    imports: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names if alias.name == LEGACY_TRAJECTORY_FACADE_MODULE)
        elif isinstance(node, ast.ImportFrom) and node.module == LEGACY_TRAJECTORY_FACADE_MODULE:
            imports.append(node.module)
    return tuple(imports)
    ####


def _validate_compatibility_resource_quarantine() -> None:
    """Keep the static aggregate ordering out of selected-catalog core code."""

    try:
        selected_source = SELECTED_CATALOG_RESOURCE_RESOLVER.read_text(encoding="utf-8")
        compatibility_source = COMPATIBILITY_CATALOG_RESOURCE_RESOLVER.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ValueError(f"could not inspect vehicle catalog resolver quarantine: {error}") from error
    if "_LEGACY_VEHICLE_CATALOG_PACKAGES" in selected_source:
        raise ValueError("selected vehicle catalog resolver retains the historical aggregate package list")
    if "_LEGACY_VEHICLE_CATALOG_PACKAGES" not in compatibility_source:
        raise ValueError("compatibility vehicle catalog resolver has no explicit historical aggregate ordering")
    ####


def _project_by_plugin_id(projects: tuple[PluginProject, ...]) -> dict[str, PluginProject]:
    """Index package projects by their declared plug-in identity."""

    indexed: dict[str, PluginProject] = {}
    for project in projects:
        for plugin_id, _target in project.entry_points:
            existing = indexed.setdefault(plugin_id, project)
            if existing != project:
                raise ValueError(f"plug-in ID {plugin_id!r} is declared by multiple package projects")
    return indexed
    ####


def _validate_install_profiles(
    *,
    by_distribution: dict[str, PluginProject],
    expected_direct: set[str],
) -> None:
    """Validate aggregate profile membership for the catalog-wide route."""

    profile_distributions = {
        profile: tuple(_normalized_distribution(Path(path).name) for path in paths)
        for profile, paths in PROFILE_PROJECTS.items()
    }
    models = set(profile_distributions["models"])
    developer = set(profile_distributions["developer"])
    compatibility = set(profile_distributions["compatibility"])
    full = set(profile_distributions["full"])
    compatibility_distribution = _normalized_distribution(COMPATIBILITY_DISTRIBUTION)
    if models != expected_direct - {_normalized_distribution("taoryx-reachability")}:
        raise ValueError("models profile does not contain exactly the direct non-overlay plug-in packages")
    if developer != expected_direct:
        raise ValueError("developer profile does not contain exactly the direct plug-in packages")
    compatibility_project = by_distribution[compatibility_distribution]
    expected_compatibility = _transitive_plugin_dependencies(compatibility_project, by_distribution) | {
        compatibility_distribution
    }
    if compatibility != expected_compatibility:
        raise ValueError("compatibility profile must contain exactly the aggregate's declared plug-in dependencies")
    if full != developer | {compatibility_distribution}:
        raise ValueError("full profile must combine direct developer and compatibility packages")
    ####


def _validate_project_direct_route(
    project: PluginProject,
    *,
    expected_direct: set[str],
    compatibility_distribution: str,
) -> None:
    """Validate one project's direct ownership boundary."""

    distribution = _normalized_distribution(project.distribution)
    if not (project.path.parent / "README.md").is_file():
        raise ValueError(f"plug-in project {project.distribution!r} has no package README")
    if len(project.entry_points) != 1:
        raise ValueError(f"plug-in project {project.distribution!r} must declare exactly one {PLUGIN_ENTRY_POINT_GROUP!r} entry point")
    if not project.source_root.is_dir():
        raise ValueError(f"plug-in project {project.distribution!r} has no source root")
    if distribution not in expected_direct:
        return
    dependency_names = {_dependency_distribution(item) for item in project.dependencies}
    if compatibility_distribution in dependency_names:
        raise ValueError(f"direct plug-in {project.distribution!r} depends on the compatibility aggregate")
    compatibility_imports = {
        source.relative_to(project.path.parent).as_posix(): _compatibility_imports(source)
        for source in project.source_root.rglob("*.py")
    }
    leaks = {source: imports for source, imports in compatibility_imports.items() if imports}
    if leaks:
        raise ValueError(f"direct plug-in {project.distribution!r} imports the compatibility aggregate: {leaks!r}")
    legacy_provider_imports = {
        source.relative_to(project.path.parent).as_posix(): _imports_from_module(source, LEGACY_PROVIDER_MODULE)
        for source in project.source_root.rglob("*.py")
    }
    legacy_provider_leaks = {source: imports for source, imports in legacy_provider_imports.items() if imports}
    if legacy_provider_leaks:
        raise ValueError(
            f"direct plug-in {project.distribution!r} imports the legacy aggregate-provider module: "
            f"{legacy_provider_leaks!r}"
        )
    trajectory_facade_imports = {
        source.relative_to(project.path.parent).as_posix(): _trajectory_facade_imports(source)
        for source in project.source_root.rglob("*.py")
    }
    trajectory_facade_leaks = {source: imports for source, imports in trajectory_facade_imports.items() if imports}
    if trajectory_facade_leaks:
        raise ValueError(
            f"direct plug-in {project.distribution!r} imports the legacy taoryx.trajectory facade: "
            f"{trajectory_facade_leaks!r}; import a narrow core contract instead"
        )
    ####


def validate(*, plugin_ids: Iterable[str] | None = None) -> None:
    """Validate direct developer ownership and explicit compatibility isolation.

    Omitting ``plugin_ids`` retains the complete release gate, including the
    installation-profile closure.  Supplying one or more IDs intentionally
    limits source inspection to those package boundaries, which makes it the
    fast route used while a single plug-in is being developed.
    """

    _validate_compatibility_resource_quarantine()
    projects = _projects()
    by_distribution = {_normalized_distribution(project.distribution): project for project in projects}
    if len(by_distribution) != len(projects):
        raise ValueError("plug-in projects have duplicate distribution names")
    expected_direct = {
        distribution
        for distribution in by_distribution
        if distribution not in {
            _normalized_distribution(COMPATIBILITY_DISTRIBUTION),
            _normalized_distribution(OPTIONAL_SOURCE_BOUND_DISTRIBUTION),
        }
    }
    compatibility_distribution = _normalized_distribution(COMPATIBILITY_DISTRIBUTION)
    requested = None if plugin_ids is None else frozenset(item.strip() for item in plugin_ids if item.strip())
    if requested == frozenset():
        raise ValueError("plugin_ids must contain at least one non-empty plug-in ID")
    if requested is None:
        _validate_install_profiles(by_distribution=by_distribution, expected_direct=expected_direct)
        projects_to_validate = projects
    else:
        by_plugin_id = _project_by_plugin_id(projects)
        unknown = tuple(sorted(requested - set(by_plugin_id)))
        if unknown:
            raise ValueError(f"unknown Taoryx plug-in ID(s): {', '.join(unknown)}")
        projects_to_validate = tuple(by_plugin_id[plugin_id] for plugin_id in sorted(requested))

    for project in projects_to_validate:
        _validate_project_direct_route(
            project,
            expected_direct=expected_direct,
            compatibility_distribution=compatibility_distribution,
        )
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Run the developer-route ownership check from the command line."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plugin",
        action="append",
        metavar="PLUGIN_ID",
        help="limit source-boundary validation to one named plug-in; repeat to select several",
    )
    args = parser.parse_args(argv)
    validate(plugin_ids=args.plugin)
    scope = "all direct plug-ins" if not args.plugin else ", ".join(sorted(args.plugin))
    print(f"validated direct plug-in developer route: {scope}")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
