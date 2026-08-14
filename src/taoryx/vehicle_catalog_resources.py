"""Locate one installed vehicle-catalog fragment without global merging.

The source checkout keeps its canonical ``verification/`` directory. Wheels
instead obtain coherent non-overlapping family fragments plus explicitly
dependent additive overlays from packages such as Hummingbird, source-table
fixed wing, or reachability.
This small resolver is deliberately about data ownership only; executable
factories still come exclusively from the normal plug-in discovery registry.
Unscoped calls remain source-compatible through an explicitly quarantined
compatibility adapter.  Direct host and vehicle-package paths must instead use
the typed selected-catalog route in this module.
"""

from __future__ import annotations

from pathlib import Path, PurePosixPath
from typing import TYPE_CHECKING

from .plugins.contracts import VehicleCatalogFragment, VehicleCatalogOverlayFragment
from .plugins.resources import packaged_resource

if TYPE_CHECKING:
    from .plugins.discovery import PluginCatalog

def _selected_catalog_fragments(
    plugins: PluginCatalog | None,
) -> tuple[VehicleCatalogFragment | VehicleCatalogOverlayFragment, ...] | None:
    """Return typed data owners for an explicit plug-in selection, if any.

    ``None`` deliberately means that the caller has not opted into selected
    catalog ownership.  The old unscoped resolver remains available for
    compatibility aggregate imports that occur before plug-in discovery.
    """

    if plugins is None:
        return None
    fragments: list[VehicleCatalogFragment | VehicleCatalogOverlayFragment] = []
    for contribution in plugins.records("vehicle_catalog_fragment"):
        fragment = contribution.value
        if not isinstance(fragment, VehicleCatalogFragment):
            raise TypeError(
                f"vehicle catalog fragment {contribution.id!r} from {contribution.plugin.id!r} has an invalid contract"
            )
        if contribution.id != fragment.id:
            raise ValueError(
                f"vehicle catalog fragment registration {contribution.id!r} disagrees with payload {fragment.id!r}"
            )
        fragments.append(fragment)
    for contribution in plugins.records("vehicle_catalog_overlay_fragment"):
        fragment = contribution.value
        if not isinstance(fragment, VehicleCatalogOverlayFragment):
            raise TypeError(
                f"vehicle catalog overlay fragment {contribution.id!r} from {contribution.plugin.id!r} has an invalid contract"
            )
        if contribution.id != fragment.id:
            raise ValueError(
                f"vehicle catalog overlay registration {contribution.id!r} disagrees with payload {fragment.id!r}"
            )
        fragments.append(fragment)
    return tuple(fragments)
    ####


def _active_selected_catalog_fragments(
    fragments: tuple[VehicleCatalogFragment | VehicleCatalogOverlayFragment, ...],
) -> tuple[VehicleCatalogFragment | VehicleCatalogOverlayFragment, ...]:
    """Return base fragments plus overlays whose declared bases are selected.

    A host may select an optional workbench for a different vehicle family.
    Its unrelated overlays must remain dormant rather than preventing that
    focused host from resolving its own base data. A direct lookup for an
    inactive overlay's resource still fails closed below.
    """

    base_fragments = {fragment.id: fragment for fragment in fragments if isinstance(fragment, VehicleCatalogFragment)}
    active: list[VehicleCatalogFragment | VehicleCatalogOverlayFragment] = list(base_fragments.values())
    for overlay in fragments:
        if not isinstance(overlay, VehicleCatalogOverlayFragment):
            continue
        missing = tuple(identifier for identifier in overlay.extends_fragment_ids if identifier not in base_fragments)
        if missing:
            continue
        extended_family_ids = {
            family_id
            for fragment_id in overlay.extends_fragment_ids
            for family_id in base_fragments[fragment_id].family_ids
        }
        unknown_families = tuple(family_id for family_id in overlay.family_ids if family_id not in extended_family_ids)
        if unknown_families:
            raise ValueError(
                f"selected vehicle catalog overlay {overlay.id!r} targets family IDs {unknown_families!r} "
                "that are not declared by its selected base fragment(s)"
            )
        active.append(overlay)
    return tuple(active)
    ####


def _inactive_overlay_requirements(
    fragments: tuple[VehicleCatalogFragment | VehicleCatalogOverlayFragment, ...],
) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Describe selected overlays whose required vehicle base is absent."""

    base_ids = {fragment.id for fragment in fragments if isinstance(fragment, VehicleCatalogFragment)}
    return tuple(
        (overlay.id, tuple(identifier for identifier in overlay.extends_fragment_ids if identifier not in base_ids))
        for overlay in fragments
        if isinstance(overlay, VehicleCatalogOverlayFragment)
        and any(identifier not in base_ids for identifier in overlay.extends_fragment_ids)
    )
    ####


def _ambient_catalog_with_vehicle_fragments() -> PluginCatalog | None:
    """Return the active host selection when it declares vehicle data owners.

    Deferred providers already carry their selected catalog through nested
    core calls. Honor that scope here so a late execution/preflight helper
    cannot widen back to aggregate data. A nonphysical active catalog has no
    vehicle fragments and therefore preserves the legacy resolver behavior.
    """

    from .plugins.discovery import current_plugin_catalog

    catalog = current_plugin_catalog()
    if catalog is None or not _selected_catalog_fragments(catalog):
        return None
    return catalog
    ####


def _relative_resource_path(relative_path: str) -> str:
    """Validate one repository-relative package-data lookup path."""

    relative = PurePosixPath(relative_path)
    if not relative_path.strip() or relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"vehicle catalog resource must be a nonempty relative path: {relative_path!r}")
    return relative.as_posix()
    ####


def _selected_package_resources(relative_path: str, plugins: PluginCatalog | None) -> tuple[Path, ...] | None:
    """Resolve only resources declared by an explicit plug-in catalog."""

    fragments = _selected_catalog_fragments(plugins)
    if fragments is None:
        return None
    fragments = _active_selected_catalog_fragments(fragments)
    relative = _relative_resource_path(relative_path)
    resources: list[Path] = []
    seen_resources: set[Path] = set()
    for fragment in fragments:
        root = fragment.resource_root.strip().strip("/")
        resource = packaged_resource(
            package=fragment.resource_package,
            resource=f"{root}/{relative}",
        )
        if resource is not None and resource not in seen_resources:
            resources.append(resource)
            seen_resources.add(resource)
    return tuple(resources)
    ####


def _missing_selected_resource(relative_path: str, plugins: PluginCatalog) -> FileNotFoundError:
    """Return a fail-closed diagnostic for a selected catalog data gap."""

    fragments = _selected_catalog_fragments(plugins) or ()
    owners = tuple(fragment.id for fragment in fragments)
    inactive_overlays = _inactive_overlay_requirements(fragments)
    overlay_detail = (
        f"; selected overlay requirements are inactive: {inactive_overlays!r}; select the named base fragment(s)"
        if inactive_overlays
        else ""
    )
    return FileNotFoundError(
        f"selected vehicle catalog plug-ins {owners!r} do not own resource {relative_path!r}; "
        "install/select its owning vehicle plug-in rather than falling back to an aggregate catalog"
        f"{overlay_detail}"
    )
    ####


def vehicle_catalog_resource(relative_path: str, *, plugins: PluginCatalog | None = None) -> Path:
    """Return one resource from a selected plug-in scope or compatibility aggregate.

    The unscoped branch is retained only for legacy callers.  New host code
    should always pass the selected ``PluginCatalog`` so missing package data
    fails closed instead of widening to the aggregate catalogue.
    """

    selected_plugins = plugins if plugins is not None else _ambient_catalog_with_vehicle_fragments()
    selected = _selected_package_resources(relative_path, selected_plugins)
    if selected is not None:
        if not selected:
            assert selected_plugins is not None
            raise _missing_selected_resource(relative_path, selected_plugins)
        return selected[0]

    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    return legacy_vehicle_catalog_resource(relative_path)
    ####


def vehicle_catalog_resources(
    relative_path: str,
    *,
    plugins: PluginCatalog | None = None,
    required: bool = True,
) -> tuple[Path, ...]:
    """Return every fragment for one selected plug-in scope or legacy aggregate.

    Passing ``plugins`` is an ownership boundary: only typed
    ``vehicle_catalog_fragment`` and explicitly dependent
    ``vehicle_catalog_overlay_fragment`` contributions from that catalog are eligible,
    even in a complete source checkout. Without a selection, a complete
    checkout still resolves to its canonical aggregate file and installed
    legacy callers retain deterministic compatibility-then-family lookup.
    ``required=False`` is reserved for a catalog whose empty selected scope is
    itself a meaningful declaration, such as the absence of any registered
    batch/episode parity replay witness.

    Without a selected scope this delegates to the quarantined compatibility
    aggregate.  It is not a supported route for new plug-in code.
    """

    selected_plugins = plugins if plugins is not None else _ambient_catalog_with_vehicle_fragments()
    selected = _selected_package_resources(relative_path, selected_plugins)
    if selected is not None:
        if not selected:
            if not required:
                return ()
            assert selected_plugins is not None
            raise _missing_selected_resource(relative_path, selected_plugins)
        return selected

    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resources

    return legacy_vehicle_catalog_resources(relative_path)
    ####


def vehicle_catalog_root(*, plugins: PluginCatalog | None = None) -> Path:
    """Return the root that owns the selected vehicle-model fragment."""

    return vehicle_catalog_resource("verification/vehicle_models.yaml", plugins=plugins).parents[1]
    ####


__all__ = ["vehicle_catalog_resource", "vehicle_catalog_resources", "vehicle_catalog_root"]
