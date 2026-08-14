"""Historical aggregate lookup for vehicle-catalog package data.

This module is deliberately separate from the selected-catalog resolver.  It
exists only for callers that still use the old unscoped vehicle catalogue or
the aggregate Mission Composition provider.  New code must pass its selected
``PluginCatalog`` to :mod:`taoryx.vehicle_catalog_resources` instead of using
these functions.
"""

from __future__ import annotations

from importlib.util import find_spec
from pathlib import Path

from taoryx.plugins.resources import packaged_resource

ROOT = Path(__file__).resolve().parents[3]

# This is an intentionally compatibility-only ordering.  Direct plug-ins use
# their typed ``vehicle_catalog_fragment`` contributions and never consult it.
_LEGACY_VEHICLE_CATALOG_PACKAGES: tuple[str, ...] = (
    "taoryx_reference_models",
    "taoryx_source_table_fixed_wing",
    "taoryx_hl20",
    "taoryx_a320",
    "taoryx_f16",
    "taoryx_hummingbird",
    "taoryx_nesc",
    "taoryx_passive_bodies",
    "taoryx_x15",
    "taoryx_reachability",
)


def _available_catalog_packages() -> tuple[str, ...]:
    """Return legacy aggregate owners visible without importing them."""

    return tuple(package for package in _LEGACY_VEHICLE_CATALOG_PACKAGES if find_spec(package) is not None)
    ####


def _use_canonical_checkout(canonical: Path) -> bool:
    """Use root data only for core-only or complete source-checkout contexts."""

    available = _available_catalog_packages()
    return canonical.is_file() and (not available or len(available) == len(_LEGACY_VEHICLE_CATALOG_PACKAGES))
    ####


def _package_resources(relative_path: str) -> tuple[Path, ...]:
    """Return installed compatibility resources in their historical order."""

    return tuple(
        resource
        for package in _LEGACY_VEHICLE_CATALOG_PACKAGES
        if (resource := packaged_resource(package=package, resource=f"data/{relative_path}")) is not None
    )
    ####


def legacy_vehicle_catalog_resource(relative_path: str) -> Path:
    """Resolve one historical aggregate resource without selected ownership."""

    canonical = ROOT / relative_path
    if _use_canonical_checkout(canonical):
        return canonical
    return next(iter(_package_resources(relative_path)), canonical)
    ####


def legacy_vehicle_catalog_resources(relative_path: str) -> tuple[Path, ...]:
    """Resolve every historical aggregate resource without selected ownership."""

    canonical = ROOT / relative_path
    if _use_canonical_checkout(canonical):
        return (canonical,)
    resources = _package_resources(relative_path)
    return resources or (canonical,)
    ####


__all__ = ["legacy_vehicle_catalog_resource", "legacy_vehicle_catalog_resources"]
