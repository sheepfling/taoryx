"""Catalog-scoped Mission Composition host for direct vehicle plug-ins.

This is the direct-package counterpart to the historical aggregate provider.
Unlike :class:`RegistryMissionCompositionProvider`, it requires a resolved
catalog at construction time. A focused provider can therefore never widen
silently to the legacy all-vehicle catalogue when an owning package forgets to
thread its selected data scope through construction.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

from .registry_mission_composition import RegistryMissionCompositionProvider

if TYPE_CHECKING:
    from taoryx.plugins.discovery import PluginCatalog
    from taoryx.vehicle_composition_registry import ResolvedVehicleCompositionCatalog


class CatalogMissionCompositionProvider(RegistryMissionCompositionProvider):
    """A Mission Composition provider bound to one explicit vehicle catalog.

    The aggregate ``RegistryMissionCompositionProvider`` remains a compatibility
    adapter for existing callers. New vehicle packages must instantiate this
    type with their package-owned resolved catalog and should also preserve the
    active ``PluginCatalog`` when their runtime invokes generic core helpers.
    """

    def __init__(
        self,
        catalog: ResolvedVehicleCompositionCatalog | None = None,
        *,
        provider_id: str = "taoryx.registry.mission-composition",
        provider_name: str = "TAORYX Mission Composition Registry",
        provider_short_name: str = "TAORYX Registry",
        provider_summary: str = "Canonical vehicle families and configuration-ready workflows exposed through one portable catalog.",
        provider_version: str = "1",
        plugin_catalog: PluginCatalog | None = None,
        include_builtin_workflows: bool = True,
        allowed_mission_template_ids: Mapping[str, Sequence[str]] | None = None,
    ) -> None:
        """Create a direct provider without an unscoped registry fallback."""

        if catalog is None:
            raise TypeError(
                "CatalogMissionCompositionProvider requires an explicit resolved vehicle catalog; "
                "use RegistryMissionCompositionProvider only for the legacy compatibility aggregate"
            )
        super().__init__(
            catalog,
            provider_id=provider_id,
            provider_name=provider_name,
            provider_short_name=provider_short_name,
            provider_summary=provider_summary,
            provider_version=provider_version,
            plugin_catalog=plugin_catalog,
            include_builtin_workflows=include_builtin_workflows,
            allowed_mission_template_ids=allowed_mission_template_ids,
        )
        ####

    ####


__all__ = ["CatalogMissionCompositionProvider"]
