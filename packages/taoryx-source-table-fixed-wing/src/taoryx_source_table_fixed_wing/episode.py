"""Package-owned interactive binding for source-table fixed-wing routes."""

from __future__ import annotations

from taoryx.composition_episode import LanguageBackedCompositionEpisode
from taoryx.language_backed_racetrack import LanguageBackedRacetrackAssets
from taoryx.plugins import current_plugin_catalog
from taoryx.vehicle_composition import CompiledVehicleComposition

from .resources import model_resource_root


class SourceTableFixedWingCompositionEpisode(LanguageBackedCompositionEpisode):
    """Reuse the core kernel while pinning X8/B747 routes to this package data."""

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None) -> None:
        super().__init__(
            composition,
            seed=seed,
            assets=LanguageBackedRacetrackAssets.from_root(model_resource_root()),
            plugins=current_plugin_catalog(),
        )
        ####

    ####


__all__ = ["SourceTableFixedWingCompositionEpisode"]
