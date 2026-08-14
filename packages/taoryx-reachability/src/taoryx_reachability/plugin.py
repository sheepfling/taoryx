"""Plug-in registration for the optional Taoryx reachability workbench."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogOverlayFragment

if TYPE_CHECKING:
    from taoryx.vehicle_composition import CompiledVehicleComposition


_X15_CAPABILITY_ID = "taoryx.x15_staged_reachability.capability.v1"
_HL20_GLIDE_ENERGY_CAPABILITY_ID = "taoryx.hl20_glide_energy.capability.v1"
_HL20_SOURCE_RELEASE_CAPABILITY_ID = "taoryx.hl20_source_booster_release_replay.capability.v1"
_HL20_SOURCE_RELEASE_PREFLIGHT_ID = "taoryx.hl20_source_booster_release_replay.v1"
_HL20_GLIDE_ENERGY_PREFLIGHT_ID = "taoryx.hl20_glide_energy_intent.v1"
_X15_CATALOG_OVERLAY = VehicleCatalogOverlayFragment(
    id="taoryx.reachability.x15-staged-catalog-overlay",
    resource_package="taoryx_reachability",
    extends_fragment_ids=("taoryx.x15.vehicle-catalog",),
    resource_root="data/x15_overlay",
    family_ids=("x15",),
)
_HL20_CATALOG_OVERLAY = VehicleCatalogOverlayFragment(
    id="taoryx.reachability.hl20-catalog-overlay",
    resource_package="taoryx_reachability",
    extends_fragment_ids=("taoryx.hl20.vehicle-catalog",),
    resource_root="data/hl20_overlay",
    family_ids=("hl20_mod_k",),
)


class _LazyMissionCapabilityAdapter:
    """Retain a reachability translator identity until a composition selects it."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> object:
        """Build and validate the exact reachability adapter on first use."""

        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy reachability capability adapter resolved mismatched identity {self.id!r}")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Delegate exact mission ownership only after a composition is inspected."""

        return bool(getattr(self._resolve(), "supports")(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Delegate the selected reachability estimate to its owning implementation."""

        return getattr(self._resolve(), "estimate")(composition)
        ####

    ####


def _x15_capability_adapter() -> object:
    """Load the X-15 overlay planner only when its mission is selected."""

    from .capabilities import X15StagedReachabilityCapabilityAdapter

    return X15StagedReachabilityCapabilityAdapter()
    ####


def _hl20_glide_energy_capability_adapter() -> object:
    """Load the HL-20 glide-energy planner only when its mission is selected."""

    from .capabilities import HL20GlideEnergyCapabilityAdapter

    return HL20GlideEnergyCapabilityAdapter()
    ####


def _hl20_source_release_capability_adapter() -> object:
    """Load the HL-20 source-release planner only when its mission is selected."""

    from .capabilities import HL20SourceBoosterReleaseReplayCapabilityAdapter

    return HL20SourceBoosterReleaseReplayCapabilityAdapter()
    ####


def _preflight_hl20_source_release(composition: CompiledVehicleComposition) -> object:
    """Load the source-release preflight implementation only when dispatched."""

    from .preflight import _preflight_hl20_source_booster_release_replay

    return _preflight_hl20_source_booster_release_replay(composition)
    ####


def _preflight_hl20_glide_energy(composition: CompiledVehicleComposition) -> object:
    """Load the glide-energy preflight implementation only when dispatched."""

    from .preflight import _preflight_hl20_glide_energy_intent

    return _preflight_hl20_glide_energy_intent(composition)
    ####


def _preflight_x15_staged(composition: CompiledVehicleComposition) -> object:
    """Load the staged X-15 preflight implementation only when dispatched."""

    from .preflight import _preflight_x15_staged_reachability

    return _preflight_x15_staged_reachability(composition)
    ####


def _execute_hl20_source_release(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    if max_steps is not None:
        raise ValueError("--max-steps is not available for the HL-20 source-release reachability factory")
    from taoryx.hl20_source_release_composition_execution import execute_hl20_source_booster_release_composition

    return execute_hl20_source_booster_release_composition(composition, output_dir)
    ####


def _execute_x15_staged(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    if max_steps is not None:
        raise ValueError("--max-steps is not available for the X-15 staged reachability factory")
    from taoryx.x15_staged_composition_execution import execute_x15_staged_reachability_composition

    return execute_x15_staged_reachability_composition(composition, output_dir)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the workbench and its composed mission overlays."""

    registrar.register_vehicle_catalog_overlay_fragment(_X15_CATALOG_OVERLAY)
    registrar.register_vehicle_catalog_overlay_fragment(_HL20_CATALOG_OVERLAY)
    from .provider import TaoryxReachabilityProvider

    registrar.register_reachability_provider(TaoryxReachabilityProvider())
    registrar.register_mission_capability_adapter(_LazyMissionCapabilityAdapter(_X15_CAPABILITY_ID, _x15_capability_adapter))
    registrar.register_mission_capability_adapter(_LazyMissionCapabilityAdapter(_HL20_GLIDE_ENERGY_CAPABILITY_ID, _hl20_glide_energy_capability_adapter))
    registrar.register_mission_capability_adapter(_LazyMissionCapabilityAdapter(_HL20_SOURCE_RELEASE_CAPABILITY_ID, _hl20_source_release_capability_adapter))
    registrar.register_semantic_preflight_handler_callback(
        _HL20_SOURCE_RELEASE_PREFLIGHT_ID,
        _preflight_hl20_source_release,
    )
    registrar.register_semantic_preflight_handler_callback(
        _HL20_GLIDE_ENERGY_PREFLIGHT_ID,
        _preflight_hl20_glide_energy,
    )
    registrar.register_semantic_preflight_handler_callback(
        _X15_CAPABILITY_ID,
        _preflight_x15_staged,
    )
    registrar.register_execution_factory(
        "hl20_source_booster_release_replay.v1",
        _execute_hl20_source_release,
    )
    registrar.register_execution_factory(
        "x15_staged_reachability.v1",
        _execute_x15_staged,
    )
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.reachability",
        package="taoryx-reachability",
        version="0.1.0a0",
        api_version="1",
        description="Optional reachability envelopes, overlays, plotting, and composed mission workflows.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
