"""Declared Composition endpoints for local native-coordinate LQI screens."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .local_native_coordinate_lqi import LocalNativeCoordinateLqiScreenConfig
    from .plugins import PluginCatalog
    from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenDefinition:
    """One exact model-owned local LQI endpoint using named controls."""

    id: str
    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_id: str
    capability_adapter_id: str
    config_factory: Callable[[], LocalNativeCoordinateLqiScreenConfig]
    advertisement: Mapping[str, object]
    claim_boundary: str

    def __post_init__(self) -> None:
        """Reject incomplete static metadata without constructing a candidate."""

        if not self.id.strip():
            raise ValueError("local native-coordinate LQI definitions require an ID")
        required = {
            "schema",
            "id",
            "fidelity",
            "operations",
            "control_realization",
            "controller",
            "native_controls",
            "claim_boundary",
        }
        missing = sorted(required - set(self.advertisement))
        if missing:
            raise ValueError(f"local native-coordinate LQI advertisement is missing: {', '.join(missing)}")
        if self.advertisement["id"] != self.id:
            raise ValueError("local native-coordinate LQI advertisement ID must match its definition")
        if self.advertisement["fidelity"] != self.fidelity:
            raise ValueError("local native-coordinate LQI advertisement fidelity must match its definition")
        if self.advertisement["control_realization"] != "native_named_coordinates":
            raise ValueError("local native-coordinate LQI advertisement must retain named-coordinate realization")
        if self.advertisement["operations"] != ["batch"]:
            raise ValueError("local native-coordinate LQI advertisements must be batch-only")
        if not self.claim_boundary.strip():
            raise ValueError("local native-coordinate LQI definitions require a claim boundary")
        ####

    ####

    def public_advertisement(self) -> dict[str, object]:
        """Return portable static screen metadata without tuning or executing."""

        return deepcopy(dict(self.advertisement))
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether a compiled request selects this exact endpoint."""

        return (
            composition.family_id == self.family_id
            and composition.mission == self.mission_id
            and composition.fidelity == self.fidelity
            and composition.initialization.id == self.initialization_id
        )
        ####

    ####


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenRegistry:
    """Fail-closed typed lookup for plug-in-owned named-coordinate LQI screens."""

    definitions: tuple[LocalNativeCoordinateLqiScreenDefinition, ...] = ()

    def __post_init__(self) -> None:
        """Reject duplicate public screen identities and exact runtime selections."""

        identifiers = tuple(item.id for item in self.definitions)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("local native-coordinate LQI registry has duplicate IDs")
        selections = tuple(
            (
                item.family_id,
                item.mission_id,
                item.fidelity,
                item.initialization_id,
            )
            for item in self.definitions
        )
        if len(selections) != len(set(selections)):
            raise ValueError("local native-coordinate LQI registry has duplicate endpoint selections")
        ####

    def resolve(
        self,
        composition: CompiledVehicleComposition,
    ) -> LocalNativeCoordinateLqiScreenDefinition | None:
        """Resolve one exact composition endpoint without a family fallback."""

        return next((item for item in self.definitions if item.supports(composition)), None)
        ####

    def advertisement(
        self,
        *,
        family_id: str | None,
        mission_id: str | None,
        fidelity: str,
    ) -> dict[str, object] | None:
        """Return one exact static screen record without constructing a plant."""

        if family_id is None or mission_id is None:
            return None
        definition = next(
            (item for item in self.definitions if item.family_id == family_id and item.mission_id == mission_id and item.fidelity == fidelity),
            None,
        )
        return definition.public_advertisement() if definition is not None else None
        ####

    ####


def _registry(*, plugins: PluginCatalog | None = None) -> LocalNativeCoordinateLqiScreenRegistry:
    """Resolve the selected plug-in catalog before inspecting local-screen ownership."""

    if plugins is None:
        from .plugins import current_plugin_catalog, discover_plugins

        plugins = current_plugin_catalog()
        if plugins is None:
            plugins = discover_plugins()
    return plugins.build_local_native_coordinate_lqi_screen_registry()
    ####


def local_native_coordinate_lqi_screen_definitions(
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[LocalNativeCoordinateLqiScreenDefinition, ...]:
    """Return installed model-owned native-coordinate LQI endpoints."""

    return _registry(plugins=plugins).definitions
    ####


def resolve_local_native_coordinate_lqi_screen_definition(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> LocalNativeCoordinateLqiScreenDefinition | None:
    """Resolve a screen without family or fidelity fallback."""

    return _registry(plugins=plugins).resolve(composition)
    ####


def resolve_local_native_coordinate_lqi_screen_advertisement(
    *,
    family_id: str | None,
    mission_id: str | None,
    fidelity: str,
    plugins: PluginCatalog | None = None,
) -> dict[str, object] | None:
    """Return selected static metadata without compiling a composition or tuning."""

    return _registry(plugins=plugins).advertisement(
        family_id=family_id,
        mission_id=mission_id,
        fidelity=fidelity,
    )
    ####


__all__ = [
    "LocalNativeCoordinateLqiScreenDefinition",
    "LocalNativeCoordinateLqiScreenRegistry",
    "local_native_coordinate_lqi_screen_definitions",
    "resolve_local_native_coordinate_lqi_screen_advertisement",
    "resolve_local_native_coordinate_lqi_screen_definition",
]
