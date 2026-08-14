"""Declared plug-in-owned local direct-wrench controller screens.

The common host knows how to validate, execute, and advertise a local
direct-wrench screen.  It does not know which vehicle module builds the local
source plant.  A vehicle package contributes that exact factory and static
contract through this registry, so a focused plug-in need not import an
aggregate compatibility package merely to run its own controller witness.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .local_direct_wrench import LocalDirectWrenchScreenConfig
    from .plugins import PluginCatalog
    from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenDefinition:
    """One exact family-owned source-local direct-wrench endpoint."""

    id: str
    family_id: str
    mission_id: str
    initialization_id: str
    segment_id: str
    capability_adapter_id: str
    config_factory: Callable[[], LocalDirectWrenchScreenConfig]
    advertisement: Mapping[str, object]
    claim_boundary: str
    paired_lqi_mission_id: str | None = None
    paired_lqi_capability_adapter_id: str | None = None
    paired_lqi_campaign_id: str | None = None

    def __post_init__(self) -> None:
        """Reject incomplete static metadata without constructing a controller."""

        if not self.id.strip():
            raise ValueError("local direct-wrench definitions require an ID")
        required = {
            "schema",
            "id",
            "fidelity",
            "operations",
            "control_realization",
            "controller",
            "direct_wrench_controls",
            "claim_boundary",
        }
        missing = sorted(required - set(self.advertisement))
        if missing:
            raise ValueError(f"local direct-wrench advertisement is missing: {', '.join(missing)}")
        if self.advertisement["id"] != self.id:
            raise ValueError("local direct-wrench advertisement ID must match its definition")
        if self.advertisement["fidelity"] != "rigid_body_6dof_direct_wrench":
            raise ValueError("local direct-wrench advertisements require direct-wrench fidelity")
        if self.advertisement["control_realization"] != "direct_wrench":
            raise ValueError("local direct-wrench advertisement must retain direct-wrench realization")
        operations = self.advertisement["operations"]
        if not isinstance(operations, list) or "batch" not in operations:
            raise ValueError("local direct-wrench advertisements must include batch execution")
        if not self.claim_boundary.strip():
            raise ValueError("local direct-wrench definitions require a claim boundary")
        ####
    ####

    def public_advertisement(self) -> dict[str, object]:
        """Return portable static screen metadata without tuning or executing."""

        return deepcopy(dict(self.advertisement))
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether the composition exactly selects this local screen."""

        return (
            composition.family_id == self.family_id
            and composition.mission == self.mission_id
            and composition.initialization.id == self.initialization_id
            and composition.fidelity == "rigid_body_6dof_direct_wrench"
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenRegistry:
    """Fail-closed typed lookup for plug-in-owned direct-wrench screens."""

    definitions: tuple[LocalDirectWrenchScreenDefinition, ...] = ()

    def __post_init__(self) -> None:
        """Reject duplicate public identities and endpoint selections."""

        identifiers = tuple(item.id for item in self.definitions)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("local direct-wrench registry has duplicate IDs")
        selections = tuple(
            (
                item.family_id,
                item.mission_id,
                "rigid_body_6dof_direct_wrench",
                item.initialization_id,
            )
            for item in self.definitions
        )
        if len(selections) != len(set(selections)):
            raise ValueError("local direct-wrench registry has duplicate endpoint selections")
        ####

    def resolve(
        self,
        composition: CompiledVehicleComposition,
    ) -> LocalDirectWrenchScreenDefinition | None:
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

        if family_id is None or mission_id is None or fidelity != "rigid_body_6dof_direct_wrench":
            return None
        definition = next(
            (
                item
                for item in self.definitions
                if item.family_id == family_id and item.mission_id == mission_id
            ),
            None,
        )
        return definition.public_advertisement() if definition is not None else None
        ####

    ####


class LocalDirectWrenchScreenCapabilityAdapter:
    """Expose one bounded authority contract for a pinned local source screen.

    This generic adapter is host-owned because its lowering semantics and
    output contract are shared.  Its definition and configuration factory stay
    strictly vehicle-owned through ``LocalDirectWrenchScreenDefinition``.
    """

    def __init__(self, definition: LocalDirectWrenchScreenDefinition) -> None:
        self.definition = definition
        self.id = definition.capability_adapter_id
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return self.definition.supports(composition)
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Return the source-pinned local state, cadence, and wrench contract."""

        from .local_direct_wrench_mission_translation import compile_local_direct_wrench_screen_mission
        from .mission_capability import MissionCapabilityEstimate

        config = self.definition.config_factory()
        plan = compile_local_direct_wrench_screen_mission(
            composition,
            family_id=self.definition.family_id,
            mission_id=self.definition.mission_id,
            initialization_id=self.definition.initialization_id,
            segment_id=self.definition.segment_id,
            screen_config_id=config.id,
        )
        authority_span = {
            axis: float(config.limits.upper[axis]) - float(config.limits.lower[axis])
            for axis in config.limits.axes
        }
        rate_limited_axes = tuple(
            axis for axis in config.limits.axes if config.limits.rate_limit_per_s[axis] is not None
        )
        manifest = plan.manifest()
        capability: dict[str, Any] = {
            "control_realization": "direct_wrench",
            "participating_nonlinear_plant": True,
            "physical_effector_allocation": False,
            "source_physical_trim": False,
            "navigation_guidance": False,
            "screen_config_id": config.id,
            "plant_id": config.plant_id,
            "state_names": list(config.state_names),
            "assessment_state_names": list(config.assessment_state_names or config.state_names),
            "integration_dt_s": config.dt_s,
            "screen_duration_s": config.duration_s,
            "controller_method": config.controller_method,
            "integral_output_names": list(config.lqi_result.output_names) if config.lqi_result is not None else [],
            "controller_tuning_campaign_id": config.lqi_campaign_id,
            "controller_screen_execution": {
                "status": f"executed_by_this_{config.controller_method}_screen",
                "mission_id": self.definition.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": "direct_wrench",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
            "claim_boundary": self.definition.claim_boundary,
            "direct_wrench_limits": {
                "lower": dict(config.limits.lower),
                "upper": dict(config.limits.upper),
                "rate_limit_per_s": dict(config.limits.rate_limit_per_s),
                "authority_span": authority_span,
                "rate_limited_axes": list(rate_limited_axes),
            },
        }
        manifest["capability"] = capability
        if self.definition.paired_lqi_mission_id is not None:
            if (
                self.definition.paired_lqi_capability_adapter_id is None
                or self.definition.paired_lqi_campaign_id is None
            ):
                raise ValueError("paired local direct-wrench LQI advertisement is incomplete")
            capability["offset_free_tuning_candidate"] = {
                "campaign_id": self.definition.paired_lqi_campaign_id,
                "method": "lqi",
                "availability": "executed_by_paired_composition_screen",
                "controller_screen_execution": {
                    "status": "executed_by_paired_lqi_screen",
                    "mission_id": self.definition.paired_lqi_mission_id,
                    "capability_adapter_id": self.definition.paired_lqi_capability_adapter_id,
                    "operations": ["validate", "batch"],
                    "control_realization": "direct_wrench",
                },
                "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
                "claim_boundary": (
                    "The current public batch screen executes its declared LQR controller. The paired LQI mission "
                    "executes one integrated body-speed recovery through the same bounded generalized direct-wrench "
                    "bridge; neither screen establishes physical effectors, a wind or mass-variation environment, "
                    "navigation, or a vehicle qualification result."
                ),
            }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                f"pinned {self.definition.family_id} local source-load screen declares finite six-axis direct-wrench "
                "bounds and cadence; executing the recovery screen remains required to establish its local result",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


def _registry(*, plugins: PluginCatalog | None = None) -> LocalDirectWrenchScreenRegistry:
    """Resolve selected plug-in contributions before inspecting screen ownership."""

    if plugins is None:
        from .plugins import current_plugin_catalog, discover_plugins

        plugins = current_plugin_catalog()
        if plugins is None:
            plugins = discover_plugins()
    contributed = plugins.build_local_direct_wrench_screen_registry().definitions
    return LocalDirectWrenchScreenRegistry(definitions=contributed)
    ####


def local_direct_wrench_screen_definitions(
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[LocalDirectWrenchScreenDefinition, ...]:
    """Return direct-wrench endpoints visible in the selected plug-in scope."""

    return _registry(plugins=plugins).definitions
    ####


def resolve_local_direct_wrench_screen_definition(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> LocalDirectWrenchScreenDefinition | None:
    """Resolve a screen without family or fidelity fallback."""

    return _registry(plugins=plugins).resolve(composition)
    ####


def resolve_local_direct_wrench_screen_advertisement(
    *,
    family_id: str | None,
    mission_id: str | None,
    fidelity: str,
    plugins: PluginCatalog | None = None,
) -> dict[str, object] | None:
    """Return selected static metadata without compiling a screen or tuning."""

    return _registry(plugins=plugins).advertisement(
        family_id=family_id,
        mission_id=mission_id,
        fidelity=fidelity,
    )
    ####


__all__ = [
    "LocalDirectWrenchScreenCapabilityAdapter",
    "LocalDirectWrenchScreenDefinition",
    "LocalDirectWrenchScreenRegistry",
    "local_direct_wrench_screen_definitions",
    "resolve_local_direct_wrench_screen_advertisement",
    "resolve_local_direct_wrench_screen_definition",
]
