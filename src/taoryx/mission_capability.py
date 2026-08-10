"""Plugin-neutral mission-capability contracts and registry resolution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Protocol, cast

from .plugins import PluginCatalog, discover_plugins
from .vehicle_composition import CompiledVehicleComposition

if TYPE_CHECKING:
    from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack

MissionFeasibility = Literal[
    "feasible",
    "likely_feasible",
    "unknown",
    "likely_infeasible",
    "certainly_infeasible",
]


@dataclass(frozen=True, slots=True)
class MissionCapabilityEstimate:
    """One transparent first-pass feasibility estimate for a composition."""

    adapter_id: str
    family_id: str
    mission_id: str
    fidelity: str
    feasibility: MissionFeasibility
    diagnostics: tuple[str, ...]
    manifest: dict[str, object]
    plan: object

    def as_dict(self) -> dict[str, object]:
        """Return the discovery/preflight-safe part of the estimate."""

        return {
            "schema": "taoryx.mission-capability-estimate/v1alpha1",
            "adapter_id": self.adapter_id,
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "feasibility": self.feasibility,
            "diagnostics": list(self.diagnostics),
            "derived_mission": self.manifest,
            "claim_boundary": (
                "This is a family-owned first-pass capability estimate. It does not replace native adapter "
                "binding, trim, control, integration, truth-objective evaluation, or qualification."
            ),
        }
        ####

    ####


class MissionCapabilityAdapter(Protocol):
    """Family-specific semantic mission estimator contract."""

    id: str

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this exact family/mission/fidelity is owned here."""

        ...

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Derive a source-owned first-pass estimate or raise for invalid intent."""

        ...


def mission_capability_adapters(
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[MissionCapabilityAdapter, ...]:
    """Return installed family-owned planners in deterministic plugin order."""

    selected = plugins or discover_plugins()
    adapters: list[MissionCapabilityAdapter] = []
    for contribution in selected.records("mission_capability_adapter"):
        value = contribution.value
        if (
            not isinstance(getattr(value, "id", None), str)
            or not callable(getattr(value, "supports", None))
            or not callable(getattr(value, "estimate", None))
        ):
            raise TypeError(
                f"plug-in {contribution.plugin.id!r} supplied an invalid mission capability adapter "
                f"for {contribution.id!r}"
            )
        adapters.append(cast(MissionCapabilityAdapter, value))
    return tuple(adapters)
    ####


def resolve_mission_capability_adapter(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> MissionCapabilityAdapter | None:
    """Resolve one exact installed planner without a nearest-family fallback."""

    matches = tuple(adapter for adapter in mission_capability_adapters(plugins=plugins) if adapter.supports(composition))
    if len(matches) > 1:
        raise ValueError(
            "multiple mission capability adapters claim "
            f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
        )
    declared = declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    if not matches:
        if declared is not None:
            raise ValueError(
                "mission template declares capability adapter "
                f"{declared!r}, but no installed adapter supports "
                f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
            )
        return None
    adapter = matches[0]
    if declared is None:
        raise ValueError(
            "installed capability adapter "
            f"{adapter.id!r} has no mission-template declaration for "
            f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
        )
    if adapter.id != declared:
        raise ValueError(
            "mission-template capability adapter does not match installed adapter: "
            f"declared {declared!r}, observed {adapter.id!r}"
        )
    return adapter
    ####


def declared_mission_capability_adapter(
    family_id: str,
    mission_id: str,
    fidelity: str,
) -> str | None:
    """Return a discoverable adapter ID without constructing a composition."""

    from .vehicle_composition_registry import load_vehicle_composition_registry

    registry = load_vehicle_composition_registry()
    family = next((item for item in registry.vehicles if item.family_id == family_id), None)
    if family is None:
        return None
    mission = next((item for item in family.mission_templates if item.id == mission_id), None)
    if mission is None or fidelity not in mission.mission_capability_fidelities:
        return None
    return mission.mission_capability_adapter_id
    ####


def estimate_mission_capability(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> MissionCapabilityEstimate | None:
    """Return the exact installed estimate or ``None`` when no family owns it."""

    adapter = resolve_mission_capability_adapter(composition, plugins=plugins)
    return None if adapter is None else adapter.estimate(composition)
    ####


def compile_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> CapabilityScaledRacetrack:
    """Project the registered fixed-wing capability planner's concrete plan."""

    estimate = estimate_mission_capability(composition, plugins=plugins)
    from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack

    if estimate is None or not isinstance(estimate.plan, CapabilityScaledRacetrack):
        observed = None if estimate is None else estimate.adapter_id
        raise ValueError(
            "powered-fixed-wing racetrack requires a family-owned CapabilityScaledRacetrack plan; "
            f"observed adapter {observed!r}"
        )
    return estimate.plan
    ####


__all__ = [
    "MissionCapabilityAdapter",
    "MissionCapabilityEstimate",
    "MissionFeasibility",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "declared_mission_capability_adapter",
    "estimate_mission_capability",
    "mission_capability_adapters",
    "resolve_mission_capability_adapter",
]
####
