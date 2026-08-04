"""Declared source-local direct-wrench screen definitions.

These screens are deliberately narrow bridge evidence.  A definition binds a
single family-owned source operating point to the common local LQR evaluator;
it does not turn that point into a route mission, trim result, or physical
effector-allocation claim.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenDefinition:
    """One exact family-owned source-local screen declaration."""

    family_id: str
    mission_id: str
    initialization_id: str
    capability_adapter_id: str
    config_factory: Callable[[], LocalDirectWrenchScreenConfig]
    claim_boundary: str

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


def local_direct_wrench_screen_definitions() -> tuple[LocalDirectWrenchScreenDefinition, ...]:
    """Return installed source-local direct-wrench screens in stable order."""

    from .hl20_adapter import build_hl20_local_direct_wrench_screen_config
    from .x15_adapter import build_x15_local_direct_wrench_screen_config

    return (
        LocalDirectWrenchScreenDefinition(
            family_id="x15",
            mission_id="x15_local_direct_wrench_screen_v1",
            initialization_id="source_release_glide_local_point",
            capability_adapter_id="taoryx.x15_local_direct_wrench_screen.capability.v1",
            config_factory=build_x15_local_direct_wrench_screen_config,
            claim_boundary=(
                "This is a local X-15 source-load direct-wrench LQR recovery screen. It proves only the declared "
                "source-local derivative, explicit bounded wrench projection, and local error-recovery result. It "
                "does not prove X-15 flight trim, release, propulsion scheduling, navigation, terminal handoff, "
                "physical stabilator/rudder/RCS allocation, or vehicle-family qualification."
            ),
        ),
        LocalDirectWrenchScreenDefinition(
            family_id="hl20_mod_k",
            mission_id="hl20_local_direct_wrench_screen_v1",
            initialization_id="source_subsonic_local_point",
            capability_adapter_id="taoryx.hl20_local_direct_wrench_screen.capability.v1",
            config_factory=build_hl20_local_direct_wrench_screen_config,
            claim_boundary=(
                "This is a local HL-20 source-load direct-wrench LQR recovery screen at one pinned subsonic "
                "source condition. It proves only the declared source-local derivative, explicit bounded wrench "
                "projection, and local error-recovery result. It does not prove flight trim, release, glide guidance, "
                "high-energy performance, physical control-surface allocation, or lifting-body-family qualification."
            ),
        ),
    )
    ####


def resolve_local_direct_wrench_screen_definition(
    composition: CompiledVehicleComposition,
) -> LocalDirectWrenchScreenDefinition | None:
    """Resolve one exact screen without a nearest-family fallback."""

    return next((definition for definition in local_direct_wrench_screen_definitions() if definition.supports(composition)), None)
    ####


__all__ = [
    "LocalDirectWrenchScreenDefinition",
    "local_direct_wrench_screen_definitions",
    "resolve_local_direct_wrench_screen_definition",
]
