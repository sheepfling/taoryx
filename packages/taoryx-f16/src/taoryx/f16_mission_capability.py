"""F-16-owned capability advertisement for its source-subsonic racetrack."""

from __future__ import annotations

from taoryx.mission_capability import MissionCapabilityEstimate, MissionFeasibility
from taoryx.vehicle_composition import CompiledVehicleComposition

from .f16_mission_translation import compile_f16_powered_fixed_wing_racetrack_from_composition


class F16SourceRacetrackCapabilityAdapter:
    """Plan only the F-16 family through its package-owned source profile."""

    id = "taoryx.f16_racetrack.source_route.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "f16_s119"
            and composition.mission == "powered_fixed_wing_racetrack_v1"
            and composition.fidelity
            in {
                "point_mass_3dof",
                "pseudo_6dof",
                "rigid_body_6dof_direct_wrench",
                "rigid_body_6dof_surface_allocated",
            }
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        proposal = compile_f16_powered_fixed_wing_racetrack_from_composition(composition)
        feasibility: MissionFeasibility = "feasible" if proposal.status == "capability_feasible" else "likely_feasible"
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=proposal.diagnostics,
            manifest=proposal.manifest(),
            plan=proposal,
        )
        ####

    ####


__all__ = ["F16SourceRacetrackCapabilityAdapter"]
