"""Mission-capability advertisement owned by the passive-bodies plug-in."""

from __future__ import annotations

import math

from taoryx.mission_capability import MissionCapabilityAdapter, MissionCapabilityEstimate, MissionFeasibility
from taoryx.vehicle_composition import CompiledVehicleComposition


class PassiveTumblingReleaseCapabilityAdapter:
    """Release/impact plausibility for the explicitly uncontrolled body witness."""

    id = "taoryx.passive_tumbling_release.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "tumbling_body"
            and composition.mission == "tumbling_body_release_damping_impact_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Report the declared release-energy and area-policy contract."""

        from taoryx.passive_tumbling_mission_translation import compile_passive_tumbling_mission

        plan = compile_passive_tumbling_mission(composition)
        release_altitude_m = plan.vehicle.initial_altitude_m
        release_speed_m_s = plan.vehicle.initial_speed_m_s
        gravity_m_s2 = plan.vehicle.gravity_m_s2
        vacuum_fall_time_s = math.sqrt(2.0 * release_altitude_m / gravity_m_s2)
        ballistic_coefficient_kg_m2 = plan.body.mass_kg / (plan.vehicle.drag_coefficient * plan.body.reference_area_m2)
        within_horizon = vacuum_fall_time_s <= plan.horizon_s
        feasibility: MissionFeasibility = "likely_feasible" if within_horizon else "unknown"
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "uncontrolled_passive_body",
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "release_altitude_m": release_altitude_m,
            "release_speed_m_s": release_speed_m_s,
            "release_specific_energy_j_kg": 0.5 * release_speed_m_s**2 + gravity_m_s2 * release_altitude_m,
            "ballistic_coefficient_kg_m2": ballistic_coefficient_kg_m2,
            "vacuum_fall_time_lower_bound_s": vacuum_fall_time_s,
            "simulation_horizon_s": plan.horizon_s,
            "horizon_contains_vacuum_fall_lower_bound": within_horizon,
            "area_policy": plan.area_policy,
        }
        diagnostic = (
            "declared release horizon contains the vacuum-fall lower bound; native passive propagation still determines "
            "atmospheric impact and rotational behavior"
            if within_horizon
            else "declared horizon is shorter than the vacuum-fall lower bound; impact cannot be preflight-assured"
        )
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=(diagnostic,),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def passive_mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return the complete passive-bodies capability surface."""

    return (PassiveTumblingReleaseCapabilityAdapter(),)
    ####


__all__ = ["PassiveTumblingReleaseCapabilityAdapter", "passive_mission_capability_adapters"]
