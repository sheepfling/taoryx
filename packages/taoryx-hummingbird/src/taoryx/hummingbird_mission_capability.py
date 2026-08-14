"""Hummingbird-owned Mission Composition capability estimate."""

from __future__ import annotations

import math
from typing import Any

from .hummingbird_mission_translation import compile_hummingbird_pseudo_mission
from .mission_capability import MissionCapabilityEstimate, MissionFeasibility
from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFModel
from .vehicle_composition import CompiledVehicleComposition


class HummingbirdHoverTranslationCapabilityAdapter:
    """Aggregate-thrust feasibility estimate for the Hummingbird pseudo tier."""

    id = "taoryx.multirotor_hover_translation.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hummingbird"
            and composition.mission == "multirotor_pad_box_yaw_recovery_land_v1"
            and composition.fidelity == "pseudo_6dof"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Derive aggregate thrust reserve without claiming rotor allocation."""

        plan = compile_hummingbird_pseudo_mission(composition)
        model = HummingbirdPseudo6DOFModel(mass_kg=plan.initial_mass_kg)
        hover_thrust_n = model.mass_kg * model.gravity_m_s2
        thrust_margin_n = model.maximum_thrust_n - hover_thrust_n
        maximum_vertical_acceleration_m_s2 = thrust_margin_n / model.mass_kg
        maximum_level_lateral_acceleration_m_s2 = math.sqrt(
            max(0.0, model.maximum_thrust_n**2 - hover_thrust_n**2)
        ) / model.mass_kg
        infeasible = thrust_margin_n <= 0.0
        feasibility: MissionFeasibility = "certainly_infeasible" if infeasible else "likely_feasible"
        diagnostics = (
            "aggregate-thrust reserve is insufficient to maintain a stationary hover"
            if infeasible
            else "aggregate-thrust reserve supports hover; translation/yaw tracking remains a response-law and mission-execution check"
        )
        manifest = plan.manifest()
        capability: dict[str, Any] = {
            "model_id": model.profile_id,
            "control_realization": "aggregate_thrust_vector_surrogate",
            "mass_kg": model.mass_kg,
            "maximum_thrust_n": model.maximum_thrust_n,
            "hover_thrust_n": hover_thrust_n,
            "thrust_margin_n": thrust_margin_n,
            "maximum_vertical_acceleration_m_s2": maximum_vertical_acceleration_m_s2,
            "maximum_level_lateral_acceleration_m_s2": maximum_level_lateral_acceleration_m_s2,
            "physical_motor_allocation": False,
        }
        manifest["capability"] = capability
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=(diagnostics,),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


__all__ = ["HummingbirdHoverTranslationCapabilityAdapter"]
