"""NESC-owned capability advertisement for the pinned source replay."""

from __future__ import annotations

from typing import Any

from taoryx.nesc_mission_translation import compile_nesc_source_replay_mission

from taoryx.mission_capability import MissionCapabilityAdapter, MissionCapabilityEstimate, MissionFeasibility
from taoryx.vehicle_composition import CompiledVehicleComposition


class NescSourceReplayCapabilityAdapter:
    """Stage/event continuity report for the immutable NESC replay witness."""

    id = "taoryx.nesc_staged_source_replay.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this exact NESC replay selection is owned here."""

        return (
            composition.family_id == "reference_nesc_two_stage_rocket"
            and composition.mission == "staged_rocket_launch_target_state_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose pinned stage chronology without calling it a rocket controller."""

        plan = compile_nesc_source_replay_mission(composition)
        segments = tuple(plan.segments)
        event_order_valid = all(
            segment.end_time_s >= segment.start_time_s for segment in segments
        ) and all(
            current.start_time_s >= previous.start_time_s
            for previous, current in zip(segments, segments[1:], strict=False)
        )
        manifest = plan.manifest()
        capability: dict[str, Any] = {
            "control_realization": (
                "uncontrolled_source_replay"
                if composition.fidelity == "point_mass_3dof"
                else "named_attitude_response_source_replay"
            ),
            "participating_nonlinear_plant": False,
            "physical_gimbal_allocation": False,
            "source_initial_mass_kg": plan.source_initial_mass_kg,
            "source_initial_heading_deg": plan.source_initial_heading_deg,
            "source_replay_duration_s": segments[-1].end_time_s,
            "stage_event_order_valid": event_order_valid,
            "required_truth_event_count": sum(len(segment.required_truth_events) for segment in segments),
        }
        manifest["capability"] = capability
        feasibility: MissionFeasibility = "likely_feasible" if event_order_valid else "unknown"
        diagnostic = (
            "pinned source history provides ordered ignition, separation, cutoff, and terminal events; "
            "the replay has no participating propulsion, gimbal, or guidance plant"
            if event_order_valid
            else "pinned source history has non-monotonic stage intervals; replay feasibility is unknown"
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


def nesc_mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return the NESC-only capability adapter in stable registration order."""

    return (NescSourceReplayCapabilityAdapter(),)
    ####


__all__ = ["NescSourceReplayCapabilityAdapter", "nesc_mission_capability_adapters"]
