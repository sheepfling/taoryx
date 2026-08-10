"""Reachability-owned mission-capability planning adapters."""

from __future__ import annotations

import math
from typing import Any

from taoryx.hl20_source_aerodynamics import HL20_SOURCE_ENVELOPE, HL20_SOURCE_MODEL_ID, HL20_SOURCE_SPEED_OF_SOUND_M_S
from taoryx.hl20_source_model import HL20_FIXED_MASS_KG, HL20_PACKAGE_SHA256

from taoryx.mission_capability import MissionCapabilityAdapter, MissionCapabilityEstimate, MissionFeasibility
from taoryx.vehicle_composition import CompiledSegment, CompiledVehicleComposition


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
        """Report the canonical body's release-energy and area-policy contract."""

        from taoryx.passive_tumbling_mission_translation import compile_passive_tumbling_mission

        plan = compile_passive_tumbling_mission(composition)
        release_altitude_m = plan.vehicle.initial_altitude_m
        release_speed_m_s = plan.vehicle.initial_speed_m_s
        gravity_m_s2 = 9.80665
        vacuum_fall_time_s = math.sqrt(2.0 * release_altitude_m / gravity_m_s2)
        ballistic_coefficient_kg_m2 = plan.body.mass_kg / (
            plan.vehicle.drag_coefficient * plan.body.reference_area_m2
        )
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
            "declared release horizon contains the vacuum-fall lower bound; native passive propagation still "
            "determines atmospheric impact and rotational behavior"
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


class X15StagedReachabilityCapabilityAdapter:
    """Source-pinned staging and horizon estimate for the reduced X-15 witness."""

    id = "taoryx.x15_staged_reachability.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "x15"
            and composition.mission == "x15_staged_booster_reachability_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose source-pinned boost/release chronology and reduced-model limits."""

        from taoryx.x15_staged_mission_translation import compile_x15_staged_reachability_mission

        plan = compile_x15_staged_reachability_mission(composition)
        staging = dict(plan.source_staging)
        powered_duration_s = float(staging["powered_duration_s"])
        release_time_s = float(staging["release_time_s"])
        ordered = 0.0 <= powered_duration_s <= release_time_s <= plan.horizon_s
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "open_loop" if composition.fidelity == "point_mass_3dof" else "response_law",
            "participating_nonlinear_plant": False,
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "source_launch_mass_kg": float(staging["launch_mass_kg"]),
            "source_initial_speed_m_s": float(staging["initial_speed_m_s"]),
            "powered_duration_s": powered_duration_s,
            "release_time_s": release_time_s,
            "simulation_horizon_s": plan.horizon_s,
            "staging_and_horizon_order_valid": ordered,
        }
        feasibility: MissionFeasibility = "likely_feasible" if ordered else "unknown"
        diagnostic = (
            "source-pinned boost, release, and reduced glide horizon are ordered; the retained witness still "
            "does not provide guidance, allocation, or a physical X-15 plant"
            if ordered
            else "source staging/release chronology is inconsistent with the declared horizon"
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


class HL20GlideEnergyCapabilityAdapter:
    """Source-bound first-pass energy and bank-intent check for the HL-20."""

    id = "taoryx.hl20_glide_energy.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hl20_mod_k"
            and composition.mission == "lifting_body_glide_energy_management_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Check energy direction and required opposing bank intent."""

        release = composition.initialization.inputs
        trim_capture = _segment(composition, "trim_capture")
        bank_reversals = tuple(segment for segment in composition.segments if segment.id == "glide_bank_reversal")
        handoff = _segment(composition, "energy_handoff")
        if len(bank_reversals) != 2:
            raise ValueError("HL-20 glide-energy mission requires exactly two glide_bank_reversal segments")

        release_altitude_m = _number(release, "altitude_m")
        release_mach = _number(release, "mach")
        release_heading_deg = _number(release, "heading_deg")
        release_mass_kg = _optional_number(release, "mass_kg", HL20_FIXED_MASS_KG)
        release_speed_m_s = release_mach * HL20_SOURCE_SPEED_OF_SOUND_M_S
        gravity_m_s2 = 9.80665
        release_specific_energy_j_kg = 0.5 * release_speed_m_s**2 + gravity_m_s2 * release_altitude_m

        bank_commands_deg = tuple(_number(segment, "bank_command_deg") for segment in bank_reversals)
        crossrange_targets_m = tuple(_number(segment, "target_crossrange_m") for segment in bank_reversals)
        target_altitude_m = _number(handoff, "target_altitude_m")
        target_energy_j_kg = _number(handoff, "target_energy")
        target_heading_deg = _number(handoff, "target_heading_deg")
        trim_duration_s = _number(trim_capture, "duration_s")
        energy_margin_j_kg = release_specific_energy_j_kg - target_energy_j_kg
        opposing_bank_intent = bank_commands_deg[0] * bank_commands_deg[1] < 0.0
        finite_bank_geometry = all(abs(command_deg) < 90.0 for command_deg in bank_commands_deg)
        valid_release = release_altitude_m >= 0.0 and release_mach >= 0.0 and release_mass_kg > 0.0
        valid_trim_duration = trim_duration_s > 0.0
        source_domain_covered = (
            HL20_SOURCE_ENVELOPE["mach_min"] <= release_mach <= HL20_SOURCE_ENVELOPE["mach_max"]
            and HL20_SOURCE_ENVELOPE["altitude_min_m"] <= release_altitude_m <= HL20_SOURCE_ENVELOPE["altitude_max_m"]
        )
        source_runtime_admission = {
            "status": "source_domain_covered_runtime_unbound" if source_domain_covered else "outside_source_domain",
            "source_model_id": HL20_SOURCE_MODEL_ID,
            "release": {"mach": release_mach, "altitude_m": release_altitude_m},
            "declared_source_envelope": {
                "mach": [HL20_SOURCE_ENVELOPE["mach_min"], HL20_SOURCE_ENVELOPE["mach_max"]],
                "altitude_m": [HL20_SOURCE_ENVELOPE["altitude_min_m"], HL20_SOURCE_ENVELOPE["altitude_max_m"]],
            },
            "source_domain_covered": source_domain_covered,
            "runtime_factory_declared": False,
            "required_before_runtime_binding": (
                [
                    "source_aerodynamic_model_covering_release_domain",
                    "high_altitude_trim_and_gravity_binding",
                    "energy_crossrange_segment_executor",
                    "terminal_handoff_evaluator",
                ]
                if not source_domain_covered
                else [
                    "high_altitude_trim_and_gravity_binding",
                    "energy_crossrange_segment_executor",
                    "terminal_handoff_evaluator",
                    "runtime_factory_binding",
                ]
            ),
            "claim_boundary": (
                "This is a source-domain and runtime-admission advertisement. It does not establish a valid "
                "trajectory, trim point, control realization, or qualification result."
            ),
        }

        diagnostics: list[str] = [
            "HL-20 estimate is an unpowered specific-energy necessary-condition screen; it does not prove "
            "aerodynamic trim, crossrange closure, physical effector allocation, or nonlinear execution",
        ]
        if not valid_release:
            feasibility: MissionFeasibility = "certainly_infeasible"
            diagnostics.append("release altitude, Mach, and mass must define a nonnegative, positive-mass release state")
        elif energy_margin_j_kg < 0.0:
            feasibility = "certainly_infeasible"
            diagnostics.append("terminal specific-energy target exceeds the unpowered release specific energy")
        elif not finite_bank_geometry:
            feasibility = "certainly_infeasible"
            diagnostics.append("bank-intent geometry reaches or exceeds 90 degrees and is not representable as a finite glide turn")
        elif not valid_trim_duration or not opposing_bank_intent:
            feasibility = "likely_infeasible"
            diagnostics.append(
                "mission lacks a positive trim-capture duration or opposing signed bank commands, so it does not yet "
                "express the declared bank-reversal objective"
            )
        else:
            feasibility = "likely_feasible"
            diagnostics.append(
                "release energy exceeds the requested handoff energy and two finite, opposing bank commands are declared; "
                "native trajectory execution must still establish crossrange and terminal-corridor success"
            )
        if source_domain_covered:
            diagnostics.append(
                "release is inside the declared DAVE-ML Mach/altitude domain, but no high-altitude trim, gravity, "
                "energy/crossrange executor, terminal evaluator, or runtime factory is bound"
            )
        else:
            diagnostics.append(
                "release is outside the declared DAVE-ML source Mach/altitude domain, so the current source model "
                "cannot be used as a high-altitude glide-energy runtime"
            )

        manifest: dict[str, object] = {
            "schema": "taoryx.hl20-glide-energy-capability/v1alpha1",
            "source_model_id": HL20_SOURCE_MODEL_ID,
            "source_package_sha256": HL20_PACKAGE_SHA256,
            "release": {
                "altitude_m": release_altitude_m,
                "mach": release_mach,
                "speed_of_sound_m_s": HL20_SOURCE_SPEED_OF_SOUND_M_S,
                "speed_m_s": release_speed_m_s,
                "heading_deg": release_heading_deg,
                "mass_kg": release_mass_kg,
                "specific_energy_j_kg": release_specific_energy_j_kg,
            },
            "mission_intent": {
                "trim_capture_duration_s": trim_duration_s,
                "bank_commands_deg": list(bank_commands_deg),
                "crossrange_targets_m": list(crossrange_targets_m),
                "opposing_bank_intent": opposing_bank_intent,
                "target_altitude_m": target_altitude_m,
                "target_specific_energy_j_kg": target_energy_j_kg,
                "target_heading_deg": target_heading_deg,
            },
            "capability": {
                "control_realization": "unpowered_lifting_body_energy_bank_intent",
                "participating_nonlinear_plant": False,
                "physical_effector_allocation": False,
                "direct_wrench_injection": False,
                "release_specific_energy_j_kg": release_specific_energy_j_kg,
                "target_specific_energy_j_kg": target_energy_j_kg,
                "available_specific_energy_margin_j_kg": energy_margin_j_kg,
                "opposing_bank_intent": opposing_bank_intent,
                "finite_bank_geometry": finite_bank_geometry,
                "valid_trim_duration": valid_trim_duration,
                "source_runtime_admission": source_runtime_admission,
            },
            "claim_boundary": (
                "This capability estimate is not a trim result, a native flight-plan lowering, a surface-allocation "
                "claim, or a qualified HL-20 trajectory."
            ),
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=tuple(diagnostics),
            manifest=manifest,
            plan=manifest,
        )
        ####

    ####


class HL20SourceBoosterReleaseReplayCapabilityAdapter:
    """Chronology check for the exact source-aerodynamic HL-20 release witness."""

    id = "taoryx.hl20_source_booster_release_replay.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hl20_mod_k"
            and composition.mission == "hl20_source_booster_release_replay_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose the source schedule without promoting it to guidance control."""

        from taoryx.hl20_source_release_mission_translation import compile_hl20_source_booster_release_mission

        plan = compile_hl20_source_booster_release_mission(composition)
        ordered = all(
            current.end_time_s >= current.start_time_s
            and current.start_time_s >= previous.start_time_s
            for previous, current in zip(plan.segments, plan.segments[1:], strict=False)
        )
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": (
                "source_scheduled_force_model"
                if composition.fidelity == "point_mass_3dof"
                else "source_scheduled_named_attitude_response"
            ),
            "source_aerodynamic_graph": True,
            "participating_guidance_controller": False,
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "source_schedule_order_valid": ordered,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible" if ordered else "unknown",
            diagnostics=(
                "the fixed source booster/release and bank schedule is chronologically ordered; it is a source-scheduled "
                "aerodynamic witness, not a user-commanded guidance or effector-allocation mission",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def reachability_mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return reachability-owned planners in deterministic order."""

    return (
        PassiveTumblingReleaseCapabilityAdapter(),
        X15StagedReachabilityCapabilityAdapter(),
        HL20GlideEnergyCapabilityAdapter(),
        HL20SourceBoosterReleaseReplayCapabilityAdapter(),
    )
    ####


def _segment(composition: CompiledVehicleComposition, segment_id: str) -> CompiledSegment:
    matches = tuple(segment for segment in composition.segments if segment.id == segment_id)
    if len(matches) != 1:
        raise ValueError(f"composition requires exactly one {segment_id!r} segment")
    return matches[0]
    ####


def _number(inputs: Any, field: str) -> float:
    values = inputs if isinstance(inputs, dict) else inputs.inputs
    value = values[field].value
    if isinstance(value, bool):
        raise ValueError(f"{field} must be numeric")
    return float(value)
    ####


def _optional_number(inputs: Any, field: str, default: float) -> float:
    values = inputs if isinstance(inputs, dict) else inputs.inputs
    return default if field not in values else _number(values, field)
    ####


__all__ = [
    "HL20GlideEnergyCapabilityAdapter",
    "HL20SourceBoosterReleaseReplayCapabilityAdapter",
    "PassiveTumblingReleaseCapabilityAdapter",
    "X15StagedReachabilityCapabilityAdapter",
    "reachability_mission_capability_adapters",
]
####
