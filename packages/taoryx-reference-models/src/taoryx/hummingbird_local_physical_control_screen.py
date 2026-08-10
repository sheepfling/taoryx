"""Composition-owned individual-rotor local LQI screen for Hummingbird.

The source package already contains a RotorPy-derived quad-X local plant,
bounded four-motor allocation, and motor lag.  This module exposes that exact
path through Vehicle Composition instead of treating the standalone evidence
tool as a public runtime endpoint.  The screen is intentionally local: it is
not a pad-to-pad mission, a wind qualification, or a battery model.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path

from .composition_control_trace import (
    BatchControlSample,
    build_committed_control_trace,
    control_trace_summary,
)
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import (
    build_committed_resource_ledger,
    resource_ledger_summary,
)
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_capability import MissionCapabilityEstimate
from .physical_lqr import PhysicalWrenchLqiDesign, PhysicalWrenchLqiValidation, validate_nonlinear_wrench_lqi
from .runtime_control_adapter import RuntimeRigidBodyLocalPlant
from .source_table_multirotor import (
    build_hummingbird_individual_rotor_source_table_plant,
    build_hummingbird_local_physical_wrench_lqi_design,
    build_hummingbird_local_vertical_force_lqi_design,
)
from .trim import TrimResult
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_HOVER_MISSION_ID = "hummingbird_local_individual_rotor_lqi_screen_v1"
_HORIZONTAL_MISSION_ID = "hummingbird_local_horizontal_translation_lqi_screen_v1"
_VERTICAL_MISSION_ID = "hummingbird_local_vertical_translation_lqi_screen_v1"
_INITIALIZATION_ID = "source_individual_rotor_hover_local_point"
_HOVER_SEGMENT_ID = "individual_rotor_hover_lqi_screen"
_HORIZONTAL_SEGMENT_ID = "individual_rotor_horizontal_translation_lqi_screen"
_VERTICAL_SEGMENT_ID = "individual_rotor_vertical_translation_lqi_screen"
_CAPABILITY_ADAPTER_ID = "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1"
_HORIZONTAL_CAPABILITY_ADAPTER_ID = "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1"
_VERTICAL_CAPABILITY_ADAPTER_ID = "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1"
_SCREEN_DURATION_S = 1.0
_SCREEN_DT_S = 0.01
_HORIZONTAL_SCREEN_DT_S = 0.02
_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
_ATTITUDE_NAMES = ("roll_error_rad", "pitch_error_rad", "yaw_error_rad")
_RATE_NAMES = ("p_rad_s", "q_rad_s", "r_rad_s")
_MOMENT_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
_INTEGRAL_NAMES = (
    "integral_roll_error_rad_s",
    "integral_pitch_error_rad_s",
    "integral_yaw_error_rad_s",
)
_LOCAL_LIMITS = {
    "roll_error_rad": math.radians(15.0),
    "pitch_error_rad": math.radians(15.0),
    "yaw_error_rad": math.radians(20.0),
    "p_rad_s": math.radians(90.0),
    "q_rad_s": math.radians(90.0),
    "r_rad_s": math.radians(90.0),
}


@dataclass(frozen=True, slots=True)
class _HorizontalPhase:
    """One bounded horizontal local-frame controller target."""

    id: str
    target_xy_m: tuple[float, float]
    target_yaw_rad: float
    duration_s: float
    direction_axis: str | None = None
    direction_sign: int = 0
    ####


@dataclass(frozen=True, slots=True)
class _VerticalPhase:
    """One bounded source-local vertical target and capture interval."""

    id: str
    target_down_m: float
    duration_s: float
    ####


_HORIZONTAL_PHASES = (
    _HorizontalPhase("forward_body_leg", (5.0, 0.0), 0.0, 8.0, "u", 1),
    _HorizontalPhase("yaw_scan_gate", (5.0, 0.0), math.pi / 2.0, 6.0),
    _HorizontalPhase("lateral_body_right_leg", (2.0, 0.0), math.pi / 2.0, 8.0, "v", 1),
    _HorizontalPhase("rearward_body_leg", (2.0, -5.0), math.pi / 2.0, 8.0, "u", -1),
    _HorizontalPhase("horizontal_return_gate", (0.0, 0.0), math.pi / 2.0, 8.0),
)

_VERTICAL_PHASES = (
    _VerticalPhase("vertical_climb_capture", -1.5, 5.0),
    _VerticalPhase("vertical_hover_capture", -1.5, 3.0),
    _VerticalPhase("vertical_descent_capture", 0.0, 5.0),
    _VerticalPhase("vertical_return_hover_capture", 0.0, 3.0),
)

# The source problem declares the nominal 0.5 kg mass explicitly.  These are
# separate re-trimmed local cases with the *same* nominal-mass LQI design; they
# are not gain-schedule nodes or an in-flight payload transition.
_MASS_VARIATION_FACTORS = (0.85, 1.0, 1.15)


@dataclass(frozen=True, slots=True)
class HummingbirdLocalPhysicalControlScreenPlan:
    """Exact parameter-free lowering for the source-hover rotor LQI screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    duration_s: float
    dt_s: float

    @property
    def horizontal_translation(self) -> bool:
        """Return whether this plan owns the bounded horizontal waypoint screen."""

        return self.mission_id == _HORIZONTAL_MISSION_ID
        ####

    @property
    def vertical_translation(self) -> bool:
        """Return whether this plan owns the bounded vertical waypoint screen."""

        return self.mission_id == _VERTICAL_MISSION_ID
        ####

    @property
    def control_realization(self) -> str:
        """Return the exact physical-allocation realization for this screen."""

        return (
            "individual_rotor_source_lqi_force_moment_allocation"
            if self.vertical_translation
            else "individual_rotor_source_lqi_allocation"
        )
        ####

    @property
    def segment_id(self) -> str:
        """Return the one exact segment admitted by this plan."""

        if self.horizontal_translation:
            return _HORIZONTAL_SEGMENT_ID
        if self.vertical_translation:
            return _VERTICAL_SEGMENT_ID
        return _HOVER_SEGMENT_ID
        ####

    def manifest(self) -> dict[str, object]:
        """Return the bounded Composition-owned execution plan."""

        return {
            "schema": "taoryx.hummingbird-local-individual-rotor-lqi-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": self.segment_id,
                    "transition_semantics": "bounded_phase_completion_only"
                    if self.horizontal_translation or self.vertical_translation
                    else "screen_completion_only",
                }
            ],
            "control_realization": self.control_realization,
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "mass_variation_screen": {
                "status": "executed_by_this_vertical_lqi_screen" if self.vertical_translation else "not_applicable",
                "mass_factors": list(_MASS_VARIATION_FACTORS) if self.vertical_translation else [],
                "controller_policy": "one_fixed_nominal_mass_lqi_design_across_each_retrimmed_case",
                "claim_boundary": (
                    "Each declared mass case is an independent source-state re-trim of the same bounded local vertical "
                    "screen. It is not gain scheduling, an in-flight mass transition, a payload envelope, or qualification."
                    if self.vertical_translation
                    else "No mass-variation screen is selected for this endpoint."
                ),
            },
            "claim_boundary": (
                "This selects a bounded source-plant horizontal translation LQI screen with no altitude, collective, "
                "battery, contact, landing, wind, gain-schedule, or qualification claim."
                if self.horizontal_translation
                else "This selects a bounded source-plant vertical climb, hover, descent, and return-to-hover LQI "
                "screen through collective force and four allocated rotors. It excludes wind, battery, contact, "
                "landing, gain-schedule, and qualification claims."
                if self.vertical_translation
                else "This selects a pinned Hummingbird source-hover attitude/rate LQI screen. It does not execute "
                "takeoff, translation, landing, wind rejection, battery depletion, gain scheduling, or a qualified mission."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class HummingbirdLocalPhysicalControlScreenExecution:
    """Public result for one bounded source-rotor LQI local screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: HummingbirdLocalPhysicalControlScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return the deliberately narrow local-screen disposition."""

        return self.evaluation.get("mission_pass") is True
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the compact public record for CLI and batch callers."""

        return {
            "schema": "taoryx.hummingbird-local-individual-rotor-lqi-screen-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_hummingbird_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> HummingbirdLocalPhysicalControlScreenPlan:
    """Validate the exact source-hover local rotor-allocation selection."""

    if composition.family_id != "hummingbird":
        raise ValueError("Hummingbird physical LQI screen requires the hummingbird family")
    if composition.mission not in {_HOVER_MISSION_ID, _HORIZONTAL_MISSION_ID, _VERTICAL_MISSION_ID}:
        raise ValueError(
            "Hummingbird physical LQI screen requires one of "
            f"{_HOVER_MISSION_ID!r}, {_HORIZONTAL_MISSION_ID!r}, {_VERTICAL_MISSION_ID!r}"
        )
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("Hummingbird physical LQI screen requires the surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"Hummingbird physical LQI screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("Hummingbird physical LQI screen does not accept initialization overrides")
    segment_id = (
        _HORIZONTAL_SEGMENT_ID
        if composition.mission == _HORIZONTAL_MISSION_ID
        else _VERTICAL_SEGMENT_ID
        if composition.mission == _VERTICAL_MISSION_ID
        else _HOVER_SEGMENT_ID
    )
    if len(composition.segments) != 1 or composition.segments[0].id != segment_id:
        raise ValueError(f"Hummingbird physical LQI screen requires exactly one {segment_id!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("Hummingbird physical LQI screen does not accept segment overrides")
    return HummingbirdLocalPhysicalControlScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        duration_s=sum(phase.duration_s for phase in _HORIZONTAL_PHASES)
        if composition.mission == _HORIZONTAL_MISSION_ID
        else sum(phase.duration_s for phase in _VERTICAL_PHASES)
        if composition.mission == _VERTICAL_MISSION_ID
        else _SCREEN_DURATION_S,
        dt_s=_HORIZONTAL_SCREEN_DT_S
        if composition.mission in {_HORIZONTAL_MISSION_ID, _VERTICAL_MISSION_ID}
        else _SCREEN_DT_S,
    )
    ####


class HummingbirdLocalPhysicalControlScreenCapabilityAdapter:
    """Advertise the source-backed individual-motor LQI screen."""

    id = _CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this exact bounded source-hover screen is selected."""

        return (
            composition.family_id == "hummingbird"
            and composition.mission == _HOVER_MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose physical authority without promoting it into a full mission."""

        plan = compile_hummingbird_local_physical_control_screen(composition)
        plant = build_hummingbird_individual_rotor_source_table_plant()
        design = (
            build_hummingbird_local_vertical_force_lqi_design()
            if plan.vertical_translation
            else build_hummingbird_local_physical_wrench_lqi_design()
        )
        manifest = plan.manifest()
        horizontal = plan.horizontal_translation
        vertical = plan.vertical_translation
        manifest["capability"] = {
            "control_realization": plan.control_realization,
            "controller_id": design.id,
            "controller_method": "lqi",
            "controller_hurwitz": design.result.hurwitz,
            "integral_output_names": list(design.result.output_names),
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "physical_motor_allocation": True,
            "motor_lag_s": 0.005,
            "effector_names": list(plant.control_names),
            "effector_bounds_rad_s": {
                name: [limits.lower, limits.upper]
                for name, limits in plant.effector_limits.items()
            },
            "source_mass_kg": plant.source_state.mass,
            "screen_duration_s": plan.duration_s,
            "integration_dt_s": plan.dt_s,
            "physical_screen_status": "executed_by_this_lqi_screen",
            "physical_screen_execution": {
                "status": "executed_by_this_lqi_screen",
                "mission_id": plan.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": plan.control_realization,
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
            "mass_variation_status": (
                "executed_by_this_vertical_lqi_screen"
                if vertical
                else "not_applicable_to_this_endpoint"
            ),
            "mass_variation_factors": list(_MASS_VARIATION_FACTORS) if vertical else [],
            "horizontal_translation": horizontal,
            "horizontal_phase_ids": [phase.id for phase in _HORIZONTAL_PHASES] if horizontal else [],
            "vertical_translation": vertical,
            "vertical_phase_ids": [phase.id for phase in _VERTICAL_PHASES] if vertical else [],
            "claim_boundary": (
                "This public batch screen executes a bounded horizontal local-frame position-error to tilt/yaw-reference "
                "layer above source-hover attitude LQI, then bounded four-rotor allocation and the declared motor lag. "
                "It is not altitude, wind, battery, landing, gain-schedule, or flight-qualification evidence."
                if horizontal
                else "This public batch screen executes a bounded local down-position to vertical-speed-reference layer "
                "above source-hover vertical-speed/attitude LQI, then requests collective force and body moments through "
                "six-axis effectiveness, bounded four-rotor allocation, and the declared motor lag. It is not wind, battery, "
                "contact, landing, gain-schedule, or flight-qualification evidence."
                if vertical
                else "This public batch screen executes one source-hover attitude/rate LQI controller through bounded "
                "four-rotor allocation and the declared motor lag. It is not position, wind, battery, landing, "
                "gain-schedule, or flight-qualification evidence."
            ),
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "bounded local horizontal position-error/tilt reference, output-integrating wrench LQI, bounded quad-X allocation, and 5 ms source motor lag are available; "
                "altitude, wind, battery, and landing mission evidence remain separate gates"
                if horizontal
                else "bounded local vertical position-error/speed reference, output-integrating collective-force and attitude LQI, six-axis bounded quad-X allocation, and 5 ms source motor lag are available; "
                "wind, battery, contact, and landing mission evidence remain separate gates"
                if vertical
                else "pinned RotorPy-derived hover trim, output-integrating wrench LQI, bounded quad-X allocation, and 5 ms source motor lag are available; "
                "position, wind, battery, and landing mission evidence remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


class HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter(
    HummingbirdLocalPhysicalControlScreenCapabilityAdapter
):
    """Advertise the bounded source-plant horizontal LQI waypoint screen."""

    id = _HORIZONTAL_CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether the exact horizontal local-frame screen is selected."""

        return (
            composition.family_id == "hummingbird"
            and composition.mission == _HORIZONTAL_MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    ####


class HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter(
    HummingbirdLocalPhysicalControlScreenCapabilityAdapter
):
    """Advertise the bounded source-plant vertical LQI waypoint screen."""

    id = _VERTICAL_CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether the exact vertical source-local screen is selected."""

        return (
            composition.family_id == "hummingbird"
            and composition.mission == _VERTICAL_MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    ####


def preflight_hummingbird_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Preflight the exact LQI/motor-allocation Composition route."""

    plan = compile_hummingbird_local_physical_control_screen(composition)
    adapter = (
        HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter()
        if plan.horizontal_translation
        else HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter()
        if plan.vertical_translation
        else HummingbirdLocalPhysicalControlScreenCapabilityAdapter()
    )
    estimate = adapter.estimate(composition)
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("Hummingbird physical LQI capability record is missing")
    controller_hurwitz = capability.get("controller_hurwitz") is True
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if controller_hurwitz else "blocked",
        translator_id=adapter.id,
        checks=(
            ExecutionPreflightCheck(
                "hummingbird.semantic_local_horizontal_translation_lqi_screen"
                if plan.horizontal_translation
                else "hummingbird.semantic_local_vertical_translation_lqi_screen"
                if plan.vertical_translation
                else "hummingbird.semantic_local_individual_rotor_lqi_screen",
                [_INITIALIZATION_ID, plan.segment_id],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "hummingbird.local_horizontal_translation_lqi_hurwitz"
                if plan.horizontal_translation
                else "hummingbird.local_vertical_translation_lqi_hurwitz"
                if plan.vertical_translation
                else "hummingbird.local_individual_rotor_lqi_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
        ),
        diagnostics=(
            (
                "composition lowers exactly to the Hummingbird bounded local horizontal translation LQI screen; "
                "it is not an altitude, wind, route, or landing translator"
                if plan.horizontal_translation
                else "composition lowers exactly to the Hummingbird bounded local vertical translation LQI screen; "
                "it is not a wind, contact, landing, or full-route translator"
                if plan.vertical_translation
                else "composition lowers exactly to the Hummingbird source-hover individual-rotor LQI screen; it is not a route or landing translator"
            ),
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_hummingbird_local_physical_control_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> HummingbirdLocalPhysicalControlScreenExecution:
    """Run the source motor/LQI path through the exact Composition binding."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence Hummingbird physical-control screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready Hummingbird local physical screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_hummingbird_local_physical_control_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = build_hummingbird_individual_rotor_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird physical-control screen trim failed: {trim.as_dict()}")
    design = (
        build_hummingbird_local_vertical_force_lqi_design()
        if plan.vertical_translation
        else build_hummingbird_local_physical_wrench_lqi_design()
    )
    mass_variation_report: dict[str, object] | None = None
    robustness_report: dict[str, object] | None = None
    validation_payload: dict[str, object]
    if plan.horizontal_translation:
        rows = _run_horizontal_translation_screen(plant, trim, design, plan)
        validation_payload = {
            "schema": "taoryx.hummingbird-local-horizontal-translation-lqi-validation/v1alpha1",
            "design": design.as_dict(),
            "duration_s": plan.duration_s,
            "dt_s": plan.dt_s,
            "sample_count": len(rows),
            "control_path": (
                "local horizontal position error -> bounded tilt/yaw reference -> output-integrating attitude LQI "
                "with explicit full-state feedback reference -> requested moments -> bounded four-rotor allocation -> "
                "source motor lag -> nonlinear source plant"
            ),
        }
    elif plan.vertical_translation:
        rows = _run_vertical_translation_screen(plant, trim, design, plan)
        mass_variation_report = _vertical_mass_variation_report(
            plant,
            trim,
            design,
            plan,
            nominal_rows=rows,
        )
        validation_payload = {
            "schema": "taoryx.hummingbird-local-vertical-translation-lqi-validation/v1alpha1",
            "design": design.as_dict(),
            "duration_s": plan.duration_s,
            "dt_s": plan.dt_s,
            "sample_count": len(rows),
            "mass_variation": mass_variation_report,
            "control_path": (
                "local down-position error -> bounded vertical-speed reference -> output-integrating vertical-speed/"
                "attitude LQI with explicit full-state feedback reference -> requested collective body-z force and "
                "moments -> six-axis bounded four-rotor allocation -> source motor lag -> nonlinear source plant"
            ),
        }
    else:
        validation, rows = _run_screen(plant, trim, design, plan)
        validation_payload = validation.as_dict()
        mass_variation_report = _hover_mass_variation_report(
            plant,
            trim,
            design,
            plan,
            nominal_validation=validation,
            nominal_rows=rows,
        )
        robustness_report = _hover_mass_variation_endpoint_artifact(mass_variation_report)
    for row in rows:
        row["mass_kg"] = float(plant.source_state.mass)
    finite = _finite_rows(rows)
    envelope = (
        _horizontal_local_envelope(rows)
        if plan.horizontal_translation
        else _vertical_local_envelope(rows)
        if plan.vertical_translation
        else _local_envelope(rows)
    )
    allocation_pass = all(
        row["allocation_status"] in {"feasible", "feasible_near_limit"}
        and _saturation_count(row) == 0
        for row in rows
    )
    integrators_exercised = (
        any(
            abs(_number(row, name)) > 1.0e-10
            for row in rows
            for name in (
                (*_INTEGRAL_NAMES, "integral_vertical_speed_m_s_s")
                if plan.vertical_translation
                else _INTEGRAL_NAMES
            )
        )
        if plan.horizontal_translation or plan.vertical_translation
        else validation.integrators_exercised
    )
    if plan.horizontal_translation:
        evaluation = _horizontal_screen_evaluation(
            rows,
            finite=finite,
            envelope=envelope,
            allocation_pass=allocation_pass,
            integrators_exercised=integrators_exercised,
            plan=plan,
        )
        recovery_pass = evaluation["mission_pass"] is True
        initial_error = _attitude_rate_error_norm(rows[0]) if rows else math.inf
        final_error = _attitude_rate_error_norm(rows[-1]) if rows else math.inf
        screen_pass = recovery_pass
    elif plan.vertical_translation:
        evaluation = _vertical_screen_evaluation(
            rows,
            finite=finite,
            envelope=envelope,
            allocation_pass=allocation_pass,
            integrators_exercised=integrators_exercised,
            plan=plan,
            mass_variation_report=mass_variation_report,
        )
        recovery_pass = evaluation["mission_pass"] is True
        initial_error = _vertical_position_speed_error_norm(rows[0]) if rows else math.inf
        final_error = _vertical_position_speed_error_norm(rows[-1]) if rows else math.inf
        screen_pass = recovery_pass
    else:
        initial_error = _attitude_rate_error_norm(validation.initial_state)
        final_error = _attitude_rate_error_norm(validation.final_state)
        recovery_pass = final_error < initial_error
        screen_pass = finite and bool(envelope["pass"]) and allocation_pass and integrators_exercised and recovery_pass
        evaluation = _screen_evaluation(
            finite,
            envelope,
            allocation_pass,
            integrators_exercised,
            recovery_pass,
            screen_pass,
            plan,
        )
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.multirotor.native_quad_x.v1",
        "controller_id": design.id,
        "controller_method": "lqi",
        "integral_output_names": list(design.result.output_names),
        "control_realization": plan.control_realization,
        "physical_effector_allocation": True,
        "physical_motor_allocation": True,
        "motor_lag_s": 0.005,
        "dt_s": plan.dt_s,
        "duration_s": _number(rows[-1], "time_s") if rows else 0.0,
        "numerical_valid": finite,
        "allocation_pass": allocation_pass,
        "integrators_exercised": integrators_exercised,
        "initial_attitude_rate_error_norm": initial_error,
        "final_attitude_rate_error_norm": final_error,
        "hard_gates_passed": screen_pass,
        "mass_variation": mass_variation_report,
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            (
                "The bounded horizontal local LQI screen records its declared phase sequence but has no general route "
                "graph dispatcher and makes no altitude, landing, or mission-transition claim."
                if plan.horizontal_translation
                else "The bounded vertical local LQI screen records source-local climb, hover, descent, and return captures "
                "but has no wind, contact, landing, or general route graph dispatcher."
                if plan.vertical_translation
                else "The local individual-rotor LQI screen has no route graph dispatcher and makes no mission-transition claim."
            ),
        ).as_dict(),
    }
    status_trace = build_committed_status_trace(composition, _status_samples(rows, float(plant.source_state.mass)))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = HummingbirdLocalPhysicalControlScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This bounded Hummingbird source-plant horizontal screen uses local horizontal position error only to select "
            "bounded tilt/yaw references for the same output-integrating attitude LQI. Every requested moment is allocated "
            "to four bounded RotorPy-source motor-speed controls with declared motor lag. It is not altitude/collective control, "
            "wind-bias rejection evidence, battery/SOC modeling, landing/contact validation, gain scheduling, or qualification."
            if plan.horizontal_translation
            else "This bounded Hummingbird source-plant vertical screen maps local down-position error to a bounded vertical-speed "
            "reference for output-integrating vertical-speed/attitude LQI. Every collective-force and moment request is allocated "
            "through six-axis source effectiveness to four bounded RotorPy-source motor-speed controls with declared motor lag. "
            "It is not wind-bias rejection evidence, battery/SOC modeling, contact/landing validation, gain scheduling, or qualification."
            if plan.vertical_translation
            else "This is a one-second Hummingbird source-hover attitude/rate LQI screen. Every requested moment is allocated "
            "to four bounded RotorPy-source motor-speed controls with declared motor lag. It is not position/altitude control, "
            "wind-bias rejection evidence, battery/SOC modeling, flight-path guidance, landing/contact validation, gain scheduling, or qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "nonlinear_validation.json", validation_payload)
    if mass_variation_report is not None:
        _write_json(destination / "mass_variation_report.json", mass_variation_report)
    if robustness_report is not None:
        _write_json(destination / "robustness_report.json", robustness_report)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "objective_report.json", evaluation)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "semantic_action_trace.json", control_trace)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            evaluation,
            runtime=runtime,
            envelope=envelope,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _run_screen(
    plant: object,
    trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
) -> tuple[PhysicalWrenchLqiValidation, list[dict[str, object]]]:
    """Run the shared physical-LQI validator on the pinned source-hover case."""

    initial_state = {name: float(trim.state[name]) for name in _STATE_NAMES}
    initial_state.update(
        {
            "roll_error_rad": math.radians(4.0),
            "pitch_error_rad": math.radians(-3.0),
            "yaw_error_rad": math.radians(5.0),
            "p_rad_s": math.radians(5.0),
            "q_rad_s": math.radians(-4.0),
            "r_rad_s": math.radians(5.0),
        }
    )
    validation = validate_nonlinear_wrench_lqi(
        plant,  # type: ignore[arg-type]
        trim,
        design,
        initial_state=initial_state,
        duration_s=plan.duration_s,
        dt_s=plan.dt_s,
        integral_lower={name: -0.5 for name in design.result.output_names},
        integral_upper={name: 0.5 for name in design.result.output_names},
    )
    return validation, _rows(validation)
    ####


def _run_horizontal_translation_screen(
    plant: object,
    trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
) -> list[dict[str, object]]:
    """Run bounded local horizontal targets through the actual rotor LQI path.

    Horizontal position is a source-local kinematic reconstruction of the
    source-owned body-velocity state.  It supplies only the outer reference
    layer; the participating plant still receives the four allocated motor
    commands and never a directly injected moment.
    """

    state = {name: float(trim.state[name]) for name in _STATE_NAMES}
    previous_effectors = {name: float(trim.controls[name]) for name in plant.control_names}  # type: ignore[attr-defined]
    controller = design.build_controller(
        integral_lower={name: -0.5 for name in design.result.output_names},
        integral_upper={name: 0.5 for name in design.result.output_names},
    )
    tracked_reference = {
        name: float(trim.state[name])
        for name in design.result.output_names
    }
    position_xy = [0.0, 0.0]
    rows: list[dict[str, object]] = []
    time_s = 0.0
    for phase in _HORIZONTAL_PHASES:
        for _ in range(round(phase.duration_s / plan.dt_s)):
            world_velocity = _world_velocity(state)
            error_x = phase.target_xy_m[0] - position_xy[0]
            error_y = phase.target_xy_m[1] - position_xy[1]
            desired_ax = _clamp(0.7 * error_x - 0.8 * world_velocity[0], -2.0, 2.0)
            desired_ay = _clamp(0.7 * error_y - 0.8 * world_velocity[1], -2.0, 2.0)
            yaw = float(state["yaw_error_rad"])
            body_ax = math.cos(yaw) * desired_ax + math.sin(yaw) * desired_ay
            body_ay = -math.sin(yaw) * desired_ax + math.cos(yaw) * desired_ay
            desired_reference = {
                "roll_error_rad": _clamp(body_ay / 9.80665, -0.18, 0.18),
                "pitch_error_rad": _clamp(-body_ax / 9.80665, -0.18, 0.18),
                "yaw_error_rad": phase.target_yaw_rad,
            }
            _slew_output_reference(tracked_reference, desired_reference, plan.dt_s)
            state_reference = {name: float(trim.state[name]) for name in design.result.state_names}
            state_reference.update(tracked_reference)
            command = controller.command(
                state,
                tracked_reference,
                dt=plan.dt_s,
                state_reference=state_reference,
            )
            requested = dict(design.projection.nominal_wrench)
            requested.update({name: float(value) for name, value in command.controls.items()})
            allocation = plant.allocate(state, requested, previous_effectors, plan.dt_s)  # type: ignore[attr-defined]
            previous_effectors = {name: float(value) for name, value in allocation.actuator.actual_positions.items()}
            derivative = plant.state_derivative(state, previous_effectors, {})  # type: ignore[attr-defined]
            state = {
                name: float(state[name]) + plan.dt_s * float(derivative[name])
                for name in _STATE_NAMES
            }
            time_s += plan.dt_s
            world_velocity = _world_velocity(state)
            position_xy[0] += plan.dt_s * world_velocity[0]
            position_xy[1] += plan.dt_s * world_velocity[1]
            wrench_saturated = tuple(command.saturated)
            position_saturated = tuple(allocation.allocation.position_saturated) + tuple(
                allocation.actuator.position_saturated
            )
            rate_limited = tuple(allocation.allocation.rate_limited) + tuple(allocation.actuator.rate_limited)
            row: dict[str, object] = {
                "time_s": time_s,
                "phase_id": phase.id,
                "position_local_m": [position_xy[0], position_xy[1], 0.0],
                "velocity_local_m_s": [world_velocity[0], world_velocity[1], float(state["w_m_s"])],
                "target_position_local_m": [phase.target_xy_m[0], phase.target_xy_m[1], 0.0],
                "target_yaw_rad": phase.target_yaw_rad,
                **{name: float(state[name]) for name in _STATE_NAMES},
                **{
                    f"requested_moment_{axis}_nm": float(requested[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"achieved_moment_{axis}_nm": float(allocation.achieved_wrench[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"residual_moment_{axis}_nm": float(allocation.achieved_residual_wrench[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                "allocation_status": str(allocation.allocation.status),
                "allocation_residual_norm": float(allocation.achieved_controlled_residual_norm),
                "saturation_count": len(wrench_saturated) + len(position_saturated) + len(rate_limited),
                "integral_roll_error_rad_s": float(controller.integral_error["roll_error_rad"]),
                "integral_pitch_error_rad_s": float(controller.integral_error["pitch_error_rad"]),
                "integral_yaw_error_rad_s": float(controller.integral_error["yaw_error_rad"]),
            }
            for rotor_index, name in enumerate(design.projection.effector_names, start=1):
                row[f"rotor_{rotor_index}_speed_rad_s"] = float(previous_effectors[name])
            rows.append(row)
    return rows
    ####


def _run_vertical_translation_screen(
    plant: object,
    trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
) -> list[dict[str, object]]:
    """Run bounded vertical targets through collective force and actual rotors.

    Down-position is a source-local kinematic reconstruction from the
    source-owned body-z velocity state, matching the retained native vertical
    witness.  It supplies only the outer speed reference.  The nonlinear
    plant receives four allocated motor commands, never a directly injected
    collective force or body moment.
    """

    state = {name: float(trim.state[name]) for name in _STATE_NAMES}
    previous_effectors = {name: float(trim.controls[name]) for name in plant.control_names}  # type: ignore[attr-defined]
    controller = design.build_controller(
        integral_lower={
            "roll_error_rad": -0.5,
            "pitch_error_rad": -0.5,
            "yaw_error_rad": -0.5,
            "w_m_s": -1.0,
        },
        integral_upper={
            "roll_error_rad": 0.5,
            "pitch_error_rad": 0.5,
            "yaw_error_rad": 0.5,
            "w_m_s": 1.0,
        },
    )
    tracked_reference = {
        name: float(trim.state[name])
        for name in design.result.output_names
    }
    down_position_m = 0.0
    mass_kg = float(plant.source_state.mass)  # type: ignore[attr-defined]
    wrench_weights = {
        "force_x_n": 0.0,
        "force_y_n": 0.0,
        "force_z_n": 1.0,
        "moment_x_nm": 1.0,
        "moment_y_nm": 1.0,
        "moment_z_nm": 1.0,
    }
    rows: list[dict[str, object]] = []
    time_s = 0.0
    for phase in _VERTICAL_PHASES:
        for _ in range(round(phase.duration_s / plan.dt_s)):
            down_error_m = phase.target_down_m - down_position_m
            desired_vertical_speed = _clamp(0.8 * down_error_m, -0.75, 0.75)
            tracked_reference["w_m_s"] += _clamp(
                desired_vertical_speed - tracked_reference["w_m_s"],
                -1.0 * plan.dt_s,
                1.0 * plan.dt_s,
            )
            state_reference = {
                name: float(trim.state[name])
                for name in design.result.state_names
            }
            state_reference["w_m_s"] = tracked_reference["w_m_s"]
            command = controller.command(
                state,
                tracked_reference,
                dt=plan.dt_s,
                state_reference=state_reference,
            )
            requested = dict(design.projection.nominal_wrench)
            requested.update({name: float(value) for name, value in command.controls.items()})
            allocation = plant.allocate_force_moment(  # type: ignore[attr-defined]
                state,
                requested,
                previous_effectors,
                plan.dt_s,
                wrench_weights=wrench_weights,
            )
            previous_effectors = {name: float(value) for name, value in allocation.actuator.actual_positions.items()}
            derivative = plant.state_derivative(state, previous_effectors, {})  # type: ignore[attr-defined]
            state = {
                name: float(state[name]) + plan.dt_s * float(derivative[name])
                for name in _STATE_NAMES
            }
            time_s += plan.dt_s
            down_position_m += plan.dt_s * float(state["w_m_s"])
            wrench_saturated = tuple(command.saturated)
            position_saturated = tuple(allocation.allocation.position_saturated) + tuple(
                allocation.actuator.position_saturated
            )
            rate_limited = tuple(allocation.allocation.rate_limited) + tuple(allocation.actuator.rate_limited)
            row: dict[str, object] = {
                "time_s": time_s,
                "phase_id": phase.id,
                "position_local_m": [0.0, 0.0, down_position_m],
                "velocity_local_m_s": [float(state["u_m_s"]), float(state["v_m_s"]), float(state["w_m_s"])],
                "target_position_local_m": [0.0, 0.0, phase.target_down_m],
                "target_vertical_speed_down_m_s": tracked_reference["w_m_s"],
                "vertical_speed_down_m_s": float(state["w_m_s"]),
                **{name: float(state[name]) for name in _STATE_NAMES},
                **{
                    f"requested_force_{axis}_n": float(requested[f"force_{axis}_n"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"achieved_force_{axis}_n": float(allocation.achieved_wrench[f"force_{axis}_n"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"residual_force_{axis}_n": float(allocation.achieved_residual_wrench[f"force_{axis}_n"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"requested_moment_{axis}_nm": float(requested[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"achieved_moment_{axis}_nm": float(allocation.achieved_wrench[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                **{
                    f"residual_moment_{axis}_nm": float(allocation.achieved_residual_wrench[f"moment_{axis}_nm"])
                    for axis in ("x", "y", "z")
                },
                "allocation_status": str(allocation.allocation.status),
                "allocation_residual_norm": float(allocation.achieved_controlled_residual_norm),
                "saturation_count": len(wrench_saturated) + len(position_saturated) + len(rate_limited),
                "integral_roll_error_rad_s": float(controller.integral_error["roll_error_rad"]),
                "integral_pitch_error_rad_s": float(controller.integral_error["pitch_error_rad"]),
                "integral_yaw_error_rad_s": float(controller.integral_error["yaw_error_rad"]),
                "integral_vertical_speed_m_s_s": float(controller.integral_error["w_m_s"]),
                "source_mass_kg": mass_kg,
            }
            for rotor_index, name in enumerate(design.projection.effector_names, start=1):
                row[f"rotor_{rotor_index}_speed_rad_s"] = float(previous_effectors[name])
            rows.append(row)
    return rows
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    """Bound one finite controller-reference coordinate."""

    return max(lower, min(upper, value))
    ####


def _slew_output_reference(
    current: dict[str, float],
    target: Mapping[str, float],
    dt_s: float,
) -> None:
    """Rate-limit outer guidance outputs before the source-local LQI layer."""

    maximum_rates = {
        "roll_error_rad": 1.0,
        "pitch_error_rad": 1.0,
        "yaw_error_rad": 0.35,
    }
    for name, maximum_rate in maximum_rates.items():
        current[name] += _clamp(
            float(target[name]) - current[name],
            -maximum_rate * dt_s,
            maximum_rate * dt_s,
        )
    ####


def _world_velocity(state: Mapping[str, float]) -> tuple[float, float]:
    """Resolve source-local horizontal velocity into the initial hover frame."""

    yaw = float(state["yaw_error_rad"])
    u = float(state["u_m_s"])
    v = float(state["v_m_s"])
    return (math.cos(yaw) * u - math.sin(yaw) * v, math.sin(yaw) * u + math.cos(yaw) * v)
    ####


def _rows(validation: PhysicalWrenchLqiValidation) -> list[dict[str, object]]:
    """Flatten shared LQI validation into Hummingbird public truth telemetry."""

    rows: list[dict[str, object]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = _mapping(payload.get("state"), "physical LQI sample state")
        requested = _mapping(payload.get("requested_wrench"), "physical LQI requested wrench")
        achieved = _mapping(payload.get("achieved_wrench"), "physical LQI achieved wrench")
        residual = _mapping(payload.get("achieved_residual"), "physical LQI achieved residual")
        actual = _mapping(payload.get("actual_effectors"), "physical LQI actual effectors")
        integral = _mapping(payload.get("lqi_integral_error"), "physical LQI integral error")
        wrench_saturated = _strings(payload.get("lqi_wrench_saturated"), "physical LQI wrench saturation")
        position_saturated = _strings(payload.get("position_saturated"), "physical LQI position saturation")
        rate_limited = _strings(payload.get("rate_limited"), "physical LQI rate limitation")
        row: dict[str, object] = {
            "time_s": _number(payload, "time_s"),
            "position_local_m": [0.0, 0.0, 0.0],
            "velocity_local_m_s": [
                _number(state, "u_m_s"),
                _number(state, "v_m_s"),
                _number(state, "w_m_s"),
            ],
            **{name: _number(state, name) for name in _STATE_NAMES},
            **{f"requested_moment_{axis}_nm": _number(requested, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
            **{f"achieved_moment_{axis}_nm": _number(achieved, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
            **{f"residual_moment_{axis}_nm": _number(residual, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
            "allocation_status": _text(payload, "allocation_status"),
            "allocation_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
            "saturation_count": len(wrench_saturated) + len(position_saturated) + len(rate_limited),
            "integral_roll_error_rad_s": _number(integral, "roll_error_rad"),
            "integral_pitch_error_rad_s": _number(integral, "pitch_error_rad"),
            "integral_yaw_error_rad_s": _number(integral, "yaw_error_rad"),
        }
        for rotor_index, name in enumerate(validation.design.projection.effector_names, start=1):
            row[f"rotor_{rotor_index}_speed_rad_s"] = _number(actual, name)
        rows.append(row)
    return rows
    ####


def _local_envelope(rows: list[dict[str, object]]) -> dict[str, object]:
    """Validate the declared source-hover neighbourhood and motor bounds."""

    checks: list[dict[str, object]] = []
    for name, limit in _LOCAL_LIMITS.items():
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": -limit, "maximum": limit}
            for row in rows
            if not _within(row[name], -limit, limit)
        ]
        checks.append({"channel": name, "minimum": -limit, "maximum": limit, "violations": violations, "pass": not violations})
    for rotor_index in range(1, 5):
        name = f"rotor_{rotor_index}_speed_rad_s"
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": 0.0, "maximum": 1500.0}
            for row in rows
            if not _within(row[name], 0.0, 1500.0)
        ]
        checks.append({"channel": name, "minimum": 0.0, "maximum": 1500.0, "violations": violations, "pass": not violations})
    return {
        "schema_version": 1,
        "checks": checks,
        "pass": all(item["pass"] is True for item in checks),
        "claim_boundary": "These are source-hover local state and declared motor-speed bounds, not a flight or wind envelope.",
    }
    ####


def _horizontal_local_envelope(rows: list[dict[str, object]]) -> dict[str, object]:
    """Validate the declared attitude/rate/motor bounds for horizontal-only motion."""

    limits = {
        "roll_error_rad": math.radians(15.0),
        "pitch_error_rad": math.radians(15.0),
        "yaw_error_rad": math.radians(100.0),
        "p_rad_s": math.radians(90.0),
        "q_rad_s": math.radians(90.0),
        "r_rad_s": math.radians(90.0),
    }
    checks: list[dict[str, object]] = []
    for name, limit in limits.items():
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": -limit, "maximum": limit}
            for row in rows
            if not _within(row[name], -limit, limit)
        ]
        checks.append({"channel": name, "minimum": -limit, "maximum": limit, "violations": violations, "pass": not violations})
    for rotor_index in range(1, 5):
        name = f"rotor_{rotor_index}_speed_rad_s"
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": 0.0, "maximum": 1500.0}
            for row in rows
            if not _within(row[name], 0.0, 1500.0)
        ]
        checks.append({"channel": name, "minimum": 0.0, "maximum": 1500.0, "violations": violations, "pass": not violations})
    return {
        "schema_version": 1,
        "checks": checks,
        "pass": all(item["pass"] is True for item in checks),
        "claim_boundary": (
            "These are bounded horizontal local-frame attitude, rate, and motor-speed checks. They do not establish "
            "altitude/collective, wind, electrical, contact, landing, or full-flight envelope behavior."
        ),
    }
    ####


def _vertical_local_envelope(rows: list[dict[str, object]]) -> dict[str, object]:
    """Validate the local vertical screen's state, rate, and motor bounds."""

    limits = {
        "roll_error_rad": math.radians(12.0),
        "pitch_error_rad": math.radians(12.0),
        "yaw_error_rad": math.radians(20.0),
        "w_m_s": 1.0,
        "p_rad_s": math.radians(90.0),
        "q_rad_s": math.radians(90.0),
        "r_rad_s": math.radians(90.0),
    }
    checks: list[dict[str, object]] = []
    for name, limit in limits.items():
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": -limit, "maximum": limit}
            for row in rows
            if not _within(row[name], -limit, limit)
        ]
        checks.append({"channel": name, "minimum": -limit, "maximum": limit, "violations": violations, "pass": not violations})
    for rotor_index in range(1, 5):
        name = f"rotor_{rotor_index}_speed_rad_s"
        violations = [
            {"time_s": row["time_s"], "value": row[name], "minimum": 0.0, "maximum": 1500.0}
            for row in rows
            if not _within(row[name], 0.0, 1500.0)
        ]
        checks.append({"channel": name, "minimum": 0.0, "maximum": 1500.0, "violations": violations, "pass": not violations})
    return {
        "schema_version": 1,
        "checks": checks,
        "pass": all(item["pass"] is True for item in checks),
        "claim_boundary": (
            "These are bounded source-local vertical-speed, attitude, rate, and motor-speed checks. They do not "
            "establish wind, electrical, contact, landing, or full-flight envelope behavior."
        ),
    }
    ####


def _horizontal_screen_evaluation(
    rows: list[dict[str, object]],
    *,
    finite: bool,
    envelope: Mapping[str, object],
    allocation_pass: bool,
    integrators_exercised: bool,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
) -> dict[str, object]:
    """Evaluate each exact horizontal target using committed source-plant telemetry."""

    results: list[dict[str, object]] = [
        {"id": "finite_telemetry", "status": "pass" if finite else "fail", "required": True},
        {"id": "horizontal_local_envelope", "status": "pass" if envelope.get("pass") is True else "fail", "required": True},
        {"id": "individual_rotor_allocation", "status": "pass" if allocation_pass else "fail", "required": True},
        {"id": "lqi_integrators_exercised", "status": "pass" if integrators_exercised else "fail", "required": True},
    ]
    for phase in _HORIZONTAL_PHASES:
        phase_rows = [row for row in rows if row.get("phase_id") == phase.id]
        terminal = phase_rows[-1] if phase_rows else None
        if terminal is None:
            position_error = math.inf
            yaw_error_deg = math.inf
            sustained_s = 0.0
        else:
            position = _vector3(terminal, "position_local_m")
            position_error = math.hypot(position[0] - phase.target_xy_m[0], position[1] - phase.target_xy_m[1])
            yaw_error_deg = abs(math.degrees(_angle_error(phase.target_yaw_rad, _number(terminal, "yaw_error_rad"))))
            sustained_s = _sustained_direction_s(phase_rows, phase, plan.dt_s)
        direction_pass = phase.direction_axis is None or sustained_s >= 0.8
        passed = position_error <= 0.25 and yaw_error_deg <= 8.0 and direction_pass
        results.append(
            {
                "id": phase.id,
                "status": "pass" if passed else "fail",
                "required": True,
                "actual": {
                    "terminal_position_error_m": position_error,
                    "terminal_yaw_error_deg": yaw_error_deg,
                    "sustained_direction_duration_s": sustained_s,
                },
                "tolerance": {
                    "terminal_position_error_m": 0.25,
                    "terminal_yaw_error_deg": 8.0,
                    "direction_speed_m_s": 0.30,
                    "direction_duration_s": 0.8,
                },
            }
        )
    mission_pass = all(item["status"] == "pass" for item in results)
    return {
        "schema": "taoryx.hummingbird-local-horizontal-translation-lqi-screen-evaluation/v1alpha1",
        "kind": "local_horizontal_physical_control_screen",
        "mission_pass": mission_pass,
        "results": results,
        "screen_duration_s": plan.duration_s,
        "controller_method": "lqi",
        "control_realization": "individual_rotor_source_lqi_allocation",
        "claim_boundary": (
            "The exact local-frame directional target gates establish only bounded horizontal source-plant behavior. "
            "They do not establish altitude/collective control, wind rejection, battery, contact, landing, scheduling, or qualification."
        ),
    }
    ####


def _vertical_screen_evaluation(
    rows: list[dict[str, object]],
    *,
    finite: bool,
    envelope: Mapping[str, object],
    allocation_pass: bool,
    integrators_exercised: bool,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
    mass_variation_report: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Evaluate exact vertical capture gates from committed source telemetry."""

    results: list[dict[str, object]] = [
        {"id": "finite_telemetry", "status": "pass" if finite else "fail", "required": True},
        {"id": "vertical_local_envelope", "status": "pass" if envelope.get("pass") is True else "fail", "required": True},
        {"id": "individual_rotor_force_moment_allocation", "status": "pass" if allocation_pass else "fail", "required": True},
        {"id": "lqi_integrators_exercised", "status": "pass" if integrators_exercised else "fail", "required": True},
    ]
    if mass_variation_report is not None:
        mass_variation_pass = mass_variation_report.get("pass") is True
        results.append(
            {
                "id": "fixed_nominal_lqi_mass_variation",
                "status": "pass" if mass_variation_pass else "fail",
                "required": True,
                "actual": {
                    "mass_factors": mass_variation_report.get("mass_factors"),
                    "case_count": mass_variation_report.get("case_count"),
                    "controller_policy": mass_variation_report.get("controller_policy"),
                },
                "tolerance": {
                    "required_case_status": "pass",
                    "required_factors": list(_MASS_VARIATION_FACTORS),
                },
            }
        )
    for phase in _VERTICAL_PHASES:
        phase_rows = [row for row in rows if row.get("phase_id") == phase.id]
        errors = [abs(_vector3(row, "position_local_m")[2] - phase.target_down_m) for row in phase_rows]
        speeds = [abs(_number(row, "vertical_speed_down_m_s")) for row in phase_rows]
        longest_capture = current_capture = 0
        for error, speed in zip(errors, speeds, strict=True):
            if error <= 0.25 and speed <= 0.35:
                current_capture += 1
                longest_capture = max(longest_capture, current_capture)
            else:
                current_capture = 0
        dwell_s = longest_capture * plan.dt_s
        passed = dwell_s >= 0.8
        results.append(
            {
                "id": phase.id,
                "status": "pass" if passed else "fail",
                "required": True,
                "actual": {
                    "closest_down_position_error_m": min(errors, default=math.inf),
                    "minimum_absolute_vertical_speed_m_s": min(speeds, default=math.inf),
                    "longest_capture_dwell_s": dwell_s,
                    "terminal_down_position_m": _vector3(phase_rows[-1], "position_local_m")[2] if phase_rows else math.nan,
                    "terminal_vertical_speed_down_m_s": _number(phase_rows[-1], "vertical_speed_down_m_s") if phase_rows else math.nan,
                },
                "tolerance": {
                    "down_position_radius_m": 0.25,
                    "vertical_speed_m_s": 0.35,
                    "dwell_s": 0.8,
                },
            }
        )
    mission_pass = all(item["status"] == "pass" for item in results)
    return {
        "schema": "taoryx.hummingbird-local-vertical-translation-lqi-screen-evaluation/v1alpha1",
        "kind": "local_vertical_physical_control_screen",
        "mission_pass": mission_pass,
        "results": results,
        "screen_duration_s": plan.duration_s,
        "controller_method": "lqi",
        "control_realization": "individual_rotor_source_lqi_force_moment_allocation",
        "claim_boundary": (
            "The exact source-local vertical capture gates establish only bounded climb, hover, descent, and return "
            "through collective force/moment rotor allocation. When present, the mass cases are independent source-state "
            "re-trims under one fixed nominal-mass LQI design. They do not establish wind rejection, battery, contact, "
            "landing, gain scheduling, mass transitions, or qualification."
        ),
    }
    ####


def _hover_mass_variation_report(
    plant: RuntimeRigidBodyLocalPlant,
    nominal_trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
    *,
    nominal_validation: PhysicalWrenchLqiValidation,
    nominal_rows: list[dict[str, object]],
) -> dict[str, object]:
    """Screen source-hover recovery under fixed-design, independently re-trimmed masses.

    This stays deliberately narrower than a gain schedule: the nominal design
    is retained exactly while each source mass has its own hover trim.  The
    report supplies the source evidence from which the endpoint-standard
    robustness artifact is projected.
    """

    nominal_mass_kg = float(plant.source_state.mass)
    cases: list[dict[str, object]] = []
    for factor in _MASS_VARIATION_FACTORS:
        case_mass_kg = nominal_mass_kg * factor
        if factor == 1.0:
            case_plant = plant
            trim = nominal_trim
            validation = nominal_validation
            rows = nominal_rows
        else:
            case_plant = replace(
                plant,
                source_state=replace(plant.source_state, mass=case_mass_kg),
            )
            trim = case_plant.trim(case_plant.source_local_state, case_plant.source_effectors)
            if not trim.success:
                cases.append(
                    {
                        "id": f"mass-{factor:.2f}x",
                        "parameters": {"mass_factor": factor},
                        "mass_factor": factor,
                        "source_mass_kg": case_mass_kg,
                        "status": "fail",
                        "trim": trim.as_dict(),
                        "metrics": {
                            "final_attitude_rate_error_fraction": math.inf,
                            "saturation_fraction": math.inf,
                        },
                        "blockers": ["source_hover_retrim_failed"],
                    }
                )
                continue
            validation, rows = _run_screen(case_plant, trim, design, plan)

        finite = _finite_rows(rows)
        envelope = _local_envelope(rows)
        allocation_pass = all(
            row["allocation_status"] in {"feasible", "feasible_near_limit"}
            and _saturation_count(row) == 0
            for row in rows
        )
        integrators_exercised = any(
            abs(_number(row, name)) > 1.0e-10
            for row in rows
            for name in _INTEGRAL_NAMES
        )
        initial_error = _attitude_rate_error_norm(validation.initial_state)
        final_error = _attitude_rate_error_norm(validation.final_state)
        final_error_fraction = final_error / initial_error if initial_error > 0.0 else math.inf
        saturation_fraction = (
            sum(_saturation_count(row) > 0 for row in rows) / len(rows)
            if rows
            else math.inf
        )
        recovery_pass = final_error < initial_error
        screen_pass = finite and bool(envelope["pass"]) and allocation_pass and integrators_exercised and recovery_pass
        cases.append(
            {
                "id": f"mass-{factor:.2f}x",
                "parameters": {"mass_factor": factor},
                "mass_factor": factor,
                "source_mass_kg": case_mass_kg,
                "status": "pass" if screen_pass else "fail",
                "trim": trim.as_dict(),
                "finite_telemetry": finite,
                "local_envelope_pass": envelope.get("pass") is True,
                "allocation_pass": allocation_pass,
                "integrators_exercised": integrators_exercised,
                "metrics": {
                    "initial_attitude_rate_error_norm": initial_error,
                    "final_attitude_rate_error_norm": final_error,
                    "final_attitude_rate_error_fraction": final_error_fraction,
                    "saturation_fraction": saturation_fraction,
                },
            }
        )
    passed = len(cases) == len(_MASS_VARIATION_FACTORS) and all(case["status"] == "pass" for case in cases)
    return {
        "schema": "taoryx.hummingbird-local-hover-lqi-mass-variation/v1alpha1",
        "status": "passed" if passed else "failed",
        "pass": passed,
        "mass_factors": list(_MASS_VARIATION_FACTORS),
        "case_count": len(cases),
        "nominal_source_mass_kg": nominal_mass_kg,
        "controller_policy": "one_fixed_nominal_mass_lqi_design_across_each_retrimmed_case",
        "controller_design_mass_kg": nominal_mass_kg,
        "cases": cases,
        "claim_boundary": (
            "This is a discrete 85/100/115 percent source-mass attitude/rate recovery screen with an independently "
            "re-trimmed plant and one fixed nominal-mass LQI design. It does not establish gain scheduling, an in-flight "
            "mass transition, payload-envelope coverage, wind rejection, translation, or qualification."
        ),
    }
    ####


def _hover_mass_variation_endpoint_artifact(report: Mapping[str, object]) -> dict[str, object]:
    """Project source-owned hover cases into the generic endpoint robustness format."""

    raw_cases = report.get("cases")
    if not isinstance(raw_cases, list):
        raise ValueError("Hummingbird hover mass-variation report must contain a cases list")
    cases: list[dict[str, object]] = []
    for raw_case in raw_cases:
        if not isinstance(raw_case, Mapping):
            raise ValueError("Hummingbird hover mass-variation cases must be mappings")
        identifier = raw_case.get("id")
        parameters = raw_case.get("parameters")
        metrics = raw_case.get("metrics")
        status = raw_case.get("status")
        if not isinstance(identifier, str) or not isinstance(parameters, Mapping) or not isinstance(metrics, Mapping):
            raise ValueError("Hummingbird hover mass-variation case lacks endpoint fields")
        cases.append(
            {
                "id": identifier,
                "parameters": dict(parameters),
                "status": status,
                "metrics": {
                    "final_attitude_rate_error_fraction": metrics.get("final_attitude_rate_error_fraction"),
                    "saturation_fraction": metrics.get("saturation_fraction"),
                },
                "source_mass_kg": raw_case.get("source_mass_kg"),
                "initial_attitude_rate_error_norm": metrics.get("initial_attitude_rate_error_norm"),
                "final_attitude_rate_error_norm": metrics.get("final_attitude_rate_error_norm"),
            }
        )
    return {
        "schema": "taoryx.endpoint-robustness-screen/v1alpha1",
        "id": "hummingbird-hover-fixed-lqi-mass-variation",
        "kind": "mass_variation",
        "status": "pass" if report.get("pass") is True else "fail",
        "pass": report.get("pass") is True,
        "cases": cases,
        "controller_policy": report.get("controller_policy"),
        "claim_boundary": report.get("claim_boundary"),
    }
    ####


def _vertical_mass_variation_report(
    plant: RuntimeRigidBodyLocalPlant,
    nominal_trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
    *,
    nominal_rows: list[dict[str, object]],
) -> dict[str, object]:
    """Retain discrete source-mass cases under one fixed nominal LQI design.

    The plant is independently re-trimmed for each declared mass.  The LQI
    design is deliberately *not* regenerated: this exercises the selected
    nominal controller against a bounded mass mismatch while keeping the
    re-trim and control assumptions visible.  It is not a gain schedule.
    """

    nominal_mass_kg = float(plant.source_state.mass)
    cases: list[dict[str, object]] = []
    for factor in _MASS_VARIATION_FACTORS:
        case_mass_kg = nominal_mass_kg * factor
        if factor == 1.0:
            case_plant = plant
            trim = nominal_trim
            rows = nominal_rows
        else:
            case_plant = replace(
                plant,
                source_state=replace(plant.source_state, mass=case_mass_kg),
            )
            trim = case_plant.trim(case_plant.source_local_state, case_plant.source_effectors)
            if not trim.success:
                cases.append(
                    {
                        "id": f"mass-{factor:g}x",
                        "mass_factor": factor,
                        "source_mass_kg": case_mass_kg,
                        "status": "fail",
                        "trim": trim.as_dict(),
                        "blockers": ["source_local_retrim_failed"],
                    }
                )
                continue
            rows = _run_vertical_translation_screen(case_plant, trim, design, plan)

        finite = _finite_rows(rows)
        envelope = _vertical_local_envelope(rows)
        allocation_pass = all(
            row["allocation_status"] in {"feasible", "feasible_near_limit"}
            and _saturation_count(row) == 0
            for row in rows
        )
        integrators_exercised = any(
            abs(_number(row, name)) > 1.0e-10
            for row in rows
            for name in (*_INTEGRAL_NAMES, "integral_vertical_speed_m_s_s")
        )
        evaluation = _vertical_screen_evaluation(
            rows,
            finite=finite,
            envelope=envelope,
            allocation_pass=allocation_pass,
            integrators_exercised=integrators_exercised,
            plan=plan,
        )
        passed = evaluation["mission_pass"] is True
        cases.append(
            {
                "id": f"mass-{factor:g}x",
                "mass_factor": factor,
                "source_mass_kg": case_mass_kg,
                "status": "pass" if passed else "fail",
                "trim": trim.as_dict(),
                "finite_telemetry": finite,
                "local_envelope_pass": envelope.get("pass") is True,
                "allocation_pass": allocation_pass,
                "integrators_exercised": integrators_exercised,
                "vertical_capture": evaluation,
            }
        )
    passed = len(cases) == len(_MASS_VARIATION_FACTORS) and all(case["status"] == "pass" for case in cases)
    return {
        "schema": "taoryx.hummingbird-local-vertical-lqi-mass-variation/v1alpha1",
        "status": "passed" if passed else "failed",
        "pass": passed,
        "mass_factors": list(_MASS_VARIATION_FACTORS),
        "case_count": len(cases),
        "nominal_source_mass_kg": nominal_mass_kg,
        "controller_policy": "one_fixed_nominal_mass_lqi_design_across_each_retrimmed_case",
        "controller_design_mass_kg": nominal_mass_kg,
        "cases": cases,
        "claim_boundary": (
            "This is a discrete 85/100/115 percent source-mass local vertical capture screen with an independently "
            "re-trimmed plant and one fixed nominal-mass LQI design. It does not establish gain scheduling, an in-flight "
            "mass transition, payload-envelope coverage, wind rejection, or qualification."
        ),
    }
    ####


def _angle_error(target: float, actual: float) -> float:
    """Return the wrapped signed heading difference in radians."""

    return (target - actual + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _sustained_direction_s(rows: list[dict[str, object]], phase: _HorizontalPhase, dt_s: float) -> float:
    """Measure sustained signed body motion for a declared directional leg."""

    if phase.direction_axis is None:
        return 0.0
    state_name = "u_m_s" if phase.direction_axis == "u" else "v_m_s"
    longest = current = 0
    for row in rows:
        if phase.direction_sign * _number(row, state_name) >= 0.30:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest * dt_s
    ####


def _screen_evaluation(
    finite: bool,
    envelope: Mapping[str, object],
    allocation_pass: bool,
    integrators_exercised: bool,
    recovery_pass: bool,
    screen_pass: bool,
    plan: HummingbirdLocalPhysicalControlScreenPlan,
) -> dict[str, object]:
    """Describe only the objective gates earned by this local screen."""

    return {
        "schema": "taoryx.hummingbird-local-individual-rotor-lqi-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen",
        "mission_pass": screen_pass,
        "results": [
            {"id": "finite_telemetry", "status": "pass" if finite else "fail", "required": True},
            {"id": "source_hover_envelope", "status": "pass" if envelope.get("pass") is True else "fail", "required": True},
            {"id": "individual_rotor_allocation", "status": "pass" if allocation_pass else "fail", "required": True},
            {"id": "lqi_integrators_exercised", "status": "pass" if integrators_exercised else "fail", "required": True},
            {"id": "local_attitude_rate_recovery", "status": "pass" if recovery_pass else "fail", "required": True},
        ],
        "screen_duration_s": plan.duration_s,
        "controller_method": "lqi",
        "control_realization": "individual_rotor_source_lqi_allocation",
        "claim_boundary": (
            "All gates apply only to the pinned source-hover LQI entry screen. They do not establish position, altitude, "
            "wind-bias, battery, contact, mission, scheduling, or qualification performance."
        ),
    }
    ####


def _status_samples(rows: list[dict[str, object]], mass_kg: float) -> tuple[BatchTruthSample, ...]:
    """Project actual local plant, LQI, allocator, and motor truth values."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "local_attitude_rad": [_number(row, name) for name in _ATTITUDE_NAMES],
                "body_rate_rad_s": [_number(row, name) for name in _RATE_NAMES],
                "body_velocity_m_s": [_number(row, name) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "position_local_m": _vector3(row, "position_local_m"),
                "velocity_local_m_s": _vector3(row, "velocity_local_m_s"),
                "requested_moment_body_nm": [_number(row, f"requested_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "achieved_moment_body_nm": [_number(row, f"achieved_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "residual_moment_body_nm": [_number(row, f"residual_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "wrench_status": str(row["allocation_status"]),
                "wrench_saturated": _saturation_count(row) > 0,
                "lqi_integral_error_rad_s": [
                    _number(row, "integral_roll_error_rad_s"),
                    _number(row, "integral_pitch_error_rad_s"),
                    _number(row, "integral_yaw_error_rad_s"),
                ],
                "integral_vertical_speed_m_s_s": _number(row, "integral_vertical_speed_m_s_s")
                if "integral_vertical_speed_m_s_s" in row
                else 0.0,
                "control_realization": "individual_rotor_source_lqi_force_moment_allocation"
                if "integral_vertical_speed_m_s_s" in row
                else "individual_rotor_source_lqi_allocation",
                "controller_method": "lqi",
                "physical_effector_allocation": True,
                "physical_motor_allocation": True,
                "mass_kg": mass_kg,
            }
        )
        samples.append(
            BatchTruthSample(
                time_s=_number(row, "time_s"),
                execution_status="completed" if index == len(rows) - 1 else "active",
                raw_values=raw,
            )
        )
    return tuple(samples)
    ####


def _control_samples(rows: list[dict[str, object]]) -> tuple[BatchControlSample, ...]:
    """Retain the actual four rotor speeds held for every accepted interval."""

    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for row in rows:
        time_s = _number(row, "time_s")
        effectors = {
            f"effector.rotor.{index}.speed.position": _number(row, f"rotor_{index}_speed_rad_s")
            for index in range(1, 5)
        }
        samples.append(
            BatchControlSample(
                interval_start_time_s=time_s if previous_time is None else previous_time,
                committed_truth_time_s=time_s,
                requested_actions={},
                achieved_effectors=effectors,
            )
        )
        previous_time = time_s
    return tuple(samples)
    ####


def _attitude_rate_error_norm(row: Mapping[str, object]) -> float:
    """Return a scale-free local attitude/rate norm for the screen gate."""

    return math.sqrt(sum(_number(row, name) ** 2 for name in (*_ATTITUDE_NAMES, *_RATE_NAMES)))
    ####


def _vertical_position_speed_error_norm(row: Mapping[str, object]) -> float:
    """Return a local position/speed norm for vertical-screen diagnostics only."""

    return math.hypot(_vector3(row, "position_local_m")[2], _number(row, "vertical_speed_down_m_s"))
    ####


def _finite_rows(rows: list[dict[str, object]]) -> bool:
    """Reject missing or non-finite numerical source telemetry."""

    return bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    ####


def _within(value: object, lower: float, upper: float) -> bool:
    """Return whether one finite scalar lies in the declared local bounds."""

    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value)) and lower <= float(value) <= upper
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Return a checked telemetry mapping from the shared LQI artifact."""

    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _strings(value: object, label: str) -> tuple[str, ...]:
    """Return a checked text sequence from the shared LQI artifact."""

    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must be a list of strings")
    return tuple(value)
    ####


def _number(row: Mapping[str, object], identifier: str) -> float:
    """Read one finite source-owned local control telemetry scalar."""

    value = row.get(identifier)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"Hummingbird physical-control telemetry {identifier!r} must be finite numeric")
    return float(value)
    ####


def _saturation_count(row: Mapping[str, object]) -> int:
    """Read the integer saturation count from one committed local row."""

    value = _number(row, "saturation_count")
    if not value.is_integer():
        raise ValueError("Hummingbird physical-control saturation count must be integral")
    return int(value)
    ####


def _vector3(row: Mapping[str, object], identifier: str) -> list[float]:
    """Read one finite source-owned vector-three telemetry value."""

    value = row.get(identifier)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"Hummingbird physical-control telemetry {identifier!r} must be a vector3 list")
    if any(isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)) for item in value):
        raise ValueError(f"Hummingbird physical-control telemetry {identifier!r} must contain finite numeric values")
    return [float(item) for item in value]
    ####


def _text(row: Mapping[str, object], identifier: str) -> str:
    """Read one non-empty source-owned text telemetry field."""

    value = row.get(identifier)
    if not isinstance(value, str) or not value:
        raise ValueError(f"Hummingbird physical-control telemetry {identifier!r} must be nonempty text")
    return value
    ####


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """Write complete source telemetry with deterministic columns."""

    fields = sorted({identifier for row in rows for identifier in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


def _write_json(path: Path, value: object) -> None:
    """Write one deterministic readable runtime artifact."""

    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = [
    "HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter",
    "HummingbirdLocalPhysicalControlScreenCapabilityAdapter",
    "HummingbirdLocalPhysicalControlScreenExecution",
    "HummingbirdLocalPhysicalControlScreenPlan",
    "compile_hummingbird_local_physical_control_screen",
    "execute_hummingbird_local_physical_control_screen",
    "preflight_hummingbird_local_physical_control_screen",
]
