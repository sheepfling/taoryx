"""Runtime-owned source plant construction for the F-16 S-119 family.

This module owns the exact first operating-point construction used by the
F-16 adapter registry and validation tools.  It is intentionally local: the
catalog's other operating points, gain scheduling, mission translation, and
release qualification remain separate integration gates.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from functools import lru_cache
from typing import Any

import numpy as np
import yaml
from taoryx_reference_models.resources import model_resource_root

from .control_allocation import EffectorEffectiveness, EffectorLimits, PhysicalAllocationStep, allocate_and_advance_wrench
from .control_automation import ControlAutomationDeclaration
from .generic_tuning import LinearAuthorityRequirement, NormalizedLqrProfileGrid
from .physical_lqr import (
    PhysicalWrenchLqiDesign,
    PhysicalWrenchLqrDesign,
    PhysicalWrenchLqrSchedule,
    PhysicalWrenchLqrScheduleNode,
    apply_tuning_context_to_physical_wrench_lqr_design,
    design_physical_wrench_lqi,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    run_scheduled_physical_wrench_transition,
)
from .trajectory.f16_operating_points import runtime_trim_result, solve_f16_source_trim
from .trajectory.f16_reference import F16ReferencePhysicalPlant, load_f16_reference_plant
from .tuning_application import TuningApplicationContextSet
from .tuning_campaign import TuningCampaign, TuningCampaignNode

ROOT = model_resource_root()
F16_OPERATING_POINT_CATALOG = ROOT / "families/reference_f16_s119/qualification/operating-points.yaml"
F16_SOURCE_SIDECAR = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
F16_ATMOSPHERE = ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml"
F16_ACTUATOR_CONTRACT = ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml"
F16_PHYSICAL_WRENCH_LQR_PROFILE = ROOT / "families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml"
F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS = (
    "f16-sea-level-152mps",
    "f16-3km-152mps",
    "f16-6km-152mps",
    "f16-9km-152mps",
)
F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DURATION_S = 60.0
F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DT_S = 0.05
F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_RECOVERY_THRESHOLD = 0.25
F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION = 0.05
F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_CASES: dict[str, tuple[str, str, dict[str, float]]] = {
    "upward_q_perturbation": (F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[0], F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[-1], {"q_rad_s": 0.0004}),
    "upward_coupled_reversal": (
        F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[0],
        F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[-1],
        {"u_m_s": -0.1, "w_m_s": -0.02, "q_rad_s": -0.0004},
    ),
    "downward_q_perturbation": (F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[-1], F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[0], {"q_rad_s": -0.0004}),
    "downward_coupled_reversal": (
        F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[-1],
        F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS[0],
        {"u_m_s": 0.1, "w_m_s": 0.02, "q_rad_s": 0.0004},
    ),
}


@dataclass(frozen=True, slots=True)
class F16SourcePhysicalScheduleNode:
    """One independently retrimmed F-16 physical-control schedule node.

    The node deliberately keeps controller selection discrete.  It is useful
    for source-node campaigns and schedule-interior evidence, but it does not
    imply continuous gain interpolation or a transition-flight controller.
    """

    point_id: str
    altitude_m: float
    true_airspeed_m_s: float
    trim_pitch_rad: float
    plant: F16ReferencePhysicalPlant
    trim: Any
    design: PhysicalWrenchLqrDesign

    ####


@dataclass(frozen=True, slots=True)
class F16SourcePhysicalScheduleLqiNode:
    """One independently retrimmed F-16 source node with a local LQI design.

    This intentionally mirrors :class:`F16SourcePhysicalScheduleNode` rather
    than replacing it: LQR and LQI are separately selectable tuner outcomes.
    The design remains held at one exact source retrimmed condition and is not
    a continuous gain schedule.
    """

    point_id: str
    altitude_m: float
    true_airspeed_m_s: float
    trim_pitch_rad: float
    plant: F16ReferencePhysicalPlant
    trim: Any
    design: PhysicalWrenchLqiDesign

    ####


def _f16_effectors() -> dict[str, EffectorLimits]:
    """Load the declared local F-16 actuator overlay without inventing limits."""

    payload = yaml.safe_load(F16_ACTUATOR_CONTRACT.read_text(encoding="utf-8"))
    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    dynamics = payload.get("dynamics", {})
    default_time_constant = float(dynamics.get("time_constant_s", 0.0))
    limits: dict[str, EffectorLimits] = {}
    for source_name, values in payload["limits"].items():
        position = values.get("position")
        if position is None:
            position = [values["lower"], values["upper"]]
        rate = values.get("rate_per_s", values.get("rate_limit_per_s"))
        name = names[source_name]
        limits[name] = EffectorLimits(
            name,
            float(position[0]),
            float(position[1]),
            str(values.get("unit", "fraction" if source_name == "throttle" else "deg")),
            float(rate) if rate is not None else None,
            float(values.get("time_constant_s", default_time_constant)),
        )
    return limits
    ####


def f16_source_control_bounds() -> dict[str, tuple[float, float]]:
    """Return declared source-coordinate limits without allocating controls.

    The same elevator, aileron, rudder, and throttle ranges constrain the
    source-calibrated reduced F-16 controls. Returning only position bounds
    keeps native-coordinate controller saturation observable without claiming
    actuator response or physical allocation.
    """

    return {name: (limits.lower, limits.upper) for name, limits in _f16_effectors().items()}
    ####


@lru_cache(maxsize=1)
def build_f16_source_physical_plant() -> F16ReferencePhysicalPlant:
    """Build the first source-backed F-16 physical operating-point plant."""

    catalog = yaml.safe_load(F16_OPERATING_POINT_CATALOG.read_text(encoding="utf-8"))
    point = catalog["points"][0]
    source = load_f16_reference_plant(F16_SOURCE_SIDECAR, F16_ATMOSPHERE)
    resolved = solve_f16_source_trim(
        source,
        point_id=str(point["id"]),
        altitude_m=float(point["environment"]["geometric_altitude_m"]),
        true_airspeed_m_s=float(point["environment"]["true_airspeed_m_s"]),
        initial_alpha_deg=float(point["state"]["alpha_deg"]),
        initial_elevator_deg=float(point["controls"]["elevator_deg"]),
        initial_throttle_fraction=float(point["controls"]["throttle_fraction"]),
    )
    return F16ReferencePhysicalPlant(
        source,
        runtime_trim_result(resolved),
        resolved.trim_pitch_rad,
        resolved.altitude_m,
        _f16_effectors(),
    )
    ####


@lru_cache(maxsize=1)
def _f16_reference_source() -> Any:
    """Load the shared source evaluator once for all physical schedule nodes."""

    return load_f16_reference_plant(F16_SOURCE_SIDECAR, F16_ATMOSPHERE)
    ####


@lru_cache(maxsize=1)
def _f16_schedule_catalog_by_id() -> dict[str, dict[str, Any]]:
    """Load and validate the explicit four-node source schedule catalogue."""

    payload = yaml.safe_load(F16_OPERATING_POINT_CATALOG.read_text(encoding="utf-8"))
    points = payload.get("points") if isinstance(payload, dict) else None
    if not isinstance(points, list):
        raise ValueError(f"{F16_OPERATING_POINT_CATALOG} must contain a points list")
    catalog = {str(point["id"]): point for point in points if isinstance(point, dict) and isinstance(point.get("id"), str)}
    missing = [point_id for point_id in F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS if point_id not in catalog]
    if missing:
        raise ValueError(f"F-16 physical schedule catalog is missing points: {missing}")
    return catalog
    ####


@lru_cache(maxsize=None)
def build_f16_source_physical_schedule_node(point_id: str) -> F16SourcePhysicalScheduleNode:
    """Build one source-retrimmed, bounded-effector F-16 LQR node.

    This is the runtime-owned equivalent of the previously tool-local node
    construction.  The local controller uses the same checked physical-wrench
    profile at each retrimmed source point; choosing a node is explicit and
    held for the duration of a local recovery.
    """

    if point_id not in F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS:
        raise ValueError(f"unknown F-16 physical schedule point: {point_id!r}")
    point = _f16_schedule_catalog_by_id()[point_id]
    environment = point.get("environment")
    state = point.get("state")
    controls = point.get("controls")
    if not isinstance(environment, dict) or not isinstance(state, dict) or not isinstance(controls, dict):
        raise ValueError(f"F-16 physical schedule point {point_id!r} is malformed")
    resolved = solve_f16_source_trim(
        _f16_reference_source(),
        point_id=point_id,
        altitude_m=float(environment["geometric_altitude_m"]),
        true_airspeed_m_s=float(environment["true_airspeed_m_s"]),
        initial_alpha_deg=float(state["alpha_deg"]),
        initial_elevator_deg=float(controls["elevator_deg"]),
        initial_throttle_fraction=float(controls["throttle_fraction"]),
    )
    trim = runtime_trim_result(resolved)
    plant = F16ReferencePhysicalPlant(
        _f16_reference_source(),
        trim,
        resolved.trim_pitch_rad,
        resolved.altitude_m,
        _f16_effectors(),
    )
    profile = yaml.safe_load(F16_PHYSICAL_WRENCH_LQR_PROFILE.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise ValueError(f"{F16_PHYSICAL_WRENCH_LQR_PROFILE} must contain one mapping")
    projection = project_linearization_to_wrench(
        plant.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5}),
        plant.effectiveness(trim.state, trim.controls),
        state_names=tuple(str(item) for item in profile["state_names"]),
        wrench_names=plant.wrench_names,
        effector_names=tuple(str(item) for item in profile["effector_names"]),
    )
    design = design_physical_wrench_lqr(
        f"f16.physical_schedule.{point_id}",
        projection,
        q_diagonal=profile["q_diagonal"],
        r_diagonal=profile["r_diagonal"],
        state_scales=profile["state_scales"],
        wrench_scales=profile["wrench_scales"],
    )
    return F16SourcePhysicalScheduleNode(
        point_id=point_id,
        altitude_m=resolved.altitude_m,
        true_airspeed_m_s=resolved.true_airspeed_m_s,
        trim_pitch_rad=resolved.trim_pitch_rad,
        plant=plant,
        trim=trim,
        design=design,
    )
    ####


def build_f16_source_physical_schedule_nodes() -> tuple[F16SourcePhysicalScheduleNode, ...]:
    """Return every declared source schedule node in deterministic altitude order."""

    return tuple(build_f16_source_physical_schedule_node(point_id) for point_id in F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS)
    ####


@lru_cache(maxsize=len(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS))
def build_f16_source_physical_schedule_lqi_node(point_id: str) -> F16SourcePhysicalScheduleLqiNode:
    """Synthesize the selectable offset-free velocity design at one source node."""

    lqr_node = build_f16_source_physical_schedule_node(point_id)
    lqr = lqr_node.design
    design = design_physical_wrench_lqi(
        f"{lqr.id}.velocity_lqi",
        lqr.projection,
        output_names=("u_m_s", "v_m_s", "w_m_s"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        integral_q_diagonal=(0.03, 0.03, 0.03),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    return F16SourcePhysicalScheduleLqiNode(
        point_id=lqr_node.point_id,
        altitude_m=lqr_node.altitude_m,
        true_airspeed_m_s=lqr_node.true_airspeed_m_s,
        trim_pitch_rad=lqr_node.trim_pitch_rad,
        plant=lqr_node.plant,
        trim=lqr_node.trim,
        design=design,
    )
    ####


def build_f16_source_physical_schedule_lqi_nodes() -> tuple[F16SourcePhysicalScheduleLqiNode, ...]:
    """Return LQI designs at every exact source-schedule node in stable order."""

    return tuple(build_f16_source_physical_schedule_lqi_node(point_id) for point_id in F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS)
    ####


def _blend_f16_schedule_scalar(lower: float, upper: float, fraction: float) -> float:
    """Interpolate one finite source-node value under the declared transition policy."""

    return (1.0 - fraction) * float(lower) + fraction * float(upper)
    ####


def _blend_f16_schedule_mapping(
    lower: Mapping[str, float],
    upper: Mapping[str, float],
    fraction: float,
) -> dict[str, float]:
    """Interpolate compatible source-node maps without changing channel names."""

    if set(lower) != set(upper):
        raise ValueError("F-16 scheduled transition maps must have identical channel names")
    return {name: _blend_f16_schedule_scalar(lower[name], upper[name], fraction) for name in lower}
    ####


@dataclass(frozen=True, slots=True)
class _BlendedF16SourcePhysicalSchedulePlant:
    """One explicit derivative/effectiveness blend between validated source nodes.

    The generic scheduled-controller runner owns the time marching, controller
    demand, and evidence rules.  This F-16 seam owns only the auditable family
    policy: source derivatives and source effectiveness are linearly blended
    at the active altitude coordinate before demand is allocated through the
    declared elevator, aileron, rudder, and throttle overlay.
    """

    lower: F16ReferencePhysicalPlant
    upper: F16ReferencePhysicalPlant
    fraction: float
    altitude_m: float
    trim_pitch_rad: float
    preferred_effectors: Mapping[str, float]

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the shared source state ordering."""

        return self.lower.state_names
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the actual F-16 surface/throttle ordering."""

        return self.lower.control_names
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Blend source derivatives and apply an explicitly declared external moment.

        The optional pitch bias is an external plant-dynamics load, not a
        controller request or an allocator output.  Applying it after the
        source-model derivatives preserves the source force/moment path and
        keeps the robustness seam auditable at the exact point where an
        external body moment changes angular acceleration.
        """

        altitude_m = float(environment.get("altitude_m", self.altitude_m))
        trim_pitch_rad = float(environment.get("trim_pitch_rad", self.trim_pitch_rad))
        lower = self.lower.state_derivative(
            state,
            effectors,
            {"altitude_m": altitude_m, "trim_pitch_rad": trim_pitch_rad},
        )
        upper = self.upper.state_derivative(
            state,
            effectors,
            {"altitude_m": altitude_m, "trim_pitch_rad": trim_pitch_rad},
        )
        derivative = {name: _blend_f16_schedule_scalar(float(lower[name]), float(upper[name]), self.fraction) for name in self.state_names}
        external_pitch_moment_bias_nm = float(environment.get("external_pitch_moment_bias_nm", 0.0))
        if not math.isfinite(external_pitch_moment_bias_nm):
            raise ValueError("F-16 scheduled external pitch moment must be finite")
        if external_pitch_moment_bias_nm == 0.0:
            return derivative
        lower_inertia = np.asarray(self.lower.source.inertia_matrix_kg_m2, dtype=float)
        upper_inertia = np.asarray(self.upper.source.inertia_matrix_kg_m2, dtype=float)
        inertia = (1.0 - self.fraction) * lower_inertia + self.fraction * upper_inertia
        external_angular_acceleration = np.linalg.solve(
            inertia,
            np.asarray((0.0, external_pitch_moment_bias_nm, 0.0), dtype=float),
        )
        derivative["p_rad_s"] += float(external_angular_acceleration[0])
        derivative["q_rad_s"] += float(external_angular_acceleration[1])
        derivative["r_rad_s"] += float(external_angular_acceleration[2])
        return derivative
        ####

    def effectiveness(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
    ) -> EffectorEffectiveness:
        """Blend source effectiveness while preserving actual effectors."""

        lower = self.lower.effectiveness(state, effectors)
        upper = self.upper.effectiveness(state, effectors)
        matrix = tuple(
            tuple(_blend_f16_schedule_scalar(lower.matrix[row][column], upper.matrix[row][column], self.fraction) for column in range(len(self.control_names)))
            for row in range(len(lower.wrench_names))
        )
        return EffectorEffectiveness(
            wrench_names=lower.wrench_names,
            effector_names=lower.effector_names,
            matrix=matrix,
            reference_wrench=_blend_f16_schedule_mapping(
                lower.reference_wrench,
                upper.reference_wrench,
                self.fraction,
            ),
            reference_effectors=dict(effectors),
            source="linear_blend_of_validated_f16_source_effectiveness_nodes",
        )
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate a scheduled wrench only through bounded physical effectors."""

        return allocate_and_advance_wrench(
            self.effectiveness(state, previous_effectors),
            self.lower.effectors,
            desired_wrench,
            previous_effectors,
            dt_s,
            preferred_effectors=self.preferred_effectors,
            regularization=1.0e-8,
        )
        ####


def build_f16_source_physical_lqr_schedule() -> tuple[
    PhysicalWrenchLqrSchedule,
    tuple[F16SourcePhysicalScheduleNode, ...],
]:
    """Build the ordered source-node schedule used by the LQR transition screen."""

    schedule, nodes, _ = _build_f16_source_physical_lqr_schedule_with_context_set()
    return schedule, nodes
    ####


def _build_f16_source_physical_lqr_schedule_with_context_set(
    tuning_context_set: TuningApplicationContextSet | None = None,
) -> tuple[PhysicalWrenchLqrSchedule, tuple[F16SourcePhysicalScheduleNode, ...], tuple[dict[str, str], ...]]:
    """Build the schedule after applying every exact selected LQR candidate.

    A schedule interpolates four local gains, so accepting a single context
    would make its provenance ambiguous.  The optional set must name every
    retained source node before any transition replay begins.
    """

    nodes = build_f16_source_physical_schedule_nodes()
    tuning_bindings: tuple[dict[str, str], ...] = ()
    if tuning_context_set is not None:
        tuning_context_set.require_exact_nodes(tuple(node.point_id for node in nodes))
        applied_nodes: list[F16SourcePhysicalScheduleNode] = []
        receipts: list[dict[str, str]] = []
        for node in nodes:
            design, receipt = apply_tuning_context_to_physical_wrench_lqr_design(
                node.design,
                tuning_context_set.for_node(node.point_id),
            )
            applied_nodes.append(replace(node, design=design))
            receipts.append(receipt.as_dict())
        nodes = tuple(applied_nodes)
        tuning_bindings = tuple(receipts)
    schedule = PhysicalWrenchLqrSchedule(
        tuple(PhysicalWrenchLqrScheduleNode(node.altitude_m, node.design) for node in nodes)
    )
    return schedule, nodes, tuning_bindings
    ####


def _f16_source_physical_transition_plant(
    schedule: PhysicalWrenchLqrSchedule,
    nodes: tuple[F16SourcePhysicalScheduleNode, ...],
    coordinate_m: float,
) -> _BlendedF16SourcePhysicalSchedulePlant:
    """Resolve one declared source derivative/effectiveness blend by altitude."""

    lower_schedule, upper_schedule, fraction = schedule.bracket(coordinate_m)
    node_by_design_id = {node.design.id: node for node in nodes}
    try:
        lower = node_by_design_id[lower_schedule.design.id]
        upper = node_by_design_id[upper_schedule.design.id]
    except KeyError as error:  # pragma: no cover - schedule construction invariant
        raise RuntimeError("F-16 source schedule node provenance is incomplete") from error
    return _BlendedF16SourcePhysicalSchedulePlant(
        lower=lower.plant,
        upper=upper.plant,
        fraction=fraction,
        altitude_m=coordinate_m,
        trim_pitch_rad=_blend_f16_schedule_scalar(lower.trim_pitch_rad, upper.trim_pitch_rad, fraction),
        preferred_effectors=_blend_f16_schedule_mapping(
            lower.design.projection.trim_effectors,
            upper.design.projection.trim_effectors,
            fraction,
        ),
    )
    ####


def run_f16_source_physical_lqr_schedule_transition_cases(
    *,
    duration_s: float = F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DURATION_S,
    dt_s: float = F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DT_S,
    sample_stride_steps: int = 60,
    external_pitch_wrench_bias_fraction: float = 0.0,
    tuning_context_set: TuningApplicationContextSet | None = None,
) -> dict[str, object]:
    """Run the exact bounded four-case F-16 source schedule-transition campaign.

    This is a reusable plugin-level seam for the CLI evidence packet and the
    Vehicle Composition execution factory.  It does not turn a coordinate
    interpolation into a route, wind/mass robustness result, or full-envelope
    aircraft controller.
    """

    if not math.isfinite(external_pitch_wrench_bias_fraction):
        raise ValueError("F-16 scheduled external pitch wrench bias fraction must be finite")
    schedule, nodes, tuning_bindings = _build_f16_source_physical_lqr_schedule_with_context_set(tuning_context_set)
    node_by_point_id = {node.point_id: node for node in nodes}
    try:
        pitch_wrench_index = nodes[0].design.projection.wrench_names.index("total_moment_y_nm")
    except ValueError as error:  # pragma: no cover - checked controller-profile invariant
        raise RuntimeError("F-16 schedule controller does not define a pitch wrench axis") from error
    pitch_wrench_scales_nm = {float(node.design.wrench_scales[pitch_wrench_index]) for node in nodes}
    if len(pitch_wrench_scales_nm) != 1:
        raise ValueError("F-16 schedule nodes must declare one shared pitch wrench scale")
    pitch_wrench_scale_nm = pitch_wrench_scales_nm.pop()
    external_pitch_moment_bias_nm = external_pitch_wrench_bias_fraction * pitch_wrench_scale_nm

    def plant_for_coordinate(coordinate_m: float) -> _BlendedF16SourcePhysicalSchedulePlant:
        return _f16_source_physical_transition_plant(schedule, nodes, coordinate_m)

    def environment_for_coordinate(coordinate_m: float) -> dict[str, float]:
        plant = plant_for_coordinate(coordinate_m)
        return {
            "altitude_m": plant.altitude_m,
            "trim_pitch_rad": plant.trim_pitch_rad,
            "external_pitch_moment_bias_nm": external_pitch_moment_bias_nm,
        }

    cases: dict[str, dict[str, Any]] = {}
    for case_id, (start_point_id, end_point_id, perturbation) in F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_CASES.items():
        start = node_by_point_id[start_point_id]
        end = node_by_point_id[end_point_id]
        case = run_scheduled_physical_wrench_transition(
            schedule,
            start_coordinate=start.altitude_m,
            end_coordinate=end.altitude_m,
            initial_state=start.trim.state,
            initial_effectors=start.trim.controls,
            plant_for_coordinate=plant_for_coordinate,
            state_scales=start.design.state_scales,
            duration_s=duration_s,
            dt_s=dt_s,
            perturbation=perturbation,
            environment_for_coordinate=environment_for_coordinate,
            recovery_threshold=F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_RECOVERY_THRESHOLD,
            minimum_final_norm=0.05,
            sample_stride_steps=sample_stride_steps,
        )
        case["start_point_id"] = start_point_id
        case["end_point_id"] = end_point_id
        case["start_coordinate_m"] = case.pop("start_coordinate")
        case["end_coordinate_m"] = case.pop("end_coordinate")
        case["plant_policy"] = "linear_blend_of_validated_source_endpoint_derivatives_and_effectiveness; actual bounded effectors at every sample"
        cases[case_id] = case
    result: dict[str, object] = {
        "schema": "taoryx.f16-physical-effector-schedule-transition/v1alpha1",
        "status": ("pass" if all(bool(case["passed"]) for case in cases.values()) else "failed"),
        "family_id": "f16_s119",
        "controller_method": "lqr",
        "controller_selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
        "control_path": (
            "scheduled source-derived wrench demand -> blended endpoint effectiveness -> bounded physical effectors -> blended source nonlinear derivative"
        ),
        "direct_body_moment_injection": False,
        "external_dynamics": {
            "status": "applied" if external_pitch_moment_bias_nm != 0.0 else "nominal",
            "input": "external_pitch_moment_bias_nm",
            "body_moment_axis": "total_moment_y_nm",
            "pitch_wrench_scale_nm": pitch_wrench_scale_nm,
            "pitch_wrench_bias_fraction": external_pitch_wrench_bias_fraction,
            "external_pitch_moment_bias_nm": external_pitch_moment_bias_nm,
            "application": "post_source_derivative_full_inertia_angular_acceleration",
        },
        "node_ids": list(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
        "source_mass_kg": float(nodes[0].plant.source.mass_kg),
        "cases": cases,
        "summary": {
            "case_count": len(cases),
            "passed_case_count": sum(bool(case["passed"]) for case in cases.values()),
            "failed_case_count": sum(not bool(case["passed"]) for case in cases.values()),
            "all_allocation_statuses": sorted({status for case in cases.values() for status in case["allocation_statuses"]}),
            "maximum_controlled_allocation_residual": max(float(case["maximum_controlled_allocation_residual"]) for case in cases.values()),
        },
        "claim_boundary": (
            "Local scheduled-transition evidence only. Endpoint source derivatives and effectiveness are linearly blended "
            "by explicit policy. When selected, a constant external pitch moment is converted through the blended full "
            "source inertia after source derivative evaluation; it is not added to a controller request or allocator "
            "output. No new aerodynamic table, servo certification, wind robustness, mass variation, statistical "
            "reliability, navigation, or full-envelope flight-control qualification is claimed."
        ),
    }
    if tuning_bindings:
        result["tuning_bindings"] = list(tuning_bindings)
    return result
    ####


@lru_cache(maxsize=1)
def build_f16_local_physical_wrench_design() -> PhysicalWrenchLqrDesign:
    """Derive the checked local wrench controller from the owned F-16 plant.

    This is the production seam for the existing local physical-controller
    evidence.  Keeping it beside plant construction prevents public
    Composition execution from importing a developer validation tool merely
    to recover the controller profile.
    """

    plant = build_f16_source_physical_plant()
    profile = yaml.safe_load(F16_PHYSICAL_WRENCH_LQR_PROFILE.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise ValueError(f"{F16_PHYSICAL_WRENCH_LQR_PROFILE} must contain one mapping")
    trim = plant.trim_result
    state_names = tuple(str(item) for item in profile["state_names"])
    effector_names = tuple(str(item) for item in profile["effector_names"])
    projection = project_linearization_to_wrench(
        plant.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5}),
        plant.effectiveness(trim.state, trim.controls),
        state_names=state_names,
        wrench_names=plant.wrench_names,
        effector_names=effector_names,
    )
    return design_physical_wrench_lqr(
        str(profile["controller_id"]),
        projection,
        q_diagonal=profile["q_diagonal"],
        r_diagonal=profile["r_diagonal"],
        state_scales=profile["state_scales"],
        wrench_scales=profile["wrench_scales"],
    )
    ####


@lru_cache(maxsize=1)
def build_f16_local_physical_wrench_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the F-16 source-local velocity LQI candidate.

    The local source adapter exposes body velocity and rate coordinates, not
    navigation attitude states. Integrating ``u``, ``v``, and ``w`` therefore
    keeps this candidate in the same fixed-altitude, source-trim local scope
    as the LQR screen. Its command remains a four-axis desired wrench which
    must traverse the declared bounded first-order surface/throttle overlay
    and physical allocator before the nonlinear source derivative advances.
    """

    lqr = build_f16_local_physical_wrench_design()
    return design_physical_wrench_lqi(
        "f16.source_local_velocity_wrench_lqi.v1",
        lqr.projection,
        output_names=("u_m_s", "v_m_s", "w_m_s"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        integral_q_diagonal=(0.03, 0.03, 0.03),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    ####


def build_f16_source_surface_lqr_tuning_campaign() -> TuningCampaign:
    """Declare the source-trim surface-coordinate local LQR design screen.

    The physical source plant exposes velocity and rate states plus declared
    elevator, aileron, rudder, and throttle coordinates.  This shared campaign
    synthesizes candidates in that actual effector basis.  It does not replace
    the local direct-wrench comparator or establish scheduled flight control.
    """

    return ControlAutomationDeclaration(
        id="f16-source-surface-full-local",
        campaign_id="f16-source-surface-local-lqr-v1",
        family_id="f16_s119",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        node_id="source-trim-local",
        state_scales={
            "u_m_s": 50.0,
            "v_m_s": 50.0,
            "w_m_s": 50.0,
            "p_rad_s": 0.5,
            "q_rad_s": 0.5,
            "r_rad_s": 0.5,
        },
        control_scales={
            "elevator_deg": 10.0,
            "aileron_deg": 10.0,
            "rudder_deg": 10.0,
            "throttle_fraction": 0.2,
        },
        authority_state_names=("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s"),
        profile_grid_id_prefix="f16-source-surface-local",
    ).build_campaign()
    ####


def build_f16_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the exact source-trim physical-wrench LQI runtime campaign.

    The nonlinear F-16 LQI screen controls the frozen source-local velocity
    and rate projection by requesting four body-wrench coordinates, then
    allocates those requests through the bounded surface/throttle overlay.
    Keep the automatic campaign in those same state-to-wrench coordinates;
    tuning raw elevator/aileron/rudder/throttle gains would not be a candidate
    that the allocator-backed runtime could truthfully apply.
    """

    design = build_f16_local_physical_wrench_lqi_design()

    return ControlAutomationDeclaration(
        id="f16-source-trim-velocity-wrench-local",
        campaign_id="f16-source-surface-local-lqi-v1",
        family_id="f16_s119",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        node_id="source-trim-local-velocity-wrench",
        state_scales=dict(zip(design.projection.state_names, design.state_scales, strict=True)),
        control_scales=dict(zip(design.projection.wrench_names, design.wrench_scales, strict=True)),
        authority_state_names=design.projection.state_names,
        offset_free_outputs=design.result.output_names,
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        state_base_weights=design.q_diagonal,
        control_base_weights=design.r_diagonal,
        integral_base_weights=design.integral_q_diagonal,
        integral_weight_multipliers=(1.0,),
        profile_grid_id_prefix="f16-source-velocity-wrench-lqi",
    ).build_campaign()
    ####


def f16_source_physical_schedule_tuning_targets() -> dict[str, dict[str, float]]:
    """Return the explicit source-altitude selector for each retained node.

    The selector identifies a discrete source retrimmed projection for the
    common campaign.  It is not a gain-interpolation coordinate or a request
    to synthesize an unlisted operating point.
    """

    return {
        node.point_id: {"source_geometric_altitude_m": node.altitude_m}
        for node in build_f16_source_physical_schedule_lqi_nodes()
    }
    ####


def build_f16_source_physical_schedule_lqi_tuning_campaign() -> TuningCampaign:
    """Declare one exact LQI candidate per executable F-16 source schedule node.

    The scheduled interior runtime conducts independent, held-node
    recoveries.  This campaign therefore creates four separate velocity/rate
    wrench candidates in exactly those four source-derived coordinates; it
    makes no claim of continuous gain scheduling or transition control.
    """

    nodes = build_f16_source_physical_schedule_lqi_nodes()
    targets = f16_source_physical_schedule_tuning_targets()
    return TuningCampaign(
        campaign_id="f16-source-surface-schedule-lqi-v1",
        family_id="f16_s119",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        nodes=tuple(
            TuningCampaignNode(
                node_id=node.point_id,
                trim_target=targets[node.point_id],
                trim_initial_guess={},
                state_scales=node.design.state_scales,
                control_scales=node.design.wrench_scales,
                authority_requirement=LinearAuthorityRequirement(
                    f"f16-source-schedule-lqi.{node.point_id}.authority",
                    node.design.projection.state_names,
                ),
                profile_grid=NormalizedLqrProfileGrid(
                    f"f16-source-schedule-lqi.{node.point_id}",
                    state_weight_multipliers=(1.0,),
                    control_effort_multipliers=(1.0,),
                    integral_weight_multipliers=(1.0,),
                    state_base_weights=node.design.q_diagonal,
                    control_base_weights=node.design.r_diagonal,
                ),
                design_state_names=node.design.projection.state_names,
                design_control_names=node.design.projection.wrench_names,
                controller_method="lqi",
                integral_output_names=node.design.result.output_names,
                integral_q_diagonal=node.design.integral_q_diagonal,
            )
            for node in nodes
        ),
    )
    ####


def build_f16_source_physical_schedule_lqr_tuning_campaign() -> TuningCampaign:
    """Declare the exact four-node physical-wrench LQR schedule campaign.

    Every candidate has the same velocity/rate-to-wrench coordinates used by
    the scheduled interior and transition runtimes.  Applying all four is a
    prerequisite to any gain interpolation; this campaign is not a request
    to synthesize an unlisted source operating point.
    """

    nodes = build_f16_source_physical_schedule_nodes()
    targets = f16_source_physical_schedule_tuning_targets()
    return TuningCampaign(
        campaign_id="f16-source-surface-schedule-lqr-v1",
        family_id="f16_s119",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        nodes=tuple(
            TuningCampaignNode(
                node_id=node.point_id,
                trim_target=targets[node.point_id],
                trim_initial_guess={},
                state_scales=node.design.state_scales,
                control_scales=node.design.wrench_scales,
                authority_requirement=LinearAuthorityRequirement(
                    f"f16-source-schedule-lqr.{node.point_id}.authority",
                    node.design.projection.state_names,
                ),
                profile_grid=NormalizedLqrProfileGrid(
                    f"f16-source-schedule-lqr.{node.point_id}",
                    state_weight_multipliers=(1.0,),
                    control_effort_multipliers=(1.0,),
                    state_base_weights=node.design.q_diagonal,
                    control_base_weights=node.design.r_diagonal,
                ),
                design_state_names=node.design.projection.state_names,
                design_control_names=node.design.projection.wrench_names,
                controller_method="lqr",
            )
            for node in nodes
        ),
    )
    ####


__all__ = [
    "F16_ACTUATOR_CONTRACT",
    "F16_ATMOSPHERE",
    "F16_OPERATING_POINT_CATALOG",
    "F16_PHYSICAL_WRENCH_LQR_PROFILE",
    "F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS",
    "F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION",
    "F16_SOURCE_SIDECAR",
    "F16SourcePhysicalScheduleLqiNode",
    "F16SourcePhysicalScheduleNode",
    "build_f16_local_physical_wrench_lqi_design",
    "build_f16_local_physical_wrench_design",
    "build_f16_source_physical_schedule_lqi_node",
    "build_f16_source_physical_schedule_lqi_nodes",
    "build_f16_source_physical_schedule_node",
    "build_f16_source_physical_schedule_nodes",
    "build_f16_source_surface_lqi_tuning_campaign",
    "build_f16_source_physical_schedule_lqi_tuning_campaign",
    "build_f16_source_physical_schedule_lqr_tuning_campaign",
    "build_f16_source_surface_lqr_tuning_campaign",
    "build_f16_source_physical_plant",
    "f16_source_control_bounds",
    "f16_source_physical_schedule_tuning_targets",
    "run_f16_source_physical_lqr_schedule_transition_cases",
]
