#!/usr/bin/env python3
"""Replay scheduled F-16 physical control through a bounded local transition.

The node validator proves four source-retrimmed physical plants.  This tool
adds a deliberately narrower transition witness: between adjacent nodes it
linearly blends the validated local effectiveness matrices and evaluates the
same source nonlinear derivative at the interpolated altitude/pitch.  Every
sample still passes the scheduled wrench through bounded elevator, aileron,
rudder, and throttle allocation before integrating the local state.

This is not a new aerodynamic database or a full-envelope flight simulation.
The interpolation policy and its claim boundary are recorded in the artifact.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import (
    EffectorEffectiveness,
    PhysicalAllocationStep,
    allocate_and_advance_wrench,
)
from taoryx.physical_lqr import (
    PhysicalWrenchLqrSchedule,
    PhysicalWrenchLqrScheduleNode,
    run_scheduled_physical_wrench_transition,
)
from taoryx.trajectory import F16ReferencePhysicalPlant, load_f16_reference_plant

try:
    from validate_f16_physical_schedule import POINT_IDS, _build_node, _load_limits
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_f16_physical_schedule import POINT_IDS, _build_node, _load_limits

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_physical_schedule_transition"
DT_S = 0.05
DURATION_S = 60.0
NORMALIZED_RECOVERY_THRESHOLD = 0.25
CASES: dict[str, tuple[str, str, dict[str, float]]] = {
    "upward_q_perturbation": (POINT_IDS[0], POINT_IDS[-1], {"q_rad_s": 0.0004}),
    "upward_coupled_reversal": (POINT_IDS[0], POINT_IDS[-1], {"u_m_s": -0.1, "w_m_s": -0.02, "q_rad_s": -0.0004}),
    "downward_q_perturbation": (POINT_IDS[-1], POINT_IDS[0], {"q_rad_s": -0.0004}),
    "downward_coupled_reversal": (POINT_IDS[-1], POINT_IDS[0], {"u_m_s": 0.1, "w_m_s": 0.02, "q_rad_s": 0.0004}),
}


def _blend(lower: float, upper: float, fraction: float) -> float:
    """Linearly interpolate two finite scalars."""

    return (1.0 - fraction) * float(lower) + fraction * float(upper)
    ####


def _mapping_blend(lower: dict[str, float], upper: dict[str, float], fraction: float) -> dict[str, float]:
    """Linearly interpolate one compatible named vector."""

    if set(lower) != set(upper):
        raise ValueError("scheduled F-16 mappings must have identical names")
    return {name: _blend(lower[name], upper[name], fraction) for name in lower}
    ####


@dataclass(frozen=True, slots=True)
class _BlendedF16Plant:
    """One explicit local interpolation between two validated source plants."""

    lower: F16ReferencePhysicalPlant
    upper: F16ReferencePhysicalPlant
    fraction: float
    altitude_m: float
    trim_pitch_rad: float
    preferred_effectors: dict[str, float]

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the shared local state ordering."""

        return self.lower.state_names
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the shared physical effector ordering."""

        return self.lower.control_names
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Blend endpoint source derivatives at the current schedule state."""

        altitude = float(environment.get("altitude_m", self.altitude_m))
        pitch = float(environment.get("trim_pitch_rad", self.trim_pitch_rad))
        lower = self.lower.state_derivative(state, effectors, {"altitude_m": altitude, "trim_pitch_rad": pitch})
        upper = self.upper.state_derivative(state, effectors, {"altitude_m": altitude, "trim_pitch_rad": pitch})
        return {name: _blend(float(lower[name]), float(upper[name]), self.fraction) for name in self.state_names}
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Blend validated endpoint source-load effectiveness matrices."""

        lower = self.lower.effectiveness(state, effectors)
        upper = self.upper.effectiveness(state, effectors)
        matrix = tuple(
            tuple(_blend(lower.matrix[row][column], upper.matrix[row][column], self.fraction) for column in range(len(self.control_names)))
            for row in range(len(lower.wrench_names))
        )
        return EffectorEffectiveness(
            wrench_names=lower.wrench_names,
            effector_names=lower.effector_names,
            matrix=matrix,
            reference_wrench=_mapping_blend(dict(lower.reference_wrench), dict(upper.reference_wrench), self.fraction),
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
        """Allocate the scheduled demand through actual bounded effectors."""

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


def _fraction_for_coordinate(schedule: PhysicalWrenchLqrSchedule, coordinate: float) -> tuple[int, int, float]:
    """Return endpoint indices and interpolation fraction for a coordinate."""

    coordinates = [node.coordinate for node in schedule.nodes]
    if coordinate <= coordinates[0]:
        return 0, 0, 0.0
    if coordinate >= coordinates[-1]:
        last = len(coordinates) - 1
        return last, last, 0.0
    for index, (lower, upper) in enumerate(zip(coordinates[:-1], coordinates[1:], strict=True)):
        if lower <= coordinate <= upper:
            return index, index + 1, (coordinate - lower) / (upper - lower)
    raise RuntimeError("F-16 schedule coordinate did not bracket an operating node")
    ####


def _make_plant(
    schedule: PhysicalWrenchLqrSchedule,
    adapters: list[F16ReferencePhysicalPlant],
    trim_states: list[dict[str, float]],
    trim_pitches: list[float],
    coordinate: float,
) -> _BlendedF16Plant:
    """Resolve the explicit source-node blend at one schedule coordinate."""

    lower_index, upper_index, fraction = _fraction_for_coordinate(schedule, coordinate)
    preferred = _mapping_blend(
        dict(schedule.nodes[lower_index].design.projection.trim_effectors),
        dict(schedule.nodes[upper_index].design.projection.trim_effectors),
        fraction,
    )
    return _BlendedF16Plant(
        adapters[lower_index],
        adapters[upper_index],
        fraction,
        coordinate,
        _blend(trim_pitches[lower_index], trim_pitches[upper_index], fraction),
        preferred,
    )
    ####


def _run_case(
    schedule: PhysicalWrenchLqrSchedule,
    adapters: list[F16ReferencePhysicalPlant],
    trim_states: list[dict[str, float]],
    trim_pitches: list[float],
    trim_effectors: list[dict[str, float]],
    start_coordinate: float,
    end_coordinate: float,
    perturbation: dict[str, float],
) -> dict[str, Any]:
    """Run one bidirectional local scheduled transition."""

    start_index, _, _ = _fraction_for_coordinate(schedule, start_coordinate)
    state = dict(trim_states[start_index])
    actual_effectors = dict(trim_effectors[start_index])
    def plant_for_coordinate(coordinate: float) -> _BlendedF16Plant:
        return _make_plant(schedule, adapters, trim_states, trim_pitches, coordinate)

    def environment_for_coordinate(coordinate: float) -> dict[str, float]:
        plant = plant_for_coordinate(coordinate)
        return {"altitude_m": plant.altitude_m, "trim_pitch_rad": plant.trim_pitch_rad}

    result = run_scheduled_physical_wrench_transition(
        schedule,
        start_coordinate=start_coordinate,
        end_coordinate=end_coordinate,
        initial_state=state,
        initial_effectors=actual_effectors,
        plant_for_coordinate=plant_for_coordinate,
        state_scales=schedule.nodes[start_index].design.state_scales,
        duration_s=DURATION_S,
        dt_s=DT_S,
        perturbation=perturbation,
        environment_for_coordinate=environment_for_coordinate,
        recovery_threshold=NORMALIZED_RECOVERY_THRESHOLD,
        minimum_final_norm=0.05,
        sample_stride_steps=60,
    )
    result["start_coordinate_m"] = result.pop("start_coordinate")
    result["end_coordinate_m"] = result.pop("end_coordinate")
    result["plant_policy"] = "linear_blend_of_validated_source_endpoint_derivatives_and_effectiveness; actual bounded effectors at every sample"
    return result
    ####


def build_transition(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the deterministic F-16 schedule-transition evidence packet."""

    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    catalog = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/qualification/operating-points.yaml").read_text(encoding="utf-8")
    )
    catalog_by_id = {str(point["id"]): point for point in catalog["points"]}
    limits = _load_limits()
    nodes = []
    adapters: list[F16ReferencePhysicalPlant] = []
    trim_states: list[dict[str, float]] = []
    trim_pitches: list[float] = []
    trim_effectors: list[dict[str, float]] = []
    for point_id in POINT_IDS:
        operating_point, trim, adapter, design = _build_node(source, limits, catalog_by_id[point_id])
        nodes.append(PhysicalWrenchLqrScheduleNode(operating_point.altitude_m, design))
        adapters.append(adapter)
        trim_states.append({name: float(value) for name, value in trim.state.items()})
        trim_pitches.append(float(operating_point.trim_pitch_rad))
        trim_effectors.append({name: float(value) for name, value in trim.controls.items()})
    schedule = PhysicalWrenchLqrSchedule(tuple(nodes))
    coordinate_by_id = {point_id: schedule.nodes[index].coordinate for index, point_id in enumerate(POINT_IDS)}
    cases = {
        case_id: _run_case(
            schedule,
            adapters,
            trim_states,
            trim_pitches,
            trim_effectors,
            coordinate_by_id[start_id],
            coordinate_by_id[end_id],
            perturbation,
        )
        for case_id, (start_id, end_id, perturbation) in CASES.items()
    }
    report: dict[str, Any] = {
        "schema": "taoryx.f16-physical-effector-schedule-transition/v1alpha1",
        "status": "F16_physical_effector_schedule_transition_complete" if all(case["passed"] for case in cases.values()) else "F16_physical_effector_schedule_transition_boundary_recorded",
        "family_id": "reference_f16_s119",
        "source_schedule_artifact": "verification/alpha3_f16_physical_schedule/manifest.json",
        "control_path": "scheduled source-derived wrench demand -> blended endpoint effectiveness -> bounded physical effectors -> blended source nonlinear derivative",
        "direct_body_moment_injection": False,
        "cases": cases,
        "summary": {
            "case_count": len(cases),
            "passed_case_count": sum(bool(case["passed"]) for case in cases.values()),
            "failed_case_count": sum(not bool(case["passed"]) for case in cases.values()),
            "all_allocation_statuses": sorted({status for case in cases.values() for status in case["allocation_statuses"]}),
            "maximum_controlled_allocation_residual": max(float(case["maximum_controlled_allocation_residual"]) for case in cases.values()),
        },
        "claim_boundary": "Local scheduled-transition evidence only. Endpoint source derivatives and effectiveness are linearly blended by explicit policy; no new aerodynamic table, servo certification, wind robustness, statistical reliability, or full-envelope flight-control qualification is claimed.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_physical_schedule_transition.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Build the F-16 scheduled-transition evidence packet."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_transition(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "F16_physical_effector_schedule_transition_complete" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
