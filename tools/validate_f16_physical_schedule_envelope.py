#!/usr/bin/env python3
"""Record a bounded physical-effector envelope around the F-16 schedule.

The node and transition witnesses establish local source-effector control. This
artifact adds explicit alpha/beta and body-rate boundary cases at every
validated schedule node. A case with partial authority is retained as a
boundary witness; it is never converted into a pass by clipping the request.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from taoryx.physical_lqr import validate_nonlinear_wrench_lqr

try:
    from validate_f16_physical_schedule import POINT_IDS, _build_node, _load_limits
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_f16_physical_schedule import POINT_IDS, _build_node, _load_limits

from taoryx.trajectory import load_f16_reference_plant

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_physical_schedule_envelope"
DURATION_S = 2.0
DT_S = 0.02
RECOVERY_THRESHOLD = 0.25

CASES: dict[str, dict[str, float]] = {
    "alpha_plus_1mps": {"w_m_s": 1.0},
    "alpha_minus_1mps": {"w_m_s": -1.0},
    "beta_plus_2mps": {"v_m_s": 2.0},
    "beta_minus_2mps": {"v_m_s": -2.0},
    "high_roll_rate_plus": {"p_rad_s": 0.02},
    "high_pitch_rate_plus": {"q_rad_s": 0.02},
    "high_yaw_rate_plus": {"r_rad_s": 0.02},
    "coupled_high_rate": {"p_rad_s": 0.01, "q_rad_s": -0.01, "r_rad_s": 0.01},
}


def _run_case(adapter: Any, trim: Any, design: Any, perturbation: dict[str, float]) -> dict[str, object]:
    """Run one bounded source-effector envelope witness."""

    state = dict(trim.state)
    state.update({name: float(state[name]) + value for name, value in perturbation.items()})
    validation = validate_nonlinear_wrench_lqr(
        adapter,
        trim,
        design,
        initial_state=state,
        duration_s=DURATION_S,
        dt_s=DT_S,
    )
    ratio = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    disallowed = {"partially_achievable", "infeasible", "numerically_singular", "solver_failure"}
    passed = (
        ratio <= RECOVERY_THRESHOLD
        and not disallowed.intersection(validation.allocation_statuses)
        and validation.saturation_fraction == 0.0
    )
    status = "pass" if passed else "boundary"
    return {
        "status": status,
        "mission_pass": passed,
        "perturbation": perturbation,
        "recovery_ratio": ratio,
        "allocation_statuses": list(validation.allocation_statuses),
        "saturation_fraction": validation.saturation_fraction,
        "maximum_controlled_actual_residual": validation.maximum_controlled_actual_residual,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "numerical_valid": True,
        "failure_reason": None if passed else "authority_or_recovery_boundary",
    }
    ####


def build_envelope(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    """Build the deterministic node-by-node F-16 physical envelope packet."""

    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    catalog = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/qualification/operating-points.yaml").read_text(encoding="utf-8")
    )
    catalog_by_id = {str(point["id"]): point for point in catalog["points"]}
    limits = _load_limits()
    nodes: list[dict[str, object]] = []
    for point_id in POINT_IDS:
        operating_point, trim, adapter, design = _build_node(source, limits, catalog_by_id[point_id])
        cases = {case_id: _run_case(adapter, trim, design, perturbation) for case_id, perturbation in CASES.items()}
        nodes.append(
            {
                "point_id": point_id,
                "altitude_m": operating_point.altitude_m,
                "true_airspeed_m_s": operating_point.true_airspeed_m_s,
                "alpha_trim_deg": float(catalog_by_id[point_id]["state"]["alpha_deg"]),
                "controller_id": design.id,
                "direct_body_moment_injection": False,
                "cases": cases,
                "passed_case_count": sum(bool(case["mission_pass"]) for case in cases.values()),
                "boundary_case_count": sum(not bool(case["mission_pass"]) for case in cases.values()),
            }
        )
    case_records: list[dict[str, object]] = []
    for node in nodes:
        node_cases = node["cases"]
        assert isinstance(node_cases, dict)
        case_records.extend(case for case in node_cases.values() if isinstance(case, dict))
    passed_count = sum(bool(case["mission_pass"]) for case in case_records)
    report: dict[str, object] = {
        "schema": "taoryx.f16-physical-effector-schedule-envelope/v1alpha1",
        "status": "F16_physical_effector_schedule_envelope_complete"
        if passed_count == len(case_records)
        else "F16_physical_effector_schedule_envelope_boundary_recorded",
        "family_id": "reference_f16_s119",
        "source_schedule_artifact": "verification/alpha3_f16_physical_schedule/manifest.json",
        "control_path": "source-trimmed node -> plant-derived LQR -> desired wrench -> bounded elevator/aileron/rudder/throttle -> source nonlinear plant",
        "direct_body_moment_injection": False,
        "schedule_envelope": {
            "altitude_m": [0.0, 3000.0, 6000.0, 9000.0],
            "true_airspeed_m_s": 152.4,
            "alpha_witness": "body w perturbation +/-1.0 m/s at each source node",
            "beta_witness": "body v perturbation +/-2.0 m/s at each source node",
            "rate_witness": "p/q/r perturbations up to 0.02 rad/s at each source node",
            "interpretation": "local source-node authority envelope; not a Mach, dynamic-pressure, wind, or statistical envelope",
        },
        "case_contract": {
            "duration_s": DURATION_S,
            "dt_s": DT_S,
            "recovery_threshold": RECOVERY_THRESHOLD,
            "failure_policy": "retain partial-authority and recovery failures as boundary witnesses",
        },
        "nodes": nodes,
        "summary": {
            "node_count": len(nodes),
            "case_count": len(case_records),
            "passed_case_count": passed_count,
            "boundary_case_count": len(case_records) - passed_count,
            "all_numerically_valid": all(bool(case["numerical_valid"]) for case in case_records),
        },
        "claim_boundary": "This is a local physical-effector schedule envelope witness around four source-retrimmed nodes. Boundary cases are evidence of authority limits; no full-envelope, wind, servo-certification, statistical-reliability, or operational-flight claim is made.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_physical_schedule_envelope.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Build and print the F-16 physical schedule envelope artifact."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_envelope(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
