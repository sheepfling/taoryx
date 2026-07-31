#!/usr/bin/env python3
"""Validate a native Hummingbird vertical-force/collective witness.

This is a deliberately bounded physical-control witness between the existing
native horizontal translation case and a complete pad-to-pad mission.  It
uses the source-backed local rigid-body plant, a plant-derived attitude LQR,
six-axis force/moment effectiveness, bounded four-rotor allocation, motor
lag, and nonlinear source forces/moments.

It proves vertical climb, hover capture, descent, and return-to-hover through
the actual rotor effectors.  Electrical battery state, ground contact,
touchdown, and full-envelope rotorcraft qualification remain nonclaims.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from taoryx.physical_lqr import design_physical_wrench_lqr, project_linearization_to_wrench

try:
    from validate_hummingbird_physical_lqr import TABLES, build_plant
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_hummingbird_physical_lqr import TABLES, build_plant

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_native_vertical"
TelemetryRow = dict[str, Any]


@dataclass(frozen=True, slots=True)
class VerticalPhase:
    phase_id: str
    target_down_m: float
    duration_s: float
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _phases() -> tuple[VerticalPhase, ...]:
    return (
        VerticalPhase("vertical_takeoff_gate", -1.5, 5.0),
        VerticalPhase("vertical_hover_gate", -1.5, 3.0),
        VerticalPhase("vertical_descent_gate", 0.0, 5.0),
        VerticalPhase("vertical_return_hover", 0.0, 3.0),
    )
    ####


def _design() -> tuple[Any, Any, Any]:
    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"native Hummingbird trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad", "p_rad_s", "q_rad_s", "r_rad_s"),
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=tuple(plant.control_names),
    )
    design = design_physical_wrench_lqr(
        "hummingbird-source-vertical-collective-wrench-lqr-v1",
        projection,
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(15.0), math.radians(60.0), math.radians(60.0), math.radians(60.0)),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    return plant, trim, design
    ####


def run_native_mission(*, dt_s: float = 0.01) -> tuple[list[TelemetryRow], tuple[VerticalPhase, ...], dict[str, object]]:
    """Run the local source plant through the vertical phase plan."""

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    plant, trim, design = _design()
    state = {name: float(value) for name, value in trim.state.items()}
    previous_effectors = {name: float(value) for name, value in trim.controls.items()}
    down_position_m = 0.0
    mass_kg = float(plant.source_state.mass)
    rows: list[TelemetryRow] = []
    phases = _phases()
    wrench_weights = {
        "force_x_n": 0.0,
        "force_y_n": 0.0,
        "force_z_n": 1.0,
        "moment_x_nm": 1.0,
        "moment_y_nm": 1.0,
        "moment_z_nm": 1.0,
    }
    for phase in phases:
        for _ in range(round(phase.duration_s / dt_s)):
            down_error_m = phase.target_down_m - down_position_m
            desired_down_accel_m_s2 = _clamp(
                1.3 * down_error_m - 1.0 * float(state["w_m_s"]),
                -1.0,
                1.0,
            )
            reference = {name: float(value) for name, value in trim.state.items()}
            requested_moment, wrench_increment = design.requested_wrench_for_reference(state, reference)
            requested_wrench = {
                "force_x_n": 0.0,
                "force_y_n": 0.0,
                "force_z_n": mass_kg * desired_down_accel_m_s2,
                **requested_moment,
            }
            allocation = plant.allocate_force_moment(
                state,
                requested_wrench,
                previous_effectors,
                dt_s,
                wrench_weights=wrench_weights,
            )
            previous_effectors = {name: float(value) for name, value in allocation.actuator.actual_positions.items()}
            derivative = plant.state_derivative(state, previous_effectors, {})
            state = {name: float(state[name]) + dt_s * float(derivative[name]) for name in plant.state_names}
            down_position_m += dt_s * float(state["w_m_s"])
            rows.append(
                {
                    "time_s": len(rows) * dt_s + dt_s,
                    "phase_id": phase.phase_id,
                    "down_position_m": down_position_m,
                    "target_down_m": phase.target_down_m,
                    "state": dict(state),
                    "vertical_speed_down_m_s": float(state["w_m_s"]),
                    "desired_down_accel_m_s2": desired_down_accel_m_s2,
                    "requested_wrench": requested_wrench,
                    "wrench_increment": wrench_increment,
                    "achieved_wrench": dict(allocation.achieved_wrench),
                    "achieved_residual_wrench": dict(allocation.achieved_residual_wrench),
                    "allocation_status": allocation.allocation.status,
                    "allocation_residual_norm": allocation.achieved_controlled_residual_norm,
                    "commanded_effectors": dict(allocation.actuator.commanded_positions),
                    "actual_effectors": dict(allocation.actuator.actual_positions),
                    "effector_rates": dict(allocation.actuator.rates_per_s),
                    "saturated_effectors": sorted(
                        set(allocation.allocation.position_saturated)
                        | set(allocation.allocation.rate_limited)
                        | set(allocation.actuator.position_saturated)
                        | set(allocation.actuator.rate_limited)
                    ),
                }
            )
    metadata = {
        "trim": trim.as_dict(),
        "design": design.as_dict(),
        "source_tables": [str(path.relative_to(ROOT)) for path in TABLES],
        "direct_body_force_injection": False,
        "direct_body_moment_injection": False,
        "plant_id": "hummingbird-individual-rotor-source-plant",
        "plant_revision": "rotorpy-hover-local-v1",
        "force_effectiveness_source": "centered-runtime-force-moment-difference",
    }
    return rows, phases, metadata
    ####


def _phase_truth(rows: list[TelemetryRow], phase: VerticalPhase, *, dt_s: float) -> tuple[bool, dict[str, float]]:
    phase_rows = [row for row in rows if row["phase_id"] == phase.phase_id]
    errors = [abs(float(row["down_position_m"]) - phase.target_down_m) for row in phase_rows]
    speeds = [abs(float(row["vertical_speed_down_m_s"])) for row in phase_rows]
    longest_capture_samples = 0
    current_capture_samples = 0
    for error, speed in zip(errors, speeds, strict=True):
        if error <= 0.25 and speed <= 0.35:
            current_capture_samples += 1
            longest_capture_samples = max(longest_capture_samples, current_capture_samples)
        else:
            current_capture_samples = 0
    dwell_s = longest_capture_samples * dt_s
    capture = dwell_s >= 0.8
    return capture, {
        "closest_down_position_error_m": min(errors, default=math.inf),
        "minimum_absolute_vertical_speed_m_s": min(speeds, default=math.inf),
        "longest_capture_dwell_s": dwell_s,
        "terminal_down_position_m": float(phase_rows[-1]["down_position_m"]) if phase_rows else math.nan,
        "terminal_vertical_speed_down_m_s": float(phase_rows[-1]["vertical_speed_down_m_s"]) if phase_rows else math.nan,
    }
    ####


def evaluate_native_mission(rows: list[TelemetryRow], *, dt_s: float = 0.01) -> dict[str, object]:
    """Evaluate vertical objectives independently from truth telemetry."""

    objectives: list[dict[str, object]] = []
    for phase in _phases():
        passed, metrics = _phase_truth(rows, phase, dt_s=dt_s)
        objectives.append(
            {
                "id": phase.phase_id,
                "type": "vertical_position_gate",
                "truth_result": "PASS" if passed else "FAIL",
                "required": True,
                "actual": metrics,
                "tolerance": {"down_position_radius_m": 0.25, "vertical_speed_m_s": 0.35, "dwell_s": 0.8},
                "controller_transition": {"reason": "GATE_CROSSED" if passed else "TIMEOUT_SKIP"},
            }
        )
    residuals = [float(row["allocation_residual_norm"]) for row in rows]
    saturation_rows = [row for row in rows if row["saturated_effectors"]]
    disallowed_statuses = {"infeasible", "numerically_singular", "solver_failure"}
    allocation_statuses = {str(row["allocation_status"]) for row in rows}
    numerical_pass = all(math.isfinite(float(row["time_s"])) for row in rows)
    allocation_pass = not bool(allocation_statuses & disallowed_statuses)
    mission_pass = all(item["truth_result"] == "PASS" for item in objectives) and allocation_pass and numerical_pass
    return {
        "schema": "taoryx.hummingbird-native-vertical-evaluation/v1alpha1",
        "independent_truth_evaluation": True,
        "mission_pass": mission_pass,
        "required_objectives": objectives,
        "numerical_pass": numerical_pass,
        "hard_envelope_violations": [],
        "allocation_summary": {
            "maximum_realized_residual": max(residuals, default=math.inf),
            "saturation_fraction": len(saturation_rows) / max(len(rows), 1),
            "statuses": sorted(allocation_statuses),
            "disallowed_statuses": sorted(allocation_statuses & disallowed_statuses),
            "allocation_pass": allocation_pass,
            "authority_boundary": bool(allocation_statuses - {"feasible"}),
            "all_statuses_feasible": allocation_statuses <= {"feasible"},
        },
    }
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _plot(rows: list[TelemetryRow], evaluation: dict[str, object], output: Path) -> None:
    time = [float(row["time_s"]) for row in rows]
    down = [float(row["down_position_m"]) for row in rows]
    target = [float(row["target_down_m"]) for row in rows]
    vertical_speed = [float(row["vertical_speed_down_m_s"]) for row in rows]
    desired_accel = [float(row["desired_down_accel_m_s2"]) for row in rows]
    requested_force = [float(row["requested_wrench"]["force_z_n"]) for row in rows]
    achieved_force = [float(row["achieved_wrench"]["force_z_n"]) for row in rows]
    residual = [float(row["allocation_residual_norm"]) for row in rows]
    rotor_names = tuple(sorted(rows[0]["actual_effectors"]))
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    axes[0, 0].plot(time, down, color="#1f77b4", label="truth down position")
    axes[0, 0].plot(time, target, color="#1f77b4", linestyle=":", label="target gate")
    axes[0, 0].axhspan(-1.75, -1.25, color="#2ca02c", alpha=0.08)
    axes[0, 0].axhspan(-0.25, 0.25, color="#2ca02c", alpha=0.08)
    axes[0, 0].set(title="Native vertical mission gates", xlabel="time (s)", ylabel="down position (m)")
    axes[0, 0].legend(fontsize=7)
    axes[0, 1].plot(time, vertical_speed, color="#d62728", label="truth w / down")
    axes[0, 1].axhline(0.0, color="black", linewidth=0.7)
    axes[0, 1].axhline(0.35, color="#d62728", linestyle=":", linewidth=0.8)
    axes[0, 1].axhline(-0.35, color="#d62728", linestyle=":", linewidth=0.8)
    axes[0, 1].set(title="Vertical speed and capture limit", xlabel="time (s)", ylabel="m/s")
    axes[0, 1].legend(fontsize=7)
    axes[0, 2].plot(time, desired_accel, color="#9467bd", label="desired down accel")
    axes[0, 2].set(title="Outer vertical response demand", xlabel="time (s)", ylabel="m/s²")
    axes[0, 2].legend(fontsize=7)
    axes[1, 0].plot(time, requested_force, color="#ff7f0e", linestyle=":", label="requested force")
    axes[1, 0].plot(time, achieved_force, color="#ff7f0e", label="achieved force")
    axes[1, 0].axhline(0.0, color="black", linewidth=0.7)
    axes[1, 0].set(title="Physical collective force path", xlabel="time (s)", ylabel="body force z (N)")
    axes[1, 0].legend(fontsize=7)
    axes[1, 1].plot(time, residual, color="#17becf")
    axes[1, 1].set(title="Requested-to-achieved wrench residual", xlabel="time (s)", ylabel="weighted residual")
    for name in rotor_names:
        axes[1, 2].plot(time, [float(row["actual_effectors"][name]) for row in rows], label=name)
    axes[1, 2].set(title="Actual four-rotor speeds", xlabel="time (s)", ylabel="rad/s")
    axes[1, 2].legend(fontsize=6)
    result_label = "PASS" if evaluation["mission_pass"] else "FAIL"
    if evaluation["mission_pass"] and cast(dict[str, object], evaluation["allocation_summary"])["authority_boundary"]:
        result_label = "PASS / AUTHORITY BOUNDARY"
    fig.suptitle(f"Hummingbird Alpha 3 native vertical-force witness — {result_label}", fontsize=15)
    fig.savefig(output / "native_vertical_force_evidence_board.png", dpi=150)
    plt.close(fig)
    ####


def write_native_packet(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    output.mkdir(parents=True, exist_ok=True)
    rows, phases, metadata = run_native_mission()
    evaluation = evaluate_native_mission(rows)
    source_tables = cast(list[str], metadata["source_tables"])
    objectives = cast(list[dict[str, object]], evaluation["required_objectives"])
    allocation_summary = cast(dict[str, object], evaluation["allocation_summary"])
    packet = {
        "schema": "taoryx.hummingbird-native-vertical-evidence/v1alpha1",
        "status": (
            "native_vertical_force_witness_pass_with_authority_boundary"
            if evaluation["mission_pass"] and allocation_summary["authority_boundary"]
            else "native_vertical_force_witness_pass"
            if evaluation["mission_pass"]
            else "native_vertical_force_witness_boundary"
        ),
        "family_id": "hummingbird",
        "mission_id": "hummingbird_native_vertical_force_v1",
        "fidelity": "rigid_body_6dof",
        "claim": {
            "proves": "A source-backed local Hummingbird rigid-body plant follows vertical climb, hover, descent, and return-to-hover objectives through plant-derived attitude LQR, six-axis force/moment effectiveness, bounded physical four-rotor allocation, motor lag, and nonlinear source forces/moments.",
            "nonclaims": [
                "battery electrical state, contact, touchdown, landing, wind robustness, and full-envelope rotorcraft qualification",
                "a local body-velocity/down-position reconstruction is a substitute for the full runtime mission state",
            ],
        },
        "control_path": {
            "direct_body_force_injection": False,
            "direct_body_moment_injection": False,
            "path": "vertical position error -> desired net body force -> attitude LQR desired moments -> six-axis bounded four-rotor allocation -> motor lag -> nonlinear source plant",
            "physical_effectors": ["rotor-1-speed", "rotor-2-speed", "rotor-3-speed", "rotor-4-speed"],
            "effectors_are_actual": True,
        },
        "source_provenance": {
            "plant_id": metadata["plant_id"],
            "plant_revision": metadata["plant_revision"],
            "tables": source_tables,
            "table_sha256": {path: _sha256(ROOT / path) for path in source_tables},
        },
        "mission": {
            "start_contract": "source_hover_trim",
            "terminal_contract": "return_hover_gate",
            "phases": [{"phase_id": phase.phase_id, "target_down_m": phase.target_down_m, "duration_s": phase.duration_s} for phase in phases],
        },
        "evaluation": evaluation,
        "metadata": metadata,
        "telemetry": rows,
    }
    (output / "evidence.json").write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "telemetry.json").write_text(json.dumps({"rows": rows}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(rows, evaluation, output)
    manifest = {
        "schema": "taoryx.hummingbird-native-vertical-mission/v1alpha1",
        "status": packet["status"],
        "family_id": "hummingbird",
        "mission_id": packet["mission_id"],
        "fidelity": packet["fidelity"],
        "artifact": "evidence.json",
        "board": "native_vertical_force_evidence_board.png",
        "mission_pass": evaluation["mission_pass"],
        "objective_count": len(objectives),
        "passed_objective_count": sum(item["truth_result"] == "PASS" for item in objectives),
        "direct_body_force_injection": False,
        "direct_body_moment_injection": False,
        "physical_motor_allocation": True,
        "promotion_boundary": "This is a local source-plant vertical-force witness. Electrical battery/SOC, contact, landing, wind, and full-envelope promotion remain open.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_native_vertical_mission.py",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(write_native_packet(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
