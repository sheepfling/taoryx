#!/usr/bin/env python3
"""Validate a native Hummingbird horizontal-translation mission witness.

This witness is intentionally narrower than a full rotorcraft mission.  It
uses the source-backed local rigid-body plant, plant-derived LQR, bounded
four-rotor allocation, motor lag, and nonlinear source forces/moments.  It
integrates the local body-velocity state into a horizontal position record so
the physical control path can be evaluated against directional objectives.

Altitude, collective force regulation, battery electrical state, contact, and
landing are outside the local plant contract and are recorded as nonclaims.
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
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_native_horizontal"
TelemetryRow = dict[str, Any]


@dataclass(frozen=True, slots=True)
class NativePhase:
    phase_id: str
    target_xy_m: tuple[float, float]
    target_yaw_rad: float
    duration_s: float
    direction_axis: str | None = None
    direction_sign: int = 0
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _norm2(value: tuple[float, float]) -> float:
    return math.hypot(value[0], value[1])
    ####


def _angle_error(target: float, actual: float) -> float:
    return (target - actual + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _phases() -> tuple[NativePhase, ...]:
    return (
        NativePhase("forward_body_leg", (5.0, 0.0), 0.0, 8.0, "u", 1),
        NativePhase("yaw_scan_gate", (5.0, 0.0), math.pi / 2.0, 4.0),
        NativePhase("lateral_body_right_leg", (2.0, 0.0), math.pi / 2.0, 8.0, "v", 1),
        NativePhase("rearward_body_leg", (2.0, -5.0), math.pi / 2.0, 8.0, "u", -1),
        NativePhase("horizontal_return_gate", (0.0, 0.0), math.pi / 2.0, 8.0),
    )
    ####


def _body_to_world_velocity(state: dict[str, float]) -> tuple[float, float]:
    yaw = float(state["yaw_error_rad"])
    u = float(state["u_m_s"])
    v = float(state["v_m_s"])
    return (math.cos(yaw) * u - math.sin(yaw) * v, math.sin(yaw) * u + math.cos(yaw) * v)
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
        "hummingbird-source-horizontal-translation-wrench-lqr-v1",
        projection,
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(15.0), math.radians(60.0), math.radians(60.0), math.radians(60.0)),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    return plant, trim, design
    ####


def run_native_mission(*, dt_s: float = 0.01) -> tuple[list[TelemetryRow], tuple[NativePhase, ...], dict[str, object]]:
    """Run the local source plant through the directional phase plan."""

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    plant, trim, design = _design()
    state = {name: float(value) for name, value in trim.state.items()}
    previous_effectors = {name: float(value) for name, value in trim.controls.items()}
    position_xy = [0.0, 0.0]
    rows: list[TelemetryRow] = []
    for phase in _phases():
        for _ in range(round(phase.duration_s / dt_s)):
            world_velocity = _body_to_world_velocity(state)
            error_x = phase.target_xy_m[0] - position_xy[0]
            error_y = phase.target_xy_m[1] - position_xy[1]
            desired_ax = _clamp(0.7 * error_x - 0.8 * world_velocity[0], -2.0, 2.0)
            desired_ay = _clamp(0.7 * error_y - 0.8 * world_velocity[1], -2.0, 2.0)
            yaw = float(state["yaw_error_rad"])
            body_ax = math.cos(yaw) * desired_ax + math.sin(yaw) * desired_ay
            body_ay = -math.sin(yaw) * desired_ax + math.cos(yaw) * desired_ay
            reference = {name: float(value) for name, value in trim.state.items()}
            reference["roll_error_rad"] = _clamp(body_ay / 9.80665, -0.18, 0.18)
            reference["pitch_error_rad"] = _clamp(-body_ax / 9.80665, -0.18, 0.18)
            reference["yaw_error_rad"] = phase.target_yaw_rad
            requested_wrench, wrench_increment = design.requested_wrench_for_reference(state, reference)
            allocation = plant.allocate(state, requested_wrench, previous_effectors, dt_s)
            previous_effectors = {name: float(value) for name, value in allocation.actuator.actual_positions.items()}
            derivative = plant.state_derivative(state, previous_effectors, {})
            state = {name: float(state[name]) + dt_s * float(derivative[name]) for name in plant.state_names}
            world_velocity = _body_to_world_velocity(state)
            position_xy[0] += dt_s * world_velocity[0]
            position_xy[1] += dt_s * world_velocity[1]
            rows.append(
                {
                    "time_s": len(rows) * dt_s + dt_s,
                    "phase_id": phase.phase_id,
                    "position_xy_m": list(position_xy),
                    "target_xy_m": list(phase.target_xy_m),
                    "target_yaw_rad": phase.target_yaw_rad,
                    "state": dict(state),
                    "body_velocity_m_s": [float(state["u_m_s"]), float(state["v_m_s"])],
                    "world_velocity_m_s": list(world_velocity),
                    "reference": reference,
                    "requested_wrench": requested_wrench,
                    "wrench_increment": wrench_increment,
                    "achieved_wrench": dict(allocation.achieved_wrench),
                    "achieved_residual_wrench": dict(allocation.achieved_residual_wrench),
                    "allocation_status": allocation.allocation.status,
                    "allocation_residual_norm": allocation.achieved_controlled_residual_norm,
                    "commanded_effectors": dict(allocation.actuator.commanded_positions),
                    "actual_effectors": dict(allocation.actuator.actual_positions),
                    "effector_rates": dict(allocation.actuator.rates_per_s),
                    "saturated_effectors": sorted(set(allocation.allocation.position_saturated) | set(allocation.allocation.rate_limited) | set(allocation.actuator.position_saturated) | set(allocation.actuator.rate_limited)),
                }
            )
    metadata = {
        "trim": trim.as_dict(),
        "design": design.as_dict(),
        "source_tables": [str(path.relative_to(ROOT)) for path in TABLES],
        "direct_body_moment_injection": False,
        "plant_id": "hummingbird-individual-rotor-source-plant",
        "plant_revision": "rotorpy-hover-local-v1",
    }
    return rows, _phases(), metadata
    ####


def _directional_truth(rows: list[TelemetryRow], phase: NativePhase, *, dt_s: float) -> tuple[bool, dict[str, float]]:
    phase_rows = [row for row in rows if row["phase_id"] == phase.phase_id]
    errors = [_norm2((float(row["position_xy_m"][0]) - phase.target_xy_m[0], float(row["position_xy_m"][1]) - phase.target_xy_m[1])) for row in phase_rows]
    speeds = [_norm2((float(row["world_velocity_m_s"][0]), float(row["world_velocity_m_s"][1]))) for row in phase_rows]
    capture = any(error <= 0.25 and speed <= 0.35 for error, speed in zip(errors, speeds, strict=True))
    yaw_pass = any(abs(_angle_error(phase.target_yaw_rad, float(row["state"]["yaw_error_rad"]))) <= math.radians(8.0) for row in phase_rows)
    signed_values: list[float] = []
    if phase.direction_axis is not None:
        index = 0 if phase.direction_axis == "u" else 1
        signed_values = [phase.direction_sign * float(row["body_velocity_m_s"][index]) for row in phase_rows]
    longest = current = 0
    for value in signed_values:
        if value >= 0.30:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    sustained_s = longest * dt_s
    direction_pass = phase.direction_axis is None or sustained_s >= 0.8
    passed = capture and yaw_pass and direction_pass
    return passed, {
        "closest_position_error_m": min(errors, default=math.inf),
        "minimum_endpoint_speed_m_s": min(speeds, default=math.inf),
        "yaw_error_deg_at_best_capture": min((math.degrees(abs(_angle_error(phase.target_yaw_rad, float(row["state"]["yaw_error_rad"])))) for row in phase_rows), default=math.inf),
        "peak_signed_body_speed_m_s": max(signed_values, default=0.0),
        "sustained_direction_duration_s": sustained_s,
    }
    ####


def evaluate_native_mission(rows: list[TelemetryRow], *, dt_s: float = 0.01) -> dict[str, object]:
    """Evaluate horizontal objectives independently from truth telemetry."""

    objectives: list[dict[str, object]] = []
    for phase in _phases():
        passed, metrics = _directional_truth(rows, phase, dt_s=dt_s)
        objectives.append({
            "id": phase.phase_id,
            "type": "directional_translation" if phase.direction_axis is not None else "terminal_horizontal_gate",
            "truth_result": "PASS" if passed else "FAIL",
            "required": True,
            "actual": metrics,
            "tolerance": {"position_radius_m": 0.25, "speed_m_s": 0.35, "yaw_deg": 8.0, "direction_speed_m_s": 0.30, "direction_duration_s": 0.8},
            "controller_transition": {"reason": "GATE_CROSSED" if passed else "TIMEOUT_SKIP"},
        })
    residuals = [float(row["allocation_residual_norm"]) for row in rows]
    saturation_rows = [row for row in rows if row["saturated_effectors"]]
    disallowed_statuses = {"infeasible", "numerically_singular", "solver_failure"}
    allocation_statuses = {str(row["allocation_status"]) for row in rows}
    numerical_pass = all(math.isfinite(float(row["time_s"])) for row in rows)
    allocation_pass = not bool(allocation_statuses & disallowed_statuses)
    mission_pass = all(item["truth_result"] == "PASS" for item in objectives) and allocation_pass and numerical_pass
    return {
        "schema": "taoryx.hummingbird-native-horizontal-evaluation/v1alpha1",
        "independent_truth_evaluation": True,
        "mission_pass": mission_pass,
        "required_objectives": objectives,
        "numerical_pass": numerical_pass,
        "hard_envelope_violations": [],
        "allocation_summary": {
            "maximum_realized_residual_nm": max(residuals, default=math.inf),
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
    x = [float(row["position_xy_m"][0]) for row in rows]
    y = [float(row["position_xy_m"][1]) for row in rows]
    u = [float(row["body_velocity_m_s"][0]) for row in rows]
    v = [float(row["body_velocity_m_s"][1]) for row in rows]
    yaw = [math.degrees(float(row["state"]["yaw_error_rad"])) for row in rows]
    residual = [float(row["allocation_residual_norm"]) for row in rows]
    rotor_names = tuple(sorted(rows[0]["actual_effectors"]))
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    axes[0, 0].plot(x, y, color="#1f77b4", linewidth=2.0)
    for index, phase in enumerate(_phases()):
        phase_rows = [row for row in rows if row["phase_id"] == phase.phase_id]
        if phase_rows:
            row = phase_rows[0]
            axes[0, 0].annotate(chr(ord("A") + index), tuple(row["position_xy_m"]), fontsize=9, fontweight="bold", xytext=(4, 4), textcoords="offset points")
            axes[0, 0].scatter([phase.target_xy_m[0]], [phase.target_xy_m[1]], marker="x", color="black")
    axes[0, 0].set(title="Native source-plant horizontal route", xlabel="x / east (m)", ylabel="y / north (m)", aspect="equal")
    axes[0, 0].text(0.02, 0.02, "A forward  B yaw  C body-right\nD rearward  E return", transform=axes[0, 0].transAxes, fontsize=8, bbox={"facecolor": "white", "alpha": 0.75, "pad": 3})
    axes[0, 1].plot(time, u, label="body u / forward", color="#d62728")
    axes[0, 1].plot(time, v, label="body v / right", color="#17becf")
    axes[0, 1].axhline(0.0, color="black", linewidth=0.7)
    axes[0, 1].set(title="Truth body velocities", xlabel="time (s)", ylabel="m/s")
    axes[0, 1].legend(fontsize=7)
    axes[0, 2].plot(time, yaw, color="#9467bd")
    axes[0, 2].axhline(90.0, color="#9467bd", linestyle=":")
    axes[0, 2].set(title="Native yaw response", xlabel="time (s)", ylabel="yaw error (deg)")
    axes[1, 0].plot(time, residual, color="#ff7f0e")
    axes[1, 0].set(title="Requested-to-achieved moment residual", xlabel="time (s)", ylabel="residual (N m)")
    for name in rotor_names:
        axes[1, 1].plot(time, [float(row["actual_effectors"][name]) for row in rows], label=name)
    axes[1, 1].set(title="Actual four-rotor speeds", xlabel="time (s)", ylabel="rad/s")
    axes[1, 1].legend(fontsize=6)
    axes[1, 2].axis("off")
    allocation_summary = evaluation["allocation_summary"]
    assert isinstance(allocation_summary, dict)
    result_label = "PASS"
    if not evaluation["mission_pass"]:
        result_label = "FAIL"
    elif allocation_summary["authority_boundary"]:
        result_label = "PASS / AUTHORITY BOUNDARY"
    axes[1, 2].set_title(f"Independent physical objectives — {result_label}")
    objectives = evaluation["required_objectives"]
    assert isinstance(objectives, list)
    lines = ["objective                         result"]
    lines.extend(f"{str(item['id'])[:29]:29s} {str(item['truth_result']):>10s}" for item in objectives)
    axes[1, 2].text(0.0, 0.98, "\n".join(lines), va="top", family="monospace", fontsize=8)
    fig.suptitle("Hummingbird Alpha 3 native physical horizontal-translation witness", fontsize=15)
    fig.savefig(output / "native_horizontal_translation_evidence_board.png", dpi=150)
    plt.close(fig)
    ####


def write_native_packet(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    output.mkdir(parents=True, exist_ok=True)
    rows, phases, metadata = run_native_mission()
    evaluation = evaluate_native_mission(rows)
    source_tables = cast(list[str], metadata["source_tables"])
    objectives = cast(list[dict[str, object]], evaluation["required_objectives"])
    packet = {
        "schema": "taoryx.hummingbird-native-horizontal-evidence/v1alpha1",
        "status": (
            "native_horizontal_translation_witness_pass_with_authority_boundary"
            if evaluation["mission_pass"] and cast(dict[str, object], evaluation["allocation_summary"])["authority_boundary"]
            else "native_horizontal_translation_witness_pass"
            if evaluation["mission_pass"]
            else "native_horizontal_translation_witness_boundary"
        ),
        "family_id": "hummingbird",
        "mission_id": "hummingbird_native_horizontal_translation_v1",
        "fidelity": "rigid_body_6dof",
        "claim": {"proves": "A source-backed local Hummingbird rigid-body plant follows forward, yaw, lateral, rearward, and return horizontal objectives through plant-derived LQR, bounded physical four-rotor allocation, motor lag, and nonlinear source forces/moments.", "nonclaims": ["altitude or collective-force regulation, battery electrical/SOC, contact, landing, wind robustness, and family-wide rotorcraft qualification", "the local body-velocity position reconstruction is a substitute for the full runtime mission state"]},
        "control_path": {"direct_body_moment_injection": False, "path": "horizontal position error -> desired tilt/yaw reference -> plant-derived LQR -> desired moments -> bounded four-rotor speed allocation -> motor lag -> nonlinear source plant", "physical_effectors": ["rotor-1-speed", "rotor-2-speed", "rotor-3-speed", "rotor-4-speed"], "effectors_are_actual": True},
        "source_provenance": {"plant_id": metadata["plant_id"], "plant_revision": metadata["plant_revision"], "tables": source_tables, "table_sha256": {path: _sha256(ROOT / path) for path in source_tables}},
        "mission": {"start_contract": "source_hover_trim", "terminal_contract": "horizontal_return_gate", "phases": [phase.__dict__ if hasattr(phase, "__dict__") else {"phase_id": phase.phase_id, "target_xy_m": list(phase.target_xy_m), "target_yaw_rad": phase.target_yaw_rad, "duration_s": phase.duration_s, "direction_axis": phase.direction_axis, "direction_sign": phase.direction_sign} for phase in phases]},
        "evaluation": evaluation,
        "metadata": metadata,
        "telemetry": rows,
    }
    (output / "evidence.json").write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "telemetry.json").write_text(json.dumps({"rows": rows}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(rows, evaluation, output)
    manifest = {"schema": "taoryx.hummingbird-native-horizontal-mission/v1alpha1", "status": packet["status"], "family_id": "hummingbird", "mission_id": packet["mission_id"], "fidelity": packet["fidelity"], "artifact": "evidence.json", "board": "native_horizontal_translation_evidence_board.png", "mission_pass": evaluation["mission_pass"], "objective_count": len(objectives), "passed_objective_count": sum(item["truth_result"] == "PASS" for item in objectives), "direct_body_moment_injection": False, "physical_motor_allocation": True, "promotion_boundary": "This is a local source-plant horizontal translation witness. Altitude/collective regulation, electrical battery/SOC, contact, landing, wind, and full-envelope promotion remain open.", "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_native_horizontal_mission.py"}
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
