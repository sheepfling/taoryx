#!/usr/bin/env python3
"""Validate the Hummingbird directional-translation development witness.

This is deliberately separate from the small hover-box regression in
``validate_hummingbird_fidelity_ladder.py``.  It exercises a reusable
mission-level controller contract: altitude capture, sustained body-forward,
body-lateral, and body-rearward translation, yaw, descent, disturbance
recovery, and touchdown.

The point-mass and aggregate pseudo runs share the semantic phase plan and
controller equations, but remain independent plants.  The directional labels
are evaluated from truth telemetry.  At pseudo fidelity, the vehicle remains
an aggregate thrust-vector response law; this witness does not promote
individual motor allocation or physical rotor authority.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from taoryx.trajectory.hummingbird_3dof import Hummingbird3DOFCommand, Hummingbird3DOFModel, Hummingbird3DOFState
from taoryx.trajectory.hummingbird_pseudo6dof import (
    HummingbirdPseudo6DOFCommand,
    HummingbirdPseudo6DOFModel,
    HummingbirdPseudo6DOFState,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_directional"
TelemetryRow = dict[str, Any]
Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DirectionalPhase:
    phase_id: str
    target_m: Vector3
    target_yaw_rad: float
    duration_s: float
    direction_axis: str | None = None
    direction_sign: int = 0
    landing: bool = False
    shutdown: bool = False
    disturbance_velocity_m_s: Vector3 | None = None
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _distance(left: list[float], right: Vector3) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right, strict=True)))
    ####


def _angle_error(target: float, actual: float) -> float:
    return (target - actual + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _phases() -> tuple[DirectionalPhase, ...]:
    """Return the shared body-direction and altitude mission plan.

    The absolute points are generated from the declared yaw/orientation
    changes.  The resulting route is intentionally long enough to show a
    sustained translation interval instead of only point-to-point braking.
    """

    return (
        DirectionalPhase("takeoff_altitude_gate", (0.0, 0.0, 3.0), 0.0, 6.0),
        DirectionalPhase("hover_capture", (0.0, 0.0, 3.0), 0.0, 2.5),
        DirectionalPhase("forward_body_leg", (8.0, 0.0, 3.0), 0.0, 12.0, "u", 1),
        DirectionalPhase("yaw_scan_gate", (8.0, 0.0, 3.0), math.pi / 2.0, 4.5),
        DirectionalPhase("lateral_body_right_leg", (4.0, 0.0, 3.0), math.pi / 2.0, 10.0, "v", 1),
        DirectionalPhase("rearward_body_leg", (4.0, -6.0, 3.0), math.pi / 2.0, 11.0, "u", -1),
        DirectionalPhase("descent_altitude_gate", (4.0, -6.0, 2.0), math.pi / 2.0, 5.0),
        DirectionalPhase(
            "disturbance_recovery",
            (0.0, 0.0, 2.0),
            math.pi / 2.0,
            12.0,
            disturbance_velocity_m_s=(0.8, -0.6, 0.0),
        ),
        DirectionalPhase("landing_contact", (0.0, 0.0, 0.0), math.pi / 2.0, 8.0, landing=True),
        DirectionalPhase("post_contact_shutdown", (0.0, 0.0, 0.0), math.pi / 2.0, 2.5, shutdown=True),
    )
    ####


def _body_velocity(velocity: list[float], yaw_rad: float) -> Vector3:
    """Resolve horizontal truth velocity into the declared body frame."""

    vx, vy, vz = (float(value) for value in velocity)
    return (
        math.cos(yaw_rad) * vx + math.sin(yaw_rad) * vy,
        -math.sin(yaw_rad) * vx + math.cos(yaw_rad) * vy,
        vz,
    )
    ####


def _command_for_phase(
    model: HummingbirdPseudo6DOFModel,
    state: HummingbirdPseudo6DOFState,
    phase: DirectionalPhase,
) -> HummingbirdPseudo6DOFCommand:
    position = state.position_m
    velocity = state.velocity_m_s
    ax = _clamp(1.2 * (phase.target_m[0] - position[0]) - 1.6 * velocity[0], -3.0, 3.0)
    ay = _clamp(1.2 * (phase.target_m[1] - position[1]) - 1.6 * velocity[1], -3.0, 3.0)
    if phase.landing:
        az = _clamp(0.4 * (phase.target_m[2] - position[2]) - 0.8 * velocity[2], -0.2, 0.2)
    else:
        az = _clamp(1.8 * (phase.target_m[2] - position[2]) - 1.8 * velocity[2], -1.0, 1.0)
    thrust = model.mass_kg * math.sqrt(ax * ax + ay * ay + (model.gravity_m_s2 + az) ** 2)
    thrust = max(thrust, model.mass_kg * (model.gravity_m_s2 + az), 0.0)
    # The directional witness uses the achieved yaw as a body-to-world frame
    # for the aggregate thrust vector.  This is still a pseudo realization,
    # but it is a named body-frame surrogate rather than world-force injection.
    yaw = state.attitude_rad[2]
    body_ax = math.cos(yaw) * ax + math.sin(yaw) * ay
    body_ay = -math.sin(yaw) * ax + math.cos(yaw) * ay
    pitch = math.asin(_clamp(model.mass_kg * body_ax / max(thrust, 1.0e-9), -0.5, 0.5))
    roll = math.asin(_clamp(-model.mass_kg * body_ay / max(thrust, 1.0e-9), -0.5, 0.5))
    requested_ratio = _clamp(thrust / model.maximum_thrust_n / max(state.battery_fraction, 1.0e-6), 0.0, 1.0)
    return HummingbirdPseudo6DOFCommand(
        roll_rad=roll,
        pitch_rad=pitch,
        yaw_rad=phase.target_yaw_rad,
        thrust_ratio=requested_ratio,
        motors_enabled=not phase.shutdown,
        thrust_frame="body_euler",
    )
    ####


def _command_for_phase_3dof(model: Hummingbird3DOFModel, state: Hummingbird3DOFState, phase: DirectionalPhase) -> Hummingbird3DOFCommand:
    position = state.position_m
    velocity = state.velocity_m_s
    ax = _clamp(1.2 * (phase.target_m[0] - position[0]) - 1.6 * velocity[0], -3.0, 3.0)
    ay = _clamp(1.2 * (phase.target_m[1] - position[1]) - 1.6 * velocity[1], -3.0, 3.0)
    if phase.landing:
        az = _clamp(0.4 * (phase.target_m[2] - position[2]) - 0.8 * velocity[2], -0.2, 0.2)
    else:
        az = _clamp(1.8 * (phase.target_m[2] - position[2]) - 1.8 * velocity[2], -1.0, 1.0)
    return Hummingbird3DOFCommand(
        thrust_vector_n=(model.mass_kg * ax, model.mass_kg * ay, model.mass_kg * (model.gravity_m_s2 + az)),
        motors_enabled=not phase.shutdown,
    )
    ####


def _annotate_row(row: TelemetryRow, phase: DirectionalPhase, *, yaw_rad: float) -> TelemetryRow:
    result = dict(row)
    result["phase_id"] = phase.phase_id
    result["target_position_m"] = list(phase.target_m)
    result["target_yaw_rad"] = phase.target_yaw_rad
    result["direction_axis"] = phase.direction_axis
    result["direction_sign"] = phase.direction_sign
    result["body_velocity_m_s"] = list(_body_velocity(row["velocity_m_s"], yaw_rad))
    result["guidance_frame_yaw_rad"] = yaw_rad
    return result
    ####


def run_directional_mission(*, dt_s: float = 0.05, model: HummingbirdPseudo6DOFModel | None = None, initial_state: HummingbirdPseudo6DOFState | None = None) -> tuple[list[TelemetryRow], tuple[DirectionalPhase, ...]]:
    """Run the aggregate pseudo directional mission."""

    selected_model = HummingbirdPseudo6DOFModel() if model is None else model
    state = selected_model.initial_state() if initial_state is None else initial_state
    rows: list[TelemetryRow] = []
    for phase in _phases():
        disturbance_applied = False
        for sample_index in range(round(phase.duration_s / dt_s)):
            if sample_index == 0 and phase.disturbance_velocity_m_s is not None:
                updated = tuple(float(a) + float(b) for a, b in zip(state.velocity_m_s, phase.disturbance_velocity_m_s, strict=True))
                state = replace(state, velocity_m_s=(updated[0], updated[1], updated[2]))
                disturbance_applied = True
            command = _command_for_phase(selected_model, state, phase)
            state, row = selected_model.step(state, command, dt_s)
            annotated = _annotate_row(row, phase, yaw_rad=float(state.attitude_rad[2]))
            annotated["disturbance_applied"] = disturbance_applied and sample_index == 0
            annotated["disturbance_velocity_impulse_m_s"] = list(phase.disturbance_velocity_m_s or (0.0, 0.0, 0.0))
            rows.append(annotated)
    return rows, _phases()
    ####


def run_directional_mission_3dof(*, dt_s: float = 0.05, model: Hummingbird3DOFModel | None = None, initial_state: Hummingbird3DOFState | None = None) -> tuple[list[TelemetryRow], tuple[DirectionalPhase, ...]]:
    """Run the independent point-mass directional mission."""

    selected_model = Hummingbird3DOFModel() if model is None else model
    state = selected_model.initial_state() if initial_state is None else initial_state
    rows: list[TelemetryRow] = []
    for phase in _phases():
        disturbance_applied = False
        for sample_index in range(round(phase.duration_s / dt_s)):
            if sample_index == 0 and phase.disturbance_velocity_m_s is not None:
                updated = tuple(float(a) + float(b) for a, b in zip(state.velocity_m_s, phase.disturbance_velocity_m_s, strict=True))
                state = replace(state, velocity_m_s=(updated[0], updated[1], updated[2]))
                disturbance_applied = True
            command = _command_for_phase_3dof(selected_model, state, phase)
            state, row = selected_model.step(state, command, dt_s)
            annotated = _annotate_row(row, phase, yaw_rad=phase.target_yaw_rad)
            annotated["disturbance_applied"] = disturbance_applied and sample_index == 0
            annotated["disturbance_velocity_impulse_m_s"] = list(phase.disturbance_velocity_m_s or (0.0, 0.0, 0.0))
            rows.append(annotated)
    return rows, _phases()
    ####


def _phase_rows(rows: list[TelemetryRow], phase_id: str) -> list[TelemetryRow]:
    return [row for row in rows if row.get("phase_id") == phase_id]
    ####


def _capture(rows: list[TelemetryRow], phase: DirectionalPhase, *, supports_yaw: bool) -> tuple[bool, dict[str, float]]:
    candidates = [row for row in rows if _distance(row["position_m"], phase.target_m) <= 0.5 and _norm(tuple(float(value) for value in row["velocity_m_s"])) <= 0.5]
    yaw_error = math.inf
    if supports_yaw and candidates:
        yaw_error = min(abs(_angle_error(phase.target_yaw_rad, float(row["achieved_attitude_rad"][2]))) for row in candidates)
        candidates = [row for row in candidates if abs(_angle_error(phase.target_yaw_rad, float(row["achieved_attitude_rad"][2]))) <= math.radians(10.0)]
    closest = min((_distance(row["position_m"], phase.target_m) for row in rows), default=math.inf)
    return bool(candidates), {"closest_position_error_m": closest, "yaw_error_deg": math.degrees(yaw_error) if math.isfinite(yaw_error) else math.inf}
    ####


def _directional_truth(rows: list[TelemetryRow], phase: DirectionalPhase) -> tuple[bool, dict[str, float]]:
    if phase.direction_axis is None:
        return True, {"sustained_duration_s": 0.0, "peak_signed_body_speed_m_s": 0.0}
    axis_index = {"u": 0, "v": 1, "w": 2}[phase.direction_axis]
    signed_values = [phase.direction_sign * float(row["body_velocity_m_s"][axis_index]) for row in rows]
    peak = max(signed_values, default=-math.inf)
    dt = 0.05
    longest = 0
    current = 0
    for value in signed_values:
        if value >= 0.5:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    duration = longest * dt
    return duration >= 1.0, {"sustained_duration_s": duration, "peak_signed_body_speed_m_s": peak}
    ####


def evaluate_directional(rows: list[TelemetryRow], *, supports_yaw: bool = True) -> dict[str, object]:
    """Evaluate directional objectives from truth telemetry only."""

    objectives: list[dict[str, object]] = []
    for phase in _phases():
        phase_rows = _phase_rows(rows, phase.phase_id)
        if phase.phase_id == "post_contact_shutdown":
            continue
        if phase.phase_id == "landing_contact":
            contact_rows = [row for row in phase_rows if row.get("contact_state") is True]
            touchdown = bool(contact_rows) and _distance(contact_rows[0]["position_m"], phase.target_m) <= 0.15
            objectives.append({"id": phase.phase_id, "type": "touchdown", "truth_result": "PASS" if touchdown else "FAIL", "truth_time_s": contact_rows[0]["time_s"] if contact_rows else None, "critical_metric": "contact_and_pad_error", "actual": {"contact": bool(contact_rows), "pad_error_m": min((_distance(row["position_m"], phase.target_m) for row in phase_rows), default=math.inf)}, "tolerance": {"pad_radius_m": 0.15}})
            continue
        captured, capture_metrics = _capture(phase_rows, phase, supports_yaw=supports_yaw)
        direction_pass, direction_metrics = _directional_truth(phase_rows, phase)
        passed = captured and direction_pass
        if phase.phase_id in {"takeoff_altitude_gate", "hover_capture", "yaw_scan_gate", "descent_altitude_gate", "disturbance_recovery"}:
            passed = captured
        if phase.phase_id == "yaw_scan_gate" and not supports_yaw:
            objectives.append({"id": phase.phase_id, "type": "yaw_gate", "required": False, "truth_result": "NOT_APPLICABLE", "truth_time_s": None, "critical_metric": "attitude_not_represented_at_point_mass_3dof", "actual": None, "tolerance": None})
            continue
        objectives.append({
            "id": phase.phase_id,
            "type": "directional_translation" if phase.direction_axis is not None else "altitude_or_hover_gate",
            "required": True,
            "truth_result": "PASS" if passed else "FAIL",
            "truth_time_s": next((row["time_s"] for row in phase_rows if _distance(row["position_m"], phase.target_m) <= 0.5), None),
            "critical_metric": "body_velocity_direction_and_endpoint_capture" if phase.direction_axis is not None else "endpoint_capture",
            "actual": {**capture_metrics, **direction_metrics, "target_position_m": list(phase.target_m), "target_yaw_rad": phase.target_yaw_rad},
            "tolerance": {"position_radius_m": 0.5, "speed_m_s": 0.5, "yaw_deg": 10.0, "minimum_directional_speed_m_s": 0.5, "minimum_directional_duration_s": 1.0},
            "controller_transition": {"reason": "GATE_CROSSED" if passed else "TIMEOUT_SKIP", "phase_id": phase.phase_id},
        })
    contact_index = next((index for index, row in enumerate(rows) if row.get("contact_state") is True), None)
    # The first contact sample is produced by the landing command.  Shutdown
    # is a subsequent phase, so do not incorrectly require that contact sample
    # itself to already carry the shutdown flag.
    post_rows = rows[contact_index + 1 :] if contact_index is not None else []
    shutdown_rows = _phase_rows(rows, "post_contact_shutdown")
    shutdown_pass = bool(shutdown_rows) and all(row.get("shutdown") is True for row in shutdown_rows)
    settled = bool(post_rows) and all(_norm(tuple(float(value) for value in row["velocity_m_s"])) <= 0.05 and _distance(row["position_m"], (0.0, 0.0, 0.0)) <= 0.15 for row in post_rows)
    objectives.append({"id": "post_contact_shutdown", "type": "touchdown_settle", "required": True, "truth_result": "PASS" if shutdown_pass and settled else "FAIL", "truth_time_s": rows[-1]["time_s"] if rows else None, "critical_metric": "shutdown_and_post_contact_settle", "actual": {"shutdown_after_contact": shutdown_pass, "settled": settled}, "tolerance": {"post_contact_speed_m_s": 0.05, "pad_radius_m": 0.15, "post_contact_dwell_s": 2.0}})
    mission_pass = all(item.get("truth_result") in {"PASS", "NOT_APPLICABLE"} for item in objectives if item.get("required", True))
    return {"schema": "taoryx.hummingbird-directional-evaluation/v1alpha1", "independent_truth_evaluation": True, "mission_pass": mission_pass, "required_objectives": objectives, "terminal_pass": all(item["truth_result"] == "PASS" for item in objectives[-2:]), "hard_envelope_violations": [], "numerical_pass": all(math.isfinite(float(row["time_s"])) for row in rows)}
    ####


def _record(rows: list[TelemetryRow], evaluation: dict[str, object], fidelity: str) -> dict[str, object]:
    return {
        "schema": "taoryx.hummingbird-directional-evidence/v1alpha1",
        "status": "nominal_case_pass" if evaluation["mission_pass"] else "nominal_case_fail",
        "family_id": "hummingbird",
        "fidelity": fidelity,
        "mission_id": "hummingbird_directional_translation_v1",
        "claim": {"proves": "A deterministic altitude, yaw, body-forward, body-lateral, body-rearward, disturbance-recovery, and touchdown mission at the declared reduced fidelity.", "nonclaims": ["individual motor allocation, rotor inflow, reaction torque, physical moment closure, and electrical battery/SOC", "the pseudo body-direction witness is not a native rotorcraft translation qualification"]},
        "control_path": {"realization": "body_frame_aggregate_thrust_vector_surrogate" if fidelity == "pseudo_6dof" else "point_mass_force_model", "direct_force_moment_injection": False, "physical_motor_allocation": False, "body_direction_frame": "truth velocity resolved using achieved yaw in pseudo_6dof; declared guidance yaw only in point_mass_3dof"},
        "evaluation": evaluation,
        "telemetry": rows,
    }
    ####


def _plot(rows: list[TelemetryRow], evaluation: dict[str, object], output: Path) -> None:
    time = [float(row["time_s"]) for row in rows]
    x = [float(row["position_m"][0]) for row in rows]
    y = [float(row["position_m"][1]) for row in rows]
    z = [float(row["position_m"][2]) for row in rows]
    speed = [_norm(tuple(float(value) for value in row["velocity_m_s"])) for row in rows]
    body_u = [float(row["body_velocity_m_s"][0]) for row in rows]
    body_v = [float(row["body_velocity_m_s"][1]) for row in rows]
    yaw = [math.degrees(float(row.get("achieved_attitude_rad", [0.0, 0.0, row["guidance_frame_yaw_rad"]])[2])) for row in rows]
    thrust = [float(row["achieved_thrust_n"]) for row in rows]
    battery = [float(row["battery_fraction"]) for row in rows]
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    axes[0, 0].plot(x, y, color="#1f77b4", linewidth=2.0)
    phase_labels = {
        "takeoff_altitude_gate": "A",
        "hover_capture": "B",
        "forward_body_leg": "C",
        "yaw_scan_gate": "D",
        "lateral_body_right_leg": "E",
        "rearward_body_leg": "F",
        "descent_altitude_gate": "G",
        "disturbance_recovery": "H",
        "landing_contact": "I",
        "post_contact_shutdown": "J",
    }
    for index, row in enumerate(rows):
        if index == 0 or row["phase_id"] != rows[index - 1]["phase_id"]:
            axes[0, 0].annotate(phase_labels[str(row["phase_id"])], (x[index], y[index]), fontsize=9, fontweight="bold", xytext=(4, 4), textcoords="offset points")
    targets = [phase.target_m for phase in _phases() if phase.phase_id not in {"post_contact_shutdown"}]
    axes[0, 0].scatter([target[0] for target in targets], [target[1] for target in targets], marker="x", color="black", label="truth objectives")
    axes[0, 0].scatter([0.0], [0.0], marker="o", color="green", label="pad")
    axes[0, 0].set(title="Directional mission geometry", xlabel="x / east (m)", ylabel="y / north (m)", aspect="equal")
    axes[0, 0].legend(fontsize=7)
    axes[0, 0].text(0.02, 0.02, "A takeoff  B hover  C body-forward  D yaw\nE body-right  F body-rearward  G descend  H recover  I land  J shutdown", transform=axes[0, 0].transAxes, fontsize=7, va="bottom", bbox={"facecolor": "white", "alpha": 0.75, "pad": 3})
    axes[0, 1].plot(time, z, color="#2ca02c", label="altitude")
    axes[0, 1].set(title="Altitude gates and speed", xlabel="time (s)", ylabel="altitude (m)")
    twin = axes[0, 1].twinx()
    twin.plot(time, speed, color="#ff7f0e", label="speed")
    twin.set_ylabel("speed (m/s)")
    axes[0, 2].plot(time, yaw, color="#9467bd", label="yaw")
    axes[0, 2].axhline(90.0, color="#9467bd", linestyle=":", label="yaw target")
    axes[0, 2].set(title="Yaw scan and frame change", xlabel="time (s)", ylabel="yaw (deg)")
    axes[0, 2].legend(fontsize=7)
    axes[1, 0].plot(time, body_u, color="#d62728", label="body u / forward")
    axes[1, 0].plot(time, body_v, color="#17becf", label="body v / right")
    axes[1, 0].axhline(0.0, color="black", linewidth=0.7)
    axes[1, 0].set(title="Truth body-frame translation", xlabel="time (s)", ylabel="velocity (m/s)")
    axes[1, 0].legend(fontsize=7)
    axes[1, 1].plot(time, thrust, color="#d62728", label="aggregate thrust")
    axes[1, 1].set(title="Bounded thrust and resource", xlabel="time (s)", ylabel="thrust (N)")
    twin = axes[1, 1].twinx()
    twin.plot(time, battery, color="#17becf", label="battery fraction")
    twin.set_ylabel("battery fraction")
    table_rows = evaluation["required_objectives"]
    assert isinstance(table_rows, list)
    axes[1, 2].axis("off")
    axes[1, 2].set_title(f"Independent truth objectives — {'PASS' if evaluation['mission_pass'] else 'FAIL'}")
    lines = ["objective                         result"]
    lines.extend(f"{str(item['id'])[:29]:29s} {str(item['truth_result']):>10s}" for item in table_rows)
    axes[1, 2].text(0.0, 0.98, "\n".join(lines), va="top", family="monospace", fontsize=8)
    fig.suptitle("Hummingbird Alpha 3 directional translation witness", fontsize=15)
    fig.savefig(output / "directional_translation_evidence_board.png", dpi=150)
    plt.close(fig)
    ####


def write_directional_packet(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    output.mkdir(parents=True, exist_ok=True)
    pseudo_rows, phases = run_directional_mission()
    point_rows, _ = run_directional_mission_3dof()
    pseudo_eval = evaluate_directional(pseudo_rows)
    point_eval = evaluate_directional(point_rows, supports_yaw=False)
    pseudo = _record(pseudo_rows, pseudo_eval, "pseudo_6dof")
    point = _record(point_rows, point_eval, "point_mass_3dof")
    point["claim"]["nonclaims"].append("yaw control and attitude response")
    comparison = {"schema": "taoryx.hummingbird-directional-comparison/v1alpha1", "mission_semantics_equal": True, "pseudo_mission_pass": pseudo_eval["mission_pass"], "point_mass_mission_pass": point_eval["mission_pass"], "point_mass_yaw_objective": "NOT_APPLICABLE", "body_direction_interpretation": "pseudo uses achieved response-law yaw; point mass uses declared guidance-frame yaw only", "terminal_position_delta_m": [float(pseudo_rows[-1]["position_m"][index]) - float(point_rows[-1]["position_m"][index]) for index in range(3)]}
    (output / "pseudo_6dof_evidence.json").write_text(json.dumps(pseudo, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "point_mass_3dof_evidence.json").write_text(json.dumps(point, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "phases.json").write_text(json.dumps([phase.__dict__ if hasattr(phase, "__dict__") else {"phase_id": phase.phase_id, "target_m": list(phase.target_m), "target_yaw_rad": phase.target_yaw_rad, "duration_s": phase.duration_s, "direction_axis": phase.direction_axis, "direction_sign": phase.direction_sign} for phase in phases], indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(pseudo_rows, pseudo_eval, output)
    manifest = {"schema": "taoryx.hummingbird-directional-mission/v1alpha1", "status": "directional_translation_witness_pass" if pseudo_eval["mission_pass"] and point_eval["mission_pass"] else "directional_translation_witness_boundary", "family_id": "hummingbird", "mission_id": "hummingbird_directional_translation_v1", "fidelity_records": [{"fidelity": "point_mass_3dof", "artifact": "point_mass_3dof_evidence.json", "mission_pass": point_eval["mission_pass"]}, {"fidelity": "pseudo_6dof", "artifact": "pseudo_6dof_evidence.json", "mission_pass": pseudo_eval["mission_pass"]}], "objective_count": len(pseudo_eval["required_objectives"]), "passed_objective_count": sum(item["truth_result"] == "PASS" for item in pseudo_eval["required_objectives"]), "physical_motor_allocation": False, "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_directional_mission.py"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(write_directional_packet(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
