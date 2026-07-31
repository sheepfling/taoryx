#!/usr/bin/env python3
"""Run an explicitly engineering-labeled NESC direct-wrench bridge witness.

The retained NESC package contains staged translation and mass history but no
attitude state, gimbal input, or control-effectiveness output.  This witness
therefore derives a local translational acceleration from adjacent source
velocity samples, supplies declared engineering inertia, and closes a bounded
six-axis direct-wrench loop around that extension.  It is useful for testing
the common wrench bridge only; it is not a source-exact NESC attitude model.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from taoryx.contracts import Vector3
from taoryx.direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from taoryx.runtime.lqr import solve_scaled_continuous_lqr

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "verification/daveml_nesc_reduction_qualification.json"
OUTPUT = ROOT / "verification/alpha3_nesc_direct_wrench"
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
MASS_KG = 314_000.0
# Engineering screen assumption only; the retained NESC package has no
# participating attitude inertia channels in the imported reduction.
INERTIA_KG_M2 = Vector3(2.0e6, 4.0e7, 4.0e7)
DT_S = 0.002
DURATION_S = 5.0


def _load_history() -> list[dict[str, Any]]:
    payload = json.loads(SOURCE.read_text(encoding="utf-8"))
    history = payload.get("history")
    if not isinstance(history, list) or len(history) < 2:
        raise ValueError("NESC reduction artifact does not contain adjacent history samples")
    return [row for row in history if isinstance(row, dict)]
    ####


def _source_acceleration(history: list[dict[str, Any]]) -> tuple[Vector3, dict[str, Any]]:
    """Estimate one local source translation acceleration from adjacent rows."""

    first, second = history[0], history[1]
    dt = float(second["time_s"]) - float(first["time_s"])
    if dt <= 0.0:
        raise ValueError("NESC source history is not strictly increasing at the selected rows")
    v0 = Vector3(*(float(value) for value in first["replay_velocity_eci_mps"]))
    v1 = Vector3(*(float(value) for value in second["replay_velocity_eci_mps"]))
    acceleration = (v1 - v0).scaled(1.0 / dt)
    return acceleration, {"sample_times_s": [float(first["time_s"]), float(second["time_s"])], "frame": "ECI_assumed_body_alignment"}
    ####


def _limits() -> DirectWrenchLimits:
    """Return a local authority box that contains the derived source bias."""

    return DirectWrenchLimits(
        lower={name: value for name, value in zip(DIRECT_WRENCH_NAMES, (-2.0e7, -2.0e7, -2.0e7, -1.0e8, -1.0e8, -1.0e8), strict=True)},
        upper={name: value for name, value in zip(DIRECT_WRENCH_NAMES, (2.0e7, 2.0e7, 2.0e7, 1.0e8, 1.0e8, 1.0e8), strict=True)},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _norm(state: dict[str, float], reference: dict[str, float]) -> float:
    scales = {"u_m_s": 20.0, "v_m_s": 20.0, "w_m_s": 20.0, "p_rad_s": 0.15, "q_rad_s": 0.15, "r_rad_s": 0.15}
    return float(math.sqrt(sum(((state[name] - reference[name]) / scales[name]) ** 2 for name in STATE_NAMES)))
    ####


def _base_derivative(state: dict[str, float], acceleration: Vector3) -> dict[str, float]:
    rates = Vector3(state["p_rad_s"], state["q_rad_s"], state["r_rad_s"])
    velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
    transport = rates.cross(velocity)
    return {
        "u_m_s": acceleration.x - transport.x,
        "v_m_s": acceleration.y - transport.y,
        "w_m_s": acceleration.z - transport.z,
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }
    ####


def _linearize(reference: dict[str, float], acceleration: Vector3, bias: dict[str, float], limits: DirectWrenchLimits) -> tuple[np.ndarray, np.ndarray, Any]:
    step = 1.0e-5

    def evaluate(state: dict[str, float], feedback: dict[str, float]) -> np.ndarray:
        source = _base_derivative(state, acceleration)
        projection = limits.project(
            {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES},
            {name: 0.0 for name in DIRECT_WRENCH_NAMES},
            1.0,
        )
        derivative = add_direct_wrench_to_local_derivative(source, projection, mass_kg=MASS_KG, inertia_kg_m2=INERTIA_KG_M2)
        return np.asarray([derivative[name] for name in STATE_NAMES], dtype=float)

    zero = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    a_matrix = np.zeros((6, 6))
    for column, name in enumerate(STATE_NAMES):
        plus, minus = dict(reference), dict(reference)
        plus[name] += step
        minus[name] -= step
        a_matrix[:, column] = (evaluate(plus, zero) - evaluate(minus, zero)) / (2.0 * step)
    b_matrix = np.zeros((6, 6))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus, minus = dict(zero), dict(zero)
        plus[name] += step
        minus[name] -= step
        b_matrix[:, column] = (evaluate(reference, plus) - evaluate(reference, minus)) / (2.0 * step)
    design = solve_scaled_continuous_lqr(
        a_matrix,
        b_matrix,
        np.diag((2.0,) * 6),
        np.diag((2_000.0,) * 6),
        state_scales=(20.0, 20.0, 20.0, 0.15, 0.15, 0.15),
        control_scales=(2.0e7, 2.0e7, 2.0e7, 1.0e8, 1.0e8, 1.0e8),
        state_names=STATE_NAMES,
        control_names=DIRECT_WRENCH_NAMES,
    )
    return a_matrix, b_matrix, design
    ####


def _plot(rows: list[dict[str, Any]], output: Path) -> None:
    time = [float(row["time_s"]) for row in rows]
    error = [float(row["feedback_norm"]) for row in rows]
    residual = [float(row["wrench"]["residual_norm"]) for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    axes[0].plot(time, error, color="#4c78a8", linewidth=2.0)
    axes[0].set(title="NESC translation-derived local wrench error", xlabel="time [s]", ylabel="scaled error norm")
    axes[0].grid(alpha=0.3)
    axes[1].plot(time, residual, color="#e15759", linewidth=2.0)
    axes[1].set(title="Direct-wrench projection residual", xlabel="time [s]", ylabel="six-axis SI residual norm")
    axes[1].grid(alpha=0.3)
    fig.suptitle("NESC engineering direct-wrench screen — no source attitude/effectivity claim", fontsize=13)
    fig.savefig(output / "direct_wrench_screen_board.png", dpi=160)
    plt.close(fig)
    ####


def build_artifact(output: Path = OUTPUT) -> dict[str, object]:
    """Execute and write the NESC engineering direct-wrench screen."""

    output.mkdir(parents=True, exist_ok=True)
    history = _load_history()
    acceleration, derivation = _source_acceleration(history)
    reference = {"u_m_s": float(history[0]["replay_velocity_eci_mps"][0]), "v_m_s": float(history[0]["replay_velocity_eci_mps"][1]), "w_m_s": float(history[0]["replay_velocity_eci_mps"][2]), "p_rad_s": 0.0, "q_rad_s": 0.0, "r_rad_s": 0.0}
    base_load = {"force_x_n": MASS_KG * acceleration.x, "force_y_n": MASS_KG * acceleration.y, "force_z_n": MASS_KG * acceleration.z, "moment_x_nm": 0.0, "moment_y_nm": 0.0, "moment_z_nm": 0.0}
    bias = {name: -base_load[name] for name in DIRECT_WRENCH_NAMES}
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize(reference, acceleration, bias, limits)
    state = dict(reference)
    state.update({"u_m_s": state["u_m_s"] + 1.0, "v_m_s": state["v_m_s"] + 0.3, "w_m_s": state["w_m_s"] + 0.5, "p_rad_s": 0.01, "q_rad_s": -0.015, "r_rad_s": 0.01})
    initial_norm = _norm(state, reference)
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    rows: list[dict[str, Any]] = []
    steps = int(round(DURATION_S / DT_S))
    for index in range(steps + 1):
        time_s = index * DT_S
        error = np.asarray([state[name] - reference[name] for name in STATE_NAMES], dtype=float)
        feedback = {name: float(value) for name, value in zip(DIRECT_WRENCH_NAMES, -np.asarray(lqr.gain) @ error, strict=True)}
        requested = {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES}
        projection = limits.project(requested, previous, DT_S)
        source = _base_derivative(state, acceleration)
        derivative = add_direct_wrench_to_local_derivative(source, projection, mass_kg=MASS_KG, inertia_kg_m2=INERTIA_KG_M2)
        rows.append({"time_s": time_s, "state": dict(state), "feedback_norm": _norm(state, reference), "source_acceleration_m_s2": [acceleration.x, acceleration.y, acceleration.z], "wrench": projection.as_dict()})
        if index == steps:
            break
        state = {name: state[name] + DT_S * derivative[name] for name in STATE_NAMES}
        previous = dict(projection.achieved)
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError("NESC direct-wrench screen produced a nonfinite state")
    final_norm = _norm(state, reference)
    statuses = sorted({str(row["wrench"]["status"]) for row in rows})
    mission_pass = math.isfinite(final_norm) and final_norm < initial_norm * 0.25 and statuses == ["feasible"]
    artifact: dict[str, object] = {
        "schema": "taoryx.nesc-direct-wrench-local/v1alpha1",
        "status": "nominal_case_pass" if mission_pass else "development_screen_pending",
        "family_id": "reference_nesc_two_stage_rocket",
        "vehicle_id": "nesc_two_stage_translation_derived_screen",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "retained NESC translation history -> derived local acceleration -> declared engineering inertia -> bounded six-axis direct feedback wrench",
        "source_boundary": {"source_artifact": str(SOURCE.relative_to(ROOT)), "source_attitude_available": False, "source_gimbal_available": False, "source_force_available": False, "translation_derived_force": True, "body_frame_alignment_assumption": True, "inertia_source": "engineering_screen_assumption"},
        "source_derivation": derivation,
        "source_load_balance_bias": bias,
        "direct_wrench_limits": {"axes": list(DIRECT_WRENCH_NAMES), "lower": dict(limits.lower), "upper": dict(limits.upper), "rate_limit_per_s": dict(limits.rate_limit_per_s)},
        "linearization": {"state_names": list(STATE_NAMES), "wrench_names": list(DIRECT_WRENCH_NAMES), "a_matrix": a_matrix.tolist(), "b_matrix": b_matrix.tolist(), "gain": np.asarray(lqr.gain).tolist(), "closed_loop_poles": [{"real": float(value.real), "imaginary": float(value.imag)} for value in lqr.closed_loop_eigenvalues]},
        "evaluation": {"mission_pass": mission_pass, "initial_feedback_norm": initial_norm, "final_feedback_norm": final_norm, "final_feedback_error_fraction": final_norm / max(initial_norm, 1.0e-12), "sample_count": len(rows), "dt_s": DT_S, "wrench_statuses_observed": statuses},
        "claim": {"status": "nominal_case_pass" if mission_pass else "nominal_case_failed", "evidence_tier": "T3_direct_wrench_bridge", "proves": "The common bounded six-axis direct-wrench contract can recover a local perturbation around an engineering extension of retained NESC translation data.", "direct_body_moment_injection": True, "physical_effector_allocation": False},
        "nonclaims": ["source-exact NESC attitude or gimbal response", "source force/moment balance", "physical thrust-vector allocation", "stage-dependent controlled 6DOF mission", "source-exact inertia or body-frame mapping"],
        "telemetry": rows,
        "reproduction": "PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_nesc_direct_wrench_local.py",
    }
    (output / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    _plot(rows, output)
    return artifact
    ####


def main() -> int:
    artifact = build_artifact()
    print(json.dumps(artifact["evaluation"], indent=2, sort_keys=True))
    return 0 if artifact["status"] == "nominal_case_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
