#!/usr/bin/env python3
"""Run a source-load HL-20 local direct-wrench recovery witness.

The pinned HL-20 DAVE-ML graph supplies the nonlinear aerodynamic force and
moment load at one declared operating point.  A bounded six-axis body wrench
then contains two explicitly separated pieces:

* a local load-balance bias that cancels the nonzero source load at the
  selected point; and
* an LQR feedback wrench that recovers a small velocity/rate perturbation.

This is a direct-wrench bridge witness. It is not a source-bounded trim, a
surface allocation result, or a closed-loop flight qualification. The bias is
shown separately so it cannot be mistaken for a solved physical trim.
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
from taoryx.reachability_aerodynamics import HL20DavemlAerodynamics
from taoryx.runtime.lqr import solve_scaled_continuous_lqr

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_hl20_direct_wrench"
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
MASS_KG = 8_664.1
INERTIA_KG_M2 = Vector3(31_190.76, 160_480.6870520227, 160_480.6870520227)
MACH = 2.0
ALPHA_DEG = 5.0
ALTITUDE_M = 10_000.0
DT_S = 0.001
DURATION_S = 4.0


def _limits() -> DirectWrenchLimits:
    """Return a declared local screen authority envelope."""

    bounds = {
        "force_x_n": (-1.0e6, 1.0e6),
        "force_y_n": (-1.0e6, 1.0e6),
        "force_z_n": (-1.0e6, 1.0e6),
        "moment_x_nm": (-5.0e6, 5.0e6),
        "moment_y_nm": (-5.0e6, 5.0e6),
        "moment_z_nm": (-5.0e6, 5.0e6),
    }
    return DirectWrenchLimits(
        lower={name: values[0] for name, values in bounds.items()},
        upper={name: values[1] for name, values in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _reference_velocity(provider: HL20DavemlAerodynamics) -> dict[str, float]:
    """Return the source-query velocity for the declared Mach and alpha."""

    speed = MACH * provider.speed_of_sound_m_s
    alpha_rad = math.radians(ALPHA_DEG)
    return {
        "u_m_s": speed * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": -speed * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }
    ####


def _source_derivative(provider: HL20DavemlAerodynamics, state: dict[str, float]) -> tuple[dict[str, float], dict[str, object]]:
    """Evaluate the source-load local body velocity/rate equations."""

    velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
    rates = Vector3(state["p_rad_s"], state["q_rad_s"], state["r_rad_s"])
    loads = provider.evaluate((velocity.x, velocity.y, velocity.z), ALTITUDE_M, (rates.x, rates.y, rates.z))
    force = Vector3(*loads.force_body_n)
    moment = Vector3(*loads.moment_body_nm)
    transport = rates.cross(velocity)
    inertia_rate = Vector3(moment.x / INERTIA_KG_M2.x, moment.y / INERTIA_KG_M2.y, moment.z / INERTIA_KG_M2.z)
    derivative = {
        "u_m_s": force.x / MASS_KG - transport.x,
        "v_m_s": force.y / MASS_KG - transport.y,
        "w_m_s": force.z / MASS_KG - transport.z,
        "p_rad_s": inertia_rate.x,
        "q_rad_s": inertia_rate.y,
        "r_rad_s": inertia_rate.z,
    }
    return derivative, loads.as_dict()
    ####


def _norm(state: dict[str, float], reference: dict[str, float]) -> float:
    """Return the scaled local velocity/rate error."""

    scales = {"u_m_s": 20.0, "v_m_s": 20.0, "w_m_s": 20.0, "p_rad_s": 0.15, "q_rad_s": 0.15, "r_rad_s": 0.15}
    return float(math.sqrt(sum(((state[name] - reference[name]) / scales[name]) ** 2 for name in STATE_NAMES)))
    ####


def _linearize(provider: HL20DavemlAerodynamics, reference: dict[str, float], bias: dict[str, float], limits: DirectWrenchLimits) -> tuple[np.ndarray, np.ndarray, Any]:
    """Derive source-plus-direct-wrench local A/B matrices."""

    step = 1.0e-5

    def evaluate(state: dict[str, float], feedback: dict[str, float]) -> np.ndarray:
        base, _ = _source_derivative(provider, state)
        requested = {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES}
        projection = limits.project(requested, {name: 0.0 for name in DIRECT_WRENCH_NAMES}, 1.0)
        derivative = add_direct_wrench_to_local_derivative(base, projection, mass_kg=MASS_KG, inertia_kg_m2=INERTIA_KG_M2)
        return np.asarray([derivative[name] for name in STATE_NAMES], dtype=float)

    zero = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    a_matrix = np.zeros((len(STATE_NAMES), len(STATE_NAMES)))
    for column, name in enumerate(STATE_NAMES):
        plus = dict(reference)
        minus = dict(reference)
        plus[name] += step
        minus[name] -= step
        a_matrix[:, column] = (evaluate(plus, zero) - evaluate(minus, zero)) / (2.0 * step)
    b_matrix = np.zeros((len(STATE_NAMES), len(DIRECT_WRENCH_NAMES)))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus = dict(zero)
        minus = dict(zero)
        plus[name] += step
        minus[name] -= step
        b_matrix[:, column] = (evaluate(reference, plus) - evaluate(reference, minus)) / (2.0 * step)
    design = solve_scaled_continuous_lqr(
        a_matrix,
        b_matrix,
        np.diag((2.0,) * 6),
        np.diag((20.0,) * 6),
        state_scales=(20.0, 20.0, 20.0, 0.15, 0.15, 0.15),
        control_scales=(1.0e6, 1.0e6, 1.0e6, 5.0e6, 5.0e6, 5.0e6),
        state_names=STATE_NAMES,
        control_names=DIRECT_WRENCH_NAMES,
    )
    return a_matrix, b_matrix, design
    ####


def _plot(rows: list[dict[str, Any]], output: Path) -> None:
    """Render a compact screen-only diagnostic board."""

    time = [float(row["time_s"]) for row in rows]
    errors = [float(row["feedback_norm"]) for row in rows]
    residuals = [float(row["wrench"]["residual_norm"]) for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    axes[0].plot(time, errors, color="#4c78a8", linewidth=2.0)
    axes[0].set_title("HL-20 local velocity/rate error")
    axes[0].set_xlabel("time [s]")
    axes[0].set_ylabel("scaled error norm")
    axes[0].grid(alpha=0.3)
    axes[1].plot(time, residuals, color="#e15759", linewidth=2.0)
    axes[1].set_title("Direct-wrench projection residual")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("six-axis SI residual norm")
    axes[1].grid(alpha=0.3)
    fig.suptitle("HL-20 source-load direct-wrench screen — not surface qualification", fontsize=13)
    fig.savefig(output / "direct_wrench_screen_board.png", dpi=160)
    plt.close(fig)
    ####


def build_artifact(output: Path = OUTPUT) -> dict[str, object]:
    """Execute and write the local source-load direct-wrench screen."""

    output.mkdir(parents=True, exist_ok=True)
    provider = HL20DavemlAerodynamics()
    reference = _reference_velocity(provider)
    _, baseline_load = _source_derivative(provider, reference)
    force_values = baseline_load["force_body_n"]
    moment_values = baseline_load["moment_body_nm"]
    if not isinstance(force_values, list) or not isinstance(moment_values, list):
        raise TypeError("HL-20 source load serialization must contain force and moment lists")
    baseline_wrench = tuple(float(value) for value in (*force_values, *moment_values))
    bias = {name: -float(value) for name, value in zip(DIRECT_WRENCH_NAMES, baseline_wrench, strict=True)}
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize(provider, reference, bias, limits)
    state = dict(reference)
    state.update({"u_m_s": state["u_m_s"] + 1.0, "v_m_s": 0.4, "w_m_s": state["w_m_s"] + 0.8, "p_rad_s": 0.02, "q_rad_s": -0.015, "r_rad_s": 0.01})
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
        base_derivative, source_load = _source_derivative(provider, state)
        derivative = add_direct_wrench_to_local_derivative(base_derivative, projection, mass_kg=MASS_KG, inertia_kg_m2=INERTIA_KG_M2)
        rows.append({"time_s": time_s, "state": dict(state), "feedback_norm": _norm(state, reference), "source_load": source_load, "bias_wrench": dict(bias), "feedback_wrench": feedback, "wrench": projection.as_dict()})
        if index == steps:
            break
        state = {name: state[name] + DT_S * derivative[name] for name in STATE_NAMES}
        previous = dict(projection.achieved)
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError("HL-20 direct-wrench screen produced a nonfinite state")
    final_norm = _norm(state, reference)
    statuses = sorted({str(row["wrench"]["status"]) for row in rows})
    mission_pass = math.isfinite(final_norm) and final_norm < initial_norm * 0.25 and statuses == ["feasible"]
    artifact: dict[str, object] = {
        "schema": "taoryx.hl20-direct-wrench-local/v1alpha1",
        "status": "nominal_case_pass" if mission_pass else "development_screen_pending",
        "family_id": "hl20_mod_k",
        "vehicle_id": "hl20_mod_k_daveml_source_v1",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "pinned DAVE-ML source loads + explicit local load-balance bias + bounded six-axis direct feedback wrench",
        "operating_point": {"mach": MACH, "alpha_deg": ALPHA_DEG, "altitude_m": ALTITUDE_M, "mass_kg": MASS_KG, "inertia_body_kg_m2": [INERTIA_KG_M2.x, INERTIA_KG_M2.y, INERTIA_KG_M2.z]},
        "source_load_balance_bias": bias,
        "source_baseline_load": baseline_load,
        "source_provenance": provider.provenance,
        "direct_wrench_limits": {"axes": list(DIRECT_WRENCH_NAMES), "lower": dict(limits.lower), "upper": dict(limits.upper), "rate_limit_per_s": dict(limits.rate_limit_per_s)},
        "linearization": {"state_names": list(STATE_NAMES), "wrench_names": list(DIRECT_WRENCH_NAMES), "a_matrix": a_matrix.tolist(), "b_matrix": b_matrix.tolist(), "gain": np.asarray(lqr.gain).tolist(), "closed_loop_poles": [{"real": float(value.real), "imaginary": float(value.imag)} for value in lqr.closed_loop_eigenvalues]},
        "evaluation": {"mission_pass": mission_pass, "initial_feedback_norm": initial_norm, "final_feedback_norm": final_norm, "final_feedback_error_fraction": final_norm / max(initial_norm, 1.0e-12), "sample_count": len(rows), "dt_s": DT_S, "wrench_statuses_observed": statuses},
        "claim": {"status": "nominal_case_pass" if mission_pass else "nominal_case_failed", "evidence_tier": "T3_direct_wrench_bridge", "proves": "A bounded six-axis direct-wrench controller recovers a small local HL-20 source-load velocity/rate perturbation at one declared source operating point.", "direct_body_moment_injection": True, "physical_effector_allocation": False},
        "nonclaims": ["source-bounded trim or equilibrium using physical surfaces", "closed-loop surface allocation", "bank reversal, glide energy, landing, or family envelope qualification", "source-exact flight-control law", "hardware or manufacturer validation"],
        "telemetry": rows,
        "reproduction": "PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_hl20_direct_wrench_local.py",
    }
    (output / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    _plot(rows, output)
    return artifact
    ####


def main() -> int:
    """Write the HL-20 direct-wrench screen artifact."""

    artifact = build_artifact()
    print(json.dumps(artifact["evaluation"], indent=2, sort_keys=True))
    return 0 if artifact["status"] == "nominal_case_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
