"""Run a local F-16 source-plant direct-wrench recovery witness."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from taoryx.contracts import Vector3
from taoryx.direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from taoryx.runtime.lqr import solve_scaled_continuous_lqr

try:
    from validate_f16_physical_wrench_perturbations import _build_case
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_f16_physical_wrench_perturbations import _build_case

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_f16_direct_wrench"
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
DT_S = 0.02
DURATION_S = 10.0


def _limits() -> DirectWrenchLimits:
    bounds = {
        "force_x_n": (-5_000.0, 5_000.0),
        "force_y_n": (-5_000.0, 5_000.0),
        "force_z_n": (-5_000.0, 5_000.0),
        "moment_x_nm": (-10_000.0, 10_000.0),
        "moment_y_nm": (-10_000.0, 10_000.0),
        "moment_z_nm": (-10_000.0, 10_000.0),
    }
    return DirectWrenchLimits(
        lower={name: value[0] for name, value in bounds.items()},
        upper={name: value[1] for name, value in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _norm(state: dict[str, float], reference: dict[str, float]) -> float:
    scales = {"u_m_s": 50.0, "v_m_s": 50.0, "w_m_s": 50.0, "p_rad_s": 0.5, "q_rad_s": 0.5, "r_rad_s": 0.5}
    return float(np.sqrt(sum(((state[name] - reference[name]) / scales[name]) ** 2 for name in STATE_NAMES)))
    ####


def _linearize(plant: Any, trim: Any, limits: DirectWrenchLimits) -> tuple[np.ndarray, np.ndarray, Any]:
    step = 1.0e-5
    nominal = {name: float(trim.state[name]) for name in STATE_NAMES}
    zero = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    diagonal_inertia = plant.source.inertia_matrix_kg_m2
    inertia = (float(diagonal_inertia[0][0]), float(diagonal_inertia[1][1]), float(diagonal_inertia[2][2]))

    def evaluate(state: dict[str, float], wrench: dict[str, float]) -> np.ndarray:
        derivative = plant.state_derivative(state, plant.source_effectors if hasattr(plant, "source_effectors") else trim.controls, {})
        projection = limits.project(wrench, zero, DT_S)
        derivative = add_direct_wrench_to_local_derivative(
            derivative,
            projection,
            mass_kg=plant.source.mass_kg,
            inertia_kg_m2=Vector3(*inertia),
        )
        return np.asarray([float(derivative[name]) for name in STATE_NAMES], dtype=float)

    a_matrix = np.zeros((len(STATE_NAMES), len(STATE_NAMES)))
    for column, name in enumerate(STATE_NAMES):
        plus, minus = dict(nominal), dict(nominal)
        plus[name] += step
        minus[name] -= step
        a_matrix[:, column] = (evaluate(plus, zero) - evaluate(minus, zero)) / (2.0 * step)
    b_matrix = np.zeros((len(STATE_NAMES), len(DIRECT_WRENCH_NAMES)))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus, minus = dict(zero), dict(zero)
        plus[name] += step
        minus[name] -= step
        b_matrix[:, column] = (evaluate(nominal, plus) - evaluate(nominal, minus)) / (2.0 * step)
    design = solve_scaled_continuous_lqr(
        a_matrix.tolist(),
        b_matrix.tolist(),
        np.diag((20.0,) * 6).tolist(),
        np.diag((0.01,) * 6).tolist(),
        state_scales=(50.0, 50.0, 50.0, 0.5, 0.5, 0.5),
        control_scales=(5_000.0, 5_000.0, 5_000.0, 10_000.0, 10_000.0, 10_000.0),
        state_names=STATE_NAMES,
        control_names=DIRECT_WRENCH_NAMES,
    )
    return a_matrix, b_matrix, design
    ####


def build_artifact() -> dict[str, object]:
    plant, trim, _ = _build_case()
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize(plant, trim, limits)
    state = {name: float(trim.state[name]) for name in STATE_NAMES}
    state.update({"u_m_s": state["u_m_s"] + 0.1, "w_m_s": state["w_m_s"] + 0.02, "q_rad_s": state["q_rad_s"] + 0.0004})
    reference = {name: float(trim.state[name]) for name in STATE_NAMES}
    initial_norm = _norm(state, reference)
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    rows: list[dict[str, object]] = []
    inertia_matrix = plant.source.inertia_matrix_kg_m2
    inertia = (float(inertia_matrix[0][0]), float(inertia_matrix[1][1]), float(inertia_matrix[2][2]))
    for index in range(int(round(DURATION_S / DT_S)) + 1):
        time_s = index * DT_S
        error = np.asarray([state[name] - float(trim.state[name]) for name in STATE_NAMES], dtype=float)
        requested = {
            name: float(value)
            for name, value in zip(DIRECT_WRENCH_NAMES, -np.asarray(lqr.gain) @ error, strict=True)
        }
        projection = limits.project(requested, previous, DT_S)
        derivative = plant.state_derivative(state, trim.controls, {})
        derivative = add_direct_wrench_to_local_derivative(
            derivative,
            projection,
            mass_kg=plant.source.mass_kg,
            inertia_kg_m2=Vector3(*inertia),
        )
        rows.append({"time_s": time_s, "state": dict(state), "direct_wrench": projection.as_dict()})
        if index == int(round(DURATION_S / DT_S)):
            break
        state = {name: state[name] + DT_S * float(derivative[name]) for name in STATE_NAMES}
        previous = dict(projection.achieved)
    final_norm = _norm(state, reference)
    statuses = sorted({str(row["direct_wrench"]["status"]) for row in rows})  # type: ignore[index]
    mission_pass = final_norm < initial_norm * 0.20
    artifact: dict[str, object] = {
        "schema": "taoryx.f16-direct-wrench-local/v1alpha1",
        "status": "nominal_case_pass" if mission_pass else "development_screen_pending",
        "family_id": "reference_f16_s119",
        "vehicle_id": "reference_f16_s119",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "source-plant local state error -> bounded six-axis direct wrench -> source nonlinear body-load plant",
        "source_trim": trim.as_dict(),
        "direct_wrench_limits": {"axes": list(DIRECT_WRENCH_NAMES), "lower": limits.lower, "upper": limits.upper, "rate_limit_per_s": limits.rate_limit_per_s},
        "linearization": {"state_names": list(STATE_NAMES), "wrench_names": list(DIRECT_WRENCH_NAMES), "a_matrix": a_matrix.tolist(), "b_matrix": b_matrix.tolist(), "gain": np.asarray(lqr.gain).tolist(), "maximum_real_pole": lqr.maximum_real_pole},
        "metrics": {"mission_pass": mission_pass, "initial_feedback_norm": initial_norm, "final_feedback_norm": final_norm, "final_feedback_error_fraction": final_norm / initial_norm, "sample_count": len(rows), "dt_s": DT_S, "statuses_observed": statuses},
        "claim": {"status": "nominal_case_pass" if mission_pass else "nominal_case_failed", "proves": "A bounded six-axis direct body wrench recovers a local F-16 source-plant velocity/rate perturbation.", "direct_body_moment_injection": True, "physical_effector_allocation": False},
        "nonclaims": ["physical elevator/aileron/rudder/throttle allocation", "racetrack route closure", "scheduled or full-envelope control", "manufacturer flight-control fidelity"],
        "telemetry": rows,
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_direct_wrench_local.py",
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    return artifact
    ####


def main() -> int:
    artifact = build_artifact()
    print(json.dumps(artifact["metrics"], indent=2, sort_keys=True))
    return 0 if artifact["status"] == "nominal_case_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
