"""Run the Hummingbird six-axis direct-wrench debug comparator.

The native individual-rotor path is the promoted Hummingbird realization. This
tool deliberately bypasses rotor allocation and injects a bounded body wrench
into the same local source plant so controller-screen behavior can be compared
against the physical path. Its result is never physical-effector evidence.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from taoryx.contracts import Vector3
from taoryx.direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from taoryx.runtime.lqr import solve_scaled_continuous_lqr

try:
    from validate_hummingbird_physical_lqr import build_plant
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_hummingbird_physical_lqr import build_plant

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_hummingbird_direct_wrench_debug"
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
DT_S = 0.002
DURATION_S = 2.0


def _limits() -> DirectWrenchLimits:
    bounds = {
        "force_x_n": (-0.25, 0.25),
        "force_y_n": (-0.25, 0.25),
        "force_z_n": (-0.50, 0.50),
        "moment_x_nm": (-0.01, 0.01),
        "moment_y_nm": (-0.01, 0.01),
        "moment_z_nm": (-0.01, 0.01),
    }
    return DirectWrenchLimits(
        lower={name: values[0] for name, values in bounds.items()},
        upper={name: values[1] for name, values in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _norm(state: dict[str, float], reference: dict[str, float]) -> float:
    scales = {"u_m_s": 2.0, "v_m_s": 2.0, "w_m_s": 2.0, "p_rad_s": 1.0, "q_rad_s": 1.0, "r_rad_s": 1.0}
    return float(np.sqrt(sum(((state[name] - reference[name]) / scales[name]) ** 2 for name in STATE_NAMES)))
    ####


def _linearize(plant: Any, trim: Any, limits: DirectWrenchLimits) -> Any:
    nominal = {name: float(trim.state[name]) for name in plant.state_names}
    zero = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    inertia = plant.inertia_kg_m2

    def evaluate(state: dict[str, float], wrench: dict[str, float]) -> np.ndarray:
        base = plant.state_derivative(state, trim.controls, {})
        projection = limits.project(wrench, zero, DT_S)
        derivative = add_direct_wrench_to_local_derivative(
            base,
            projection,
            mass_kg=float(plant.source_state.mass),
            inertia_kg_m2=Vector3(inertia.x, inertia.y, inertia.z),
        )
        return np.asarray([float(derivative[name]) for name in STATE_NAMES], dtype=float)

    a_matrix = np.zeros((6, 6))
    for column, name in enumerate(STATE_NAMES):
        plus, minus = dict(nominal), dict(nominal)
        plus[name] += 1.0e-5
        minus[name] -= 1.0e-5
        a_matrix[:, column] = (evaluate(plus, zero) - evaluate(minus, zero)) / 2.0e-5
    b_matrix = np.zeros((6, 6))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus, minus = dict(zero), dict(zero)
        plus[name] += 1.0e-6
        minus[name] -= 1.0e-6
        b_matrix[:, column] = (evaluate(nominal, plus) - evaluate(nominal, minus)) / 2.0e-6
    return a_matrix, b_matrix, solve_scaled_continuous_lqr(
        a_matrix.tolist(),
        b_matrix.tolist(),
        np.diag((8.0, 8.0, 8.0, 2.0, 2.0, 2.0)).tolist(),
        np.diag((0.01,) * 6).tolist(),
        state_scales=(2.0, 2.0, 2.0, 1.0, 1.0, 1.0),
        control_scales=(0.25, 0.25, 0.50, 0.01, 0.01, 0.01),
        state_names=STATE_NAMES,
        control_names=DIRECT_WRENCH_NAMES,
    )
    ####


def build_artifact() -> dict[str, object]:
    """Build the bounded direct-wrench comparator artifact."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird source trim did not converge: {trim.as_dict()}")
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize(plant, trim, limits)
    reference = {name: float(trim.state[name]) for name in plant.state_names}
    state = dict(reference)
    state.update({"u_m_s": state["u_m_s"] + 0.10, "v_m_s": state["v_m_s"] - 0.06, "p_rad_s": state["p_rad_s"] + 0.08, "r_rad_s": state["r_rad_s"] - 0.06})
    initial_norm = _norm(state, reference)
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    rows: list[dict[str, object]] = []
    for index in range(round(DURATION_S / DT_S) + 1):
        time_s = index * DT_S
        error = np.asarray([state[name] - reference[name] for name in STATE_NAMES], dtype=float)
        requested = {name: float(value) for name, value in zip(DIRECT_WRENCH_NAMES, -np.asarray(lqr.gain) @ error, strict=True)}
        projection = limits.project(requested, previous, DT_S)
        base = plant.state_derivative(state, trim.controls, {})
        derivative = add_direct_wrench_to_local_derivative(base, projection, mass_kg=float(plant.source_state.mass), inertia_kg_m2=plant.inertia_kg_m2)
        rows.append({"time_s": time_s, "state": dict(state), "direct_wrench": projection.as_dict()})
        if index == round(DURATION_S / DT_S):
            break
        state = {name: state[name] + DT_S * float(derivative[name]) for name in plant.state_names}
        previous = dict(projection.achieved)
    final_norm = _norm(state, reference)
    mission_pass = math.isfinite(final_norm) and final_norm < initial_norm * 0.20
    artifact: dict[str, object] = {
        "schema": "taoryx.hummingbird-direct-wrench-debug/v1alpha1",
        "status": "debug_comparator_pass" if mission_pass else "debug_comparator_failed",
        "family_id": "hummingbird",
        "fidelity": "rigid_body_6dof_direct_wrench_debug",
        "control_realization": "direct_wrench_debug_bypass",
        "control_path": "source individual-rotor plant loads plus bounded six-axis injected wrench; rotor allocation bypassed",
        "native_reference": "verification/generated/hummingbird_individual_rotor_physical_lqr.json",
        "trim": trim.as_dict(),
        "direct_wrench_limits": {"axes": list(DIRECT_WRENCH_NAMES), "lower": limits.lower, "upper": limits.upper, "rate_limit_per_s": limits.rate_limit_per_s},
        "linearization": {"state_names": list(STATE_NAMES), "wrench_names": list(DIRECT_WRENCH_NAMES), "a_matrix": a_matrix.tolist(), "b_matrix": b_matrix.tolist(), "gain": np.asarray(lqr.gain).tolist()},
        "metrics": {"initial_feedback_norm": initial_norm, "final_feedback_norm": final_norm, "final_feedback_error_fraction": final_norm / max(initial_norm, 1.0e-12), "sample_count": len(rows), "dt_s": DT_S, "mission_pass": mission_pass},
        "claim": {"proves": "The common bounded direct-wrench seam can be compared against the Hummingbird source individual-rotor plant at one local hover condition.", "evidence_tier": "T3_direct_wrench_bridge", "direct_body_moment_injection": True, "physical_effector_allocation": False},
        "nonclaims": ["motor or rotor allocation", "motor lag or rotor authority qualification", "translation, contact, battery, wind, or envelope qualification"],
        "telemetry": rows,
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_direct_wrench_debug.py",
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    return artifact
    ####


def main() -> int:
    artifact = build_artifact()
    print(json.dumps(artifact["metrics"], indent=2, sort_keys=True))
    return 0 if artifact["status"] == "debug_comparator_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
