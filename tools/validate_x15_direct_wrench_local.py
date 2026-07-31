#!/usr/bin/env python3
"""Run a source-plant X-15 local direct-wrench bridge witness.

The retained X-15 rigid-body source evaluator supplies the nonlinear body
loads at the unpowered release/glide condition.  A bounded six-axis direct
wrench contains an explicit local load-balance bias and LQR feedback.  The
bias is not a physical stabilator, rudder, RCS, or propulsion trim solution.
The separate source-trim artifact remains the promotion gate.
"""

from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from taoryx.contracts import Vector3
from taoryx.direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.modes import Quaternion
from taoryx.runtime.common import RuntimeState
from taoryx.runtime.lqr import solve_scaled_continuous_lqr
from taoryx.runtime.program import LoadedProgram
from tools.solve_x15_trim import PROBLEM, TABLES, _candidate, _steady_glide_seed

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_x15_direct_wrench"
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
MASS_KG = 14_641.0545
INERTIA_KG_M2 = Vector3(4_948.7355, 129_114.2666, 131_825.9024)
DT_S = 0.002
DURATION_S = 2.0


def _limits() -> DirectWrenchLimits:
    """Return the declared local coast-screen authority envelope."""

    bounds = {
        # Keep the debug screen inside a deliberately local authority box.
        # The source-load bias fits inside this box; large LQR demands are
        # reported as partial achievement instead of becoming an unbounded
        # surrogate for an absent physical control system.
        "force_x_n": (-4.0e5, 4.0e5),
        "force_y_n": (-4.0e5, 4.0e5),
        "force_z_n": (-4.0e5, 4.0e5),
        "moment_x_nm": (-1.0e6, 1.0e6),
        "moment_y_nm": (-1.0e6, 1.0e6),
        "moment_z_nm": (-1.0e6, 1.0e6),
    }
    return DirectWrenchLimits(
        lower={name: value[0] for name, value in bounds.items()},
        upper={name: value[1] for name, value in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _context() -> tuple[Any, RuntimeState, Quaternion, dict[str, float]]:
    """Load the source evaluator and derive its local body velocity."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x15-direct-wrench-") as directory:
        candidate = Path(directory) / "candidate.prb"
        candidate.write_text(
            _candidate(
                PROBLEM.read_text(encoding="utf-8"),
                {},
                {"symmetric_stabilator_deg": 0.0, "differential_stabilator_deg": 0.0, "rudder_deg": 0.0},
            ),
            encoding="utf-8",
        )
        program = LoadedProgram.load(candidate, TABLES, profile=GrammarProfile.TAORYX)
    vehicle = program.case().vehicles["1"]
    if vehicle.environment_evaluator is None:
        raise RuntimeError("X-15 source candidate has no rigid-body evaluator")
    base = _steady_glide_seed(vehicle.state)
    attitude = Quaternion(float(base.named["qw"]), float(base.named["qx"]), float(base.named["qy"]), float(base.named["qz"]))
    inertial_velocity = Vector3(float(base.named["vx"]), float(base.named["vy"]), float(base.named["vz"]))
    body_velocity = attitude.conjugate().rotate(inertial_velocity)
    reference = {
        "u_m_s": body_velocity.x,
        "v_m_s": body_velocity.y,
        "w_m_s": body_velocity.z,
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }
    return vehicle, base, attitude, reference
    ####


def _runtime_state(base: RuntimeState, attitude: Quaternion, state: dict[str, float]) -> RuntimeState:
    """Map local body velocity/rates into the source evaluator namespace."""

    body_velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
    inertial_velocity = attitude.rotate(body_velocity)
    values = list(base.values)
    for name, value in zip(("vx", "vy", "vz"), (inertial_velocity.x, inertial_velocity.y, inertial_velocity.z), strict=True):
        values[base.value_names.index(name)] = value
    for name, value in zip(("wx", "wy", "wz"), (state["p_rad_s"], state["q_rad_s"], state["r_rad_s"]), strict=True):
        values[base.value_names.index(name)] = value
    named = {
        **base.named,
        "vx": inertial_velocity.x,
        "vy": inertial_velocity.y,
        "vz": inertial_velocity.z,
        "wx": state["p_rad_s"],
        "wy": state["q_rad_s"],
        "wz": state["r_rad_s"],
        "p": state["p_rad_s"],
        "q": state["q_rad_s"],
        "r": state["r_rad_s"],
        "vel": body_velocity.norm(),
        "wt": float(base.named["mass"]),
    }
    return RuntimeState(base.time, tuple(values), base.frame, named, base.value_names, base.segment_endpoints)
    ####


def _source_derivative(vehicle: Any, base: RuntimeState, attitude: Quaternion, state: dict[str, float]) -> tuple[dict[str, float], dict[str, float]]:
    """Evaluate source total loads and local body velocity/rate dynamics."""

    candidate = _runtime_state(base, attitude, state)
    observed = vehicle.environment_evaluator({**candidate.named, **vehicle.control_values})
    force = Vector3(float(observed["total_force_body_x_n"]), float(observed["total_force_body_y_n"]), float(observed["total_force_body_z_n"]))
    moment = Vector3(float(observed["total_moment_body_x_nm"]), float(observed["total_moment_body_y_nm"]), float(observed["total_moment_body_z_nm"]))
    rates = Vector3(state["p_rad_s"], state["q_rad_s"], state["r_rad_s"])
    velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
    transport = rates.cross(velocity)
    derivative = {
        "u_m_s": force.x / MASS_KG - transport.x,
        "v_m_s": force.y / MASS_KG - transport.y,
        "w_m_s": force.z / MASS_KG - transport.z,
        "p_rad_s": moment.x / INERTIA_KG_M2.x,
        "q_rad_s": moment.y / INERTIA_KG_M2.y,
        "r_rad_s": moment.z / INERTIA_KG_M2.z,
    }
    load = {
        "force_x_n": float(observed["total_force_body_x_n"]),
        "force_y_n": float(observed["total_force_body_y_n"]),
        "force_z_n": float(observed["total_force_body_z_n"]),
        "moment_x_nm": float(observed["total_moment_body_x_nm"]),
        "moment_y_nm": float(observed["total_moment_body_y_nm"]),
        "moment_z_nm": float(observed["total_moment_body_z_nm"]),
        "alpha_deg": float(observed["aero_alpha_deg"]),
        "beta_deg": float(observed["aero_sideslip_deg"]),
        "mach": float(observed["aero_mach"]),
    }
    return derivative, load
    ####


def _norm(state: dict[str, float], reference: dict[str, float]) -> float:
    """Return a scaled body velocity/rate error."""

    scales = {"u_m_s": 20.0, "v_m_s": 20.0, "w_m_s": 20.0, "p_rad_s": 0.15, "q_rad_s": 0.15, "r_rad_s": 0.15}
    return float(math.sqrt(sum(((state[name] - reference[name]) / scales[name]) ** 2 for name in STATE_NAMES)))
    ####


def _linearize(vehicle: Any, base: RuntimeState, attitude: Quaternion, reference: dict[str, float], bias: dict[str, float], limits: DirectWrenchLimits) -> tuple[np.ndarray, np.ndarray, Any]:
    """Derive local A/B matrices from source loads and direct wrench."""

    step = 1.0e-5

    def evaluate(state: dict[str, float], feedback: dict[str, float]) -> np.ndarray:
        source, _ = _source_derivative(vehicle, base, attitude, state)
        request = {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES}
        projection = limits.project(request, {name: 0.0 for name in DIRECT_WRENCH_NAMES}, 1.0)
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
        # The source-load screen is integrated with an explicit 2 ms step.
        # Penalize wrench effort enough to keep the scheduled pole set below
        # the numerical bandwidth of that step rather than hiding a stiff
        # controller behind a nominally stable continuous-time LQR result.
        np.diag((2_000.0,) * 6),
        state_scales=(20.0, 20.0, 20.0, 0.15, 0.15, 0.15),
        control_scales=(2.0e6, 2.0e6, 2.0e6, 2.0e7, 2.0e7, 2.0e7),
        state_names=STATE_NAMES,
        control_names=DIRECT_WRENCH_NAMES,
    )
    return a_matrix, b_matrix, design
    ####


def _plot(rows: list[dict[str, Any]], output: Path) -> None:
    """Render the local screen diagnostic board."""

    time = [float(row["time_s"]) for row in rows]
    error = [float(row["feedback_norm"]) for row in rows]
    residual = [float(row["wrench"]["residual_norm"]) for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    axes[0].plot(time, error, color="#4c78a8", linewidth=2.0)
    axes[0].set(title="X-15 local release/glide velocity-rate error", xlabel="time [s]", ylabel="scaled error norm")
    axes[0].grid(alpha=0.3)
    axes[1].plot(time, residual, color="#e15759", linewidth=2.0)
    axes[1].set(title="Direct-wrench projection residual", xlabel="time [s]", ylabel="six-axis SI residual norm")
    axes[1].grid(alpha=0.3)
    fig.suptitle("X-15 source-plant direct-wrench screen — not physical-effector qualification", fontsize=13)
    fig.savefig(output / "direct_wrench_screen_board.png", dpi=160)
    plt.close(fig)
    ####


def build_artifact(output: Path = OUTPUT) -> dict[str, object]:
    """Execute and write the X-15 local direct-wrench screen."""

    output.mkdir(parents=True, exist_ok=True)
    vehicle, base, attitude, reference = _context()
    _, baseline_load = _source_derivative(vehicle, base, attitude, reference)
    bias = {name: -baseline_load[name] for name in DIRECT_WRENCH_NAMES}
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize(vehicle, base, attitude, reference, bias, limits)
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
        source_derivative, source_load = _source_derivative(vehicle, base, attitude, state)
        derivative = add_direct_wrench_to_local_derivative(source_derivative, projection, mass_kg=MASS_KG, inertia_kg_m2=INERTIA_KG_M2)
        rows.append({"time_s": time_s, "state": dict(state), "feedback_norm": _norm(state, reference), "source_load": source_load, "bias_wrench": dict(bias), "feedback_wrench": feedback, "wrench": projection.as_dict()})
        if index == steps:
            break
        state = {name: state[name] + DT_S * derivative[name] for name in STATE_NAMES}
        previous = dict(projection.achieved)
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError("X-15 direct-wrench screen produced a nonfinite state")
    final_norm = _norm(state, reference)
    statuses = sorted({str(row["wrench"]["status"]) for row in rows})
    mission_pass = math.isfinite(final_norm) and final_norm < initial_norm * 0.25 and statuses == ["feasible"]
    artifact: dict[str, object] = {
        "schema": "taoryx.x15-direct-wrench-local/v1alpha1",
        "status": "nominal_case_pass" if mission_pass else "development_screen_pending",
        "family_id": "x15",
        "vehicle_id": "x15_canonical_research_6dof",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "retained X-15 source rigid-body loads + explicit local load-balance bias + bounded six-axis direct feedback wrench",
        "operating_point": {"regime": "unpowered_release_glide", "source_state_time_s": float(base.time), "mass_kg": MASS_KG, "inertia_body_kg_m2": [INERTIA_KG_M2.x, INERTIA_KG_M2.y, INERTIA_KG_M2.z]},
        "source_trim_boundary": {"source_trim_artifact": "verification/generated/x15_physical_lqr_readiness.json", "source_trim_status": "blocked_before_T1_trim", "direct_bias_is_not_source_trim": True},
        "source_load_balance_bias": bias,
        "source_provenance": {"problem": str(PROBLEM.relative_to(ROOT)), "tables": [str(path.relative_to(ROOT)) for path in TABLES]},
        "direct_wrench_limits": {"axes": list(DIRECT_WRENCH_NAMES), "lower": dict(limits.lower), "upper": dict(limits.upper), "rate_limit_per_s": dict(limits.rate_limit_per_s)},
        "linearization": {"state_names": list(STATE_NAMES), "wrench_names": list(DIRECT_WRENCH_NAMES), "a_matrix": a_matrix.tolist(), "b_matrix": b_matrix.tolist(), "gain": np.asarray(lqr.gain).tolist(), "closed_loop_poles": [{"real": float(value.real), "imaginary": float(value.imag)} for value in lqr.closed_loop_eigenvalues]},
        "evaluation": {"mission_pass": mission_pass, "initial_feedback_norm": initial_norm, "final_feedback_norm": final_norm, "final_feedback_error_fraction": final_norm / max(initial_norm, 1.0e-12), "sample_count": len(rows), "dt_s": DT_S, "wrench_statuses_observed": statuses},
        "claim": {"status": "nominal_case_pass" if mission_pass else "nominal_case_failed", "evidence_tier": "T3_direct_wrench_bridge", "proves": "A bounded six-axis direct-wrench controller recovers a small local X-15 release/glide source-plant velocity/rate perturbation.", "direct_body_moment_injection": True, "physical_effector_allocation": False},
        "nonclaims": ["source-bounded trim or equilibrium", "physical stabilator/rudder/RCS/propulsion allocation", "powered-to-coast authority transition", "controlled terminal handoff or mission qualification", "full-envelope or manufacturer validation"],
        "telemetry": rows,
        "reproduction": "PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_x15_direct_wrench_local.py",
    }
    (output / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    _plot(rows, output)
    return artifact
    ####


def main() -> int:
    """Write the X-15 direct-wrench screen artifact."""

    artifact = build_artifact()
    print(json.dumps(artifact["evaluation"], indent=2, sort_keys=True))
    return 0 if artifact["status"] == "nominal_case_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
