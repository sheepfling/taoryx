#!/usr/bin/env python3
"""Run the first X8 nonlinear six-axis direct-wrench witness.

This is deliberately separate from the X8 elevon allocator.  The controller
requests a body-frame wrench, the generic projector applies declared wrench
limits, and the same table-backed local nonlinear plant supplies the remaining
load.  The result is a Tier-3 direct-wrench bridge, not physical elevon proof.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from taoryx.direct_wrench import (
    DIRECT_WRENCH_NAMES,
    DirectWrenchLimits,
    add_direct_wrench_to_local_derivative,
)
from taoryx.runtime.lqr import solve_scaled_continuous_lqr

try:
    from validate_x8_physical_lqr import build_plant
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_x8_physical_lqr import build_plant

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_x8_direct_wrench"
DT_S = 0.01
DURATION_S = 2.0
FEEDBACK_NAMES = ("roll_error_rad", "pitch_error_rad", "yaw_error_rad", "p_rad_s", "q_rad_s", "r_rad_s")


def _limits() -> DirectWrenchLimits:
    """Return the declared X8 direct-wrench debug authority."""

    force_limits = {
        "force_x_n": (-25.0, 25.0),
        "force_y_n": (-25.0, 25.0),
        "force_z_n": (-25.0, 25.0),
    }
    moment_limits = {
        "moment_x_nm": (-0.20, 0.20),
        "moment_y_nm": (-0.20, 0.20),
        "moment_z_nm": (-0.20, 0.20),
    }
    bounds = force_limits | moment_limits
    return DirectWrenchLimits(
        lower={name: values[0] for name, values in bounds.items()},
        upper={name: values[1] for name, values in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _norm(state: dict[str, float]) -> float:
    """Return the RMS-scaled attitude/rate error used by this witness."""

    scales = {
        "roll_error_rad": math.radians(10.0),
        "pitch_error_rad": math.radians(10.0),
        "yaw_error_rad": math.radians(10.0),
        "p_rad_s": math.radians(30.0),
        "q_rad_s": math.radians(30.0),
        "r_rad_s": math.radians(30.0),
    }
    return math.sqrt(sum((state[name] / scales[name]) ** 2 for name in FEEDBACK_NAMES))
    ####


def _linearize_direct_wrench(plant: Any, trim: Any) -> tuple[np.ndarray, np.ndarray, Any]:
    """Derive local A/B matrices from the source plant plus direct wrench."""

    state_names = FEEDBACK_NAMES
    step = 1.0e-5

    def evaluate(state: dict[str, float], wrench: dict[str, float]) -> np.ndarray:
        derivative = plant.state_derivative(state, plant.source_effectors, {})
        projection = _limits().project(wrench, {name: 0.0 for name in DIRECT_WRENCH_NAMES}, 1.0)
        derivative = add_direct_wrench_to_local_derivative(
            derivative,
            projection,
            mass_kg=plant.source_state.mass,
            inertia_kg_m2=plant.inertia_kg_m2,
        )
        return np.asarray([float(derivative[name]) for name in state_names], dtype=float)

    nominal = dict(trim.state)
    zero = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    a_matrix = np.zeros((len(state_names), len(state_names)))
    for column, name in enumerate(state_names):
        plus = dict(nominal)
        minus = dict(nominal)
        plus[name] += step
        minus[name] -= step
        a_matrix[:, column] = (evaluate(plus, zero) - evaluate(minus, zero)) / (2.0 * step)
    b_matrix = np.zeros((len(state_names), len(DIRECT_WRENCH_NAMES)))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus = dict(zero)
        minus = dict(zero)
        plus[name] += step
        minus[name] -= step
        b_matrix[:, column] = (evaluate(nominal, plus) - evaluate(nominal, minus)) / (2.0 * step)
    state_scales = (math.radians(5.0),) * 3 + (math.radians(20.0),) * 3
    wrench_scales = (25.0, 25.0, 25.0, 0.10, 0.10, 0.10)
    q = np.diag((4.0,) * 6)
    r = np.diag((0.1,) * 6)
    design = solve_scaled_continuous_lqr(
        a_matrix,
        b_matrix,
        q,
        r,
        state_scales=state_scales,
        control_scales=wrench_scales,
        state_names=state_names,
        control_names=DIRECT_WRENCH_NAMES,
    )
    return a_matrix, b_matrix, design
    ####


def build_artifact() -> dict[str, Any]:
    """Execute the bounded direct-wrench local nonlinear recovery witness."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"X8 source trim failed: {trim.as_dict()}")
    state = dict(trim.state)
    state.update(
        {
            "roll_error_rad": math.radians(2.0),
            "pitch_error_rad": math.radians(-1.5),
            # Keep the direct-wrench witness inside the published local beta
            # envelope.  Larger yaw perturbations remain a deliberate source
            # table boundary case, not an invitation to extrapolate.
            "yaw_error_rad": math.radians(0.5),
            "p_rad_s": math.radians(1.5),
            "q_rad_s": math.radians(-1.0),
            "r_rad_s": math.radians(0.25),
        }
    )
    initial_norm = _norm(state)
    limits = _limits()
    a_matrix, b_matrix, lqr = _linearize_direct_wrench(plant, trim)
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    samples: list[dict[str, Any]] = []
    steps = int(round(DURATION_S / DT_S))
    for index in range(steps + 1):
        time_s = index * DT_S
        error = np.asarray([state[name] for name in FEEDBACK_NAMES], dtype=float)
        increment = -np.asarray(lqr.gain, dtype=float) @ error
        requested = {name: float(value) for name, value in zip(DIRECT_WRENCH_NAMES, increment, strict=True)}
        projection = limits.project(requested, previous, DT_S)
        derivative = plant.state_derivative(state, plant.source_effectors, {})
        derivative = add_direct_wrench_to_local_derivative(
            derivative,
            projection,
            mass_kg=plant.source_state.mass,
            inertia_kg_m2=plant.inertia_kg_m2,
        )
        samples.append(
            {
                "time_s": time_s,
                "state": {name: state[name] for name in FEEDBACK_NAMES},
                "direct_wrench": projection.as_dict(),
            }
        )
        if index == steps:
            break
        state = {name: state[name] + float(derivative[name]) * DT_S for name in plant.state_names}
        previous = dict(projection.achieved)
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError("X8 direct-wrench witness produced a nonfinite state")

    final_norm = _norm(state)
    statuses = sorted({str(sample["direct_wrench"]["status"]) for sample in samples})
    acceptance = {
        "final_feedback_error_fraction_max": 0.20,
        "disallowed_statuses": [],
        "direct_body_moment_injection": True,
        "physical_effector_allocation": False,
    }
    mission_pass = (
        final_norm / max(initial_norm, 1.0e-12) <= acceptance["final_feedback_error_fraction_max"]
        and not set(statuses).intersection(acceptance["disallowed_statuses"])
    )
    return {
        "schema": "taoryx.x8-direct-wrench-6dof/v1alpha1",
        "vehicle": "skywalker_x8",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "local attitude/rate error -> bounded six-axis direct wrench -> table-backed nonlinear rigid-body plant",
        "source_trim": trim.as_dict(),
        "direct_wrench_limits": {
            "axes": list(DIRECT_WRENCH_NAMES),
            "lower": dict(limits.lower),
            "upper": dict(limits.upper),
            "rate_limit_per_s": dict(limits.rate_limit_per_s),
        },
        "linearization": {
            "state_names": list(FEEDBACK_NAMES),
            "wrench_names": list(DIRECT_WRENCH_NAMES),
            "a_matrix": a_matrix.tolist(),
            "b_matrix": b_matrix.tolist(),
            "gain": np.asarray(lqr.gain, dtype=float).tolist(),
            "closed_loop_poles": [
                {"real": float(value.real), "imaginary": float(value.imag)}
                for value in lqr.closed_loop_eigenvalues
            ],
            "maximum_real_pole": lqr.maximum_real_pole,
        },
        "acceptance": acceptance,
        "evaluation": {
            "mission_pass": mission_pass,
            "initial_feedback_norm": initial_norm,
            "final_feedback_norm": final_norm,
            "final_feedback_error_fraction": final_norm / max(initial_norm, 1.0e-12),
            "statuses_observed": statuses,
            "sample_count": len(samples),
        },
        "claim": {
            "status": "nominal_case_pass" if mission_pass else "nominal_case_failed",
            "proves": "A bounded six-axis direct body-wrench command can recover a coupled local X8 rigid-body perturbation through the table-backed nonlinear plant.",
            "nonclaims": [
                "No left/right elevon, servo, throttle, or physical-effector allocation is used by this witness.",
                "The direct wrench is an explicit injected control load and is not a hardware-realizable control claim.",
                "This is one source-trim local operating point, not a scheduled or family-wide qualification.",
            ],
            "direct_body_moment_injection": True,
            "physical_effector_allocation": False,
        },
        "telemetry": samples,
        "reproduction": "PYTHONPATH=src python3 tools/validate_x8_direct_wrench.py",
    }
    ####


def main() -> int:
    """Write the reproducible X8 direct-wrench evidence packet."""

    artifact = build_artifact()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "reproduction.txt").write_text(str(artifact["reproduction"]) + "\n", encoding="utf-8")
    print(json.dumps({"status": artifact["claim"]["status"], **artifact["evaluation"]}, indent=2, sort_keys=True))
    return 0 if bool(artifact["evaluation"]["mission_pass"]) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
