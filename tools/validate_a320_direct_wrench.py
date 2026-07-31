"""Validate the A320 surrogate at the bounded direct-wrench evidence tier.

This is intentionally a local source-backed response witness, not a full
aircraft mission or a physical actuator claim.  OpenAP supplies the
translation/performance anchor and the normalized JSBSim moment channels
supply the nonlinear rotational load.  The controller adds an explicitly
bounded body wrench through :mod:`taoryx.direct_wrench`.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Final

from taoryx.direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits
from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_a320_direct_wrench"
DT_S: Final[float] = 0.02
DURATION_S: Final[float] = 8.0


def _limits() -> DirectWrenchLimits:
    return DirectWrenchLimits(
        lower={
            "force_x_n": -20_000.0,
            "force_y_n": -50_000.0,
            "force_z_n": -50_000.0,
            "moment_x_nm": -1_000_000.0,
            "moment_y_nm": -1_000_000.0,
            "moment_z_nm": -1_000_000.0,
        },
        upper={
            "force_x_n": 20_000.0,
            "force_y_n": 50_000.0,
            "force_z_n": 50_000.0,
            "moment_x_nm": 1_000_000.0,
            "moment_y_nm": 1_000_000.0,
            "moment_z_nm": 1_000_000.0,
        },
        rate_limit_per_s={name: 1.0e12 for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _base_derivative(model: A320Pseudo6DOFModel, operating_point: A320OpenAPOperatingPoint, trim: object, state: dict[str, float]) -> dict[str, float]:
    controls = trim.controls  # type: ignore[attr-defined]
    point = A320Pseudo6DOFOperatingPoint(
        operating_point,
        alpha_rad=state["alpha_rad"],
        beta_rad=state["beta_rad"],
        roll_rate_rad_s=state["p_rad_s"],
        pitch_rate_rad_s=state["q_rad_s"],
        yaw_rate_rad_s=state["r_rad_s"],
        aileron_rad=float(controls["aileron_rad"]),
        elevator_rad=float(controls["elevator_rad"]),
        rudder_rad=float(controls["rudder_rad"]),
    )
    result = model.evaluate(point)
    inertia = model.inertia_for_mass(operating_point.mass_kg)
    return {
        "alpha_rad": state["q_rad_s"] - 0.4 * state["alpha_rad"] + 0.05 * float(controls["elevator_rad"]),
        "beta_rad": state["r_rad_s"] - 0.4 * state["beta_rad"] + 0.1 * float(controls["rudder_rad"]),
        "p_rad_s": result.roll_moment_nm / inertia[0],
        "q_rad_s": result.pitch_moment_nm / inertia[1],
        "r_rad_s": result.yaw_moment_nm / inertia[2],
        "speed_error_m_s": 0.0,
    }
    ####


def _requested_wrench(state: dict[str, float], mass_kg: float, inertia: tuple[float, float, float]) -> dict[str, float]:
    """Request a decoupled local recovery wrench in SI units."""

    return {
        "force_x_n": -mass_kg * 0.8 * state["speed_error_m_s"],
        "force_y_n": -mass_kg * 231.85534691678674 * 0.8 * state["beta_rad"],
        "force_z_n": -mass_kg * 231.85534691678674 * 0.8 * state["alpha_rad"],
        "moment_x_nm": -inertia[0] * (8.0 * state["p_rad_s"]),
        "moment_y_nm": -inertia[1] * (6.0 * state["q_rad_s"] + 4.0 * state["alpha_rad"]),
        "moment_z_nm": -inertia[2] * (5.0 * state["r_rad_s"] + 4.0 * state["beta_rad"]),
    }
    ####


def build_artifact() -> dict[str, object]:
    model = A320Pseudo6DOFModel.from_repository(ROOT)
    operating_point = A320OpenAPOperatingPoint(10500.0, 0.78, 60_000.0)
    trim = model.trim_pseudo6dof(operating_point)
    inertia = model.inertia_for_mass(operating_point.mass_kg)
    limits = _limits()
    state = {
        "alpha_rad": float(trim.state["alpha_rad"]) + 0.03,
        "beta_rad": 0.02,
        "p_rad_s": 0.02,
        "q_rad_s": -0.015,
        "r_rad_s": 0.025,
        "speed_error_m_s": 2.0,
    }
    initial_norm = math.sqrt(sum(value * value for value in state.values()))
    rows: list[dict[str, float | str]] = []
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    time_s = 0.0
    while time_s <= DURATION_S + 1.0e-12:
        requested = _requested_wrench(state, operating_point.mass_kg, inertia)
        projection = limits.project(requested, previous, DT_S)
        derivative = _base_derivative(model, operating_point, trim, state)
        achieved = projection.achieved
        derivative["alpha_rad"] += -achieved["force_z_n"] / (operating_point.mass_kg * 231.85534691678674)
        derivative["beta_rad"] += achieved["force_y_n"] / (operating_point.mass_kg * 231.85534691678674)
        derivative["p_rad_s"] += achieved["moment_x_nm"] / inertia[0]
        derivative["q_rad_s"] += achieved["moment_y_nm"] / inertia[1]
        derivative["r_rad_s"] += achieved["moment_z_nm"] / inertia[2]
        derivative["speed_error_m_s"] += achieved["force_x_n"] / operating_point.mass_kg
        row: dict[str, float | str] = {"time_s": time_s, **state}
        row.update({f"requested_{name}": value for name, value in requested.items()})
        row.update({f"achieved_{name}": value for name, value in achieved.items()})
        row.update({f"residual_{name}": value for name, value in projection.residual.items()})
        row["wrench_status"] = projection.status
        row["wrench_residual_norm"] = projection.residual_norm
        rows.append(row)
        if time_s >= DURATION_S:
            break
        for name in state:
            state[name] += DT_S * derivative[name]
        previous = dict(achieved)
        time_s += DT_S
    final_norm = math.sqrt(sum(state[name] * state[name] for name in state))
    mission_pass = math.isfinite(final_norm) and final_norm < initial_norm * 0.25
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "telemetry.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    artifact: dict[str, object] = {
        "schema_version": "taoryx.a320-direct-wrench-evidence/v1",
        "status": "nominal_case_pass" if mission_pass else "development_screen_pending",
        "family_id": "a320_openap_jsbsim_pseudo6dof",
        "vehicle_id": "a320_openap",
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_realization": "direct_wrench",
        "control_path": "source-calibrated local state response -> bounded six-axis direct wrench -> pseudo-source nonlinear rotational load",
        "claim": {
            "proves": "A bounded direct body wrench recovers a coupled A320 local attitude/rate and speed perturbation using the pinned OpenAP/JSBSim surrogate channels.",
            "evidence_tier": "T3_direct_wrench_bridge",
            "direct_body_moment_injection": True,
            "physical_effector_allocation": False,
        },
        "nonclaims": [
            "physical elevator, aileron, rudder, or engine allocation",
            "full aircraft mission closure at rigid-body 6DOF",
            "manufacturer-validated A320 flight dynamics",
            "scheduled envelope qualification or fuel depletion",
        ],
        "trim": trim.as_dict(),
        "model_provenance": model.provenance,
        "operating_point": {"altitude_m": operating_point.altitude_m, "mach": operating_point.mach, "mass_kg": operating_point.mass_kg},
        "direct_wrench_limits": {"lower": limits.lower, "upper": limits.upper, "rate_limit_per_s": limits.rate_limit_per_s},
        "metrics": {
            "mission_pass": mission_pass,
            "initial_feedback_norm": initial_norm,
            "final_feedback_norm": final_norm,
            "final_feedback_error_fraction": final_norm / initial_norm,
            "wrench_statuses_observed": sorted({str(row["wrench_status"]) for row in rows}),
            "sample_count": len(rows),
            "dt_s": DT_S,
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_a320_direct_wrench.py",
    }
    (OUTPUT / "manifest.json").write_text(json.dumps(artifact, indent=2, sort_keys=True, default=float) + "\n", encoding="utf-8")
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
